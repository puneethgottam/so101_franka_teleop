"""Franka + MoveIt stack for SO101 teleop, with the leader arm shown in RViz.

Sim : ros2 launch launch/franka_stack.launch.py                       (default: fake hardware)
Real: ros2 launch launch/franka_stack.launch.py use_fake_hardware:=false robot_ip:=172.16.0.2

Wraps franka_moveit_config/moveit.launch.py (ros2_control + panda_arm_controller,
move_group with /compute_ik, gripper) and swaps its RViz for one that also shows
the SO101 leader model and the teleop target pose.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = Path(__file__).resolve().parent.parent


def _setup(context: LaunchContext):
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "franka_moveit_launch",
        os.path.join(get_package_share_directory("franka_moveit_config"), "launch", "moveit.launch.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    franka_ld = mod.generate_launch_description()

    with_gripper = LaunchConfiguration("gripper").perform(context).lower() in ("true", "1")
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() in ("true", "1")
    entities = []
    for e in franka_ld.entities:
        if isinstance(e, IncludeLaunchDescription) and (fake or not with_gripper):
            # the only include in franka's moveit.launch.py is franka_gripper; in sim it is replaced
            # by tools/sim_gripper.py, which also implements the move/grasp actions
            continue
        if isinstance(e, Node) and "rviz2" in str(
                [getattr(s, "text", s) for s in (e.node_executable if isinstance(e.node_executable, list)
                                                  else [e.node_executable])]):
            continue
        if isinstance(e, DeclareLaunchArgument):
            continue  # re-declared below
        entities.append(e)

    # parameters for the MotionPlanning panel (same as franka_moveit_config's RViz)
    franka_share = get_package_share_directory("franka_description")
    moveit_share = get_package_share_directory("franka_moveit_config")
    rviz_params = [
        {"robot_description": ParameterValue(Command([
            FindExecutable(name="xacro"), " ", os.path.join(franka_share, "robots", "panda_arm.urdf.xacro"),
            " hand:=true robot_ip:=", LaunchConfiguration("robot_ip"),
            " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware")]), value_type=str)},
        {"robot_description_semantic": ParameterValue(Command([
            FindExecutable(name="xacro"), " ", os.path.join(moveit_share, "srdf", "panda_arm.srdf.xacro"),
            " hand:=true"]), value_type=str)},
        {"robot_description_kinematics": mod.load_yaml("franka_moveit_config", "config/kinematics.yaml")},
    ]

    # mesh paths in the URDF are relative ("assets/..."); make them absolute for RViz
    leader_urdf = (HERE / "urdf" / "so101_new_calib.urdf").read_text().replace(
        'filename="assets/', f'filename="file://{HERE / "urdf" / "assets"}/')
    leader_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="leader",
        parameters=[{"robot_description": leader_urdf, "frame_prefix": "leader/"}],
        output="log",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(HERE / "config" / "teleop.rviz")],
        parameters=rviz_params,
    )
    out = entities + [leader_rsp]
    if fake and with_gripper:
        # the mock hand in ros2_control owns the finger joints in sim; start its controller and let
        # sim_gripper.py (franka-style move/grasp actions) drive it
        out.append(Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=["panda_hand_controller", "--param-file", os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
                "config", "ros2_controllers.yaml")]))
        out.append(ExecuteProcess(cmd=["python3", str(HERE / "tools" / "sim_gripper.py")],
                                  name="sim_gripper", output="screen"))
    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1"):
        out.append(rviz)
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="dont-care",
                              description="Franka IP (ignored with fake hardware)"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("db", default_value="False"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gripper", default_value="true",
                              description="start franka_gripper node (false: finger joints shown at 0)"),
        OpaqueFunction(function=_setup),
    ])
