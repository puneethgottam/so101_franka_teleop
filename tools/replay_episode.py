#!/usr/bin/env python3
"""Replay a recorded episode (recorder_node.py raw format) on the Franka - sim or real.

    tools/replay_episode.sh data/raw/episode_000000            # asks before moving
    tools/replay_episode.sh data/raw/episode_000000 --speed 0.5 --no-gripper

1. checks the robot can move (teleop not ENGAGED; on the real robot: robot mode Move)
2. moves slowly to the episode's first commanded joint pose
3. sends the recorded joint commands as one timed trajectory to the arm controller
   (FollowJointTrajectory) and replays the gripper commands at their recorded times
Ctrl+C cancels: the controller stops the arm where it is.
Paused frames at the start/end are skipped unless --keep-paused.
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = Path(__file__).resolve().parent.parent
APPROACH_VEL = 0.2  # rad/s, max joint speed when moving to the start pose


def duration(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class Replayer(Node):
    def __init__(self, cfg):
        super().__init__("so101_franka_replay")
        self.cfg = cfg
        F = cfg["franka"]
        self.joint_names = F["joint_names"]
        self.q = None
        self.status = ""
        self.robot_mode = None
        self.create_subscription(JointState, F["joint_states_topic"], self.on_js, 10)
        self.create_subscription(String, "/so101_franka_teleop/status", lambda m: setattr(self, "status", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(FrankaRobotState, F["robot_state_topic"],
                                 lambda m: setattr(self, "robot_mode", m.robot_mode), qos_profile_sensor_data)
        controller = F["trajectory_topic"].rsplit("/", 1)[0]  # /panda_arm_controller
        self.traj = ActionClient(self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory")
        G = cfg["gripper"]
        self.grasp = ActionClient(self, Grasp, G["grasp_action"])
        self.move = ActionClient(self, Move, G["move_action"])

    def on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in self.joint_names):
            self.q = np.array([msg.position[idx[n]] for n in self.joint_names])

    def wait(self, fut, timeout=None):
        end = None if timeout is None else time.monotonic() + timeout
        while not fut.done():
            if end is not None and time.monotonic() > end:
                return None
            time.sleep(0.01)
        return fut.result()

    def send_trajectory(self, times, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=list(self.joint_names))
        for t, q in zip(times, positions):
            goal.trajectory.points.append(JointTrajectoryPoint(positions=[float(v) for v in q],
                                                               time_from_start=duration(t)))
        gh = self.wait(self.traj.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected by the controller")
        return gh

    def gripper(self, closed):
        G = self.cfg["gripper"]
        if closed:
            goal = Grasp.Goal(width=G["grasp_width"], speed=G["speed"], force=G["grasp_force"])
            goal.epsilon.inner, goal.epsilon.outer = G["grasp_epsilon_inner"], G["grasp_epsilon_outer"]
            self.grasp.send_goal_async(goal)
        else:
            self.move.send_goal_async(Move.Goal(width=G["open_width"], speed=G["speed"]))
        self.get_logger().info(f"gripper -> {'GRASP' if closed else 'OPEN'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episode", type=Path, help="raw episode directory, e.g. data/raw/episode_000000")
    ap.add_argument("--speed", type=float, default=1.0, help="time scale (0.5 = half speed), max 1.0")
    ap.add_argument("--no-gripper", action="store_true", help="do not replay gripper commands")
    ap.add_argument("--keep-paused", action="store_true", help="also replay paused frames at start/end")
    ap.add_argument("-y", "--yes", action="store_true", help="do not ask before moving")
    ap.add_argument("--config", default=str(HERE / "config" / "teleop.yaml"))
    args, ros_args = ap.parse_known_args()
    if not 0.05 <= args.speed <= 1.0:
        sys.exit("--speed must be in [0.05, 1.0]")

    meta = json.loads((args.episode / "meta.json").read_text())
    f = np.load(args.episode / "frames.npz")
    idx = np.arange(meta["num_frames"])
    if not args.keep_paused and f["engaged"].any():
        eng = np.flatnonzero(f["engaged"])
        idx = idx[eng[0]:eng[-1] + 1]
    t = f["timestamp"][idx]
    t = (t - t[0]) / args.speed
    q_cmd = f["action"][idx, :7]
    grip = f["action"][idx, 7]

    cfg = yaml.safe_load(Path(args.config).read_text())
    rclpy.init(args=ros_args)
    node = Replayer(cfg)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    log = node.get_logger()
    gh = None
    try:
        log.info(f"{args.episode.name}: \"{meta['task']}\", {len(idx)} frames, {t[-1]:.1f} s at speed {args.speed}")
        end = time.monotonic() + 5.0
        while node.q is None and time.monotonic() < end:
            time.sleep(0.05)
        if node.q is None:
            sys.exit("no joint states - is the Franka stack running?")
        if not node.traj.wait_for_server(timeout_sec=5.0):
            sys.exit("arm controller trajectory action not available")
        time.sleep(0.5)  # latched teleop status / robot state
        if node.status.startswith("ENGAGED"):
            sys.exit("teleop is ENGAGED - pause it first (SPACE / OPTIONS)")
        if node.robot_mode is not None and node.robot_mode != FrankaRobotState.ROBOT_MODE_MOVE:
            sys.exit(f"robot mode is {node.robot_mode}, not Move - run tools/franka_recover")

        dist = np.abs(q_cmd[0] - node.q).max()
        t_approach = max(3.0, dist / APPROACH_VEL)
        log.info(f"start pose is {dist:.2f} rad away (max joint) -> approach in {t_approach:.1f} s")
        if not args.yes and input("Move the robot? [y/N] ").strip().lower() != "y":
            sys.exit("aborted")

        use_gripper = not args.no_gripper and node.grasp.wait_for_server(timeout_sec=2.0) \
            and node.move.wait_for_server(timeout_sec=2.0)
        if not args.no_gripper and not use_gripper:
            log.warning("gripper actions not available - replaying the arm only")
        if use_gripper:
            node.gripper(bool(grip[0] > 0.5))

        log.info("approaching start pose ...")
        gh = node.send_trajectory([t_approach], [q_cmd[0]])
        res = node.wait(gh.get_result_async())
        gh = None
        if res.result.error_code != 0:
            sys.exit(f"approach failed: {res.result.error_string or res.result.error_code}")

        log.info("REPLAYING (Ctrl+C stops the robot)")
        gh = node.send_trajectory(t[1:], q_cmd[1:])  # first point = current pose
        t0 = time.monotonic()
        changes = np.flatnonzero(np.diff(grip) != 0) + 1 if use_gripper else []
        for i in changes:
            while time.monotonic() - t0 < t[i]:
                time.sleep(0.005)
            node.gripper(bool(grip[i] > 0.5))
        res = node.wait(gh.get_result_async())
        gh = None
        code = res.result.error_code
        log.info("replay done" if code == 0 else f"replay ended with error {code}: {res.result.error_string}")
    except KeyboardInterrupt:
        if gh is not None:
            log.warning("cancelling - robot stops")
            node.wait(gh.cancel_goal_async(), 2.0)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
