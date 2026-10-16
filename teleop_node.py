#!/usr/bin/env python3
"""SO101 leader -> Franka Panda end-effector teleoperation through MoveIt IK.

No joint mapping: the leader's end-effector pose (URDF forward kinematics) is
tracked *relatively*. When you engage (clutch in), the current leader EE pose
and the current Franka TCP pose are latched; afterwards every leader EE motion
(delta) is applied to the latched Franka pose:

    p_des = p_F0 + scale * M (p_L - p_L0)
    R_des = (M dR_L M^T) R_F0          with dR_L = R_L R_L0^T

The Cartesian target is rate-limited, clipped to a workspace box, solved with
MoveIt's /compute_ik service (collision-aware, seeded with the last solution),
checked for joint jumps and streamed to the joint trajectory controller.

With --gamepad (PS5 / any SDL game controller via joy's game_controller_node) the leader
arm is replaced by velocity control: sticks/triggers move the commanded TCP, integrated from
the last pose that passed IK (no wind-up at the workspace box or on IK failures).

Keys (terminal):  SPACE engage/pause   r toggle orientation   h home (MoveIt plan)
                  g toggle gripper     q quit
Service:          ~/engage  (std_srvs/SetBool)
"""

import math
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_leader import (ARM_JOINTS, FakeLeader, SO101LeaderReader, UrdfChainFK, matrix_to_quat,
                          quat_to_matrix, rotation_log, rotvec_to_matrix, rpy_to_matrix)

HERE = Path(__file__).resolve().parent
ROBOT_MODES = {0: "Other", 1: "Idle", 2: "Move", 3: "Guiding", 4: "Reflex", 5: "UserStopped",
               6: "AutomaticErrorRecovery"}


class LeaderEEFilter:
    """Exponential smoothing of a pose (position + rotation vector increments)."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.T = None

    def __call__(self, T):
        if self.T is None:
            self.T = T.copy()
            return self.T
        a = self.alpha
        self.T[:3, 3] += a * (T[:3, 3] - self.T[:3, 3])
        dR = rotation_log(self.T[:3, :3].T @ T[:3, :3])
        self.T[:3, :3] = self.T[:3, :3] @ rotvec_to_matrix(a * dR)
        return self.T


class TeleopNode(Node):
    def __init__(self, cfg):
        super().__init__("so101_franka_teleop")
        self.cfg = cfg
        L, F, C = cfg["leader"], cfg["franka"], cfg["control"]

        # ---------------- leader / gamepad -----------------
        self.gamepad = cfg["input"] == "gamepad"
        self.fk = UrdfChainFK(HERE / L["urdf"], L["base_link"], L["ee_link"])
        if self.gamepad:
            self.leader = None
        elif L.get("fake", False):
            self.leader = FakeLeader()
            self.get_logger().warning("Using FAKE leader (synthetic sinusoidal motion)")
        else:
            self.leader = SO101LeaderReader(L["port"], L["calibration_file"])
        self.leader_filter = LeaderEEFilter(C["leader_filter_alpha"])
        self.M = rpy_to_matrix(*C["leader_to_franka_rpy"])

        # ---------------- franka -----------------
        self.joint_names = F["joint_names"]
        self.base_frame = F["base_frame"]
        self.ee_link = F["ee_link"]
        self.home = np.array(F["home_joints"])
        margin = C["joint_limit_margin"]
        self.q_lo = np.array(F["joint_lower_limits"]) + margin
        self.q_hi = np.array(F["joint_upper_limits"]) - margin
        self.ws_min = np.array(C["workspace_min"])
        self.ws_max = np.array(C["workspace_max"])

        self.cb = ReentrantCallbackGroup()
        self.loop_cb = MutuallyExclusiveCallbackGroup()
        self.q_meas = None
        self.q_meas_stamp = 0.0
        self.create_subscription(JointState, F["joint_states_topic"], self.on_joint_state, 10,
                                 callback_group=self.cb)
        self.traj_pub = self.create_publisher(JointTrajectory, F["trajectory_topic"], 10)
        # real-robot watchdog: the Franka's own clock must advance and the mode must be Move.
        # (no messages in simulation -> watchdog stays inactive)
        self.robot_state_seen = False
        self.robot_time = None
        self.robot_time_changed = 0.0
        self.robot_mode = None
        self.robot_success_rate = None
        self.robot_last_errors = []
        self.robot_was_ok = True
        self.create_subscription(FrankaRobotState, F["robot_state_topic"], self.on_robot_state,
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # gripper
        G = cfg["gripper"]
        self.gripper_enabled = G["enabled"]
        # close = franka Grasp (force controlled, succeeds for any object width inside the epsilon
        # window), open = franka Move. Commands are binary with hysteresis on the leader gripper.
        self.grasp_cli = ActionClient(self, Grasp, G["grasp_action"], callback_group=self.cb)
        self.move_cli = ActionClient(self, Move, G["move_action"], callback_group=self.cb)
        self.gripper_closed = None
        self.gripper_busy = False
        # last gripper command (1 = grasp, 0 = open) for the episode recorder
        self.gripper_cmd_pub = self.create_publisher(Float32, "~/gripper_command", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # visualisation: leader joint states + static TF so RViz can show the leader beside the robot
        self.leader_js_pub = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.target_pub = self.create_publisher(PoseStamped, "~/target_pose", 10)
        self.status_pub = self.create_publisher(String, "~/status", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = "leader/base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            L["viz_offset_xyz"]
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

        # ---------------- state -----------------
        self.engaged = False
        self.use_rotation = C["track_orientation"]
        self.T_L0 = None
        self.T_F0 = None
        self.T_cmd = None
        self.q_seed = None
        self.ik_pending = False
        self.ik_fail_count = 0
        self.homing = False
        self.lock = threading.Lock()
        self.dt = 1.0 / C["rate_hz"]
        self.last_leader_T = None
        self.last_grip = 0.0

        if self.gamepad:
            P = cfg["gamepad"]
            self.joy = None
            self.joy_stamp = 0.0
            self.joy_prev_buttons = []
            self.speed = 1.0
            self.create_subscription(Joy, P["topic"], self.on_joy, 10, callback_group=self.cb)

        self.create_service(SetBool, "~/engage", self.srv_engage, callback_group=self.cb)
        self.create_service(Trigger, "~/home", self.srv_home, callback_group=self.cb)
        self.create_timer(self.dt, self.loop, callback_group=self.loop_cb)
        self.publish_status()
        if self.gamepad:
            P = cfg["gamepad"]
            self.get_logger().info(
                f"Ready (GAMEPAD on {P['topic']}). PAUSED. max {P['max_lin_vel']} m/s, {P['max_ang_vel']} rad/s, "
                f"orientation={'ON' if self.use_rotation else 'OFF'}. OPTIONS or SPACE to engage.")
        else:
            self.get_logger().info(
                f"Ready. PAUSED. scale={C['position_scale']} max_lin_vel={C['max_lin_vel']} m/s, "
                f"orientation={'ON' if self.use_rotation else 'OFF'}. Press SPACE to engage.")

    # ------------------------------------------------------------------ io
    def on_joint_state(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in self.joint_names):
            self.q_meas = np.array([msg.position[idx[n]] for n in self.joint_names])
            self.q_meas_stamp = time.monotonic()

    def on_robot_state(self, msg):
        now = time.monotonic()
        self.robot_state_seen = True
        if self.robot_time is None or msg.time != self.robot_time:
            self.robot_time = msg.time
            self.robot_time_changed = now
        self.robot_mode = msg.robot_mode
        self.robot_success_rate = msg.control_command_success_rate
        errs = msg.last_motion_errors
        self.robot_last_errors = [f for f in errs.get_fields_and_field_types() if getattr(errs, f) is True]

    def robot_problem(self):
        """None if the robot can execute commands, else a human readable reason."""
        if not self.robot_state_seen:
            return None  # simulation / no franka_robot_state_broadcaster
        stale = time.monotonic() - self.robot_time_changed
        if stale > self.cfg["franka"]["robot_state_timeout"]:
            return (f"robot state FROZEN for {stale:.1f}s - the libfranka control loop has stopped "
                    f"(last mode {ROBOT_MODES.get(self.robot_mode, self.robot_mode)}, "
                    f"last motion errors {self.robot_last_errors or 'none'}). "
                    "Stop the stack, run tools/franka_recover, restart.")
        if self.robot_mode != FrankaRobotState.ROBOT_MODE_MOVE:
            return (f"robot mode is {ROBOT_MODES.get(self.robot_mode, self.robot_mode)}, not Move "
                    f"(last motion errors {self.robot_last_errors or 'none'})")
        return None

    def lookup_franka_ee(self):
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
        tr, ro = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(np.array([ro.x, ro.y, ro.z, ro.w]))
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def publish_status(self):
        s = "ENGAGED" if self.engaged else ("HOMING" if self.homing else "PAUSED")
        s += f" | orientation={'on' if self.use_rotation else 'off'}"
        s += f" | gripper={'on' if self.gripper_enabled else 'off'}"
        if self.gamepad:
            s += f" | gamepad speed={self.speed:.2f}"
        self.status_pub.publish(String(data=s))

    # ------------------------------------------------------------- engage
    def srv_engage(self, req, resp):
        ok, msg = self.set_engaged(req.data)
        resp.success, resp.message = ok, msg
        return resp

    def set_engaged(self, on):
        with self.lock:
            if not on:
                self.engaged = False
                self.publish_status()
                self.get_logger().info("PAUSED (robot holds last command)")
                return True, "paused"
            if self.homing:
                return False, "homing in progress"
            problem = self.robot_problem()
            if problem:
                self.get_logger().error(f"Not engaging: {problem}")
                return False, problem
            if self.q_meas is None or time.monotonic() - self.q_meas_stamp > 0.5:
                self.get_logger().error("No fresh joint states - not engaging")
                return False, "no joint states"
            if self.gamepad and not self.joy_fresh():
                return False, "no gamepad data"
            if not self.gamepad and self.last_leader_T is None:
                return False, "no leader data"
            try:
                self.T_F0 = self.lookup_franka_ee()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"TF {self.base_frame}->{self.ee_link} unavailable: {e}")
                return False, "no tf"
            self.T_L0 = None if self.gamepad else self.last_leader_T.copy()
            self.T_cmd = self.T_F0.copy()
            self.q_seed = self.q_meas.copy()
            self.ik_fail_count = 0
            self.engaged = True
            self.publish_status()
            self.get_logger().info("ENGAGED - Franka follows the "
                                   + ("gamepad" if self.gamepad else "leader EE motion"))
            return True, "engaged"

    # --------------------------------------------------------------- loop
    def loop(self):
        problem = self.robot_problem()
        if problem and self.robot_was_ok:
            self.get_logger().error(f"ROBOT NOT RESPONDING: {problem}")
            if self.engaged:
                self.set_engaged(False)
        elif not problem and not self.robot_was_ok:
            self.get_logger().info("Robot state is live again")
        self.robot_was_ok = problem is None
        if self.gamepad:
            self.gamepad_step()
            return
        try:
            joints, grip = self.leader.read()
        except IOError as e:
            self.get_logger().error(f"Leader read failed: {e}", throttle_duration_sec=1.0)
            if self.engaged:
                self.set_engaged(False)
            return
        self.last_grip = grip

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS + ["gripper"]
        js.position = [joints[n] for n in ARM_JOINTS] + [grip * 1.7 - 0.17]
        self.leader_js_pub.publish(js)

        T_L = self.leader_filter(self.fk.forward(joints))
        self.last_leader_T = T_L.copy()

        if self.gripper_enabled and self.engaged:
            self.update_gripper(grip)

        with self.lock:
            if not self.engaged or self.ik_pending:
                return
            C = self.cfg["control"]
            # desired pose from leader deltas
            dp = C["position_scale"] * (self.M @ (T_L[:3, 3] - self.T_L0[:3, 3]))
            p_des = np.clip(self.T_F0[:3, 3] + dp, self.ws_min, self.ws_max)
            if self.use_rotation:
                dR = self.M @ (T_L[:3, :3] @ self.T_L0[:3, :3].T) @ self.M.T
                dR = rotvec_to_matrix(C["rotation_scale"] * rotation_log(dR))
                R_des = dR @ self.T_F0[:3, :3]
            else:
                R_des = self.T_F0[:3, :3]

            # rate limit the commanded pose
            step = p_des - self.T_cmd[:3, 3]
            n = np.linalg.norm(step)
            max_step = C["max_lin_vel"] * self.dt
            if n > max_step:
                step *= max_step / n
            rv = rotation_log(R_des @ self.T_cmd[:3, :3].T)
            a = np.linalg.norm(rv)
            max_rot = C["max_ang_vel"] * self.dt
            if a > max_rot:
                rv *= max_rot / a
            T_next = np.eye(4)
            T_next[:3, 3] = self.T_cmd[:3, 3] + step
            T_next[:3, :3] = rotvec_to_matrix(rv) @ self.T_cmd[:3, :3]

            self.publish_target(T_next)
            self.request_ik(T_next)

    # ------------------------------------------------------------ gamepad
    def on_joy(self, msg):
        self.joy = msg
        self.joy_stamp = time.monotonic()

    def joy_fresh(self):
        """Gamepad input is valid. game_controller_node only republishes when the controller sends no
        events, so a stick held steady (jitter events, unchanged value) produces no /joy messages:
        silence still counts as 'last message holds' while the joy node runs and the controller is
        attached. Unplugged controller or dead joy node -> not fresh."""
        if self.joy is None:
            return False
        P = self.cfg["gamepad"]
        if time.monotonic() - self.joy_stamp < P["timeout"]:
            return True
        return self.count_publishers(P["topic"]) > 0 and self.controller_attached()

    def controller_attached(self):
        try:
            return self.cfg["gamepad"]["device_name"].lower() in Path("/proc/bus/input/devices").read_text().lower()
        except OSError:
            return False

    def joy_axis(self, i):
        a = self.joy.axes
        return float(a[i]) if 0 <= i < len(a) else 0.0

    def joy_trigger(self, i):
        """Trigger press 0..1 (game_controller_node, DualSense: 0.0 released, -1.0 fully pressed)."""
        return float(np.clip(-self.joy_axis(i), 0.0, 1.0))

    def joy_pressed(self, name):
        """True on the rising edge of the named button."""
        i = self.cfg["gamepad"]["buttons"][name]
        b, prev = self.joy.buttons, self.joy_prev_buttons
        return 0 <= i < len(b) and b[i] and not (i < len(prev) and prev[i])

    def gamepad_step(self):
        P = self.cfg["gamepad"]
        if not self.joy_fresh():
            if self.engaged:
                self.get_logger().error("Gamepad lost (controller disconnected or joy node stopped) - PAUSED")
                self.set_engaged(False)
            return
        # buttons (rising edges)
        if self.joy_pressed("engage"):
            self.set_engaged(not self.engaged)
        if self.joy_pressed("home"):
            self.go_home()
        if self.joy_pressed("orientation"):
            self.use_rotation = not self.use_rotation
            self.get_logger().info(f"Orientation control {'ON' if self.use_rotation else 'OFF'}")
            self.publish_status()
        for name, f in (("speed_up", 1.5), ("speed_down", 1 / 1.5)):
            if self.joy_pressed(name):
                self.speed = float(np.clip(self.speed * f, 0.1, 1.0))
                self.get_logger().info(f"Gamepad speed {self.speed:.2f}")
                self.publish_status()
        if self.gripper_enabled and self.engaged:
            if self.joy_pressed("grasp"):
                self.command_gripper(True)
            elif self.joy_pressed("open"):
                self.command_gripper(False)
        self.joy_prev_buttons = list(self.joy.buttons)

        A = P["axes"]
        ax = lambda name: self.joy_axis(A[name]) if A[name] >= 0 else 0.0  # noqa: E731
        # base-frame velocities; stick up / left are positive in the Joy message
        v = np.array([P["sign"]["x"] * ax("x"), P["sign"]["y"] * ax("y"),
                      self.joy_trigger(A["z_up"]) - self.joy_trigger(A["z_down"])])
        v = np.clip(v, -1.0, 1.0) * P["max_lin_vel"] * self.speed
        w = np.zeros(3)
        if self.use_rotation:
            b = self.joy.buttons
            btn = lambda name: float(0 <= P["buttons"][name] < len(b) and b[P["buttons"][name]])  # noqa: E731
            w = np.array([btn("roll_pos") - btn("roll_neg"),
                          P["sign"]["pitch"] * ax("pitch"), P["sign"]["yaw"] * ax("yaw")])
            w = np.clip(w, -1.0, 1.0) * P["max_ang_vel"] * self.speed

        with self.lock:
            if not self.engaged or self.ik_pending:
                return
            if not v.any() and not w.any():
                return  # hold: the robot keeps the last command
            T_next = np.eye(4)
            T_next[:3, 3] = np.clip(self.T_cmd[:3, 3] + v * self.dt, self.ws_min, self.ws_max)
            # rotation about the TCP point, axes of the base frame
            T_next[:3, :3] = rotvec_to_matrix(w * self.dt) @ self.T_cmd[:3, :3]
            self.publish_target(T_next)
            self.request_ik(T_next)

    def publish_target(self, T):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = T[:3, 3]
        q = matrix_to_quat(T[:3, :3])
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        self.target_pub.publish(ps)
        return ps

    # ----------------------------------------------------------------- IK
    def request_ik(self, T):
        if not self.ik_cli.service_is_ready():
            self.get_logger().warning("/compute_ik not available (is move_group running?)",
                                   throttle_duration_sec=2.0)
            return
        F = self.cfg["franka"]
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = F["group"]
        r.ik_link_name = self.ee_link
        r.avoid_collisions = self.cfg["control"]["avoid_collisions"]
        r.timeout.nanosec = int(self.cfg["control"]["ik_timeout"] * 1e9)
        r.robot_state.joint_state.name = list(self.joint_names)
        r.robot_state.joint_state.position = [float(v) for v in self.q_seed]
        r.pose_stamped = self.publish_target(T)
        self.ik_pending = True
        fut = self.ik_cli.call_async(req)
        fut.add_done_callback(lambda f, T=T.copy(): self.on_ik(f, T))

    def on_ik(self, fut, T):
        with self.lock:
            self.ik_pending = False
            if not self.engaged:
                return
            res = fut.result()
            if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.ik_fail_count += 1
                code = None if res is None else res.error_code.val
                self.get_logger().warning(f"IK failed (code {code}), holding pose "
                                       f"[{self.ik_fail_count}]", throttle_duration_sec=1.0)
                return
            js = res.solution.joint_state
            idx = {n: i for i, n in enumerate(js.name)}
            q = np.array([js.position[idx[n]] for n in self.joint_names])
            bad = np.where((q < self.q_lo) | (q > self.q_hi))[0]
            if len(bad):
                names = ", ".join(f"j{i + 1}={q[i]:+.2f}" for i in bad)
                self.get_logger().warning(f"IK solution too close to joint limits ({names}) - holding",
                                          throttle_duration_sec=1.0)
                return
            jump = np.max(np.abs(q - self.q_seed))
            if jump > self.cfg["control"]["max_joint_step"]:
                self.get_logger().warning(f"IK solution jumps {jump:.2f} rad (> limit) - rejected",
                                       throttle_duration_sec=1.0)
                return
            self.ik_fail_count = 0
            self.q_seed = q
            self.T_cmd = T
            self.send_joint_target(q)

    def send_joint_target(self, q):
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        la = self.cfg["control"]["trajectory_lookahead"]
        pt.time_from_start = Duration(sec=int(la), nanosec=int((la % 1) * 1e9))
        msg.points = [pt]
        self.traj_pub.publish(msg)

    # ------------------------------------------------------------ gripper
    def update_gripper(self, grip):
        G = self.cfg["gripper"]
        if G["invert"]:
            grip = 1.0 - grip
        want_closed = self.gripper_closed
        if grip < G["close_below"]:
            want_closed = True
        elif grip > G["open_above"]:
            want_closed = False
        if want_closed is not None:
            self.command_gripper(want_closed)

    def command_gripper(self, want_closed):
        G = self.cfg["gripper"]
        if want_closed == self.gripper_closed or self.gripper_busy:
            return
        cli = self.grasp_cli if want_closed else self.move_cli
        if not cli.server_is_ready():
            name = G["grasp_action"] if want_closed else G["move_action"]
            self.get_logger().warning(f"Gripper action {name} not available "
                                      "(stack started with gripper:=false?)", throttle_duration_sec=5.0)
            return
        if want_closed:
            goal = Grasp.Goal()
            goal.width = G["grasp_width"]
            goal.epsilon.inner = G["grasp_epsilon_inner"]
            goal.epsilon.outer = G["grasp_epsilon_outer"]
            goal.speed = G["speed"]
            goal.force = G["grasp_force"]
        else:
            goal = Move.Goal()
            goal.width = G["open_width"]
            goal.speed = G["speed"]
        self.gripper_busy = True
        self.gripper_closed = want_closed
        self.gripper_cmd_pub.publish(Float32(data=1.0 if want_closed else 0.0))
        what = f"GRASP ({G['grasp_force']:.0f} N)" if want_closed else f"OPEN ({G['open_width'] * 100:.0f} cm)"
        self.get_logger().info(f"Gripper -> {what}")

        def on_result(f):
            self.gripper_busy = False
            r = f.result()
            if r is not None and not r.result.success:
                self.get_logger().warning(f"Gripper {what} failed: {r.result.error or 'no object / blocked'}")

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                self.gripper_busy = False
                self.gripper_closed = None  # unknown -> resend on next threshold crossing
                self.get_logger().warning("Gripper goal rejected")
                return
            gh.get_result_async().add_done_callback(on_result)

        cli.send_goal_async(goal).add_done_callback(on_goal)

    # --------------------------------------------------------------- home
    def srv_home(self, req, resp):
        if self.homing:
            resp.success, resp.message = False, "already homing"
        elif self.robot_problem():
            resp.success, resp.message = False, self.robot_problem()
        else:
            self.go_home()
            resp.success, resp.message = True, "homing started"
        return resp

    def go_home(self):
        if self.engaged:
            self.set_engaged(False)
        if self.homing:
            return
        problem = self.robot_problem()
        if problem:
            self.get_logger().error(f"Not homing: {problem}")
            return
        self.homing = True
        self.publish_status()
        threading.Thread(target=self._home_worker, daemon=True).start()

    def _home_worker(self):
        try:
            from pymoveit2 import MoveIt2
            F = self.cfg["franka"]
            if not hasattr(self, "_moveit2"):
                self._moveit2 = MoveIt2(node=self, joint_names=self.joint_names,
                                        base_link_name=self.base_frame, end_effector_name=self.ee_link,
                                        group_name=F["group"], callback_group=self.cb,
                                        use_move_group_action=True)
            self._moveit2.max_velocity = F["home_velocity_scaling"]
            self._moveit2.max_acceleration = F["home_velocity_scaling"]
            self.get_logger().info("Planning to home with MoveIt ...")
            self._moveit2.move_to_configuration(self.home.tolist())
            ok = self._moveit2.wait_until_executed()
            self.get_logger().info(f"Home {'reached' if ok else 'FAILED'}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Homing failed: {e}")
        finally:
            self.homing = False
            self.publish_status()


def keyboard_thread(node: TeleopNode):
    if not sys.stdin.isatty():
        node.get_logger().info("stdin is not a TTY - keyboard disabled, use the ~/engage service")
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            k = sys.stdin.read(1)
            if k == " ":
                node.set_engaged(not node.engaged)
            elif k == "r":
                was = node.engaged
                node.set_engaged(False)
                node.use_rotation = not node.use_rotation
                node.get_logger().info(f"Orientation tracking {'ON' if node.use_rotation else 'OFF'}"
                                       + (" - press SPACE to re-engage" if was else ""))
                node.publish_status()
            elif k == "g":
                node.gripper_enabled = not node.gripper_enabled
                node.get_logger().info(f"Gripper {'ON' if node.gripper_enabled else 'OFF'}")
                node.publish_status()
            elif k == "h":
                node.go_home()
            elif k == "q":
                node.set_engaged(False)
                rclpy.try_shutdown()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(HERE / "config" / "teleop.yaml"))
    ap.add_argument("--fake-leader", action="store_true", help="synthetic leader motion")
    ap.add_argument("--gamepad", action="store_true", help="PS5 / game controller instead of the SO101 leader")
    ap.add_argument("--engage", action="store_true", help="engage immediately (testing)")
    args, ros_args = ap.parse_known_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    calib = Path(cfg["leader"]["calibration_file"]).expanduser()
    cfg["leader"]["calibration_file"] = str(calib if calib.is_absolute() else HERE / calib)
    if args.fake_leader:
        cfg["leader"]["fake"] = True
    cfg["input"] = "gamepad" if args.gamepad else "leader"

    rclpy.init(args=ros_args)
    node = TeleopNode(cfg)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=keyboard_thread, args=(node,), daemon=True).start()
    if args.engage:
        def auto():
            for _ in range(100):
                time.sleep(0.2)
                if node.set_engaged(True)[0]:
                    return
        threading.Thread(target=auto, daemon=True).start()
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001  context invalidated by shutdown ('q' key / SIGTERM)
        if rclpy.ok():
            raise
    finally:
        if node.leader is not None:
            node.leader.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
