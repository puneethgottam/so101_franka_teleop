#!/usr/bin/env python3
"""Simulated Franka Hand for fake-hardware runs.

Offers the same interfaces as franka_gripper_node:
  /panda_gripper/move   (franka_msgs/action/Move)
  /panda_gripper/grasp  (franka_msgs/action/Grasp)
Fingers move linearly at the commanded speed. In fake-hardware mode the finger joints belong
to the mock hand inside ros2_control (published via joint_state_broadcaster), so the width is
streamed to that mock hand through panda_hand_controller (ParallelGripperCommand) - this keeps
a single source for the finger joint states shown in RViz.
A grasp closes fully (no object in sim) and succeeds if the final width lies in
[width - epsilon.inner, width + epsilon.outer], like the real hand.
"""

import threading
import time

import rclpy
from control_msgs.action import ParallelGripperCommand
from franka_msgs.action import Grasp, Move
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

HAND_CONTROLLER_ACTION = "/panda_hand_controller/gripper_cmd"

MAX_WIDTH = 0.08


class SimGripper(Node):
    def __init__(self):
        super().__init__("panda_gripper")
        self.width = MAX_WIDTH
        self.lock = threading.Lock()
        self.hand = ActionClient(self, ParallelGripperCommand, HAND_CONTROLLER_ACTION)
        self.hand_synced = False
        self.last_sent = 0.0
        self.create_timer(0.5, self.initial_sync)
        ActionServer(self, Move, "~/move", self.on_move)
        ActionServer(self, Grasp, "~/grasp", self.on_grasp)
        self.get_logger().info("Simulated Franka Hand ready (move / grasp)")

    def send_to_hand(self, force=False):
        """Command the mock hand (rate-limited to 20 Hz unless forced)."""
        now = time.monotonic()
        if not force and now - self.last_sent < 0.05:
            return
        if not self.hand.server_is_ready():
            self.get_logger().warning(f"{HAND_CONTROLLER_ACTION} not available - fingers not shown",
                                      throttle_duration_sec=5.0)
            return
        self.last_sent = now
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["panda_finger_joint1"]
        goal.command.position = [self.width / 2]
        self.hand.send_goal_async(goal)

    def initial_sync(self):
        if not self.hand_synced and self.hand.server_is_ready():
            self.send_to_hand(force=True)  # mock hand starts closed; show our initial (open) width
            self.hand_synced = True

    def drive(self, goal_handle, target, speed, fb):
        target = min(max(target, 0.0), MAX_WIDTH)
        speed = max(speed, 1e-3)
        while abs(self.width - target) > 1e-4:
            step = min(speed / 100, abs(target - self.width))
            with self.lock:
                self.width += step if target > self.width else -step
            fb.current_width = self.width
            goal_handle.publish_feedback(fb)
            self.send_to_hand()
            time.sleep(0.01)
        self.send_to_hand(force=True)

    def on_move(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(f"move to {g.width:.3f} m")
        self.drive(goal_handle, g.width, g.speed, Move.Feedback())
        goal_handle.succeed()
        return Move.Result(success=True)

    def on_grasp(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(f"grasp width={g.width:.3f} force={g.force:.0f} N")
        self.drive(goal_handle, 0.0, g.speed, Grasp.Feedback())  # nothing to grasp in sim
        ok = g.width - g.epsilon.inner <= self.width <= g.width + g.epsilon.outer
        goal_handle.succeed()
        return Grasp.Result(success=ok, error="" if ok else "width outside epsilon window")


def main():
    rclpy.init()
    node = SimGripper()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
