#!/usr/bin/env python3
"""Episode recorder for SO101 -> Franka teleop (raw format, convert with tools/to_lerobot.py).

Runs beside teleop_node.py and samples at a fixed rate while an episode is being recorded:

  observation.state     [7 joint positions, finger width (m)]
  observation.ee_pose   [x, y, z, qx, qy, qz, qw]  TCP in base_frame (TF)
  action                [7 commanded joint positions, gripper command (1 = grasp, 0 = open)]
  action.ee_pose        [x, y, z, qx, qy, qz, qw]  commanded TCP pose
  engaged               teleop clutch state (while paused, action = measured state)
  images                one JPEG per frame and camera

Episode layout:  <output_dir>/episode_000000/{meta.json, frames.npz, <camera>/000000.jpg ...}

Keys (terminal):  s start / stop+save   d discard current episode   t set task   q quit (saves)
Services:         ~/start  ~/stop  ~/discard  (std_srvs/Trigger)
Gamepad (/joy):   SQUARE start / stop+save   D-pad left discard   (config/record.yaml)
"""

import json
import queue
import select
import shutil
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

HERE = Path(__file__).resolve().parent
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def image_to_rgb(msg):
    """sensor_msgs Image / CompressedImage -> HxWx3 uint8 RGB."""
    if isinstance(msg, CompressedImage):
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img.copy() if enc == "rgb8" else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB if enc == "rgba8" else cv2.COLOR_BGRA2RGB)
    if enc == "mono8":
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if enc in ("yuv422", "yuyv", "yuv422_yuy2"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUY2)
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


class V4L2Camera:
    """USB camera read directly with OpenCV; a thread keeps only the newest frame."""

    def __init__(self, cfg, on_frame, logger):
        self.cfg, self.on_frame, self.log = cfg, on_frame, logger
        self.cap = self.open()
        threading.Thread(target=self.run, daemon=True).start()

    def open(self):
        c = self.cfg
        cap = cv2.VideoCapture(c["device"], cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera {c['device']}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*c.get("fourcc", "MJPG")))
        if "capture_width" in c:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c["capture_width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c["capture_height"])
        cap.set(cv2.CAP_PROP_FPS, c.get("capture_fps", 30))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def run(self):
        while True:
            ok, bgr = self.cap.read()
            if ok:
                self.on_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                continue
            self.log.error(f"camera {self.cfg['name']} ({self.cfg['device']}) read failed - reopening",
                           throttle_duration_sec=2.0)
            self.cap.release()
            time.sleep(1.0)
            try:
                self.cap = self.open()
            except RuntimeError:
                pass


class Episode:
    def __init__(self, path, task, fps, cameras):
        self.path = path
        self.task = task
        self.fps = fps
        self.cameras = cameras
        self.start = datetime.now()
        self.rows = {k: [] for k in ("timestamp", "engaged", "observation.state", "observation.ee_pose",
                                     "action", "action.ee_pose")}
        self.image_shapes = {}
        for c in cameras:
            (path / c["name"]).mkdir(parents=True, exist_ok=True)

    def __len__(self):
        return len(self.rows["timestamp"])


class RecorderNode(Node):
    def __init__(self, cfg, task):
        super().__init__("so101_franka_recorder")
        self.cfg = cfg
        self.task = task
        T, F = cfg["topics"], cfg["franka"]
        self.joint_names = F["joint_names"]
        self.finger_joints = F["finger_joints"]
        self.base_frame, self.ee_link = F["base_frame"], F["ee_link"]
        self.out = Path(cfg["output_dir"])
        self.out = self.out if self.out.is_absolute() else HERE / self.out
        self.out.mkdir(parents=True, exist_ok=True)

        self.cb = ReentrantCallbackGroup()
        self.q = None
        self.width = None
        self.js_stamp = 0.0
        self.q_cmd = None
        self.target = None
        self.grip_cmd = None
        self.engaged = False
        self.images = {}  # camera name -> (msg, receive time)
        self.create_subscription(JointState, T["joint_states"], self.on_js, 10, callback_group=self.cb)
        self.create_subscription(JointTrajectory, T["joint_command"], self.on_cmd, 10, callback_group=self.cb)
        self.create_subscription(PoseStamped, T["target_pose"], self.on_target, 10, callback_group=self.cb)
        self.create_subscription(Float32, T["gripper_command"], self.on_grip, LATCHED, callback_group=self.cb)
        self.create_subscription(String, T["status"], self.on_status, LATCHED, callback_group=self.cb)
        self.v4l2 = []
        for c in cfg["cameras"]:
            if "device" in c:
                self.v4l2.append(V4L2Camera(c, lambda img, n=c["name"]: self.on_image(n, img), self.get_logger()))
                continue
            typ = CompressedImage if c.get("compressed", False) else Image
            self.create_subscription(typ, c["topic"], lambda m, n=c["name"]: self.on_image(n, m),
                                     qos_profile_sensor_data, callback_group=self.cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.joy_prev = []
        if cfg.get("gamepad"):
            self.create_subscription(Joy, cfg["gamepad"]["topic"], self.on_joy, 10, callback_group=self.cb)

        self.lock = threading.Lock()
        self.episode = None
        self.writer_q = queue.Queue(maxsize=600)
        threading.Thread(target=self.writer, daemon=True).start()
        for name, fn in (("start", self.start), ("stop", self.stop), ("discard", lambda: self.stop(save=False))):
            self.create_service(Trigger, f"~/{name}", lambda req, resp, fn=fn: self.srv(fn, resp),
                                callback_group=self.cb)
        self.create_timer(1.0 / cfg["fps"], self.sample, callback_group=MutuallyExclusiveCallbackGroup())
        cams = ", ".join(f"{c['name']}={c.get('device', c.get('topic'))}" for c in cfg["cameras"]) or "none"
        self.get_logger().info(f"Recorder ready: {cfg['fps']} Hz, cameras: {cams}, output {self.out}")
        self.get_logger().info(f"Task: \"{self.task}\".  Keys: s start/stop+save  d discard  t task  q quit")

    # ------------------------------------------------------------------ io
    def on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in self.joint_names):
            self.q = np.array([msg.position[idx[n]] for n in self.joint_names])
            self.js_stamp = time.monotonic()
        if all(n in idx for n in self.finger_joints):
            self.width = float(sum(msg.position[idx[n]] for n in self.finger_joints))

    def on_cmd(self, msg):
        if not msg.points:
            return
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        if all(n in idx for n in self.joint_names):
            p = msg.points[-1].positions
            self.q_cmd = np.array([p[idx[n]] for n in self.joint_names])

    def on_target(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.target = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])

    def on_grip(self, msg):
        self.grip_cmd = float(msg.data)

    def on_status(self, msg):
        self.engaged = msg.data.startswith("ENGAGED")

    def on_joy(self, msg):
        G = self.cfg["gamepad"]
        pressed = lambda i: 0 <= i < len(msg.buttons) and msg.buttons[i] and not (  # noqa: E731
            i < len(self.joy_prev) and self.joy_prev[i])
        if pressed(G["start_stop_button"]):
            threading.Thread(target=self.stop if self.episode is not None else self.start, daemon=True).start()
        elif pressed(G["discard_button"]):
            threading.Thread(target=self.stop, kwargs={"save": False}, daemon=True).start()
        self.joy_prev = list(msg.buttons)

    def on_image(self, name, msg):
        """msg: ROS Image / CompressedImage, or an RGB array from a V4L2Camera."""
        self.images[name] = (msg, time.monotonic())

    def ee_pose(self):
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    def srv(self, fn, resp):
        fn()
        resp.success = True
        resp.message = "recording" if self.episode is not None else "idle"
        return resp

    # ------------------------------------------------------------ episodes
    def next_index(self):
        done = [int(p.name.split("_")[1]) for p in self.out.glob("episode_*") if p.name.split("_")[1].isdigit()]
        return max(done, default=-1) + 1

    def start(self):
        with self.lock:
            if self.episode is not None:
                return
            if self.q is None or time.monotonic() - self.js_stamp > 0.5:
                self.get_logger().error("No fresh joint states - not recording")
                return
            missing = [c["name"] for c in self.cfg["cameras"] if c["name"] not in self.images]
            if missing:
                self.get_logger().error(f"No images from camera(s) {missing} - not recording")
                return
            try:
                self.ee_pose()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"TF {self.base_frame}->{self.ee_link} unavailable: {e}")
                return
            path = self.out / f"episode_{self.next_index():06d}"
            self.episode = Episode(path, self.task, self.cfg["fps"], self.cfg["cameras"])
            self.t0 = time.monotonic()
            self.get_logger().info(f"RECORDING {path.name}  task=\"{self.task}\"")

    def stop(self, save=True):
        with self.lock:
            ep, self.episode = self.episode, None
        if ep is None:
            return
        self.writer_q.join()  # all images on disk
        if not save or len(ep) == 0:
            shutil.rmtree(ep.path, ignore_errors=True)
            self.get_logger().warning(f"DISCARDED {ep.path.name}")
            return
        np.savez(ep.path / "frames.npz", **{k: np.array(v) for k, v in ep.rows.items()})
        meta = {
            "task": ep.task,
            "fps": ep.fps,
            "num_frames": len(ep),
            "start_time": ep.start.isoformat(timespec="seconds"),
            "robot_type": "franka_panda",
            "names": {
                "observation.state": [*self.joint_names, "gripper_width"],
                "observation.ee_pose": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                "action": [*self.joint_names, "gripper_command"],
                "action.ee_pose": ["x", "y", "z", "qx", "qy", "qz", "qw"],
            },
            "cameras": {c["name"]: list(ep.image_shapes.get(c["name"], ())) for c in ep.cameras},
        }
        (ep.path / "meta.json").write_text(json.dumps(meta, indent=2))
        engaged = int(np.sum(ep.rows["engaged"]))
        self.get_logger().info(f"SAVED {ep.path.name}: {len(ep)} frames ({len(ep) / ep.fps:.1f} s, "
                               f"{engaged} engaged)")

    def writer(self):
        quality = [cv2.IMWRITE_JPEG_QUALITY, int(self.cfg["jpeg_quality"])]
        while True:
            path, rgb = self.writer_q.get()
            try:
                cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), quality)
            finally:
                self.writer_q.task_done()

    # -------------------------------------------------------------- sample
    def sample(self):
        with self.lock:
            ep = self.episode
            if ep is None:
                return
            now = time.monotonic()
            if now - self.js_stamp > 0.5:
                self.get_logger().warning("joint states stale - frame skipped", throttle_duration_sec=1.0)
                return
            try:
                ee = self.ee_pose()
            except Exception:  # noqa: BLE001
                self.get_logger().warning("TF unavailable - frame skipped", throttle_duration_sec=1.0)
                return
            width = self.width if self.width is not None else 0.0
            # gripper command: last teleop command, else the current hand state
            grip = self.grip_cmd if self.grip_cmd is not None else float(width < 0.04)
            # while paused (or before the first command) the robot holds still -> action = state
            q_cmd = self.q_cmd if self.engaged and self.q_cmd is not None else self.q
            target = self.target if self.engaged and self.target is not None else ee

            frame = len(ep)
            for c in self.cfg["cameras"]:
                msg, stamp = self.images[c["name"]]
                if now - stamp > self.cfg["max_image_age"]:
                    self.get_logger().warning(f"camera {c['name']} frame is {now - stamp:.2f}s old",
                                              throttle_duration_sec=1.0)
                rgb = msg if isinstance(msg, np.ndarray) else image_to_rgb(msg)
                if "width" in c and "height" in c and rgb.shape[:2] != (c["height"], c["width"]):
                    rgb = cv2.resize(rgb, (c["width"], c["height"]), interpolation=cv2.INTER_AREA)
                ep.image_shapes[c["name"]] = rgb.shape
                try:
                    self.writer_q.put_nowait((ep.path / c["name"] / f"{frame:06d}.jpg", rgb))
                except queue.Full:
                    self.get_logger().error("image writer cannot keep up - frame dropped")
                    return

            r = ep.rows
            r["timestamp"].append(now - self.t0)
            r["engaged"].append(self.engaged)
            r["observation.state"].append(np.append(self.q, width))
            r["observation.ee_pose"].append(ee)
            r["action"].append(np.append(q_cmd, grip))
            r["action.ee_pose"].append(np.asarray(target, dtype=float))
            if frame % (5 * ep.fps) == 0 and frame:
                self.get_logger().info(f"  {ep.path.name}: {frame / ep.fps:.0f} s")


def keyboard_thread(node: RecorderNode):
    if not sys.stdin.isatty():
        node.get_logger().info("stdin is not a TTY - keyboard disabled")
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            k = sys.stdin.read(1)
            if k == "s":
                node.stop() if node.episode is not None else node.start()
            elif k == "d":
                node.stop(save=False)
            elif k == "t":
                if node.episode is not None:
                    node.get_logger().warning("stop the episode before changing the task")
                    continue
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                try:
                    task = input("task> ").strip()
                finally:
                    tty.setcbreak(fd)
                if task:
                    node.task = task
                node.get_logger().info(f"Task: \"{node.task}\"")
            elif k == "q":
                node.stop()
                rclpy.try_shutdown()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(HERE / "config" / "record.yaml"))
    ap.add_argument("--task", help="language instruction for the episodes (default: from config)")
    ap.add_argument("--output-dir", help="override output_dir from the config")
    args, ros_args = ap.parse_known_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    cfg["cameras"] = cfg.get("cameras") or []
    if args.output_dir:
        cfg["output_dir"] = args.output_dir

    rclpy.init(args=ros_args)
    node = RecorderNode(cfg, args.task or cfg["task"])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=keyboard_thread, args=(node,), daemon=True).start()
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001  context invalidated by shutdown ('q' key / SIGTERM)
        if rclpy.ok():
            raise
    finally:
        node.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
