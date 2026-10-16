"""SO101 leader arm: read-only Feetech bus reader + URDF forward kinematics.

Only READS from the motors (Present_Position). It never writes registers or
enables torque, so it cannot move or re-calibrate the leader arm.

Joint angle convention matches lerobot (MotorNormMode.DEGREES):
    deg = (raw - (range_min + range_max) / 2) * 360 / 4095
which is the zero of TheRobotStudio's `so101_new_calib.urdf`.
"""

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_MOTORS = ARM_JOINTS + ["gripper"]

ADDR_PRESENT_POSITION = 56  # STS3215, 2 bytes, sign bit 15
MAX_RES = 4095


# --------------------------------------------------------------------------
# Small rotation helpers (no scipy dependency)
# --------------------------------------------------------------------------
def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def matrix_to_quat(R):
    """Return quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_log(R):
    """Rotation matrix -> rotation vector (axis * angle)."""
    cos = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:
        # near pi: use the diagonal for a stable axis
        axis = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        axis[1] = math.copysign(axis[1], R[0, 1] + R[1, 0]) if axis[0] > 1e-6 else axis[1]
        axis[2] = math.copysign(axis[2], R[0, 2] + R[2, 0]) if axis[0] > 1e-6 else axis[2]
        return axis / np.linalg.norm(axis) * angle
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(angle))
    return w * angle


def rotvec_to_matrix(v):
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_matrix(v / angle, angle)


# --------------------------------------------------------------------------
# URDF forward kinematics
# --------------------------------------------------------------------------
class UrdfChainFK:
    """Forward kinematics along the serial chain base_link -> tip_link."""

    def __init__(self, urdf_path, base_link="base_link", tip_link="gripper_frame_link"):
        root = ET.parse(urdf_path).getroot()
        by_child = {}
        for j in root.findall("joint"):
            by_child[j.find("child").get("link")] = j
        chain = []
        link = tip_link
        while link != base_link:
            j = by_child[link]
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
            a = j.find("axis")
            axis = [float(v) for v in a.get("xyz").split()] if a is not None else [1, 0, 0]
            T = np.eye(4)
            T[:3, :3] = rpy_to_matrix(*rpy)
            T[:3, 3] = xyz
            chain.append((j.get("name"), j.get("type"), T, np.array(axis)))
            link = j.find("parent").get("link")
        self.chain = list(reversed(chain))

    def forward(self, joints_rad: dict) -> np.ndarray:
        T = np.eye(4)
        for name, jtype, T_origin, axis in self.chain:
            T = T @ T_origin
            if jtype in ("revolute", "continuous"):
                Tj = np.eye(4)
                Tj[:3, :3] = axis_angle_to_matrix(axis, joints_rad.get(name, 0.0))
                T = T @ Tj
            elif jtype == "prismatic":
                Tj = np.eye(4)
                Tj[:3, 3] = axis * joints_rad.get(name, 0.0)
                T = T @ Tj
        return T


# --------------------------------------------------------------------------
# Feetech bus reader (read only)
# --------------------------------------------------------------------------
class SO101LeaderReader:
    def __init__(self, port, calibration_file, baudrate=1_000_000):
        import scservo_sdk as scs

        self._scs = scs
        self.calib = json.loads(Path(calibration_file).expanduser().read_text())
        self.port = scs.PortHandler(port)
        self.packet = scs.PacketHandler(0)
        if not self.port.openPort():
            raise RuntimeError(f"Could not open {port}")
        if not self.port.setBaudRate(baudrate):
            raise RuntimeError(f"Could not set baudrate {baudrate}")
        self.sync = scs.GroupSyncRead(self.port, self.packet, ADDR_PRESENT_POSITION, 2)
        for m in ALL_MOTORS:
            self.sync.addParam(self.calib[m]["id"])

    def read_raw(self, retries=3):
        scs = self._scs
        for _ in range(retries):
            if self.sync.txRxPacket() == scs.COMM_SUCCESS:
                out = {}
                for m in ALL_MOTORS:
                    mid = self.calib[m]["id"]
                    if not self.sync.isAvailable(mid, ADDR_PRESENT_POSITION, 2):
                        break
                    out[m] = scs.SCS_TOHOST(self.sync.getData(mid, ADDR_PRESENT_POSITION, 2), 15)
                else:
                    return out
            time.sleep(0.002)
        raise IOError("Failed to sync-read SO101 leader positions")

    def read(self):
        """Return (arm joint angles in radians dict, gripper opening 0..1)."""
        raw = self.read_raw()
        joints = {}
        for m in ARM_JOINTS:
            c = self.calib[m]
            val = raw[m]
            if c.get("drive_mode", 0):
                val = c["range_max"] - (val - c["range_min"])
            mid = (c["range_min"] + c["range_max"]) / 2
            joints[m] = math.radians((val - mid) * 360.0 / MAX_RES)
        c = self.calib["gripper"]
        g = (raw["gripper"] - c["range_min"]) / max(1, (c["range_max"] - c["range_min"]))
        return joints, float(np.clip(g, 0.0, 1.0))

    def close(self):
        self.port.closePort()


class FakeLeader:
    """Synthetic leader motion for testing the pipeline without hardware."""

    def __init__(self):
        self.t0 = time.time()

    def read(self):
        t = time.time() - self.t0
        j = {
            "shoulder_pan": 0.35 * math.sin(0.4 * t),
            "shoulder_lift": -0.2 + 0.25 * math.sin(0.3 * t),
            "elbow_flex": 0.2 + 0.25 * math.sin(0.35 * t + 1.0),
            "wrist_flex": 0.6 + 0.3 * math.sin(0.5 * t),
            "wrist_roll": 0.4 * math.sin(0.25 * t),
        }
        return j, 0.5 + 0.5 * math.sin(0.2 * t)

    def close(self):
        pass


if __name__ == "__main__":
    # Quick hardware check:  python so101_leader.py [/dev/ttyACM0]
    import sys

    here = Path(__file__).parent
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    calib = here / "calibration" / "main_leader.json"
    fk = UrdfChainFK(here / "urdf" / "so101_new_calib.urdf")
    leader = SO101LeaderReader(port, calib)
    try:
        while True:
            j, g = leader.read()
            T = fk.forward(j)
            degs = " ".join(f"{k[:6]}={math.degrees(v):7.1f}" for k, v in j.items())
            p = T[:3, 3]
            print(f"\r{degs} grip={g:4.2f} | EE xyz=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        leader.close()
