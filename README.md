# SO101 → Franka Panda teleoperation

Teleoperate a Franka Emika Panda with a LeRobot **SO101 leader arm**, using end-effector
tracking instead of joint mapping: the leader's EE pose (URDF forward kinematics) is tracked
relative to the moment you engage, solved with MoveIt IK (`/compute_ik`) and streamed to the
`panda_arm_controller`. The leader gripper opens/closes the Franka Hand.

Targets ROS 2 Jazzy (Ubuntu 24.04, PREEMPT_RT); originally tested with ROS 2 Lyrical on Ubuntu 26.04, franka_ros2 0.1.x, libfranka 0.9.2.

## Setup on a new PC

Requirements: ROS 2 Jazzy with the Franka + MoveIt packages
(`franka_moveit_config`, `franka_hardware`, `franka_gripper`, `libfranka`, `pymoveit2`,
`moveit_ros_move_group`, `parallel_gripper_controller`, `moveit_resources_panda_moveit_config`).

```bash
git clone <this repo> && cd so101_franka_teleop
./setup.sh
```
`setup.sh` checks the ROS packages, creates `.venv` (system Python, so `rclpy` works) with
`feetech-servo-sdk` + `pyserial`, builds `tools/franka_recover`, and reports serial-port and
real-time settings.

**Leader calibration**: `calibration/main_leader.json` is the lerobot calibration of *this*
leader arm (same format as `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`).
Using a different leader arm → replace the file. The serial port is set in
`config/teleop.yaml` (`leader.port`, by-id path of this arm's USB adapter; `/dev/ttyACM0` also works).

Check the leader (read only, never moves it):
```bash
./check_leader.sh
```
`grip=` must go up when the leader gripper opens (otherwise set `gripper.invert: true`).

## Run

Two terminals.

**Simulation** (fake hardware, simulated hand):
```bash
./run_franka_stack.sh        # Franka + MoveIt + RViz (shows Franka, leader model, target)
./run_teleop.sh              # add --fake-leader to test without the leader arm
```

**Real robot** (brakes unlocked, FCI active in Desk, lights blue, e-stop in hand):
```bash
tools/franka_recover <robot_ip>                          # clears old reflex/errors, no motion
sudo prlimit --pid $$ --memlock=unlimited:unlimited      # in terminal 1, see below
./run_franka_stack.sh real <robot_ip>                    # optional: gripper:=false
./run_teleop.sh
```

| Key | Action |
|---|---|
| `SPACE` | engage / pause (clutch). On engage the current leader and Franka poses are latched → no jump |
| `h` | move Franka to the ready pose with MoveIt (10 % speed) |
| `r` | orientation tracking on/off (off = TCP orientation fixed, position only) |
| `g` | gripper control on/off |
| `q` | quit |

Services: `/so101_franka_teleop/engage` (SetBool), `/so101_franka_teleop/home` (Trigger).

Terminal-1 output is saved to `logs/`.

### PS5 controller instead of the leader

```bash
./run_teleop.sh --gamepad     # instead of ./run_teleop.sh; also starts joy's game_controller_node
```
Works with a DualSense over USB or Bluetooth (any SDL game controller should work). Instead of
following the leader's pose, the sticks and triggers set a **velocity** for the Franka TCP: the arm
moves while you hold a stick and stops when you let go. Directions are in the robot base frame
(`panda_link0`): +x points away from the base, +y to the robot's left, +z up. Speed limits,
workspace box, MoveIt IK, joint-limit/jump checks and the robot watchdog are the same as with the leader.

**Typical use**
1. Start the stack (terminal 1), then `./run_teleop.sh --gamepad` (terminal 2).
2. Press `CREATE` to move to the ready pose, then `OPTIONS` to engage.
3. Drive with the sticks and triggers; `CROSS` / `CIRCLE` to grasp / open.
4. `OPTIONS` again to pause; the robot holds its last pose.

**Controls**

| Control | Action |
|---|---|
| `OPTIONS` | engage / pause (keyboard `SPACE` also works) |
| left stick up / down | move towards / away from the base (x, inverted with `sign.x: -1`) |
| left stick left / right | move sideways: towards -y / +y (y, inverted with `sign.y: -1`) |
| `L2` / `R2` | move up / down (z); analog, press harder to go faster |
| right stick left / right | yaw: rotate the gripper about the vertical axis |
| right stick up / down | pitch: tilt the gripper about the base y axis |
| `L1` / `R1` | roll: rotate the gripper about the base x axis |
| `TRIANGLE` | rotation control on / off (off = only position, orientation stays fixed) |
| `CROSS` | close gripper (grasp, force controlled) |
| `CIRCLE` | open gripper |
| D-pad up / down | speed up / slow down: x1.5 / ÷1.5 per press, 10 %..100 % |
| `CREATE` | home: MoveIt moves the arm to the ready pose (10 % speed); pauses teleop |
| `SQUARE` | recorder: start episode / stop and save (needs `./run_recorder.sh`) |
| D-pad left | recorder: discard the current episode |

Full speed is `max_lin_vel` 0.10 m/s and `max_ang_vel` 0.5 rad/s. The current speed factor is
printed in the terminal and on `/so101_franka_teleop/status`.

**Notes**
- **Holding still:** if a pose is out of reach (`IK failed (code -31)`) or outside the workspace box,
  the arm stops at the last reachable pose. Move back the other way to recover. Poses close
  to the base or with large rotations fail most often; turning rotation off (`TRIANGLE`) helps.
- **Controller lost:** teleop pauses if the controller is unplugged / switched off or the joy node stops.
- **Real robot:** press D-pad down a few times after engaging so the first moves are slow.

**Changing the mapping:** everything is under `gamepad:` in `config/teleop.yaml`. `axes:` and
`buttons:` hold the controller indices (check them with `ros2 topic echo /joy` while pressing), and
`sign:` flips a direction (`x`, `y`, `yaw`, `pitch`). To flip up/down, swap `z_up` and `z_down`.
Restart `./run_teleop.sh --gamepad` after editing.

## Recording episodes (VLA / LeRobot data)

A third terminal records episodes while you teleoperate; a converter turns them into a
[LeRobot](https://github.com/huggingface/lerobot) dataset (parquet + mp4).

```bash
./run_recorder.sh --task "pick up the red cube"     # terminal 3
```
| Key | Action |
|---|---|
| `s` | start episode / stop and save |
| `d` | discard the current episode |
| `t` | type a new task (language instruction) |
| `q` | quit (saves a running episode) |

Services: `/so101_franka_recorder/start`, `/stop`, `/discard` (Trigger). Settings in `config/record.yaml`.

Recorded per frame (30 Hz): `observation.state` = 7 joints + finger width, `observation.ee_pose` = TCP
xyz + quaternion (xyzw), `action` = 7 commanded joints + gripper command (1 grasp / 0 open),
`action.ee_pose` = commanded TCP pose, and one image per camera. While teleop is paused the action
equals the measured state. Raw episodes go to `data/raw/episode_NNNNNN/`.

**Cameras**: listed under `cameras:` in `config/record.yaml`. A USB camera is read directly
with OpenCV (`device:`); a camera with a ROS driver is read from its image topic (`topic:`). The
`front` camera (HDMI/USB capture, scene view) is configured: 1280x720 MJPG at 30 fps, saved at 640x360.
```yaml
cameras:
  - name: front
    device: /dev/v4l/by-id/usb-HDMI_3.0_USB_Camera_HDMI_USB_Camera_2023121018-video-index0
    capture_width: 1280
    capture_height: 720
    width: 640
    height: 360
  - name: wrist                            # e.g. Orbbec: ros2 launch orbbec_camera <model>.launch.py
    topic: /camera/color/image_raw
```
With a camera configured, recording only starts once it delivers frames.

**Convert** (separate venv: lerobot needs numpy 2, ROS Jazzy needs numpy 1.26):
```bash
python3 -m venv .venv-lerobot
.venv-lerobot/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu "lerobot[dataset]"
tools/to_lerobot.sh data/raw --repo-id <hf_user>/franka_pick --root data/lerobot/franka_pick   # --overwrite, --push
```
Paused frames at the start/end of each episode are trimmed (`--keep-paused` keeps them) and each
episode is resampled onto an exact 1/fps grid.

## How it works

```
SO101 (USB, 30 Hz) → FK → Δpose since engage ×scale → speed/workspace limits
   → /compute_ik (move_group, KDL, collision-checked, seeded with last solution)
   → joint-limit margin + jump check → /panda_arm_controller/joint_trajectory
   → JointTrajectoryController (1 kHz, effort PD) → libfranka (FCI UDP 1 kHz) → robot
leader gripper < close_below → /panda_gripper/grasp (force), > open_above → /panda_gripper/move
```
Safety layers: TCP speed limit, workspace box, IK rejection near joint limits or on joint
jumps, and a **watchdog** on `/franka_robot_state_broadcaster/robot_state`: if the robot's clock
stops advancing or the mode is not *Move*, teleop pauses and refuses to engage/home.

All tuning is in `config/teleop.yaml` (scale, speeds, filter, workspace, gripper force/thresholds).

## Files

| Path | |
|---|---|
| `teleop_node.py` | the teleop ROS 2 node |
| `so101_leader.py` | read-only Feetech bus reader + URDF forward kinematics |
| `launch/franka_stack.launch.py` | wraps `franka_moveit_config/moveit.launch.py`, adds leader model, RViz, sim hand |
| `recorder_node.py` | episode recorder (state, actions, camera images) -> `data/raw/` |
| `tools/to_lerobot.py` | raw episodes -> LeRobot dataset |
| `tools/sim_gripper.py` | simulated Franka Hand (move/grasp actions) for fake hardware |
| `tools/franka_recover.cpp` | libfranka `automaticErrorRecovery()` + prints mode/last errors |
| `urdf/` | SO101 URDF + meshes (TheRobotStudio `so101_new_calib.urdf`, matches lerobot calibration) |

## Real-robot PC requirements (communication_constraints_violation)

The Franka needs a reply every 1 ms. If the PC misses many packets, the robot stops with the
`communication_constraints_violation` reflex (`tools/franka_recover` prints it under
*last motion errors*; teleop reports `robot state FROZEN`). On the control PC:

- PREEMPT_RT kernel, user in a group with `rtprio 99` and **`memlock unlimited`**
  (`/etc/security/limits.conf`; desktop sessions may need a reboot or the `prlimit` line above)
- CPU governor `performance`:
  `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`
- Energy-Efficient Ethernet off on the robot NIC: `sudo ethtool --set-eee <iface> eee off`
- Prefer an **Intel NIC** dedicated to the robot; Realtek (r8169) NICs are known to drop packets
- Avoid heavy GPU jobs on the same PC while controlling the robot

The governor and EEE settings reset on reboot.
