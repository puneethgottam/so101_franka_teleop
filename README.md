# SO101 → Franka Panda teleoperation

Teleoperate a Franka Emika Panda with a LeRobot **SO101 leader arm**, using end-effector
tracking instead of joint mapping: the leader's EE pose (URDF forward kinematics) is tracked
relative to the moment you engage, solved with MoveIt IK (`/compute_ik`) and streamed to the
`panda_arm_controller`. The leader gripper opens/closes the Franka Hand.

Tested with ROS 2 Lyrical on Ubuntu 26.04 (PREEMPT_RT), franka_ros2 0.1.x, libfranka 0.9.2.

## Setup on a new PC

Requirements: ROS 2 Lyrical with the Franka + MoveIt packages
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
