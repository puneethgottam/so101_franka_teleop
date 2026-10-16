#!/usr/bin/env bash
# One-time setup on a new PC. Safe to re-run.
#   ./setup.sh
set -e
cd "$(dirname "$0")"
ROS=/opt/ros/jazzy

echo "== checking ROS 2 Jazzy + packages"
[ -f $ROS/setup.bash ] || { echo "!! ROS 2 Jazzy not found in $ROS"; exit 1; }
missing=()
for p in franka_moveit_config franka_description franka_hardware franka_gripper franka_msgs \
         franka_robot_state_broadcaster libfranka moveit_ros_move_group pymoveit2 \
         joint_trajectory_controller parallel_gripper_controller moveit_resources_panda_moveit_config \
         robot_state_publisher joint_state_publisher rviz2 xacro; do
  [ -d $ROS/share/$p ] || [ -f $ROS/include/franka/robot.h -a $p = libfranka ] || missing+=("$p")
done
if [ ${#missing[@]} -gt 0 ]; then
  echo "!! missing ROS packages: ${missing[*]}"
  echo "   install e.g.: sudo apt install $(printf 'ros-jazzy-%s ' "${missing[@]//_/-}")"
  exit 1
fi

echo "== python venv (.venv, uses the system python so rclpy works)"
PY=$(head -1 $ROS/bin/ros2 | sed 's/^#!//')
[ -x "$PY" ] || PY=/usr/bin/python3
if [ ! -d .venv ]; then
  if command -v uv >/dev/null; then uv venv --python "$PY" --system-site-packages .venv
  else "$PY" -m venv --system-site-packages .venv; fi
fi
if command -v uv >/dev/null; then uv pip install --python .venv/bin/python feetech-servo-sdk pyserial
else .venv/bin/pip install -q feetech-servo-sdk pyserial; fi
source $ROS/setup.bash
.venv/bin/python -c "import rclpy, scservo_sdk, numpy, yaml; from franka_msgs.action import Grasp; print('   python deps OK')"

echo "== building tools/franka_recover (clears Franka reflex/error state, no motion)"
LIBDIR=$(dirname "$(find $ROS/lib -name 'libfranka.so' | head -1)")
g++ -std=c++17 -O2 tools/franka_recover.cpp -I$ROS/include -L"$LIBDIR" -Wl,-rpath,"$LIBDIR" -lfranka \
    -o tools/franka_recover && echo "   built tools/franka_recover"

echo "== leader arm serial port"
ls /dev/serial/by-id/ 2>/dev/null || echo "   (no USB serial device found - plug in the SO101 leader)"
id -nG | grep -qw dialout || echo "   note: you are not in 'dialout'; if the port is not readable: sudo usermod -aG dialout $USER (re-login)"

echo "== real-time checks (needed for the real robot)"
uname -v | grep -q PREEMPT_RT && echo "   PREEMPT_RT kernel: yes" || echo "!! not a PREEMPT_RT kernel"
echo "   rtprio limit: $(ulimit -r)  (want 99)   memlock: $(ulimit -l)  (want unlimited)"
echo "   CPU governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)  (want performance)"
echo
echo "Done. Next: ./run_franka_stack.sh (sim)   and   ./run_teleop.sh   - see README.md"
