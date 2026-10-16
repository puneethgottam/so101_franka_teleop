#!/usr/bin/env bash
# Terminal 1: Franka + MoveIt + RViz (shows Franka, SO101 leader model and teleop target)
#   ./run_franka_stack.sh                 -> SIMULATION (fake hardware)   [default]
#   ./run_franka_stack.sh real 172.16.0.2 -> REAL robot via FCI
#   extra launch args go at the end, e.g.:  ./run_franka_stack.sh real 172.16.0.2 gripper:=false
set -eo pipefail
cd "$(dirname "$0")"
mkdir -p logs
LOG="logs/franka_stack_$(date +%Y%m%d_%H%M%S).log"
echo ">>> full output is also saved to $LOG"
source /opt/ros/jazzy/setup.bash
if [ "$1" = "real" ]; then
  IP="${2:?usage: $0 real <robot_ip>}"
  echo ">>> REAL ROBOT at $IP"
  echo ">>> memlock limit: $(ulimit -l)   CPU governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
  [ "$(ulimit -l)" = "unlimited" ] || echo "!!! memlock is not unlimited - run: sudo prlimit --pid \$\$ --memlock=unlimited:unlimited"
  ros2 launch launch/franka_stack.launch.py use_fake_hardware:=false robot_ip:="$IP" "${@:3}" 2>&1 | tee "$LOG"
else
  echo ">>> SIMULATION (fake hardware)"
  ros2 launch launch/franka_stack.launch.py use_fake_hardware:=true "${@:2}" 2>&1 | tee "$LOG"
fi
