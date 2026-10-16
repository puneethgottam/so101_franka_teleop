#!/usr/bin/env bash
# Terminal 2: SO101 leader -> Franka EE teleop.  Extra args are passed through (e.g. --fake-leader).
#   ./run_teleop.sh --gamepad   PS5 / game controller instead of the leader (starts joy's game_controller_node)
# Keys: SPACE engage/pause | r orientation on/off | g gripper on/off | h home | q quit
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
if [[ " $* " == *" --gamepad "* ]]; then
  # autorepeat keeps /joy flowing while sticks are held still (teleop pauses if it stops)
  ros2 run joy game_controller_node --ros-args -p deadzone:=0.1 -p autorepeat_rate:=20.0 \
    > logs/joy_$(date +%Y%m%d_%H%M%S).log 2>&1 &
  JOY_PID=$!
  trap 'kill $JOY_PID 2>/dev/null' EXIT
  .venv/bin/python teleop_node.py "$@"
else
  exec .venv/bin/python teleop_node.py "$@"
fi
