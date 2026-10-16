#!/usr/bin/env bash
# Terminal 2: SO101 leader -> Franka EE teleop.  Extra args are passed through (e.g. --fake-leader).
# Keys: SPACE engage/pause | r orientation on/off | g gripper on/off | h home | q quit
set -e
cd "$(dirname "$0")"
source /opt/ros/lyrical/setup.bash
exec .venv/bin/python teleop_node.py "$@"
