#!/usr/bin/env bash
# Terminal 3: record teleop episodes (raw). Extra args are passed through, e.g. --task "pick up the red cube".
# Keys: s start/stop+save | d discard | t set task | q quit
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
exec .venv/bin/python recorder_node.py "$@"
