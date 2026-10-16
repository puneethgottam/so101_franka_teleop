#!/usr/bin/env bash
# Read-only check of the SO101 leader: prints joint angles, gripper and EE position.
cd "$(dirname "$0")" && exec .venv/bin/python so101_leader.py "${1:-/dev/ttyACM0}"
