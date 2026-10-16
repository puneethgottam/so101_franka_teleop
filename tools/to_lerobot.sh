#!/usr/bin/env bash
# Convert raw recorder episodes to a LeRobot dataset (runs in .venv-lerobot, no ROS).
#   tools/to_lerobot.sh data/raw --repo-id <hf_user>/<name> --root data/lerobot/<name> [--overwrite] [--push]
set -e
cd "$(dirname "$0")/.."
exec .venv-lerobot/bin/python tools/to_lerobot.py "$@"
