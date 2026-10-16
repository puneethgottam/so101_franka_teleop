#!/usr/bin/env bash
# Replay a recorded episode on the Franka (sim or real; the stack must be running).
#   tools/replay_episode.sh data/raw/episode_000000 [--speed 0.5] [--no-gripper] [-y]
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
exec .venv/bin/python tools/replay_episode.py "$@"
