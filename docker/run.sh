#!/usr/bin/env bash
# Start the ROS 2 Humble container for the G2 torque subscriber.
#
#   docker/run.sh                      # interactive shell
#   docker/run.sh colcon build --symlink-install
#   docker/run.sh ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=...
#
# Mounts: the package at /ws/src/g2_torque_subscriber, the GDK app dir at /opt/agibot/app
# (GDK_APP_DIR, default <repo>/agibot_gdk/app), the workspace build output at /ws (G2_WS_DIR,
# default <repo>/docker/ws), and optionally MODELS_DIR (URDFs) at /models.
# Host networking is required: the robot streams its state to 10.42.1.102:8852/UDP on the host.
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=${IMAGE:-g2-torque-subscriber:humble}
GDK_APP_DIR=${GDK_APP_DIR:-$REPO/agibot_gdk/app}
G2_WS_DIR=${G2_WS_DIR:-$REPO/docker/ws}

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO/docker"
fi
mkdir -p "$G2_WS_DIR/src"

args=(--rm --network host --ipc host
      --ulimit rtprio=99 --cap-add SYS_NICE      # lets the torque loop use SCHED_FIFO
      --user "$(id -u):$(id -g)" -e HOME=/ws
      -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION -e ROS_LOCALHOST_ONLY
      -e G2_FAKE_URDF -e G2_FAKE_INITIAL_POSE -e G2_FAKE_FRICTION
      -v "$G2_WS_DIR":/ws
      -v "$REPO/g2_torque_subscriber":/ws/src/g2_torque_subscriber)
if [ -d "$GDK_APP_DIR" ]; then
  args+=(-v "$(cd "$GDK_APP_DIR" && pwd)":/opt/agibot/app:ro)
else
  echo "run.sh: GDK_APP_DIR=$GDK_APP_DIR not found; starting without the GDK" >&2
fi
if [ -n "${MODELS_DIR:-}" ]; then
  args+=(-v "$(cd "$MODELS_DIR" && pwd)":/models:ro)
fi
[ -t 0 ] && args+=(-it)
[ $# -eq 0 ] && set -- bash

exec docker run "${args[@]}" "$IMAGE" "$@"
