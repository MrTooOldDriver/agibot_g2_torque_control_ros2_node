#!/usr/bin/env bash
# Start the ROS 2 container for the G2 torque subscriber.
#
#   docker/run.sh                      # interactive shell
#   docker/run.sh colcon build --symlink-install
#   docker/run.sh ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=...
#
# Two profiles, picked automatically:
#   dev PC (default): ROS 2 Humble image, GDK from GDK_APP_DIR (default <repo>/agibot_gdk/app)
#     mounted at /opt/agibot/app. The robot streams its state to 10.42.1.102:8852/UDP on the host.
#   robot: the robot's own computer (aarch64 with the GDK at /home/agi/app). ROS 2 Jazzy image
#     (the robot GDK needs Ubuntu 24.04), GDK mounted read-only at the same path /home/agi/app,
#     ROS discovery kept on localhost, and real-time priority capped below the robot's own
#     control threads. The state stream is local (10.42.1.101:8852). The container runs as root,
#     like the vendor's torque demo: the robot services' /dev/shm segments are root-only (0644).
#     /data is mounted read-only (the GDK reads /data/version, /data/parameters) and /data/logs
#     writable (the GDK's LOG_PATH). Use sudo there: agi is not in the docker group.
#
# Other mounts: the package at /ws/src/g2_torque_subscriber, the workspace build output at /ws
# (G2_WS_DIR, default <repo>/docker/ws), and optionally MODELS_DIR (URDFs) at /models.
# Overrides: G2_PROFILE=robot|pc, G2_ROS_DISTRO, IMAGE, GDK_APP_DIR, G2_GDK_MOUNT, G2_RTPRIO.
# sudo drops the caller's environment: pass overrides inline, e.g. sudo MODELS_DIR=... docker/run.sh
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ROBOT_GDK=/home/agi/app

if [ -z "${G2_PROFILE:-}" ]; then
  G2_PROFILE=pc
  if [ "$(uname -m)" = aarch64 ] && [ -f "$ROBOT_GDK/env.sh" ]; then G2_PROFILE=robot; fi
fi
case "$G2_PROFILE" in
  robot)
    G2_ROS_DISTRO=${G2_ROS_DISTRO:-jazzy}
    GDK_APP_DIR=${GDK_APP_DIR:-$ROBOT_GDK}
    G2_GDK_MOUNT=${G2_GDK_MOUNT:-$ROBOT_GDK}
    # SCHED_FIFO ceiling without CAP_SYS_NICE. The GDK torque loop asks for 10; the robot's own
    # EtherCAT / control threads run at 20 and above, and nothing in here may outrank them.
    G2_RTPRIO=${G2_RTPRIO:-15}
    rt_args=(--ulimit "rtprio=$G2_RTPRIO")
    user_args=(--user 0:0)
    extra_mounts=()
    [ -d /data ] && extra_mounts+=(-v /data:/data:ro)
    [ -d /data/logs ] && extra_mounts+=(-v /data/logs:/data/logs)
    # Everything (node, command publisher) runs on the robot: keep DDS off its internal networks.
    export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
    ;;
  pc)
    G2_ROS_DISTRO=${G2_ROS_DISTRO:-humble}
    GDK_APP_DIR=${GDK_APP_DIR:-$REPO/agibot_gdk/app}
    G2_GDK_MOUNT=${G2_GDK_MOUNT:-/opt/agibot/app}
    G2_RTPRIO=${G2_RTPRIO:-99}
    rt_args=(--ulimit "rtprio=$G2_RTPRIO" --cap-add SYS_NICE)  # lets the torque loop use SCHED_FIFO
    user_args=(--user "$(id -u):$(id -g)")
    extra_mounts=()
    ;;
  *)
    echo "run.sh: G2_PROFILE must be robot or pc, not '$G2_PROFILE'" >&2
    exit 2
    ;;
esac
IMAGE=${IMAGE:-g2-torque-subscriber:$G2_ROS_DISTRO}
G2_WS_DIR=${G2_WS_DIR:-$REPO/docker/ws}

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  docker build --build-arg ROS_DISTRO="$G2_ROS_DISTRO" -t "$IMAGE" \
    -f "$REPO/docker/Dockerfile" "$REPO/docker"
fi
mkdir -p "$G2_WS_DIR/src"

args=(--rm --network host --ipc host
      "${rt_args[@]}" "${user_args[@]}" "${extra_mounts[@]}"
      -e HOME=/ws
      -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION -e ROS_LOCALHOST_ONLY
      -e ROS_AUTOMATIC_DISCOVERY_RANGE -e ROS_STATIC_PEERS
      -e G2_FAKE_URDF -e G2_FAKE_INITIAL_POSE -e G2_FAKE_FRICTION
      -e G2_GDK_MOUNT="$G2_GDK_MOUNT"
      -v "$G2_WS_DIR":/ws
      -v "$REPO/g2_torque_subscriber":/ws/src/g2_torque_subscriber)
if [ -d "$GDK_APP_DIR" ]; then
  args+=(-v "$(cd "$GDK_APP_DIR" && pwd)":"$G2_GDK_MOUNT":ro)
else
  echo "run.sh: GDK_APP_DIR=$GDK_APP_DIR not found; starting without the GDK" >&2
fi
if [ -n "${MODELS_DIR:-}" ]; then
  args+=(-v "$(cd "$MODELS_DIR" && pwd)":/models:ro)
fi
[ -t 0 ] && args+=(-it)
[ $# -eq 0 ] && set -- bash

exec docker run "${args[@]}" "$IMAGE" "$@"
