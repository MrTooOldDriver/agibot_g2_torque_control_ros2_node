#!/usr/bin/env bash
# Environment of the G2 torque subscriber container: ROS 2, then the AGIBOT GDK, then the workspace
# overlay (if built). Never source the GDK's gdk/scripts/ros_env.sh here: it points ROS at the
# robot's own DDS domain.
#
# G2_GDK_MOUNT is where docker/run.sh mounted the GDK app dir: /opt/agibot/app on the dev PC, and
# /home/agi/app (the same path as on the host) on the robot's own computer.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
GDK=${G2_GDK_MOUNT:-/opt/agibot/app}
if [ -f "$GDK/env.sh" ]; then
  source "$GDK/env.sh" "$GDK" > /dev/null
  export GDK_APP_DIR=$GDK
  # The GDK's cosine bus reads its UDP topic table (robot state on 8852, commands on 8851) only
  # from COSINE_BUS_CONFIG_PATH. The dev-PC env.sh sets it; the robot's env.sh does not.
  if [ -z "${COSINE_BUS_CONFIG_PATH:-}" ] && [ -f "$GDK/conf/dds/cosine_ipc.prototxt" ]; then
    export COSINE_BUS_CONFIG_PATH=$GDK/conf/dds/cosine_ipc.prototxt
  fi
  export COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=${COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE:-1}
else
  echo "g2-entrypoint: no GDK at $GDK (only backend:=fake / the fake agibot_gdk work)" >&2
fi
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
