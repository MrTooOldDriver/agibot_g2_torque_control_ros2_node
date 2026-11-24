#!/usr/bin/env bash
# Environment of the G2 torque subscriber container: ROS 2 Humble, then the AGIBOT GDK, then the
# workspace overlay (if built). Never source the GDK's gdk/scripts/ros_env.sh here: it points ROS at
# the robot's own DDS domain.
source /opt/ros/humble/setup.bash
if [ -f /opt/agibot/app/env.sh ]; then
  source /opt/agibot/app/env.sh /opt/agibot/app > /dev/null
  export GDK_APP_DIR=/opt/agibot/app
else
  echo "g2-entrypoint: no GDK at /opt/agibot/app (only backend:=fake / the fake agibot_gdk work)" >&2
fi
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
