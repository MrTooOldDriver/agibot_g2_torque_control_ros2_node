# ROS 2 Humble (Ubuntu 22.04, Python 3.10) image for the G2 torque subscriber.
# Python 3.10 is required by the GDK Python binding (agibot_gdk.cpython-310-x86_64-linux-gnu.so).
# The GDK itself is NOT baked in: docker/run.sh mounts it at /opt/agibot/app.
#
#   docker build -t g2-torque-subscriber:humble -f docker/Dockerfile docker
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-pinocchio \
      ros-humble-rmw-cyclonedds-cpp \
      python3-numpy \
      iproute2 iputils-ping \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/g2-entrypoint
RUN chmod 0755 /usr/local/bin/g2-entrypoint

WORKDIR /ws
# docker stop sends SIGINT (Ctrl+C semantics): the node then restores the robot's control mode.
STOPSIGNAL SIGINT
ENTRYPOINT ["/usr/local/bin/g2-entrypoint"]
CMD ["bash"]
