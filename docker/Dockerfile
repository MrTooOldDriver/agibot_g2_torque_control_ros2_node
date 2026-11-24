# ROS 2 image for the G2 torque subscriber. The ROS distro must match the GDK it runs with:
#   humble (Ubuntu 22.04, Python 3.10): x86_64 dev PC, GDK binding agibot_gdk.cpython-310-x86_64
#   jazzy  (Ubuntu 24.04, Python 3.12): the robot's own computer (Jetson, aarch64). Its GDK needs
#          glibc 2.38 and GLIBCXX_3.4.32 and its binding is cpython-312, so Humble cannot load it.
# docker/run.sh picks the distro from the CPU (override with ROS_DISTRO=...).
# The GDK itself is NOT baked in: docker/run.sh mounts it.
#
#   docker build --build-arg ROS_DISTRO=jazzy -t g2-torque-subscriber:jazzy -f docker/Dockerfile docker
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-pinocchio \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
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
