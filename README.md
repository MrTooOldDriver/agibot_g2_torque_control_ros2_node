# agibot_g2_torque_control_ros2_node

## Check the robot connection

Run this before the first launch. Steps 1 and 2 are read-only: the robot never enters torque mode.

```bash
# 1. Network (on the host)
ip -4 addr | grep 10.42.1.102                       # wired card must be exactly 10.42.1.102/24
ping -c 5 10.42.1.101                               # robot answers
sudo ping 10.42.1.101 -i 0.001 -c 10000 -s 1200     # 1 kHz link: 0% loss, round trip well under 1 ms
nc -zv 10.42.1.101 2379                             # GDK discovery service on the robot
sudo ufw status                                     # if active: sudo ufw allow 8852/udp

# 2. GDK, read-only (inside docker/run.sh)
echo $LOCATOR_IP $AORTA_DISCOVERY_URI               # expect: 10.42.1.102 http://10.42.1.101:2379
ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true urdf_path:=/models/<g2>.urdf
ros2 topic hz /g2_torque_subscriber/left/joint_states   # second shell: ~100 Hz

# 3. Torque-mode stream, UDP 8852 (watch during the first real run)
sudo tcpdump -ni <wired interface> udp port 8852 -c 20
```

- **The `echo` prints nothing:** the GDK's `env.sh` found no 10.42.1.x address, so the GDK
  cannot find the robot.
- **The dry run stops at "connecting to the robot":** the robot is not reachable through the GDK.
- **The dry run prints the gravity table:** the connection works. With the arms at rest, a small
  `diff` also shows the URDF matches the robot.
- **The first real run logs `JointTorqueControl returned kTimeout`:** no state arrives on UDP 8852
  (IP, firewall or link). The node restores the previous control mode and exits. Steps 1 and 2
  cannot check this stream, because the robot only uses it in torque mode.

## Quick start

```bash
cd agibot_g2_torque_control_ros2_node
docker/run.sh colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
MODELS_DIR=~/agibot_g2_torque_control/models/urdf docker/run.sh

# inside the container
ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true urdf_path:=/models/<g2>.urdf
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=/models/<g2>.urdf
ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py urdf_path:=/models/<g2>.urdf

# from a second shell (docker/run.sh): send a command to the left arm
ros2 topic pub -r 500 --qos-reliability best_effort /g2_torque_subscriber/left/external_torques \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 2.0, 0, 0, 0]}"
```

Direct joint torque control of the **AGIBOT G2** arms from ROS 2 topics: the G2 counterpart of the
`TorqueSubscriberController` in the franka_ros2 fork (`~/franka_ros2`,
`franka_example_controllers/src/torque_subscriber_controller.cpp`). There are two implementations
with the same interface: C++ (`torque_subscriber_node`) and Python (`torque_subscriber_node.py`).
Both run in a ROS 2 Humble container.

| | Franka `TorqueSubscriberController` | G2 `g2_torque_subscriber` |
|---|---|---|
| Command | `~/external_torques`, `Float64MultiArray`, 7 values | `~/left/external_torques` and `~/right/external_torques`, `Float64MultiArray`, 7 values each (joint1..7) |
| QoS | KeepLast(1), BestEffort, Volatile | same |
| Meaning | torque on top of the robot's gravity compensation | the same, but the **node** adds the gravity term g(q) (Pinocchio + URDF), because G2 mode 9 does none |
| No message for 100 ms | zero torque, so the arm floats | that arm's external torque goes to 0 and **gravity compensation continues**, so the arm floats. Zero *total* torque would drop a G2 arm. |
| Runs as | ros2_control controller | standalone node that owns the GDK connection (G2 has no ros2_control hardware interface) |
| Joint states | joint_state_broadcaster | published by the node from the 1 kHz robot frames |

## Torque law (every 1 ms frame, 14 arm joints)

```
tau = g(q) + tau_ext - joint_damping * dq                      g: Pinocchio gravity, waist/head at measured pose
tau = clamp(tau, +- effort_limit_scale * URDF effort limit)
tau = tau_prev + clamp(tau - tau_prev, +- max_torque_step)       tau_prev of frame 0 = measured effort (robot's holding torque)
```

`tau_ext` is the most recent command of each arm, or 0 if that arm's command is older than
`command_timeout`. The robot then shapes the torque further (about 2 Nm per frame slew, a 100 Hz
low-pass filter, and a clamp at the limit). See the GDK docs, `agibot_gdk/app/gdk/site/contents/howto/direct_torque/`.

## ROS interface

Default node name `g2_torque_subscriber` (`~` = `/<namespace>/g2_torque_subscriber`).

| Topic | Type | |
|---|---|---|
| `~/left/external_torques`, `~/right/external_torques` | `std_msgs/Float64MultiArray` (sub) | 7 values, Nm, joint1..7. Wrong length or NaN/Inf is rejected. |
| `joint_states` | `sensor_msgs/JointState` (pub) | all joints the robot reports (arms, waist, head) |
| `~/left/joint_states`, `~/right/joint_states` | `sensor_msgs/JointState` (pub) | the 7 joints of that arm: measured q, dq, effort |
| `~/commanded_torques` | `sensor_msgs/JointState` (pub) | 14 arm joints, `effort` = total torque sent (gravity included) |

Arm joints: left `idx21_arm_l_joint1` … `idx27_arm_l_joint7`, right `idx61_arm_r_joint1` …
`idx67_arm_r_joint7`. The publishers are reliable and publish every `publish_decimation`-th frame
(default: every frame, 1 kHz). All parameters, with comments, are in
[`g2_torque_subscriber/config/torque_subscriber.yaml`](g2_torque_subscriber/config/torque_subscriber.yaml).

## Lifecycle and safety

1. Connect (GDK), load the URDF, and read one state frame.
2. Pre-flight: the node checks that all 14 arm joints are present and that the model gravity g(q)
   matches the measured effort within `gravity_check_tolerance` (10 Nm). If it does not, the node
   refuses to start. A wrong URDF, an unregistered payload or a wrong waist pose would otherwise
   cause drift or the 50 Nm step e-stop.
3. Remember the control mode. The node refuses to start if the robot is already in mode 9. It then
   switches to mode 9 and waits for confirmation. **Do not touch the arms while entering.**
4. 1 kHz loop. Abort (final frame, then step 5) on: joint speed above `velocity_limit` (2 rad/s),
   non-finite state or torque, or a GDK read timeout or error.
5. **Restore the previous control mode** (the robot then holds the arms) and exit. Ctrl+C, SIGTERM,
   `docker stop`, aborts and errors all take this path. GDK never leaves mode 9 by itself.

Robot-side limits that still apply (from the GDK docs):

- A change of more than 50 Nm per frame on one joint triggers an **e-stop**; clear it with
  `ClearMotionControlError()`.
- 20 ms without a frame makes the robot leave mode 9 and hold the arms (communication error).
- There are no joint-limit walls and no extra collision layer in mode 9.

`max_torque_step` (15 Nm) keeps the node below the 50 Nm e-stop threshold.

## Why the GDK is loaded with `dlopen`

The GDK bundles Fast DDS 3.2, boost 1.83, protobuf and other libraries. If the node links the GDK
directly, ROS's `rmw_fastrtps` binds `eprosima::fastdds::dds::DomainParticipantFactory` to the
GDK's Fast DDS 3 and aborts (`std::bad_array_new_length`) when the first `rclcpp::Node` is created.
This was reproduced on the host. The C++ node therefore never links the GDK. It `dlopen`s
`libg2_gdk_backend.so` with `RTLD_LOCAL | RTLD_DEEPBIND` and talks to it only through the
`RobotBackend` vtable (`include/g2_torque_subscriber/robot_backend.hpp`). The Python node imports
`agibot_gdk` with the same flags. With either rmw_fastrtps or rmw_cyclonedds, both nodes load the
real GDK and keep their ROS interface working (`test/coload_check.sh`). This was verified without a
robot only, so the GDK's own bus never came up next to ROS.

## Setup (ROS 2 Humble container)

The GDK's Python binding is CPython 3.10 only, and the robot streams its state to
`10.42.1.102:8852/UDP`. The machine needs a direct 10 GbE link to the robot's debug port with IP
10.42.1.102, and the container uses host networking.

```bash
# Build the image (ros:humble-ros-base + pinocchio + cyclonedds) and the workspace.
# GDK_APP_DIR defaults to ./agibot_gdk/app; MODELS_DIR is mounted at /models for URDFs.
docker/run.sh colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Shell in the container (ROS, GDK env.sh and the workspace are already sourced)
MODELS_DIR=~/agibot_g2_torque_control/models/urdf docker/run.sh
```

To build outside this container, use any Humble environment with `ros-humble-pinocchio`, pass
`-DGDK_APP_DIR=<gdk>/app` (or set `GDK_APP_DIR`), and source the GDK's `env.sh` before running.
Without a GDK the package still builds, but only `backend:=fake` works. With Pinocchio from pip
(`pin`, as in the `g2-torque` image of `~/agibot_g2_torque_control`), add
`-DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix` and put that prefix's
`lib/` on `LD_LIBRARY_PATH`. Both smoke tests pass that way with `pin` 2.7.0.

## Run

Do a dry run first. It reads states, computes torques and publishes everything, but never
switches mode or sends torque. Check the pre-flight table: `diff` should be small with the arms at
rest.

```bash
ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true urdf_path:=/models/<g2>.urdf
```

Then run for real (C++, recommended for 1 kHz), or the Python version:

```bash
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=/models/<g2>.urdf
ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py urdf_path:=/models/<g2>.urdf
```

Send a command (Nm on top of gravity compensation; stop publishing and the arm floats after 100 ms):

```bash
ros2 topic pub -r 500 --qos-reliability best_effort /g2_torque_subscriber/left/external_torques \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 2.0, 0, 0, 0]}"
```

For real-time priority, the container already has `--ulimit rtprio=99 --cap-add SYS_NICE`; set
`enforce_realtime:=true` to fail rather than warn when SCHED_FIFO is unavailable. Python at 1 kHz
is best effort. If it struggles, raise `publish_decimation`.

## Offline rehearsal without the robot

The fakes simulate both arms (Pinocchio forward dynamics on the URDF) and emulate the mode-9
rules: no gravity compensation, the 50 Nm step e-stop, rejected frames, and joint-limit stops.

```bash
export G2_FAKE_URDF=/models/<g2>.urdf
# C++
ros2 launch g2_torque_subscriber torque_subscriber.launch.py backend:=fake urdf_path:=$G2_FAKE_URDF
# Python: put the fake agibot_gdk first on PYTHONPATH
PYTHONPATH=$(ros2 pkg prefix g2_torque_subscriber)/share/g2_torque_subscriber/fake_gdk:$PYTHONPATH \
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py urdf_path:=$G2_FAKE_URDF
```

## Tests (no robot needed, run inside the container)

```bash
# End-to-end against the simulated arms. Checks float under gravity comp, command added on top
# of g(q), timeout, rejected messages, velocity abort and SIGINT, each with mode restore. The fake
# plant uses the same URDF as the node, so this tests the plumbing, not whether the URDF matches
# the robot. That is what the on-robot dry-run table is for.
python3 src/g2_torque_subscriber/test/smoke_test_fake.py --impl cpp --urdf /models/<g2>.urdf
python3 src/g2_torque_subscriber/test/smoke_test_fake.py --impl py  --urdf /models/<g2>.urdf
# Real GDK next to ROS 2 (rmw_fastrtps and rmw_cyclonedds, C++ and Python): no crash, ROS
# interface alive, clean exit on Ctrl+C while the GDK waits for a robot
bash src/g2_torque_subscriber/test/coload_check.sh /models/<g2>.urdf
```

## Layout

```
g2_torque_subscriber/
  src/torque_subscriber_node.cpp       C++ node (ROS interface, torque law, lifecycle)
  src/gravity_model.cpp                Pinocchio gravity of the 14 arm joints
  src/gdk_backend.cpp                  RobotBackend on the real GDK (dlopen'ed module)
  src/fake_backend.cpp                 RobotBackend with simulated arms (dlopen'ed module)
  g2_torque_subscriber/torque_subscriber_node.py   Python node (same interface)
  test/fake_gdk/agibot_gdk/            fake GDK Python module
  config/torque_subscriber.yaml        parameters (shared by both nodes)
  launch/torque_subscriber.launch.py   impl:=cpp|py, dry_run, backend, urdf_path
docker/                                Humble image, entrypoint, run.sh
joint_torque_demo.py                   vendor example the torque handling follows
```
