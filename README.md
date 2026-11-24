# agibot_g2_torque_control_ros2_node

The node can run in two places:

- **On the robot's own computer** (`agi@10.42.1.101`). This is the vendor's current instruction
  and the only setup tested on the robot so far. See
  [Run on the robot's computer](#run-on-the-robots-computer).
- **On a dev PC wired to the robot as 10.42.1.102.** The rest of this README describes this
  setup. With the robot's current GDK config, the robot sends its torque-mode state to itself
  (10.42.1.101:8852), so a dev PC probably receives nothing.

## Run on the robot's computer

The robot's computer is a Jetson Thor running Ubuntu 24.04 (aarch64, Python 3.12). The GDK is
installed at `/home/agi/app`. `docker/run.sh` detects the robot and switches to its robot
profile:

| | Robot profile |
|---|---|
| Image | ROS 2 **Jazzy**, arm64 (`g2-torque-subscriber:jazzy`, built on first use, needs internet). The robot's GDK needs glibc 2.38 and `GLIBCXX_3.4.32`, so Humble (Ubuntu 22.04) cannot load it. |
| GDK | `/home/agi/app`, mounted read-only at the same path. The vendor's paths (URDF, `cosine_ipc.prototxt`) therefore work unchanged in the container. The entrypoint sets `COSINE_BUS_CONFIG_PATH` and `COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=1`, as in the vendor's instructions. |
| State stream | UDP to `10.42.1.101:8852`, on the robot itself. |
| User | The container always runs as root, like the vendor demo (`sudo su`), because the robot services' `/dev/shm` segments are root-only. `agi` is not in the `docker` group, so use `sudo docker/run.sh`. `docker/ws` ends up owned by root. |
| Other mounts | `/data` read-only (the GDK reads `/data/version` and `/data/parameters`) and `/data/logs` writable (the GDK's `LOG_PATH`). |
| CPUs | Docker uses the systemd cgroup driver, so containers inherit the robot's `system.slice` cpuset (CPUs 11-13). These are the same CPUs as the vendor demo run from ssh. The robot's isolated real-time CPUs stay untouched. The kernel is PREEMPT_RT with RT throttling off (`sched_rt_runtime_us=-1`), one more reason for the low priority cap. |
| Real time | `--ulimit rtprio=15`, no `CAP_SYS_NICE`. The GDK runs the torque loop at SCHED_FIFO 10. The robot's own control threads run at 20 and above (EtherCAT 20, `servo_server` 52, `hal-loop` 99), and nothing in the container can outrank them. The Python node's request for priority 80 falls back to normal priority with a warning. |
| ROS | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so DDS stays off the robot's internal networks. Run the command publisher on the robot too, in a second `sudo docker/run.sh`. Keep `ROS_DOMAIN_ID` away from 4, the domain of the robot's own DDS. The GDK's `run.conf` sets `FASTDDS_DEFAULT_PROFILES_FILE` (domain 4, robot interfaces only). Only the GDK's Fast DDS 3 reads it; ROS's Fast DDS 2.14 reads `FASTRTPS_DEFAULT_PROFILES_FILE`. |

Both implementations should work here, because the robot's GDK binding is CPython 3.12, which
matches Jazzy. They have been checked on Jazzy only on the PC, with the x86 GDK and no robot:
- built against Pinocchio 4.1;
- smoke tests pass for both implementations;
- the C++ co-load check passes with `rmw_fastrtps` and with `rmw_cyclonedds`.

The Python co-load check cannot run there, because the x86 binding is CPython 3.10. The first
on-robot build is the first aarch64 build.

```bash
# On the PC: copy the repo to the robot. This excludes the x86 GDK, the zip and the PC build.
rsync -a --delete --exclude .git --exclude /agibot_gdk --exclude /agibot.zip --exclude /docker/ws \
  ./ agi@10.42.1.101:g2_torque/

# On the robot (ssh agi@10.42.1.101)
cd ~/g2_torque
ls /home/agi/app/gdk/build_dep/cpp/aarch64/include       # GDK headers: expect common/ and gdk/
sudo docker/run.sh colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#   the CMake output must say "GDK: /home/agi/app (aarch64)"

U=/home/agi/app/share/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf

# Read-only checks. Neither one switches the control mode or sends torque.
sudo docker/run.sh bash -c 'echo $LOCATOR_IP $AORTA_DISCOVERY_URI $COSINE_BUS_CONFIG_PATH'
#   expect: 10.42.1.101 http://10.42.1.101:2379 /home/agi/app/conf/dds/cosine_ipc.prototxt
sudo docker/run.sh bash src/g2_torque_subscriber/test/coload_check.sh $U    # GDK next to ROS 2
sudo docker/run.sh ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true urdf_path:=$U
#   with the arms at rest, the gravity table's diff column must be small

# Real run: the arms move. Clear the area and keep the e-stop in reach.
sudo docker/run.sh ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=$U \
  compensate_gravity:=true
# Second ssh session: commands, as in Quick start
sudo docker/run.sh ros2 topic pub -r 500 --qos-reliability best_effort \
  /g2_torque_subscriber/left/external_torques std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0, 0]}"
```

- **The robot build fails on missing GDK headers:** copy
  `agibot_gdk/app/gdk/build_dep/cpp/aarch64/include` from the PC. It is the same GDK version
  (`snap-direct-torque-mode-1102-1`).
- **The first real run logs `JointTorqueControl returned kTimeout`:** the vendor's on-robot script
  uses `state_timeout_ms=500` rather than the default 100. Set it in
  `config/torque_subscriber.yaml`. With `--symlink-install`, no rebuild is needed.
- **Watch the state stream during a real run:** run `sudo tcpdump -ni any udp port 8852 -c 20` on
  the robot.
- **To skip sudo:** `sudo usermod -aG docker agi`, then log in again. The container still runs as
  root. Membership in the `docker` group is root-equivalent anyway.

## Check the robot connection (dev PC)

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

# inside the container. Add dry_run:=true to any step first: it computes and publishes, but never
# switches mode or sends torque. Add impl:=py for the Python node.
U=/models/<g2>.urdf

# 0. No compensation (default): your commands carry the full torque, gravity included
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=$U

# 1. + gravity g(q)
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=$U \
  compensate_gravity:=true

# 2. + Coriolis / centrifugal C(q, dq) dq
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=$U \
  compensate_gravity:=true compensate_coriolis:=true

# 3. + inertia feed-forward M(q) ddq_d
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=$U \
  compensate_gravity:=true compensate_coriolis:=true compensate_inertia:=true

# second shell (docker/run.sh), steps 1-3: command the left arm (Nm, on top of the enabled terms)
ros2 topic pub -r 500 --qos-reliability best_effort /g2_torque_subscriber/left/external_torques \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 2.0, 0, 0, 0]}"
# step 3: desired accelerations of the left arm (rad/s^2)
ros2 topic pub -r 500 --qos-reliability best_effort /g2_torque_subscriber/left/desired_accelerations \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0.5, 0, 0, 0]}"
# what the enabled terms add (Nm)
ros2 topic echo /g2_torque_subscriber/compensation_torques --field effort
```

Direct joint torque control of the **AGIBOT G2** arms from ROS 2 topics: the G2 counterpart of the
`TorqueSubscriberController` in the franka_ros2 fork (`~/franka_ros2`,
`franka_example_controllers/src/torque_subscriber_controller.cpp`). There are two implementations
with the same interface: C++ (`torque_subscriber_node`) and Python (`torque_subscriber_node.py`).
Both run in a ROS 2 container: Humble on the dev PC, Jazzy on the robot's computer.

| | Franka `TorqueSubscriberController` | G2 `g2_torque_subscriber` |
|---|---|---|
| Command | `~/external_torques`, `Float64MultiArray`, 7 values | `~/left/external_torques` and `~/right/external_torques`, `Float64MultiArray`, 7 values each (joint1..7) |
| QoS | KeepLast(1), BestEffort, Volatile | same |
| Meaning | torque on top of the robot's gravity compensation | torque on top of the model terms the **node** adds (Pinocchio + URDF, the terms of `joint_torque_demo.py`). Each term has a flag and all are **off by default**, so by default a command is the full torque. G2 mode 9 compensates nothing. |
| No message for 100 ms | zero torque, so the arm floats | `compensate_gravity` on: that arm's command goes to 0 and the model terms continue, so the arm floats. Off: the node ends the torque loop and restores the previous mode, because zero *total* torque would drop a G2 arm. |
| Runs as | ros2_control controller | standalone node that owns the GDK connection (G2 has no ros2_control hardware interface) |
| Joint states | joint_state_broadcaster | published by the node from the 1 kHz robot frames |

## Torque law (every 1 ms frame, 14 arm joints)

```
tau = [compensate_gravity]  g(q)                                  gravity
    + [compensate_coriolis] C(q, dq) dq                           Coriolis + centrifugal
    + [compensate_inertia]  M(q) ddq_d                            inertia feed-forward
    + tau_ext - joint_damping * dq
tau = clamp(tau, +- cap)                                          cap = effort_limit_scale * URDF effort limit
tau = tau_prev + clamp(tau - tau_prev, +- max_torque_step)       tau_prev of frame 0 = measured effort (robot's holding torque)
tau = clamp(tau, +- cap)                                          no-op: the holding torque must be inside the cap (pre-flight)
```

The three terms are those of the vendor demo's `rnea(q, dq, ddq_d) = M ddq_d + C dq + g`:

- **Waist and head:** these joints take their measured position, as in the demo. With
  `compensate_coriolis`, their measured velocities also contribute, so waist motion feeds into the
  arm torques.
- **Desired acceleration:** `ddq_d` is the latest message on `~/<arm>/desired_accelerations`. The
  demo plans its own path; the node has none, so your controller sends it.

`tau_ext` and `ddq_d` are the most recent inputs of each arm, or 0 if older than `command_timeout`.
The robot then shapes the torque further (about 2 Nm per frame slew, a 100 Hz low-pass filter, and
a clamp at the limit). See the GDK docs, `agibot_gdk/app/gdk/site/contents/howto/direct_torque/`.

## ROS interface

Default node name `g2_torque_subscriber` (`~` = `/<namespace>/g2_torque_subscriber`).

| Topic | Type | |
|---|---|---|
| `~/left/external_torques`, `~/right/external_torques` | `std_msgs/Float64MultiArray` (sub) | 7 values, Nm, joint1..7. Wrong length or NaN/Inf is rejected. |
| `~/left/desired_accelerations`, `~/right/desired_accelerations` | `std_msgs/Float64MultiArray` (sub) | 7 values, rad/s², joint1..7. Used by `compensate_inertia`. |
| `joint_states` | `sensor_msgs/JointState` (pub) | all joints the robot reports (arms, waist, head) |
| `~/left/joint_states`, `~/right/joint_states` | `sensor_msgs/JointState` (pub) | the 7 joints of that arm: measured q, dq, effort |
| `~/commanded_torques` | `sensor_msgs/JointState` (pub) | 14 arm joints, `effort` = total torque sent |
| `~/compensation_torques` | `sensor_msgs/JointState` (pub) | 14 arm joints, `effort` = sum of the enabled model terms |

Arm joints: left `idx21_arm_l_joint1` … `idx27_arm_l_joint7`, right `idx61_arm_r_joint1` …
`idx67_arm_r_joint7`. The publishers are reliable and publish every `publish_decimation`-th frame
(default: every frame, 1 kHz) from a separate thread, so a publish never delays the torque write.
While the node waits for commands (`compensate_gravity` off), the measured states (`joint_states`
and `~/<arm>/joint_states`) are already published at about 100 Hz, so a controller can compute its
first full-torque command from them. `~/commanded_torques` and `~/compensation_torques` start with
the torque loop. All parameters, with comments, are in
[`g2_torque_subscriber/config/torque_subscriber.yaml`](g2_torque_subscriber/config/torque_subscriber.yaml).

## Lifecycle and safety

1. Connect (GDK), load the URDF, and read one state frame.
2. Pre-flight: the node checks that all 14 arm joints are present and prints model gravity g(q)
   against the measured effort. It refuses to start (exit 1) if:
   - an arm joint's position, velocity or effort is not finite;
   - the measured holding torque of a joint is above the effort cap (`effort_limit_scale` × URDF
     limit). The node could not hold that arm, and its first torque would exceed the cap.
   - with `compensate_gravity` on, g(q) and the effort differ by more than
     `gravity_check_tolerance` (10 Nm). A wrong URDF, an unregistered payload or a wrong waist pose
     would otherwise cause drift or the 50 Nm step e-stop.
3. Remember the control mode. The node refuses to start if the robot is already in mode 9.
4. **With `compensate_gravity` off only:** wait until both arms are commanded and the first torque
   is within `gravity_check_tolerance` of the measured holding torque. Commands that leave out
   gravity therefore never reach the robot. The measured states are published meanwhile.
5. Read the mode again. If it is no longer the remembered one (someone else is operating the
   robot), refuse. Otherwise switch to mode 9 and wait until the robot reports it. **Do not touch
   the arms while entering.**
6. 1 kHz loop. Abort (final frame, then step 7) on any of these:
   - a joint faster than `velocity_limit` (2 rad/s);
   - a non-finite state or torque;
   - a GDK read timeout or error;
   - with `compensate_gravity` off, an arm without a command for `command_timeout`.
7. **Restore the previous control mode** (the robot then holds the arms) and exit. Ctrl+C, SIGTERM,
   `docker stop`, aborts and errors all take this path. GDK never leaves mode 9 by itself. The node
   logs `control mode restored to <mode>` only once the robot reports that mode (500 ms wait).
   Otherwise it logs an error and exits 2: check the mode, and clear an e-stop with
   `ClearMotionControlError()`.

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

## Setup (dev PC, ROS 2 Humble container)

For the robot's own computer, see [Run on the robot's computer](#run-on-the-robots-computer).
`docker/run.sh` picks the right profile by itself. On the PC:
- The x86 GDK's Python binding is CPython 3.10, hence Humble.
- The GDK docs have the robot stream its state to `10.42.1.102:8852/UDP`. The machine therefore
  needs a direct 10 GbE link to the robot's debug port with IP 10.42.1.102.
- The container uses host networking.

After pulling changes to `docker/`, rebuild the image with `docker rmi g2-torque-subscriber:humble`;
`run.sh` then rebuilds it on the next start.

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

Then run for real (C++, recommended for 1 kHz), or the Python version. With no `compensate_*`
flags the node adds nothing and waits for full-torque commands on both arms. See Quick start for
enabling the terms one at a time.

```bash
ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=/models/<g2>.urdf
ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py urdf_path:=/models/<g2>.urdf
```

Send a command. With `compensate_gravity:=true` it is Nm on top of the model terms, and if you stop
publishing the arm floats after 100 ms. With gravity off it must be the full torque.

```bash
ros2 topic pub -r 500 --qos-reliability best_effort /g2_torque_subscriber/left/external_torques \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 2.0, 0, 0, 0]}"
```

For real-time priority, the PC container already has `--ulimit rtprio=99 --cap-add SYS_NICE`.
The robot profile deliberately caps it at 15; see
[Run on the robot's computer](#run-on-the-robots-computer). Set
`enforce_realtime:=true` to fail rather than warn when SCHED_FIFO is unavailable. Python at 1 kHz
is best effort. If it struggles, raise `publish_decimation`.

## Offline rehearsal without the robot

The fakes simulate both arms (Pinocchio forward dynamics on the URDF) and emulate the mode-9
rules: no gravity compensation, the 50 Nm step e-stop, rejected frames, and joint-limit stops.

```bash
export G2_FAKE_URDF=/models/<g2>.urdf
# C++
ros2 launch g2_torque_subscriber torque_subscriber.launch.py backend:=fake urdf_path:=$G2_FAKE_URDF \
  compensate_gravity:=true
# Python: put the fake agibot_gdk first on PYTHONPATH AND pass backend:=fake. The Python node
# refuses to connect when the imported agibot_gdk is not the one backend names, so a mistyped
# PYTHONPATH can never turn a rehearsal into a run on the real robot.
PYTHONPATH=$(ros2 pkg prefix g2_torque_subscriber)/share/g2_torque_subscriber/fake_gdk:$PYTHONPATH \
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py backend:=fake \
  urdf_path:=$G2_FAKE_URDF compensate_gravity:=true
```

## Tests (no robot needed, run inside the container)

```bash
# End-to-end against the simulated arms, 25 checks (26 with --impl py). The nodes run in a private
# namespace (/g2_smoke_<pid>_<time>), so the test commands never reach a live node.
#  - all three terms on: each term against an independent Pinocchio model, timeout, rejected
#    messages, velocity abort and SIGINT, each with mode restore;
#  - all off: measured states while waiting; no mode 9 without commands or with commands that
#    leave out gravity; mode 9 with full-torque commands; abort and mode restore when they stop;
#  - refusals before mode 9: holding torque above the effort cap, and (Python) backend:=gdk with
#    the fake agibot_gdk.
# The fake plant uses the same URDF as the node, so this tests the plumbing, not whether the URDF
# matches the robot. That is what the on-robot dry-run table is for.
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
  src/dynamics_model.cpp               Pinocchio g(q), C(q,dq)dq, M(q)ddq of the 14 arm joints
  src/gdk_backend.cpp                  RobotBackend on the real GDK (dlopen'ed module)
  src/fake_backend.cpp                 RobotBackend with simulated arms (dlopen'ed module)
  g2_torque_subscriber/torque_subscriber_node.py   Python node (same interface)
  test/fake_gdk/agibot_gdk/            fake GDK Python module
  config/torque_subscriber.yaml        parameters (shared by both nodes)
  launch/torque_subscriber.launch.py   impl:=cpp|py, dry_run, backend, urdf_path, compensate_*
docker/                                image (Humble on the PC, Jazzy on the robot), entrypoint, run.sh
joint_torque_demo.py                   vendor example the torque handling follows
```
