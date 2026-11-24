# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 Humble package (`g2_torque_subscriber`) for direct joint torque control of the AGIBOT G2's
14 arm joints through GDK control mode 9. It is the G2 counterpart of the franka_ros2
`TorqueSubscriberController`. The package is the `g2_torque_subscriber/` directory at the repo
root, and package paths below are relative to it. There are **two implementations of the same node**: C++
(`src/torque_subscriber_node.cpp`, the one to use at 1 kHz) and Python
(`g2_torque_subscriber/torque_subscriber_node.py`). They share the ROS interface, the parameter
file (`config/torque_subscriber.yaml`), the torque law, the lifecycle and the log messages that the
tests grep for. **A behavior change to one must be made to the other too**, and both must pass the
smoke test.

The top-level `joint_torque_demo.py` (vendor example the torque handling follows) and
`zero_torque_joint_check.py` (standalone GDK-only hand-guided joint mapping and effort check, no
ROS) are scripts, not part of the package.

## Build, run, test

Everything runs in the container started by `docker/run.sh`, which has two profiles (`G2_PROFILE`,
auto-detected):

- **pc (x86 dev PC).** ROS 2 Humble. The GDK (`GDK_APP_DIR`, default `./agibot_gdk/app`,
  gitignored) is mounted at `/opt/agibot/app`, because the x86 GDK's Python binding is cpython-310.
- **robot (the robot's own Jetson, `agi@10.42.1.101`, aarch64 Ubuntu 24.04).** This is where the
  vendor now says to run torque control.
  - ROS 2 Jazzy. The robot GDK needs glibc 2.38 and GLIBCXX_3.4.32, and its binding is
    cpython-312, so Humble cannot load it.
  - The GDK `/home/agi/app` is mounted at the same path, `/data` read-only and `/data/logs`
    writable.
  - The container always runs as root, because the robot's `/dev/shm` segments are root-only.
  - `--ulimit rtprio=15` and no `SYS_NICE`. The GDK loop is FIFO 10; the robot's control threads
    run at 20 and above.
  - `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
  - Run it with `sudo docker/run.sh`.

Both nodes must build and run on Humble and Jazzy. Jazzy ships Pinocchio 4.x.

In both profiles:
- The package is mounted at `/ws/src/g2_torque_subscriber`. `MODELS_DIR` (optional, other URDFs)
  is mounted at `/models`.
- The entrypoint sources `/opt/ros/$ROS_DISTRO`, then the GDK `env.sh` from `$G2_GDK_MOUNT`, then
  `/ws/install/setup.bash`. It also sets `COSINE_BUS_CONFIG_PATH` and
  `COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=1`, because the robot's `env.sh` does not.
- The build/install output lives on the host in `docker/ws` (`G2_WS_DIR`, gitignored). Never
  point a Jazzy build at a Humble workspace.
- `run.sh` builds the image (tag `g2-torque-subscriber:<distro>`) on first use. The entrypoint
  is baked into the image, so rebuild after editing `docker/`.

Never source the GDK's `gdk/scripts/ros_env.sh`: it points ROS at the robot's own DDS domain (4).

Never run anything on the robot that is not read-only without the user's confirmation. That
includes rsync, docker, sudo and launching the node. The on-robot steps are in the README section
"Run on the robot's computer".

```bash
docker/run.sh colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
docker/run.sh     # shell

# inside the container
ros2 launch g2_torque_subscriber torque_subscriber.launch.py backend:=fake compensate_gravity:=true
ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true   # on the robot, sends nothing

# tests (no robot), from /ws (the container WORKDIR). The smoke test is the main regression test: run it for both impls.
python3 src/g2_torque_subscriber/test/smoke_test_fake.py --impl cpp
python3 src/g2_torque_subscriber/test/smoke_test_fake.py --impl py
bash src/g2_torque_subscriber/test/coload_check.sh   # needs the real GDK mounted
```

There are no unit tests or linters. `smoke_test_fake.py` runs four scenarios (A: all terms on,
B: SIGINT, C: all terms off, D: refusals before mode 9) and checks the node's stdout for strings
such as `control mode restored to 3`, `velocity_limit`, `no torque command`, `exceeds the effort
cap` and `not connecting`, plus the exit codes. If you change those log lines or exit codes,
update the test. Scenario C takes `~/commanded_torques` (published only by the torque loop) as the
sign of mode 9, because the measured states are already published while the node waits. Each run
uses a private ROS namespace (`/g2_smoke_<pid>_<time>`) so its commands never reach a live node. Node exit codes: 0 is a clean stop, 1 is a
setup failure (connect, URDF, pre-flight, entering mode 9), 2 is a GDK torque-loop or read error
(or a failed mode restore), 3 is a safety abort. The smoke test cannot run a single scenario:
`--impl` is the only selector. It runs the *installed* binaries (`ros2 pkg prefix`), so rebuild
after a C++ change. Its fake plant uses the node's own URDF, so it tests the plumbing, not whether
the URDF matches the robot (the on-robot dry-run gravity table checks that).

Launch args (`impl:=cpp|py`, `backend`, `urdf_path`, `dry_run`, `compensate_*`) override the YAML.
An empty arg keeps the YAML value. The Python node uses whichever `agibot_gdk` is first on
`PYTHONPATH`, but refuses to connect unless it matches `backend` (`fake` needs the fake's
`IS_G2_FAKE` marker, `gdk` refuses it). Backend module paths are C++ only. A new parameter goes in the YAML and is declared in both
nodes. To make it launch-overridable, also add a `DeclareLaunchArgument(..., default_value="")`
and an entry in `STRING_ARGS`/`BOOL_ARGS` in `launch/torque_subscriber.launch.py`.

**Fake Python GDK.** `test/fake_gdk/agibot_gdk/` is the Python counterpart of
`fake_backend.cpp`, used by `--impl py` and by the README's Python offline run. It provides the
`gdk.*` and `robot.*` calls the Python node and the top-level scripts make, with the real
binding's return types (dicts from `get_joint_states` / `get_whole_body_status`, `TimeoutError`
from `wait_control_mode`). `CMakeLists.txt` installs it with `install(DIRECTORY test/fake_gdk
...)`, so the build fails without it. The `.gitignore` rule for the vendor SDK is anchored
(`/agibot_gdk/`) so that it does not hide this directory. It imports numpy and pinocchio inside
`Robot()`, because the node imports `agibot_gdk` under `RTLD_DEEPBIND`.

**Robot model (URDF).** The confirmed model is `urdf/genie_g2_crs_omnipicker.urdf` in the
package, installed to `share/g2_torque_subscriber/urdf/`. It was generated from the simulation
model `genie_sim_robot_model` with arm crs and the omnipicker grippers, which add 16 gripper
joints. It is the default URDF everywhere, so commands need no `urdf_path:=`:
- both nodes' `urdf_path` (via `ament_index`);
- the C++ fake (through `urdf_path`) and the Python fake (`G2_FAKE_URDF`);
- `smoke_test_fake.py --urdf` and `coload_check.sh`;
- the two top-level scripts (relative to the script).

The robot's reported `urdf_type` (`G2_t2_crsB`) is **not** the right model. It is used only when
`urdf_path` is explicitly `""`, and the node then warns. An empty launch argument does not count.
Do not reintroduce a different default. The URDF must live inside the package, because the
container only mounts the package directory.

## Architecture

**GDK isolation via dlopen.** This is the most important constraint. The GDK bundles Fast DDS 3,
boost 1.83 and protobuf. Linked directly into the node, it makes ROS's `rmw_fastrtps` (Fast DDS 2)
abort on the first `rclcpp::Node`. So the C++ node **never links the GDK**. It `dlopen`s a backend
module (`libg2_gdk_backend.so` or `libg2_fake_backend.so`) with `RTLD_LOCAL | RTLD_DEEPBIND` and
uses it only through the `RobotBackend` vtable in `include/g2_torque_subscriber/robot_backend.hpp`,
via the `extern "C"` factory `g2_create_robot_backend`. Keep that header free of GDK and ROS
types, and of function-local statics, because `RTLD_DEEPBIND` gives each module its own copy of
inline functions. The Python node imports `agibot_gdk` with the same dlopen
flags. CMake builds `g2_gdk_backend` only if it finds the GDK (`-DGDK_APP_DIR` or env
`GDK_APP_DIR`). Without it, only `backend:=fake` works.

**Backends.** `gdk_backend.cpp` wraps the real GDK. `fake_backend.cpp` simulates both arms with
Pinocchio forward dynamics on the same URDF, and emulates the mode-9 rules: no gravity
compensation, a 50 Nm per-frame step e-stop, rejected frames and joint-limit stops. It is
built from the node's `urdf_path` and configured by the env vars `G2_FAKE_INITIAL_POSE` and
`G2_FAKE_FRICTION`. The fake Python GDK reads its URDF from `G2_FAKE_URDF`, which defaults to
the packaged model.
`RunTorqueLoop` owns the 1 kHz read, callback, write cycle. It never changes the control mode
itself.

**Dynamics.** `DynamicsModel` (`dynamics_model.cpp`, a static library) computes g(q), C(q,dq)dq
and M(q)ddq for the 14 arm joints. The waist and head joints take their measured q and dq from
each frame, so the waist pose affects the arm gravity. `Bind()` runs on every frame. It compares
the frame's full name list (not only the arm joints, so a waist/head reorder is caught) and
rebuilds the joint-order index, which allocates, only when the layout changes. It is otherwise
RT-safe.

**Conventions.** Every 14-vector uses `kArmJointNames` order: indices 0..6 are the left arm
`idx21..idx27_arm_l_joint1..7`, and 7..13 are the right arm `idx61..idx67`. Per-arm topics carry
7 values.

**C++ node threads.** The torque loop runs on its own thread (the GDK callback). The ROS executor
runs on the main thread, and commands cross under a mutex. A third thread publishes the joint
states, so ROS never blocks the loop. In Python, the torque loop runs on the main thread, rclpy
spins on a background thread, and a third thread publishes the states the loop hands over without
waiting (stopped and joined before the node is destroyed).

**Torque law and safety** (the README has the full lifecycle). The node adds whichever of g, C dq
and M ddq_d are enabled (all off by default) to the command, subtracts damping, clamps to the URDF
effort limit (cap = `effort_limit_scale` × URDF limit), then slew-limits by `max_torque_step`
starting from the measured effort, then clamps to the cap again. The pre-flight refuses a
non-finite state and a holding torque above the cap, so every torque stays inside the cap. The
semantics depend on `compensate_gravity`:
- **On:** a timed-out command becomes 0, and the arm floats.
- **Off:** commands must carry the full torque. The node enters mode 9 only once both arms are
  commanded close to the measured holding torque, and a timed-out command **ends the loop**,
  because zero total torque drops a G2 arm. While it waits, it publishes the measured states.

Every exit path (abort, error, Ctrl+C, SIGTERM, `docker stop`) must send a final frame and
**restore the previous control mode**. The node refuses to start if the robot is already in mode
9, and it refuses mode 9 if the mode changed between the first read and entering. Mode 9 and the
restored mode each count as reached only after `WaitControlMode` plus a status read confirm them.
`control mode restored to <m>` is logged only then; otherwise the node exits 2. This follows the
vendor's on-robot `joint_torque.py`. Preserve these invariants when editing the lifecycle code. The GDK's own rules for mode 9
(per-frame slew, the 20 ms watchdog, the e-stop) are documented in
`agibot_gdk/app/gdk/site/contents/howto/direct_torque/`.
