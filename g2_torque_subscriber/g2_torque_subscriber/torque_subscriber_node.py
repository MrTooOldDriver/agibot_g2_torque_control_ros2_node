"""G2 torque subscriber, Python (rclpy + agibot_gdk) version of torque_subscriber_node.cpp.

Same ROS interface, parameters and torque law as the C++ node; see the header of
src/torque_subscriber_node.cpp. Each arm listens on its own topic (std_msgs/Float64MultiArray,
7 values, joint1..joint7). A command is the torque added ON TOP OF GRAVITY COMPENSATION (the meaning
it has on a Franka):

    tau = g(q) + tau_ext - joint_damping * dq
    tau = clamp(tau, +-effort_limit_scale * URDF effort limit)
    tau = tau_prev + clamp(tau - tau_prev, +-max_torque_step)     (tau_prev of frame 0 = measured effort)

A timed-out arm keeps gravity compensation (tau_ext = 0), it does NOT get zero torque: in G2 mode 9
zero total torque drops the arm.

Threads: the GDK torque loop (agibot_gdk Robot.joint_torque_control: read -> callback -> write,
1 kHz best effort in Python) runs on the main thread; the rclpy executor spins on a background
thread. The C++ node is the one to use when timing matters.

Needs Python 3.10 (the GDK binding is cpython-310), i.e. the ROS 2 Humble container.
"""

import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS_PER_ARM = 7
ARM_JOINT_NAMES = [  # left joint1..7, then right joint1..7 (every 14-vector uses this order)
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3", "idx24_arm_l_joint4",
    "idx25_arm_l_joint5", "idx26_arm_l_joint6", "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3", "idx64_arm_r_joint4",
    "idx65_arm_r_joint5", "idx66_arm_r_joint6", "idx67_arm_r_joint7",
]
ARM_DOF = len(ARM_JOINT_NAMES)
ARM_LABELS = ("left", "right")
MODE_JOINT_TORQUE = 9


def import_gdk():
    """Import agibot_gdk with RTLD_LOCAL | RTLD_DEEPBIND.

    The GDK bundles Fast DDS 3, boost 1.83, protobuf, ...; loaded this way they stay out of the
    global symbol scope that ROS uses, and the GDK prefers its own copies (see robot_backend.hpp).
    """
    flags = sys.getdlopenflags()
    sys.setdlopenflags(os.RTLD_NOW | os.RTLD_LOCAL | getattr(os, "RTLD_DEEPBIND", 0))
    try:
        import agibot_gdk  # noqa: PLC0415
    finally:
        sys.setdlopenflags(flags)
    return agibot_gdk


class GravityModel:
    """Gravity torques of the 14 arm joints from a URDF (Pinocchio).

    Non-arm joints (waist, head) take their measured position from the frame.
    """

    def __init__(self, urdf_path):
        import pinocchio as pin  # noqa: PLC0415

        self._pin = pin
        self.urdf_path = urdf_path
        self.model = pin.buildModelFromUrdf(urdf_path)  # fixed base
        self.data = self.model.createData()
        self._q = pin.neutral(self.model)
        self._arm_q, self._arm_v, limits = [], [], []
        for name in ARM_JOINT_NAMES:
            if not self.model.existJointName(name):
                raise RuntimeError(f"URDF has no joint {name}")
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise RuntimeError(f"URDF joint {name} is not 1-DoF")
            self._arm_q.append(joint.idx_q)
            self._arm_v.append(joint.idx_v)
            limits.append(max(1.0, float(self.model.effortLimit[joint.idx_v])))
        self.effort_limits = np.array(limits)
        self._others = []  # (name, idx_q) of the non-arm 1-DoF joints
        for jid in range(1, self.model.njoints):
            joint, name = self.model.joints[jid], self.model.names[jid]
            if joint.nq == 1 and joint.nv == 1 and name not in ARM_JOINT_NAMES:
                self._others.append((name, joint.idx_q))
        self._layout = None
        self._arm_idx = None
        self._other_idx = None  # (frame index array, idx_q array)

    def bind(self, names):
        """Locate the joints in a frame layout. False if an arm joint is missing."""
        if names == self._layout:
            return True
        index = {n: i for i, n in enumerate(names)}
        if any(n not in index for n in ARM_JOINT_NAMES):
            self._layout = None
            return False
        self._arm_idx = np.array([index[n] for n in ARM_JOINT_NAMES])
        pairs = [(index[n], iq) for n, iq in self._others if n in index]
        self._other_idx = (np.array([p[0] for p in pairs], dtype=int),
                           np.array([p[1] for p in pairs], dtype=int))
        self._layout = list(names)
        return True

    def arm(self, values):
        """The 14 arm entries of a whole-body array, ARM_JOINT_NAMES order."""
        return values[self._arm_idx]

    def gravity(self, position):
        """Gravity torques of the 14 arm joints at the whole-body position array of a frame."""
        frame_idx, q_idx = self._other_idx
        self._q[q_idx] = position[frame_idx]
        self._q[self._arm_q] = position[self._arm_idx]
        g = self._pin.computeGeneralizedGravity(self.model, self.data, self._q)
        return np.asarray(g)[self._arm_v].copy()


def frame_arrays(state):
    """(names, position, velocity, effort) of a GDK state dict."""
    states = state["states"]
    n = len(states)
    names = [s["name"] for s in states]
    pos = np.fromiter((s["position"] for s in states), float, n)
    vel = np.fromiter((s["velocity"] for s in states), float, n)
    eff = np.fromiter((s["effort"] for s in states), float, n)
    return names, pos, vel, eff


class Abort(Exception):
    """Safety abort raised inside the torque law."""


class TorqueSubscriberNode(Node):
    def __init__(self, stop_event):
        super().__init__("g2_torque_subscriber")
        self.stop = stop_event
        self.phase = "idle"
        self._declare_parameters()

        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(ARM_DOF)
        self._cmd_stamp = [None, None]  # time.monotonic() of the last valid command per arm
        self._was_fresh = [False, False]

        cmd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self._subs = [
            self.create_subscription(Float64MultiArray, self.p["torque_topics"][arm],
                                     lambda msg, arm=arm: self._on_arm_command(arm, msg), cmd_qos)
            for arm in range(2)
        ]
        state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)

        def pub(topic):
            return self.create_publisher(JointState, topic, state_qos) if topic else None

        self._joint_states_pub = pub(self.p["joint_states_topic"])
        self._arm_state_pubs = [pub(t) for t in self.p["arm_state_topics"]]
        self._commanded_pub = pub(self.p["commanded_torques_topic"])
        self._arm_msgs = []
        for arm in range(2):
            m = JointState()
            m.name = ARM_JOINT_NAMES[arm * JOINTS_PER_ARM:(arm + 1) * JOINTS_PER_ARM]
            self._arm_msgs.append(m)
        self._full_msg = JointState()
        self._cmd_msg = JointState()
        self._cmd_msg.name = list(ARM_JOINT_NAMES)
        self.create_timer(0.1, self._status_tick)

        self.model = None
        self.frames = 0
        self._prev_tau = None
        self._first_frame = True
        self.abort_reason = None
        self.get_logger().info(
            f"G2 torque subscriber (Python): left '{self._subs[0].topic_name}', right "
            f"'{self._subs[1].topic_name}' (7 values each, Nm on top of gravity compensation; "
            f"QoS KeepLast(1)/BestEffort/Volatile), timeout {self.p['command_timeout']:.3f} s"
            + (", DRY RUN (nothing is sent to the robot)" if self.p["dry_run"] else ""))

    # ------------------------------------------------------------------------------------------
    def _declare_parameters(self):
        d = self.declare_parameter
        p = {
            "urdf_path": d("urdf_path", "").value,
            "urdf_dir": d("urdf_dir", "/home/agi/app/share/genie_robot_description/urdf").value,
            "torque_topics": [d("left_torque_topic", "~/left/external_torques").value,
                              d("right_torque_topic", "~/right/external_torques").value],
            "arm_state_topics": [d("left_joint_states_topic", "~/left/joint_states").value,
                                 d("right_joint_states_topic", "~/right/joint_states").value],
            "joint_states_topic": d("joint_states_topic", "joint_states").value,
            "commanded_torques_topic": d("commanded_torques_topic", "~/commanded_torques").value,
            "publish_decimation": max(1, int(d("publish_decimation", 1).value)),
            "command_timeout": float(d("command_timeout", 0.1).value),
            "max_torque_step": float(d("max_torque_step", 15.0).value),
            "effort_limit_scale": float(d("effort_limit_scale", 1.0).value),
            "joint_damping": float(d("joint_damping", 0.0).value),
            "velocity_limit": float(d("velocity_limit", 2.0).value),
            "gravity_check_tolerance": float(d("gravity_check_tolerance", 10.0).value),
            "dry_run": bool(d("dry_run", False).value),
            "dry_run_rate": float(d("dry_run_rate", 100.0).value),
            "enforce_realtime": bool(d("enforce_realtime", False).value),
            "state_timeout_ms": int(d("state_timeout_ms", 100).value),
        }
        # Accepted for a shared parameter file with the C++ node; the Python node always uses the
        # agibot_gdk module found on PYTHONPATH (test/fake_gdk for offline tests).
        d("backend", "gdk")
        if not 0.0 < p["max_torque_step"] < 50.0:
            raise ValueError("max_torque_step must be in (0, 50) Nm (robot e-stops at 50)")
        if not 0.0 < p["effort_limit_scale"] <= 1.0:
            raise ValueError("effort_limit_scale must be in (0, 1]")
        if p["command_timeout"] <= 0.0 or p["dry_run_rate"] <= 0.0:
            raise ValueError("command_timeout and dry_run_rate must be > 0")
        self.p = p

    # ------------------------------------------------------------------------------------------
    # Command subscription (executor thread)
    # ------------------------------------------------------------------------------------------
    def _on_arm_command(self, arm, msg):
        data = msg.data
        if len(data) != JOINTS_PER_ARM:
            self.get_logger().error(
                f"{ARM_LABELS[arm]} arm: torque command with {len(data)} elements, expected "
                f"{JOINTS_PER_ARM}; ignored", throttle_duration_sec=1.0)
            return
        values = np.asarray(data, dtype=float)
        if not np.all(np.isfinite(values)):
            self.get_logger().error(f"{ARM_LABELS[arm]} arm: torque command contains NaN/Inf; "
                                    "ignored", throttle_duration_sec=1.0)
            return
        with self._cmd_lock:
            self._cmd[arm * JOINTS_PER_ARM:(arm + 1) * JOINTS_PER_ARM] = values
            self._cmd_stamp[arm] = time.monotonic()

    def _external_torque(self):
        ext = np.zeros(ARM_DOF)
        now = time.monotonic()
        with self._cmd_lock:
            for arm in range(2):
                stamp = self._cmd_stamp[arm]
                if stamp is not None and now - stamp < self.p["command_timeout"]:
                    sl = slice(arm * JOINTS_PER_ARM, (arm + 1) * JOINTS_PER_ARM)
                    ext[sl] = self._cmd[sl]
        return ext

    def _status_tick(self):
        now = time.monotonic()
        with self._cmd_lock:
            fresh = [s is not None and now - s < self.p["command_timeout"] for s in self._cmd_stamp]
        for arm in range(2):
            if fresh[arm] != self._was_fresh[arm]:
                if fresh[arm]:
                    self.get_logger().info(f"{ARM_LABELS[arm]} arm: receiving torque commands")
                else:
                    self.get_logger().warn(
                        f"{ARM_LABELS[arm]} arm: no torque command for "
                        f"{self.p['command_timeout']:.3f} s -> external torque 0 (gravity "
                        "compensation only)")
                self._was_fresh[arm] = fresh[arm]

    # ------------------------------------------------------------------------------------------
    # Control (main thread)
    # ------------------------------------------------------------------------------------------
    def run_control(self, gdk):
        log = self.get_logger()
        self.phase = "connecting"
        robot = self._connect(gdk)
        if robot is None:
            return 1
        try:
            self.phase = "preparing"
            return self._prepare_and_run(gdk, robot)
        finally:
            del robot
            try:
                gdk.gdk_release()
            except Exception as e:  # noqa: BLE001
                log.warn(f"gdk_release failed: {e}")

    def _connect(self, gdk):
        """gdk_init() + Robot() on a helper thread, so Ctrl+C works while no robot answers."""
        result = {}

        def work():
            try:
                gdk.gdk_init()
                result["robot"] = gdk.Robot()
            except Exception as e:  # noqa: BLE001
                result["error"] = e

        self.get_logger().info("connecting to the robot (GDK)")
        t = threading.Thread(target=work, daemon=True)
        t.start()
        while t.is_alive():
            t.join(0.2)
            if self.stop.is_set() and t.is_alive():
                # Still inside gdk_init / Robot(): no mode was changed yet, nothing to clean up.
                self.get_logger().error("GDK still connecting (robot not reachable?); exiting")
                sys.stdout.flush()
                os._exit(1)
        if "error" in result:
            self.get_logger().fatal(f"GDK connect failed: {result['error']}")
            return None
        return result["robot"]

    def _prepare_and_run(self, gdk, robot):
        log = self.get_logger()
        # ---- Dynamics model ----
        urdf = self.p["urdf_path"]
        if not urdf:
            urdf_type = getattr(robot.get_whole_body_status(), "urdf_type", "")
            if not urdf_type:
                log.fatal("robot did not report its urdf_type; set parameter urdf_path")
                return 1
            urdf = f"{self.p['urdf_dir']}/{urdf_type}/{urdf_type}.urdf"
        try:
            self.model = GravityModel(urdf)
        except Exception as e:  # noqa: BLE001
            log.fatal(f"cannot load URDF '{urdf}': {e}")
            return 1
        log.info(f"gravity model: {urdf}")

        # ---- Pre-flight ----
        names, pos, vel, eff = frame_arrays(robot.get_joint_states())
        if not self.model.bind(names):
            log.fatal("the robot state misses one of the 14 arm joints")
            return 1
        if not self._gravity_check(pos, vel, eff):
            return 1
        if self.p["dry_run"]:
            return self._run_dry(robot)

        # ---- Remember the control mode, then enter mode 9 ----
        old_mode = int(robot.get_motion_control_status().control_mode)
        if old_mode == MODE_JOINT_TORQUE:
            log.fatal("robot is already in mode 9 (a previous run did not exit cleanly?); cannot "
                      "tell which mode to restore. Switch it to mode 3 first.")
            return 1
        if self.stop.is_set():
            return 0
        self._set_realtime()
        log.info(f"control mode {old_mode} -> 9 (direct torque). Do not touch the arms.")
        rc = 0
        # From the first set_control_mode(9) on, every exit goes through the restore in `finally`.
        try:
            try:
                robot.set_control_mode(MODE_JOINT_TORQUE)
                robot.wait_control_mode(MODE_JOINT_TORQUE, 500)
            except TimeoutError:
                log.fatal("mode 9 not confirmed by the robot (arm under external force, "
                          "unregistered payload or failed force-control admission?)")
                rc = 1
            except Exception as e:  # noqa: BLE001
                log.fatal(f"entering mode 9 failed: {e!r}")
                rc = 1
            if rc == 0:
                self.phase = "streaming"
                log.info("torque loop running (1 kHz, Python best effort)")
                try:
                    robot.joint_torque_control(self._on_frame,
                                               state_timeout_ms=self.p["state_timeout_ms"])
                except Exception as e:  # noqa: BLE001
                    log.error(f"torque loop failed: {e!r}")
                    rc = 2
                if self.abort_reason:  # _on_frame ended the stream with a final frame
                    log.error(f"safety abort: {self.abort_reason}")
                    rc = 3
                log.info(f"torque loop ended after {self.frames} frames")
        finally:
            # ---- Leave mode 9 ourselves: GDK never does it ----
            self.phase = "restoring"
            try:
                robot.set_control_mode(old_mode)
                log.info(f"control mode restored to {old_mode}")
            except Exception as e:  # noqa: BLE001
                log.error(f"set_control_mode({old_mode}) failed ({e!r}): restore the mode "
                          "manually and check the robot's motion-control error state")
                rc = rc or 2
        return rc

    def _set_realtime(self):
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
        except (PermissionError, OSError) as e:
            msg = f"cannot set SCHED_FIFO for the torque loop ({e})"
            if self.p["enforce_realtime"]:
                raise RuntimeError(msg) from e
            self.get_logger().warn(msg + "; continuing without real-time priority")

    def _gravity_check(self, pos, vel, eff):
        log = self.get_logger()
        g = self.model.gravity(pos)
        effort = self.model.arm(eff)
        diff = g - effort
        lines = ["pre-flight: model gravity vs measured effort (Nm)"]
        for i, name in enumerate(ARM_JOINT_NAMES):
            lines.append(f"  {name:20s} g={g[i]:8.3f}  effort={effort[i]:8.3f}  diff={diff[i]:8.3f}")
        worst = float(np.max(np.abs(diff)))
        log.info("\n".join(lines) + f"\n  max |diff| = {worst:.3f} Nm")
        speed = float(np.max(np.abs(self.model.arm(vel))))
        if speed > 0.05:
            log.warn(f"arms are moving ({speed:.3f} rad/s): the gravity check is unreliable")
        if not np.all(np.isfinite(g)):
            log.fatal("gravity model returned NaN/Inf")
            return False
        tol = self.p["gravity_check_tolerance"]
        if tol > 0.0 and worst > tol:
            log.fatal(f"gravity model differs from the measured effort by {worst:.3f} Nm > "
                      f"gravity_check_tolerance {tol:.3f} Nm: wrong URDF / payload / waist "
                      "posture? Refusing to enter mode 9.")
            return False
        # Fallback torque for an abort before the first loop frame (frame 0 re-seeds it).
        self._prev_tau = effort.copy() if np.all(np.isfinite(effort)) else g
        return True

    def _run_dry(self, robot):
        log = self.get_logger()
        self.phase = "streaming"
        rate = self.p["dry_run_rate"]
        log.info(f"dry run: reading states at {rate:.0f} Hz, computing torques, sending nothing, "
                 "never switching the control mode")
        period = 1.0 / rate
        t_next = time.monotonic()
        while not self.stop.is_set():
            try:
                self._compute(robot.get_joint_states())
            except Abort:
                log.error(f"dry run: safety abort would trigger: {self.abort_reason}")
                return 3
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
        return 0

    def _on_frame(self, state, _period_ns):
        """GDK callback, once per 1 ms frame."""
        req = gdk_req_type()
        req.joint_names = ARM_JOINT_NAMES
        try:
            tau = self._compute(state)
            req.finished = self.stop.is_set()
        except Abort:
            # Final frame: repeat the last torque sent (finite, within the step limit); the
            # previous control mode is restored right after, which holds the arms.
            req.joint_efforts = self._prev_tau.tolist()
            req.finished = True
            return req
        req.joint_efforts = tau.tolist()
        return req

    def _compute(self, state):
        """Torque law for one frame. Raises Abort on a safety violation."""
        self.frames += 1
        names, pos, vel, eff = frame_arrays(state)
        if not self.model.bind(names):
            self._abort("robot state misses an arm joint")
        q, dq, effort = self.model.arm(pos), self.model.arm(vel), self.model.arm(eff)
        g = self.model.gravity(pos)
        ext = self._external_torque()
        if self._first_frame:  # the robot compares frame 0 with its holding torque (~ effort)
            self._prev_tau = effort.copy() if np.all(np.isfinite(effort)) else g.copy()
            self._first_frame = False
        lim = self.p["effort_limit_scale"] * self.model.effort_limits
        step = self.p["max_torque_step"]
        tau = np.clip(g + ext - self.p["joint_damping"] * dq, -lim, lim)
        tau = self._prev_tau + np.clip(tau - self._prev_tau, -step, step)

        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
            self._abort("non-finite joint state")
        if not np.all(np.isfinite(tau)):
            self._abort("non-finite torque")
        vlim = self.p["velocity_limit"]
        if vlim > 0.0:
            i = int(np.argmax(np.abs(dq)))
            if abs(dq[i]) > vlim:
                self._abort(f"{ARM_JOINT_NAMES[i]} at {dq[i]:.3f} rad/s > velocity_limit "
                            f"{vlim:.3f} rad/s")
        self._prev_tau = tau
        if self.frames % self.p["publish_decimation"] == 0:
            self._publish(names, pos, vel, eff, q, dq, effort, tau)
        return tau

    def _abort(self, reason):
        self.abort_reason = reason
        raise Abort(reason)

    def _publish(self, names, pos, vel, eff, q, dq, effort, tau):
        try:
            stamp = self.get_clock().now().to_msg()
            if self._joint_states_pub:
                m = self._full_msg
                m.header.stamp = stamp
                m.name = names
                m.position, m.velocity, m.effort = pos.tolist(), vel.tolist(), eff.tolist()
                self._joint_states_pub.publish(m)
            for arm in range(2):
                if self._arm_state_pubs[arm]:
                    sl = slice(arm * JOINTS_PER_ARM, (arm + 1) * JOINTS_PER_ARM)
                    m = self._arm_msgs[arm]
                    m.header.stamp = stamp
                    m.position, m.velocity, m.effort = (
                        q[sl].tolist(), dq[sl].tolist(), effort[sl].tolist())
                    self._arm_state_pubs[arm].publish(m)
            if self._commanded_pub:
                m = self._cmd_msg
                m.header.stamp = stamp
                m.position, m.velocity, m.effort = q.tolist(), dq.tolist(), tau.tolist()
                self._commanded_pub.publish(m)
        except Exception as e:  # noqa: BLE001  (e.g. context shut down while publishing)
            if rclpy.ok():
                self.get_logger().warn(f"publish failed: {e}", throttle_duration_sec=1.0)


gdk_req_type = None  # agibot_gdk.JointTorqueControlReq, set in main()


def main(args=None):
    global gdk_req_type
    gdk = import_gdk()  # before rclpy, see import_gdk()
    gdk_req_type = gdk.JointTorqueControlReq

    from rclpy.signals import SignalHandlerOptions  # noqa: PLC0415

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()

    def on_signal(_signum, _frame):
        stop.set()  # the torque loop sends its final frame, then the old mode is restored

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rc = 1
    node = None
    executor = SingleThreadedExecutor()
    spin_stop = threading.Event()

    def spin():
        while not spin_stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    # Not a daemon: it is stopped and joined below. A daemon thread killed inside rclpy's C++ code
    # at interpreter exit aborts the process ("terminate called without an active exception").
    spin_thread = threading.Thread(target=spin, name="rclpy_executor")
    try:
        node = TorqueSubscriberNode(stop)
        executor.add_node(node)
        spin_thread.start()
        rc = node.run_control(gdk)
    except Exception as e:  # noqa: BLE001
        (node.get_logger() if node else rclpy.logging.get_logger("g2_torque_subscriber")).fatal(
            f"{e!r}")
    finally:
        spin_stop.set()
        if spin_thread.is_alive():
            spin_thread.join()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
