"""Fake agibot_gdk for offline tests of the Python torque subscriber (no robot, no GDK needed).

The Python counterpart of src/fake_backend.cpp. It provides the calls of the real binding that the
Python node and the top-level scripts make, with the same return types (dicts for the state reads,
TimeoutError from wait_control_mode / joint_torque_read_once), on a simulated dual-arm plant:
Pinocchio forward dynamics (ABA) on the URDF in G2_FAKE_URDF, non-arm joints locked at 0.

Emulated robot behaviour (direct torque mode 9, see the GDK direct torque docs):
  - starts in control mode 3 and holds the arms until a torque stream runs;
  - no gravity compensation: the plant feels exactly the torques that are written;
  - a torque change > 50 Nm on one joint between frames (the first frame is compared with the
    holding torque g(q)) is an e-stop: the write fails and the mode drops to 1;
  - a frame without exactly the 14 arm joints or with a non-finite torque is rejected with
    ValueError (kInvalidInput), as by the real binding;
  - URDF joint limits act as hard stops.
Environment: G2_FAKE_URDF (required), G2_FAKE_FRICTION (viscous friction, Nm*s/rad, default 0.5),
             G2_FAKE_INITIAL_POSE (14 comma-separated rad, default all 0 clamped to limits).

Use: put the directory holding this package first on PYTHONPATH and run the node with
backend:=fake (the node refuses a fake GDK otherwise, and the real GDK with backend:=fake).
numpy and pinocchio are imported in Robot(), not here: the node imports this module with
RTLD_LOCAL | RTLD_DEEPBIND, which must not apply to them.
"""

import math
import os
import threading
import time

IS_G2_FAKE = True  # checked by the Python node against its backend parameter

ARMS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3", "idx24_arm_l_joint4",
    "idx25_arm_l_joint5", "idx26_arm_l_joint6", "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3", "idx64_arm_r_joint4",
    "idx65_arm_r_joint5", "idx66_arm_r_joint6", "idx67_arm_r_joint7",
]
MODE_JOINT_TORQUE = 9
E_STOP_STEP_NM = 50.0
DT = 0.001


def gdk_init():
    return 0


def gdk_release():
    return 0


class JointTorqueControlReq:
    def __init__(self):
        self.joint_names = []
        self.joint_efforts = []
        self.end_effector_joint_names = []
        self.end_effector_joint_positions = []
        self.finished = False


class MotionControlStatus:
    def __init__(self, control_mode):
        self.control_mode = control_mode
        self.mode = control_mode
        self.error_code = 0
        self.error_msg = ""


class Robot:
    def __init__(self):
        import numpy as np  # noqa: PLC0415  (see the module docstring)
        import pinocchio as pin  # noqa: PLC0415

        self._np, self._pin = np, pin
        urdf = os.environ.get("G2_FAKE_URDF", "")
        if not urdf:
            raise RuntimeError("fake agibot_gdk: set G2_FAKE_URDF (the simulated arms are built "
                               "from it)")
        full = pin.buildModelFromUrdf(urdf)
        lock = [j for j in range(1, full.njoints) if full.names[j] not in ARMS]
        self._others = [full.names[j] for j in lock
                        if full.joints[j].nq == 1 and full.joints[j].nv == 1]
        self._model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self._data = self._model.createData()
        for name in ARMS:
            if not self._model.existJointName(name):
                raise RuntimeError(f"fake agibot_gdk: URDF has no joint {name}")
        joints = [self._model.joints[self._model.getJointId(n)] for n in ARMS]
        self._iq = np.array([j.idx_q for j in joints])
        self._iv = np.array([j.idx_v for j in joints])
        self._q = pin.neutral(self._model)
        self._v = np.zeros(self._model.nv)
        self._friction = float(os.environ.get("G2_FAKE_FRICTION", "0.5"))
        pose = os.environ.get("G2_FAKE_INITIAL_POSE")
        if pose:
            values = [float(x) for x in pose.split(",")][:len(ARMS)]
            self._q[self._iq[:len(values)]] = values
        self._clamp_to_limits()
        self._lock = threading.Lock()
        self._mode = 3
        self._held = self._gravity()  # holding torque, compared with the first torque frame
        self._applied = self._held.copy()
        self._streaming = False
        self._prev = None
        self._next_tick = 0.0

    # ---- plain reads, any mode ----
    def get_joint_states(self):
        with self._lock:
            return self._state()

    def get_whole_body_status(self):
        # A dict, like the real binding. The fake reports no URDF type: pass urdf_path.
        return {"timestamp": time.time_ns(), "urdf_type": "", "left_arm_error": 0,
                "right_arm_error": 0, "left_arm_estop": False, "right_arm_estop": False}

    def get_motion_control_status(self):
        return MotionControlStatus(self._mode)

    # ---- control mode ----
    def set_control_mode(self, control_mode):
        with self._lock:
            if control_mode != MODE_JOINT_TORQUE:  # any other mode: hold the arms where they are
                self._streaming = False
                self._v[:] = 0.0
                self._held = self._gravity()
                self._applied = self._held.copy()
            self._mode = int(control_mode)
        return 0

    def wait_control_mode(self, mode, timeout_ms):
        if self._mode != mode:
            raise TimeoutError("wait_control_mode timed out")
        return 0

    # ---- torque stream (mode 9) ----
    def joint_torque_read_once(self, timeout_ms=100):
        if self._mode != MODE_JOINT_TORQUE:
            raise RuntimeError("joint_torque_read_once failed")
        now = time.monotonic()
        if not self._streaming:  # first frame of a stream
            self._streaming = True
            self._prev = self._held.copy()
            self._next_tick = now
            period_ns = 0
        else:
            self._next_tick += DT
            if self._next_tick > now:
                time.sleep(self._next_tick - now)
            period_ns = int(DT * 1e9)
        with self._lock:
            return self._state(), period_ns

    def joint_torque_write_once(self, req):
        np = self._np
        if self._mode != MODE_JOINT_TORQUE or not self._streaming:
            raise RuntimeError("joint_torque_write_once failed")
        names, efforts = list(req.joint_names), list(req.joint_efforts)
        if (sorted(names) != sorted(ARMS) or len(efforts) != len(names)
                or not all(math.isfinite(float(e)) for e in efforts)):
            raise ValueError("joint_torque_write_once invalid input")
        by_name = dict(zip(names, (float(e) for e in efforts)))
        tau = np.array([by_name[n] for n in ARMS])
        step = np.abs(tau - self._prev)
        i = int(np.argmax(step))
        if step[i] > E_STOP_STEP_NM:
            self.set_control_mode(1)
            raise RuntimeError(f"fake robot E-STOP: torque step {tau[i] - self._prev[i]:.3f} Nm "
                               f"on {ARMS[i]} (limit {E_STOP_STEP_NM:g})")
        self._prev = tau
        with self._lock:
            self._step(tau)
        if req.finished:
            self._streaming = False
        return 0

    def joint_torque_end(self):
        self._streaming = False
        return 0

    def joint_torque_control(self, callback, state_timeout_ms=100):
        """Same loop as the real agibot_gdk.joint_torque_control (read -> callback -> write)."""
        try:
            while True:
                state, period_ns = self.joint_torque_read_once(state_timeout_ms)
                req = callback(state, period_ns)
                self.joint_torque_write_once(req)
                if req.finished:
                    return
        finally:
            try:
                self.joint_torque_end()
            except Exception:  # noqa: BLE001
                pass

    # ---- plant ----
    def _gravity(self):
        g = self._pin.computeGeneralizedGravity(self._model, self._data, self._q)
        return self._np.asarray(g)[self._iv].copy()

    def _clamp_to_limits(self):
        lo = self._model.lowerPositionLimit[self._iq]
        hi = self._model.upperPositionLimit[self._iq]
        q = self._q[self._iq]
        out = (lo < hi) & ((q < lo) | (q > hi))
        if out.any():
            self._q[self._iq[out]] = self._np.clip(q[out], lo[out], hi[out])
            self._v[self._iv[out]] = 0.0

    def _step(self, tau_arm):
        np, pin = self._np, self._pin
        tau = np.zeros(self._model.nv)
        tau[self._iv] = tau_arm - self._friction * self._v[self._iv]
        a = pin.aba(self._model, self._data, self._q, self._v, tau)
        self._v += a * DT  # semi-implicit Euler
        self._q = pin.integrate(self._model, self._q, self._v * DT)
        self._clamp_to_limits()
        self._applied = np.array(tau_arm, dtype=float)

    def _state(self):
        states = []
        for k, name in enumerate(ARMS):
            q, v = float(self._q[self._iq[k]]), float(self._v[self._iv[k]])
            states.append({"name": name, "mode": 0, "position": q, "velocity": v,
                           "effort": float(self._applied[k]), "motor_position": q,
                           "motor_velocity": v, "motor_current": 0.0, "error_code": 0})
        for name in self._others:  # locked at neutral (0)
            states.append({"name": name, "mode": 0, "position": 0.0, "velocity": 0.0,
                           "effort": 0.0, "motor_position": 0.0, "motor_velocity": 0.0,
                           "motor_current": 0.0, "error_code": 0})
        return {"timestamp": time.time_ns(), "nums": len(states), "states": states}
