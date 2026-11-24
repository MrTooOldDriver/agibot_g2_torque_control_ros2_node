#!/usr/bin/env python3
"""Hello world of GDK direct joint torque control, callback form: move both arms to a pose
with joint impedance and come back. Python counterpart of
src/joint_impedance_move_callback_example.cpp.

robot.joint_torque_control() owns the loop: it reads one robot state frame every 1 ms, calls
our callback, and sends the returned command. The callback sets req.finished = True to end the
stream. Compare with joint_impedance_move_example.py, which calls joint_torque_read_once /
joint_torque_write_once by hand.

Direct torque control (control mode 9) in short:
  1. Remember the current control mode, then switch to mode 9.
  2. Run the torque loop (the callback returns 14 joint torques, left 7 + right 7).
  3. Switch back to the old mode. GDK never switches the mode for you.

Torque law (joint impedance + inverse dynamics feed-forward):
  tau = rnea(q, dq, ddq_d) + Kp (q_d - q) + Kd (dq_d - dq)
ArmDynamics (Pinocchio) is defined at the top of this file; q_d is a quintic
(minimum jerk) path from the start pose to the "elbow offset" pose (left joint 4 +DELTA,
right joint 4 -DELTA), then, after a short hold, the same kind of path brings the arms back to
the start pose.

Python 1 kHz is best-effort; use the C++ sample for hard real-time.

Run from this directory (the robot moves, keep the area around it clear):
  python3 joint_impedance_move_callback_example.py [urdf_path]
Without urdf_path the URDF of the robot model reported by the robot is used.
"""

import signal
import sys

import agibot_gdk as gdk
import numpy as np
import pinocchio as pin

DT = 0.001
URDF_DIR = "/home/agi/app/share/genie_robot_description/urdf"
ARMS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3", "idx24_arm_l_joint4",
    "idx25_arm_l_joint5", "idx26_arm_l_joint6", "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3", "idx64_arm_r_joint4",
    "idx65_arm_r_joint5", "idx66_arm_r_joint6", "idx67_arm_r_joint7",
]
# Per-joint gains (left 7, right 7), aligned with JointImpedance's dual-arm values.
KP = np.array([400, 400, 400, 500, 300, 200, 200, 400, 400, 400, 500, 300, 200, 200], float)
KD = np.array([30, 30, 30, 25, 25, 10, 10, 30, 30, 30, 25, 25, 10, 10], float)


class ArmDynamics:
    """Pinocchio RNEA of the 14 arm joints; waist/head take their measured values."""

    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.neutral = pin.neutral(self.model)
        self.arm_q, self.arm_v, self.limit = [], [], []
        for name in ARMS:
            if not self.model.existJointName(name):
                raise RuntimeError(f"URDF has no joint {name}")
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise RuntimeError(f"joint {name} is not 1-DoF")
            self.arm_q.append(joint.idx_q)
            self.arm_v.append(joint.idx_v)
            self.limit.append(max(1.0, float(self.model.effortLimit[joint.idx_v])))
        self.limit = np.array(self.limit)
        self.others = []  # (name, idx_q, idx_v) of the non-arm 1-DoF joints
        for jid in range(1, self.model.njoints):
            joint, name = self.model.joints[jid], self.model.names[jid]
            if joint.nq == 1 and joint.nv == 1 and name not in ARMS:
                self.others.append((name, joint.idx_q, joint.idx_v))

    @staticmethod
    def extract(state):
        """(q, dq) of the 14 arm joints from one frame, or None if a joint is missing."""
        by_name = {s["name"]: s for s in state["states"]}
        if any(n not in by_name for n in ARMS):
            return None
        return (np.array([by_name[n]["position"] for n in ARMS]),
                np.array([by_name[n]["velocity"] for n in ARMS]))

    def rnea(self, state, q, dq, ddq):
        """Joint torques of the 14 arm joints; the non-arm joints take their measured values."""
        by_name = {s["name"]: s for s in state["states"]}
        qf, vf, af = self.neutral.copy(), np.zeros(self.model.nv), np.zeros(self.model.nv)
        for name, iq, iv in self.others:
            if name in by_name:
                qf[iq], vf[iv] = by_name[name]["position"], by_name[name]["velocity"]
        qf[self.arm_q], vf[self.arm_v], af[self.arm_v] = q, dq, ddq
        return pin.rnea(self.model, self.data, qf, vf, af)[self.arm_v]


def quintic(q0, q1, duration, t):
    """Minimum-jerk interpolation: position, velocity, acceleration at time t."""
    s = min(max(t / duration, 0.0), 1.0)
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5
    vel = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    acc = (60 * s - 180 * s**2 + 120 * s**3) / duration**2
    d = q1 - q0
    return q0 + d * pos, d * vel, d * acc

DELTA = 0.15          # rad, elbow offset of the goal
SOFT_START = 0.5      # s, ramp the impedance part up from 0 so torque does not step in
MOVE_SECONDS = 3.0    # s, duration of each quintic path (out and back)
HOLD_SECONDS = 1.0    # s, hold the goal before returning
N_CYCLES = 0          # out-and-back cycles to run, 0 = until Ctrl+C
MAX_STEP_NM = 15.0    # Nm per frame, keep below MC joint_torque_control_config.max_step_nm
MAX_TRACK_ERR = 0.35  # rad, abort when the arm lags the path by more than this
MODE_JOINT_TORQUE = 9

stops = 0


def on_signal(_signum, _frame):
    global stops  # 1st Ctrl+C: finish the current cycle (arms end at the start pose); 2nd: stop now
    stops += 1


def reference(k, q_from, q_to):
    """(q_d, dq_d, ddq_d, scale) of frame k: soft start, then cycles of quintic out -> hold -> back.

    Every cycle ends at the start pose with zero velocity, so cycles chain without a jump.
    """
    n_soft, n_move, n_hold = int(SOFT_START / DT), int(MOVE_SECONDS / DT), int(HOLD_SECONDS / DT)
    zero = np.zeros(len(ARMS))
    if k < n_soft:  # soft start: stay at the start pose, impedance ramps 0 -> 1
        return q_from, zero, zero, (k + 1) / n_soft
    kc = (k - n_soft) % (2 * n_move + n_hold)  # frame within the current cycle
    if n_move <= kc < n_move + n_hold:  # hold the goal
        return q_to, zero, zero, 1.0
    out = kc < n_move  # quintic path out in the first phase, back in the last
    a, b = (q_from, q_to) if out else (q_to, q_from)
    k0 = 0 if out else n_move + n_hold
    return quintic(a, b, MOVE_SECONDS, (kc - k0 + 1) * DT) + (1.0,)


def run(robot, urdf):
    # ---- Dynamics model (Pinocchio wrapper). Needs a URDF that matches the robot. ----
    if not urdf:
        name = getattr(robot.get_whole_body_status(), "urdf_type", "")
        if not name:
            print("urdf_type unavailable, pass the urdf path as argument", file=sys.stderr)
            return 1
        urdf = f"{URDF_DIR}/{name}/{name}.urdf"
    print(f"urdf: {urdf}")
    dyn = ArmDynamics(urdf)

    # ---- Start pose and goal. get_joint_states is a plain state read, usable in any mode. ----
    arm = dyn.extract(robot.get_joint_states())
    if arm is None:
        print("the frame misses an arm joint", file=sys.stderr)
        return 1
    q_start = arm[0]
    goal = q_start.copy()
    goal[3] += DELTA    # left elbow
    goal[10] -= DELTA   # right elbow (mirrored)

    # ---- Remember the control mode so it can be restored at the end. ----
    old_mode = robot.get_motion_control_status().control_mode
    if old_mode == MODE_JOINT_TORQUE:
        print("robot is already in mode 9 (a previous run did not exit cleanly?), "
              "cannot tell which mode to restore", file=sys.stderr)
        return 1
    print(f"old control mode: {old_mode}")

    # ---- Callback: GDK calls it once per frame and sends what it returns. ----
    zero = np.zeros(len(ARMS))
    prev_tau = zero
    state = {"frames": 0, "rc": 0, "lost": False}
    n_soft = int(SOFT_START / DT)
    n_cycle = 2 * int(MOVE_SECONDS / DT) + int(HOLD_SECONDS / DT)

    def callback(frame, _period_ns):
        nonlocal prev_tau
        req = gdk.JointTorqueControlReq()
        req.joint_names = list(ARMS)  # order of joint_efforts below
        req.finished = False
        arm = dyn.extract(frame)      # (q, dq): measured position / velocity of the 14 arm joints
        if arm is None:
            print("frame misses an arm joint", file=sys.stderr)
            state["rc"] = 2
            req.joint_efforts = [0.0] * len(ARMS)
            req.finished = True
            return req
        q, dq = arm

        # Reference and impedance weight for this frame.
        q_d, dq_d, ddq_d, scale = reference(state["frames"], q_start, goal)
        # tau = rnea + impedance. rnea(frame, ...) needs the frame for the non-arm joints.
        tau = dyn.rnea(frame, q, dq, ddq_d) + scale * (KP * (q_d - q) + KD * (dq_d - dq))
        # Safety: clamp to the URDF effort limit, and limit the change per frame.
        tau = np.clip(tau, -dyn.limit, dyn.limit)
        tau = prev_tau + np.clip(tau - prev_tau, -MAX_STEP_NM, MAX_STEP_NM)
        prev_tau = tau
        req.joint_efforts = [float(v) for v in tau]

        err = float(np.max(np.abs(q_d - q)))
        if err > MAX_TRACK_ERR:  # open loop is not following the path: stop commanding
            print(f"tracking error {err:.3f} rad > {MAX_TRACK_ERR}, abort", file=sys.stderr)
            state["rc"], state["lost"] = 3, True
            req.finished = True
            return req
        if state["frames"] % 500 == 0:
            print(f"t={state['frames'] * DT:.1f}s |e|_max={err:.4f}", flush=True)

        state["frames"] += 1
        # The final command ends the stream. Stop only where the arms are at the start pose
        # (end of the soft start or of a cycle); a second Ctrl+C stops at once.
        done = state["frames"] - n_soft
        at_start = done <= 0 or done % n_cycle == 0
        if done > 0 and done % n_cycle == 0:
            print(f"cycle {done // n_cycle} done", flush=True)
        req.finished = stops >= 2 or (at_start and (
            stops >= 1 or (N_CYCLES > 0 and done >= N_CYCLES * n_cycle)))
        return req

    # ---- Enter mode 9 and wait until the MC reports it. ----
    rc = 0
    try:
        robot.set_control_mode(MODE_JOINT_TORQUE)
        robot.wait_control_mode(MODE_JOINT_TORQUE, 500)
        print("move (callback form): hold the arms until the motion starts")
        robot.joint_torque_control(callback, state_timeout_ms=100)
        # The stream is already ended here, no joint_torque_end() needed.
    except (TimeoutError, ValueError, RuntimeError) as e:  # GDK read/write/mode failures
        print(f"torque loop failed: {e}", file=sys.stderr)
        rc = 2
    rc = state["rc"] or rc

    # ---- Cleanup: leave mode 9 ourselves. ----
    if state["lost"]:
        # Tracking was lost: do not switch modes, the MC may have estopped. Check it first.
        print("mode not restored (tracking lost): check the MC estop state", file=sys.stderr)
    else:
        try:
            robot.set_control_mode(old_mode)
        except (TimeoutError, ValueError, RuntimeError) as e:
            print(f"set_control_mode({old_mode}) failed, restore it manually: {e}", file=sys.stderr)
            rc = rc or 2
    print(f"done, rc={rc}")
    return rc


def main():
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    gdk.gdk_init()
    try:
        return run(gdk.Robot(), sys.argv[1] if len(sys.argv) > 1 else "")
    finally:
        gdk.gdk_release()


if __name__ == "__main__":
    sys.exit(main())