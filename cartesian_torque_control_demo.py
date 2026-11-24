#!/usr/bin/env python3
"""Cartesian spring-damper on the arm TCPs, with GDK direct joint torque control (mode 9).

Builds on joint_torque_demo.py, which must sit in the same directory. It reuses that script's
Pinocchio ArmDynamics, joint order, gains and safety limits, and the same mode-9 lifecycle:
remember the control mode, switch to mode 9, run robot.joint_torque_control(callback), and
restore the mode at the end.

Each arm runs in the mode set in ARM_MODE:
  "cartesian"  A virtual spring and damper pull the TCP towards a fixed target point. The TCP is
               arm_<l|r>_end_link, the flange after joint 7, before the gripper.
                   F_s = K (p_target - p), |F_s| clipped to MAX_FORCE     spring, base_link axes
                   F   = F_s - D v,        |F|   clipped to MAX_FORCE     spring + damper
                   tau = J_v^T F + rnea(q, dq, 0)        rnea(q, dq, 0) = g(q) + C(q, dq) dq
               The clips scale the norm and keep the direction. The spring is clipped before
               the damper is added, so the damper still acts when the spring saturates (TCP more
               than MAX_FORCE / K from the target): a TCP pushed far away comes back at no more
               than about MAX_FORCE / D (1 m/s), instead of accelerating under MAX_FORCE.
               Push the TCP away and the spring pulls it back to the target. Only the TCP
               position is controlled. There is no nullspace control, so the TCP orientation
               and the elbow (4 of the 7 DoF) have no stiffness or damping: they float on the
               gravity compensation and stay where you leave them. Joint friction leaves a
               steady-state error of about friction / K.
  "hold"       The arm is held at its start pose with the joint impedance of joint_torque_demo
               (rnea + KP/KD). Mode 9 has no gravity compensation, so an arm cannot be left at
               zero torque.
Start with the left arm "cartesian" and the right arm "hold"; once the left arm works, set the
right arm to "cartesian" too.

Frames: everything is expressed in base_link, the URDF root (x forward, y left, z up).
  - p is the TCP origin in base_link, and the target is a point in base_link.
  - J_v is the 3 x nv translational part of the TCP frame Jacobian in LOCAL_WORLD_ALIGNED: it
    maps joint velocities to the velocity of the TCP origin, in base_link axes. So v = J_v vf is
    the TCP velocity in the same axes as p and F, and J_v^T F is the joint torque that exerts
    F at the TCP.
  - The waist sits between base_link and the arms. The model takes the measured waist (and head)
    joints from every frame, so p and J_v hold for any waist pose. The waist is not torque
    controlled: only the 7 arm columns of J_v produce torque.

Joint limits: the robot has no joint-limit protection in mode 9, so the script adds two layers on
all 14 arm joints. The limits are the tighter of the URDF and robot.get_joint_limits().
  - Wall: within LIMIT_ZONE of a limit, a virtual spring-damper pushes the joint back. Its gains
    are fractions of the vendor joint impedance gains KP and KD, so they scale with the joint.
    At startup the script checks that the wall beats the clipped Cartesian force, so the spring
    alone cannot drive a joint into a limit.
  - Stop: if a joint is pushed through the wall to within LIMIT_ABORT of a limit, the script
    stops and restores the control mode, and the robot holds the arms where they are. The check
    counts the distance the joint covers in LIMIT_STOP_TIME at its current speed, because the
    robot needs a few tens of ms to take over.

The script runs until Ctrl+C (or SIGTERM), with no time limit, so you can push the arms around.
On every stop (Ctrl+C, SIGTERM, a safety abort or an error) it sends a last frame, ending the
torque stream, and restores the previous control mode, which holds the arms where they are.

Run from this directory. The robot moves, so keep the area around it clear:
  python3 cartesian_torque_control_demo.py [urdf_path]
Without urdf_path it uses the default of joint_torque_demo.py (genie_g2_crs_omnipicker.urdf).
"""

import signal
import sys

import agibot_gdk as gdk
import numpy as np
import pinocchio as pin

from joint_torque_demo import (ARMS, DEFAULT_URDF, DT, KD, KP, MAX_STEP_NM, MAX_TRACK_ERR,
                               MODE_JOINT_TORQUE, SOFT_START, URDF_DIR, ArmDynamics)

# ---- What each arm does: "cartesian" (spring-damper to its target) or "hold" (start pose). ----
ARM_MODE = {"left": "cartesian", "right": "cartesian"}

# ---- Cartesian spring-damper: edit here. ----
K_SPRING = 200.0    # N/m, translational stiffness
D_DAMPER = 30.0    # N*s/m, translational damping
MAX_FORCE = 20.0   # N, limit on |F| of spring + damper together
# Target of each "cartesian" TCP: an offset (m) in base_link axes (x forward, y left, z up) from
# the TCP position at start. The axes are the same for both arms, so mirror a y offset for the
# right arm (+y moves the left arm outwards but the right arm inwards). Set TARGET_POSITION[side]
# to an absolute [x, y, z] in base_link (m) to use that instead. Beyond MAX_FORCE / K_SPRING
# (0.2 m) the spring saturates and the TCP approaches at up to MAX_FORCE / D_DAMPER (1 m/s).
TARGET_OFFSET = {"left": np.array([0.20, 0.0, 0.20]),    # 10 cm forward, 10 cm up
                 "right": np.array([0.20, 0.0, 0.20])}
TARGET_POSITION = {"left": None, "right": None}

# ---- Joint limit protection, all 14 arm joints ----
LIMIT_ZONE = 0.20        # rad, the wall acts within this distance of a limit
LIMIT_K_SCALE = 0.3     # wall stiffness = LIMIT_K_SCALE * KP (Nm/rad per joint)
LIMIT_D_SCALE = 0.50     # wall damping = LIMIT_D_SCALE * KD (Nm*s/rad per joint); with 0.25 * KP
                         # this keeps the damping ratio of the vendor joint impedance
LIMIT_ABORT = 0.05       # rad, stop when a joint gets this close to a limit,
LIMIT_STOP_TIME = 0.05   # s, counting how far it moves in this time at its current speed

# ---- Other safety ----
MAX_TARGET_OFFSET = 0.60   # m, refuse a target farther than this from the start TCP
REACH_MARGIN = 0.95        # refuse a target farther from the shoulder than this * arm length
                           # (0.95: the elbow stays bent by about 37 deg or more)
MAX_JOINT_VEL = 2.5        # rad/s, abort if any arm joint moves faster (runaway guard)
START_MISMATCH = 0.01      # m, abort if the first torque-loop frame puts a TCP this far from
                           # the TCP of the start read (the two state sources disagree)

# side: (TCP frame, shoulder centre frame where the axes of joints 1-3 meet, slice of the
# 14-vectors in ARMS order)
SIDES = {
    "left": ("arm_l_end_link", "arm_l_link2", slice(0, 7)),
    "right": ("arm_r_end_link", "arm_r_link2", slice(7, 14)),
}
MODES = ("cartesian", "hold")

stop_requested = False


def on_signal(_signum, _frame):
    global stop_requested  # Ctrl+C / SIGTERM: end the torque stream on the next frame
    stop_requested = True


class CartesianArmDynamics(ArmDynamics):
    """ArmDynamics plus the TCP position and translational Jacobian of each arm."""

    def __init__(self, urdf_path):
        super().__init__(urdf_path)
        self.tcp, self.shoulder, self.cols = {}, {}, {}
        for side, (tcp, shoulder, sl) in SIDES.items():
            for name in (tcp, shoulder):
                if not self.model.existFrame(name):
                    raise RuntimeError(f"URDF has no frame {name}")
            self.tcp[side] = self.model.getFrameId(tcp)
            self.shoulder[side] = self.model.getFrameId(shoulder)
            self.cols[side] = self.arm_v[sl]   # velocity indices of the arm's 7 joints
        self.waist = [name for name, _, _ in self.others if "body_joint" in name]
        self.urdf_lo = self.model.lowerPositionLimit[self.arm_q].copy()
        self.urdf_hi = self.model.upperPositionLimit[self.arm_q].copy()
        # The kinematics use their own Data, because rnea also writes the placements in self.data.
        self.kin = self.model.createData()
        self.zero_a = np.zeros(self.model.nv)
        # Lever of each arm joint: distance from the joint to its TCP with the arm straight (all
        # arm joints at 0), about the largest |J_v column| in any pose.
        pin.framesForwardKinematics(self.model, self.kin, self.neutral)
        self.lever = np.zeros(len(ARMS))
        for side, (_, _, sl) in SIDES.items():
            p_tcp = self.kin.oMf[self.tcp[side]].translation
            for k in range(sl.start, sl.stop):
                p_joint = self.kin.oMi[self.model.getJointId(ARMS[k])].translation
                self.lever[k] = np.linalg.norm(p_tcp - p_joint)

    def read(self, state):
        """(q, dq, qf, vf) of one frame, or None if an arm joint is missing.

        q, dq: the 14 arm joints. qf, vf: the full model configuration, made of the neutral pose,
        then the measured non-arm joints (waist, head, gripper), then the arms.
        """
        by_name = {s["name"]: s for s in state["states"]}
        if any(n not in by_name for n in ARMS):
            return None
        q = np.array([by_name[n]["position"] for n in ARMS])
        dq = np.array([by_name[n]["velocity"] for n in ARMS])
        qf, vf = self.neutral.copy(), np.zeros(self.model.nv)
        for name, iq, iv in self.others:
            s = by_name.get(name)
            if s is not None:
                qf[iq], vf[iv] = s["position"], s["velocity"]
        qf[self.arm_q], vf[self.arm_v] = q, dq
        return q, dq, qf, vf

    def bias(self, qf, vf):
        """g(q) + C(q, dq) dq of the 14 arm joints: rnea with zero joint acceleration."""
        return pin.rnea(self.model, self.data, qf, vf, self.zero_a)[self.arm_v]

    def tcp_kinematics(self, qf, sides):
        """{side: (p, J_v)}: TCP origin in base_link, and its 3 x nv translational Jacobian.

        LOCAL_WORLD_ALIGNED: J_v vf is the velocity of the TCP origin in base_link axes. WORLD
        would give the velocity of the point at the base_link origin, and LOCAL would use the TCP
        axes; neither matches F.
        """
        pin.computeJointJacobians(self.model, self.kin, qf)   # also runs forward kinematics
        out = {}
        for side in sides:
            fid = self.tcp[side]
            pin.updateFramePlacement(self.model, self.kin, fid)
            J = pin.getFrameJacobian(self.model, self.kin, fid, pin.LOCAL_WORLD_ALIGNED)
            out[side] = (self.kin.oMf[fid].translation.copy(), J[:3, :].copy())
        return out

    def shoulder_and_reach(self, qf, side):
        """Shoulder centre in base_link at qf, and the arm length from shoulder to TCP.

        The arm length is measured with all arm joints at 0, where the arm is straight.
        """
        tcp, shoulder = self.tcp[side], self.shoulder[side]
        pin.framesForwardKinematics(self.model, self.kin, self.neutral)
        reach = np.linalg.norm(self.kin.oMf[tcp].translation - self.kin.oMf[shoulder].translation)
        pin.framesForwardKinematics(self.model, self.kin, qf)
        return self.kin.oMf[shoulder].translation.copy(), reach


def joint_limits(robot, dyn):
    """(lo, hi) of the 14 arm joints: the tighter of the URDF and robot.get_joint_limits()."""
    lo, hi = dyn.urdf_lo.copy(), dyn.urdf_hi.copy()
    try:
        runtime = robot.get_joint_limits()   # {name: {"min": rad, "max": rad}}
        robot_lo = np.array([float(runtime[n]["min"]) for n in ARMS])
        robot_hi = np.array([float(runtime[n]["max"]) for n in ARMS])
    except (AttributeError, KeyError, TypeError, ValueError, RuntimeError, TimeoutError) as e:
        print(f"joint limits: URDF only, robot.get_joint_limits() failed ({e!r})")
        return lo, hi
    for k, name in enumerate(ARMS):
        if abs(robot_lo[k] - lo[k]) > 1e-3 or abs(robot_hi[k] - hi[k]) > 1e-3:
            print(f"  {name}: URDF [{lo[k]:+.4f}, {hi[k]:+.4f}], "
                  f"robot [{robot_lo[k]:+.4f}, {robot_hi[k]:+.4f}]")
    print("joint limits: the tighter of the URDF and the robot")
    return np.maximum(lo, robot_lo), np.minimum(hi, robot_hi)


def fmt(v):
    return "[" + ", ".join(f"{x:+.3f}" for x in v) + "]"


def run(robot, urdf):
    if set(ARM_MODE) != set(SIDES) or any(m not in MODES for m in ARM_MODE.values()):
        print(f"ARM_MODE must map 'left' and 'right' to one of {MODES}", file=sys.stderr)
        return 1
    cart = [s for s in SIDES if ARM_MODE[s] == "cartesian"]
    hold = [s for s in SIDES if ARM_MODE[s] == "hold"]
    print(f"arm modes: left {ARM_MODE['left']}, right {ARM_MODE['right']}")

    # ---- Dynamics model (Pinocchio wrapper). Needs a URDF that matches the robot. ----
    if not urdf:
        name = robot.get_whole_body_status().get("urdf_type", "")  # the binding returns a dict
        if not name:
            print("urdf_type unavailable, pass the urdf path as argument", file=sys.stderr)
            return 1
        urdf = f"{URDF_DIR}/{name}/{name}.urdf"
    print(f"urdf: {urdf}")
    dyn = CartesianArmDynamics(urdf)

    # ---- Start pose. get_joint_states is a plain state read, usable in any mode. ----
    start_state = robot.get_joint_states()
    start = dyn.read(start_state)
    if start is None:
        print("the frame misses an arm joint", file=sys.stderr)
        return 1
    q_start, _, qf_start, _ = start
    names = {s["name"] for s in start_state["states"]}
    missing = [n for n in dyn.waist if n not in names]
    if missing:
        print(f"warning: the joint states miss the waist joints {missing}; the model takes them "
              "as 0, so the TCP and the target are off if the waist is not at 0", file=sys.stderr)

    # ---- Joint limits and the wall. ----
    q_lo, q_hi = joint_limits(robot, dyn)
    k_wall, d_wall = LIMIT_K_SCALE * KP, LIMIT_D_SCALE * KD
    if not 0.0 < LIMIT_ABORT < LIMIT_ZONE or np.any(q_hi - q_lo <= 2 * LIMIT_ZONE):
        print("need 0 < LIMIT_ABORT < LIMIT_ZONE < half of every joint range", file=sys.stderr)
        return 1
    # The wall at the stop depth must beat the largest torque the clipped force can make.
    for side in cart:
        sl = SIDES[side][2]
        wall_max = k_wall[sl] * (LIMIT_ZONE - LIMIT_ABORT)
        spring_max = MAX_FORCE * dyn.lever[sl]
        weak = [f"{ARMS[sl.start + k]} ({wall_max[k]:.1f} < {spring_max[k]:.1f} Nm)"
                for k in range(7) if wall_max[k] < spring_max[k]]
        if weak:
            print(f"the limit wall is weaker than MAX_FORCE x lever on {weak}: raise "
                  "LIMIT_K_SCALE or lower MAX_FORCE", file=sys.stderr)
            return 1
    dist_start = np.minimum(q_start - q_lo, q_hi - q_start)
    for side, (_, _, sl) in SIDES.items():
        print(f"{side} arm, distance to the nearest joint limit (rad): "
              + " ".join(f"{d:.2f}" for d in dist_start[sl]))
    j = int(np.argmin(dist_start))
    if dist_start[j] < LIMIT_ABORT:
        print(f"{ARMS[j]} starts {dist_start[j]:.3f} rad from its limit, closer than "
              f"LIMIT_ABORT={LIMIT_ABORT}: move it away first", file=sys.stderr)
        return 1
    near = [ARMS[k] for k in range(len(ARMS)) if dist_start[k] < LIMIT_ZONE]
    if near:
        print(f"warning: {near} start within LIMIT_ZONE={LIMIT_ZONE} rad of a limit; the wall "
              "will push them out, so these joints move by themselves at start", file=sys.stderr)

    def limit_wall(q, dq):
        """(tau, d_lo, d_hi): wall torque near the limits, distances to the lower/upper limit.

        One-sided: zero outside LIMIT_ZONE, LIMIT_K_SCALE*KP*depth inside it, plus damping.
        """
        d_lo, d_hi = q - q_lo, q_hi - q
        depth_lo = np.maximum(LIMIT_ZONE - d_lo, 0.0)
        depth_hi = np.maximum(LIMIT_ZONE - d_hi, 0.0)
        inside = (depth_lo + depth_hi) > 0.0
        return k_wall * (depth_lo - depth_hi) - inside * d_wall * dq, d_lo, d_hi

    # ---- Targets of the "cartesian" arms. ----
    p_start, target = {}, {}
    kin = dyn.tcp_kinematics(qf_start, cart)
    for side in cart:
        p_start[side] = kin[side][0]
        if TARGET_POSITION[side] is not None:
            target[side] = np.array(TARGET_POSITION[side], float)
        else:
            target[side] = p_start[side] + TARGET_OFFSET[side]
        print(f"{side} TCP {SIDES[side][0]} in base_link: start {fmt(p_start[side])} m, "
              f"target {fmt(target[side])} m")
        offset = np.linalg.norm(target[side] - p_start[side])
        if offset > MAX_TARGET_OFFSET:
            print(f"{side} target is {offset:.3f} m from the start TCP, more than "
                  f"{MAX_TARGET_OFFSET} m", file=sys.stderr)
            return 1
        # Rough reach check: refuse a target that stretches the arm further than it is now and
        # past REACH_MARGIN of its length.
        shoulder, reach = dyn.shoulder_and_reach(qf_start, side)
        d_target = np.linalg.norm(target[side] - shoulder)
        if d_target > max(REACH_MARGIN * reach, np.linalg.norm(p_start[side] - shoulder)):
            print(f"{side} target is {d_target:.3f} m from the shoulder, out of reach (arm "
                  f"length {reach:.3f} m x {REACH_MARGIN})", file=sys.stderr)
            return 1
    if cart:
        print(f"K={K_SPRING} N/m, D={D_DAMPER} N*s/m, max |F|={MAX_FORCE} N")

    # ---- Remember the control mode so it can be restored at the end. ----
    old_mode = robot.get_motion_control_status().control_mode
    if old_mode == MODE_JOINT_TORQUE:
        print("robot is already in mode 9 (a previous run did not exit cleanly?), "
              "cannot tell which mode to restore", file=sys.stderr)
        return 1
    print(f"old control mode: {old_mode}")

    # ---- Callback: GDK calls it once per frame and sends what it returns. ----
    state = {"frames": 0, "rc": 0, "prev_tau": None}
    n_soft = int(SOFT_START / DT)

    def callback(frame, _period_ns):
        req = gdk.JointTorqueControlReq()
        req.joint_names = list(ARMS)  # order of joint_efforts below
        req.finished = False
        cur = dyn.read(frame)
        if cur is None:
            print("frame misses an arm joint", file=sys.stderr)
            state["rc"] = 2
            req.joint_efforts = [0.0] * len(ARMS)
            req.finished = True
            return req
        q, dq, qf, vf = cur
        if state["prev_tau"] is None:  # the slew limit starts from the measured holding torque
            by_name = {s["name"]: s for s in frame["states"]}
            state["prev_tau"] = np.array([by_name[n]["effort"] for n in ARMS], float)
        # Soft start: ramp the spring-damper, the hold impedance and the wall in from 0.
        scale = min(1.0, (state["frames"] + 1) / n_soft)

        tau = dyn.bias(qf, vf)   # g + C dq of both arms
        # "cartesian" arms: tau += J_v^T F, F the clipped spring-damper force in base_link.
        cart_state = {}
        if cart:
            kin = dyn.tcp_kinematics(qf, cart)
        for side in cart:
            p, J_v = kin[side]
            v = J_v @ vf                  # TCP velocity in base_link (waist motion included)
            spring = K_SPRING * (target[side] - p)
            s_norm = float(np.linalg.norm(spring))
            if s_norm > MAX_FORCE:        # clip the spring first, so the damper is never cut
                spring *= MAX_FORCE / s_norm
            force = spring - D_DAMPER * v
            f_norm = float(np.linalg.norm(force))
            if f_norm > MAX_FORCE:        # then the output of spring + damper
                force *= MAX_FORCE / f_norm
            force *= scale
            tau[SIDES[side][2]] += J_v[:, dyn.cols[side]].T @ force
            cart_state[side] = (p, v, force)
        # "hold" arms: joint impedance towards the start pose.
        for side in hold:
            sl = SIDES[side][2]
            tau[sl] += scale * (KP[sl] * (q_start[sl] - q[sl]) - KD[sl] * dq[sl])
        # Joint limit wall, all 14 joints.
        wall, d_lo, d_hi = limit_wall(q, dq)
        tau += scale * wall
        # Safety: clamp to the URDF effort limit, and limit the change per frame.
        tau = np.clip(tau, -dyn.limit, dyn.limit)
        prev_tau = state["prev_tau"]
        tau = prev_tau + np.clip(tau - prev_tau, -MAX_STEP_NM, MAX_STEP_NM)
        state["prev_tau"] = tau
        req.joint_efforts = [float(x) for x in tau]

        # Guards. Distance to the limit predicted LIMIT_STOP_TIME ahead at the current speed.
        d_pred = np.minimum(d_lo + np.minimum(dq, 0.0) * LIMIT_STOP_TIME,
                            d_hi - np.maximum(dq, 0.0) * LIMIT_STOP_TIME)
        j = int(np.argmin(d_pred))
        vmax = float(np.max(np.abs(dq)))
        gaps = {}   # first frame only: TCP from the loop frame vs the start read
        if state["frames"] == 0:
            gaps = {s: float(np.linalg.norm(cart_state[s][0] - p_start[s])) for s in cart}
        mismatch = [f"{s} {g * 1000:.1f} mm" for s, g in gaps.items() if g > START_MISMATCH]
        hold_err = {s: float(np.max(np.abs(q_start[SIDES[s][2]] - q[SIDES[s][2]]))) for s in hold}
        pushed = [f"{s} {e:.3f} rad" for s, e in hold_err.items() if e > MAX_TRACK_ERR]
        abort, abort_rc = None, 3
        if mismatch:
            abort, abort_rc = (f"the first torque-loop frame puts the TCP away from the start "
                               f"read ({mismatch}; waist joints missing from one of them?)"), 1
        elif d_pred[j] < LIMIT_ABORT:
            abort = (f"{ARMS[j]} is {min(d_lo[j], d_hi[j]):.3f} rad from its limit at "
                     f"{dq[j]:+.2f} rad/s (pushed through the wall)")
        elif vmax > MAX_JOINT_VEL:
            abort = f"joint velocity {vmax:.2f} rad/s > {MAX_JOINT_VEL}"
        elif pushed:
            abort = f"held arm off its start pose ({pushed}, limit {MAX_TRACK_ERR} rad)"
        if abort:
            print(f"{abort}, abort", file=sys.stderr)
            state["rc"] = abort_rc
            req.finished = True
            return req

        if state["frames"] % 5000 == 0:
            parts = [f"t={state['frames'] * DT:.1f}s"]
            for side, (p, v, force) in cart_state.items():
                parts.append(f"{side}: |target-p|={np.linalg.norm(target[side] - p) * 1000:.1f} mm"
                             f" |v|={np.linalg.norm(v):.3f} m/s |F|={np.linalg.norm(force):.2f} N")
            dist = np.minimum(d_lo, d_hi)
            k = int(np.argmin(dist))
            parts.append(f"closest limit: {ARMS[k]} {dist[k]:.3f} rad")
            in_zone = [ARMS[i] for i in range(len(ARMS)) if dist[i] < LIMIT_ZONE]
            if in_zone:
                parts.append(f"wall active on {in_zone}")
            print(" | ".join(parts), flush=True)

        state["frames"] += 1
        req.finished = stop_requested  # runs until Ctrl+C / SIGTERM, no time limit
        return req

    # ---- Enter mode 9 and wait until the MC reports it. ----
    rc = 0
    try:
        robot.set_control_mode(MODE_JOINT_TORQUE)
        robot.wait_control_mode(MODE_JOINT_TORQUE, 500)
        print("running: push the arms around; Ctrl+C to stop. Cartesian arms: only the TCP "
              "position springs back, the wrist and elbow stay where you leave them.", flush=True)
        robot.joint_torque_control(callback, state_timeout_ms=100)
        # The stream is already ended here, no joint_torque_end() needed.
    except (TimeoutError, ValueError, RuntimeError) as e:  # GDK read/write/mode failures
        print(f"torque loop failed: {e}", file=sys.stderr)
        rc = 2
    rc = state["rc"] or rc

    # ---- Cleanup: leave mode 9 ourselves on every exit path; the GDK never does it. ----
    try:
        robot.set_control_mode(old_mode)
        print(f"control mode restored to {old_mode}")
    except (TimeoutError, ValueError, RuntimeError) as e:
        print(f"set_control_mode({old_mode}) failed, restore it manually (after an estop, clear "
              f"the MC error first): {e}", file=sys.stderr)
        rc = rc or 2
    print(f"done, rc={rc}")
    return rc


def main():
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    gdk.gdk_init()
    try:
        return run(gdk.Robot(), sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URDF)
    finally:
        gdk.gdk_release()


if __name__ == "__main__":
    sys.exit(main())
