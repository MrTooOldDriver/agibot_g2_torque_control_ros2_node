#!/usr/bin/env python3
"""Gripper insertion task on a Cartesian spring-damper triangle, with GDK direct torque control.

Extends cartesian_torque_control_demo.py, which must sit in the same directory together with
joint_torque_demo.py. It imports that script's dynamics model (TCP kinematics) and joint limit
reading, follows its Cartesian spring-damper, joint limit wall, guards and mode-9 lifecycle
(remember the control mode, switch to mode 9, run robot.joint_torque_control(callback), restore
the mode), and adds the gripper, a triangle of virtual points that holds the gripper orientation,
and a task sequence.

Each arm runs in the mode set in ARM_MODE:
  "insert"  The task, all inside one torque stream (the arm is never left at zero torque):
              1. open   The gripper is held at its start pose and opens. With LEVEL_FINGERS it also
                        rolls until the finger line is horizontal.
              2. close  The gripper closes on the part: after Enter (WAIT_FOR_ENTER) or after
                        OPEN_WAIT s.
              3. move   After Enter (or as soon as the gripper has settled), the triangle target
                        follows a minimum-jerk path from the start pose to the end pose in
                        MOVE_TIME s. With END_ROTATION = 0 it only translates, so the gripper keeps
                        its orientation.
              4. hold   The gripper is held at the end pose, closed, until Ctrl+C.
            The start pose is where the gripper is when the script starts. The end pose is
            END_OFFSET from it (or END_POSITION), turned by END_ROTATION.
  "hold"    As in cartesian_torque_control_demo.py: joint impedance at the start pose. Its gripper
            is never commanded.
Start with the left arm "insert" and the right arm "hold".

Triangle of virtual points (flange = arm_<l|r>_end_link, before the gripper; its z axis points
along the gripper, its x axis is the line the two fingers open along):
  Three virtual links of length VIRTUAL_LINK_LENGTH leave the flange origin. Their tips form an
  equilateral triangle with side TRIANGLE_SIDE, in a plane across the gripper axis and centred on
  it, sqrt(VIRTUAL_LINK_LENGTH^2 - TRIANGLE_SIDE^2 / 3) from the flange. Two tips lie on a line
  parallel to the fingers, the third on the flange +y side. The tips move with the gripper, and
  each has a target point. Three points that are not on one line fix both the position and the
  orientation of the gripper, so the triangle holds the orientation as well: moving the three
  targets down together moves the gripper down without turning it.
  The pull on the tips is split in two parts:
    centre  c = mean of the tips, c_d its target (the start pose, the path, then the end pose):
              F_s = K (c_d - c),        |F_s| clipped to MAX_FORCE
              F_c = F_s + D (v_d - v),  |F_c| clipped to MAX_FORCE      shared by the three tips
            This is the single-point law of the earlier version, applied at the triangle centre.
    shape   a spring-damper per tip on its offset from the centre, d_i = p_i - c, towards the
            target offset d_i* = R_d (tip_i - centre) of the target orientation R_d:
              S_i = K_TIP (d_i* - d_i)
              T_i = S_i + D_TIP (w_d x d_i* - (v_i - v_c))
            Each of S and T is scaled by one common factor so that every |S_i|, |T_i| <=
            MAX_TIP_FORCE. The three T_i sum to zero: they turn the gripper, never push it, and
            keep turning it back even when F_c is clipped, as in a jammed insertion.
    tau = sum_i J_i^T (F_c / 3 + T_i) + rnea(q, dq, 0)      J_i: 3 x 7 Jacobian of tip i
  Equivalent stiffness: translation K_SPRING; orientation K_TIP * side^2 about the gripper axis
  (roll) and K_TIP * side^2 / 2 about the two axes across it (tilt), with damping D_TIP * side^2
  and D_TIP * side^2 / 2. The tip forces saturate beyond an orientation error of
  MAX_TIP_FORCE / (K_TIP * r), r = side / sqrt(3), and the turning moment then stays at its
  largest: 3 r MAX_TIP_FORCE in roll, about 1.5 r MAX_TIP_FORCE in tilt. So:
    TRIANGLE_SIDE sets how hard the orientation is held.
    VIRTUAL_LINK_LENGTH does not stiffen the orientation: it moves the triangle centre, which is
    the centre of compliance, along the gripper axis. A push at the centre moves the gripper
    without turning it; with the defaults the centre sits between the fingertips.
  The startup check refuses tips whose joint torque could beat the joint limit wall.
  The elbow (the arm's 7th degree of freedom) still floats on the gravity compensation. Joint
  friction leaves a steady-state error of about friction / stiffness.

Finger line: by default the triangle holds the start orientation, and the script prints how far
the finger line tilts from horizontal (a warning above TILT_WARN deg). With LEVEL_FINGERS = True it
rolls the gripper about its own axis during the open step, over LEVEL_TIME s, until the finger line
is horizontal, and holds that orientation from then on (refused above MAX_LEVEL_ANGLE).

Not modelled: the grasped part's weight. At K_SPRING = 200 N/m, each 100 g lowers the gripper by
about 5 mm. Its moment about the triangle centre turns the gripper by moment / tilt stiffness.

Gripper (omnipicker, one joint per side, travel [-0.785, 0] rad: -0.785 open, 0 closed). In mode 9
the gripper is commanded in the torque frame itself (end_effector_joint_names and
end_effector_joint_positions, target positions), here every GRIPPER_SEND_EVERY frames. The robot
keeps the last target in the frames without one. Before mode 9 the script reads the gripper joint
name and position with robot.get_end_state() and refuses to start unless each "insert" arm has one
joint inside the omnipicker travel. The target ramps at GRIPPER_SPEED from the measured position,
so the first command does not jump. Every target is clamped to the travel: the GDK rejects a frame
with an invalid gripper target as a whole, arm torques included, and the torque loop then ends
(rc 2, the mode is restored). There is no gripper feedback inside the 1 kHz loop
(get_end_state() is too slow to call there), so each gripper step lasts the ramp plus
GRIPPER_SETTLE s.

Joint limits and safety: as in cartesian_torque_control_demo.py (limit wall and stop on all 14 arm
joints, joint speed guard, start mismatch guard, held-arm guard). At startup the script also
checks that the wall beats the largest torque the clipped forces can make, and refuses an end pose
that is too far, out of reach, or reached too fast.

On every stop (Ctrl+C, SIGTERM, a safety abort or an error) the script sends a last frame, ending
the torque stream, and restores the previous control mode, which holds the arms where they are.
The gripper keeps its last target, so a closed gripper stays closed on the part.

Run from this directory, in a terminal (Enter needs a tty unless WAIT_FOR_ENTER = False). Edit
END_OFFSET for your task first. The robot moves, so keep the area around it clear:
  python3 cartesian_insertion_demo.py [urdf_path]
Without urdf_path it uses the default of joint_torque_demo.py (genie_g2_crs_omnipicker.urdf).
"""

import os
import select
import signal
import sys
import threading

import agibot_gdk as gdk
import numpy as np
import pinocchio as pin

from cartesian_torque_control_demo import (LIMIT_ABORT, LIMIT_D_SCALE, LIMIT_K_SCALE,
                                           LIMIT_STOP_TIME, LIMIT_ZONE, MAX_JOINT_VEL,
                                           REACH_MARGIN, SIDES, START_MISMATCH,
                                           CartesianArmDynamics, fmt, joint_limits)
from joint_torque_demo import (ARMS, DEFAULT_URDF, DT, KD, KP, MAX_STEP_NM, MAX_TRACK_ERR,
                               MODE_JOINT_TORQUE, SOFT_START, URDF_DIR, quintic)

# ---- What each arm does: "insert" (the task) or "hold" (start pose, gripper untouched). ----
ARM_MODE = {"left": "insert", "right": "hold"}

# ---- Task: edit END_OFFSET (or END_POSITION) for your insertion before the first run. ----
# End pose: an offset (m) of the triangle centre in base_link axes (x forward, y left, z up) from
# its start, or the absolute flange (arm_<l|r>_end_link) position [x, y, z] in base_link (m) in
# END_POSITION[side]. Mirror a y offset for the right arm.
END_OFFSET = {"left": np.array([0.0, 0.0, -0.10]),     # 5 cm straight down
              "right": np.array([0.0, 0.0, -0.10])}
END_POSITION = {"left": None, "right": None}
# Turn of the gripper from the start to the end pose, about the triangle centre: a rotation vector
# (rad) in base_link axes. 0 keeps the orientation.
END_ROTATION = {"left": np.zeros(3), "right": np.zeros(3)}
MOVE_TIME = 4.0        # s, minimum-jerk move from the start to the end pose
WAIT_FOR_ENTER = True  # True: Enter starts the close and the move; False: timed steps
OPEN_WAIT = 2.0        # s, WAIT_FOR_ENTER = False only: open gripper time before it closes

# ---- Triangle of virtual points: holds the gripper position and orientation. ----
VIRTUAL_LINK_LENGTH = 0.15   # m, flange origin to each tip; at least TRIANGLE_SIDE / sqrt(3).
                             # Places the triangle centre along the gripper axis (0.096 m with
                             # these values, between the fingertips); no effect on stiffness.
TRIANGLE_SIDE = 0.20         # m, distance between any two tips; orientation stiffness ~ side^2

# ---- Finger line. ----
LEVEL_FINGERS = False  # True: roll the gripper at start until the finger line is horizontal
LEVEL_TIME = 2.0       # s, duration of that roll (during the open step)
MAX_LEVEL_ANGLE = 0.5  # rad, refuse a larger levelling roll
LEVEL_SKIP = 0.02      # rad, no levelling roll when the finger line tilts less than this
TILT_WARN = 5.0        # deg, warn when the finger line tilts more than this (no levelling)

# ---- Gripper (omnipicker). ----
GRIPPER_OPEN = -0.785        # rad, target of the open step
GRIPPER_CLOSE = 0.0          # rad, target of the close step (fully closed: it stops on the part)
GRIPPER_RANGE = (-0.785, 0.0)  # omnipicker travel (GDK docs); every target is clamped to it
GRIPPER_SPEED = 0.5          # rad/s, ramp of the commanded target (full travel in 1.6 s)
GRIPPER_SETTLE = 1.0         # s, wait after each ramp: the gripper lags its target
GRIPPER_SEND_EVERY = 10      # frames between gripper targets in the torque stream
GRIPPER_TOL = 0.05           # rad, refuse a measured start position this far outside the travel
# Gripper joint of the omnipicker in the URDF. get_end_state() gives the name the robot uses; a
# different name only gives a warning.
EXPECTED_GRIPPER = {"left": "idx31_gripper_l_inner_joint1",
                    "right": "idx71_gripper_r_inner_joint1"}

# ---- Spring-dampers. Triangle centre (translation): ----
K_SPRING = 200.0   # N/m, translational stiffness
D_DAMPER = 30.0    # N*s/m, translational damping (on the velocity error)
MAX_FORCE = 15.0   # N, limit on the total force, also the most the gripper pushes on the part
# ---- Triangle shape (orientation), per tip: ----
K_TIP = 1000.0       # N/m; with TRIANGLE_SIDE 0.2: roll 40 Nm/rad, tilt 20 Nm/rad
D_TIP = 40.0         # N*s/m; with TRIANGLE_SIDE 0.2: roll 1.6 Nm*s/rad, tilt 0.8 Nm*s/rad
MAX_TIP_FORCE = 6.0  # N, limit on each tip's turning force; with TRIANGLE_SIDE 0.2 the largest
                     # moment is 2.1 Nm in roll, 1.0 Nm in tilt, reached beyond 3 deg of error

# ---- Other safety. ----
MAX_END_OFFSET = 0.30    # m, refuse an end pose farther than this from the start
MAX_END_ROTATION = 0.5   # rad, refuse a larger END_ROTATION
MAX_TCP_SPEED = 0.10     # m/s, refuse a move whose peak reference speed is higher
MAX_ROT_SPEED = 0.5      # rad/s, the same for the rotation (end rotation and levelling)
STATUS_EVERY = 2000      # frames between status lines (also printed at every step)

MODES = ("insert", "hold")
FINGER_AXIS = np.array([1.0, 0.0, 0.0])    # flange axis the omnipicker fingers open along (URDF)
GRIPPER_AXIS = np.array([0.0, 0.0, 1.0])   # flange axis pointing along the gripper (URDF)

stop_requested = False


def on_signal(_signum, _frame):
    global stop_requested  # Ctrl+C / SIGTERM: end the torque stream on the next frame
    stop_requested = True


class InsertionArmDynamics(CartesianArmDynamics):
    """CartesianArmDynamics plus the flange rotation and the full 6 x nv flange Jacobian."""

    def tcp_pose(self, qf, sides):
        """{side: (p, R, J)}: flange origin and rotation in base_link, and its 6 x nv Jacobian.

        LOCAL_WORLD_ALIGNED: J[:3] vf is the velocity of the flange origin and J[3:] vf the
        angular velocity, both in base_link axes. A point at offset d (base_link axes) from the
        flange origin then moves at J[:3] vf + (J[3:] vf) x d, and a force F there acts on the
        joints as J[:3]^T F + J[3:]^T (d x F).
        """
        pin.computeJointJacobians(self.model, self.kin, qf)   # also runs forward kinematics
        out = {}
        for side in sides:
            fid = self.tcp[side]
            pin.updateFramePlacement(self.model, self.kin, fid)
            J = pin.getFrameJacobian(self.model, self.kin, fid, pin.LOCAL_WORLD_ALIGNED)
            pose = self.kin.oMf[fid]
            out[side] = (pose.translation.copy(), pose.rotation.copy(), J.copy())
        return out


def triangle_tips():
    """3 x 3: the tips in the flange frame, rows 0 and 1 on a line parallel to the fingers."""
    radius = TRIANGLE_SIDE / np.sqrt(3.0)           # centre to each tip
    depth = np.sqrt(VIRTUAL_LINK_LENGTH**2 - radius**2)
    return np.array([[TRIANGLE_SIDE / 2, -radius / 2, depth],
                     [-TRIANGLE_SIDE / 2, -radius / 2, depth],
                     [0.0, radius, depth]])


def clip_norm(v, limit):
    """v scaled down to |v| <= limit, direction kept."""
    n = float(np.linalg.norm(v))
    return v * (limit / n) if n > limit else v


def clip_rows(a, limit):
    """Rows of a scaled down by one common factor so that every row norm is <= limit."""
    n = float(np.max(np.linalg.norm(a, axis=1)))
    return a * (limit / n) if n > limit else a


def rot_angle(R):
    """Angle (rad) of a rotation matrix."""
    return float(np.linalg.norm(pin.log3(R)))


def tilt(R):
    """Angle (rad) of the finger line from horizontal, for flange rotation R in base_link."""
    return float(np.arcsin(np.clip((R @ FINGER_AXIS)[2], -1.0, 1.0)))


def ramp(start, target, t):
    """Gripper target t seconds into a ramp from start to target at GRIPPER_SPEED."""
    d, reach = target - start, GRIPPER_SPEED * t
    return target if abs(d) <= reach else start + np.sign(d) * reach


def clamp_gripper(x):
    return float(min(max(x, GRIPPER_RANGE[0]), GRIPPER_RANGE[1]))


def read_grippers(robot, sides):
    """{side: (joint name, measured position)} of the grippers, from robot.get_end_state().

    Raises RuntimeError when the end effector of a side is not a one-joint gripper with a finite
    position inside the omnipicker travel.
    """
    try:
        end = robot.get_end_state()
    except (AttributeError, RuntimeError, TimeoutError, ValueError) as e:
        raise RuntimeError(f"robot.get_end_state() failed ({e!r})") from e
    out = {}
    for side in sides:
        st = end.get(f"{side}_end_state") if isinstance(end, dict) else None
        if not st:
            raise RuntimeError(f"get_end_state() has no {side}_end_state")
        names = list(st.get("names") or [])
        positions = [m.get("position") for m in (st.get("end_states") or [])]
        print(f"{side} gripper: type={st.get('type')} controlled={st.get('controlled')} "
              f"names={names} positions={positions}")
        if len(names) != 1 or len(positions) != 1:
            raise RuntimeError(f"the {side} end effector reports {len(names)} joint names and "
                               f"{len(positions)} joint states; this script drives a one-joint "
                               "gripper (omnipicker)")
        try:
            pos = float(positions[0])
        except (TypeError, ValueError):
            pos = float("nan")
        lo, hi = GRIPPER_RANGE
        if not np.isfinite(pos) or not lo - GRIPPER_TOL <= pos <= hi + GRIPPER_TOL:
            raise RuntimeError(f"the {side} gripper is at {pos}, outside the omnipicker travel "
                               f"[{lo}, {hi}]: a different gripper type?")
        if names[0] != EXPECTED_GRIPPER[side]:
            print(f"warning: the {side} gripper joint is {names[0]}, not "
                  f"{EXPECTED_GRIPPER[side]}; check that it is an omnipicker", file=sys.stderr)
        out[side] = (names[0], pos)
    return out


def read_enter(enter, done):
    """Set `enter` on every line typed in the terminal. The torque loop never waits for input.

    Polls the raw fd (select + os.read), as zero_torque_joint_check.py does, so this daemon thread
    never holds the stdin buffer lock when the interpreter exits.
    """
    try:
        fd = sys.stdin.fileno()
    except (AttributeError, ValueError, OSError):
        return
    while not stop_requested and not done.is_set():
        if not select.select([fd], [], [], 0.2)[0]:
            continue
        if not os.read(fd, 1024):  # EOF
            return
        enter.set()


def restore_mode(robot, mode):
    """Switch back to `mode` and confirm it; True when the robot reports it."""
    try:
        robot.set_control_mode(mode)
        robot.wait_control_mode(mode, 500)
        now = robot.get_motion_control_status().control_mode
    except (TimeoutError, ValueError, RuntimeError) as e:
        print(f"set_control_mode({mode}) failed, restore it manually (after an estop, clear "
              f"the MC error first): {e}", file=sys.stderr)
        return False
    if now != mode:
        print(f"the robot reports control mode {now} after set_control_mode({mode}), restore "
              "it manually", file=sys.stderr)
        return False
    print(f"control mode restored to {mode}")
    return True


def check_settings(task):
    """Problems with the settings above, as messages (empty if none)."""
    problems = []
    if not task:
        problems.append("no arm is 'insert' in ARM_MODE")
    lo, hi = GRIPPER_RANGE
    for name, value in (("GRIPPER_OPEN", GRIPPER_OPEN), ("GRIPPER_CLOSE", GRIPPER_CLOSE)):
        if not lo <= value <= hi:
            problems.append(f"{name}={value} is outside the omnipicker travel [{lo}, {hi}]")
    if GRIPPER_SPEED <= 0 or GRIPPER_SETTLE < 0 or GRIPPER_SEND_EVERY < 1:
        problems.append("need GRIPPER_SPEED > 0, GRIPPER_SETTLE >= 0, GRIPPER_SEND_EVERY >= 1")
    if min(K_SPRING, D_DAMPER, MAX_FORCE, K_TIP, D_TIP, MAX_TIP_FORCE) < 0:
        problems.append("gains and limits must be >= 0")
    if MOVE_TIME <= 0 or OPEN_WAIT < 0 or LEVEL_TIME <= 0:
        problems.append("need MOVE_TIME > 0, LEVEL_TIME > 0 and OPEN_WAIT >= 0")
    if TRIANGLE_SIDE <= 0 or VIRTUAL_LINK_LENGTH < TRIANGLE_SIDE / np.sqrt(3.0):
        problems.append(f"need TRIANGLE_SIDE > 0 and VIRTUAL_LINK_LENGTH >= TRIANGLE_SIDE / "
                        f"sqrt(3) = {TRIANGLE_SIDE / np.sqrt(3.0):.3f} m (the links must reach "
                        "the corners of the triangle)")
    if K_TIP == 0 or MAX_TIP_FORCE == 0:
        problems.append("K_TIP and MAX_TIP_FORCE must be > 0, or the orientation is not held")
    if WAIT_FOR_ENTER and not sys.stdin.isatty():
        problems.append("WAIT_FOR_ENTER needs a terminal (stdin is not a tty): run it in a "
                        "terminal, or set WAIT_FOR_ENTER = False for timed steps")
    return problems


def run(robot, urdf):
    if set(ARM_MODE) != set(SIDES) or any(m not in MODES for m in ARM_MODE.values()):
        print(f"ARM_MODE must map 'left' and 'right' to one of {MODES}", file=sys.stderr)
        return 1
    task = [s for s in SIDES if ARM_MODE[s] == "insert"]
    hold = [s for s in SIDES if ARM_MODE[s] == "hold"]
    problems = check_settings(task)
    if problems:
        for p in problems:
            print(p, file=sys.stderr)
        return 1
    print(f"arm modes: left {ARM_MODE['left']}, right {ARM_MODE['right']}")

    # ---- Triangle of virtual points, in the flange frame. ----
    tips_local = triangle_tips()
    centre_local = tips_local.mean(axis=0)        # on the gripper axis, `depth` from the flange
    shape_local = tips_local - centre_local       # tip offsets from the centre (sum to 0)
    depth, radius = float(centre_local[2]), TRIANGLE_SIDE / np.sqrt(3.0)
    side2 = TRIANGLE_SIDE**2
    # Largest turning moment: 3 r cap in roll (all tips at the cap); in tilt the common factor
    # leaves the tips at cap, cap/2, cap/2, about 1.5 r cap. The roll value bounds the wall check.
    moment_max = 3.0 * radius * MAX_TIP_FORCE
    saturation = MAX_TIP_FORCE / (K_TIP * radius)  # rad of orientation error
    print(f"triangle: links {VIRTUAL_LINK_LENGTH} m, side {TRIANGLE_SIDE} m, centre {depth:.3f} m "
          "along the gripper axis; tips in the flange frame (m): "
          + ", ".join(fmt(t) for t in tips_local))
    print(f"stiffness: translation K={K_SPRING} N/m, D={D_DAMPER} N*s/m, max |F|={MAX_FORCE} N; "
          f"orientation roll {K_TIP * side2:.1f} / tilt {K_TIP * side2 / 2:.1f} Nm/rad, damping "
          f"roll {D_TIP * side2:.2f} / tilt {D_TIP * side2 / 2:.2f} Nm*s/rad, largest moment "
          f"roll {moment_max:.2f} / tilt {moment_max / 2:.2f} Nm beyond "
          f"{np.degrees(saturation):.1f} deg of error")

    # ---- Dynamics model (Pinocchio wrapper). Needs a URDF that matches the robot. ----
    if not urdf:
        name = robot.get_whole_body_status().get("urdf_type", "")  # the binding returns a dict
        if not name:
            print("urdf_type unavailable, pass the urdf path as argument", file=sys.stderr)
            return 1
        urdf = f"{URDF_DIR}/{name}/{name}.urdf"
    print(f"urdf: {urdf}")
    dyn = InsertionArmDynamics(urdf)

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
              "as 0, so the gripper pose and the end pose are off if the waist is not at 0",
              file=sys.stderr)

    # ---- Joint limits and the wall. ----
    q_lo, q_hi = joint_limits(robot, dyn)
    k_wall, d_wall = LIMIT_K_SCALE * KP, LIMIT_D_SCALE * KD
    if not 0.0 < LIMIT_ABORT < LIMIT_ZONE or np.any(q_hi - q_lo <= 2 * LIMIT_ZONE):
        print("need 0 < LIMIT_ABORT < LIMIT_ZONE < half of every joint range", file=sys.stderr)
        return 1
    # The wall at the stop depth must beat the largest torque the clipped forces can make. The
    # centre force acts at most (lever + depth) from a joint; the tip forces make a pure moment,
    # and |J_w column| = 1 for a revolute joint.
    for side in task:
        sl = SIDES[side][2]
        wall_max = k_wall[sl] * (LIMIT_ZONE - LIMIT_ABORT)
        push_max = MAX_FORCE * (dyn.lever[sl] + depth) + moment_max
        weak = [f"{ARMS[sl.start + k]} ({wall_max[k]:.1f} < {push_max[k]:.1f} Nm)"
                for k in range(7) if wall_max[k] < push_max[k]]
        if weak:
            print(f"the limit wall is weaker than the largest triangle torque on {weak}: lower "
                  "MAX_FORCE, MAX_TIP_FORCE, TRIANGLE_SIDE or VIRTUAL_LINK_LENGTH, or raise "
                  "LIMIT_K_SCALE", file=sys.stderr)
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
        """(tau, d_lo, d_hi): wall torque near the limits, distances to the lower/upper limit."""
        d_lo, d_hi = q - q_lo, q_hi - q
        depth_lo = np.maximum(LIMIT_ZONE - d_lo, 0.0)
        depth_hi = np.maximum(LIMIT_ZONE - d_hi, 0.0)
        inside = (depth_lo + depth_hi) > 0.0
        return k_wall * (depth_lo - depth_hi) - inside * d_wall * dq, d_lo, d_hi

    # ---- Grippers of the "insert" arms: joint name and position, before mode 9. ----
    try:
        grippers = read_grippers(robot, task)
    except RuntimeError as e:
        print(f"{e}; not starting", file=sys.stderr)
        return 1
    g_start = {s: clamp_gripper(grippers[s][1]) for s in task}  # first ramp starts here

    # ---- Start, levelling and end poses of the "insert" arms. ----
    # c: triangle centre in base_link. R_start: flange rotation at start. level: rotation vector
    # of the levelling roll (zero without). R_base = exp(level) R_start, the orientation held
    # after the open step. c_end, R_end: the end pose.
    c_start, tips_start, R_start, level, R_base, c_end, rotvec = {}, {}, {}, {}, {}, {}, {}
    kin = dyn.tcp_pose(qf_start, task)
    for side in task:
        p_f, R0, _ = kin[side]
        R_start[side] = R0
        c_start[side] = p_f + R0 @ centre_local
        tips_start[side] = p_f + tips_local @ R0.T
        # Levelling roll about the gripper axis a: the finger line u turns to u cos(t) + w sin(t)
        # with w = a x u, horizontal when tan(t) = -u_z / w_z; the smaller of the two solutions.
        u, w, a = R0 @ FINGER_AXIS, R0 @ np.cross(GRIPPER_AXIS, FINGER_AXIS), R0 @ GRIPPER_AXIS
        tilt0 = tilt(R0)
        theta = 0.0
        if LEVEL_FINGERS and abs(tilt0) >= LEVEL_SKIP:
            theta = float(np.arctan2(-u[2], w[2]))
            if theta > np.pi / 2:
                theta -= np.pi
            elif theta < -np.pi / 2:
                theta += np.pi
        level[side] = theta * a
        R_base[side] = pin.exp3(level[side]) @ R0
        print(f"{side} finger line: {np.degrees(tilt0):+.1f} deg from horizontal"
              + (f", levelling roll {np.degrees(theta):+.1f} deg about the gripper axis"
                 if LEVEL_FINGERS else ""))
        if abs(theta) > MAX_LEVEL_ANGLE:
            print(f"{side} levelling roll {theta:.3f} rad is more than MAX_LEVEL_ANGLE="
                  f"{MAX_LEVEL_ANGLE}: turn the gripper closer to level first", file=sys.stderr)
            return 1
        if 1.875 * abs(theta) / LEVEL_TIME > MAX_ROT_SPEED:
            print(f"{side} levelling roll is faster than {MAX_ROT_SPEED} rad/s: raise LEVEL_TIME",
                  file=sys.stderr)
            return 1
        if not LEVEL_FINGERS and abs(np.degrees(tilt0)) > TILT_WARN:
            print(f"warning: the {side} finger line tilts {np.degrees(tilt0):+.1f} deg; the "
                  "triangle holds that tilt (LEVEL_FINGERS = True levels it)", file=sys.stderr)
        rotvec[side] = np.asarray(END_ROTATION[side], float)
        R_end = pin.exp3(rotvec[side]) @ R_base[side]
        if END_POSITION[side] is not None:   # flange position -> triangle centre
            c_end[side] = np.array(END_POSITION[side], float) + R_end @ centre_local
        else:
            c_end[side] = c_start[side] + np.asarray(END_OFFSET[side], float)
        dist = float(np.linalg.norm(c_end[side] - c_start[side]))
        angle = float(np.linalg.norm(rotvec[side]))
        print(f"{side} triangle centre in base_link: start {fmt(c_start[side])} m, end "
              f"{fmt(c_end[side])} m ({dist * 1000:.0f} mm), end rotation {fmt(rotvec[side])} rad")
        if dist > MAX_END_OFFSET or angle > MAX_END_ROTATION:
            print(f"{side} end pose is {dist:.3f} m and {angle:.3f} rad from the start, more than "
                  f"{MAX_END_OFFSET} m or {MAX_END_ROTATION} rad", file=sys.stderr)
            return 1
        # Minimum jerk: the peak speed is 1.875 x distance / time.
        v_peak, w_peak = 1.875 * dist / MOVE_TIME, 1.875 * angle / MOVE_TIME
        if v_peak > MAX_TCP_SPEED or w_peak > MAX_ROT_SPEED:
            print(f"{side} move peaks at {v_peak:.3f} m/s and {w_peak:.3f} rad/s, faster than "
                  f"{MAX_TCP_SPEED} m/s or {MAX_ROT_SPEED} rad/s: raise MOVE_TIME",
                  file=sys.stderr)
            return 1
        # Rough reach check on the flange, as in cartesian_torque_control_demo.py.
        shoulder, reach = dyn.shoulder_and_reach(qf_start, side)
        flange_start = c_start[side] - R0 @ centre_local
        d_end = np.linalg.norm(c_end[side] - R_end @ centre_local - shoulder)
        if d_end > max(REACH_MARGIN * reach, np.linalg.norm(flange_start - shoulder)):
            print(f"{side} end pose puts the flange {d_end:.3f} m from the shoulder, out of reach "
                  f"(arm length {reach:.3f} m x {REACH_MARGIN})", file=sys.stderr)
            return 1
    print(f"gripper: open {GRIPPER_OPEN}, close {GRIPPER_CLOSE} rad at {GRIPPER_SPEED} rad/s, "
          f"settle {GRIPPER_SETTLE} s; move {MOVE_TIME} s; "
          + ("Enter starts the close and the move" if WAIT_FOR_ENTER else
             f"timed steps (open {OPEN_WAIT} s)"))

    # ---- Remember the control mode so it can be restored at the end. ----
    old_mode = robot.get_motion_control_status().control_mode
    if old_mode == MODE_JOINT_TORQUE:
        print("robot is already in mode 9 (a previous run did not exit cleanly?), "
              "cannot tell which mode to restore", file=sys.stderr)
        return 1
    print(f"old control mode: {old_mode}")

    # ---- Task sequence. Steps advance on time (and Enter), never on gripper feedback. ----
    levelling = any(np.any(level[s] != 0) for s in task)
    t_open = max(SOFT_START + (LEVEL_TIME if levelling else 0.0),
                 max(abs(GRIPPER_OPEN - g_start[s]) for s in task) / GRIPPER_SPEED
                 + GRIPPER_SETTLE)
    t_close = abs(GRIPPER_CLOSE - GRIPPER_OPEN) / GRIPPER_SPEED + GRIPPER_SETTLE
    enter, loop_done = threading.Event(), threading.Event()
    state = {"frames": 0, "t": 0.0, "rc": 0, "prev_tau": None,
             "step": "open", "t_step": 0.0, "status_now": True}

    def go(step, t):
        """Enter a step of the sequence at time t and say so."""
        state["step"], state["t_step"], state["status_now"] = step, t, True
        enter.clear()   # an Enter typed before the prompt does not count
        if step == "wait_close":
            msg = ("gripper open: put the part between the fingers, then press Enter to close"
                   if WAIT_FOR_ENTER else f"gripper open, closing in {OPEN_WAIT:g} s")
        elif step == "close":
            msg = "closing the gripper"
        elif step == "wait_move":
            msg = ("gripper closed: press Enter to move to the end pose" if WAIT_FOR_ENTER
                   else "gripper closed")
        elif step == "move":
            msg = f"moving to the end pose in {MOVE_TIME:g} s"
        else:
            msg = "move done, holding the end pose; Ctrl+C to stop (the gripper stays closed)"
        print(f"[t={t:.1f}s] {msg}", flush=True)

    def advance(t):
        """Next step once the current one is over. One transition per frame at most."""
        step, tau = state["step"], t - state["t_step"]
        if step == "open" and tau >= t_open:
            go("wait_close", t)
        elif step == "wait_close" and (enter.is_set() if WAIT_FOR_ENTER else tau >= OPEN_WAIT):
            go("close", t)
        elif step == "close" and tau >= t_close:
            go("wait_move", t)
        elif step == "wait_move" and (enter.is_set() or not WAIT_FOR_ENTER):
            go("move", t)
        elif step == "move" and tau >= MOVE_TIME:
            go("done", t)

    def gripper_target(side, step, tau):
        if step == "open":
            target = ramp(g_start[side], GRIPPER_OPEN, tau)
        elif step == "wait_close":
            target = GRIPPER_OPEN
        elif step == "close":
            target = ramp(GRIPPER_OPEN, GRIPPER_CLOSE, tau)
        else:
            target = GRIPPER_CLOSE
        return clamp_gripper(target)

    zero3 = np.zeros(3)

    def reference(side, step, tau):
        """(c_d, v_d, R_d, w_d): target of the triangle centre and of the flange orientation.

        Open step: the start pose, with the levelling roll after the soft start. Then the start
        pose (levelled), the minimum-jerk path, and the end pose.
        """
        if step == "open":
            s, ds, _ = quintic(0.0, 1.0, LEVEL_TIME, tau - SOFT_START)
            return c_start[side], zero3, pin.exp3(s * level[side]) @ R_start[side], ds * level[side]
        if step == "move":
            s, ds, _ = quintic(0.0, 1.0, MOVE_TIME, tau)
        elif step == "done":
            s, ds = 1.0, 0.0
        else:
            s, ds = 0.0, 0.0
        dc = c_end[side] - c_start[side]
        return (c_start[side] + s * dc, ds * dc, pin.exp3(s * rotvec[side]) @ R_base[side],
                ds * rotvec[side])

    # ---- Callback: GDK calls it once per frame and sends what it returns. ----
    def callback(frame, period_ns):
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
        state["t"] += period_ns * 1e-9   # robot time: 0 on the first frame
        t = state["t"]
        advance(t)
        step, tau_step = state["step"], t - state["t_step"]
        # Soft start: ramp the spring-dampers, the hold impedance and the wall in from 0.
        scale = min(1.0, (t + DT) / SOFT_START)

        tau = dyn.bias(qf, vf)   # g + C dq of both arms
        # "insert" arms: the triangle. Centre force shared by the tips, plus the tips' turning
        # forces; summed into one wrench at the flange origin.
        task_state = {}
        kin = dyn.tcp_pose(qf, task)
        for side in task:
            p_f, R, J = kin[side]
            cols = dyn.cols[side]
            twist = J @ vf                       # flange velocity and angular velocity
            off = tips_local @ R.T               # tip offsets from the flange, base_link axes
            tips = p_f + off
            tips_v = twist[:3] + np.cross(twist[3:], off)
            c, v_c = tips.mean(axis=0), tips_v.mean(axis=0)
            c_d, v_d, R_d, w_d = reference(side, step, tau_step)
            # Centre: the single-point law. Spring first, so the damper is never cut.
            spring = clip_norm(K_SPRING * (c_d - c), MAX_FORCE)
            f_c = scale * clip_norm(spring + D_DAMPER * (v_d - v_c), MAX_FORCE)
            # Shape: each tip's offset from the centre towards its target offset. Rows sum to 0.
            d_tips, d_target = tips - c, shape_local @ R_d.T
            spring_t = clip_rows(K_TIP * (d_target - d_tips), MAX_TIP_FORCE)
            f_t = scale * clip_rows(spring_t + D_TIP * (np.cross(w_d, d_target) - (tips_v - v_c)),
                                    MAX_TIP_FORCE)
            forces = f_c / 3.0 + f_t             # force on each tip
            tau[SIDES[side][2]] += (J[:3, cols].T @ forces.sum(axis=0)
                                    + J[3:, cols].T @ np.cross(off, forces).sum(axis=0))
            task_state[side] = (tips, c, c_d, R, R_d, f_c, np.cross(d_tips, f_t).sum(axis=0))
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

        # Gripper targets of the "insert" arms, every GRIPPER_SEND_EVERY frames.
        grip = {s: gripper_target(s, step, tau_step) for s in task}
        if state["frames"] % GRIPPER_SEND_EVERY == 0:
            req.end_effector_joint_names = [grippers[s][0] for s in task]
            req.end_effector_joint_positions = [grip[s] for s in task]

        # Guards. Distance to the limit predicted LIMIT_STOP_TIME ahead at the current speed.
        d_pred = np.minimum(d_lo + np.minimum(dq, 0.0) * LIMIT_STOP_TIME,
                            d_hi - np.maximum(dq, 0.0) * LIMIT_STOP_TIME)
        j = int(np.argmin(d_pred))
        vmax = float(np.max(np.abs(dq)))
        gaps = {}   # first frame only: the tips from the loop frame vs the start read
        if state["frames"] == 0:
            gaps = {s: float(np.max(np.linalg.norm(task_state[s][0] - tips_start[s], axis=1)))
                    for s in task}
        mismatch = [f"{s} {g * 1000:.1f} mm" for s, g in gaps.items() if g > START_MISMATCH]
        hold_err = {s: float(np.max(np.abs(q_start[SIDES[s][2]] - q[SIDES[s][2]]))) for s in hold}
        pushed = [f"{s} {e:.3f} rad" for s, e in hold_err.items() if e > MAX_TRACK_ERR]
        abort, abort_rc = None, 3
        if mismatch:
            abort, abort_rc = (f"the first torque-loop frame puts the triangle away from the "
                               f"start read ({mismatch}; waist joints missing from one of "
                               "them?)"), 1
        elif d_pred[j] < LIMIT_ABORT:
            abort = (f"{ARMS[j]} is {min(d_lo[j], d_hi[j]):.3f} rad from its limit at "
                     f"{dq[j]:+.2f} rad/s (pushed through the wall)")
        elif vmax > MAX_JOINT_VEL:
            abort = f"joint velocity {vmax:.2f} rad/s > {MAX_JOINT_VEL}"
        elif pushed:
            abort = f"held arm off its start pose ({pushed}, limit {MAX_TRACK_ERR} rad)"
        if abort:
            print(f"[t={t:.1f}s] {abort}, abort", file=sys.stderr)
            state["rc"] = abort_rc
            req.end_effector_joint_names, req.end_effector_joint_positions = [], []
            req.finished = True
            return req

        if state["status_now"] or state["frames"] % STATUS_EVERY == 0:
            state["status_now"] = False
            by_name = {s["name"]: s for s in frame["states"]}
            parts = [f"t={t:.1f}s {step}"]
            for side, (_, c, c_d, R, R_d, f_c, moment) in task_state.items():
                g_name = grippers[side][0]
                g_meas = (f" (frame {by_name[g_name]['position']:+.3f})" if g_name in by_name
                          else "")
                parts.append(f"{side}: |c_d-c|={np.linalg.norm(c_d - c) * 1000:.1f} mm "
                             f"|c_end-c|={np.linalg.norm(c_end[side] - c) * 1000:.1f} mm "
                             f"rot err {np.degrees(rot_angle(R_d @ R.T)):.2f} deg "
                             f"finger tilt {np.degrees(tilt(R)):+.1f} deg "
                             f"|F|={np.linalg.norm(f_c):.2f} N |M|={np.linalg.norm(moment):.2f} Nm "
                             f"gripper {grip[side]:+.3f}{g_meas}")
            dist = np.minimum(d_lo, d_hi)
            k = int(np.argmin(dist))
            parts.append(f"closest limit: {ARMS[k]} {dist[k]:.3f} rad")
            in_zone = [ARMS[i] for i in range(len(ARMS)) if dist[i] < LIMIT_ZONE]
            if in_zone:
                parts.append(f"wall active on {in_zone}")
            print(" | ".join(parts), flush=True)

        state["frames"] += 1
        req.finished = stop_requested  # runs until Ctrl+C / SIGTERM
        return req

    # ---- Enter mode 9 and run; whatever happens, restore the old mode. ----
    if WAIT_FOR_ENTER:
        threading.Thread(target=read_enter, args=(enter, loop_done), daemon=True).start()
    rc = 0
    restored = False
    try:
        robot.set_control_mode(MODE_JOINT_TORQUE)
        robot.wait_control_mode(MODE_JOINT_TORQUE, 500)
        print("running: opening the gripper at the start pose; Ctrl+C stops at any time",
              flush=True)
        robot.joint_torque_control(callback, state_timeout_ms=100)
        # The stream is already ended here, no joint_torque_end() needed.
    except (TimeoutError, ValueError, RuntimeError) as e:  # GDK read/write/mode failures
        print(f"torque loop failed: {e}", file=sys.stderr)
        rc = 2
    finally:
        loop_done.set()
        # Leave mode 9 ourselves on every exit path; the GDK never does it.
        restored = restore_mode(robot, old_mode)
    rc = state["rc"] or rc
    if not restored:
        rc = rc or 2
    print(f"done at step {state['step']}, rc={rc}")
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
