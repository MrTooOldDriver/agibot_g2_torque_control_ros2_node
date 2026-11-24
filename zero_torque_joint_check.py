#!/usr/bin/env python3
"""Zero-torque joint check of the AGIBOT G2 arms: float both arms, move them by hand joint by
joint, and check the joint mapping and the torque feedback. GDK Python API only (no ROS); the
mode-9 handling follows joint_torque_demo.py.

"Zero torque" means zero torque ON TOP OF GRAVITY COMPENSATION. In direct torque mode (control
mode 9) the robot compensates nothing, so 14 zeros would drop the arms. Every frame sends, to all
14 arm joints,
    tau = g(q) - damping * dq       g: Pinocchio + URDF, waist/head at their measured pose;
                                    damping is 0 unless --damping is given
There is no position feedback, so a wrongly mapped or sign-flipped joint cannot run away: at worst
the arm sags, and the pre-flight check refuses to start in that case.

  1. Pre-flight (nothing is sent): per joint, model gravity g(q) vs measured effort, and position
     vs motor_position (for information). If |effort - g| > --gravity-tol on any joint (wrong
     URDF, payload, waist pose or effort mapping), the script refuses to enter mode 9.
  2. Remember the control mode and switch to mode 9. Do not touch the arms while it switches:
     the robot refuses to enter under external force.
  3. Guided check (L1..L7 = idx21..idx27, R1..R7 = idx61..idx67). For each joint the script
     waits for the arms to rest, then asks you to rotate that joint (Nth from the shoulder)
     slowly by hand by more than --move-threshold, holding the links between the shoulder and
     that joint still with the other hand, and to let go.
       joint mapping:   the joint whose reported position changed most must be the one asked for.
       torque feedback: once the arm rests again, the measured effort of every joint whose g(q)
                        changed must have changed with the same sign (against it by >= 3 Nm:
                        FAIL, by less: WARN), and |effort - g| must stay within --gravity-tol on
                        all joints. An effort that changed by less than 1 Nm while g(q) changed
                        by >= 3 Nm (stuck feedback), or a non-finite effort, is a FAIL too.
                        A move changes the gravity load mostly of the joints between
                        the shoulder and the moved joint, so a flipped or swapped effort (or
                        position) shows up when moving the outer joints of a loaded arm (e.g.
                        elbow bent); an arm hanging straight down carries little load.
     Enter skips a joint; q + Enter or Ctrl+C ends the check.
     PASS covers the state side (which reported joint moved, effort vs the model). That each
     commanded torque reaches the right joint shows only as the arms floating still.
  4. Restore the old control mode (always, also after errors and aborts) and print a summary.

Abort (final frame, then step 4): an arm joint faster than --velocity-limit, a non-finite state or
torque, or a frame without the 14 arm joints. Mode 9 has no joint-limit protection: keep the moves
well inside the joint range.

--dry-run never switches the mode or sends torque: after the pre-flight it runs the same guided
check on states read at 100 Hz, for arms moved by other means (e.g. wrist-button drag teaching).

Python 1 kHz is best effort. Keep the e-stop at hand. Run in a terminal (Enter/q need a tty) with
the GDK environment sourced, e.g. from the repo root in this repo's container:
  docker run --rm -it --network host -v "$PWD":/repo:ro -v "$PWD/agibot_gdk/app":/opt/agibot/app:ro \
    g2-torque-subscriber:humble \
    python3 /repo/zero_torque_joint_check.py [urdf_path] [--dry-run] [--joints L4,R4]
Without urdf_path the confirmed G2 model of this repo is used,
g2_torque_subscriber/urdf/genie_g2_crs_omnipicker.urdf. An empty urdf_path ("") uses the URDF of
the robot model reported by the robot (G2_t2_crsB, NOT the right model; on the robot's
filesystem, not in the container).
Exit code: 0 every checked joint passed, 1 setup error, 2 GDK error, 3 safety abort, 4 a joint
failed.
"""

import argparse
import os
import select
import signal
import sys
import threading
import time

import agibot_gdk as gdk
import numpy as np
import pinocchio as pin

URDF_DIR = "/home/agi/app/share/genie_robot_description/urdf"
# The G2 model confirmed for this robot (the robot's reported urdf_type is not the right one).
DEFAULT_URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "g2_torque_subscriber",
                            "urdf", "genie_g2_crs_omnipicker.urdf")
ARMS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3", "idx24_arm_l_joint4",
    "idx25_arm_l_joint5", "idx26_arm_l_joint6", "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3", "idx64_arm_r_joint4",
    "idx65_arm_r_joint5", "idx66_arm_r_joint6", "idx67_arm_r_joint7",
]
LABELS = [f"L{i}" for i in range(1, 8)] + [f"R{i}" for i in range(1, 8)]
MODE_JOINT_TORQUE = 9
MAX_STEP_NM = 15.0    # Nm per frame, keep below MC joint_torque_control_config.max_step_nm (50)
REST_SPEED = 0.05     # rad/s: the arms rest when every arm joint is slower than this ...
REST_SECONDS = 0.5    # ... for this long
REST_WAIT_MAX = 3.0   # s, take the baseline anyway when the arms do not come to rest
SETTLE_MAX = 10.0     # s after a move was detected, evaluate it even if the arms do not rest
MIN_DG = 1.0          # Nm: g(q) of a joint changed this much and its effort the other way: WARN
CLEAR_DG = 3.0        # Nm: ... and g(q) changed this much and the effort by MIN_DG: FAIL
AMBIGUOUS = 0.5       # WARN when another joint moved more than this fraction of the moved joint
STATUS_PERIOD = 0.2   # s between live status lines (terminal only)
TTY = sys.stdout.isatty()
CLEAR = "\r\033[K" if TTY else ""  # overwrite the live status line

stops = 0


def on_signal(_signum, _frame):
    global stops  # the torque loop sends its final frame, then the old mode is restored
    stops += 1


class Gravity:
    """Pinocchio gravity torques of the 14 arm joints; waist/head take their measured values."""

    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.arm_q, self.arm_v, limit = [], [], []
        for name in ARMS:
            if not self.model.existJointName(name):
                raise RuntimeError(f"URDF has no joint {name}")
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise RuntimeError(f"joint {name} is not 1-DoF")
            self.arm_q.append(joint.idx_q)
            self.arm_v.append(joint.idx_v)
            limit.append(max(1.0, float(self.model.effortLimit[joint.idx_v])))
        self.limit = np.array(limit)
        self.others = []  # (name, idx_q) of the non-arm 1-DoF joints
        for jid in range(1, self.model.njoints):
            joint, name = self.model.joints[jid], self.model.names[jid]
            if joint.nq == 1 and joint.nv == 1 and name not in ARMS:
                self.others.append((name, joint.idx_q))

    def __call__(self, by_name, q):
        for name, iq in self.others:
            if name in by_name:
                self.q[iq] = by_name[name]["position"]
        self.q[self.arm_q] = q
        return pin.computeGeneralizedGravity(self.model, self.data, self.q)[self.arm_v]


def extract(state):
    """(by_name, q, dq, effort, motor_position) of the 14 arm joints, or None if one is missing."""
    by_name = {s["name"]: s for s in state["states"]}
    if any(n not in by_name for n in ARMS):
        return None

    def column(key):
        return np.array([by_name[n].get(key, np.nan) for n in ARMS], float)

    return (by_name, column("position"), column("velocity"), column("effort"),
            column("motor_position"))


class JointCheck:
    """Guided joint-by-joint check. step() runs once per state frame and never blocks."""

    def __init__(self, order, threshold, tol, timeout):
        self.order, self.threshold, self.tol, self.timeout = order, threshold, tol, timeout
        self.k = 0             # position in order
        self.phase = "rest"    # rest -> move -> settle -> rest (next joint)
        self.phase_t = None
        self.rest_since = None
        self.status_t = 0.0
        self.results = {}      # joint index -> (result, line)
        self.skip = threading.Event()

    @property
    def done(self):
        return self.k >= len(self.order)

    def step(self, t, q, dq, eff, motor, g):
        if self.done:
            return
        j = self.order[self.k]
        if self.phase_t is None:
            self.phase_t = t
        if np.max(np.abs(dq)) < REST_SPEED:
            self.rest_since = t if self.rest_since is None else self.rest_since
        else:
            self.rest_since = None
        rested = self.rest_since is not None and t - self.rest_since >= REST_SECONDS
        if self.skip.is_set():
            self.skip.clear()
            self._next(j, "SKIP", "skipped", t)
            return

        if self.phase == "rest":  # baseline, then ask for the joint
            if rested or t - self.phase_t >= REST_WAIT_MAX:
                self.q0, self.eff0, self.g0, self.motor0 = q.copy(), eff.copy(), g.copy(), motor
                self.peak = np.zeros(len(ARMS))           # largest |dq| since the baseline ...
                self.peak_d = np.zeros(len(ARMS))         # ... signed
                self.peak_m = np.full(len(ARMS), np.nan)  # motor_position change at that moment
                self.phase, self.phase_t = "move", t
                arm = "left" if j < 7 else "right"
                print(f"\n[{self.k + 1}/{len(self.order)}] {LABELS[j]} {ARMS[j]} ({arm} arm, "
                      f"joint {j % 7 + 1} from the shoulder): rotate it slowly by hand by more "
                      f"than {self.threshold:g} rad, holding the links between the shoulder and "
                      "it still, then let go.  Enter = skip, Ctrl+C = stop"
                      + ("" if rested else "\n    (arms not at rest, baseline taken anyway)"),
                      flush=True)
            return

        d = q - self.q0
        grow = np.abs(d) > self.peak
        self.peak[grow] = np.abs(d[grow])
        self.peak_d[grow] = d[grow]
        self.peak_m[grow] = (motor - self.motor0)[grow]
        if self.phase == "move":
            if self.peak.max() > self.threshold:
                self.phase, self.phase_t = "settle", t
                self.rest_since = None  # a slow move must still come to rest after this point
            elif self.timeout > 0 and t - self.phase_t > self.timeout:
                self._next(j, "SKIP", f"no move within {self.timeout:g} s", t)
                return
        elif rested or t - self.phase_t >= SETTLE_MAX:
            self._evaluate(j, eff, g, t)
            return
        if TTY and t - self.status_t >= STATUS_PERIOD:
            self.status_t = t
            m = int(np.argmax(np.abs(d)))
            sys.stdout.write(f"{CLEAR}    {LABELS[j]} {d[j]:+.3f} rad   most moved {LABELS[m]} "
                             f"{d[m]:+.3f} rad   effort {eff[j]:+7.2f}  g(q) {g[j]:+7.2f} Nm")
            sys.stdout.flush()

    def _evaluate(self, j, eff, g, t):
        rank = np.argsort(self.peak)[::-1]
        moved, runner = int(rank[0]), int(rank[1])
        # Torque feedback over all 14 joints: a move mostly changes the gravity load of the joints
        # between the shoulder and the moved joint, often not of the moved joint itself.
        d_eff, d_g, resid = eff - self.eff0, g - self.g0, eff - g
        b, w = int(np.argmax(np.abs(d_g))), int(np.argmax(np.abs(resid)))
        text = (f"moved {LABELS[moved]} {self.peak_d[moved]:+.3f} rad (next {LABELS[runner]} "
                f"{self.peak[runner]:.3f})")
        ratio = self.peak_m[moved] / self.peak_d[moved]
        if np.isfinite(ratio):
            text += f", motor/joint {ratio:.2f}"
        text += (f"; largest dg {LABELS[b]} {d_g[b]:+.2f} (deffort {d_eff[b]:+.2f}), "
                 f"max |effort-g| {LABELS[w]} {abs(resid[w]):.2f} Nm")
        fail, warn = [], []
        if moved != j:
            fail.append(f"{LABELS[moved]} moved, not {LABELS[j]}")
        if not all(np.all(np.isfinite(x)) for x in (eff, g, self.eff0, self.g0)):
            # NaN would pass every comparison below: no valid torque feedback to judge.
            fail.append("non-finite effort or g(q)")
        else:
            against = (np.abs(d_g) >= MIN_DG) & (d_eff * d_g < 0)
            clearly = against & (np.abs(d_g) >= CLEAR_DG) & (np.abs(d_eff) >= MIN_DG)
            # g(q) changed clearly but the effort barely did: stuck or missing feedback.
            stuck = (np.abs(d_g) >= CLEAR_DG) & (np.abs(d_eff) < MIN_DG)
            if clearly.any():
                fail.append(f"effort of {','.join(np.array(LABELS)[clearly])} changed against "
                            "g(q)")
            if stuck.any():
                fail.append(f"effort of {','.join(np.array(LABELS)[stuck])} did not follow g(q)")
            if (against & ~clearly & ~stuck).any():
                warn.append(f"effort of {','.join(np.array(LABELS)[against & ~clearly & ~stuck])} "
                            "changed slightly against g(q)")
            if abs(resid[w]) > self.tol:
                fail.append(f"|effort - g| > {self.tol:g} Nm")
        if self.peak[runner] > AMBIGUOUS * self.peak[moved]:
            warn.append(f"{LABELS[runner]} moved too, hold the other links still and redo")
        result = "FAIL" if fail else "WARN" if warn else "PASS"
        self._next(j, result, text + "".join(f"; {p}" for p in fail + warn), t)

    def _next(self, j, result, text, t):
        line = f"{LABELS[j]} {ARMS[j]:19s} {result:4s}  {text}"
        print(f"{CLEAR}  {line}", flush=True)
        self.results[j] = (result, line)
        self.k += 1
        self.phase, self.phase_t = "rest", t

    def summary(self):
        """Print the result of every joint; True if one failed."""
        print("\nsummary")
        for j in self.order:
            print("  " + self.results.get(j, ("", f"{LABELS[j]} {ARMS[j]:19s} not checked"))[1])
        return any(self.results.get(j, ("",))[0] == "FAIL" for j in self.order)


def read_keys(check):
    """Enter skips the current joint, q + Enter stops. The torque loop never waits for input.

    Polls the raw fd (select + os.read) instead of iterating sys.stdin, so this daemon thread
    never holds the stdin buffer lock when the interpreter exits.
    """
    global stops
    try:
        fd = sys.stdin.fileno()
    except (AttributeError, ValueError, OSError):
        return
    while not check.done and not stops:
        if not select.select([fd], [], [], 0.2)[0]:
            continue
        data = os.read(fd, 1024)
        if not data:  # EOF
            return
        for line in data.decode(errors="replace").splitlines() or [""]:
            if line.strip().lower() == "q":
                stops += 1
                return
            check.skip.set()


def run(robot, args):
    # ---- Dynamics model (Pinocchio). Needs a URDF that matches the robot. ----
    urdf = args.urdf
    if not urdf:
        name = robot.get_whole_body_status().get("urdf_type", "")  # the binding returns a dict
        if not name:
            print("urdf_type unavailable, pass the urdf path as argument", file=sys.stderr)
            return 1
        urdf = f"{URDF_DIR}/{name}/{name}.urdf"
    print(f"urdf: {urdf}")
    grav = Gravity(urdf)

    # ---- Pre-flight: plain state read, nothing is sent. ----
    x = extract(robot.get_joint_states())
    if x is None:
        print("the robot state misses an arm joint", file=sys.stderr)
        return 1
    by_name, q, dq, eff, motor = x
    g = grav(by_name, q)
    print("pre-flight (nothing sent): model gravity g(q) vs measured effort\n"
          f"  {'joint':22s} {'position':>9s} {'motor_pos':>9s} {'g(q)':>8s} {'effort':>8s} "
          f"{'effort-g':>8s}")
    for i in range(len(ARMS)):
        print(f"  {LABELS[i]} {ARMS[i]:19s} {q[i]:+9.3f} {motor[i]:+9.3f} {g[i]:+8.2f} "
              f"{eff[i]:+8.2f} {eff[i] - g[i]:+8.2f}")
    worst = float(np.max(np.abs(eff - g)))
    print(f"  max |effort - g| = {worst:.2f} Nm (limit {args.gravity_tol:g})")
    if np.max(np.abs(dq)) > REST_SPEED:
        print("warning: the arms are moving, the pre-flight check is unreliable")
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(g)) and np.all(np.isfinite(eff))) \
            or worst > args.gravity_tol:
        print("refusing to float the arms: g(q) does not match the measured effort (wrong URDF, "
              "payload, waist pose or joint mapping?)", file=sys.stderr)
        return 1

    # From here on Ctrl+C / SIGTERM only set a flag: the check ends and the old mode is restored.
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    check = JointCheck(args.order, args.move_threshold, args.gravity_tol, args.timeout)
    threading.Thread(target=read_keys, args=(check,), daemon=True).start()
    if args.dry_run:
        print("dry run: the mode is not changed and nothing is sent; move the arms by other means")
        while not check.done and not stops:
            x = extract(robot.get_joint_states())
            if x is not None:
                by_name, q, dq, eff, motor = x
                check.step(time.monotonic(), q, dq, eff, motor, grav(by_name, q))
            time.sleep(0.01)
        return 4 if check.summary() else 0

    # ---- Remember the control mode so it can be restored at the end. ----
    old_mode = robot.get_motion_control_status().control_mode
    if old_mode == MODE_JOINT_TORQUE:
        print("robot is already in mode 9 (a previous run did not exit cleanly?), "
              "cannot tell which mode to restore", file=sys.stderr)
        return 1
    if stops:
        return 0
    print(f"control mode {old_mode} -> 9: do not touch the arms until the first joint is asked for")

    # ---- Callback: GDK calls it once per frame and sends what it returns. ----
    prev_tau = eff.copy()  # the robot compares frame 0 with its holding torque (~ measured effort)
    ctl = {"frames": 0, "abort": None}

    def callback(frame, _period_ns):
        nonlocal prev_tau
        req = gdk.JointTorqueControlReq()
        req.joint_names = list(ARMS)  # order of joint_efforts below
        x = extract(frame)
        if x is None:
            ctl["abort"] = "frame misses an arm joint"
        else:
            by_name, q, dq, eff, motor = x
            if ctl["frames"] == 0 and np.all(np.isfinite(eff)):
                prev_tau = eff.copy()
            g = grav(by_name, q)
            # Zero torque on top of gravity compensation, clamped to the URDF effort limit and to
            # MAX_STEP_NM change per frame.
            tau = np.clip(g - args.damping * dq, -grav.limit, grav.limit)
            tau = prev_tau + np.clip(tau - prev_tau, -MAX_STEP_NM, MAX_STEP_NM)
            fast = int(np.argmax(np.abs(dq)))
            if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))
                    and np.all(np.isfinite(tau))):
                ctl["abort"] = "non-finite joint state or torque"
            elif abs(dq[fast]) > args.velocity_limit:
                ctl["abort"] = (f"{LABELS[fast]} {ARMS[fast]} at {dq[fast]:+.2f} rad/s > "
                                f"{args.velocity_limit:g} rad/s")
        if ctl["abort"]:  # final frame: repeat the last torque sent, the old mode then holds
            req.joint_efforts = [float(v) for v in prev_tau]
            req.finished = True
            return req
        prev_tau = tau
        req.joint_efforts = [float(v) for v in tau]
        ctl["frames"] += 1
        check.step(time.monotonic(), q, dq, eff, motor, g)
        req.finished = stops > 0 or check.done
        return req

    # ---- Enter mode 9, float; whatever happens, restore the old mode. ----
    rc = 0
    try:
        robot.set_control_mode(MODE_JOINT_TORQUE)
        robot.wait_control_mode(MODE_JOINT_TORQUE, 500)
        print("both arms float (gravity compensation only)")
        robot.joint_torque_control(callback, state_timeout_ms=100)
        # The stream is already ended here, no joint_torque_end() needed.
    except Exception as e:  # noqa: BLE001  GDK read/write/mode failures, or a bug in the callback
        hint = (" (arm under external force, unregistered payload or failed force-control "
                "admission?)" if ctl["frames"] == 0 else "")
        print(f"\ntorque loop failed: {e!r}{hint}", file=sys.stderr)
        rc = 2
    finally:
        try:
            robot.set_control_mode(old_mode)
            print(f"\ncontrol mode restored to {old_mode}")
        except Exception as e:  # noqa: BLE001
            print(f"set_control_mode({old_mode}) failed, restore it manually and check the MC "
                  f"error state: {e!r}", file=sys.stderr)
            rc = rc or 2
    if ctl["abort"]:
        print(f"safety abort: {ctl['abort']}", file=sys.stderr)
        rc = 3
    failed = check.summary()
    return rc or (4 if failed else 0)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("urdf", nargs="?", default=DEFAULT_URDF,
                   help="URDF of the robot (default: the confirmed genie_g2_crs_omnipicker.urdf; "
                        "\"\" = the robot's reported urdf_type, not the right model)")
    p.add_argument("--dry-run", action="store_true",
                   help="never switch the mode or send torque")
    p.add_argument("--joints", default=",".join(LABELS),
                   help="joints to check, in order: labels or names, e.g. L4,R4 (default all 14)")
    p.add_argument("--move-threshold", type=float, default=0.15,
                   help="rad a joint must be rotated by hand (default 0.15)")
    p.add_argument("--timeout", type=float, default=0.0,
                   help="s to wait for a move before skipping the joint, 0 = wait (default)")
    p.add_argument("--gravity-tol", type=float, default=10.0,
                   help="Nm allowed between g(q) and the measured effort (default 10)")
    p.add_argument("--damping", type=float, default=0.0,
                   help="Nm*s/rad of joint damping on top of g(q) (default 0)")
    p.add_argument("--velocity-limit", type=float, default=2.0,
                   help="rad/s, abort when an arm joint is faster (default 2)")
    args = p.parse_args()
    index = {**{lab: i for i, lab in enumerate(LABELS)}, **{n.upper(): i for i, n in enumerate(ARMS)}}
    names = [s.strip().upper() for s in args.joints.split(",") if s.strip()]
    unknown = [s for s in names if s not in index]
    if unknown or not names:
        p.error(f"--joints: unknown joint(s) {unknown}, use L1..L7, R1..R7 or the joint names")
    args.order = list(dict.fromkeys(index[s] for s in names))
    if args.move_threshold <= 0 or args.damping < 0 or args.velocity_limit <= 0 \
            or args.gravity_tol <= 0 or args.timeout < 0:
        p.error("--move-threshold, --velocity-limit and --gravity-tol must be > 0, "
                "--damping and --timeout >= 0")
    return args


def main():
    args = parse_args()
    gdk.gdk_init()
    try:
        return run(gdk.Robot(), args)
    finally:
        gdk.gdk_release()


if __name__ == "__main__":
    sys.exit(main())
