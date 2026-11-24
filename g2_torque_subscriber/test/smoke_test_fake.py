#!/usr/bin/env python3
"""End-to-end smoke test of the torque subscriber against the simulated arms (no robot needed).

  python3 smoke_test_fake.py --impl cpp --urdf /models/<g2>.urdf
  python3 smoke_test_fake.py --impl py  --urdf /models/<g2>.urdf

Starts the node (backend:=fake; for --impl py also the fake agibot_gdk first on PYTHONPATH) in a
private namespace /g2_smoke_<pid>_<time>, so that the test commands (up to 15 Nm) can never reach
a node driving the real robot in the same ROS domain (the container uses host networking):
  A. gravity + coriolis + inertia on: arms float; sent torque = g + C dq (+ M ddq_d) + command,
     each term checked against an independent Pinocchio model; timeout; rejected message; velocity
     abort with mode restore.
  B. gravity on, SIGINT: final frame, mode restored, exit 0.
  C. all compensation off (the default): the measured state is published while the node waits;
     no mode 9 (no ~/commanded_torques) without commands, nor with commands far from the holding
     torque (zeros); mode 9 once both arms send the full gravity torque; a missing command then
     ends the loop (exit 3) and restores the mode.
  D. refusals before mode 9 (exit 1): measured holding torque above the effort cap
     (effort_limit_scale); --impl py also: backend:=gdk with the fake agibot_gdk.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

RUN_NS = f"/g2_smoke_{os.getpid()}_{int(time.time())}"  # private to this test run
NS = RUN_NS + "/g2_torque_subscriber"
# Bent arms (shoulder, elbow, wrist), so gravity torques are significant.
DEFAULT_POSE = "0.3,0.4,0.2,-1.0,0.2,0.3,0.2,-0.3,-0.4,-0.2,-1.0,-0.2,-0.3,-0.2"
POSE = np.array([float(x) for x in DEFAULT_POSE.split(",")])
ARM_NAMES = ([f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
             + [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)])
ATOL = 1e-6


class Probe(Node):
    def __init__(self):
        super().__init__("smoke_probe", namespace=RUN_NS)
        self.msgs = {}
        for key, topic in (("left", NS + "/left/joint_states"), ("cmd", NS + "/commanded_torques"),
                           ("comp", NS + "/compensation_torques"), ("full", RUN_NS + "/joint_states")):
            self.create_subscription(JointState, topic,
                                     lambda m, key=key: self.msgs.__setitem__(key, m), 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pubs = {k: self.create_publisher(Float64MultiArray, NS + t, qos) for k, t in (
            ("left", "/left/external_torques"), ("right", "/right/external_torques"),
            ("left_acc", "/left/desired_accelerations"))}

    def spin_for(self, seconds, **publish):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for key, values in publish.items():
                self.pubs[key].publish(Float64MultiArray(data=[float(v) for v in values]))
            rclpy.spin_once(self, timeout_sec=0.002)

    def reset(self):
        """Drain messages still queued from a previous run, then forget them."""
        self.spin_for(0.5)
        self.msgs.clear()

    def wait_for(self, key, proc, timeout=20.0):
        t0 = time.monotonic()
        while key not in self.msgs and time.monotonic() - t0 < timeout and proc.poll() is None:
            self.spin_for(0.1)
        return key in self.msgs


class Model:
    """Independent Pinocchio model; non-arm joints at 0 (as the fake robot reports them)."""

    def __init__(self, urdf):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()

    def _idx(self, names):
        return ([self.model.joints[self.model.getJointId(n)].idx_q for n in names],
                [self.model.joints[self.model.getJointId(n)].idx_v for n in names])

    def terms(self, msg, ddq=None):
        """(g, C dq, M ddq) of the 14 joints of a ~/commanded_torques-style message."""
        iq, iv = self._idx(msg.name)
        q, v = pin.neutral(self.model), np.zeros(self.model.nv)
        q[iq], v[iv] = msg.position, msg.velocity
        g = np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))[iv].copy()
        c = np.asarray(pin.nonLinearEffects(self.model, self.data, q, v))[iv] - g
        a = np.zeros(self.model.nv)
        if ddq is not None:
            a[iv] = ddq
        m = np.asarray(pin.rnea(self.model, self.data, q, np.zeros(self.model.nv), a))[iv] - g
        return g, c, m

    def gravity_at(self, names, positions):
        iq, iv = self._idx(names)
        q = pin.neutral(self.model)
        q[iq] = positions
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))[iv].copy()


def start_node(impl, urdf, params=(), backend="fake"):
    env = dict(os.environ)
    env.setdefault("G2_FAKE_INITIAL_POSE", DEFAULT_POSE)
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "g2_torque_subscriber"],
                                     text=True).strip()
    if impl == "py":
        env["PYTHONPATH"] = prefix + "/share/g2_torque_subscriber/fake_gdk:" + env.get(
            "PYTHONPATH", "")
        env["G2_FAKE_URDF"] = urdf
        exe = [prefix + "/lib/g2_torque_subscriber/torque_subscriber_node.py"]
    else:
        exe = [prefix + "/lib/g2_torque_subscriber/torque_subscriber_node"]
    cmd = exe + ["--ros-args", "-r", f"__ns:={RUN_NS}", "-p", f"backend:={backend}",
                 "-p", f"urdf_path:={urdf}", "-p", "publish_decimation:=5"]
    for p in params:
        cmd += ["-p", p]
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def finish(proc, sig=None, timeout=15):
    if sig is not None and proc.poll() is None:
        os.killpg(proc.pid, sig)
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
        return None, out
    return proc.returncode, out


results = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""), flush=True)
    results.append(bool(ok))
    return ok


def run_a(probe, model, a):
    print("--- A: gravity + coriolis + inertia on")
    probe.reset()
    proc = start_node(a.impl, a.urdf, ["compensate_gravity:=true", "compensate_coriolis:=true",
                                       "compensate_inertia:=true"])
    if not check("node streams (~/left/joint_states)", probe.wait_for("left", proc)):
        return finish(proc, signal.SIGINT)[1]
    q0 = np.array(probe.msgs["left"].position)
    probe.spin_for(1.5)
    drift = float(np.max(np.abs(np.array(probe.msgs["left"].position) - q0)))
    check("arms float under compensation", drift < 0.02, f"max drift {drift:.4f} rad in 1.5 s")
    full = probe.msgs.get("full")
    check("/joint_states has all joints", full is not None and len(full.name) > 14,
          f"{len(full.name) if full else 0} joints")

    msg = probe.msgs["cmd"]
    g, c, _ = model.terms(msg)
    err = np.max(np.abs(np.array(msg.effort) - (g + c)))
    check("no command: sent = g + C dq", err < ATOL, f"max err {err:.1e}")

    command = np.array([0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0])  # small: light links
    ddq_left = np.array([0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0])
    ddq = np.concatenate([ddq_left, np.zeros(7)])
    probe.spin_for(0.3, left=command, left_acc=ddq_left)
    msg = probe.msgs["cmd"]
    g, c, m = model.terms(msg, ddq)
    ext = np.array(msg.effort) - (g + c + m)
    check("command + desired acceleration: sent = g + C dq + M ddq_d + command",
          np.allclose(ext, np.concatenate([command, np.zeros(7)]), atol=ATOL),
          f"sent - model = {np.round(ext, 4)}")
    comp = probe.msgs["comp"]
    g, c, m = model.terms(comp, ddq)
    err = np.max(np.abs(np.array(comp.effort) - (g + c + m)))
    check("~/compensation_torques = g + C dq + M ddq_d", err < ATOL,
          f"max err {err:.1e}; |M ddq_d| max {np.max(np.abs(m)):.3f} Nm, "
          f"|C dq| max {np.max(np.abs(c)):.4f} Nm")

    probe.spin_for(0.4)  # > command_timeout
    msg = probe.msgs["cmd"]
    g, c, _ = model.terms(msg)
    err = np.max(np.abs(np.array(msg.effort) - (g + c)))
    check("timeout: command and acceleration -> 0", err < ATOL, f"max |sent - (g + C dq)| {err:.1e}")

    probe.pubs["left"].publish(Float64MultiArray(data=[1.0, 2.0]))
    probe.spin_for(0.2)
    check("wrong-length command rejected, node alive", proc.poll() is None)

    probe.spin_for(3.0, left=[0, 0, 0, 0, 0, 0, 15.0])  # joint7: 15 Nm on a light link
    rc, out = finish(proc, signal.SIGINT)
    check("velocity abort -> exit 3", rc == 3, f"rc={rc}")
    check("abort reported", "velocity_limit" in out)
    check("mode restored after abort", "control mode restored to 3" in out)
    return out


def run_b(probe, a):
    print("--- B: gravity on, SIGINT")
    probe.reset()
    proc = start_node(a.impl, a.urdf, ["compensate_gravity:=true"])
    probe.wait_for("left", proc)
    probe.spin_for(0.5, left=[0, 0, 0, 0.2, 0, 0, 0])
    rc, out = finish(proc, signal.SIGINT)
    check("SIGINT -> exit 0", rc == 0, f"rc={rc}")
    check("mode restored after SIGINT", "control mode restored to 3" in out)
    return out


def run_c(probe, model, a):
    print("--- C: all compensation off (default)")
    probe.reset()
    proc = start_node(a.impl, a.urdf)
    # ~/commanded_torques only comes from the torque loop: its absence means "not in mode 9".
    check("measured state published while waiting for commands", probe.wait_for("left", proc))
    probe.spin_for(1.0)
    check("no commands: does not enter mode 9", "cmd" not in probe.msgs and proc.poll() is None)
    zeros = np.zeros(7)
    probe.spin_for(1.0, left=zeros, right=zeros)
    check("zero commands (no gravity): does not enter mode 9",
          "cmd" not in probe.msgs and proc.poll() is None)
    g0 = model.gravity_at(ARM_NAMES, POSE)
    probe.spin_for(1.5, left=g0[:7], right=g0[7:])
    check("full gravity commands: enters mode 9 and streams", "cmd" in probe.msgs)
    if "cmd" in probe.msgs and "left" in probe.msgs:
        sent = np.array(probe.msgs["cmd"].effort)
        err = np.max(np.abs(sent - g0))
        check("sent torque = command (no model terms)", err < ATOL, f"max |sent - command| {err:.1e}")
        drift = float(np.max(np.abs(np.array(probe.msgs["left"].position) - POSE[:7])))
        check("arms held by the commands", drift < 0.02, f"max drift {drift:.4f} rad")
    probe.spin_for(1.0)  # stop publishing
    rc, out = finish(proc, signal.SIGINT)
    check("missing command -> abort, exit 3", rc == 3, f"rc={rc}")
    check("abort says why", "no torque command" in out)
    check("mode restored after abort", "control mode restored to 3" in out)
    return out


def run_d(probe, a):
    print("--- D: refusals before mode 9")
    probe.reset()
    proc = start_node(a.impl, a.urdf, ["compensate_gravity:=true", "effort_limit_scale:=0.01"])
    rc, out = finish(proc, timeout=30)
    check("holding torque above the effort cap -> exit 1", rc == 1, f"rc={rc}")
    check("effort cap refusal says why", "exceeds the effort cap" in out)
    check("effort cap refusal: mode 9 never requested",
          "-> 9" not in out and "cmd" not in probe.msgs)
    outs = [out]
    if a.impl == "py":  # the fake agibot_gdk is first on PYTHONPATH, the real GDK is asked for
        proc = start_node(a.impl, a.urdf, ["compensate_gravity:=true"], backend="gdk")
        rc, out = finish(proc, timeout=30)
        check("backend:=gdk with the fake agibot_gdk -> exit 1 before connecting",
              rc == 1 and "not connecting" in out and "connecting to the robot" not in out,
              f"rc={rc}")
        outs.append(out)
    return "\n".join(outs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--impl", choices=["cpp", "py"], default="cpp")
    ap.add_argument("--urdf", required=True)
    a = ap.parse_args()
    rclpy.init()
    probe = Probe()
    model = Model(a.urdf)
    outputs = [run_a(probe, model, a), run_b(probe, a), run_c(probe, model, a), run_d(probe, a)]
    if not all(results):
        for label, out in zip("ABCD", outputs):
            print(f"------ node output, run {label} ------\n{out}")
    probe.destroy_node()
    rclpy.shutdown()
    print("RESULT:", "PASS" if all(results) else "FAIL")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
