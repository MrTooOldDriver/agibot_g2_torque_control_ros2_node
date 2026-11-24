#!/usr/bin/env python3
"""End-to-end smoke test of the torque subscriber against the simulated arms (no robot needed).

  python3 smoke_test_fake.py --impl cpp --urdf /models/<g2>.urdf
  python3 smoke_test_fake.py --impl py  --urdf /models/<g2>.urdf

Starts the node (backend:=fake, or the fake agibot_gdk for --impl py) and checks:
  1. with no command the arms float: gravity compensation holds them (drift < 0.02 rad in 1.5 s);
  2. a left-arm command shows up in ~/commanded_torques as g(q) + command on the left arm only;
  3. after the command stops, the external torque times out (commanded torque back to ~g(q));
  4. a wrong-length command is rejected (the node keeps running);
  5. a large command drives a joint past velocity_limit -> safety abort, mode restored, exit 3;
  6. (separate run) SIGINT -> final frame, previous mode restored, exit 0.
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

NS = "/g2_torque_subscriber"


class Probe(Node):
    def __init__(self):
        super().__init__("smoke_probe")
        self.left = None
        self.cmd = None
        self.full = None
        self.create_subscription(JointState, NS + "/left/joint_states",
                                 lambda m: setattr(self, "left", m), 10)
        self.create_subscription(JointState, NS + "/commanded_torques",
                                 lambda m: setattr(self, "cmd", m), 10)
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "full", m), 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_left = self.create_publisher(Float64MultiArray, NS + "/left/external_torques", qos)

    def spin_for(self, seconds, publish=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish is not None:
                self.pub_left.publish(Float64MultiArray(data=list(publish)))
            rclpy.spin_once(self, timeout_sec=0.002)


class Gravity:
    """g(q) of the 14 arm joints, non-arm joints at 0 (as the fake robot reports them)."""

    def __init__(self, urdf):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)

    def external(self, msg):
        """tau_sent - g(q) of a ~/commanded_torques message (position = q of the same frame)."""
        iq = [self.model.joints[self.model.getJointId(n)].idx_q for n in msg.name]
        iv = [self.model.joints[self.model.getJointId(n)].idx_v for n in msg.name]
        self.q[iq] = msg.position
        g = np.asarray(pin.computeGeneralizedGravity(self.model, self.data, self.q))[iv]
        return np.array(msg.effort) - g


# Bent arms (shoulder, elbow, wrist), so gravity torques are significant.
DEFAULT_POSE = "0.3,0.4,0.2,-1.0,0.2,0.3,0.2,-0.3,-0.4,-0.2,-1.0,-0.2,-0.3,-0.2"


def start_node(impl, urdf, extra=()):
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
    cmd = exe + ["--ros-args", "-p", "backend:=fake", "-p", f"urdf_path:={urdf}",
                 "-p", "publish_decimation:=5", *extra]
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def finish(proc, sig=None, timeout=15):
    if sig is not None:
        os.killpg(proc.pid, sig)
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
        return None, out
    return proc.returncode, out


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""), flush=True)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--impl", choices=["cpp", "py"], default="cpp")
    ap.add_argument("--urdf", required=True)
    a = ap.parse_args()
    rclpy.init()
    probe = Probe()
    results = []

    # ---------------- run 1: float, command, timeout, bad message, velocity abort ----------
    proc = start_node(a.impl, a.urdf)
    t0 = time.monotonic()
    while probe.left is None and time.monotonic() - t0 < 20 and proc.poll() is None:
        probe.spin_for(0.1)
    if not check("node publishes ~/left/joint_states", probe.left is not None):
        print(finish(proc, signal.SIGINT)[1])
        return 1
    q0 = np.array(probe.left.position)
    probe.spin_for(1.5)
    drift = float(np.max(np.abs(np.array(probe.left.position) - q0)))
    results.append(check("arms float under gravity compensation", drift < 0.02,
                         f"max drift {drift:.4f} rad in 1.5 s"))
    results.append(check("/joint_states has all joints", probe.full is not None
                         and len(probe.full.name) > 14, f"{len(probe.full.name)} joints"
                         if probe.full else "none"))
    grav = Gravity(a.urdf)
    ext = grav.external(probe.cmd)
    results.append(check("no command -> sent torque = g(q)", np.max(np.abs(ext)) < 1e-6,
                         f"max |tau - g| {np.max(np.abs(ext)):.2e} Nm"))

    command = [0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0]  # small: light links
    probe.spin_for(0.3, publish=command)
    ext = grav.external(probe.cmd)
    results.append(check("left command added on top of gravity",
                         np.allclose(ext[:7], command, atol=1e-6) and np.allclose(ext[7:], 0,
                                                                                  atol=1e-6),
                         f"tau - g = {np.round(ext, 4)}"))
    probe.spin_for(0.4)  # > command_timeout
    ext = grav.external(probe.cmd)
    results.append(check("command timeout -> external torque 0 (gravity only)",
                         np.max(np.abs(ext)) < 1e-6, f"max |tau - g| {np.max(np.abs(ext)):.2e}"))

    probe.pub_left.publish(Float64MultiArray(data=[1.0, 2.0]))
    probe.spin_for(0.2)
    results.append(check("wrong-length command rejected, node alive", proc.poll() is None))

    probe.spin_for(3.0, publish=[0, 0, 0, 0, 0, 0, 15.0])  # joint7: 15 Nm on a light link
    rc, out = finish(proc, timeout=10) if proc.poll() is not None else finish(proc, signal.SIGINT)
    results.append(check("velocity abort -> exit 3", rc == 3, f"rc={rc}"))
    results.append(check("abort reported", "velocity_limit" in out))
    results.append(check("mode restored after abort", "control mode restored to 3" in out))

    # ---------------- run 2: SIGINT ------------------------------------------------------------
    probe.left = None
    proc = start_node(a.impl, a.urdf)
    t0 = time.monotonic()
    while probe.left is None and time.monotonic() - t0 < 20 and proc.poll() is None:
        probe.spin_for(0.1)
    probe.spin_for(0.5, publish=command)
    rc, out = finish(proc, signal.SIGINT)
    results.append(check("SIGINT -> exit 0", rc == 0, f"rc={rc}"))
    results.append(check("mode restored after SIGINT", "control mode restored to 3" in out))
    if not all(results):
        print("------ node output (run 2) ------\n" + out)
    probe.destroy_node()
    rclpy.shutdown()
    print("RESULT:", "PASS" if all(results) else "FAIL")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
