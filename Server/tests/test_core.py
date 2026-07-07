#!/usr/bin/python3
"""
Unit tests for the hardware-independent control stack:
PID, kinematics, odometry, quadrature encoders, the closed-loop drive
controller (against a simulated plant), and the wire protocol.

Run from the Server directory:
    python -m unittest discover -s tests
or:
    python tests/test_core.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CONFIG, SideMapping, WheelGeometry  # noqa: E402
from drive_controller import DriveController, SimulatedDrivePlant  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders  # noqa: E402
from kinematics import SkidSteerKinematics, Twist, WheelSpeeds  # noqa: E402
from odometry import SkidSteerOdometry, wrap_angle  # noqa: E402
from pid import PID  # noqa: E402
import protocol  # noqa: E402


class TestPID(unittest.TestCase):
    def test_sign_of_output(self):
        pid = PID(kp=10.0)
        self.assertGreater(pid.update(1.0, 0.0, 0.1), 0)   # under -> push up
        pid.reset()
        self.assertLess(pid.update(0.0, 1.0, 0.1), 0)      # over -> push down

    def test_output_clamped(self):
        pid = PID(kp=1e9, output_limit=4095)
        self.assertEqual(pid.update(1.0, 0.0, 0.1), 4095)

    def test_feedforward(self):
        pid = PID(feedforward=2600.0)
        # zero error -> output is pure feedforward on the setpoint
        self.assertAlmostEqual(pid.update(1.0, 1.0, 0.1), 2600.0, places=6)

    def test_integral_term_clamped(self):
        # integral_limit bounds the *contribution* ki*integral (output units).
        pid = PID(ki=1000.0, integral_limit=1500.0, output_limit=1e9)
        out = 0.0
        for _ in range(100):
            out = pid.update(10.0, 0.0, 0.1)
        self.assertAlmostEqual(out, 1500.0, delta=1e-6)          # I-term capped
        self.assertLessEqual(abs(pid.ki * pid._integral), 1500.0 + 1e-6)

    def test_no_windup_during_saturation(self):
        # Command an unreachable setpoint so the output pins at the limit while
        # the error stays large. A naive integrator would wind up; ours must
        # not, so when the setpoint drops the output returns to ~0 immediately.
        pid = PID(kp=100.0, ki=500.0, kd=0.0,
                  output_limit=4095.0, integral_limit=4095.0)
        for _ in range(300):                       # saturated the whole time
            self.assertEqual(pid.update(100.0, 0.0, 0.02), 4095.0)
        # Integrator stayed bounded (no runaway).
        self.assertLessEqual(abs(pid.ki * pid._integral), 4095.0 + 1e-6)
        # Setpoint drops to zero: output must not stay pinned from stored windup.
        out = pid.update(0.0, 0.0, 0.02)
        self.assertLess(abs(out), 100.0)

    def test_integrator_still_unwinds(self):
        # Sanity: when the error reverses, the integrator is allowed to move
        # back (conditional integration must not freeze the recovery path).
        pid = PID(ki=100.0, output_limit=1e9)
        for _ in range(50):
            pid.update(1.0, 0.0, 0.05)             # build positive integral
        high = pid._integral
        for _ in range(50):
            pid.update(-1.0, 0.0, 0.05)            # reverse
        self.assertLess(pid._integral, high)


class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.kin = SkidSteerKinematics(WheelGeometry())

    def test_roundtrip(self):
        t = Twist(linear=0.23, angular=0.7)
        w = self.kin.inverse(t)
        back = self.kin.forward(w)
        self.assertAlmostEqual(back.linear, t.linear, places=9)
        self.assertAlmostEqual(back.angular, t.angular, places=9)

    def test_straight(self):
        w = self.kin.inverse(Twist(linear=0.3, angular=0.0))
        self.assertAlmostEqual(w.left, w.right)
        self.assertAlmostEqual(w.left, 0.3)

    def test_spin_in_place(self):
        w = self.kin.inverse(Twist(linear=0.0, angular=1.0))
        self.assertAlmostEqual(w.left, -w.right)


class TestOdometry(unittest.TestCase):
    def setUp(self):
        self.odom = SkidSteerOdometry(WheelGeometry())

    def test_straight_line(self):
        for _ in range(100):
            self.odom.update_from_distances(0.01, 0.01, 0.02)
        self.assertAlmostEqual(self.odom.pose.x, 1.0, places=6)
        self.assertAlmostEqual(self.odom.pose.y, 0.0, places=6)
        self.assertAlmostEqual(self.odom.pose.theta, 0.0, places=6)

    def test_spin_in_place(self):
        track = self.odom.track
        # d_theta per step = (dR - dL)/track ; make a known rotation
        d = 0.001
        steps = 200
        for _ in range(steps):
            self.odom.update_from_distances(-d, d, 0.01)
        expected = wrap_angle((2 * d / track) * steps)
        self.assertAlmostEqual(self.odom.pose.theta, expected, places=6)
        self.assertAlmostEqual(self.odom.pose.x, 0.0, places=6)
        self.assertAlmostEqual(self.odom.pose.y, 0.0, places=6)

    def test_arc_matches_analytic_circle(self):
        # Constant per-side speeds trace a circle of radius R = v/w.
        vl, vr = 0.10, 0.20
        track = self.odom.track
        v = (vr + vl) / 2.0
        w = (vr - vl) / track
        R = v / w
        dt = 0.0005
        t_quarter = (math.pi / 2.0) / w
        n = int(round(t_quarter / dt))
        for _ in range(n):
            self.odom.update_from_distances(vl * dt, vr * dt, dt)
        # After a quarter turn about center (0, R): x=R, y=R, theta=pi/2
        self.assertAlmostEqual(self.odom.pose.theta, math.pi / 2, places=2)
        self.assertAlmostEqual(self.odom.pose.x, R, places=2)
        self.assertAlmostEqual(self.odom.pose.y, R, places=2)


class TestEncoders(unittest.TestCase):
    def test_quadrature_direction(self):
        enc = SimulatedEncoder(0, 0)
        enc.feed_states([0b00, 0b10, 0b11, 0b01, 0b00])
        self.assertEqual(enc.count, 4)
        enc.reset()
        enc.feed_states([0b00, 0b01, 0b11, 0b10, 0b00])
        self.assertEqual(enc.count, -4)

    def test_side_aggregation_and_signs(self):
        sides = SideMapping(signs={"M1": 1, "M2": 1, "M3": -1, "M4": -1})
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ("M1", "M2", "M3", "M4")}
        wheels = WheelEncoders(sides, encoders=encs)
        encs["M1"].add(100); encs["M2"].add(100)   # left forward
        encs["M3"].add(-80); encs["M4"].add(-80)   # right: raw -80, sign -1 -> +80
        left, right = wheels.side_counts()
        self.assertAlmostEqual(left, 100.0)
        self.assertAlmostEqual(right, 80.0)
        # read_reset returns the same and zeroes them
        left2, right2 = wheels.read_reset_sides()
        self.assertAlmostEqual(left2, 100.0)
        self.assertAlmostEqual(right2, 80.0)
        self.assertEqual(wheels.side_counts(), (0.0, 0.0))


class _RecordingMotor:
    """Stand-in for Motor.setMotorModel that records the last duties."""
    def __init__(self):
        self.last = (0, 0, 0, 0)

    def setMotorModel(self, d1, d2, d3, d4):
        self.last = (d1, d2, d3, d4)


class TestDriveControllerClosedLoop(unittest.TestCase):
    def _make(self):
        sides = CONFIG.sides
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ("M1", "M2", "M3", "M4")}
        wheels = WheelEncoders(sides, encoders=encs)
        motor = _RecordingMotor()
        plant = SimulatedDrivePlant(wheels, CONFIG)
        ctrl = DriveController(motor, wheels, CONFIG, plant=plant)
        return ctrl

    def test_forward_velocity_converges(self):
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.set_twist(0.2, 0.0)
        tel = {}
        for _ in range(400):        # ~8 s of simulated time
            tel = ctrl.step(dt)
        self.assertAlmostEqual(tel["wheel_speed"]["left"], 0.2, delta=0.02)
        self.assertAlmostEqual(tel["wheel_speed"]["right"], 0.2, delta=0.02)
        self.assertGreater(tel["pose"]["x"], 1.0)          # travelled forward
        self.assertAlmostEqual(tel["pose"]["y"], 0.0, delta=0.05)

    def test_spin_changes_heading(self):
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.set_twist(0.0, 1.0)
        tel = {}
        for _ in range(60):         # ~1.2 s: stays below pi, so no wrap
            tel = ctrl.step(dt)
        self.assertGreater(tel["pose"]["theta"], 0.3)
        self.assertLess(tel["pose"]["theta"], math.pi)
        self.assertAlmostEqual(tel["pose"]["x"], 0.0, delta=0.05)

    def test_release_stops_actuation(self):
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.set_twist(0.2, 0.0)
        ctrl.step(dt)
        ctrl.release()
        tel = ctrl.step(dt)
        self.assertFalse(tel["engaged"])
        self.assertEqual(tel["duty"], {"left": 0, "right": 0})

    def _run_until_move_done(self, ctrl, dt, max_steps=6000):
        tel = {}
        for _ in range(max_steps):
            tel = ctrl.step(dt)
            if not tel["goal_active"]:
                return tel
        return tel

    def test_drive_distance_reaches_target(self):
        # Position PID: drives 3 m straight and stops, closed on the encoders.
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.drive_distance(3.0)
        tel = self._run_until_move_done(ctrl, dt)
        self.assertFalse(tel["goal_active"])
        self.assertAlmostEqual(tel["pose"]["x"], 3.0, delta=0.02)
        self.assertAlmostEqual(tel["pose"]["y"], 0.0, delta=0.02)
        # Holds still afterwards.
        for _ in range(20):
            tel = ctrl.step(dt)
        self.assertEqual(tel["duty"], {"left": 0, "right": 0})

    def test_drive_distance_short_move(self):
        # The exact case from the field: 0.24 m straight.
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.drive_distance(0.24)
        tel = self._run_until_move_done(ctrl, dt)
        self.assertAlmostEqual(tel["pose"]["x"], 0.24, delta=0.01)
        self.assertAlmostEqual(tel["pose"]["theta"], 0.0, delta=0.02)

    def test_drive_distance_reverse(self):
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.drive_distance(-1.0)
        tel = self._run_until_move_done(ctrl, dt)
        self.assertAlmostEqual(tel["pose"]["x"], -1.0, delta=0.02)

    def test_turn_in_place_reaches_angle(self):
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.turn_in_place(90.0)
        tel = self._run_until_move_done(ctrl, dt)
        self.assertFalse(tel["goal_active"])
        # The move stops when each side is within `tolerance` (1 cm) of its
        # target; on a 0.151 m track that maps to up to ~2*tol/track rad of
        # heading, so the achievable angle band is ~0.13 rad, not tighter.
        band = 2 * CONFIG.position.tolerance / CONFIG.wheel.track + 0.02
        self.assertAlmostEqual(tel["pose"]["theta"], math.pi / 2, delta=band)
        self.assertAlmostEqual(tel["pose"]["x"], 0.0, delta=0.03)

    def test_raw_encoder_totals_exposed(self):
        # Telemetry must surface raw per-motor counts for calibration.
        ctrl = self._make()
        dt = 1.0 / CONFIG.control.loop_hz
        ctrl.drive_distance(0.5)
        tel = self._run_until_move_done(ctrl, dt)
        enc = tel["encoders"]
        self.assertEqual(set(enc.keys()), {"M1", "M2", "M3", "M4"})
        # Telemetry exposes RAW counts; the raw sign depends on wiring
        # (SideMapping.signs), so a forward move is verified on the POST-sign
        # count -- robust to any per-motor sign calibration.
        signs = CONFIG.sides.signs
        for tag, total in enc.items():
            self.assertGreater(total * signs.get(tag, 1), 0,
                               f"{tag} did not count forward")


class TestProtocol(unittest.TestCase):
    def test_parse_legacy(self):
        cmd = protocol.parse("CMD_MOTOR#1000#-500#1000#-500")
        self.assertEqual(cmd.name, "motor")
        self.assertEqual(cmd.arg_int(0), 1000)
        self.assertEqual(cmd.arg_int(1), -500)

    def test_parse_json(self):
        cmd = protocol.parse('{"type":"drive","linear":0.3,"angular":-0.2}')
        self.assertEqual(cmd.name, "drive")
        self.assertAlmostEqual(cmd.num("linear", 0), 0.3)
        self.assertAlmostEqual(cmd.num("angular", 1), -0.2)

    def test_parse_garbage(self):
        self.assertIsNone(protocol.parse(""))
        self.assertIsNone(protocol.parse("   "))
        self.assertIsNone(protocol.parse("{not json"))

    def test_router_dispatch(self):
        router = protocol.CommandRouter()
        seen = []
        router.register("motor", lambda c: seen.append(c.arg_int(0)))
        router.dispatch("CMD_MOTOR#42#0#0#0")
        router.dispatch('{"type":"motor","duty":[7,0,0,0]}')  # handler ignores kwargs
        self.assertEqual(seen[0], 42)

    def test_telemetry_message(self):
        import json
        msg = protocol.telemetry_message(battery=7.9, mode="one",
                                         drive={"pose": {"x": 1.0}})
        obj = json.loads(msg)
        self.assertEqual(obj["type"], "telemetry")
        self.assertEqual(obj["battery"], 7.9)
        self.assertEqual(obj["drive"]["pose"]["x"], 1.0)


class TestProtocolDriveWiring(unittest.TestCase):
    """Mirror server.py's router wiring for the closed-loop drive path."""

    def test_json_drive_engages_controller(self):
        sides = CONFIG.sides
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ("M1", "M2", "M3", "M4")}
        wheels = WheelEncoders(sides, encoders=encs)
        motor = _RecordingMotor()
        plant = SimulatedDrivePlant(wheels, CONFIG)
        ctrl = DriveController(motor, wheels, CONFIG, plant=plant)

        router = protocol.CommandRouter()
        router.register("drive",
                        lambda c: ctrl.set_twist(c.num("linear", 0),
                                                 c.num("angular", 1)))
        router.register("reset_odometry", lambda c: ctrl.reset_odometry())

        # A client sends a JSON velocity command over the wire.
        router.dispatch('{"type":"drive","linear":0.2,"angular":0.0}')
        dt = 1.0 / CONFIG.control.loop_hz
        tel = {}
        for _ in range(400):
            tel = ctrl.step(dt)
        self.assertTrue(tel["engaged"])
        self.assertAlmostEqual(tel["wheel_speed"]["left"], 0.2, delta=0.03)
        self.assertGreater(tel["pose"]["x"], 1.0)

        # And can zero the odometry.
        router.dispatch('{"type":"reset_odometry"}')
        self.assertEqual(ctrl.odom.pose.x, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
