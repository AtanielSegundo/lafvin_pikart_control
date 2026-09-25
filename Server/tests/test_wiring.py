#!/usr/bin/python3
"""
Cross-process command wiring.

The command path is now two halves: a handler in P_web decides which subsystem
owns a command and enqueues it under some name, and a handler in that subsystem
acts on that name. Nothing at import time connects the two, so a typo on either
side produces the worst possible symptom -- the UI works, the queue accepts the
message, and the command simply never happens.

These tests close that gap by reading the names P_web actually enqueues straight
out of its source and checking each one against the receiving side's declared
handler set. They also pin the end-to-end path for the drive commands, applied to
a real DriveController through a real queue.
"""
import ast
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER_DIR = os.path.dirname(HERE)
sys.path.insert(0, SERVER_DIR)

import ipc as ipc_mod                                            # noqa: E402
import proc_aux                                                  # noqa: E402
import proc_control                                              # noqa: E402
import proc_sensors                                              # noqa: E402
import protocol                                                  # noqa: E402
from config import CONFIG, ENCODER_TAGS, SideMapping              # noqa: E402
from drive_controller import DriveController, SimulatedDrivePlant  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders              # noqa: E402


def _pump_until(applier_or_queue, predicate, timeout: float = 2.0) -> None:
    """Poll until ``predicate()`` holds.

    ``multiprocessing.Queue`` puts are handed to a feeder thread, so an item is
    not gettable the instant it is queued. The control loop tolerates that (it
    drains again 50 ms later); a test asserting on one command cannot, so it
    polls instead of sleeping a guessed interval.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if callable(getattr(applier_or_queue, "pump", None)):
            applier_or_queue.pump()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for the queued command to arrive")


def _drain_at_least(q, count: int, timeout: float = 2.0) -> list:
    """Drain ``q`` until at least ``count`` items have been collected."""
    items = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(items) < count:
        items.extend(ipc_mod.drain(q))
        if len(items) < count:
            time.sleep(0.005)
    return items


def _enqueued_names(method_name: str) -> set:
    """Every literal command name passed to ``self.<method_name>(...)`` in
    server.py, read from the AST.

    Source inspection rather than instantiating Server, because constructing it
    forks four child processes and opens the I2C bus.
    """
    with open(os.path.join(SERVER_DIR, "server.py"), encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr != method_name:
            continue
        if not node.args:
            continue
        first = node.args[0]
        if isinstance(first, ast.Constant) and isinstance(first.value, str):
            names.add(first.value)
    return names


class _StubDrive:
    """Records calls instead of driving anything."""

    def __init__(self):
        self.calls = []
        self.encoders = type("E", (), {"sides": SideMapping()})()

    def __getattr__(self, name):
        def recorder(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return recorder


class _StubMotor:
    def __init__(self):
        self.last = None
        self.rotate_stopped = 0

    def setMotorModel(self, *d):
        self.last = d

    def stop_rotate(self):
        self.rotate_stopped += 1

    def Rotate(self, angle):
        pass


class _StubServo:
    def __init__(self):
        self.angles = {}

    def setServoPwm(self, channel, angle, error=10):
        self.angles[str(channel)] = int(angle)


class TestCommandWiring(unittest.TestCase):
    def setUp(self):
        self.ipc = ipc_mod.IPC(CONFIG)
        self.drive = _StubDrive()
        self.motor = _StubMotor()
        self.servo = _StubServo()
        self.applier = proc_control.ControlApplier(self.drive, self.motor,
                                                   self.servo, self.ipc, CONFIG)

    def test_every_control_command_has_a_handler(self):
        enqueued = _enqueued_names("_to_control")
        self.assertTrue(enqueued, "found no _to_control calls -- test is broken")
        missing = enqueued - self.applier.handled
        self.assertEqual(missing, set(),
                         f"server.py enqueues {sorted(missing)} to P_control but "
                         f"ControlApplier has no handler for them")

    def test_every_aux_command_has_a_handler(self):
        enqueued = _enqueued_names("_to_aux")
        self.assertTrue(enqueued)
        missing = enqueued - proc_aux.HANDLED
        self.assertEqual(missing, set(),
                         f"server.py enqueues {sorted(missing)} to P_aux but "
                         f"proc_aux.HANDLED does not list them")

    def test_every_sensors_command_has_a_handler(self):
        enqueued = _enqueued_names("_to_sensors")
        self.assertTrue(enqueued)
        missing = enqueued - proc_sensors.HANDLED
        self.assertEqual(missing, set(),
                         f"server.py enqueues {sorted(missing)} to P_sensors but "
                         f"proc_sensors.HANDLED does not list them")

    def test_no_handler_is_dead_code(self):
        """The other direction: a handler nothing can reach is either a missing
        route in server.py or leftover code."""
        reachable = (_enqueued_names("_to_control")
                     # RemoteMotor / RemoteServo in P_aux reach these directly,
                     # bypassing server.py's routing.
                     | {"motor", "servo"})
        unreachable = self.applier.handled - reachable
        self.assertEqual(unreachable, set(),
                         f"ControlApplier handles {sorted(unreachable)} but "
                         f"nothing sends them")

    def test_unknown_command_is_logged_not_swallowed(self):
        ipc_mod.put_drop_oldest(self.ipc.control_q,
                                protocol.Command(name="does_not_exist"))
        events = []

        def seen():
            events.extend(ipc_mod.drain(self.ipc.event_q))
            return any("unknown command" in str(e.get("message", ""))
                       for e in events if isinstance(e, dict))

        _pump_until(self.applier, seen)


class TestRemoteProxies(unittest.TestCase):
    """P_aux must reach the PCA9685 only through P_control."""

    def setUp(self):
        self.ipc = ipc_mod.IPC(CONFIG)

    def test_remote_motor_forwards_duty(self):
        motor = ipc_mod.RemoteMotor(self.ipc.control_q)
        motor.setMotorModel(100, 100, -200, -200)
        cmds = _drain_at_least(self.ipc.control_q, 1)
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0].name, "motor")
        self.assertEqual(cmds[0].get("duty"), [100, 100, -200, -200])

    def test_remote_servo_forwards_and_remembers(self):
        servo = ipc_mod.RemoteServo(self.ipc.control_q)
        servo.setServoPwm("1", 45)
        self.assertEqual(servo.angles, {"1": 45})
        cmds = _drain_at_least(self.ipc.control_q, 1)
        self.assertEqual(cmds[0].name, "servo")
        self.assertEqual(cmds[0].get("angle"), 45)

    def test_remote_adc_maps_channels(self):
        self.ipc.publish_adc(11.5, 1.23, 4.56)
        adc = ipc_mod.RemoteAdc(self.ipc)
        self.assertAlmostEqual(adc.recvADC(0), 1.23, places=4)
        self.assertAlmostEqual(adc.recvADC(1), 4.56, places=4)
        # Callers do recvADC(2) * 5 to get volts, so the raw read is volts / 5.
        self.assertAlmostEqual(adc.recvADC(2) * 5, 11.5, places=4)


class TestControlApplierBehaviour(unittest.TestCase):
    """The applier against a real controller and a real queue."""

    def setUp(self):
        self.ipc = ipc_mod.IPC(CONFIG)
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        wheels = WheelEncoders(sides, encoders=encs)
        self.motor = _StubMotor()
        self.ctrl = DriveController(self.motor, wheels, CONFIG,
                                    plant=SimulatedDrivePlant(wheels, CONFIG))
        self.applier = proc_control.ControlApplier(self.ctrl, self.motor,
                                                   _StubServo(), self.ipc, CONFIG)
        # Count applied commands, so _send can wait for arrival rather than
        # guessing at the feeder thread's latency.
        self.applier_calls = []
        _real_dispatch = self.applier.router.dispatch

        def counting_dispatch(message):
            result = _real_dispatch(message)
            self.applier_calls.append(message)
            return result

        self.applier.router.dispatch = counting_dispatch

    def _send(self, name, **kwargs):
        """Enqueue a command and pump until it has actually been applied."""
        before = len(self.applier_calls)
        ipc_mod.put_drop_oldest(self.ipc.control_q,
                                protocol.Command(name=name, kwargs=kwargs))
        _pump_until(self.applier, lambda: len(self.applier_calls) > before)

    def test_drive_engages_the_controller(self):
        self._send("drive", linear=0.2, angular=0.0)
        tel = self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self.assertTrue(tel["engaged"])
        self.assertAlmostEqual(tel["target"]["linear"], 0.2)

    def test_raw_motor_releases_the_pid_first(self):
        """Raw duty and the PID must not fight: the old handler called release()
        before writing, and that has to survive the queue hop."""
        self._send("drive", linear=0.3, angular=0.0)
        self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self._send("motor", duty=[1000, 1000, 1000, 1000])
        self.assertEqual(self.motor.last, (1000, 1000, 1000, 1000))
        tel = self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self.assertFalse(tel["engaged"])

    def test_turn_and_distance_start_moves(self):
        self._send("drive_distance", distance=0.4, speed=0.2)
        self.assertTrue(self.ctrl.move_active())
        self._send("release")
        self.assertFalse(self.ctrl.move_active())
        self._send("turn", angle=90.0, speed=1.0)
        self.assertTrue(self.ctrl.move_active())

    def test_set_sign_mutates_the_live_mapping(self):
        signs = self.ctrl.encoders.sides.signs
        before = signs["M1"]
        self._send("set_sign", motor="M1", sign=-before)
        self.assertEqual(signs["M1"], -before)
        self._send("set_sign", signs={"M1": 1, "M2": 1, "M3": -1, "M4": -1})
        self.assertEqual(dict(signs), {"M1": 1, "M2": 1, "M3": -1, "M4": -1})

    def test_stop_brakes_and_releases(self):
        self._send("drive", linear=0.3, angular=0.0)
        self._send("stop")
        self.assertEqual(self.motor.last, (0, 0, 0, 0))
        self.assertGreaterEqual(self.motor.rotate_stopped, 1)

    def test_goto_accepts_a_null_theta(self):
        """theta is optional and arrives as JSON null; float(None) would raise."""
        self._send("goto", x=1.0, y=0.5, theta=None)
        self.assertTrue(self.ctrl.move_active())

    def test_reset_odometry(self):
        self.ctrl.odom.pose.x = 5.0
        self._send("reset_odometry")
        self.assertEqual(self.ctrl.odom.pose.x, 0.0)

    def test_drive_credits_the_ingress_timestamp(self):
        """command_timeout is a dead-man switch: it must measure how long since
        the operator was last heard from, queue hop included."""
        issued = time.monotonic() - 0.5 * CONFIG.control.command_timeout
        self._send("drive", linear=0.2, angular=0.0, ts=issued)
        self.assertAlmostEqual(self.ctrl._target_time, issued, places=4)

    def test_drive_ignores_an_implausible_timestamp(self):
        """A nonsensical timestamp (clock skew) must not wedge teleop by making
        every command instantly stale -- it falls back to now."""
        for bogus in (time.monotonic() + 60.0,       # future
                      time.monotonic() - 3600.0):    # ancient
            self._send("drive", linear=0.2, angular=0.0, ts=bogus)
            self.assertAlmostEqual(self.ctrl._target_time, time.monotonic(),
                                   delta=0.5)

    def test_a_stale_drive_command_hits_the_safety_stop(self):
        """The behaviour the timestamp protects: an input older than
        command_timeout stays engaged but commands zero velocity."""
        issued = time.monotonic() - 3.0 * CONFIG.control.command_timeout
        self._send("drive", linear=0.3, angular=0.0, ts=issued)
        tel = self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self.assertTrue(tel["engaged"])
        self.assertEqual(tel["target"]["linear"], 0.0)

    def test_drive_without_a_timestamp_still_works(self):
        """Single-process callers and the tests pass no ts."""
        self._send("drive", linear=0.2, angular=0.0)
        tel = self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self.assertTrue(tel["engaged"])
        self.assertAlmostEqual(tel["target"]["linear"], 0.2)

    def test_pump_survives_a_bad_command(self):
        """One malformed command must not take the control loop down with it."""
        self._send("drive", linear="not-a-number")
        self._send("drive", linear=0.1, angular=0.0)
        tel = self.ctrl.step(1.0 / CONFIG.control.loop_hz)
        self.assertTrue(tel["engaged"])


class TestTelemetryEnvelope(unittest.TestCase):
    def test_envelope_carries_servo_and_signs(self):
        """step() stays pure, so the servo angles and the live encoder signs ride
        in the envelope around it; P_web's get_telemetry depends on both keys."""
        ipc = ipc_mod.IPC(CONFIG)
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        wheels = WheelEncoders(sides, encoders=encs)
        motor, servo = _StubMotor(), _StubServo()
        ctrl = DriveController(motor, wheels, CONFIG)
        servo.setServoPwm("0", 90)

        snapshot = ctrl.step(1.0 / CONFIG.control.loop_hz)
        ipc_mod.put_drop_oldest(ipc.telemetry_q, {
            "drive": snapshot, "servo": dict(servo.angles),
            "signs": dict(sides.signs), "ts": 1.0})
        envelope = _drain_at_least(ipc.telemetry_q, 1)[-1]

        self.assertIn("drive", envelope)
        self.assertEqual(envelope["servo"], {"0": 90})
        self.assertEqual(envelope["signs"], dict(sides.signs))
        for key in ("pose", "duty", "engaged", "encoders", "gyro",
                    "front_distance_cm", "goal_active"):
            self.assertIn(key, envelope["drive"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
