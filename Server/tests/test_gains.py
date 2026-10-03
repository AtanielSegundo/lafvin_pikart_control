#!/usr/bin/python3
"""
Runtime PID retuning (the web UI's tuning table).

The trap this guards is specific. ``PositionGains`` / ``HeadingGains`` are
frozen dataclasses, and the control loop consumes them in TWO different ways:

  * read live every tick -- tolerance, decel_gain, min_*_duty, max_time,
    settle_rate, settle_ticks;
  * COPIED once into the PID objects at construction -- kp, ki, kd,
    output_limit, integral_limit.

So a handler that only replaces the config would appear to work (the telemetry
would show the new numbers) while changing nothing about the terms that matter
most. Several tests below exist purely to catch that.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import dataclasses                                               # noqa: E402
import ipc as ipc_mod                                            # noqa: E402
import proc_control                                              # noqa: E402
from config import CONFIG, ENCODER_TAGS, RobotConfig, SideMapping  # noqa: E402
from drive_controller import DriveController, SimulatedDrivePlant  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders             # noqa: E402
from protocol import Command                                     # noqa: E402


class _Motor:
    last = (0, 0, 0, 0)

    def setMotorModel(self, *d):
        self.last = d

    def stop_rotate(self):
        pass


class GainsFixture(unittest.TestCase):
    def setUp(self):
        # A private RobotConfig: these tests mutate it, and the module-level
        # CONFIG is shared with every other test in the suite.
        self.config = RobotConfig()
        self.ipc = ipc_mod.IPC(self.config)
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        wheels = WheelEncoders(sides, encoders=encs)
        self.motor = _Motor()
        self.drive = DriveController(self.motor, wheels, self.config,
                                     plant=SimulatedDrivePlant(wheels,
                                                               self.config))
        self.applier = proc_control.ControlApplier(self.drive, self.motor,
                                                   None, self.ipc, self.config)

    def apply(self, section, values):
        self.applier.router.dispatch(
            Command(name="set_gains",
                    kwargs={"section": section, "values": values}))

    def logs(self, expect="", timeout=2.0):
        """Drained log messages, polling until `expect` shows up.

        multiprocessing.Queue hands items to a feeder thread, so a message
        logged microseconds ago is not yet gettable -- see ipc.drain's
        docstring. Production tolerates that; a test asserting on one line
        cannot, so it polls rather than sleeping a guessed interval.
        """
        import time
        found = []
        deadline = time.monotonic() + timeout
        while True:
            found.extend(m for m in
                         (e.get("message", "") for e in
                          ipc_mod.drain(self.ipc.event_q)
                          if isinstance(e, dict)))
            if not expect or any(expect in m for m in found):
                return found
            if time.monotonic() >= deadline:
                return found
            time.sleep(0.01)


class TestPositionGains(GainsFixture):
    def test_config_is_replaced(self):
        self.apply("position", {"kp": 7777.0})
        self.assertEqual(self.config.position.kp, 7777.0)

    def test_live_pids_are_patched_too(self):
        """The one that matters: kp/ki/kd were copied into the PIDs at
        construction, so replacing the config alone would leave the loop
        running the OLD gains while telemetry advertised the new ones."""
        self.apply("position", {"kp": 7777.0, "ki": 11.0, "kd": 22.0})
        for pid in (self.drive.pos_left, self.drive.pos_right):
            self.assertEqual(pid.kp, 7777.0)
            self.assertEqual(pid.ki, 11.0)
            self.assertEqual(pid.kd, 22.0)

    def test_limits_reach_the_pids(self):
        self.apply("position", {"output_limit": 1234.0,
                                "integral_limit": 321.0})
        for pid in (self.drive.pos_left, self.drive.pos_right):
            self.assertEqual(pid.output_limit, 1234.0)
            self.assertEqual(pid.integral_limit, 321.0)

    def test_a_new_gain_changes_the_duty_the_loop_commands(self):
        """End-to-end: the retune has to show up in what the motors are told,
        not merely in an attribute."""
        dt = 1.0 / self.config.control.loop_hz
        self.apply("position", {"kp": 200.0, "ki": 0.0, "kd": 0.0,
                                "min_move_duty": 0.0, "decel_gain": 0.0})
        self.drive.drive_distance(0.5)
        self.drive.step(dt)
        weak = abs(self.motor.last[0])

        self.drive.release()
        self.apply("position", {"kp": 20000.0})
        self.drive.drive_distance(0.5)
        self.drive.step(dt)
        strong = abs(self.motor.last[0])

        self.assertGreater(strong, weak,
                           f"kp 200 -> 20000 did not raise duty "
                           f"({weak} -> {strong})")

    def test_live_read_fields_take_effect(self):
        """tolerance is read from the config every tick rather than copied, so
        it exercises the other half of the handler."""
        self.apply("position", {"tolerance": 0.25})
        self.assertAlmostEqual(self.config.position.tolerance, 0.25)
        # A move shorter than the new tolerance finishes almost immediately.
        self.drive.drive_distance(0.05)
        for _ in range(5):
            self.drive.step(1.0 / self.config.control.loop_hz)
        self.assertFalse(self.drive.move_active(),
                         "a 5 cm move did not finish inside a 25 cm tolerance")


class TestHeadingGains(GainsFixture):
    def test_config_and_pid_are_both_updated(self):
        self.apply("heading", {"kp": 4321.0, "output_limit": 999.0})
        self.assertEqual(self.config.heading.kp, 4321.0)
        self.assertEqual(self.drive.heading_pid.kp, 4321.0)
        self.assertEqual(self.drive.heading_pid.output_limit, 999.0)

    def test_position_and_heading_are_independent(self):
        before = self.config.position.kp
        self.apply("heading", {"kp": 1111.0})
        self.assertEqual(self.config.position.kp, before,
                         "tuning heading moved the position gains")
        self.assertEqual(self.drive.pos_left.kp, before)

    def test_bool_field_round_trips(self):
        self.apply("heading", {"pulse_floor": 0})
        self.assertIs(self.config.heading.pulse_floor, False)
        self.apply("heading", {"pulse_floor": 1})
        self.assertIs(self.config.heading.pulse_floor, True)

    def test_int_field_is_coerced(self):
        """settle_ticks counts loop iterations; a float would break the
        `>=` comparison in a way that is awkward to debug."""
        self.apply("heading", {"settle_ticks": 4.7})
        self.assertIsInstance(self.config.heading.settle_ticks, int)
        self.assertEqual(self.config.heading.settle_ticks, 5)


class TestValidation(GainsFixture):
    def test_unknown_section_is_refused(self):
        self.apply("nonsense", {"kp": 1.0})
        self.assertTrue(any("unknown section" in m
                            for m in self.logs("unknown section")))

    def test_unknown_field_is_refused_and_named(self):
        before = self.config.position.kp
        self.apply("position", {"kp": 5000.0, "bogus_field": 1.0})
        self.assertEqual(self.config.position.kp, 5000.0, "good field dropped")
        self.assertTrue(any("bogus_field" in m
                            for m in self.logs("bogus_field")))
        self.assertNotEqual(before, 5000.0)

    def test_out_of_range_is_refused(self):
        before = self.config.position.output_limit
        self.apply("position", {"output_limit": 99999.0})
        self.assertEqual(self.config.position.output_limit, before,
                         "a duty above the PWM range was accepted")
        self.assertTrue(any("outside" in m for m in self.logs("outside")))

    def test_nan_and_garbage_are_refused(self):
        before = self.config.heading.kp
        self.apply("heading", {"kp": float("nan")})
        self.assertEqual(self.config.heading.kp, before)
        self.apply("heading", {"kp": "abc"})
        self.assertEqual(self.config.heading.kp, before)

    def test_a_bad_field_does_not_block_the_good_ones(self):
        """Partial application on purpose: refusing the whole payload because
        one box was mistyped would make the table frustrating to use."""
        self.apply("position", {"kp": 8000.0, "output_limit": 99999.0})
        self.assertEqual(self.config.position.kp, 8000.0)
        self.assertNotEqual(self.config.position.output_limit, 99999.0)

    def test_empty_payload_is_a_no_op(self):
        before = dataclasses.asdict(self.config.position)
        self.apply("position", {})
        self.assertEqual(dataclasses.asdict(self.config.position), before)

    def test_every_advertised_field_actually_exists(self):
        """GAIN_LIMITS is the contract the UI builds its table from. A name in
        there that is not a real dataclass field would set an attribute nobody
        reads -- the change would 'work' and do nothing."""
        for section in ("position", "heading"):
            real = {f.name for f in
                    dataclasses.fields(getattr(self.config, section))}
            advertised = set(proc_control.ControlApplier.GAIN_LIMITS[section])
            self.assertEqual(advertised - real, set(),
                             f"{section}: GAIN_LIMITS names non-existent "
                             f"field(s) {sorted(advertised - real)}")

    def test_defaults_sit_inside_their_own_limits(self):
        """If a shipped default is outside the bounds, the UI shows a value the
        operator cannot re-enter after changing it."""
        for section in ("position", "heading"):
            gains = getattr(self.config, section)
            for field, (lo, hi) in \
                    proc_control.ControlApplier.GAIN_LIMITS[section].items():
                value = float(getattr(gains, field))
                self.assertTrue(lo <= value <= hi,
                                f"{section}.{field} default {value:g} is "
                                f"outside [{lo:g}, {hi:g}]")


class TestTelemetry(GainsFixture):
    def test_gains_serialise_to_plain_scalars(self):
        payload = proc_control.gains_as_dict(self.config.position)
        self.assertIn("kp", payload)
        for key, value in payload.items():
            self.assertIsInstance(value, (int, float, bool),
                                  f"{key} is not JSON-safe: {type(value)}")

    def test_serialised_gains_survive_json(self):
        import json
        payload = {"position": proc_control.gains_as_dict(self.config.position),
                   "heading": proc_control.gains_as_dict(self.config.heading)}
        self.assertEqual(json.loads(json.dumps(payload)), payload)

    def test_telemetry_reflects_a_change(self):
        self.apply("heading", {"kp": 2468.0})
        payload = proc_control.gains_as_dict(self.config.heading)
        self.assertEqual(payload["kp"], 2468.0)

    def test_every_ui_field_is_published(self):
        """The table's grey 'live' column reads these. A tunable field missing
        from the payload would render as '--' forever."""
        for section in ("position", "heading"):
            published = set(proc_control.gains_as_dict(
                getattr(self.config, section)))
            tunable = set(proc_control.ControlApplier.GAIN_LIMITS[section])
            self.assertEqual(tunable - published, set(),
                             f"{section}: not published {tunable - published}")


class TestRouting(unittest.TestCase):
    def test_set_gains_is_an_accepted_command_name(self):
        import protocol
        self.assertEqual(protocol.ALIASES.get("set_gains"), "set_gains")
        cmd = protocol.parse('{"type":"set_gains","section":"position",'
                             '"values":{"kp":1.0}}')
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.name, "set_gains")
        self.assertEqual(cmd.get("section"), "position")
        self.assertEqual(cmd.get("values"), {"kp": 1.0})


if __name__ == "__main__":
    unittest.main(verbosity=2)
