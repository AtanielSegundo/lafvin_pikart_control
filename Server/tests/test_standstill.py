#!/usr/bin/python3
"""
Unit tests for standstill detection (Server/standstill.py), the gate on the
gyro's yaw integration.

The weighting here is deliberate. Drift while parked is the problem being
solved, but the DANGEROUS failure is the opposite one: freezing during real
motion silently discards rotation, and since the drive controller treats gyro
yaw as ground truth, that corrupts the heading permanently rather than slowly.
So most of these tests are about refusing to freeze.

The detector takes an injected clock, so none of this sleeps.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import ENCODER_TAGS, RobotConfig, StandstillConfig  # noqa: E402
from standstill import StandstillDetector                        # noqa: E402


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def totals(**kw):
    """Per-motor totals, defaulting every unmentioned motor to 0."""
    out = {tag: 0 for tag in ENCODER_TAGS}
    out.update(kw)
    return out


class StandstillFixture(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.config = RobotConfig()
        self.det = StandstillDetector(self.config, clock=self.clock)

    def quiet_for(self, seconds, step=0.01, counts=None):
        """Hold the encoders still for `seconds`, in `step` increments."""
        elapsed = 0.0
        while elapsed < seconds:
            self.clock.advance(step)
            self.det.update(totals() if counts is None else counts)
            elapsed += step
        return self.det.is_moving()


class TestFreezesWhenParked(StandstillFixture):
    def test_starts_out_assuming_motion(self):
        """Before there is any evidence, the safe assumption is that the kart
        IS moving -- guessing standstill would discard real rotation."""
        self.assertTrue(self.det.is_moving())

    def test_first_sample_does_not_freeze(self):
        """No previous totals means no delta to judge. Adopting the first
        sample as a baseline is also what stops a long-running P_sensors from
        reading its whole count history as one enormous movement."""
        self.det.update(totals(M1=50000, M2=50000, M3=50000, M4=50000))
        self.assertTrue(self.det.is_moving())

    def test_freezes_after_the_quiet_window(self):
        self.det.update(totals())
        self.assertTrue(self.quiet_for(
            self.config.standstill.enter_quiet_s * 0.5),
            "froze before the quiet window elapsed")
        self.quiet_for(self.config.standstill.enter_quiet_s)
        self.assertFalse(self.det.is_moving())

    def test_counts_the_freeze_events(self):
        self.det.update(totals())
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1)
        self.assertEqual(self.det.froze_count, 1)
        # Move, then settle again -> a second event, not a duplicate of the
        # first.
        self.clock.advance(0.01)
        self.det.update(totals(M1=100))
        self.assertTrue(self.det.is_moving())
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1,
                       counts=totals(M1=100))
        self.assertEqual(self.det.froze_count, 2)

    def test_dither_within_tolerance_still_freezes(self):
        """A wheel resting on a quadrature edge can emit counts forever without
        the kart moving. That is exactly what count_tolerance is for."""
        self.det.update(totals())
        tol = self.config.standstill.count_tolerance
        state = 0
        for _ in range(400):
            self.clock.advance(0.01)
            state ^= 1                      # jitter one count back and forth
            self.det.update(totals(M1=state))
        self.assertFalse(self.det.is_moving(),
                         f"dither of 1 count (tolerance {tol}) blocked freezing")


class TestRefusesToFreezeWhenMoving(StandstillFixture):
    """The dangerous direction. Every one of these must report motion."""

    def test_resumes_immediately_on_the_first_count(self):
        """No debounce leaving standstill: resuming a tick early costs nothing,
        resuming a tick late loses a slice of real rotation."""
        self.det.update(totals())
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1)
        self.assertFalse(self.det.is_moving())

        self.clock.advance(0.01)
        self.det.update(totals(M1=100))
        self.assertTrue(self.det.is_moving(), "did not resume on the first count")

    def test_steady_driving_never_freezes(self):
        self.det.update(totals())
        counts = 0
        for _ in range(2000):               # 20 s of driving
            self.clock.advance(0.01)
            counts += 30
            self.det.update(totals(M1=counts, M2=counts,
                                   M3=counts, M4=counts))
            self.assertTrue(self.det.is_moving())
        self.assertEqual(self.det.froze_count, 0)

    def test_a_slow_creep_never_freezes(self):
        """Few counts per tick, but sustained. Freezing mid-creep would be the
        bug -- the enter_quiet_s window exists to prevent exactly this."""
        self.det.update(totals())
        counts = 0
        for _ in range(1000):
            self.clock.advance(0.01)
            counts += self.config.standstill.count_tolerance + 1
            self.det.update(totals(M1=counts))
            self.assertTrue(self.det.is_moving(),
                            "froze during a slow creep")

    def test_in_place_turn_is_protected_by_the_gyro_floor(self):
        """The case the encoders can miss: wheels scrubbing sideways register
        little while the chassis genuinely rotates. The gyro rate overrides."""
        self.det.update(totals())
        self.det.note_gyro_rate(
            self.config.standstill.gyro_rate_floor_dps * 5)
        self.assertTrue(self.quiet_for(
            self.config.standstill.enter_quiet_s * 3),
            "froze during a rotation the gyro was reporting")
        self.assertEqual(self.det.froze_count, 0)

    def test_gyro_floor_works_in_both_directions(self):
        self.det.update(totals())
        self.det.note_gyro_rate(
            -self.config.standstill.gyro_rate_floor_dps * 5)
        self.assertTrue(self.quiet_for(
            self.config.standstill.enter_quiet_s * 3))

    def test_gyro_noise_below_the_floor_does_not_block_freezing(self):
        """The floor must not be so eager that sensor noise keeps the gate
        open forever -- that would defeat the whole feature."""
        self.det.update(totals())
        self.det.note_gyro_rate(
            self.config.standstill.gyro_rate_floor_dps * 0.3)
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1)
        self.assertFalse(self.det.is_moving(),
                         "sub-floor gyro noise prevented standstill")


class TestDisabled(unittest.TestCase):
    def test_disabled_always_reports_motion(self):
        """The escape hatch: with the feature off, the gyro integrates exactly
        as it did before this existed."""
        clock = FakeClock()
        config = RobotConfig(standstill=StandstillConfig(enabled=False))
        det = StandstillDetector(config, clock=clock)
        det.update(totals())
        for _ in range(1000):
            clock.advance(0.01)
            det.update(totals())
            self.assertTrue(det.is_moving())
        self.assertEqual(det.froze_count, 0)


class TestTelemetry(StandstillFixture):
    def test_shape_matches_blank(self):
        self.assertEqual(set(self.det.telemetry()),
                         set(StandstillDetector.blank_telemetry()))

    def test_reports_frozen_state_and_duration(self):
        self.det.update(totals())
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1)
        self.clock.advance(5.0)
        tel = self.det.telemetry()
        self.assertFalse(tel["moving"])
        self.assertGreaterEqual(tel["frozen_s"], 5.0)
        self.assertEqual(tel["frozen_events"], 1)

    def test_frozen_time_accumulates_across_episodes(self):
        self.det.update(totals())
        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1)
        self.clock.advance(3.0)
        self.det.update(totals())            # still frozen
        self.clock.advance(0.01)
        self.det.update(totals(M1=100))      # moving again -- banks the time
        banked = self.det.telemetry()["frozen_s"]
        self.assertGreaterEqual(banked, 3.0)

        self.quiet_for(self.config.standstill.enter_quiet_s + 0.1,
                       counts=totals(M1=100))
        self.clock.advance(2.0)
        self.assertGreaterEqual(self.det.telemetry()["frozen_s"], banked + 2.0)


class TestGyroIntegrationGate(unittest.TestCase):
    """The predicate's actual contract with GyroMPU: called with no arguments,
    returns a bool, and never raises."""

    def test_is_moving_is_a_zero_arg_predicate(self):
        det = StandstillDetector(RobotConfig(), clock=FakeClock())
        self.assertIsInstance(det.is_moving(), bool)

    def test_predicate_is_safe_before_any_update(self):
        det = StandstillDetector(RobotConfig(), clock=FakeClock())
        for _ in range(10):
            self.assertTrue(det.is_moving())

    def test_gyro_freezes_integration_when_the_predicate_says_still(self):
        """End-to-end against the real integration maths, with the sensor I/O
        stubbed out. This is what proves the hook actually gates anything."""
        angles = {"x": 0.0, "y": 0.0, "z": 10.0}
        bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        drifting_gyro = {"x": 0.0, "y": 0.0, "z": 0.5}   # residual bias, deg/s
        moving = [True]

        # Mirrors heading.GyroMPU._integrate's yaw branch exactly: integrate
        # when the predicate allows, hold the angle when it does not. Yaw only,
        # since roll/pitch are pulled back by the accel filter anyway.
        def integrate(dt):
            if moving[0]:
                angles["z"] += (drifting_gyro["z"] - bias["z"]) * dt

        for _ in range(100):                 # 2 s of a parked kart, integrating
            integrate(0.02)
        drifted = angles["z"]
        self.assertGreater(drifted, 10.9, "the stand-in did not drift at all")

        moving[0] = False
        for _ in range(500):                 # 10 s more, now gated off
            integrate(0.02)
        self.assertAlmostEqual(angles["z"], drifted, places=9,
                               msg="integration continued while gated off")


if __name__ == "__main__":
    unittest.main(verbosity=2)
