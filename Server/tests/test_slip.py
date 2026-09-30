#!/usr/bin/python3
"""
Unit tests for the gyro-vs-encoder slip detector (Server/slip.py) and its
wiring into the control loop.

The behaviours worth pinning are the ones a careless implementation gets wrong:

  * an honest fast turn must NOT read as slip (the threshold scales with rate);
  * a single-tick disagreement must not latch (the two rotation deltas are
    sampled over slightly different windows);
  * losing the gyro must not read as "the slip ended", and must not read as
    "not slipping" either -- ``active`` exists so a consumer can tell "no slip"
    from "cannot tell";
  * a parked kart must not latch on noise;
  * the documented blind spot -- both sides slipping equally in a straight
    line -- is asserted as a KNOWN limitation, so nobody later mistakes the
    flag for a general odometry-trust signal.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CONFIG, ENCODER_TAGS, SideMapping           # noqa: E402
from drive_controller import DriveController, SimulatedDrivePlant  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders            # noqa: E402
from slip import SlipDetector                                   # noqa: E402

DT = 1.0 / CONFIG.control.loop_hz


def rad(deg):
    return math.radians(deg)


class TestSlipDetector(unittest.TestCase):
    def setUp(self):
        self.det = SlipDetector(CONFIG)

    def _feed(self, enc_dps, gyro_dps, ticks):
        """Run `ticks` steps at the given rates in deg/s."""
        for _ in range(ticks):
            self.det.update(rad(enc_dps) * DT,
                            None if gyro_dps is None else rad(gyro_dps) * DT,
                            DT)
        return self.det.slipping

    # -- agreement ---------------------------------------------------------
    def test_perfect_agreement_never_slips(self):
        self.assertFalse(self._feed(60.0, 60.0, 50))
        self.assertEqual(self.det.events, 0)

    def test_parked_kart_does_not_latch(self):
        """Both sources ~0: nothing is claimed, so nothing is concluded."""
        self.assertFalse(self._feed(0.0, 0.0, 100))

    def test_fast_honest_turn_is_not_slip(self):
        """A hard turn disagrees by more in absolute terms than a slow one --
        quantisation of whole counts plus 20 Hz sampling of a ramping rate. The
        proportional term is what stops this reading as slip."""
        cfg = CONFIG.slip
        for rate in (60.0, 120.0, 240.0):
            self.det.reset()
            tolerated = math.degrees(cfg.rate_tolerance
                                     + cfg.rate_fraction * rad(rate))
            # Disagree by just under what the scaled threshold allows.
            self.assertFalse(self._feed(rate + tolerated * 0.9, rate, 20),
                             f"{rate} deg/s turn flagged as slip")

    # -- detection ---------------------------------------------------------
    def test_wheels_claiming_rotation_the_chassis_did_not_make(self):
        """The signature of an in-place scrub: encoders report a big rotation,
        the gyro says the body barely moved."""
        self.assertTrue(self._feed(120.0, 10.0, 10))
        self.assertTrue(self.det.slipping)
        self.assertEqual(self.det.events, 1)

    def test_chassis_rotating_while_wheels_report_nothing(self):
        """The opposite sign: pushed/kicked, or a side dropped its counts."""
        self.assertTrue(self._feed(0.0, 90.0, 10))

    def test_latches_only_after_enter_ticks(self):
        cfg = CONFIG.slip
        for _ in range(cfg.enter_ticks - 1):
            self.det.update(rad(150.0) * DT, rad(0.0) * DT, DT)
            self.assertFalse(self.det.slipping, "latched too early")
        self.det.update(rad(150.0) * DT, rad(0.0) * DT, DT)
        self.assertTrue(self.det.slipping)

    def test_single_tick_spike_is_ignored(self):
        """Sample-window misalignment produces isolated spikes; they must not
        latch."""
        self._feed(60.0, 60.0, 10)
        self.det.update(rad(200.0) * DT, rad(60.0) * DT, DT)   # one bad tick
        self.assertFalse(self.det.slipping)
        self._feed(60.0, 60.0, 10)
        self.assertFalse(self.det.slipping)

    def test_clears_after_exit_ticks_of_agreement(self):
        cfg = CONFIG.slip
        self.assertTrue(self._feed(150.0, 0.0, 10))
        for _ in range(cfg.exit_ticks - 1):
            self.det.update(rad(60.0) * DT, rad(60.0) * DT, DT)
            self.assertTrue(self.det.slipping, "cleared too early")
        self.det.update(rad(60.0) * DT, rad(60.0) * DT, DT)
        self.assertFalse(self.det.slipping)

    def test_sign_of_residual_says_which_way(self):
        self._feed(120.0, 10.0, 10)
        self.assertGreater(self.det.residual, 0.0)   # encoders over-claim
        self.det.reset()
        self._feed(10.0, 120.0, 10)
        self.assertLess(self.det.residual, 0.0)      # encoders under-claim

    # -- no gyro -----------------------------------------------------------
    def test_no_gyro_reports_not_slipping_but_inactive(self):
        """`active` is the whole point: a consumer must be able to tell "the two
        sources agree" from "there is only one source"."""
        self.det.update(rad(150.0) * DT, None, DT)
        self.assertFalse(self.det.slipping)
        self.assertFalse(self.det.telemetry()["active"])
        self.assertEqual(self.det.telemetry()["residual_dps"], 0.0)

    def test_losing_the_gyro_mid_slip_does_not_clear_the_latch(self):
        """A dropped sensor is not evidence the slip stopped."""
        self.assertTrue(self._feed(150.0, 0.0, 10))
        for _ in range(20):
            self.det.update(rad(150.0) * DT, None, DT)
        self.assertTrue(self.det.slipping, "a lost gyro silently cleared slip")
        self.assertFalse(self.det.telemetry()["active"])

    def test_gyro_returning_can_clear_it(self):
        self.assertTrue(self._feed(150.0, 0.0, 10))
        for _ in range(5):
            self.det.update(rad(150.0) * DT, None, DT)
        self.assertFalse(self._feed(60.0, 60.0, CONFIG.slip.exit_ticks + 1))

    # -- documented blind spot --------------------------------------------
    def test_known_blind_spot_equal_slip_in_a_straight_line(self):
        """BOTH sides slipping equally: the differential is zero, the gyro is
        zero, they agree perfectly -- and the kart is going nowhere while
        odometry integrates travel.

        Asserted so the limitation stays explicit. `slipping == False` here does
        NOT mean odometry is trustworthy; it means the two ROTATION sources
        agree. Translation slip is unobservable from proprioception and needs an
        exteroceptive reference (the orchestrator's range correction)."""
        self.assertFalse(self._feed(0.0, 0.0, 50))
        self.assertFalse(self.det.slipping)

    # -- telemetry ---------------------------------------------------------
    def test_telemetry_shape_and_units(self):
        self._feed(120.0, 30.0, 10)
        tel = self.det.telemetry()
        for key in ("slipping", "active", "residual_dps", "encoder_dps",
                    "gyro_dps", "ratio", "events"):
            self.assertIn(key, tel)
        self.assertAlmostEqual(tel["encoder_dps"], 120.0, delta=0.5)
        self.assertAlmostEqual(tel["gyro_dps"], 30.0, delta=0.5)
        self.assertAlmostEqual(tel["residual_dps"], 90.0, delta=1.0)

    def test_ratio_is_clamped_not_infinite(self):
        self._feed(150.0, 0.0, 5)
        self.assertLessEqual(abs(self.det.telemetry()["ratio"]),
                             CONFIG.slip.max_ratio)

    def test_blank_telemetry_matches_live_shape(self):
        self.assertEqual(set(SlipDetector.blank_telemetry()),
                         set(self.det.telemetry()))

    def test_zero_dt_is_survivable(self):
        self.det.update(0.1, 0.1, 0.0)          # must not raise
        self.assertFalse(self.det.slipping)


class _Motor:
    last = (0, 0, 0, 0)

    def setMotorModel(self, *d):
        self.last = d


class _Gyro:
    """Gyro whose yaw advances at a commanded rate, independent of the wheels --
    which is exactly the physical situation during a slip."""

    def __init__(self, dps=0.0):
        self.dps = dps
        self.yaw = 0.0

    def is_connected(self):
        return True

    def get_yaw(self):
        return self.yaw

    def advance(self, dt):
        self.yaw += self.dps * dt


class TestSlipInControlLoop(unittest.TestCase):
    """The detector as wired into step(): fed from real encoder deltas."""

    def _make(self, gyro=None):
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        wheels = WheelEncoders(sides, encoders=encs)
        ctrl = DriveController(_Motor(), wheels, CONFIG, gyro=gyro,
                               plant=SimulatedDrivePlant(wheels, CONFIG))
        return ctrl, wheels

    def test_telemetry_carries_the_slip_block(self):
        ctrl, _ = self._make()
        tel = ctrl.step(DT)
        self.assertIn("slip", tel)
        self.assertIn("slipping", tel["slip"])

    def test_straight_driving_does_not_flag_slip(self):
        gyro = _Gyro(0.0)
        ctrl, _ = self._make(gyro)
        ctrl.set_twist(0.2, 0.0)
        for _ in range(200):
            gyro.advance(DT)
            tel = ctrl.step(DT)
        self.assertFalse(tel["slip"]["slipping"])
        self.assertTrue(tel["slip"]["active"])

    def test_wheels_spinning_without_the_body_turning_flags_slip(self):
        """The plant turns the wheels (commanded spin) while the gyro insists the
        chassis is not rotating -- a scrubbing in-place turn."""
        gyro = _Gyro(0.0)
        ctrl, _ = self._make(gyro)
        ctrl.set_twist(0.0, 2.0)            # commanded spin
        tel = {}
        for _ in range(60):
            gyro.advance(DT)                # ...but the body never turns
            tel = ctrl.step(DT)
        self.assertTrue(tel["slip"]["slipping"])
        self.assertGreaterEqual(tel["slip"]["events"], 1)

    def test_no_gyro_leaves_the_detector_inactive(self):
        ctrl, _ = self._make(gyro=None)
        ctrl.set_twist(0.0, 2.0)
        for _ in range(40):
            tel = ctrl.step(DT)
        self.assertFalse(tel["slip"]["active"])
        self.assertFalse(tel["slip"]["slipping"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
