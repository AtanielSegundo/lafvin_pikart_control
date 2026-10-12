#!/usr/bin/python3
"""
Unit tests for the per-motor PWM correction (motor_correction.py) and its use
in the gyro heading turn.

Run from the Server directory:
    python -m unittest discover -s tests
"""
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CONFIG  # noqa: E402
from drive_controller import DriveController  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders  # noqa: E402
from motor_correction import (coefficients_from_curves, correct_duties,  # noqa: E402
                              correct_pwm, load_calibration, validate)

# Wheel speed lines w = k*u + c (rev/s), as the calibration script fits them.
CURVES = {
    "m1": {"fwd": {"k": 1.00e-3, "c": -0.50}, "rev": {"k": 0.95e-3, "c": -0.45}},
    "m2": {"fwd": {"k": 1.05e-3, "c": -0.60}, "rev": {"k": 1.00e-3, "c": -0.55}},
    "m3": {"fwd": {"k": 0.90e-3, "c": -0.40}, "rev": {"k": 0.92e-3, "c": -0.50}},
    "m4": {"fwd": {"k": 1.10e-3, "c": -0.70}, "rev": {"k": 1.02e-3, "c": -0.52}},
}
PARAMS = {"correction": coefficients_from_curves(CURVES)}


def _speed(slot, duty):
    f = CURVES[slot]["fwd" if duty > 0 else "rev"]
    return f["k"] * abs(duty) + f["c"]


class TestCorrectPwm(unittest.TestCase):
    def test_no_params_is_pass_through(self):
        self.assertEqual(correct_pwm(2000, "m3", None), 2000)
        self.assertEqual(correct_pwm(-1500.4, 2, {}), -1500)

    def test_zero_stays_brake(self):
        for slot in ("m1", "m2", "m3", "m4"):
            self.assertEqual(correct_pwm(0, slot, PARAMS), 0)

    def test_reference_forward_is_identity(self):
        self.assertEqual(correct_pwm(2200, "m1", PARAMS), 2200)

    def test_every_motor_matches_reference_speed(self):
        # The defining property: the corrected duty spins each motor, in either
        # direction, at m1-forward's speed for the same reference duty.
        for ref in (1600, 2200, 2800, -1600, -2800):
            target = _speed("m1", abs(ref))
            for slot in ("m1", "m2", "m3", "m4"):
                duty = correct_pwm(ref, slot, PARAMS)
                self.assertEqual(math.copysign(1, duty), math.copysign(1, ref))
                self.assertAlmostEqual(_speed(slot, duty), target, delta=1e-3)

    def test_slot_names(self):
        self.assertEqual(correct_pwm(2000, 3, PARAMS),
                         correct_pwm(2000, "M3", PARAMS))

    def test_clamped_to_duty_range(self):
        self.assertEqual(correct_pwm(4095, "m3", PARAMS), 4095)
        self.assertEqual(correct_pwm(-4095, "m3", PARAMS), -4095)

    def test_correct_duties_maps_setmotormodel_order(self):
        out = correct_duties(-2000, -2000, 2000, 2000, PARAMS)
        self.assertEqual(out, tuple(correct_pwm(d, s, PARAMS) for d, s in
                                    zip((-2000, -2000, 2000, 2000),
                                        ("m1", "m2", "m3", "m4"))))


class TestLoadCalibration(unittest.TestCase):
    def _write(self, params):
        fd, path = tempfile.mkstemp(suffix=".json")
        with os.fdopen(fd, "w") as f:
            json.dump(params, f)
        self.addCleanup(os.remove, path)
        return path

    def test_round_trip(self):
        self.assertEqual(load_calibration(self._write(PARAMS)), PARAMS)

    def test_missing_file_means_uncorrected(self):
        self.assertIsNone(load_calibration(os.path.join(tempfile.gettempdir(),
                                                        "no_such_cal.json")))

    def test_implausible_gain_ratio_is_refused(self):
        bad = {"correction": {"m2": {"fwd": {"a": 3.0, "b": 0.0}}}}
        self.assertIsNotNone(validate(bad))
        self.assertIsNone(load_calibration(self._write(bad)))


class _RecordingMotor:
    def __init__(self):
        self.last = (0, 0, 0, 0)

    def setMotorModel(self, d1, d2, d3, d4):
        self.last = (d1, d2, d3, d4)


class _FixedGyro:
    def is_connected(self):
        return True

    def get_yaw(self):
        return 0.0


class TestHeadingTurnUsesCorrection(unittest.TestCase):
    def _first_turn_duties(self, calibration):
        motor = _RecordingMotor()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ("M1", "M2", "M3", "M4")}
        ctrl = DriveController(motor, WheelEncoders(CONFIG.sides, encoders=encs),
                               CONFIG, gyro=_FixedGyro(),
                               motor_calibration=calibration)
        ctrl.turn_in_place(90.0)
        ctrl.step(1.0 / CONFIG.control.loop_hz)
        return motor.last, ctrl.telemetry()["duty"]

    def test_uncorrected_turn_sends_one_magnitude(self):
        (d1, d2, d3, d4), _ = self._first_turn_duties(None)
        self.assertGreater(d3, 0)                     # ccw: right forward
        self.assertEqual((d1, d2, d3, d4), (-d3, -d3, d3, d3))

    def test_corrected_turn_sends_per_motor_duties(self):
        (raw, _), (cor, tel) = (self._first_turn_duties(None),
                                self._first_turn_duties(PARAMS))
        # The controller corrects the unrounded duty; +-1 from rounding first.
        for got, want in zip(cor, correct_duties(*raw, PARAMS)):
            self.assertAlmostEqual(got, want, delta=1)
        self.assertNotEqual(cor[2], cor[3])           # m3 and m4 differ
        self.assertEqual(tel["right"], raw[2])        # telemetry: m1-fwd units


if __name__ == "__main__":
    unittest.main()
