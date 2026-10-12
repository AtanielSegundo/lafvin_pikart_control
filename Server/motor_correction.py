#!/usr/bin/python3
"""
Per-motor PWM correction: make every motor spin as fast as the reference.

Four "identical" gearmotors are not identical. Given the same duty they settle
at different wheel speeds (winding/magnet spread, gearbox friction, brush
timing -- which is also why a motor can be faster one way than the other). In
an in-place turn the controller sends one duty magnitude to all four wheels,
so a slow wheel drags its side and the kart pivots about a point that is not
its centre: the rotation walks.

Model, per motor slot and per direction, measured with the kart SUSPENDED
(Scripts/motor_calibration.py):

    w(u) = k * u + c          w = wheel speed (rev/s), u = |duty|

The reference is m1 spinning FORWARD. For any other (motor, direction) the duty
that matches the reference speed is the inverse of its own line evaluated at
the reference speed, which is again linear in the reference duty:

    u_target = a * u_ref + b,      a = k_ref / k_t,   b = (c_ref - c_t) / k_t

One reference for BOTH directions on purpose: a ccw turn runs the left side in
reverse and the right side forward, and a centred turn needs the two sides at
the same |speed| -- so m1-reverse is corrected onto m1-forward like the rest.

Motors are named by their slot in ``Motor.setMotorModel(duty1..duty4)`` /
``CMD_MOTOR#m1#m2#m3#m4`` ("m1".."m4"), NOT by encoder tag: the encoder that
each slot actually drives (M1..M4 in config.py) is measured by the calibration
and stored under "slots" for reference only.

Pure stdlib, no hardware: imported by P_control (DriveController) and by the
laptop-side calibration script alike.
"""
from __future__ import annotations

import json
import math
import os
from typing import Optional, Tuple, Union

CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "motor_calibration.json")
SLOTS = ("m1", "m2", "m3", "m4")
DIRECTIONS = ("fwd", "rev")
MAX_DUTY = 4095

# A gain ratio outside this band is not motor spread, it is a broken
# measurement (dead/half encoder, wrong slot mapping). Refuse it rather than
# drive one wheel at a third of the others' speed.
A_MIN, A_MAX = 0.5, 2.0


def _slot_key(target_motor: Union[int, str]) -> str:
    if isinstance(target_motor, int):
        return f"m{target_motor}"
    return str(target_motor).lower()


def direction_of(duty: float) -> str:
    return "fwd" if duty > 0 else "rev"


def correct_pwm(reference_motor_pwm: float, target_motor: Union[int, str],
                calibration_params: Optional[dict]) -> int:
    """Duty for ``target_motor`` that spins it at the speed the reference motor
    (m1, forward) reaches with ``reference_motor_pwm``.

    ``reference_motor_pwm`` is signed; its sign is the direction the target is
    driven in and is preserved. ``target_motor`` is a slot: 1..4 or "m1".."m4".
    0 stays 0 (brake) and missing params / a missing slot pass the duty through
    unchanged, so an uncalibrated kart behaves exactly as before.
    """
    ref = float(reference_motor_pwm)
    if ref == 0.0:
        return 0
    coef = None
    if calibration_params:
        coef = (calibration_params.get("correction", {})
                .get(_slot_key(target_motor), {})
                .get(direction_of(ref)))
    if coef is None:
        mag = abs(ref)
    else:
        mag = coef["a"] * abs(ref) + coef["b"]
    mag = min(max(mag, 0.0), float(MAX_DUTY))
    return int(round(math.copysign(mag, ref)))


def correct_duties(duty1: float, duty2: float, duty3: float, duty4: float,
                   calibration_params: Optional[dict]) -> Tuple[int, int, int, int]:
    """:func:`correct_pwm` over the four ``setMotorModel`` arguments."""
    return tuple(correct_pwm(d, slot, calibration_params)
                 for slot, d in zip(SLOTS, (duty1, duty2, duty3, duty4)))


def coefficients_from_curves(curves: dict, reference=("m1", "fwd")) -> dict:
    """{slot: {dir: {"a", "b"}}} from fitted ``{slot: {dir: {"k", "c"}}}``.

    A (slot, dir) without a usable curve is left out, which correct_pwm treats
    as pass-through.
    """
    ref = curves[reference[0]][reference[1]]
    k_r, c_r = float(ref["k"]), float(ref["c"])
    out = {}
    for slot, by_dir in curves.items():
        for d, fit in by_dir.items():
            if not fit or float(fit.get("k", 0.0)) <= 0.0:
                continue
            k_t, c_t = float(fit["k"]), float(fit["c"])
            out.setdefault(slot, {})[d] = {"a": k_r / k_t,
                                           "b": (c_r - c_t) / k_t}
    return out


def validate(params: dict) -> Optional[str]:
    """None if ``params`` is usable by correct_pwm, else the reason it is not."""
    corr = params.get("correction") if isinstance(params, dict) else None
    if not isinstance(corr, dict) or not corr:
        return "no 'correction' table"
    for slot, by_dir in corr.items():
        if slot not in SLOTS or not isinstance(by_dir, dict):
            return f"bad slot entry {slot!r}"
        for d, coef in by_dir.items():
            if d not in DIRECTIONS:
                return f"bad direction {slot}.{d}"
            try:
                a, b = float(coef["a"]), float(coef["b"])
            except (KeyError, TypeError, ValueError):
                return f"{slot}.{d}: missing/non-numeric a, b"
            if not (math.isfinite(a) and math.isfinite(b)):
                return f"{slot}.{d}: non-finite a, b"
            if not A_MIN <= a <= A_MAX:
                return (f"{slot}.{d}: a={a:.3f} outside [{A_MIN}, {A_MAX}] -- "
                        f"broken encoder or slot mapping, not motor spread")
    return None


def load_calibration(path: str = CALIBRATION_PATH) -> Optional[dict]:
    """Load the calibration JSON, or None (= no correction) if it is missing
    or fails :func:`validate`."""
    try:
        with open(path) as f:
            params = json.load(f)
    except FileNotFoundError:
        print(f"[motor_correction] no calibration at {path}; duties uncorrected")
        return None
    except (OSError, ValueError) as e:
        print(f"[motor_correction] unreadable calibration {path} ({e}); "
              f"duties uncorrected")
        return None
    problem = validate(params)
    if problem:
        print(f"[motor_correction] rejected {path}: {problem}; duties uncorrected")
        return None
    print(f"[motor_correction] loaded {path} "
          f"(created {params.get('created', '?')})")
    return params
