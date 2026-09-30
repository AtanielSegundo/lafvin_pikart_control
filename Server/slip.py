#!/usr/bin/python3
"""
Wheel-slip detection from the gyro/encoder heading residual.

The kart measures its own rotation twice per tick, by independent means:

    d_theta_gyro     MPU6050 integration -- the chassis turned, full stop.
                     Immune to slip: the body rotates whether the wheels grip
                     or not.
    d_theta_encoder  (d_right - d_left) / track -- what the wheels CLAIM the
                     body did.

``DriveController.step`` already computes both and then discards the encoder one
whenever the gyro is alive. The difference between them is free information, and
it is a slip detector: if the wheels report a rotation the chassis did not
perform, they were scrubbing rather than rolling.

Why this instead of the accelerometer test in the literature
-----------------------------------------------------------
De Giorgi, De Palma & Parlangeli (*Online Odometry Calibration for Differential
Drive Mobile Robots in Low Traction Conditions with Slippage*, Robotics 2024,
13, 7) detect slip by checking the IMU's lateral and frontal accelerations
against the differential model (their Eq. 7-8), then reconstruct the motion by
double-integrating acceleration through the slip (Eq. 12-13).

That second half is not reachable on an MPU6050: double-integrating a $3
consumer MEMS accelerometer accumulates position error as t^2, and the authors
themselves scope their reconstruction to "an amount of time of the order of a
minute" with a datasheet-grade IMU. Yaw rate, on the other hand, is the one
quantity this chip measures well -- so the residual below extracts the paper's
*detection* idea using the sensor axis that is actually trustworthy here, and
skips the reconstruction entirely.

What it deliberately cannot see
-------------------------------
Both sides slipping EQUALLY in a straight line. The encoder differential is zero
and so is the gyro, so the residual is zero while the kart makes no progress.
That failure mode is unobservable from proprioception alone -- it needs an
exteroceptive reference, which is what the orchestrator's range correction
against known maze walls provides. Callers must not read "not slipping" as
"odometry is trustworthy"; it means "the two rotation sources agree".
"""
from __future__ import annotations

from config import CONFIG, RobotConfig


class SlipDetector:
    """Latching slip flag from the gyro-vs-encoder rotation residual.

    Pure and hardware-free: fed two rotation deltas and a dt, it keeps no
    reference to any device, so it unit-tests directly and adds nothing to the
    control loop's dependency surface.

    Latching (``enter_ticks`` over threshold to set, ``exit_ticks`` clean to
    clear) rather than instantaneous, because the two deltas are sampled
    microseconds apart over slightly different windows: a single-tick
    disagreement is usually that misalignment, while real slip lasts many ticks
    at 20 Hz. This mirrors the paper's own Remark 1 -- test the condition over a
    few steps, not one.
    """

    def __init__(self, config: RobotConfig = CONFIG):
        self.config = config
        self.slipping = False
        self.residual = 0.0          # rad/s, signed (encoder - gyro)
        self.ratio = 1.0             # encoder rate / gyro rate
        self.gyro_rate = 0.0         # rad/s
        self.encoder_rate = 0.0      # rad/s
        self.events = 0              # times slip has latched since boot
        self._over = 0               # consecutive ticks above threshold
        self._under = 0              # consecutive ticks below it
        self._active = False         # detector had usable data this tick

    def reset(self) -> None:
        self.slipping = False
        self.residual = 0.0
        self.ratio = 1.0
        self._over = 0
        self._under = 0
        self._active = False

    def update(self, d_theta_encoder: float, d_theta_gyro, dt: float) -> bool:
        """One tick. Both deltas in radians, ``dt`` in seconds.

        ``d_theta_gyro`` of None means no gyro (disconnected, or stale across
        the process boundary). There is then nothing to compare against, so the
        detector goes quiet and reports not-slipping -- it must not guess, since
        every consumer treats the flag as positive evidence.
        """
        if dt <= 0.0:
            return self.slipping

        if d_theta_gyro is None:
            self._active = False
            self.encoder_rate = d_theta_encoder / dt
            self.gyro_rate = 0.0
            self.residual = 0.0
            self.ratio = 1.0
            # Deliberately NOT clearing the latch here: losing the gyro
            # mid-slip should not read as "the slip ended". It stays as it was
            # until real data either confirms or clears it.
            return self.slipping

        cfg = self.config.slip
        self._active = True
        self.encoder_rate = d_theta_encoder / dt
        self.gyro_rate = d_theta_gyro / dt
        self.residual = self.encoder_rate - self.gyro_rate

        reference = max(abs(self.encoder_rate), abs(self.gyro_rate))
        self.ratio = _safe_ratio(self.encoder_rate, self.gyro_rate,
                                 cfg.max_ratio)

        if reference < cfg.min_rate:
            # Both sources read ~zero: nothing is claimed either way, so this
            # tick is evidence of neither slip nor grip. Counted as clean so a
            # kart that stops moving eventually drops the latch.
            self._tick_clean(cfg)
            return self.slipping

        # Threshold grows with the rate: a hard turn legitimately disagrees by
        # more in absolute terms than a slow one (quantisation of whole encoder
        # counts, 20 Hz sampling of a ramping rate), so a fixed tolerance alone
        # would flag every fast turn as slip.
        threshold = cfg.rate_tolerance + cfg.rate_fraction * reference
        if abs(self.residual) > threshold:
            self._under = 0
            self._over += 1
            if not self.slipping and self._over >= cfg.enter_ticks:
                self.slipping = True
                self.events += 1
        else:
            self._tick_clean(cfg)
        return self.slipping

    def _tick_clean(self, cfg) -> None:
        self._over = 0
        self._under += 1
        if self.slipping and self._under >= cfg.exit_ticks:
            self.slipping = False

    def telemetry(self) -> dict:
        """Slip sub-dict for the drive telemetry snapshot.

        ``active`` says whether the detector had a gyro to compare against, so a
        consumer can tell "not slipping" from "cannot tell" -- collapsing those
        two is how a downstream user ends up trusting odometry during a slip.
        """
        return {
            "slipping": self.slipping,
            "active": self._active,
            "residual_dps": round(_degrees(self.residual), 2),
            "encoder_dps": round(_degrees(self.encoder_rate), 2),
            "gyro_dps": round(_degrees(self.gyro_rate), 2),
            "ratio": round(self.ratio, 3),
            "events": self.events,
        }

    @staticmethod
    def blank_telemetry() -> dict:
        return {"slipping": False, "active": False, "residual_dps": 0.0,
                "encoder_dps": 0.0, "gyro_dps": 0.0, "ratio": 1.0,
                "events": 0}


def _degrees(rad: float) -> float:
    return rad * 57.29577951308232


def _safe_ratio(numerator: float, denominator: float, limit: float) -> float:
    """encoder/gyro, clamped. Reported for diagnostics only -- the decision uses
    the residual, because a ratio is meaningless when both terms are near zero
    and explodes just before it becomes so."""
    if denominator == 0.0:
        return limit if numerator else 1.0
    return max(-limit, min(limit, numerator / denominator))
