#!/usr/bin/python3
"""
Standstill detection, used to freeze the gyro's yaw integration.

Why this exists
---------------
A bias-corrected MPU6050 still carries a residual rate of a few hundredths of a
deg/s plus random-walk noise. Integrated continuously that becomes heading
drift, and unlike roll and pitch -- which `heading.py` pulls back towards the
gravity vector with a complementary filter -- yaw has NO absolute reference. The
only thing holding it is the calibrated bias, so whatever the bias gets wrong
accumulates without bound.

In this robot that lands squarely on the drive controller, which treats gyro yaw
as ground truth for rotation. A kart parked a couple of minutes between legs can
pick up several degrees of rotation it never performed, and the next
`turn_in_place` inherits all of it.

So: stop integrating while the kart is demonstrably still.

Why the encoders decide, and not the gyro
-----------------------------------------
Asking the gyro whether its own output should be trusted is circular -- the
integrated angle is exactly what is suspect. The encoders are unambiguous at
zero: wheels that are not turning emit no counts. That makes them the honest
witness for "stopped", even though they are the unreliable one for "how far"
(which is what the slip detector and the orchestrator's range fix are for).

The asymmetry is deliberate
---------------------------
Freezing during real motion would silently discard rotation, which is far worse
than the drift being prevented -- drift is slow and bounded by how long the kart
sits, while a lost slice of a turn is an immediate, permanent heading error. So:

  * entering standstill is SLOW and demands sustained quiet;
  * leaving it is IMMEDIATE on the first count;
  * and a gyro rate above `gyro_rate_floor_dps` overrides the encoders
    entirely, because an in-place turn scrubs the wheels sideways and may
    barely register on them while the chassis is genuinely rotating.

That last clause is the safety valve: if the encoders and the gyro disagree
about whether the kart is moving, this believes "moving".
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional

from config import CONFIG, ENCODER_TAGS, RobotConfig


class StandstillDetector:
    """Tracks whether the kart is at rest, from raw encoder totals.

    Hardware-free and pure: it is fed a dict of per-motor totals and a clock, so
    it unit-tests directly without pigpio, a gyro, or any timing games.

    Used as ``GyroMPU(fn_to_check_kart_movement=detector.is_moving)``: the gyro
    calls that predicate on every integration step and skips the integration
    when it returns False.
    """

    def __init__(self, config: RobotConfig = CONFIG,
                 clock: Callable[[], float] = time.monotonic):
        self.config = config
        self._clock = clock
        self._prev: Optional[Dict[str, int]] = None
        self._last_motion_ts = clock()
        self._moving = True          # assume motion until proven otherwise
        self._gyro_rate_dps = 0.0
        self.froze_count = 0         # transitions into standstill, for telemetry
        self.frozen_s = 0.0          # cumulative time integration was frozen
        self._frozen_since = None

    # -- inputs ------------------------------------------------------------
    def update(self, totals: Dict[str, int]) -> bool:
        """Feed fresh RAW per-motor totals. Returns the new moving state.

        Raw (sign-free) totals on purpose: this only asks "did anything turn",
        for which direction is irrelevant, and it keeps the detector independent
        of the sign calibration that lives in P_control.
        """
        cfg = self.config.standstill
        if not cfg.enabled:
            self._moving = True
            return True

        now = self._clock()
        prev, self._prev = self._prev, dict(totals)

        if prev is None:
            # First sample: no delta to judge, and adopting it as the baseline
            # is what keeps a long-running P_sensors from reading its whole
            # history as one enormous movement.
            self._last_motion_ts = now
            return self._set_moving(True, now)

        moved = sum(abs(totals.get(tag, 0) - prev.get(tag, 0))
                    for tag in ENCODER_TAGS)

        # The gyro overrides the encoders in the "moving" direction only. An
        # in-place turn scrubs the wheels sideways and can register far fewer
        # counts than the rotation deserves; freezing there would throw away a
        # real turn.
        if abs(self._gyro_rate_dps) > cfg.gyro_rate_floor_dps:
            self._last_motion_ts = now
            return self._set_moving(True, now)

        if moved > cfg.count_tolerance:
            self._last_motion_ts = now
            return self._set_moving(True, now)

        # Quiet this tick. Only freeze once it has been quiet long enough --
        # a slow creep emits few counts per tick, and freezing mid-creep is the
        # failure this guards against.
        quiet_for = now - self._last_motion_ts
        if quiet_for >= cfg.enter_quiet_s:
            return self._set_moving(False, now)
        return self._set_moving(True, now)

    def note_gyro_rate(self, dps: float) -> None:
        """Latest measured yaw rate, in deg/s. Optional, but it is what makes
        an in-place turn safe (see the class docstring)."""
        self._gyro_rate_dps = float(dps)

    # -- output ------------------------------------------------------------
    def is_moving(self) -> bool:
        """The predicate handed to GyroMPU. Cheap and lock-free: it is called
        at the gyro's sample rate from the gyro's own thread, and must not
        block that thread on anything."""
        return self._moving

    def _set_moving(self, moving: bool, now: float) -> bool:
        if moving != self._moving:
            if moving:
                if self._frozen_since is not None:
                    self.frozen_s += now - self._frozen_since
                    self._frozen_since = None
            else:
                self.froze_count += 1
                self._frozen_since = now
        self._moving = moving
        return moving

    def telemetry(self) -> dict:
        now = self._clock()
        frozen = self.frozen_s
        if self._frozen_since is not None:
            frozen += now - self._frozen_since
        return {
            "moving": self._moving,
            "frozen_events": self.froze_count,
            "frozen_s": round(frozen, 2),
            "quiet_s": round(max(0.0, now - self._last_motion_ts), 2),
        }

    @staticmethod
    def blank_telemetry() -> dict:
        return {"moving": True, "frozen_events": 0, "frozen_s": 0.0,
                "quiet_s": 0.0}
