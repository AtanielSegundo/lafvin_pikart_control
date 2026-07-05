#!/usr/bin/python3
"""
A small, reusable PID controller.

Features:
  * time-aware (accepts an explicit ``dt`` so it is deterministic in tests),
  * integral anti-windup via clamping,
  * output saturation,
  * optional static feed-forward term,
  * ``reset()`` for clean re-engagement after the loop was idle.

Kept dependency-free so it can be unit-tested anywhere.
"""
from __future__ import annotations

from dataclasses import dataclass


def _clamp(value: float, limit: float) -> float:
    if limit is None:
        return value
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


@dataclass
class PID:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    feedforward: float = 0.0          # duty per unit of setpoint
    output_limit: float | None = None
    integral_limit: float | None = None

    _integral: float = 0.0
    _prev_error: float = 0.0
    _initialized: bool = False

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        """Return the control output for one step.

        ``setpoint`` and ``measured`` are in the same physical units (e.g.
        m/s). The output is in actuator units (e.g. PWM duty).

        Anti-windup uses two complementary guards:

        1. **Conditional integration (clamping).** When the output is saturated
           and the error would push it *further* into saturation, integration
           is frozen for that step. This is what stops the integrator from
           winding up while the actuator is pinned at ±output_limit.
        2. **Integral-term clamp.** ``integral_limit`` bounds the integral
           *contribution* ``ki * integral`` (in output units), reflected back
           into the stored state so it can't drift. Acts as a backstop.
        """
        if dt <= 0.0:
            dt = 1e-3

        error = setpoint - measured

        # Derivative on error, guarding against the first-sample spike.
        if self._initialized:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
            self._initialized = True

        feed = self.feedforward * setpoint
        p_term = self.kp * error
        d_term = self.kd * derivative

        # Tentatively integrate this step.
        integral = self._integral + error * dt
        i_term = self.ki * integral

        # (2) Clamp the integral *contribution*, not the raw accumulator, and
        #     reflect the clamp back so the stored integral cannot run away.
        if self.integral_limit is not None:
            clamped_i = _clamp(i_term, self.integral_limit)
            if clamped_i != i_term:
                i_term = clamped_i
                integral = clamped_i / self.ki if self.ki != 0 else self._integral

        raw_output = feed + p_term + i_term + d_term
        output = _clamp(raw_output, self.output_limit)

        # (1) Only commit the new integral if we're not saturated, or if the
        #     error is driving the output back toward the linear range
        #     (opposite signs). Otherwise freeze the integrator.
        saturated = output != raw_output
        driving_back = (error > 0) != (raw_output > 0)
        if not saturated or driving_back:
            self._integral = integral

        self._prev_error = error
        return output
