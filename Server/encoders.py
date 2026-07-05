#!/usr/bin/python3
"""
Quadrature encoders for the four drive motors.

Each motor exposes two Hall/optical phases (A, B) in quadrature. Reading both
edges of both phases (x4 decoding) yields a signed tick count whose sign
encodes the direction of rotation.

Decoding uses a transition lookup table indexed by ``(prev_state << 2) | new``
where ``state = (A << 1) | B``:

        new ->  00   01   10   11
    prev 00 :    0,  -1,   1,   0
    prev 01 :    1,   0,   0,  -1
    prev 10 :   -1,   0,   0,   1
    prev 11 :    0,   1,  -1,   0

On a Raspberry Pi the counting is interrupt-driven (GPIO edge callbacks). On
any other machine the GPIO shim is a no-op, and ``SimulatedEncoder`` can be
injected for deterministic tests.
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, List, Tuple

from config import ENCODER_PINS, SideMapping
from gpio_backend import GPIO, ON_HARDWARE

# {prev_state}{new_state} transition -> delta. Flattened 4x4 table.
QUAD_TABLE = [
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
]

# Backwards-compatible export (old name used the string tags below).
HARDWARE_ENCODERS_CONNECTION = ENCODER_PINS


class Encoder:
    """Interrupt-driven quadrature decoder for a single motor."""

    def __init__(self, pin_phase_a: int, pin_phase_b: int, name: str = ""):
        self.phase_a = pin_phase_a
        self.phase_b = pin_phase_b
        self.name = name

        self._count = 0
        self._state = 0
        self._lock = threading.Lock()
        self._listening = False

    # -- lifecycle ---------------------------------------------------------
    def begin(self) -> None:
        """Configure GPIO and start listening for edges. Idempotent."""
        if self._listening:
            return
        GPIO.setup(self.phase_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.phase_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self._state = self._read_state()
        GPIO.add_event_detect(self.phase_a, GPIO.BOTH, callback=self._on_edge)
        GPIO.add_event_detect(self.phase_b, GPIO.BOTH, callback=self._on_edge)
        self._listening = True

    def stop(self) -> None:
        if not self._listening:
            return
        try:
            GPIO.remove_event_detect(self.phase_a)
            GPIO.remove_event_detect(self.phase_b)
        except Exception:
            pass
        self._listening = False

    # -- decoding ----------------------------------------------------------
    def _read_state(self) -> int:
        return (GPIO.input(self.phase_a) << 1) | GPIO.input(self.phase_b)

    def _on_edge(self, _pin) -> None:
        self._apply_state(self._read_state())

    def _apply_state(self, new_state: int) -> None:
        """Advance the counter given the freshly-read 2-bit phase state.

        Exposed (non-underscore friends may call) so tests can drive the
        decoder without real GPIO.
        """
        with self._lock:
            delta = QUAD_TABLE[(self._state << 2) | new_state]
            self._count += delta
            self._state = new_state

    # -- reading -----------------------------------------------------------
    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def read_reset(self) -> int:
        """Return the accumulated count and reset it to zero atomically."""
        with self._lock:
            value = self._count
            self._count = 0
            return value

    def reset(self) -> None:
        with self._lock:
            self._count = 0


class SimulatedEncoder(Encoder):
    """Encoder stand-in for tests / off-hardware runs.

    ``begin``/``stop`` are no-ops; feed motion with :meth:`add` (signed ticks)
    or :meth:`feed_states` (a sequence of raw 2-bit A/B states).
    """

    def begin(self) -> None:  # no GPIO
        self._listening = True

    def stop(self) -> None:
        self._listening = False

    def add(self, ticks: int) -> None:
        with self._lock:
            self._count += ticks

    def feed_states(self, states: Iterable[int]) -> None:
        for s in states:
            self._apply_state(s)


class WheelEncoders:
    """Manage the four encoders and aggregate them into left/right sides.

    Per-side value = mean of that side's encoders (sign-corrected so that
    forward motion is positive). Distances are integrated by the odometry from
    the deltas returned here.
    """

    def __init__(self, sides: SideMapping,
                 pins: Dict[str, Tuple[int, int]] = ENCODER_PINS,
                 encoders: Dict[str, Encoder] | None = None):
        self.sides = sides
        if encoders is not None:
            self.encoders = encoders
        else:
            enc_cls = Encoder if ON_HARDWARE else SimulatedEncoder
            self.encoders = {
                tag: enc_cls(pa, pb, name=tag)
                for tag, (pa, pb) in pins.items()
            }

    def begin(self) -> None:
        for enc in self.encoders.values():
            enc.begin()

    def stop(self) -> None:
        for enc in self.encoders.values():
            enc.stop()

    def reset(self) -> None:
        for enc in self.encoders.values():
            enc.reset()

    def _signed_count(self, tag: str) -> int:
        return self.encoders[tag].count * self.sides.signs.get(tag, 1)

    def side_counts(self) -> Tuple[float, float]:
        """Current sign-corrected mean count for (left, right)."""
        left = _mean(self._signed_count(t) for t in self.sides.left)
        right = _mean(self._signed_count(t) for t in self.sides.right)
        return left, right

    def read_reset_sides(self) -> Tuple[float, float]:
        """Return (left, right) mean count deltas since the last call and
        reset every encoder atomically-ish."""
        left_tags = self.sides.left
        right_tags = self.sides.right
        left = _mean(self.encoders[t].read_reset() * self.sides.signs.get(t, 1)
                     for t in left_tags)
        right = _mean(self.encoders[t].read_reset() * self.sides.signs.get(t, 1)
                      for t in right_tags)
        return left, right


def _mean(values: Iterable[float]) -> float:
    vals: List[float] = list(values)
    if not vals:
        return 0.0
    return sum(vals) / len(vals)


if __name__ == "__main__":
    # Quick smoke test of the decoder maths (runs anywhere).
    enc = SimulatedEncoder(0, 0, name="demo")
    # One full quadrature cycle, positive direction: 00 -> 10 -> 11 -> 01 -> 00
    enc.feed_states([0b00, 0b10, 0b11, 0b01, 0b00])
    print("positive cycle count (expect +4):", enc.count)
    enc.reset()
    enc.feed_states([0b00, 0b01, 0b11, 0b10, 0b00])
    print("negative cycle count (expect -4):", enc.count)
