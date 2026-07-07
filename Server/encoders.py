#!/usr/bin/python3
"""
Quadrature encoders for the four drive motors, using **pigpio**.

pigpio services GPIO edges in a C daemon (pigpiod), so it does not miss edges
at high pulse rates the way RPi.GPIO's Python-level callbacks do -- which is
essential here: with 13 PPR * 45:1 gearing * 4 (quadrature) = 2340 counts per
wheel revolution, the edge rate at speed is far too high for Python callbacks
to keep up. A hardware glitch filter debounces each phase.

x4 decoding uses a transition table indexed by ``(prev_state << 2) | new`` with
``state = (A << 1) | B``:

        new ->  00   01   10   11
    prev 00 :    0,  -1,   1,   0
    prev 01 :    1,   0,   0,  -1
    prev 10 :   -1,   0,   0,   1
    prev 11 :    0,   1,  -1,   0

Off-Pi (no pigpio / no pigpiod), ``WheelEncoders`` transparently falls back to4{]/~´ÇKIJUH7Y6}
``SimulatedEncoder`` so the control stack still imports and the tests run.

Requires the daemon:  sudo pigpiod
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, List, Tuple

from config import ENCODER_PINS, SideMapping

try:  # pragma: no cover - hardware path
    import pigpio
    _PIGPIO_AVAILABLE = True
except Exception:
    pigpio = None
    _PIGPIO_AVAILABLE = False

# Ignore edges shorter than this (microseconds) as switch bounce / noise.
GLITCH_FILTER_US = 100

# {prev_state}{new_state} transition -> delta. Flattened 4x4 table.
QUAD_TABLE = [
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
]

# Backwards-compatible export.
HARDWARE_ENCODERS_CONNECTION = ENCODER_PINS


class Encoder:
    """pigpio-driven quadrature decoder for a single motor."""

    def __init__(self, pi, pin_phase_a: int, pin_phase_b: int, name: str = ""):
        self.pi = pi
        self.phase_a = pin_phase_a
        self.phase_b = pin_phase_b
        self.name = name

        self._count = 0        # delta accumulator, reset every control step
        self._total = 0        # lifetime counter, never auto-reset (diagnostics)
        self._state = 0
        self._lock = threading.Lock()
        self._cb_a = None
        self._cb_b = None
        self._listening = False

    # -- lifecycle ---------------------------------------------------------
    def begin(self) -> None:
        """Configure the phases and register edge callbacks. Idempotent."""
        if self._listening:
            return
        pi = self.pi
        for pin in (self.phase_a, self.phase_b):
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)
            pi.set_glitch_filter(pin, GLITCH_FILTER_US)
        self._state = (pi.read(self.phase_a) << 1) | pi.read(self.phase_b)
        self._cb_a = pi.callback(self.phase_a, pigpio.EITHER_EDGE, self._pulse)
        self._cb_b = pi.callback(self.phase_b, pigpio.EITHER_EDGE, self._pulse)
        self._listening = True

    def stop(self) -> None:
        for cb in (self._cb_a, self._cb_b):
            if cb is not None:
                try:
                    cb.cancel()
                except Exception:
                    pass
        self._cb_a = self._cb_b = None
        self._listening = False

    # -- decoding ----------------------------------------------------------
    def _pulse(self, gpio, level, tick) -> None:
        a = self.pi.read(self.phase_a)
        b = self.pi.read(self.phase_b)
        self._apply_state((a << 1) | b)

    def _apply_state(self, new_state: int) -> None:
        """Advance the counter given the freshly-read 2-bit phase state.

        Exposed so tests can drive the decoder without real GPIO.
        """
        with self._lock:
            delta = QUAD_TABLE[(self._state << 2) | new_state]
            self._count += delta
            self._total += delta
            self._state = new_state

    # -- reading -----------------------------------------------------------
    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    @property
    def total(self) -> int:
        """Lifetime signed count (for calibration; not reset by the loop)."""
        with self._lock:
            return self._total

    def read_reset(self) -> int:
        """Return the accumulated count and reset it to zero atomically."""
        with self._lock:
            value = self._count
            self._count = 0
            return value

    def reset(self) -> None:
        with self._lock:
            self._count = 0

    def reset_total(self) -> None:
        with self._lock:
            self._total = 0


class SimulatedEncoder(Encoder):
    """Encoder stand-in for tests / off-hardware runs (no pigpio).

    ``begin``/``stop`` are no-ops; feed motion with :meth:`add` (signed ticks)
    or :meth:`feed_states` (a sequence of raw 2-bit A/B states).
    """

    def __init__(self, pin_phase_a: int = 0, pin_phase_b: int = 0, name: str = ""):
        super().__init__(None, pin_phase_a, pin_phase_b, name)

    def begin(self) -> None:  # no hardware
        self._listening = True

    def stop(self) -> None:
        self._listening = False

    def add(self, ticks: int) -> None:
        with self._lock:
            self._count += ticks
            self._total += ticks

    def feed_states(self, states: Iterable[int]) -> None:
        for s in states:
            self._apply_state(s)


class WheelEncoders:
    """Manage the four encoders and aggregate them into left/right sides.

    Owns a single pigpio connection shared by all four encoders. If pigpio (or
    the daemon) is unavailable, it falls back to simulated encoders so nothing
    downstream breaks.
    """

    def __init__(self, sides: SideMapping,
                 pins: Dict[str, Tuple[int, int]] = ENCODER_PINS,
                 encoders: Dict[str, Encoder] | None = None,
                 pi=None):
        self.sides = sides
        self._pi = pi
        self._owns_pi = False
        self.using_hardware = False
        if encoders is not None:
            self.encoders = encoders
        else:
            self.encoders = self._build(pins)

    def _build(self, pins) -> Dict[str, Encoder]:
        if _PIGPIO_AVAILABLE:
            pi = self._pi
            if pi is None:
                pi = pigpio.pi()          # connects to pigpiod
                self._owns_pi = True
            self._pi = pi
            if pi is not None and pi.connected:
                self.using_hardware = True
                return {tag: Encoder(pi, a, b, name=tag)
                        for tag, (a, b) in pins.items()}
            print("[encoders] pigpiod not reachable; using simulated encoders. "
                  "Start it with:  sudo pigpiod")
        else:
            print("[encoders] pigpio not installed; using simulated encoders.")
        return {tag: SimulatedEncoder(a, b, name=tag)
                for tag, (a, b) in pins.items()}

    # -- lifecycle ---------------------------------------------------------
    def begin(self) -> None:
        for enc in self.encoders.values():
            enc.begin()

    def stop(self) -> None:
        for enc in self.encoders.values():
            enc.stop()
        if self._owns_pi and self._pi is not None:
            try:
                self._pi.stop()
            except Exception:
                pass
            self._pi = None

    def reset(self) -> None:
        for enc in self.encoders.values():
            enc.reset()

    def reset_totals(self) -> None:
        for enc in self.encoders.values():
            enc.reset_total()

    # -- diagnostics -------------------------------------------------------
    def raw_totals(self) -> Dict[str, int]:
        """Lifetime RAW counts per motor tag (sign NOT applied) — the ground
        truth for calibrating signs and counts_per_rev."""
        return {tag: enc.total for tag, enc in self.encoders.items()}

    def signed_totals(self) -> Dict[str, int]:
        """Lifetime counts per motor tag with the configured sign applied."""
        return {tag: enc.total * self.sides.signs.get(tag, 1)
                for tag, enc in self.encoders.items()}

    # -- aggregation -------------------------------------------------------
    def _signed_count(self, tag: str) -> int:
        return self.encoders[tag].count * self.sides.signs.get(tag, 1)

    def side_counts(self) -> Tuple[float, float]:
        """Current sign-corrected mean count for (left, right)."""
        left = _mean(self._signed_count(t) for t in self.sides.left)
        right = _mean(self._signed_count(t) for t in self.sides.right)
        return left, right

    def read_reset_sides(self) -> Tuple[float, float]:
        """Return (left, right) mean count deltas since the last call and
        reset every encoder."""
        left = _mean(self.encoders[t].read_reset() * self.sides.signs.get(t, 1)
                     for t in self.sides.left)
        right = _mean(self.encoders[t].read_reset() * self.sides.signs.get(t, 1)
                      for t in self.sides.right)
        return left, right


def _mean(values: Iterable[float]) -> float:
    vals: List[float] = list(values)
    if not vals:
        return 0.0
    return sum(vals) / len(vals)


if __name__ == "__main__":
    # Quick smoke test of the decoder maths (runs anywhere, no pigpio needed).
    enc = SimulatedEncoder(0, 0, name="demo")
    # One full quadrature cycle, positive direction: 00 -> 10 -> 11 -> 01 -> 00
    enc.feed_states([0b00, 0b10, 0b11, 0b01, 0b00])
    print("positive cycle count (expect +4):", enc.count)
    enc.reset()
    enc.feed_states([0b00, 0b01, 0b11, 0b10, 0b00])
    print("negative cycle count (expect -4):", enc.count)
