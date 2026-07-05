#!/usr/bin/python3
"""
Thin hardware-abstraction shim around RPi.GPIO.

On a Raspberry Pi this exposes the real ``RPi.GPIO`` module. On any other
machine (Windows/CI/dev laptop) it falls back to a no-op mock so that the
pure-logic modules (encoders, odometry, PID) can be imported, unit-tested and
reasoned about without the hardware present.

Usage:
    from gpio_backend import GPIO, ON_HARDWARE
"""
from __future__ import annotations


class _MockGPIO:
    """Minimal RPi.GPIO stand-in. Records setup but does nothing physical."""
    BCM = "BCM"
    BOARD = "BOARD"
    IN = "IN"
    OUT = "OUT"
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    PUD_OFF = "PUD_OFF"
    RISING = "RISING"
    FALLING = "FALLING"
    BOTH = "BOTH"
    HIGH = 1
    LOW = 0

    def __init__(self) -> None:
        self._levels: dict[int, int] = {}
        self._callbacks: dict[int, callable] = {}

    def setmode(self, mode) -> None:  # noqa: D401 - mirrors RPi.GPIO API
        pass

    def setwarnings(self, flag) -> None:
        pass

    def setup(self, pin, direction, pull_up_down=None) -> None:
        self._levels.setdefault(pin, self.LOW)

    def input(self, pin) -> int:
        return self._levels.get(pin, self.LOW)

    def output(self, pin, value) -> None:
        self._levels[pin] = value

    def add_event_detect(self, pin, edge, callback=None, bouncetime=0) -> None:
        if callback is not None:
            self._callbacks[pin] = callback

    def remove_event_detect(self, pin) -> None:
        self._callbacks.pop(pin, None)

    def cleanup(self, pin=None) -> None:
        self._callbacks.clear()

    def PWM(self, pin, freq):  # noqa: N802 - mirrors RPi.GPIO API
        return _MockPWM()

    # --- test helpers (not part of RPi.GPIO) ---
    def _set_level(self, pin, value) -> None:
        """Simulate an external level change and fire any edge callback."""
        self._levels[pin] = value
        cb = self._callbacks.get(pin)
        if cb is not None:
            cb(pin)


class _MockPWM:
    def start(self, duty):
        pass

    def ChangeDutyCycle(self, duty):  # noqa: N802 - mirrors RPi.GPIO API
        pass

    def stop(self):
        pass


try:  # pragma: no cover - hardware path
    import RPi.GPIO as _GPIO  # type: ignore
    GPIO = _GPIO
    ON_HARDWARE = True
except Exception:  # ImportError on non-Pi, RuntimeError if not root, etc.
    GPIO = _MockGPIO()
    ON_HARDWARE = False
