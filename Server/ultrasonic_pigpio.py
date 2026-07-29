#!/usr/bin/python3
"""
HC-SR04 ultrasonic read via pigpio -- the echo pulse is timed by the pigpiod
daemon (C, microsecond hardware ticks), NOT by a Python busy-wait.

Why: Ultrasonic.get_distance() times the echo with RPi.GPIO `pulseIn`, a pure
Python loop. Under the GIL (control loop + telemetry + camera threads) that loop
gets preempted mid-pulse, so it misses the echo edge -> the 255 "failed read"
sentinel, plus latency from blocking timeouts. pigpio timestamps every echo edge
in C, immune to Python scheduling, so:
  * readings are accurate (no missed-edge dropouts / 255 spikes), and
  * get_distance() is non-blocking -- it returns the latest value the echo
    callback computed, so the caller's guard loop never stalls.

Matches the encoders' pigpio usage (Server/encoders.py): guards the import and
raises cleanly when pigpio / pigpiod isn't available so the caller can fall back
to the RPi.GPIO sensor.

Because pigpio has no missed-edge failures, "no echo within range" genuinely
means "nothing close", so we report the max range as a normal far/clear reading
(int) rather than a failure sentinel -- unambiguous for the collision guard.
"""
import threading
import time

try:
    import pigpio
    _PIGPIO_AVAILABLE = True
except ImportError:
    _PIGPIO_AVAILABLE = False


# Speed of sound ~343 m/s at 20 C = 0.0343 cm/us; distance = width_us * v / 2.
_CM_PER_US = 0.0343 / 2.0


class UltrasonicPigpio:
    def __init__(self, trigger=27, echo=22, pi=None,
                 max_distance_cm=300, ping_interval=0.06, stale_after=0.2):
        if not _PIGPIO_AVAILABLE:
            raise RuntimeError("pigpio not installed")
        self.trigger = trigger
        self.echo = echo
        self.max_distance_cm = max_distance_cm
        self.ping_interval = ping_interval          # s between trigger pulses
        self.stale_after = stale_after              # s; no echo => report far

        self._pi = pi
        self._owns_pi = pi is None
        if self._pi is None:
            self._pi = pigpio.pi()                  # connects to pigpiod
        if not self._pi.connected:
            raise RuntimeError("pigpiod not reachable (start it: sudo pigpiod)")

        self._pi.set_mode(trigger, pigpio.OUTPUT)
        self._pi.set_mode(echo, pigpio.INPUT)
        self._pi.write(trigger, 0)

        self._rise_tick = None
        self._distance_cm = None
        self._last_valid = 0.0
        self._lock = threading.Lock()
        self._cb = self._pi.callback(echo, pigpio.EITHER_EDGE, self._on_edge)

        # Light trigger thread: a 10 us pulse every ping_interval. The echo is
        # captured asynchronously by _on_edge, so this only sleeps (releases the
        # GIL) -- no busy-wait, no contention with the control loop.
        self._running = True
        self._t = threading.Thread(target=self._ping_loop, daemon=True,
                                   name="UltrasonicPing")
        self._t.start()
        time.sleep(0.05)                            # let a first echo land

    # -- pigpio callback (runs in pigpio's thread; arithmetic only) ---------
    def _on_edge(self, gpio, level, tick):
        if level == 1:                              # rising: echo pulse started
            self._rise_tick = tick
        elif level == 0 and self._rise_tick is not None:   # falling: pulse end
            width_us = pigpio.tickDiff(self._rise_tick, tick)  # 32-bit wrap-safe
            self._rise_tick = None
            d = width_us * _CM_PER_US
            if 0.0 < d <= self.max_distance_cm:
                with self._lock:
                    self._distance_cm = d
                    self._last_valid = time.monotonic()

    def _ping_loop(self):
        while self._running:
            try:
                self._pi.gpio_trigger(self.trigger, 10, 1)   # 10 us HIGH pulse
            except Exception:                       # noqa: BLE001
                pass
            time.sleep(self.ping_interval)

    # -- API: drop-in for Ultrasonic.get_distance() ------------------------
    def get_distance(self):
        """Latest distance in cm (int). Non-blocking. If no valid echo has
        arrived recently (object out of range / never measured), report the max
        range -- a normal 'far / clear' value, not a failure sentinel."""
        with self._lock:
            d = self._distance_cm
            ts = self._last_valid
        if d is None or (time.monotonic() - ts) > self.stale_after:
            return int(self.max_distance_cm)
        return int(round(d))

    def stop(self):
        self._running = False
        try:
            self._t.join(timeout=1.0)
        except Exception:                           # noqa: BLE001
            pass
        try:
            self._cb.cancel()
        except Exception:                           # noqa: BLE001
            pass
        if self._owns_pi:
            try:
                self._pi.stop()
            except Exception:                       # noqa: BLE001
                pass
