#!/usr/bin/python3
"""
HC-SR04 ultrasonic read via pigpio -- the echo pulse is timed by the pigpiod
daemon (C, microsecond hardware ticks), NOT by a Python busy-wait.

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

_CM_PER_US = 0.0343 / 2.0

class UltrasonicPigpio:
    def __init__(self, trigger=27, echo=22, pi=None,
                 max_distance_cm = 300,
                 ping_interval   = 0.06,
                 stale_after     = 0.2,
                 # A ping try fails when the echo pulse does not complete
                 # (no rising edge, or a rise with no fall) before the next
                 # trigger. max consecutive failures before the sensor is
                 # latched faulted -- 4096 * ping_interval ~= 246 s.
                 max_ping_tries_before_halt = 4096
                 ):
        if not _PIGPIO_AVAILABLE:
            raise RuntimeError("pigpio not installed")
        
        self.trigger         = trigger
        self.echo            = echo
        self.max_distance_cm = max_distance_cm
        self.ping_interval   = ping_interval    # s between trigger pulses
        self.stale_after     = stale_after      # s; no echo => report far
        self.max_ping_tries  = max_ping_tries_before_halt
        
        self._pi = pi
        self._owns_pi = pi is None
        
        if self._pi is None:
            self._pi = pigpio.pi()                  # connects to pigpiod
        if not self._pi.connected:
            raise RuntimeError("pigpiod not reachable (start it: sudo pigpiod)")

        self._pi.set_mode(trigger, pigpio.OUTPUT)
        self._pi.set_mode(echo, pigpio.INPUT)
        self._pi.write(trigger, 0)

        self._rise_tick   = None
        self._distance_cm = None
        self._last_valid  = 0.0
        self._tries_count = 0
        self._echo_completed = False    # set by the falling edge, consumed by
        self._first_ping     = True     # the ping loop just before it triggers
        self._faulted        = False

        self._lock = threading.Lock()
        self._cb = self._pi.callback(echo, pigpio.EITHER_EDGE, self._on_edge)

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
            with self._lock:
                # A complete pulse means the sensor is alive, even when the
                # width is out of range (far object / internal timeout).
                self._echo_completed = True
                if 0.0 < d <= self.max_distance_cm:
                    self._distance_cm = d
                    self._last_valid = time.monotonic()

    def _last_ping_try_check(self):
        """Run just before each trigger: did the previous ping complete an echo
        pulse? Counts consecutive failures -- no rising edge at all, or a rise
        with no matching fall -- and latches a fault at max_ping_tries."""
        with self._lock:
            completed, self._echo_completed = self._echo_completed, False

        if self._first_ping:                # nothing triggered yet, nothing to judge
            self._first_ping = False
        elif completed:
            self._tries_count = 0
            if self._faulted:
                self._faulted = False
                print("[ultrasonic] echo recovered -- sensor healthy again")
        else:
            self._tries_count += 1
            if self._tries_count >= self.max_ping_tries and not self._faulted:
                self._faulted = True
                print(f"[ultrasonic] no echo for {self._tries_count} consecutive "
                      f"pings -- sensor faulted")

        # Drop any incomplete pulse: a stale rise tick paired with a later
        # spurious falling edge can yield a plausible-looking bogus distance
        # (tickDiff wraps every ~71.6 min).
        self._rise_tick = None

    def _ping_loop(self):
        while self._running:
            self._last_ping_try_check()
            try:
                self._pi.gpio_trigger(self.trigger, 10, 1)   # 10 us HIGH pulse
            except Exception:                       # noqa: BLE001
                pass
            time.sleep(self.ping_interval)

    # -- API: drop-in for Ultrasonic.get_distance() ------------------------
    @property
    def healthy(self):
        """False once max_ping_tries consecutive pings have failed. Latches --
        clears only when an echo pulse completes again."""
        return not self._faulted

    def get_distance(self):
        """Latest distance in cm (int). Non-blocking. If no valid echo has
        arrived recently (object out of range / never measured), report the max
        range -- a normal 'far / clear' value, not a failure sentinel.

        A faulted sensor reports the max range too, so the collision guard sees
        'clear' rather than a phantom obstacle; check .healthy to distinguish."""
        if self._faulted:
            return int(self.max_distance_cm)
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
