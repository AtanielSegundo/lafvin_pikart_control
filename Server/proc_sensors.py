#!/usr/bin/python3
"""
P_sensors: every high-frequency edge and I2C reader, in one process.

What lives here and why:

  * **Quadrature encoders (pigpio).** The reason this process exists. At speed
    the four motors generate on the order of 24k Python callback invocations per
    second; in the single-process design every one of those competed for the GIL
    with the 20 Hz control loop and the aiohttp event loop. Here they have a core
    and a GIL essentially to themselves.
  * **Ultrasonic + the front collision guard.** Moved out of
    ``DriveController._dist_guard``. The pigpio reader is cheap, but the RPi.GPIO
    fallback's ``pulseIn`` is a Python busy-wait of up to 18 ms, five times per
    reading -- the most straightforwardly cpu-bound code in the project. The
    guard's hysteresis and the reported-distance TTL come with it, so what
    crosses the process boundary is one decided boolean.
  * **MPU6050 heading.** I2C-bound, but its 50 Hz integration is sensitive to
    scheduling jitter: a late sample integrates a stale rate over a longer dt.
  * **ADC and IR line sensors**, so the web layer stops reading GPIO and I2C
    itself (Server.sendLine / sendLight / Power used to).
  * **Standstill detection**, which gates the gyro's yaw integration. It lives
    here because it is decided from the encoder totals, and this is the process
    that owns them -- see standstill.py for why the encoders get that vote and
    the gyro does not.

Everything is published to shared arrays, never to a queue: these are all
latest-value-wins signals, and a queue in the encoder path would mean pickling
24k times a second.

Hardware handles are opened HERE, after the fork. Inheriting a pigpio socket or
an smbus fd from the parent would give two processes one connection.
"""
from __future__ import annotations

import os
import signal
import sys
import threading
import time

from config import CONFIG, ENCODER_TAGS
import ipc as ipc_mod

# The commands this process accepts -- see proc_aux.HANDLED for why this is data.
HANDLED = frozenset({
    "calibrate_imu", "reset_encoder_totals",
})


# ---------------------------------------------------------------------------
# Front distance + collision guard (was DriveController._dist_guard)
# ---------------------------------------------------------------------------
class FrontGuardMonitor:
    """Polls the distance sensor, decides the guard, publishes both.

    Keeps the original semantics exactly:

      * the guard trips below ``minimum_front_distance_cm`` and releases only
        above ``limit + hysteresis``, so sensor jitter cannot chatter it;
      * it decides on ``min()`` of the last 3 VALID readings -- the closest
        recent echo, not an average, because averaging an obstacle with a miss
        reads as "further away than it is";
      * the REPORTED distance expires after ``front_distance_ttl_s`` while the
        guard's own history does not. Only the number shown to the operator goes
        stale; changing the guard's behaviour was explicitly out of scope when
        that TTL was added, and moving the code between processes is not the
        moment to revisit it.
    """

    def __init__(self, sensor, ipc: ipc_mod.IPC, config=CONFIG):
        self.sensor = sensor
        self.ipc = ipc
        self.config = config
        self.recent: list[float] = []          # last <=3 valid readings
        self.reported = None
        self.reported_ts = 0.0
        self.engaged = False
        self._thread = None
        self._stop = threading.Event()

    HYSTERESIS_CM = 5

    def poll_once(self) -> None:
        now = time.monotonic()
        try:
            raw = self.sensor.get_distance()
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("sensors", f"distance read failed: {exc}")
            raw = None

        # Validity is sensor-agnostic: the pigpio reader returns real cm (incl. a
        # large far/clear value), while the RPi.GPIO fallback uses 255 as its
        # "failed read" sentinel -- drop that (and None), keep the rest.
        if raw is not None and raw > 0 and raw != 255:
            self.recent.append(raw)
            del self.recent[:-3]
            self.reported = raw
            self.reported_ts = now
        elif (self.reported is not None and
              (now - self.reported_ts) > self.config.control.front_distance_ttl_s):
            self.reported = None

        if self.recent:
            closest = min(self.recent)
            limit = self.config.control.minimum_front_distance_cm
            if closest < limit:
                self.engaged = True
            elif closest > limit + self.HYSTERESIS_CM:
                self.engaged = False

        healthy = getattr(self.sensor, "healthy", True)
        self.ipc.publish_distance(self.reported, self.engaged, healthy)

    def _run(self) -> None:
        # Same cadence as the old guard thread: twice the control rate, so the
        # loop never acts on a reading older than half a tick.
        period = 1.0 / (2 * self.config.control.loop_hz)
        while not self._stop.is_set():
            start = time.monotonic()
            self.poll_once()
            self._stop.wait(max(0.0, period - (time.monotonic() - start)))

    def start(self) -> None:
        # Its own thread because the RPi.GPIO fallback BLOCKS for up to ~90 ms
        # per reading; that must not delay the encoder publication.
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="FrontGuard")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)


class AdcMonitor:
    """Battery + photoresistors, on their own thread.

    Separate from the publish loop because ``Adc.recvADC`` re-reads until two
    samples agree, on the same I2C bus P_control uses for the PCA9685, so a
    single call can stall for milliseconds.
    """

    def __init__(self, adc, ipc: ipc_mod.IPC, config=CONFIG):
        self.adc = adc
        self.ipc = ipc
        self.config = config
        self._thread = None
        self._stop = threading.Event()

    def poll_once(self) -> None:
        try:
            light_l = self.adc.recvADC(0)
            light_r = self.adc.recvADC(1)
            battery = self.adc.recvADC(2) * 5
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("sensors", f"ADC read failed: {exc}")
            return
        self.ipc.publish_adc(round(battery, 2), light_l, light_r)

    def _run(self) -> None:
        period = 1.0 / max(0.1, self.config.process.adc_poll_hz)
        while not self._stop.is_set():
            start = time.monotonic()
            self.poll_once()
            self._stop.wait(max(0.0, period - (time.monotonic() - start)))

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="AdcMonitor")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)


# ---------------------------------------------------------------------------
# Process body
# ---------------------------------------------------------------------------
def run(ipc: ipc_mod.IPC, config=CONFIG) -> None:
    """Entry point for P_sensors. Never returns until ``ipc.stop_evt`` is set."""
    # Ctrl-C reaches the whole process group; the parent orchestrates shutdown
    # through stop_evt, so a child raising KeyboardInterrupt would just print a
    # traceback over the parent's log.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    p = config.process
    ipc_mod.apply_process_tuning("sensors", cpu=p.sensors_cpu,
                                 nice=p.sensors_nice)

    pi          = None
    encoders    = None
    gyro        = None
    guard       = None
    adc_monitor = None
    line        = None

    try:
        # -- pigpio: ONE connection for this process, opened after the fork ---
        try:
            import pigpio
            pi = pigpio.pi()
            if not pi.connected:
                ipc.log("sensors", "pigpiod not reachable (sudo pigpiod); "
                                   "encoders will be simulated")
                pi = None
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"pigpio unavailable ({exc})")
            pi = None

        # -- encoders --------------------------------------------------------
        from encoders import WheelEncoders
        encoders = WheelEncoders(config.sides, pi=pi)
        encoders.begin()
        ipc.log("sensors", f"encoders started "
                           f"({'hardware' if encoders.using_hardware else 'simulated'})")

        # -- standstill detector ---------------------------------------------
        # Built BEFORE the gyro, because the gyro takes its predicate as a
        # constructor argument and starts its integration thread immediately.
        from standstill import StandstillDetector
        standstill = StandstillDetector(config)
        # Seed it from the current totals so the gyro's first integration steps
        # see "moving" (the default) rather than a bogus standstill derived from
        # a baseline that does not exist yet.
        try:
            standstill.update(encoders.raw_totals())
        except Exception:                                       # noqa: BLE001
            pass

        # -- gyro ------------------------------------------------------------
        try:
            from heading import GyroMPU
            # The predicate is called from the gyro's own 50 Hz thread on every
            # integration step, and returns False only while the wheels have
            # been provably quiet -- which is what stops a parked kart from
            # integrating its residual bias into phantom rotation.
            gyro = GyroMPU(sample_rate=50.0,
                           fn_to_check_kart_movement=standstill.is_moving)
            if gyro.is_connected():
                ipc.log("sensors", "calibrating gyro bias -- keep the kart STILL")
                gyro.calibrate()
            else:
                ipc.log("sensors", "MPU6050 not detected; heading falls back "
                                   "to encoders")
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"gyro unavailable ({exc}); heading falls back "
                               f"to encoders")
            gyro = None

        # -- ultrasonic + front guard ----------------------------------------
        sensor = None
        try:
            from ultrasonic_pigpio import UltrasonicPigpio
            sensor = UltrasonicPigpio(pi=pi)
            ipc.log("sensors", "ultrasonic using pigpio (hardware-timed echo)")
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"ultrasonic pigpio unavailable ({exc}); "
                               f"RPi.GPIO fallback")
            try:
                from Ultrasonic import Ultrasonic
                sensor = Ultrasonic()
            except Exception as exc2:                           # noqa: BLE001
                ipc.log("sensors", f"no ultrasonic sensor ({exc2}); front guard "
                                   f"disabled")
        if sensor is not None:
            guard = FrontGuardMonitor(sensor, ipc, config)
            guard.start()

        # -- ADC -------------------------------------------------------------
        try:
            from ADC import Adc
            adc_monitor = AdcMonitor(Adc(), ipc, config)
            adc_monitor.start()
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"ADC unavailable ({exc})")

        # -- IR line sensors -------------------------------------------------
        try:
            from Line_Tracking import Line_Tracking
            line = Line_Tracking()
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"line sensors unavailable ({exc})")

        _publish_loop(ipc, config, encoders, gyro, line, standstill)

    except Exception as exc:                                    # noqa: BLE001
        import traceback
        ipc.log("sensors", f"FATAL: {exc}")
        traceback.print_exc(file=sys.stderr)
    finally:
        if guard is not None:
            guard.stop()
        if adc_monitor is not None:
            adc_monitor.stop()
        if gyro is not None:
            try:
                gyro.stop()
            except Exception:                                   # noqa: BLE001
                pass
        if encoders is not None:
            try:
                encoders.stop()
            except Exception:                                   # noqa: BLE001
                pass
        if pi is not None:
            try:
                pi.stop()
            except Exception:                                   # noqa: BLE001
                pass
        ipc.log("sensors", "stopped")


def _publish_loop(ipc: ipc_mod.IPC, config, encoders, gyro, line,
                  standstill=None) -> None:
    p = config.process
    period = 1.0 / max(1.0, p.sensor_publish_hz)
    line_every = max(1, int(round(p.sensor_publish_hz /
                                  max(1.0, p.line_poll_hz))))
    tick = 0
    prev_yaw = None
    prev_yaw_ts = None

    while not ipc.stop_evt.is_set():
        start = time.monotonic()

        # -- encoder totals: raw, monotonic, sign-free ----------------------
        # P_control differences these against its own previous copy. Publishing
        # totals rather than consuming deltas means a skipped publication (a late
        # thread, a stalled bus) self-corrects on the next tick instead of losing
        # counts the way a cross-process read-and-reset handshake would.
        totals = None
        try:
            totals = encoders.raw_totals()
            ipc.publish_encoders(totals)
        except Exception as exc:                                # noqa: BLE001
            ipc.log("sensors", f"encoder publish failed: {exc}")

        # -- standstill: the gyro's integration gate -------------------------
        # Fed from the same totals that were just published, at this loop's
        # rate rather than the gyro's. The gyro only ever READS the resulting
        # flag (a plain attribute), so it never blocks its 50 Hz thread on this
        # one.
        if standstill is not None and totals is not None:
            try:
                standstill.update(totals)
            except Exception as exc:                            # noqa: BLE001
                ipc.log("sensors", f"standstill update failed: {exc}")

        # -- gyro yaw -------------------------------------------------------
        if gyro is not None:
            try:
                yaw, connected, sample_ts = gyro.sample()
                ipc.publish_gyro(yaw, connected, sample_ts)
                # Hand the measured rate back to the detector. This is the
                # safety valve for an in-place turn, where the wheels scrub
                # sideways and may register far fewer counts than the rotation
                # deserves -- without it, a slow spin could be mistaken for
                # standstill and have its rotation discarded.
                if standstill is not None and prev_yaw is not None \
                        and sample_ts > prev_yaw_ts:
                    standstill.note_gyro_rate(
                        (yaw - prev_yaw) / (sample_ts - prev_yaw_ts))
                if sample_ts != prev_yaw_ts:
                    prev_yaw, prev_yaw_ts = yaw, sample_ts
            except Exception as exc:                            # noqa: BLE001
                ipc.log("sensors", f"gyro publish failed: {exc}")

        # -- IR line sensors (slow divisor) ---------------------------------
        if line is not None and tick % line_every == 0:
            try:
                ipc.publish_line(line.read())
            except Exception:                                   # noqa: BLE001
                pass

        if standstill is not None:
            try:
                ipc.publish_standstill(standstill.telemetry())
            except Exception:                                   # noqa: BLE001
                pass

        _handle_commands(ipc, gyro, encoders)

        tick += 1
        ipc.stop_evt.wait(max(0.0, period - (time.monotonic() - start)))


def _handle_commands(ipc: ipc_mod.IPC, gyro, encoders) -> None:
    for cmd in ipc_mod.drain(ipc.sensors_q, limit=16):
        name = getattr(cmd, "name", None) or (
            cmd.get("type") if isinstance(cmd, dict) else None)
        if name == "calibrate_imu":
            if gyro is None:
                ipc.log("sensors", "calibrate_imu ignored: no gyro")
                continue
            # Runs inline, blocking this loop for ~1.5 s (200 samples at 5 ms).
            # That is deliberate: the kart must be still to calibrate anyway, and
            # doing it on a side thread would race the integration loop for the
            # I2C bus for no benefit.
            ipc.log("sensors", "recalibrating gyro bias -- keep the kart STILL")
            ok = gyro.calibrate()
            ipc.event("imu_calibrated", ok=bool(ok))
        elif name == "reset_encoder_totals":
            try:
                encoders.reset_totals()
                ipc.log("sensors", "encoder lifetime totals reset")
            except Exception as exc:                            # noqa: BLE001
                ipc.log("sensors", f"reset_totals failed: {exc}")
        else:
            ipc.log("sensors", f"unknown command {name!r} ignored")


if __name__ == "__main__":
    # Standalone: publish sensors and print them, without the rest of the stack.
    bundle = ipc_mod.IPC()
    threading.Thread(target=run, args=(bundle,), daemon=True).start()
    try:
        while True:
            time.sleep(0.5)
            totals, _ = bundle.read_encoder_totals()
            yaw, _sts, connected, _ts = bundle.read_gyro()
            cm, engaged, healthy, _ = bundle.read_distance()
            print(f"enc={[totals[t] for t in ENCODER_TAGS]} "
                  f"yaw={yaw:+7.2f} ({'ok' if connected else 'disc'}) "
                  f"dist={cm} guard={engaged} healthy={healthy}")
    except KeyboardInterrupt:
        bundle.stop_evt.set()
        time.sleep(0.5)
