#!/usr/bin/python3
"""
P_aux: LED animations, the buzzer, and the legacy autonomous modes.

Everything here is low priority (``aux_nice``, default +10) because none of it is
allowed to preempt the control loop. It is also the process where the worst of
the old CPU behaviour lived:

  * ``Line_Tracking.run`` and ``Light.run`` were ``while True`` loops with no
    sleep at all -- each pinned a core for as long as its mode was active, and
    under one GIL that came straight out of the control loop's budget. Both now
    sleep, and both run niced.
  * the LED animations do numpy bit-twiddling plus an SPI transfer per frame, in
    tight loops.

Two things this process is NOT allowed to do: touch the PCA9685 (motors and
servos belong to P_control -- the legacy modes get an ipc.RemoteMotor that
forwards duties over the command queue) and open a second smbus client for the
photoresistors (they get an ipc.RemoteAdc reading what P_sensors publishes).

It is started eagerly at boot and then idles on its queue, rather than being
spawned when a mode is selected: by then P_web has threads and an event loop
running, and forking from a threaded process risks a child deadlocking on a lock
that was held at fork time. An idle process costs ~20 MB and no CPU.
"""
from __future__ import annotations

import signal
import sys
import threading
import time

from config import CONFIG
import ipc as ipc_mod

# The commands this process accepts. Declared as data, not left implicit in the
# if/elif chain below, so a mismatch between what P_web enqueues and what this
# side handles is a test failure rather than a command that silently vanishes --
# the most likely way to break a queue-based design.
HANDLED = frozenset({
    "led", "led_mode", "buzzer", "start_mode", "stop_mode",
})


class AuxWorker:
    def __init__(self, ipc: ipc_mod.IPC, config=CONFIG):
        self.ipc = ipc
        self.config = config
        self.led = None
        self.buzzer = None
        self.motor = ipc_mod.RemoteMotor(ipc.control_q)
        self.servo = ipc_mod.RemoteServo(ipc.control_q)
        self.adc = ipc_mod.RemoteAdc(ipc)

        # One slot for "the long-running thing that owns the motors", and one for
        # the LED animation, which is independent of it.
        self._mode_thread: threading.Thread | None = None
        self._mode_stop = threading.Event()
        self._led_thread: threading.Thread | None = None
        self._led_stop = threading.Event()

    # -- devices owned here (opened lazily, so an absent one is survivable) --
    def _get_led(self):
        if self.led is None:
            from Led import Led
            self.led = Led()
        return self.led

    def _get_buzzer(self):
        if self.buzzer is None:
            from Buzzer import Buzzer
            self.buzzer = Buzzer()
        return self.buzzer

    # -- cooperative stop (replaces Thread.stop_thread) ---------------------
    def stop_mode(self) -> None:
        """Stop the running autonomous mode and let it unwind.

        The old path was ``stop_thread()``: ctypes-injecting SystemExit into the
        thread seven times. That could land anywhere -- including halfway through
        an I2C transaction or while holding a lock -- which is why stopping a mode
        used to sometimes leave the bus wedged. The loops now poll a stop event
        and shut their own motors off on the way out.
        """
        self._mode_stop.set()
        t = self._mode_thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
            if t.is_alive():
                self.ipc.log("aux", f"mode thread {t.name} did not stop in 2 s")
        self._mode_thread = None
        self.motor.setMotorModel(0, 0, 0, 0)

    def stop_led(self) -> None:
        self._led_stop.set()
        t = self._led_thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._led_thread = None

    def _start_mode(self, name: str, target, *args, **kwargs) -> None:
        self.stop_mode()
        self._mode_stop = threading.Event()
        kwargs["stop_evt"] = self._mode_stop
        self._mode_thread = threading.Thread(target=self._guarded, name=name,
                                             args=(name, target, args, kwargs),
                                             daemon=True)
        self._mode_thread.start()
        self.ipc.log("aux", f"mode {name} started")

    def _guarded(self, name, target, args, kwargs) -> None:
        try:
            target(*args, **kwargs)
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("aux", f"mode {name} crashed: {exc}")
        finally:
            # A mode that dies must not leave the kart driving: P_control holds
            # the last duty it was given and has no way to know the sender is
            # gone.
            self.motor.setMotorModel(0, 0, 0, 0)

    # -- command handling --------------------------------------------------
    def handle(self, cmd) -> None:
        name = getattr(cmd, "name", None) or (
            cmd.get("type") if isinstance(cmd, dict) else None)
        get = (cmd.get if hasattr(cmd, "get") else lambda k, d=None: d)

        if name == "led":
            try:
                self._get_led().ledIndex(int(get("index", 255) or 255),
                                         int(get("r", 0) or 0),
                                         int(get("g", 0) or 0),
                                         int(get("b", 0) or 0))
            except Exception as exc:                            # noqa: BLE001
                self.ipc.log("aux", f"led failed: {exc}")

        elif name == "led_mode":
            mode = str(get("mode", "0"))
            self.stop_led()
            if mode in ("0", "None", ""):
                return
            try:
                led = self._get_led()
            except Exception as exc:                            # noqa: BLE001
                self.ipc.log("aux", f"led unavailable: {exc}")
                return
            if mode == "1":
                led.ledMode(mode)
                return
            self._led_stop = threading.Event()
            self._led_thread = threading.Thread(
                target=self._run_led_mode, args=(led, mode), daemon=True,
                name="LedMode")
            self._led_thread.start()

        elif name == "buzzer":
            on = get("on", None)
            if on is None:
                on = cmd.arg(0) if hasattr(cmd, "arg") else "0"
            value = "1" if on in (True, "1", "true", "True", 1) else "0"
            try:
                self._get_buzzer().run(value)
            except Exception as exc:                            # noqa: BLE001
                self.ipc.log("aux", f"buzzer failed: {exc}")

        elif name == "start_mode":
            self._dispatch_mode(str(get("mode", "one")))

        elif name == "stop_mode":
            self.stop_mode()

        else:
            self.ipc.log("aux", f"unknown command {name!r} ignored")

    def _run_led_mode(self, led, mode: str) -> None:
        """``Led.ledMode`` loops internally with no exit condition, so it cannot
        be asked to stop; it used to be killed with stop_thread(). Run one
        animation pass at a time and re-check the stop event between passes."""
        try:
            while not self._led_stop.is_set():
                led.ledMode(mode)
                if mode in ("0", "1"):
                    break
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("aux", f"led mode {mode} crashed: {exc}")
        finally:
            try:
                led.strip.set_all_led_color(0, 0, 0)
            except Exception:                                   # noqa: BLE001
                pass

    def _dispatch_mode(self, mode: str) -> None:
        if mode == "two":            # photoresistor following
            from Light import Light
            self._start_mode("LightMode", Light().run, motor=self.motor,
                             adc=self.adc)
        elif mode == "three":        # ultrasonic obstacle avoidance
            from Ultrasonic import Ultrasonic
            self._start_mode("UltrasonicMode", Ultrasonic().run,
                             motor=self.motor, servo=self.servo)
        elif mode == "four":         # line following
            from Line_Tracking import Line_Tracking
            self._start_mode("LineMode", Line_Tracking().run, motor=self.motor)
        else:
            self.stop_mode()

    def shutdown(self) -> None:
        self.stop_mode()
        self.stop_led()
        try:
            if self.buzzer is not None:
                self.buzzer.run("0")
        except Exception:                                       # noqa: BLE001
            pass
        try:
            if self.led is not None:
                self.led.strip.set_all_led_color(0, 0, 0)
        except Exception:                                       # noqa: BLE001
            pass


def run(ipc: ipc_mod.IPC, config=CONFIG) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    ipc_mod.apply_process_tuning("aux", nice=config.process.aux_nice)

    worker = AuxWorker(ipc, config)

    def _bail(signum, frame):
        ipc.stop_evt.set()

    signal.signal(signal.SIGTERM, _bail)

    try:
        while not ipc.stop_evt.is_set():
            try:
                cmd = ipc.aux_q.get(timeout=0.5)
            except Exception:                                   # Empty / closed
                continue
            try:
                worker.handle(cmd)
            except Exception as exc:                            # noqa: BLE001
                ipc.log("aux", f"command {cmd!r} failed: {exc}")
    except Exception as exc:                                    # noqa: BLE001
        import traceback
        ipc.log("aux", f"FATAL: {exc}")
        traceback.print_exc(file=sys.stderr)
    finally:
        worker.shutdown()
        ipc.log("aux", "stopped")
