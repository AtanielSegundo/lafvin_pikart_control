#!/usr/bin/python3
"""
P_control: the closed-loop drivetrain, and the ONLY owner of the PCA9685.

This process runs ``DriveController.run_loop`` on its main thread under
SCHED_FIFO, pinned to a core of its own. That is the whole point of the split:
the control arithmetic was never cpu-bound (microseconds per tick), but it was
waiting on the GIL behind ~24k encoder callbacks and 500 telemetry
serialisations a second, so a 50 ms period routinely slipped.

Sole ownership of the PCA9685 is a hard rule, not a preference. ``Motor`` is a
per-process singleton: after a fork each process gets its own instance, so two
processes would hold two clients on one chip, and ``setMotorModel`` is four
separate I2C register writes that would interleave mid-update. Both the motors
and the servos hang off that chip, so both live here, and the legacy autonomous
modes in P_aux reach them through ipc.RemoteMotor / RemoteServo.

Feedback arrives from P_sensors through shared memory, wrapped in adapters that
duck-type the in-process objects (ipc.SharedEncoderReader / SharedGyroReader /
SharedFrontGuard), so DriveController itself contains no IPC at all and stays
testable against a simulated plant.
"""
from __future__ import annotations

import signal
import sys
import time

from config import CONFIG
import ipc as ipc_mod
import protocol
from protocol import Command


def gains_as_dict(gains) -> dict:
    """A gains dataclass as plain JSON-safe scalars.

    Used for the telemetry envelope so the tuning UI can show what the control
    loop is ACTUALLY running, rather than whatever the page was last told. With
    the gains living in P_control and editable at runtime, the browser's idea
    of them and the loop's can diverge -- and a tuning table that lies about
    the current value is worse than none.
    """
    import dataclasses
    out = {}
    for field in dataclasses.fields(gains):
        value = getattr(gains, field.name)
        if isinstance(value, bool):
            out[field.name] = value
        elif isinstance(value, (int, float)):
            out[field.name] = round(float(value), 6)
    return out


class ControlApplier:
    """Applies queued commands to the drivetrain.

    A second CommandRouter, on this side of the queue. P_web's router parses,
    validates and decides *where* a command goes; this one decides *what it
    does*. Splitting it that way keeps the "add a command = register a handler"
    property the protocol layer was built for, on both sides.

    Note there is no ``Mode`` gate here. P_web owns the mode and refuses commands
    that the current mode disallows before they are ever enqueued; this side
    applies what it is given. Duplicating the gate would mean two sources of
    truth for it.
    """

    def __init__(self, drive, motor, servo, ipc: ipc_mod.IPC, config=CONFIG):
        self.drive = drive
        self.motor = motor
        self.servo = servo
        self.ipc = ipc
        self.config = config
        self.router = protocol.CommandRouter()
        self._build()

    def _build(self) -> None:
        r = self.router
        r.register("motor", self._h_motor)
        # No "mecanum" handler: the joystick mix is computed in P_web, where the
        # stick geometry arrives, and crosses the queue as four plain duties on
        # the "motor" route. A handler here would be unreachable.
        r.register("car_rotate", self._h_car_rotate)
        r.register("drive", self._h_drive)
        r.register("drive_distance", self._h_drive_distance)
        r.register("turn", self._h_turn)
        r.register("goto", self._h_goto)
        r.register("raw_turn_schedule", self._h_raw_turn_schedule)
        r.register("reset_odometry", self._h_reset_odometry)
        r.register("set_sign", self._h_set_sign)
        r.register("set_gains", self._h_set_gains)
        r.register("servo", self._h_servo)
        r.register("release", self._h_release)
        r.register("stop", self._h_stop)

    @property
    def handled(self) -> frozenset:
        """Command names this process accepts -- see proc_aux.HANDLED."""
        return frozenset(self.router._handlers)

    def pump(self, limit: int = 64) -> None:
        """Drain and apply everything queued. Called once per control tick."""
        for cmd in ipc_mod.drain(self.ipc.control_q, limit=limit):
            name = getattr(cmd, "name", None)
            if name is not None and not self.router.has(name):
                # Silently dropping this would be the hardest class of bug in a
                # queue design to notice: the UI works, the command just never
                # happens.
                self.ipc.log("control", f"unknown command {name!r} ignored")
                continue
            try:
                self.router.dispatch(cmd)
            except Exception as exc:                            # noqa: BLE001
                self.ipc.log("control", f"command {cmd!r} failed: {exc}")

    # -- raw duty ----------------------------------------------------------
    def _h_motor(self, c: Command) -> None:
        duty = c.get("duty")
        if duty and len(duty) >= 4:
            d = [int(x) for x in duty[:4]]
        else:
            d = [c.arg_int(i) for i in range(4)]
        self.drive.release()              # stop the PID fighting the raw duty
        self.motor.setMotorModel(*d)

    def _h_car_rotate(self, c: Command) -> None:
        """Continuous spin (CMD_CAR_ROTATE). ``Motor.Rotate`` loops internally and
        is stopped cooperatively through ``stop_rotate``; it runs on a side thread
        so the control loop keeps ticking (and keeps publishing telemetry)."""
        import threading
        angle = int(c.num("angle", 0, 0))
        stop = bool(c.get("stop", False))
        self.motor.stop_rotate()
        existing = getattr(self, "_rotate_thread", None)
        if existing is not None and existing.is_alive():
            existing.join(timeout=1.0)
        if stop:
            self.motor.setMotorModel(0, 0, 0, 0)
            return
        self.drive.release()
        self._rotate_thread = threading.Thread(
            target=self.motor.Rotate, args=(angle,), daemon=True, name="Rotate")
        self._rotate_thread.start()

    # -- closed loop -------------------------------------------------------
    def _h_drive(self, c: Command) -> None:
        # Pass the ingress timestamp through so command_timeout (the teleop
        # dead-man switch) measures the age of the operator's input, queue hop
        # included, rather than restarting the clock on arrival here.
        issued = c.get("ts")
        self.drive.set_twist(c.num("linear", 0, 0.0), c.num("angular", 1, 0.0),
                             issued_at=float(issued) if issued is not None else None)

    def _h_drive_distance(self, c: Command) -> None:
        self.drive.drive_distance(c.num("distance", 0, 0.0),
                                  c.num("speed", 1, 0.2))

    def _h_turn(self, c: Command) -> None:
        self.drive.turn_in_place(c.num("angle", 0, 0.0), c.num("speed", 1, 1.0))

    def _h_goto(self, c: Command) -> None:
        theta = c.get("theta")
        self.drive.goto_pose(c.num("x", 0, 0.0), c.num("y", 1, 0.0),
                             float(theta) if theta is not None else None)

    def _h_raw_turn_schedule(self, c: Command) -> None:
        params = c.get("fn_params")
        self.drive.raw_turn_schedule(
            str(c.get("turn_fn", "trapezoid")), bool(c.get("ccw", True)),
            int(c.num("pwm", 0, 2000)), int(c.num("min_pwm", 1, 1000)),
            c.num("final_turn_angle", 2, 90.0),
            params if isinstance(params, dict) else {})

    def _h_reset_odometry(self, c: Command) -> None:
        self.drive.reset_odometry()

    def _h_release(self, c: Command) -> None:
        self.drive.release()

    def _h_stop(self, c: Command) -> None:
        self.motor.stop_rotate()
        self.drive.release()
        self.motor.setMotorModel(0, 0, 0, 0)

    # -- calibration -------------------------------------------------------
    def _h_set_sign(self, c: Command) -> None:
        """Flip encoder count sign(s) at runtime.

        The signs live only in this process now: P_sensors publishes raw,
        sign-free totals and the resolution happens on this side (see
        ipc.SharedEncoderReader). So this mutates the local SideMapping and the
        very next tick picks it up -- no shared mutable dict across processes.
        """
        signs = self.drive.encoders.sides.signs
        bulk = c.get("signs")
        if isinstance(bulk, dict):
            for tag, s in bulk.items():
                if tag in signs:
                    signs[tag] = 1 if float(s) >= 0 else -1
        else:
            motor = str(c.get("motor", c.arg(0)))
            if motor in signs:
                signs[motor] = 1 if c.num("sign", 1, 1) >= 0 else -1
        self.ipc.log("control", f"encoder signs now {dict(signs)}")

    #: Fields of PositionGains / HeadingGains that may be tuned at runtime,
    #: with the bounds a value has to fall inside. Declared as data so the UI,
    #: the validator and the tests all read the same table, and so a typo in a
    #: field name is rejected rather than silently setting an attribute nothing
    #: reads. Bounds are sanity rails, not tuning advice: they exist to stop a
    #: fat-fingered 22000 from pinning the motors.
    GAIN_LIMITS = {
        "position": {
            "kp": (0.0, 60000.0), "ki": (0.0, 60000.0), "kd": (0.0, 60000.0),
            "output_limit": (0.0, 4095.0), "integral_limit": (0.0, 4095.0),
            "tolerance": (0.001, 0.5), "stop_speed": (0.0, 1.0),
            "max_time": (0.5, 120.0), "min_move_duty": (0.0, 4095.0),
            "decel_gain": (0.0, 60000.0),
        },
        "heading": {
            "kp": (0.0, 60000.0), "ki": (0.0, 60000.0), "kd": (0.0, 60000.0),
            "output_limit": (0.0, 4095.0), "integral_limit": (0.0, 4095.0),
            "decel_gain": (0.0, 60000.0), "min_turn_duty": (0.0, 4095.0),
            "pulse_floor": (0, 1), "tolerance": (0.0001, 1.5),
            "settle_rate": (0.01, 20.0), "settle_ticks": (1, 50),
            "max_time": (0.5, 120.0),
        },
    }
    _INT_FIELDS = {"settle_ticks"}
    _BOOL_FIELDS = {"pulse_floor"}

    def _h_set_gains(self, c: Command) -> None:
        """Retune the position / heading loops without restarting the stack.

        Two things have to change together, and missing either one is why this
        is not a one-liner:

        1. ``config.position`` / ``config.heading`` are frozen dataclasses, and
           the loop reads several of their fields LIVE every tick (tolerance,
           decel_gain, min_*_duty, max_time, settle_*). Those are replaced via
           dataclasses.replace.
        2. kp/ki/kd/output_limit/integral_limit were COPIED into the PID
           objects when they were constructed, so replacing the config alone
           would silently change nothing for the terms that matter most. The
           live PIDs are patched too.

        Only this process's copy of CONFIG is touched -- after the fork each
        process has its own, and the gains are meaningful only here.
        """
        import dataclasses

        section = str(c.get("section", "")).lower()
        limits = self.GAIN_LIMITS.get(section)
        if limits is None:
            self.ipc.log("control", f"set_gains: unknown section {section!r}")
            return

        values = c.get("values")
        if not isinstance(values, dict) or not values:
            self.ipc.log("control", "set_gains: no values")
            return

        clean, rejected = {}, []
        for key, raw in values.items():
            if key not in limits:
                rejected.append(f"{key} (unknown)")
                continue
            try:
                number = float(raw)
            except (TypeError, ValueError):
                rejected.append(f"{key} (not a number)")
                continue
            if number != number:                               # NaN
                rejected.append(f"{key} (NaN)")
                continue
            lo, hi = limits[key]
            if not (lo <= number <= hi):
                rejected.append(f"{key}={number:g} (outside [{lo:g}, {hi:g}])")
                continue
            if key in self._BOOL_FIELDS:
                clean[key] = bool(number)
            elif key in self._INT_FIELDS:
                clean[key] = int(round(number))
            else:
                clean[key] = number

        if rejected:
            self.ipc.log("control", f"set_gains rejected: {', '.join(rejected)}")
        if not clean:
            return

        current = getattr(self.config, section)
        updated = dataclasses.replace(current, **clean)
        object.__setattr__(self.config, section, updated)

        # Push into the already-built PIDs. A move or turn in flight keeps its
        # integrator, which is deliberate: resetting mid-move would make the
        # kart lurch, and the operator retuning gains is watching the kart, not
        # asking for a discontinuity.
        if section == "position":
            for pid in (self.drive.pos_left, self.drive.pos_right):
                pid.kp, pid.ki, pid.kd = updated.kp, updated.ki, updated.kd
                pid.output_limit = updated.output_limit
                pid.integral_limit = updated.integral_limit
        else:
            pid = self.drive.heading_pid
            pid.kp, pid.ki, pid.kd = updated.kp, updated.ki, updated.kd
            pid.output_limit = updated.output_limit
            pid.integral_limit = updated.integral_limit

        self.ipc.log("control", f"{section} gains updated: "
                                f"{', '.join(f'{k}={v:g}' for k, v in clean.items())}")
        self.ipc.event("gains_changed", section=section,
                       values=gains_as_dict(updated))

    # -- peripherals on the same PCA9685 -----------------------------------
    def _h_servo(self, c: Command) -> None:
        if self.servo is None:
            return
        self.servo.setServoPwm(str(c.get("channel", c.arg(0))),
                               int(c.num("angle", 1, 90)))


# ---------------------------------------------------------------------------
# Process body
# ---------------------------------------------------------------------------
def run(ipc: ipc_mod.IPC, config=CONFIG) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    p = config.process
    ipc_mod.apply_process_tuning("control", cpu=p.control_cpu,
                                 nice=p.control_nice,
                                 rt_priority=p.control_rt_priority)

    motor = None
    drive = None

    def _bail(signum, frame):
        """SIGTERM: brake first, then unwind. The supervisor escalates to this
        when the heartbeat goes stale, and the PCA9685 latches its last duty in
        hardware -- so stopping the motors comes before anything else."""
        try:
            if motor is not None:
                motor.stop_rotate()
                motor.setMotorModel(0, 0, 0, 0)
        except Exception:                                       # noqa: BLE001
            pass
        ipc.stop_evt.set()
        if drive is not None:
            drive.stop_loop()

    signal.signal(signal.SIGTERM, _bail)

    try:
        from Motor import Motor
        from drive_controller import DriveController

        motor = Motor()

        servo = None
        try:
            from servo import Servo
            servo = Servo()
        except Exception as exc:                                # noqa: BLE001
            ipc.log("control", f"servo unavailable ({exc})")

        encoders = ipc_mod.SharedEncoderReader(ipc, config.sides, config=config)
        gyro = ipc_mod.SharedGyroReader(ipc, config=config)
        guard = ipc_mod.SharedFrontGuard(ipc, config=config)

        drive = DriveController(motor, encoders, config,
                                guard=guard, gyro=gyro)
        applier = ControlApplier(drive, motor, servo, ipc, config)

        def telemetry_sink(snapshot: dict) -> None:
            # The envelope, not the snapshot, carries what step() has no business
            # knowing about: the servo angles and the live encoder signs. Keeping
            # them out of step() is what lets it stay a pure function.
            ipc_mod.put_drop_oldest(ipc.telemetry_q, {
                "drive": snapshot,
                "servo": dict(servo.angles) if servo is not None else {},
                "signs": dict(config.sides.signs),
                # Read fresh each tick, not captured once: _h_set_gains swaps
                # these dataclasses out at runtime, and the UI has to see the
                # value the loop is actually using.
                "gains": {"position": gains_as_dict(config.position),
                          "heading": gains_as_dict(config.heading)},
                "ts": time.monotonic(),
            })

        # run_loop waits on a threading.Event (it has to: it also serves the
        # single-process path). Bridge the cross-process stop into it, so the
        # loop still finishes the tick it is in -- including its motor write --
        # instead of being cut off mid-step.
        import threading
        threading.Thread(target=_stop_bridge, args=(ipc, drive), daemon=True,
                         name="StopBridge").start()

        ipc.log("control", f"control loop starting at "
                           f"{config.control.loop_hz:g} Hz")
        drive.run_loop(command_pump=applier.pump,
                       telemetry_sink=telemetry_sink,
                       heartbeat=ipc.beat)

    except Exception as exc:                                    # noqa: BLE001
        import traceback
        ipc.log("control", f"FATAL: {exc}")
        traceback.print_exc(file=sys.stderr)
    finally:
        # Belt and braces: the watchdog also stops the motors if this process
        # dies without getting here at all (SIGKILL, segfault in a C extension).
        try:
            if motor is not None:
                motor.stop_rotate()
                motor.setMotorModel(0, 0, 0, 0)
        except Exception:                                       # noqa: BLE001
            pass
        ipc.log("control", "stopped")


def _stop_bridge(ipc: ipc_mod.IPC, drive) -> None:
    """Bridge the global (cross-process) stop event into the controller's own
    thread-level stop flag."""
    ipc.stop_evt.wait()
    drive.stop_loop()
