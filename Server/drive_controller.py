#!/usr/bin/python3
"""
Closed-loop skid-steer drive controller.

Pipeline, run every control tick:

    encoders --(deltas)--> odometry (pose) + measured wheel speeds
    target twist --inverse kinematics--> target wheel speeds
    (target, measured) --PID (+feedforward)--> per-side PWM duty
    duty --> Motor.setMotorModel(left, left, right, right)

The controller can run its own thread (:meth:`start`) or be stepped manually
(:meth:`step`) which makes it fully unit-testable. It never instantiates
hardware itself — the motor and encoders are injected — so the same object
works on the Pi and against a simulated plant on a laptop.

Two engagement states:
  * released (default): odometry keeps integrating, but the loop does NOT drive
    the motors, leaving raw ``CMD_MOTOR`` duty commands in control.
  * engaged: a velocity command took over; PID actively drives the motors.

When a distance sensor is injected, a front collision guard runs in its own
thread (:meth:`_dist_guard`): it holds ``dist_guard_lock`` whenever an obstacle
is both closer than ``minimum_front_distance_cm`` and closing, and ``step``
then vetoes net-forward motor drive (reverse / turn-in-place stay allowed).
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional, TYPE_CHECKING

import math

from config import CONFIG, RobotConfig
from kinematics import SkidSteerKinematics, Twist, WheelSpeeds
from odometry import Pose, SkidSteerOdometry
from pid import PID

from collections import deque

if TYPE_CHECKING:
    # Import for type hints only: Ultrasonic pulls in RPi.GPIO, which is absent
    # off-Pi. `from __future__ import annotations` keeps the annotation lazy, so
    # the controller still imports (and unit-tests run) on a laptop/CI.
    from Ultrasonic import Ultrasonic

def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _stiction_floor(duty: float, remaining: float, gains) -> float:
    """Keep a not-yet-arrived position command above the motor's move threshold.

    The position PID output is ~kp*error, so it shrinks as the wheel nears its
    target and eventually falls below the PWM the motor needs to move at all --
    the wheel then stalls a few mm short and hums until the safety timeout.
    While the side is still outside `tolerance`, floor the magnitude at
    `min_move_duty` (sign preserved, so reverse moves get -min_move_duty).
    """
    if gains.min_move_duty <= 0 or duty == 0.0:
        return duty
    if abs(remaining) <= gains.tolerance:
        return duty
    return math.copysign(max(abs(duty), gains.min_move_duty), duty)


class DriveController:
    def __init__(self, motor, encoders, config: RobotConfig = CONFIG,
                 clock: Callable[[], float] = time.monotonic,
                 dist_sensor:Ultrasonic = None,
                 plant: "Optional[SimulatedDrivePlant]" = None):
        self.motor       = motor
        self.encoders    = encoders
        self.config      = config
        self._clock      = clock
        self.plant       = plant
        self.dist_sensor = dist_sensor

        self.kin  = SkidSteerKinematics(config.wheel)
        self.odom = SkidSteerOdometry(config.wheel)

        # Velocity PIDs (teleop / `drive`): error in m/s -> duty.
        gains = config.pid
        self.pid_left = PID(gains.kp, gains.ki, gains.kd, gains.feedforward,
                            gains.output_limit, gains.integral_limit)
        self.pid_right = PID(gains.kp, gains.ki, gains.kd, gains.feedforward,
                             gains.output_limit, gains.integral_limit)

        # Position PIDs (drive_distance / turn): error in METRES -> duty.
        pg = config.position
        self.pos_left = PID(pg.kp, pg.ki, pg.kd, 0.0,
                            pg.output_limit, pg.integral_limit)
        self.pos_right = PID(pg.kp, pg.ki, pg.kd, 0.0,
                             pg.output_limit, pg.integral_limit)

        self._target      = Twist()
        self._target_time = clock()
        self._engaged     = False
        self._move        = None              # active position move, if any
        self._lock        = threading.Lock()

        self._thread      : Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        
        self.dist_arr = deque(maxlen=3)     # recent VALID readings (min = closest)
        self.front_distance_cm = None       # latest valid front distance, for UI
        self._dist_thread: Optional[threading.Thread] = None
        self._dist_stop_evt = threading.Event()
        self.dist_guard_lock = threading.Lock()

        self._telemetry = self._blank_telemetry()

    # -- command API -------------------------------------------------------
    def _apply_target(self, linear: float, angular: float) -> None:
        """Set the engaged target twist (clamped). Does NOT touch any goal."""
        c = self.config.control
        with self._lock:
            self._target = Twist(_clamp(linear, c.max_linear),
                                 _clamp(angular, c.max_angular))
            self._target_time = self._clock()
            self._engaged = True

    def set_twist(self, linear: float, angular: float) -> None:
        """Command a body velocity (m/s, rad/s). Cancels any active move and
        engages velocity control. While the front guard is engaged, forward
        velocity is refused up front (reverse / turning stay allowed)."""
        if linear > 0.0 and self._front_guard_engaged():
            linear = 0.0
        with self._lock:
            self._move = None
        self._apply_target(linear, angular)

    def set_wheel_speeds(self, left: float, right: float) -> None:
        """Command per-side wheel speeds (m/s). Engages closed-loop control."""
        self.set_twist(*_twist_from_wheels(self.kin, left, right))

    def stop(self) -> None:
        """Command zero velocity but stay engaged (active braking to 0)."""
        self.set_twist(0.0, 0.0)

    def release(self) -> None:
        """Hand motor control back to raw duty commands; keep odometry alive."""
        with self._lock:
            self._engaged = False
            self._move = None
            self._target = Twist()
        self.pid_left.reset()
        self.pid_right.reset()
        self.pos_left.reset()
        self.pos_right.reset()

    # -- high-level moves (per-side POSITION control) ----------------------
    def drive_distance(self, distance: float, speed: float = None) -> None:
        """Drive straight ``distance`` metres (negative = reverse) and stop.

        Both sides are given the same distance target, so a per-side position
        PID keeps them equal -> straight line. Closed on the encoders.
        (``speed`` is accepted for API compatibility; the move speed is set by
        the position gains' output_limit.)

        A forward move is refused while the front guard is engaged, so you can't
        start driving into an obstacle that is already within the limit.
        """
        if distance > 0.0 and self._front_guard_engaged():
            return
        self._start_move(distance, distance)

    def turn_in_place(self, angle_deg: float, ang_speed: float = None) -> None:
        """Rotate in place by ``angle_deg`` (positive = ccw) and stop.

        A ccw turn of angle a rotates the body by a = (s_right - s_left)/track
        with s_left = -s, s_right = +s, so each side must travel
        s = a * track / 2 in opposite directions.
        """
        s = math.radians(angle_deg) * self.config.wheel.track / 2.0
        self._start_move(-s, s)

    def _start_move(self, target_left: float, target_right: float) -> None:
        self.pos_left.reset()
        self.pos_right.reset()
        with self._lock:
            self._move = _PositionMove(target_left, target_right,
                                       self.config.position)
            self._engaged = True

    def move_active(self) -> bool:
        with self._lock:
            return self._move is not None

    # Backwards-compatible alias.
    goal_active = move_active

    def _current_target(self) -> tuple[Twist, bool]:
        with self._lock:
            engaged = self._engaged
            target = self._target
            stale = (self._clock() - self._target_time) > \
                self.config.control.command_timeout
        if engaged and stale:
            return Twist(), True   # safety stop, still engaged
        return target, engaged

    def _front_guard_engaged(self) -> bool:
        """The front distance guard is holding: an obstacle is closer than
        `minimum_front_distance_cm` (the `_dist_guard` thread holds
        `dist_guard_lock`). With no distance sensor injected it is never engaged
        (unit tests / sim).
        """
        return self.dist_sensor is not None and self.dist_guard_lock.locked()

    def _front_blocked(self, duty_left: float, duty_right: float) -> bool:
        """Guard engaged AND the command is net-forward (driving INTO the
        obstacle). We veto only net-forward drive (duty sum > 0) so the kart can
        still reverse or turn in place to escape.
        """
        return self._front_guard_engaged() and (duty_left + duty_right) > 0.0

    # -- control loop ------------------------------------------------------
    def step(self, dt: float) -> dict:
        """Run exactly one control iteration and return the telemetry dict."""
        if dt <= 0.0:
            dt = 1.0 / self.config.control.loop_hz

        # 1. Feedback: mean count deltas per side since the last step.
        d_counts_left, d_counts_right = self.encoders.read_reset_sides()
        
        mpc = self.config.wheel.meters_per_count
        
        d_left = d_counts_left * mpc
        d_right = d_counts_right * mpc
        
        self.odom.update_from_distances(d_left, d_right, dt)
        measured = WheelSpeeds(left=d_left / dt, right=d_right / dt)

        # 2. Choose control mode: a position MOVE takes priority; otherwise
        #    fall back to velocity control (teleop / `drive`).
        with self._lock:
            move = self._move

        target = Twist()
        wheel_target = WheelSpeeds()
        move_info = None
        guard_blocked = False

        if move is not None:
            move.accumulate(d_left, d_right, dt)
        
            duty_left = self.pos_left.update(move.target_left,
                                             move.traveled_left, dt)
            duty_right = self.pos_right.update(move.target_right,
                                               move.traveled_right, dt)
            err_l, err_r = move.errors()
            # Deadband/stiction compensation so a nearly-arrived side doesn't
            # stall short below the motor's move threshold.
        
            duty_left = _stiction_floor(duty_left, err_l, self.config.position)
            duty_right = _stiction_floor(duty_right, err_r, self.config.position)
        
            if move.is_done(measured.left, measured.right):
                duty_left = duty_right = 0.0
                self.pos_left.reset()
                self.pos_right.reset()
                with self._lock:
                    if self._move is move:
                        self._move = None
                move = None                       # reflect completion in telemetry
            else:
                move_info = {
                    "target": {"left": round(move.target_left, 4),
                               "right": round(move.target_right, 4)},
                    "traveled": {"left": round(move.traveled_left, 4),
                                 "right": round(move.traveled_right, 4)},
                    "remaining": {"left": round(err_l, 4),
                                  "right": round(err_r, 4)},
                }
            engaged = True

            # Front collision guard: obstacle within the limit and this move
            # drives INTO it. Cancel the move, brake to zero, reset the position
            # PIDs (no wind-up). Do NOT early-return -- we must still command the
            # motors to 0 below and publish telemetry.
            if self._front_blocked(duty_left, duty_right):
                duty_left = duty_right = 0.0
                self.pos_left.reset()
                self.pos_right.reset()
                with self._lock:
                    self._move = None          # cancel the forward move
                    self._target = Twist()     # hold zero on following ticks
                move = None                    # -> goal_active False in telemetry
                move_info = None
                guard_blocked = True

            self.motor.setMotorModel(int(round(duty_left)), int(round(duty_left)),
                                     int(round(duty_right)), int(round(duty_right)))
            
        else:
            # --- VELOCITY control ---
            target, engaged = self._current_target()
            wheel_target = self.kin.inverse(target)
            if engaged:
                if target.linear == 0.0 and target.angular == 0.0:
                    self.pid_left.reset()
                    self.pid_right.reset()
                    duty_left = duty_right = 0.0
                else:
                    duty_left = self.pid_left.update(wheel_target.left,
                                                     measured.left, dt)
                    duty_right = self.pid_right.update(wheel_target.right,
                                                       measured.right, dt)
                # Front collision guard: veto driving into a close, closing
                # obstacle and reset the velocity PIDs to avoid a wind-up lurch.
                if self._front_blocked(duty_left, duty_right):
                    duty_left = duty_right = 0.0
                    self.pid_left.reset()
                    self.pid_right.reset()
                    guard_blocked = True
                self.motor.setMotorModel(
                    int(round(duty_left)), int(round(duty_left)),
                    int(round(duty_right)), int(round(duty_right)))
            else:
                duty_left = duty_right = 0.0

        # 3. Advance the simulated plant (no-op on real hardware).
        if self.plant is not None:
            self.plant.step(duty_left, duty_right, dt)

        # 4. Publish telemetry.
        snapshot = {
            "pose": self.odom.pose.as_dict(),
            "twist": {"linear": round(self.odom.linear_velocity, 4),
                      "angular": round(self.odom.angular_velocity, 4)},
            "target": {"linear": target.linear, "angular": target.angular},
            "wheel_speed": {"left": round(measured.left, 4),
                            "right": round(measured.right, 4)},
            "wheel_target": {"left": round(wheel_target.left, 4),
                             "right": round(wheel_target.right, 4)},
            "duty": {"left": int(round(duty_left)), "right": int(round(duty_right))},
            "engaged": engaged,
            "guard_blocked": guard_blocked,
            "front_distance_cm": self.front_distance_cm,
            "goal_active": move is not None,
            "move": move_info,
            "encoders": self.encoders.raw_totals(),   # raw per-motor counts
        }
        with self._lock:
            self._telemetry = snapshot
        
        return snapshot

    def _run(self) -> None:
        period = 1.0 / self.config.control.loop_hz
        last = self._clock()
        
        while not self._stop_evt.is_set():
            now = self._clock()
            dt = now - last
            last = now
    
            try:
                self.step(dt if dt > 0 else period)
            except Exception as exc:
                print(f"[DriveController] step error: {exc}")
            
            sleep = period - (self._clock() - now)
            
            if sleep > 0:
                self._stop_evt.wait(sleep)

    # This thread guards the physical front of the Kart: it holds
    # dist_guard_lock while an obstacle is closer than the limit, and step()
    # reads that state (via _front_blocked) to veto forward motion.
    def _dist_guard(self) -> None:
        period = 1.0 / (2 * self.config.control.loop_hz)
        limit = self.config.control.minimum_front_distance_cm
        hysteresis = 5          # cm release margin, so jitter doesn't chatter

        while not self._dist_stop_evt.is_set():
            now = self._clock()

            raw = self.dist_sensor.get_distance()
            # Validity is sensor-agnostic: the pigpio reader returns real cm
            # (incl. a large far/clear value), while the RPi.GPIO fallback uses
            # 255 as its "failed read" sentinel -- drop that (and None), keep the
            # rest. No extra smoothing here, so a fresh reading acts immediately.
            if raw is not None and raw > 0 and raw != 255:
                self.dist_arr.append(raw)
                self.front_distance_cm = raw          # latest valid, for the UI

            if self.dist_arr:
                f = min(self.dist_arr)
                if f < limit and not self.dist_guard_lock.locked():
                    self.dist_guard_lock.acquire()
                elif f > limit + hysteresis and self.dist_guard_lock.locked():
                    self.dist_guard_lock.release()

            sleep = period - (self._clock() - now)

            if sleep > 0:
                self._dist_stop_evt.wait(sleep)

    
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.encoders.begin()

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="DriveController")
        self._thread.start()

        # Front collision guard only runs if a distance sensor was injected.
        if self.dist_sensor is not None:
            self._dist_stop_evt.clear()
            self._dist_thread = threading.Thread(target=self._dist_guard, daemon=True, name="DistGuard")
            self._dist_thread.start()

    def shutdown(self) -> None:
        self._stop_evt.set()
        self._dist_stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._dist_thread:
            self._dist_thread.join(timeout=1.0)
        try:
            self.motor.setMotorModel(0, 0, 0, 0)
        except Exception:
            pass
        self.encoders.stop()

    # -- telemetry ---------------------------------------------------------
    def telemetry(self) -> dict:
        with self._lock:
            return dict(self._telemetry)

    def reset_odometry(self, pose: Pose | None = None) -> None:
        self.odom.reset(pose)

    def _blank_telemetry(self) -> dict:
        return {
            "pose": Pose().as_dict(),
            "twist": {"linear": 0.0, "angular": 0.0},
            "target": {"linear": 0.0, "angular": 0.0},
            "wheel_speed": {"left": 0.0, "right": 0.0},
            "wheel_target": {"left": 0.0, "right": 0.0},
            "duty": {"left": 0, "right": 0},
            "engaged": False,
            "guard_blocked": False,
            "front_distance_cm": None,
            "goal_active": False,
            "move": None,
            "encoders": {},
        }


def _twist_from_wheels(kin: SkidSteerKinematics, left: float,
                       right: float) -> tuple[float, float]:
    t = kin.forward(WheelSpeeds(left=left, right=right))
    return t.linear, t.angular


class _PositionMove:
    """A per-side distance target serviced by the position PIDs.

    Straight move: both targets equal (= distance). In-place turn: equal and
    opposite. Travel is accumulated from the encoder deltas each step; the move
    finishes when both sides are within tolerance and have stopped (or on a
    safety timeout).
    """

    def __init__(self, target_left: float, target_right: float, gains):
        self.target_left = target_left
        self.target_right = target_right
        self.traveled_left = 0.0
        self.traveled_right = 0.0
        self.elapsed = 0.0
        self.g = gains

    def accumulate(self, d_left: float, d_right: float, dt: float) -> None:
        self.traveled_left += d_left
        self.traveled_right += d_right
        self.elapsed += dt

    def errors(self) -> tuple[float, float]:
        return (self.target_left - self.traveled_left,
                self.target_right - self.traveled_right)

    def is_done(self, meas_left: float, meas_right: float) -> bool:
        err_l, err_r = self.errors()
        arrived = (abs(err_l) < self.g.tolerance and abs(err_r) < self.g.tolerance
                   and abs(meas_left) < self.g.stop_speed
                   and abs(meas_right) < self.g.stop_speed)
        return arrived or self.elapsed >= self.g.max_time


class SimulatedDrivePlant:
    """First-order motor+wheel model that feeds the simulated encoders.

    Only used off-hardware (tests, demos). ``speed`` relaxes toward
    ``gain * duty`` with time constant ``tau``; the resulting travel is
    converted to encoder counts and injected into each side's encoders so the
    controller sees realistic feedback and the loop actually closes.
    """

    def __init__(self, encoders, config: RobotConfig = CONFIG,
                 gain: float = 1.0 / 2600.0, tau: float = 0.15):
        self.encoders = encoders
        self.config = config
        self.gain = gain          # m/s per duty at steady state
        self.tau = tau            # s
        self._speed_left = 0.0
        self._speed_right = 0.0

    def step(self, duty_left: float, duty_right: float, dt: float) -> None:
        alpha = dt / (self.tau + dt)
        self._speed_left += alpha * (self.gain * duty_left - self._speed_left)
        self._speed_right += alpha * (self.gain * duty_right - self._speed_right)

        mpc = self.config.wheel.meters_per_count
        counts_left = (self._speed_left * dt) / mpc
        counts_right = (self._speed_right * dt) / mpc
        self._inject(self.config.sides.left, counts_left)
        self._inject(self.config.sides.right, counts_right)

    def _inject(self, tags, counts: float) -> None:
        signs = self.config.sides.signs
        modes = getattr(self.encoders, "modes", {})
        for tag in tags:
            enc = self.encoders.encoders[tag]
            mode = modes.get(tag)
            if mode:
                # Single-phase: the encoder stores unsigned magnitude and the
                # read path re-applies direction (from partner) and scale, so
                # pre-divide by scale to land on the intended value.
                scale = mode.get("scale", 1.0)
                enc.add(int(round(abs(counts) / scale)))
            else:
                # read_reset_sides multiplies by sign, so pre-divide to land on
                # the intended post-sign value.
                enc.add(int(round(counts * signs.get(tag, 1))))
