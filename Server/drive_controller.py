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
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import math

from config import CONFIG, RobotConfig
from kinematics import SkidSteerKinematics, Twist, WheelSpeeds
from odometry import Pose, SkidSteerOdometry, wrap_angle
from pid import PID


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class DriveController:
    def __init__(self, motor, encoders, config: RobotConfig = CONFIG,
                 clock: Callable[[], float] = time.monotonic,
                 plant: "Optional[SimulatedDrivePlant]" = None):
        self.motor = motor
        self.encoders = encoders
        self.config = config
        self._clock = clock
        self.plant = plant

        self.kin = SkidSteerKinematics(config.wheel)
        self.odom = SkidSteerOdometry(config.wheel)

        gains = config.pid
        self.pid_left = PID(gains.kp, gains.ki, gains.kd, gains.feedforward,
                            gains.output_limit, gains.integral_limit)
        self.pid_right = PID(gains.kp, gains.ki, gains.kd, gains.feedforward,
                             gains.output_limit, gains.integral_limit)

        self._target = Twist()
        self._target_time = clock()
        self._engaged = False
        self._goal = None          # active distance/turn goal, if any
        self._lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

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
        """Command a body velocity (m/s, rad/s). Cancels any active goal and
        engages closed-loop control."""
        with self._lock:
            self._goal = None
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
            self._goal = None
            self._target = Twist()
        self.pid_left.reset()
        self.pid_right.reset()

    # -- high-level motion goals (closed-loop on odometry) -----------------
    def drive_distance(self, distance: float, speed: float = 0.2) -> None:
        """Drive straight for ``distance`` metres (negative = reverse), holding
        heading, then stop. Progress is measured by the encoders/odometry."""
        with self._lock:
            self._goal = _StraightGoal(distance, speed)
            self._engaged = True
            self._target_time = self._clock()

    def turn_in_place(self, angle_deg: float, ang_speed: float = 1.0) -> None:
        """Rotate in place by ``angle_deg`` (positive = ccw), then stop."""
        with self._lock:
            self._goal = _TurnGoal(math.radians(angle_deg), ang_speed)
            self._engaged = True
            self._target_time = self._clock()

    def goal_active(self) -> bool:
        with self._lock:
            return self._goal is not None

    def _current_target(self) -> tuple[Twist, bool]:
        with self._lock:
            engaged = self._engaged
            target = self._target
            stale = (self._clock() - self._target_time) > \
                self.config.control.command_timeout
        if engaged and stale:
            return Twist(), True   # safety stop, still engaged
        return target, engaged

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

        # 1b. Service an active motion goal: it drives the target twist from
        #     odometry progress and finishes on its own.
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.config.wheel.track
        with self._lock:
            goal = self._goal
        if goal is not None:
            twist, done = goal.update(d_center, d_theta, self.odom.pose)
            if done:
                with self._lock:
                    if self._goal is goal:
                        self._goal = None
                goal = None                       # reflect completion in telemetry
                self._apply_target(0.0, 0.0)
            else:
                self._apply_target(twist.linear, twist.angular)

        # 2. Target wheel speeds from the commanded twist.
        target, engaged = self._current_target()
        wheel_target = self.kin.inverse(target)

        # 3. PID -> duty (only when engaged; otherwise leave motors alone).
        if engaged:
            if target.linear == 0.0 and target.angular == 0.0:
                # Hard stop: clear integrators so we don't creep.
                self.pid_left.reset()
                self.pid_right.reset()
                duty_left = duty_right = 0.0
            else:
                duty_left = self.pid_left.update(wheel_target.left,
                                                 measured.left, dt)
                duty_right = self.pid_right.update(wheel_target.right,
                                                   measured.right, dt)
            self.motor.setMotorModel(int(round(duty_left)), int(round(duty_left)),
                                     int(round(duty_right)), int(round(duty_right)))
        else:
            duty_left = duty_right = 0.0

        # 4. Advance the simulated plant (no-op on real hardware).
        if self.plant is not None:
            self.plant.step(duty_left, duty_right, dt)

        # 5. Publish telemetry.
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
            "goal_active": goal is not None,
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
            except Exception as exc:  # never let the loop die silently
                print(f"[DriveController] step error: {exc}")
            # Maintain the loop rate.
            sleep = period - (self._clock() - now)
            if sleep > 0:
                self._stop_evt.wait(sleep)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.encoders.begin()
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="DriveController")
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)
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
            "goal_active": False,
        }


def _twist_from_wheels(kin: SkidSteerKinematics, left: float,
                       right: float) -> tuple[float, float]:
    t = kin.forward(WheelSpeeds(left=left, right=right))
    return t.linear, t.angular


class _StraightGoal:
    """Drive a fixed distance in a straight line, closed-loop on odometry.

    Progress is the accumulated path length (from encoder deltas). Speed tapers
    down near the end to limit overshoot, and a proportional heading term holds
    the initial heading so the path stays straight.
    """

    def __init__(self, distance: float, speed: float,
                 heading_gain: float = 3.0, decel_dist: float = 0.15,
                 tolerance: float = 0.005, min_speed: float = 0.03):
        self.remaining = abs(distance)
        self.direction = 1.0 if distance >= 0 else -1.0
        self.speed = abs(speed)
        self.heading_gain = heading_gain
        self.decel_dist = max(1e-3, decel_dist)
        self.tolerance = tolerance
        self.min_speed = min_speed
        self.theta0 = None

    def update(self, d_center, d_theta, pose):
        if self.theta0 is None:
            self.theta0 = pose.theta
        self.remaining -= abs(d_center)
        if self.remaining <= self.tolerance:
            return Twist(0.0, 0.0), True
        v = self.speed
        if self.remaining < self.decel_dist:
            v = max(self.min_speed, self.speed * (self.remaining / self.decel_dist))
        # Heading hold: steer back toward the starting heading.
        heading_err = wrap_angle(pose.theta - self.theta0)
        w = -self.heading_gain * heading_err
        return Twist(self.direction * v, w), False


class _TurnGoal:
    """Rotate in place by a fixed angle, closed-loop on odometry heading."""

    def __init__(self, angle_rad: float, ang_speed: float,
                 decel_angle: float = 0.35, tolerance: float = 0.01,
                 min_speed: float = 0.2):
        self.remaining = abs(angle_rad)
        self.direction = 1.0 if angle_rad >= 0 else -1.0
        self.speed = abs(ang_speed)
        self.decel_angle = max(1e-3, decel_angle)
        self.tolerance = tolerance
        self.min_speed = min_speed

    def update(self, d_center, d_theta, pose):
        self.remaining -= abs(d_theta)
        if self.remaining <= self.tolerance:
            return Twist(0.0, 0.0), True
        w = self.speed
        if self.remaining < self.decel_angle:
            w = max(self.min_speed, self.speed * (self.remaining / self.decel_angle))
        return Twist(0.0, self.direction * w), False


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
        for tag in tags:
            enc = self.encoders.encoders[tag]
            # read_reset_sides multiplies by sign, so pre-divide to land on the
            # intended post-sign value.
            enc.add(int(round(counts * signs.get(tag, 1))))
