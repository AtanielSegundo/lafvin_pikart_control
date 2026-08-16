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
from odometry import Pose, SkidSteerOdometry, wrap_angle
from pid import PID

from collections import deque

if TYPE_CHECKING:
    # Import for type hints only: Ultrasonic pulls in RPi.GPIO and GyroMPU pulls
    # in mpu6050, both absent off-Pi. `from __future__ import annotations` keeps
    # the annotations lazy, so the controller still imports (unit-tests run) on
    # a laptop/CI.
    from Ultrasonic import Ultrasonic
    from heading import GyroMPU

def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _shape_move_duty(duty: float, remaining: float, speed: float, gains) -> float:
    """Shape one side's position-PID duty for a smooth, low-overshoot stop.

    Two effects, sign preserved:

    1. Deceleration ceiling: cap |duty| at ``decel_gain * sqrt(|remaining|)`` so
       the approach follows v ~ sqrt(2*a*d) -- it cruises at the PID/output_limit
       far out, then slows as it nears the target. This is what removes the
       momentum overshoot AND keeps the final approach slow enough that the
       encoders don't drop counts.

    2. Anti-stall floor: keep |duty| >= ``min_move_duty`` ONLY when the wheel has
       nearly stopped (``|speed| < stop_speed``) while still short of the target.
       Decoupling the floor from the fast approach means it no longer *forces*
       speed into the target (the old cause of overshoot) -- it only unsticks a
       side that stalled a few mm short.
    """
    if duty == 0.0:
        return 0.0
    mag = abs(duty)
    if gains.decel_gain > 0.0:
        mag = min(mag, gains.decel_gain * math.sqrt(max(0.0, abs(remaining))))
    if (gains.min_move_duty > 0.0 and abs(remaining) > gains.tolerance
            and abs(speed) < gains.stop_speed):
        mag = max(mag, gains.min_move_duty)
    return math.copysign(mag, duty)


def _lerp(a: float, b: float, frac: float) -> float:
    return a + (b - a) * frac


def _turn_profile_steps(turn_fn: str, pwm: int, min_pwm: int,
                        params: dict) -> "list[tuple[int, float]]":
    """Expand an open-loop turn profile into a list of (duty, dt) steps, run
    SERVER-SIDE (no per-step network latency). Mirrors calibration.py.

    turn_fn:
      "std"       -- constant pwm for dt, sub-stepped (params: dt, steps)
      "decayed"   -- linear ramp-down pwm->min (params: dt, N)
      "trapezoid" -- ramp up / cruise / ramp down (params: dt, steps, ramp_frac)
      "pulsed"    -- pwm bursts with brake gaps (params: on_s, off_s, n_pulses)
    duty is the UNSIGNED magnitude; a 0 duty means brake (pulsed gaps). The
    caller applies the ccw sign.
    """
    p = params or {}
    if turn_fn == "std":
        dt = float(p.get("dt", 0.5)); steps = max(1, int(p.get("steps", 20)))
        return [(int(pwm), dt / steps)] * steps
    if turn_fn == "decayed":
        dt = float(p.get("dt", 0.5)); n = max(1, int(p.get("N", 10)))
        return [(max(int(min_pwm), int(pwm * (1.0 - i / n))), dt / n)
                for i in range(n)]
    if turn_fn == "trapezoid":
        dt = float(p.get("dt", 0.6)); steps = max(1, int(p.get("steps", 15)))
        ramp_frac = max(0.0, min(0.5, float(p.get("ramp_frac", 0.3))))
        step = dt / steps
        t_up, t_dn = ramp_frac * dt, (1.0 - ramp_frac) * dt
        seq = []
        for i in range(steps):
            t = i * step
            if t_up > 0.0 and t < t_up:
                duty = _lerp(min_pwm, pwm, t / t_up)
            elif (dt - t_dn) > 0.0 and t > t_dn:
                duty = _lerp(pwm, min_pwm, (t - t_dn) / (dt - t_dn))
            else:
                duty = pwm
            seq.append((int(duty), step))
        return seq
    if turn_fn == "pulsed":
        on_s = float(p.get("on_s", 0.08)); off_s = float(p.get("off_s", 0.06))
        n = max(1, int(p.get("n_pulses", 8)))
        seq = []
        for _ in range(n):
            seq.append((int(pwm), on_s))
            seq.append((0, off_s))            # 0 = brake, wheels regain grip
        return seq
    raise ValueError(f"unknown turn_fn {turn_fn!r}")


class DriveController:
    def __init__(self, motor, encoders, config: RobotConfig = CONFIG,
                 clock: Callable[[], float] = time.monotonic,
                 dist_sensor:Ultrasonic = None,
                 gyro: "Optional[GyroMPU]" = None,
                 plant: "Optional[SimulatedDrivePlant]" = None):
        self.motor       = motor
        self.encoders    = encoders
        self.config      = config
        self._clock      = clock
        self.plant       = plant
        self.dist_sensor = dist_sensor
        self.gyro        = gyro          # MPU6050 heading source (may be None)

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

        # Heading PID (gyro turn-in-place): error in RADIANS of yaw -> duty.
        hg = config.heading
        self.heading_pid = PID(hg.kp, hg.ki, hg.kd, 0.0,
                               hg.output_limit, hg.integral_limit)

        self._target      = Twist()
        self._target_time = clock()
        self._engaged     = False
        self._move        = None              # active position move, if any
        # Gyro heading turn: absolute target yaw (rad) or None when inactive.
        self._heading_target = None
        self._heading_time   = clock()
        self._heading_prev_yaw = None         # for the measured yaw rate
        self._heading_settle   = 0            # consecutive in-window slow ticks
        self._heading_pulse    = 0.0          # delta-sigma accumulator (floor)
        self._gyro_yaw       = 0.0            # latest gyro yaw (rad), for telem
        self._gyro_yaw_prev  = None           # for per-step d_theta
        self._goto        = None              # active go-to-pose job, if any
        # Open-loop raw-turn: forced odometry heading (rad) while a scheduled
        # PWM turn runs, or None. Lets telemetry reflect the turn with no gyro.
        self._heading_override = None
        self._raw_turn_thread: Optional[threading.Thread] = None
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
            self._heading_target = None       # teleop overrides a heading turn
            self._goto = None                 # ...and cancels a go-to-pose job
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
            self._heading_target = None
            self._goto = None
            self._target = Twist()
        self.pid_left.reset()
        self.pid_right.reset()
        self.pos_left.reset()
        self.pos_right.reset()
        self.heading_pid.reset()

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

        Preferred path: close the loop on the MPU6050 gyro yaw (ground truth,
        slip-immune) with the heading PID. If no gyro is connected, fall back to
        the encoder-distance turn: a ccw turn of angle a needs each side to
        travel s = a * track / 2 in opposite directions.
        """
        yaw = self._read_gyro_yaw()
        if yaw is not None:
            self._start_heading(yaw + math.radians(angle_deg))
        else:
            s = math.radians(angle_deg) * self.config.wheel.track / 2.0
            self._start_move(-s, s)

    def _start_move(self, target_left: float, target_right: float) -> None:
        self.pos_left.reset()
        self.pos_right.reset()
        with self._lock:
            self._move = _PositionMove(target_left, target_right,
                                       self.config.position)
            self._heading_target = None
            self._engaged = True

    def _read_gyro_yaw(self):
        """Latest gyro yaw in radians (mounting sign/axis applied by GyroMPU),
        or None if no gyro / disconnected."""
        if self.gyro is None or not self.gyro.is_connected():
            return None
        try:
            if hasattr(self.gyro, "get_yaw"):
                return math.radians(self.gyro.get_yaw())
            return math.radians(self.gyro.get_angles_gyro()["z"])
        except Exception:                                       # noqa: BLE001
            return None

    def _start_heading(self, target_yaw: float) -> None:
        """Engage a gyro-closed turn to an absolute yaw (radians)."""
        self.heading_pid.reset()              # no integral/derivative carry-over
        self._heading_prev_yaw = None
        self._heading_settle   = 0
        self._heading_pulse    = 0.0
        with self._lock:
            self._move = None
            self._heading_target = target_yaw
            self._heading_time = self._clock()
            self._engaged = True

    def _shape_turn_duty(self, duty: float, error: float) -> float:
        """Shape the heading PID's duty for a stiction-limited skid turn.

        1. Deceleration ceiling (mirrors :func:`_shape_move_duty`): cap |duty| at
           ``decel_gain * sqrt(|error|)`` so the approach follows w ~ sqrt(2*a*th)
           instead of charging in at the output limit.

        2. Stiction floor, PULSED. Four wheels scrubbing sideways need about
           ``min_turn_duty`` before they move at all, so a smaller PID demand
           just hums. Raising it to the floor is what made the old turn
           bang-bang, so instead fire the FULL floor duty on a fraction of ticks
           equal to demand/floor (delta-sigma accumulator) and brake in between.
           Average torque tracks the PID; each burst still breaks static
           friction. ``pulse_floor = False`` reverts to a continuous floor.
        """
        if duty == 0.0:
            return 0.0
        hg = self.config.heading
        mag = abs(duty)
        if hg.decel_gain > 0.0:
            mag = min(mag, hg.decel_gain * math.sqrt(max(0.0, abs(error))))
        floor = hg.min_turn_duty
        if floor > 0.0 and mag < floor:
            if not hg.pulse_floor:
                mag = floor
            else:
                self._heading_pulse += mag / floor
                if self._heading_pulse >= 1.0:
                    self._heading_pulse -= 1.0
                    mag = floor
                else:
                    mag = 0.0                 # brake gap: wheels regain grip
        return math.copysign(mag, duty)

    def _heading_duty(self, target_yaw: float, yaw: float, elapsed: float,
                      dt: float) -> "tuple[float, float, bool]":
        """One gyro-turn iteration -> (duty_left, duty_right, done).

        PID on the heading error (radians) -> per-side duty, shaped by
        :meth:`_shape_turn_duty`. `error` is NOT wrapped, so a commanded 540 deg
        turns 540 deg rather than 180.

        Arrival needs BOTH |error| < tolerance AND |yaw rate| < settle_rate, held
        for ``settle_ticks`` ticks, braking meanwhile. The rate half is what the
        old bang-bang lacked: at 20 Hz a full-power turn sweeps far more than the
        tolerance window per tick, so "inside the window" alone meant "about to
        coast straight past it".
        """
        hg = self.config.heading
        error = target_yaw - yaw

        # Measured yaw rate from consecutive samples (target is constant during a
        # turn, so this is also -d(error)/dt, i.e. what the PID's D term sees).
        prev_yaw = self._heading_prev_yaw
        self._heading_prev_yaw = yaw
        rate = 0.0 if (prev_yaw is None or dt <= 0.0) else (yaw - prev_yaw) / dt

        if elapsed >= hg.max_time:
            return 0.0, 0.0, True

        if abs(error) < hg.tolerance:
            # In the window: brake and wait for the spin to actually die down.
            if abs(rate) < hg.settle_rate:
                self._heading_settle += 1
            else:
                self._heading_settle = 0
            return 0.0, 0.0, self._heading_settle >= hg.settle_ticks
        self._heading_settle = 0

        duty = self._shape_turn_duty(
            self.heading_pid.update(target_yaw, yaw, dt), error)
        # +ccw (duty > 0): left side backward, right side forward.
        return -duty, duty, False

    # -- go-to-pose (turn -> drive -> turn, on the gyro-accurate odometry) --
    def goto_pose(self, x: float, y: float, theta_deg: float = None) -> None:
        """Drive to world pose (x, y[, theta]). Executed as face-the-point ->
        drive-straight -> turn-to-final-heading, each phase recomputed from the
        live odometry pose. `theta_deg` (degrees) is optional; None leaves the
        heading wherever the drive ends."""
        with self._lock:
            self._move = None
            self._heading_target = None
            self._goto = _Goto(x, y, theta_deg)
            self._engaged = True

    def _turn_to(self, rel_turn: float) -> None:
        """Start a RELATIVE in-place turn of `rel_turn` rad -- gyro-closed when
        available, else an encoder turn. Relative, so it is frame-independent."""
        yaw = self._read_gyro_yaw()
        if yaw is not None:
            self._start_heading(yaw + rel_turn)
        else:
            s = rel_turn * self.config.wheel.track / 2.0
            self._start_move(-s, s)

    _GOTO_NEXT = {"start": "turn1", "turn1": "drive", "drive": "turn2",
                  "turn2": "done"}

    def _advance_goto(self) -> None:
        """Advance the go-to-pose sequence to its next phase and start that
        phase's move/turn (call only when no move or heading turn is running).
        `phase` names the action now running, for telemetry."""
        g = self._goto
        if g is None:
            return
        pose = self.odom.pose
        tol = self.config.position.tolerance
        g.phase = self._GOTO_NEXT[g.phase]
        if g.phase == "turn1":                    # face the target point
            dx, dy = g.x - pose.x, g.y - pose.y
            if math.hypot(dx, dy) < tol:
                return self._advance_goto()       # already there -> skip
            self._turn_to(wrap_angle(math.atan2(dy, dx) - pose.theta))
        elif g.phase == "drive":                  # drive straight to it
            dist = math.hypot(g.x - pose.x, g.y - pose.y)
            if dist < tol:
                return self._advance_goto()
            self._start_move(dist, dist)
        elif g.phase == "turn2":                  # settle on the final heading
            if g.theta is None:
                return self._advance_goto()
            self._turn_to(wrap_angle(math.radians(g.theta) - pose.theta))
        else:                                     # "done"
            self._goto = None

    def move_active(self) -> bool:
        with self._lock:
            return (self._move is not None or self._heading_target is not None
                    or self._goto is not None)

    def heading_active(self) -> bool:
        with self._lock:
            return self._heading_target is not None

    # Backwards-compatible alias.
    goal_active = move_active

    # -- open-loop raw turn (no gyro): run a PWM profile server-side and force --
    #    the odometry heading so telemetry reflects it. --------------------------
    def _set_raw_turn(self, duty: int, ccw: bool) -> None:
        """Drive an in-place spin at `duty` (0 = brake). ccw -> left back / right
        forward, matching calibration.set_all_turn and the +ccw convention."""
        d = int(duty)
        if ccw:
            self.motor.setMotorModel(-d, -d, d, d)
        else:
            self.motor.setMotorModel(d, d, -d, -d)

    def raw_turn_schedule(self, turn_fn: str, ccw: bool, pwm: int, min_pwm: int,
                          final_turn_angle: float, fn_params: dict = None) -> bool:
        """Run an open-loop PWM turn profile SERVER-SIDE (no per-step network
        latency) and, since there's no gyro, OVERWRITE the odometry heading each
        tick -- interpolating from the current heading to
        (current + `final_turn_angle`) over the profile's duration -- purely so
        telemetry / the sim mirror reflect the turn. `final_turn_angle` is a
        RELATIVE delta in degrees (the raw turn is a delta by nature), so the
        heading ends at start + final_turn_angle; its sign sets the sweep
        direction.

        turn_fn: "std" | "decayed" | "trapezoid" | "pulsed" (see
        _turn_profile_steps for fn_params). Returns False if a raw turn is
        already running or turn_fn is unknown."""
        if self._raw_turn_thread and self._raw_turn_thread.is_alive():
            return False
        try:
            steps = _turn_profile_steps(turn_fn, int(pwm), int(min_pwm),
                                        fn_params or {})
        except ValueError as e:                                  # noqa: BLE001
            print(f"[raw_turn] {e}")
            return False
        self._raw_turn_thread = threading.Thread(
            target=self._run_raw_turn, args=(steps, bool(ccw),
                                             float(final_turn_angle)),
            daemon=True, name="RawTurn")
        self._raw_turn_thread.start()
        return True

    def _run_raw_turn(self, steps, ccw: bool, final_deg: float) -> None:
        self.release()                             # stop the PID fighting the duty
        total = sum(dt for _, dt in steps) or 1e-9
        start = self.odom.pose.theta               # rad, at turn start
        delta = math.radians(final_deg)            # RELATIVE turn (delta), added
        final = start + delta                      # to the heading at turn start
        with self._lock:
            self._heading_override = wrap_angle(start)
        elapsed = 0.0
        try:
            for duty, dt in steps:
                self._set_raw_turn(duty, ccw)
                elapsed += dt
                frac = min(1.0, elapsed / total)
                with self._lock:                   # sweep the faked heading
                    self._heading_override = wrap_angle(start + delta * frac)
                time.sleep(dt)
        finally:
            self.motor.setMotorModel(0, 0, 0, 0)   # brake/stop
            with self._lock:
                self.odom.pose.theta = wrap_angle(final)   # land on the target
                self._heading_override = None              # resume normal odom

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

        # 1. Feedback: encoder deltas (translation) + gyro yaw (rotation).
        d_counts_left, d_counts_right = self.encoders.read_reset_sides()

        mpc = self.config.wheel.meters_per_count
        d_left = d_counts_left * mpc
        d_right = d_counts_right * mpc

        # Gyro yaw -> per-step d_theta (rad). Ground truth when connected;
        # odometry falls back to the encoder differential when it is None.
        gyro_yaw = self._read_gyro_yaw()
        gyro_dtheta = None
        if gyro_yaw is not None:
            self._gyro_yaw = gyro_yaw
            if self._gyro_yaw_prev is not None:
                gyro_dtheta = gyro_yaw - self._gyro_yaw_prev
            self._gyro_yaw_prev = gyro_yaw
        else:
            self._gyro_yaw_prev = None       # reset so a reconnect doesn't jump

        self.odom.update_from_distances(d_left, d_right, dt, d_theta=gyro_dtheta)
        # Raw-turn heading override: a scheduled open-loop turn forces the odom
        # heading (there's no gyro) so telemetry reflects the turn. Applied here
        # -- the loop is the single writer of pose.theta.
        with self._lock:
            ov = self._heading_override
        if ov is not None:
            self.odom.pose.theta = ov
        measured = WheelSpeeds(left=d_left / dt, right=d_right / dt)

        # Advance a go-to-pose job when the current phase's move/turn finished;
        # this starts the next move/heading, picked up by the dispatch below.
        if self._goto is not None:
            with self._lock:
                busy = self._move is not None or self._heading_target is not None
            if not busy:
                self._advance_goto()

        # 2. Choose control mode: a position MOVE takes priority; otherwise a
        #    gyro heading turn; otherwise velocity control (teleop / `drive`).
        with self._lock:
            move = self._move
            heading_running = self._heading_target is not None

        target = Twist()
        wheel_target = WheelSpeeds()
        move_info = None
        heading_info = None
        guard_blocked = False

        if move is not None:
            move.accumulate(d_left, d_right, dt)
        
            duty_left = self.pos_left.update(move.target_left,
                                             move.traveled_left, dt)
            duty_right = self.pos_right.update(move.target_right,
                                               move.traveled_right, dt)
            err_l, err_r = move.errors()
            # Decelerate into the target (low overshoot, encoder-safe) and floor
            # only a side that has stalled short.
            duty_left = _shape_move_duty(duty_left, err_l, measured.left,
                                         self.config.position)
            duty_right = _shape_move_duty(duty_right, err_r, measured.right,
                                          self.config.position)
        
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
            # PIDs (no wind-up).
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
            
        elif heading_running:
            # --- gyro turn: per-side DUTY directly (uniform power + decel + ---
            #     stiction floor), so it doesn't starve the motors near target.
            engaged = True
            with self._lock:
                target_yaw = self._heading_target
                elapsed = self._clock() - self._heading_time
            if gyro_yaw is None or target_yaw is None:
                # Gyro dropped mid-turn (or the turn was cancelled): stop safely.
                with self._lock:
                    self._heading_target = None
                self.heading_pid.reset()
                duty_left = duty_right = 0.0
            else:
                duty_left, duty_right, done = self._heading_duty(
                    target_yaw, gyro_yaw, elapsed, dt)
                heading_info = {
                    "target_deg": round(math.degrees(target_yaw), 2),
                    "error_deg": round(math.degrees(target_yaw - gyro_yaw), 2),
                    "rate_dps": round(math.degrees(gyro_dtheta / dt), 1)
                                if gyro_dtheta is not None else None,
                    "duty": int(round(duty_right))}
                if done:
                    with self._lock:
                        if self._heading_target == target_yaw:
                            self._heading_target = None
                    duty_left = duty_right = 0.0
                    heading_info = None
            
            if self._front_blocked(duty_left, duty_right):
                duty_left = duty_right = 0.0
                guard_blocked = True
            self.motor.setMotorModel(int(round(duty_left)), int(round(duty_left)),
                                     int(round(duty_right)), int(round(duty_right)))

        else:
            # --- VELOCITY control (teleop / `drive`) ---
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
        with self._lock:
            heading_now = self._heading_target is not None
            g = self._goto
        goto_info = (None if g is None else
                     {"phase": g.phase, "x": round(g.x, 3), "y": round(g.y, 3),
                      "theta": g.theta})
        gyro_connected = self.gyro is not None and self.gyro.is_connected()
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
            "goal_active": (move is not None) or heading_now or (g is not None),
            "move": move_info,
            "goto": goto_info,
            "gyro": {"connected": gyro_connected,
                     "yaw_deg": round(math.degrees(self._gyro_yaw), 2),
                     "source": "gyro" if gyro_dtheta is not None else "encoder",
                     "heading": heading_info},
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
            "goto": None,
            "gyro": {"connected": False, "yaw_deg": 0.0,
                     "source": "encoder", "heading": None},
            "encoders": {},
        }


def _twist_from_wheels(kin: SkidSteerKinematics, left: float,
                       right: float) -> tuple[float, float]:
    t = kin.forward(WheelSpeeds(left=left, right=right))
    return t.linear, t.angular


class _Goto:
    """A go-to-pose job run as turn -> drive -> turn. Each phase is recomputed
    from the live odometry pose, so heading/translation error does not build up
    across phases. `theta` (degrees) is the optional final heading."""

    def __init__(self, x: float, y: float, theta: float = None):
        self.x = x
        self.y = y
        self.theta = theta
        # start -> turn1 -> drive -> turn2 -> done (then _goto is cleared).
        self.phase = "start"


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
