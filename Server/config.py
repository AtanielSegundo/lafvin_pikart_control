#!/usr/bin/python3
"""
Central configuration for the Lafvin PiKart robot.

Everything that depends on the physical build (wheel geometry, encoder wiring,
PID gains, network ports, motor PWM channels) lives here so the rest of the
code stays hardware-agnostic and easy to re-tune without editing logic.

The wheel geometry mirrors the CoppeliaSim reference model used for the
odometry analysis (``TiredWheel``): a skid-steer / differential-drive 4-wheel
platform where the two left wheels and the two right wheels are each driven as
one "virtual" side.
"""
from __future__ import annotations
from typing import *

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

# ---------------------------------------------------------------------------
# Calibration Coefficients ( Extracted from real testing )
# ---------------------------------------------------------------------------
TRACK_COEF   = 1.0
CNT_REV_COEF = 1.0

# ---------------------------------------------------------------------------
# Wheel geometry
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class WheelGeometry:
    diameter: float = 0.065            # m
    colinear_distance: float = 0.095   # m, between motors on the same axle
    track: float = TRACK_COEF*0.151    # m, distance between left and right sides
    # counts_per_rev: int = CNT_REV_COEF*2340
    # Per-motor counts per wheel revolution, in the x4 quadrature base.
    # NOTE for M3 (single-phase): `_resolve` already scales its raw phase-A
    # count x2, so this value must be the POST-scale figure, not the ~1270
    # ticks/rev the lone phase actually produces.
    motor_counts_per_rev: Dict[str, int] = field(default_factory=lambda: {"M1": 2533, "M2": 2535, "M3": 2560, "M4": 2534})

    @property
    def radius(self) -> float:
        return self.diameter / 2.0

    @property
    def circumference(self) -> float:
        return math.pi * self.diameter

    def meters_per_count(self, sides: "SideMapping" = None) -> Tuple[float, float]:
        """(left, right) metres travelled per single encoder count.

        Takes the side mapping rather than reading ``SideMapping``'s class
        defaults: gray_regression's --lean-encoders runs one motor per side
        (``SideMapping(left=("M1",), right=("M4",))``), and averaging the class
        default's pairs there would silently use the wrong motors.

        Averaging each side's counts_per_rev is exact whenever both wheels on a
        side travel together -- mean(c)*circ/mean(cpr) reduces to d for any cpr
        split when d_1 == d_2 -- and degrades to a cpr-weighted average only
        during intra-side slip, which the odometry cannot observe anyway.
        """
        sides = sides if sides is not None else CONFIG.sides
        cpr = self.motor_counts_per_rev
        mean = lambda tags: sum(cpr[t] for t in tags) / len(tags)
        return (self.circumference / mean(sides.left),
                self.circumference / mean(sides.right))


# ---------------------------------------------------------------------------
# PID gains for per-side wheel-VELOCITY control (used for teleop / `drive`).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class PIDGains:
    kp: float = 900.0     # duty per (m/s) of error
    ki: float = 1200.0
    kd: float = 20.0
    # Static feed-forward: duty required to hold 1 m/s (open-loop guess).
    # Helps the loop converge quickly; PID trims the remainder.
    feedforward: float = 2600.0
    output_limit: float = 4095.0      # matches Motor duty range
    integral_limit: float = 4095.0    # anti-windup clamp on the integral term


# ---------------------------------------------------------------------------
# PID gains for per-side POSITION/DISTANCE control (used for drive_distance /
# turn). Error is in METRES of remaining travel; output is PWM duty.
#   - kd damps velocity (d(pos_error)/dt = -wheel_speed), preventing overshoot.
#   - output_limit caps how hard a move pushes, so moves are gentle & safe.
# These MUST be tuned on hardware once the encoders are calibrated.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class PositionGains:
    kp: float = 4000.0      # duty per metre of error
    ki: float = 2000.0       # gentle backstop for residual error; its
                             # contribution is bounded by integral_limit below
    kd: float = 1600.0       # duty per (m/s) — damping
    output_limit  : float = 2200.0  # CRUISE duty cap. Keep within the rate the
                                    # encoders can count -- at high duty (~3000)
                                    # the wheel spins faster than the quadrature
                                    # decoder tracks, counts are dropped, travel
                                    # is under-reported and the move overshoots
                                    # / never "arrives". Raise only if verified.
    integral_limit: float = 800.0
    tolerance     : float = 0.01    # m, arrival tolerance
    stop_speed    : float = 0.02    # m/s below which we consider it stopped
    max_time      : float = 12.0    # s, safety timeout per move
    min_move_duty : float = 1200.0  # anti-stall kick, applied ONLY when a side
                                    # has nearly stopped short (not while moving)
    # Deceleration ceiling: cap |duty| at decel_gain * sqrt(remaining_m) so the
    # approach slows smoothly (v ~ sqrt(2*a*d)) instead of charging in at cruise
    # duty and overshooting. Also keeps the final approach slow enough for the
    # encoders. With output_limit 2000 this starts decelerating ~0.44 m out.
    decel_gain    : float = 3000.0


# ---------------------------------------------------------------------------
# Turn-in-place closed on the MPU6050 gyro yaw. Error is in RADIANS of heading,
# output is per-side PWM duty (left = -duty, right = +duty for a ccw turn).
#
# This is a real PID (Server/pid.py), NOT the old bang-bang profile. The old one
# ran 4095 duty until |err| < 15 deg then 3800 until |err| < 3 deg, i.e. ~full
# power right up to the target. At loop_hz=20 (50 ms/tick) a full-power in-place
# skid turn sweeps ~10-20 deg PER TICK, so the 3 deg arrival window was never
# observable -- every turn stopped on the cross-target sign flip, by definition
# already past the target, and then coasted further on its own inertia. Hence
# "aggressive, overshoots a lot".
#
# The PID fixes the cause: duty now SCALES with the error, and kd (damping)
# subtracts the measured yaw rate so the kart is already slow when it arrives.
#
# Per tick:  PID(err) -> decel ceiling decel_gain*sqrt(|err|) -> stiction floor
# (pulsed, see below) -> arrival test. Only used with a live gyro; otherwise
# turns fall back to the encoder-distance move. TUNE kp/kd ON HARDWARE.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class HeadingGains:
    kp: float = 9308.24       # duty per rad of heading error
    ki: float = 10359.7 /2     # gentle backstop for a residual degree or two;
                              # bounded by integral_limit below
    kd: float = 2054.4        # duty per (rad/s) -- damping. THE anti-overshoot
                              # term: raise it if the kart still swings past,
                              # lower it if the turn crawls or judders. It is
                              # deliberately large -- it has to command REVERSE
                              # duty to brake, since coasting alone carries the
                              # kart tens of degrees past the target.
    output_limit  : float = 2800.0
    integral_limit: float = 800.0
    # Deceleration ceiling, same idea as PositionGains.decel_gain: cap |duty| at
    # decel_gain*sqrt(|err|) so the approach follows w ~ sqrt(2*a*theta).
    decel_gain    : float = 1800.0
    min_turn_duty : float = 1200.0
    pulse_floor   : bool  = True
    tolerance     : float = 0.008726646259971648 # 0.5 deg
    settle_rate   : float = 1.8     # rad/s (~7 deg/s). Loosening this is the
                                     # fastest way to reintroduce overshoot: the
                                     # kart coasts for whatever rate it is still
                                     # carrying when the loop lets go.
    settle_ticks  : int   = 3
    max_time      : float = 20.0     # s, safety timeout per turn (was 6.0; the
                                     # damped approach trades speed for accuracy)


# ---------------------------------------------------------------------------
# Encoder wiring: motor tag -> (phase_a_gpio, phase_b_gpio)
# BCM pin numbering.
#
# Physical positions on this build:
#   M1 = upper-left    M4 = upper-right
#   M2 = lower-left    M3 = lower-right
#
# NOTE: M1/M4 were originally listed on GPIO 12/13 and 10/11, but those encoders
# were physically plugged into the PCA9685 "SERVO_2..5" headers -- PCA9685 *chip*
# outputs driven over I2C, NOT the Pi GPIO of the same numbers -- so pigpio never
# saw an edge (pins floated at the pull-up, always HIGH). Now rewired to real Pi
# GPIO:
#   M1 -> GPIO 5 (phys pin 29), GPIO 6 (phys pin 31)   -- clean GPIO
#   M4 -> GPIO 7 (phys pin 26), GPIO 8 (phys pin 24)   -- SPI0 CE1/CE0
# GPIO 7/8 are the SPI0 chip-select pins, so SPI0 MUST be disabled for pigpio to
# own them: comment out `dtparam=spi=on` in /boot/firmware/config.txt and reboot.
# The only SPI0 user was the WS2812 LED strip (Led.py), which is unused here.
# Power the M1/M4 encoders from the SAME supply as M2/M3 (3.3 V) so the outputs
# stay in the Pi's GPIO-safe range -- the header pins are NOT 5 V tolerant.
# ---------------------------------------------------------------------------
ENCODER_PINS: Dict[str, Tuple[int, int]] = {
    "M1": (25, 5),    # upper-left (rewired to Pi GPIO, phys pins 29/31)
    "M2": (26, 20),   # lower-left
    "M3": (6, 12),    # lower-right (moved off 19/16 — was under-counting)
    "M4": (8, 7),     # upper-right (rewired to Pi GPIO SPI0 pins; needs SPI off)
}

# Fixed slot order for the encoder counts in shared memory (Server/ipc.py).
# Spelled out rather than derived from ENCODER_PINS' insertion order, because
# two processes have to agree on this layout and a reordered dict literal would
# silently swap motors between sides.
ENCODER_TAGS: Tuple[str, ...] = ("M1", "M2", "M3", "M4")

# ---------------------------------------------------------------------------
# Single-phase (degraded) encoders.
#
# M3's phase-B line (GPIO 12) is physically dropping ~40% of its edges and can't
# be repaired, so full quadrature under-counts and fabricates a fake heading.
# We therefore decode M3 on phase A ALONE:
#   * phase A gives clean magnitude but HALF the ticks/rev (x2 = 1170), so its
#     count is scaled x2 to match the x4 (2340) motors when aggregating a side;
#   * a single phase can't tell rotation direction, so it's borrowed from the
#     same-side partner (M4), which is healthy and always turns the same way.
# The raw per-motor count stays the honest phase-A tick count (NOT scaled); the
# x2 only applies inside the side aggregation / distance conversion.
#   tag -> {"direction_from": partner_tag, "scale": float}
SINGLE_PHASE_ENCODERS: Dict[str, dict] = {
    "M3": {"direction_from": "M4", "scale": 2.0},
}

# Which motor tags belong to which side, and the sign of their counts so that
# "forward" produces positive counts on both sides.
#
# Grouping (matches the physical positions above and Motor.setMotorModel, whose
# duty1/duty2 drive the left pair and duty3/duty4 the right pair):
#   left  = M1 (upper-left)  + M2 (lower-left)
#   right = M3 (lower-right) + M4 (upper-right)
# Only the side grouping matters for skid-steer odometry (the two encoders on a
# side are averaged), so the upper/lower order within a side is irrelevant.
#
# The SIGNS must be verified on hardware: drive forward and check each side's
# count goes positive. If a side counts backwards, flip its two signs here -
# no logic changes needed.
@dataclass(frozen=True)
class SideMapping:
    left: Tuple[str, ...]  = ("M1", "M2")
    right: Tuple[str, ...] = ("M3", "M4")
    # Per-motor count direction (+1 / -1).
    signs: Dict[str, int] = field(default_factory=lambda: {
        "M1": 1, "M2": 1, "M3": -1, "M4": -1,
    })

MOTOR_CHANNELS = {
    "left_upper":  (0, 1),
    "left_lower":  (2, 3),
    "right_upper": (7, 6),
    "right_lower": (4, 5),
}


# ---------------------------------------------------------------------------
# Control-loop and networking
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ControlConfig:
    loop_hz                  : float = 20.0
    telemetry_hz             : float = 10.0
    command_timeout          : float = 0.1   # s; stop motors if no drive cmd arrives
    minimum_front_distance_cm: int   = 10    # front guard trips below this (cm)
    front_distance_ttl_s     : float = 0.5   # drop the REPORTED distance after
                              # this long with no valid reading
    max_linear : float = 0.6  # m/s, saturates drive commands
    max_angular: float = 4.0  # rad/s


# ---------------------------------------------------------------------------
# Slip detection: gyro yaw rate vs. the encoder differential.
#
# The kart carries two independent measurements of the SAME quantity every tick:
#
#   d_theta_gyro    from the MPU6050 (ground truth -- slip-immune, since the
#                   chassis rotates whether or not the wheels grip)
#   d_theta_encoder (d_right - d_left) / track  (believes the wheels)
#
# ``step`` already computes both and then throws the encoder one away whenever
# the gyro is live. Their DISAGREEMENT is a slip detector, and a better one than
# the accelerometer test in the literature (De Giorgi et al., Robotics 2024,
# 13, 7, Eq. 7-8) gives on this hardware: yaw rate is the one thing an MPU6050
# measures well, while double-integrating its accelerometer for displacement is
# hopeless at this price point.
#
# What it catches: wheels scrubbing sideways in an in-place turn, one side
# spinning up on a slick patch, a wheel blocked against a wall while the other
# drives. All of those make the encoders claim a rotation the chassis did not
# perform (or miss one it did).
#
# What it does NOT catch: both sides slipping equally in a straight line. The
# differential is zero in that case and so is the gyro, so the residual stays
# quiet while the kart makes no progress. That mode needs an exteroceptive
# reference (which is what the orchestrator's 1-D range correction provides).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SlipConfig:
    # Absolute floor on the residual, in rad/s of disagreement. Sized above the
    # honest noise of the two sources at 20 Hz: one dropped encoder count on a
    # side is ~(mpc/track)/dt ~ 0.01 rad/s, and the gyro's own rate noise after
    # bias removal is a few hundredths.
    rate_tolerance: float = 0.35

    # Proportional term: allow this fraction of the LARGER of the two rates
    # before calling it slip. A fast turn legitimately disagrees by more in
    # absolute terms than a slow one (quantisation, the 20 Hz sampling of a
    # ramping rate), so a fixed threshold alone would flag every hard turn.
    rate_fraction: float = 0.30

    # Consecutive ticks over the threshold before `slipping` latches true, and
    # consecutive clean ticks before it clears. Slip is a physical event lasting
    # many ticks at 20 Hz; a single-tick spike is a sample alignment artefact
    # (the encoder delta and the gyro delta are read microseconds apart but
    # cover slightly different windows). Mirrors the paper's own advice to test
    # the condition over "few steps instead of only one" (Remark 1).
    enter_ticks: int = 3
    exit_ticks: int = 5

    # Below this body rate BOTH sources read ~0 and their ratio is meaningless,
    # so the detector stays quiet -- it has nothing to compare. Prevents a
    # parked kart from latching slip on pure noise.
    min_rate: float = 0.15

    # Ratio reported in telemetry is clamped here, purely so a division by a
    # near-zero gyro rate cannot publish an absurd number to the UI.
    max_ratio: float = 10.0


@dataclass(frozen=True)
class NetworkConfig:
    web_port: int = 8080
    tcp_command_port: int = 5000
    tcp_video_port: int = 8000
    interface: str = "wlan0"


# ---------------------------------------------------------------------------
# Multiprocess layout (Raspberry Pi 3B+: 4x Cortex-A53 @ 1.4 GHz, 1 GB RAM).
#
# The subsystems run as four OS processes so the 20 Hz control loop gets its own
# GIL and its own core instead of competing with ~24k/s pigpio encoder callbacks
# and the aiohttp/WebSocket loop:
#
#   P_web      (parent)  aiohttp, WebSocket, HTTP, TCP legacy, supervision
#   P_control            DriveController.step() + Motor + Servo (SOLE PWM owner)
#   P_sensors            encoders (pigpio), MPU6050, ultrasonic, ADC, IR line
#   P_camera             Picamera2 + JPEG encode -> shared frame ring
#   P_aux                LED animations, buzzer, legacy autonomous modes
#
# The control loop's arithmetic is NOT cpu-bound (microseconds per tick); what
# it needs is determinism, which is why control_cpu / control_rt_priority exist.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ProcessConfig:
    # CPU pinning. `None` disables pinning for that process. P_control gets a
    # core to itself; P_web (the parent) is pushed off it.
    control_cpu: int | None = 3
    sensors_cpu: int | None = 2
    web_cpus: Tuple[int, ...] = (0, 1)

    # Scheduling. SCHED_FIFO needs root (the project already runs under sudo for
    # pigpio/SPI); each step degrades gracefully to the next.
    control_rt_priority: int = 20     # 0 disables SCHED_FIFO, falls back to nice
    control_nice: int = -10
    sensors_nice: int = -5
    aux_nice: int = 10                # animations must never preempt control

    # Publication rates inside P_sensors.
    sensor_publish_hz: float = 100.0  # encoder totals + gyro + guard -> shm
    adc_poll_hz: float = 5.0          # battery / photoresistors (slow I2C)
    line_poll_hz: float = 20.0        # IR line sensors (GPIO)

    # Staleness limits. Across processes "the object exists" no longer proves
    # the data is fresh: a wedged or killed producer would otherwise publish its
    # last value forever and the control loop would close its heading PID on a
    # frozen yaw. Every shm consumer checks these.
    gyro_stale_s: float = 0.25
    guard_stale_s: float = 0.50
    control_heartbeat_s: float = 1.0  # watchdog: stale beyond this -> escalate
    control_kill_after_s: float = 2.0 # ...and beyond this -> terminate + stop

    # Camera frame ring (shared memory, no pickling). 3 slots at 20 fps gives a
    # reader ~150 ms to copy a frame out before its slot is reused.
    frame_slots: int = 3
    frame_slot_bytes: int = 320 * 1024

    # Bounded queues. Producers use put_nowait + drop-oldest, so a stalled
    # consumer can never block the web loop or the control loop.
    command_queue_size: int = 64
    telemetry_queue_size: int = 4
    event_queue_size: int = 256


# ---------------------------------------------------------------------------
# Aggregate configuration object
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RobotConfig:
    wheel   : WheelGeometry = field(default_factory=WheelGeometry)
    pid     : PIDGains      = field(default_factory=PIDGains)
    position: PositionGains = field(default_factory=PositionGains)
    heading : HeadingGains  = field(default_factory=HeadingGains)
    sides   : SideMapping   = field(default_factory=SideMapping)
    control : ControlConfig = field(default_factory=ControlConfig)
    network : NetworkConfig = field(default_factory=NetworkConfig)
    process : ProcessConfig = field(default_factory=ProcessConfig)
    slip    : SlipConfig    = field(default_factory=SlipConfig)


CONFIG = RobotConfig()