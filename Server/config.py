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

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple


# ---------------------------------------------------------------------------
# Wheel geometry (from the CoppeliaSim TiredWheel reference)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class WheelGeometry:
    diameter: float = 0.065           # m
    colinear_distance: float = 0.12   # m, between motors on the same axle
    track: float = 0.151              # m, distance between left and right sides
    counts_per_rev: int = 2340        
                                      # (quadrature x4). CALIBRATE for your build.

    @property
    def radius(self) -> float:
        return self.diameter / 2.0

    @property
    def circumference(self) -> float:
        return math.pi * self.diameter

    @property
    def meters_per_count(self) -> float:
        """Linear distance travelled by a wheel per single encoder count."""
        return self.circumference / self.counts_per_rev


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
    kp: float = 12000.0      # duty per metre of error
    ki: float = 0.0
    kd: float = 1500.0       # duty per (m/s) — damping
    output_limit: float = 1800.0     # gentle duty cap during moves
    integral_limit: float = 800.0
    tolerance: float = 0.01          # m, arrival tolerance
    stop_speed: float = 0.03         # m/s below which we consider it stopped
    max_time: float = 20.0           # s, safety timeout per move


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
    "M1": (25, 5),     # upper-left   (rewired to Pi GPIO, phys pins 29/31)
    "M2": (26, 20),   # lower-left
    "M3": (6, 12),    # lower-right  (moved off 19/16 — was under-counting)
    "M4": (8, 7),     # upper-right  (rewired to Pi GPIO SPI0 pins; needs SPI off)
}

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
    left: Tuple[str, ...] = ("M1", "M2")
    right: Tuple[str, ...] = ("M3", "M4")
    # Per-motor count direction (+1 / -1).
    signs: Dict[str, int] = field(default_factory=lambda: {
        "M1": 1, "M2": 1, "M3": -1, "M4": -1,
    })


# ---------------------------------------------------------------------------
# Motor PWM channel wiring on the PCA9685 (from Motor.py).
# Each wheel uses two channels (forward, reverse).
# ---------------------------------------------------------------------------
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
    loop_hz: float = 20.0             # closed-loop update rate
    telemetry_hz: float = 50.0        # rate telemetry is pushed to clients
    command_timeout: float = 0.1      # s; stop motors if no drive cmd arrives
    max_linear: float = 0.6           # m/s, saturates drive commands
    max_angular: float = 4.0          # rad/s


@dataclass(frozen=True)
class NetworkConfig:
    web_port: int = 8080
    tcp_command_port: int = 5000
    tcp_video_port: int = 8000
    interface: str = "wlan0"


# ---------------------------------------------------------------------------
# Aggregate configuration object
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RobotConfig:
    wheel: WheelGeometry = field(default_factory=WheelGeometry)
    pid: PIDGains = field(default_factory=PIDGains)
    position: PositionGains = field(default_factory=PositionGains)
    sides: SideMapping = field(default_factory=SideMapping)
    control: ControlConfig = field(default_factory=ControlConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)


# default instance
CONFIG = RobotConfig()