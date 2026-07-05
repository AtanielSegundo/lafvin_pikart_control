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
# PID gains for per-side wheel-velocity control
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
# Encoder wiring: motor tag -> (phase_a_gpio, phase_b_gpio)
# BCM pin numbering. Taken from the original encoders.py mapping.
#
# Physical positions on this build:
#   M1 = upper-left    M4 = upper-right
#   M2 = lower-left    M3 = lower-right
# ---------------------------------------------------------------------------
ENCODER_PINS: Dict[str, Tuple[int, int]] = {
    "M1": (12, 13),   # upper-left   (SERVO_4, SERVO_5 headers)
    "M2": (26, 20),   # lower-left
    "M3": (19, 16),   # lower-right
    "M4": (10, 11),   # upper-right  (SERVO_2, SERVO_3 headers)
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
        "M1": 1, "M2": 1, "M3": 1, "M4": 1,
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
    sides: SideMapping = field(default_factory=SideMapping)
    control: ControlConfig = field(default_factory=ControlConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)


# A ready-to-use default instance. Import and override fields as needed.
CONFIG = RobotConfig()