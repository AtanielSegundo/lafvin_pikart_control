#!/usr/bin/python3
"""
Skid-steer / differential-drive kinematics.

The two left wheels move together and the two right wheels move together, so
the 4-wheel platform behaves as a 2-wheel differential drive with a track
width equal to the lateral distance between the left and right sides.

Forward (wheel speeds -> body twist), matching the CoppeliaSim reference
``DiffDrive.step``:

    v = (v_right + v_left) / 2
    w = (v_right - v_left) / track

Inverse (body twist -> wheel speeds):

    v_left  = v - w * track / 2
    v_right = v + w * track / 2

All linear speeds are m/s, angular speeds rad/s, ``track`` in metres.
"""
from __future__ import annotations

from dataclasses import dataclass

from config import WheelGeometry


@dataclass(frozen=True)
class Twist:
    """Planar body velocity."""
    linear: float = 0.0    # m/s, +x forward
    angular: float = 0.0   # rad/s, +ccw


@dataclass(frozen=True)
class WheelSpeeds:
    """Linear speed of each virtual side, m/s."""
    left: float = 0.0
    right: float = 0.0


class SkidSteerKinematics:
    def __init__(self, geometry: WheelGeometry):
        self.geometry = geometry

    @property
    def track(self) -> float:
        return self.geometry.track

    def forward(self, wheels: WheelSpeeds) -> Twist:
        v = (wheels.right + wheels.left) / 2.0
        w = (wheels.right - wheels.left) / self.track
        return Twist(linear=v, angular=w)

    def inverse(self, twist: Twist) -> WheelSpeeds:
        half = twist.angular * self.track / 2.0
        return WheelSpeeds(left=twist.linear - half,
                           right=twist.linear + half)
