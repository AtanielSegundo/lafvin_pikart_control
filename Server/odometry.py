#!/usr/bin/python3
"""
Skid-steer odometry: integrate wheel encoder deltas into a planar pose.

This is the direct counterpart of the CoppeliaSim ``DiffDrive.step`` used for
the analysis. There the pose is *driven* from commanded wheel speeds; here we
*estimate* the pose from *measured* wheel travel reported by the encoders, but
the update equations are identical:

    v = (v_right + v_left) / 2
    w = (v_right - v_left) / track
    theta += w * dt
    x += v * cos(theta) * dt
    y += v * sin(theta) * dt

We integrate from per-side distance deltas (metres) so the result is
independent of the loop timing jitter:

    d_center = (d_right + d_left) / 2
    d_theta  = (d_right - d_left) / track

Heading is kept wrapped to (-pi, pi].
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from config import WheelGeometry


@dataclass
class Pose:
    x: float = 0.0        # m
    y: float = 0.0        # m
    theta: float = 0.0    # rad, wrapped to (-pi, pi]

    def as_dict(self) -> dict:
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 4),
            "theta_deg": round(math.degrees(self.theta), 2),
        }


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SkidSteerOdometry:
    def __init__(self, geometry: WheelGeometry, pose: Pose | None = None):
        self.geometry = geometry
        self.pose = pose or Pose()
        # Latest estimated body twist, useful for telemetry.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    @property
    def track(self) -> float:
        return self.geometry.track

    def reset(self, pose: Pose | None = None) -> None:
        self.pose = pose or Pose()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def update_from_distances(self, d_left: float, d_right: float,
                              dt: float) -> Pose:
        """Advance the pose given per-side distances travelled (metres).

        Uses the midpoint heading (2nd-order) for a more accurate arc estimate
        than sampling theta at the start of the step.
        """
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.track

        # Integrate position at the midpoint heading of the step.
        mid_theta = self.pose.theta + d_theta / 2.0
        self.pose.x += d_center * math.cos(mid_theta)
        self.pose.y += d_center * math.sin(mid_theta)
        self.pose.theta = wrap_angle(self.pose.theta + d_theta)

        if dt > 0.0:
            self.linear_velocity = d_center / dt
            self.angular_velocity = d_theta / dt
        return self.pose

    def update_from_counts(self, d_counts_left: int, d_counts_right: int,
                           dt: float) -> Pose:
        """Convenience wrapper: convert encoder count deltas to distances."""
        mpc = self.geometry.meters_per_count
        return self.update_from_distances(d_counts_left * mpc,
                                          d_counts_right * mpc, dt)
