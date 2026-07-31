#!/usr/bin/python3
"""
Skid-steer odometry: integrate wheel encoder deltas into a planar pose.

This is the direct counterpart of the CoppeliaSim ``DiffDrive.step`` used for
the analysis. There the pose is *driven* from commanded wheel speeds; here we
*estimate* the pose from *measured* wheel travel reported by the encoders.

We integrate from per-side distance deltas (metres) so the result is
independent of the loop timing jitter:

    d_center = (d_right + d_left) / 2
    d_theta  = (d_right - d_left) / track

Each step is modelled as a constant-curvature arc about the ICC (Instantaneous
Center of Curvature) and integrated in EXACT closed form (Dudek & Jenkin,
Computational Principles of Mobile Robotics, eq. 4-5):

    R = d_center / d_theta                 # signed turn radius
    x += R * (sin(theta + d_theta) - sin(theta))
    y += R * (cos(theta) - cos(theta + d_theta))
    theta += d_theta

When d_theta -> 0 the ICC goes to infinity (R = 0/0), so the straight-line
limit is used instead (x += d_center*cos(theta), y += d_center*sin(theta)).
This is strictly more accurate than the midpoint-heading chord approximation on
curved paths -- it closes constant-curvature arcs exactly -- and degenerates to
the same result for the tiny per-step angles seen in straight driving.

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
                              dt: float, d_theta: float | None = None) -> Pose:
        """Advance the pose given per-side distances travelled (metres).

        Integrates the step as an exact constant-curvature arc about the ICC
        (Dudek & Jenkin eq. 4-5), falling back to the straight-line limit as
        d_theta -> 0 to avoid the R = d_center / d_theta singularity.

        Translation (d_center) always comes from the encoders. Rotation uses the
        caller-supplied ``d_theta`` (radians) when given -- e.g. an MPU6050 gyro
        delta, treated as ground truth -- otherwise it falls back to the encoder
        differential (d_right - d_left) / track.
        """
        d_center = (d_right + d_left) / 2.0
        if d_theta is None:
            d_theta = (d_right - d_left) / self.track
        theta = self.pose.theta

        if abs(d_theta) < 1e-9:
            # Straight-line limit: ICC at infinity, R = d_center/d_theta is 0/0.
            self.pose.x += d_center * math.cos(theta)
            self.pose.y += d_center * math.sin(theta)
        else:
            # Exact arc about the ICC: rotate about the (signed) turn radius R.
            R = d_center / d_theta
            self.pose.x += R * (math.sin(theta + d_theta) - math.sin(theta))
            self.pose.y += R * (math.cos(theta) - math.cos(theta + d_theta))
        self.pose.theta = wrap_angle(theta + d_theta)

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
