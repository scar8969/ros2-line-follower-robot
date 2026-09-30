#!/usr/bin/env python3
"""motor_model.py — pure-logic differential-drive kinematics (shared by the
ROS2 node and the headless simulator).

Converts a Twist command (linear_x, angular_z) into left/right wheel angular
velocities using the standard differential-drive model:

    v_left  = (v - w * L/2) / r
    v_right = (v + w * L/2) / r

where L = wheel_separation, r = wheel_radius, then clamps to max_wheel_speed.

The simulator can also apply a wheel-slip model (a fraction of commanded
velocity is lost, growing with commanded speed) to make the kinematics more
realistic — the ROS2 node keeps slip disabled (the Gazebo DiffDrive plugin
handles physics).
"""


class DifferentialDrive:
    """Differential-drive kinematics with wheel-speed clamping and optional slip."""

    def __init__(
        self,
        wheel_radius: float = 0.033,
        wheel_separation: float = 0.15,
        max_wheel_speed: float = 12.0,
        slip_factor: float = 0.0,
    ):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.max_wheel_speed = max_wheel_speed
        self.slip_factor = slip_factor

    def twist_to_wheels(self, linear_x: float, angular_z: float) -> tuple:
        """Return (v_left, v_right) in rad/s, clamped to max_wheel_speed."""
        r = self.wheel_radius
        L = self.wheel_separation
        v_left = (linear_x - angular_z * L / 2.0) / r
        v_right = (linear_x + angular_z * L / 2.0) / r
        v_left = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_left))
        v_right = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_right))
        if self.slip_factor > 0.0:
            # Slip: lose a fraction of speed proportional to commanded speed.
            v_left *= (1.0 - self.slip_factor * min(1.0, abs(v_left) / self.max_wheel_speed))
            v_right *= (1.0 - self.slip_factor * min(1.0, abs(v_right) / self.max_wheel_speed))
        return float(v_left), float(v_right)

    def wheels_to_twist(self, v_left: float, v_right: float) -> tuple:
        """Inverse kinematics: (linear_x, angular_z) from wheel speeds."""
        r = self.wheel_radius
        L = self.wheel_separation
        linear_x = r * (v_left + v_right) / 2.0
        angular_z = r * (v_right - v_left) / L
        return float(linear_x), float(angular_z)
