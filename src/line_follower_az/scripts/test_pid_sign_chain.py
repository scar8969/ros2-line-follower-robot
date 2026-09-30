#!/usr/bin/env python3
"""test_pid_sign_chain.py — standalone sanity test for the PID -> Twist ->
wheel-speed sign chain.

Confirms that when the line is detected to one side, the wheel speeds come out
asymmetric in the direction that actually steers the robot back toward the
line. No ROS2 needed.

Run with: python3 scripts/test_pid_sign_chain.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from line_follower_az.pid_model import PIDController  # noqa: E402
from line_follower_az.motor_model import DifferentialDrive  # noqa: E402

wheel_radius = 0.033
wheel_separation = 0.15
base_speed = 0.12


def pid_to_twist(error):
    pid = PIDController(Kp=0.8, Ki=0.0, Kd=0.0, base_linear_speed=base_speed)
    return pid.update(error, dt=0.1)


def twist_to_wheels(linear_x, angular_z):
    drive = DifferentialDrive(wheel_radius, wheel_separation)
    return drive.twist_to_wheels(linear_x, angular_z)


cases = [
    ("line to the LEFT (error<0) -> robot should turn LEFT", -0.5),
    ("line centered (error=0) -> straight", 0.0),
    ("line to the RIGHT (error>0) -> robot should turn RIGHT", 0.5),
]

print(f"{'case':55s} {'lin_x':>7s} {'ang_z':>7s} {'v_left':>7s} {'v_right':>7s}  verdict")
all_ok = True
for label, error in cases:
    lin_x, ang_z = pid_to_twist(error)
    v_left, v_right = twist_to_wheels(lin_x, ang_z)
    if error < 0:
        # turning left means the RIGHT wheel should spin faster than left
        ok = v_right > v_left
    elif error > 0:
        ok = v_left > v_right
    else:
        ok = abs(v_left - v_right) < 1e-6
    all_ok &= ok
    print(f"{label:55s} {lin_x:7.3f} {ang_z:7.3f} {v_left:7.3f} {v_right:7.3f}  "
          f"[{'OK' if ok else 'CHECK'}]")

print("\nRESULT:", "SIGN CONVENTION CONSISTENT END-TO-END" if all_ok else "MISMATCH FOUND")
sys.exit(0 if all_ok else 1)
