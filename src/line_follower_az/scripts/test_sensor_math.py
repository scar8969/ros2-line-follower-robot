#!/usr/bin/env python3
"""test_sensor_math.py — standalone sanity test for the line-detection math.

No ROS2 / rclpy needed: builds synthetic grayscale images with a black line at
a known x position and checks that the weighted 5-zone algorithm reports an
error with the expected sign and a sane magnitude.

Run with: python3 scripts/test_sensor_math.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from line_follower_az.sensor_model import compute_line_error, synthesize_line_image  # noqa: E402

WIDTH, HEIGHT = 320, 240
cases = [
    ("line far left", 20, "negative"),
    ("line center", WIDTH // 2, "zero-ish"),
    ("line far right", WIDTH - 20, "positive"),
    ("line slightly left of center", WIDTH // 2 - 60, "negative"),
]

print(f"{'case':32s} {'error':>8s}  {'expected sign':>14s}  {'active zones'}")
all_ok = True
for name, x, expected in cases:
    img = synthesize_line_image(WIDTH, HEIGHT, x)
    error, active = compute_line_error(img)
    if error is None:
        print(f"{name:32s} {'NONE':>8s}  -> FAILED: no zone saw the line")
        all_ok = False
        continue
    sign_ok = (
        (expected == "negative" and error < -0.05) or
        (expected == "positive" and error > 0.05) or
        (expected == "zero-ish" and abs(error) < 0.4)
    )
    status = "OK" if sign_ok else "CHECK"
    if not sign_ok:
        all_ok = False
    print(f"{name:32s} {error:8.3f}  {expected:>14s}  {active}  [{status}]")

no_line_img = synthesize_line_image(WIDTH, HEIGHT, WIDTH // 2)
no_line_img[:] = 220  # erase the line
error, active = compute_line_error(no_line_img)
print(f"\nno line visible -> error={error}, active_zones={active} "
      f"(expected: None, all-zero — sensor_node holds last known error in this case)")

print("\nRESULT:", "ALL CASES BEHAVED AS EXPECTED" if all_ok else "SOME CASES NEED REVIEW")
sys.exit(0 if all_ok else 1)
