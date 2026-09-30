#!/usr/bin/env python3
"""sensor_model.py — pure-logic line sensor (shared by the ROS2 node and the headless simulator).

Clean-room reimplementation of the 5-zone virtual IR array described in the
original project: a downward camera image is split into N horizontal zones and
each zone votes on whether it "sees" the dark line. The votes are combined into
a single normalized tracking error in [-1, 1].

  -1.0 ─────────── 0.0 ─────────── +1.0
   Left            Center           Right

This module is deliberately dependency-free (numpy only) so the same math can
run inside a ROS2 node, in a unit test, or in the 2D headless simulator.
"""

import numpy as np


def compute_line_error(
    gray: np.ndarray,
    num_zones: int = 5,
    roi_top_fraction: float = 0.55,
    binary_threshold: int = 90,
    zone_active_fraction: float = 0.12,
) -> tuple:
    """Compute the normalized line-tracking error from a grayscale image.

    Returns
    -------
    (error, active) where
        error  : float in [-1, 1] (weighted zone centroid), or None if no
                 zone sees the line (caller decides how to hold the last error)
        active : list of ints, one per zone, 1 = zone sees the line
    """
    height, width = gray.shape
    roi_start_row = int(height * roi_top_fraction)
    roi = gray[roi_start_row:height, :]

    # Pixels darker than the threshold count as "line".
    _, mask = cv2_threshold_inv(roi, binary_threshold)

    zone_width = width // num_zones
    active = []
    for i in range(num_zones):
        zone = mask[:, i * zone_width:(i + 1) * zone_width]
        fraction_dark = float(np.count_nonzero(zone)) / float(zone.size + 1e-6)
        active.append(1 if fraction_dark > zone_active_fraction else 0)

    half = num_zones // 2
    weights = list(range(-half, num_zones - half))

    active_count = sum(active)
    if active_count == 0:
        return None, active

    weighted_sum = sum(w * a for w, a in zip(weights, active))
    raw_error = weighted_sum / active_count
    error = raw_error / float(half if half > 0 else 1)
    return float(error), active


def cv2_threshold_inv(roi: np.ndarray, threshold: int) -> tuple:
    """THRESH_BINARY_INV equivalent: pixels <= threshold become 255, else 0."""
    return None, np.where(roi <= threshold, 255, 0).astype(np.uint8)


def synthesize_line_image(
    width: int,
    height: int,
    line_center_x: float,
    line_width: int = 20,
    ground_value: int = 220,
    line_value: int = 10,
    roi_start_fraction: float = 0.55,
) -> np.ndarray:
    """Build a synthetic grayscale image with a dark vertical line in the ROI.

    Used by tests and the simulator to feed the sensor model without a camera.
    """
    img = np.full((height, width), ground_value, dtype=np.uint8)
    x0 = max(0, int(line_center_x - line_width // 2))
    x1 = min(width, int(line_center_x + line_width // 2))
    img[int(height * roi_start_fraction):, x0:x1] = line_value
    return img
