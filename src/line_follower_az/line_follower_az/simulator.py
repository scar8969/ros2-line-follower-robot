#!/usr/bin/env python3
"""simulator.py — headless 2D line-follower simulator (no ROS2, no Gazebo).

This is the piece the original project never shipped: a way to run the exact
same sensor -> PID -> motor pipeline on a laptop with no ROS2 install. It
simulates a differential-drive robot driving over a procedurally generated
stadium track and reports lap metrics.

Why this exists
---------------
The original repo is ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 only — you
cannot run it on Windows or without a full ROS2 install. This simulator keeps
the *identical* control architecture (sensor_model -> pid_model ->
motor_model) and replaces only the Gazebo physics with a 2D kinematic model,
so the control code is verified on any machine. The ROS2 nodes import the very
same model modules, so what you tune here is what runs in Gazebo.

Track model
-----------
A stadium track (two straights + two semicircles) is sampled into a dense
polyline of (x, y) centerline points. The robot is a point robot with
(x, y, theta). Each tick:

  1. A synthetic "camera" image is rendered: the ground is light gray and the
     track strip under the robot is dark. The strip is drawn as a thick
     polyline in image space, so the sensor sees a realistic curved line.
  2. sensor_model.compute_line_error() turns the image into a tracking error.
  3. pid_model.PIDController.update() turns the error into (v, w).
  4. motor_model.DifferentialDrive.twist_to_wheels() -> wheel speeds (telemetry).
  5. The robot pose integrates (v, w) with the standard unicycle model.

Metrics
-------
Per lap and overall: lap time, distance, mean/max |error|, line-loss count,
mean linear speed, and a "tracking score" (fraction of time |error| < 0.35).
"""

import argparse
import math
import os
import sys

# Make `python3 line_follower_az/simulator.py` work from anywhere in the repo,
# and `python3 -m line_follower_az.simulator` work from src/line_follower_az.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from line_follower_az.sensor_model import compute_line_error, synthesize_line_image
from line_follower_az.pid_model import PIDController
from line_follower_az.motor_model import DifferentialDrive

# ---------------------------------------------------------------------------
# Track geometry (same numbers as the original generate_track.py)
# ---------------------------------------------------------------------------
STRAIGHT_LEN = 2.0      # m, each straight section
TURN_RADIUS = 0.6       # m, semicircle radius
LINE_WIDTH = 0.04       # m, painted line width
TRACK_SAMPLE_M = 0.02   # m, centerline sampling density

# ---------------------------------------------------------------------------
# Robot / sensor / control parameters (mirror config/pid_params.yaml)
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.15
MAX_WHEEL_SPEED = 12.0
KP, KI, KD = 0.8, 0.02, 0.15
BASE_LINEAR = 0.12
MIN_LINEAR = 0.04
MAX_ANGULAR = 2.5
INTEGRAL_CLAMP = 1.0

# Sensor / rendering
IMG_W, IMG_H = 320, 240
ROI_TOP = 0.55
THRESHOLD = 90
NUM_ZONES = 5
ZONE_ACTIVE = 0.12
CAM_LOOKAHEAD_M = 0.18   # camera looks this far ahead of the robot center
PX_PER_M = 900           # image-space scale (pixels per meter)
LINE_PX = 26             # line thickness in image space

DT = 1.0 / 30.0          # control rate, same as the Gazebo camera (30 Hz)
MAX_SIM_S = 120.0
LAP_ERROR_THRESHOLD = 0.35


def build_track_centerline(kind: str = "stadium"):
    """Sample a track centerline into a dense polyline of (x, y) points.

    Supported kinds:
      "stadium"  — two straights + two semicircles (the classic)
      "figure8"  — two lobes crossing at the center (self-intersecting)
      "hairpin"  — long straights with tight 180-degree switchbacks
    """
    L, R = STRAIGHT_LEN, TURN_RADIUS
    pts = []

    def straight(x0, y0, x1, y1):
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(length / TRACK_SAMPLE_M))
        # Include the ENDPOINT so consecutive segments connect (no gaps).
        for i in range(n + 1):
            t = i / n
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))

    def arc(cx, cy, r, a0_deg, a1_deg):
        a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
        # Angular step that keeps chord length ~TRACK_SAMPLE_M.
        step = TRACK_SAMPLE_M / r
        steps = max(4, int(abs(a1 - a0) / step))
        # Include the ENDPOINT so consecutive segments connect (no gaps).
        for i in range(steps + 1):
            a = a0 + (a1 - a0) * i / steps
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))

    if kind == "figure8":
        # Two tangent circles of radius R, centers at (-R, 0) and (R, 0),
        # touching at the crossing (origin). Trace as ONE continuous loop:
        #   left lobe:  full circle CCW, 0 -> 360 (starts/ends at the crossing)
        #   right lobe: full circle CW, 180 -> -180 (starts/ends at the crossing)
        arc(-R, 0, R, 0, 360)
        arc(R, 0, R, 180, -180)
        return np.array(pts)

    if kind == "hairpin":
        # A "U" track: top straight, right switchback, bottom straight, then a
        # left-side return arc. Continuous, closed loop.
        y_top, y_bot = 1.2, -1.2
        # Top straight: left -> right
        straight(-L / 2, y_top, L / 2, y_top)
        # Right switchback: 180 turn from (L/2, y_top) down to (L/2, y_bot)
        arc(L / 2, 0.0, (y_top - y_bot) / 2, 90, -90)
        # Bottom straight: right -> left
        straight(L / 2, y_bot, -L / 2, y_bot)
        # Left return arc: from (-L/2, y_bot) up to (-L/2, y_top), closing the loop
        arc(-L / 2, 0.0, (y_top - y_bot) / 2, -90, 90)
        return np.array(pts)

    # Default: stadium
    # Top straight (left -> right), right semicircle (down), bottom straight
    # (right -> left), left semicircle (bottom -> top through the left side).
    straight(-L / 2, R, L / 2, R)
    arc(L / 2, 0, R, 90, -90)
    straight(L / 2, -R, -L / 2, -R)
    arc(-L / 2, 0, R, -90, -270)
    return np.array(pts)


def closest_point_on_path(pts, x, y):
    """Return (index, distance) of the closest centerline sample."""
    d = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
    i = int(np.argmin(d))
    return i, float(np.sqrt(d[i]))


def render_camera_view(pts, x, y, theta):
    """Render the synthetic downward-camera image.

    The camera looks CAM_LOOKAHEAD_M ahead of the robot along its heading.
    We build an image-space polyline of the track centerline in the camera
    frame (x forward, y left, origin at the robot), then draw it as a thick
    dark strip on a light background. The sensor model then sees exactly what
    the Gazebo camera would: a curved dark line on a light floor.
    """
    # Camera origin: ahead of the robot center.
    cx = x + CAM_LOOKAHEAD_M * math.cos(theta)
    cy = y + CAM_LOOKAHEAD_M * math.sin(theta)

    # Rotate world points into the camera frame: forward = +x, left = +y.
    cos_t, sin_t = math.cos(-theta), math.sin(-theta)
    rel = pts - np.array([cx, cy])
    cam = np.stack([
        rel[:, 0] * cos_t - rel[:, 1] * sin_t,
        rel[:, 0] * sin_t + rel[:, 1] * cos_t,
    ], axis=1)

    # Only points within the image frustum matter (roughly 0.4 m ahead).
    near = cam[(cam[:, 0] > -0.05) & (cam[:, 0] < 0.45) & (np.abs(cam[:, 1]) < 0.25)]
    if near.shape[0] < 2:
        # Nothing visible: return a BLANK image (no line). The sensor then
        # reports "no line" (None) and the controller can react, instead of
        # fabricating a centered line that makes an off-track robot think it
        # is perfectly on track.
        return np.full((IMG_H, IMG_W), 220, dtype=np.uint8)

    # Image space: x -> columns (right), y -> rows (down). Ground is light.
    # NOTE: the camera looks DOWN along the robot's forward axis with +y to
    # the robot's LEFT; a point to the robot's left must appear on the LEFT
    # side of the image (cols < center), matching the real camera orientation
    # the sensor model expects (left-of-image -> negative error).
    img = np.full((IMG_H, IMG_W), 220, dtype=np.uint8)
    cols = IMG_W // 2 - (near[:, 1] * PX_PER_M).astype(int)
    rows = IMG_H - (near[:, 0] * PX_PER_M).astype(int)
    valid = (cols >= 0) & (cols < IMG_W) & (rows >= 0) & (rows < IMG_H)
    cols, rows = cols[valid], rows[valid]
    if cols.size == 0:
        return synthesize_line_image(IMG_W, IMG_H, IMG_W // 2, LINE_PX)

    half = LINE_PX // 2
    for c, r in zip(cols, rows):
        c0, c1 = max(0, c - half), min(IMG_W, c + half)
        r0, r1 = max(0, r - half), min(IMG_H, r + half)
        img[r0:r1, c0:c1] = 10
    return img


class LineFollowerSim:
    """2D kinematic line-follower simulator (sensor -> PID -> motor)."""

    def __init__(self, seed: int = 0, base_linear_speed: float = BASE_LINEAR,
                 min_linear_speed: float = MIN_LINEAR, max_angular_z: float = MAX_ANGULAR,
                 sensor_noise: float = 0.0, wheel_slip: float = 0.0,
                 track_kind: str = "stadium"):
        self.track_kind = track_kind
        self.track = build_track_centerline(track_kind)
        self.track_len = float(self._track_length())
        self.rng = np.random.default_rng(seed)
        self.sensor_noise = sensor_noise

        # Start on the top straight, heading +x (stadium/hairpin) or the
        # left lobe (figure8).
        self.x = -STRAIGHT_LEN / 2
        self.y = TURN_RADIUS if track_kind != "figure8" else 0.0
        self.theta = 0.0
        if track_kind == "figure8":
            # Start at the leftmost point of the left lobe, heading +y (CCW).
            self.x = -2.0 * TURN_RADIUS
            self.y = 0.0
            self.theta = math.pi / 2.0
        elif track_kind == "hairpin":
            # Start at the top straight's left end, heading +x.
            self.x = -STRAIGHT_LEN / 2
            self.y = 1.2
            self.theta = 0.0

        self.pid = PIDController(KP, KI, KD, base_linear_speed, min_linear_speed,
                                 max_angular_z, INTEGRAL_CLAMP)
        self.drive = DifferentialDrive(WHEEL_RADIUS, WHEEL_SEPARATION, MAX_WHEEL_SPEED,
                                       slip_factor=wheel_slip)

        self.t = 0.0
        self.lap = 0
        self.lap_start_t = 0.0
        self.lap_start_idx = 0
        self._odom_dist = 0.0

        self.lap_metrics = []          # per-lap dicts
        self.telemetry = []            # (t, error, lin_x, ang_z, v_left, v_right, x, y, theta)

        self._last_error = 0.0
        self._have_seen = False
        self._line_losses = 0
        self._cross_track = []       # per-step distance from track centerline (m)

    def _track_length(self):
        d = np.diff(self.track, axis=0)
        return float(np.sum(np.hypot(d[:, 0], d[:, 1])))

    def _progress(self):
        """Distance along the track of the closest point, plus lap count via wraps."""
        idx, _ = closest_point_on_path(self.track, self.x, self.y)
        # cumulative arc length up to idx
        d = np.diff(self.track[:idx + 1], axis=0)
        s = float(np.sum(np.hypot(d[:, 0], d[:, 1]))) if idx > 0 else 0.0
        return s, idx

    def _traveled(self):
        """Cumulative distance traveled by the robot (odometry-style)."""
        return self._odom_dist

    def step(self):
        img = render_camera_view(self.track, self.x, self.y, self.theta)
        error, active = compute_line_error(
            img, NUM_ZONES, ROI_TOP, THRESHOLD, ZONE_ACTIVE)

        # Optional sensor noise (for robustness testing).
        if error is not None and self.sensor_noise > 0.0:
            error = float(np.clip(error + self.rng.normal(0.0, self.sensor_noise), -1.0, 1.0))

        if error is None:
            self._line_losses += 1

        lin_x, ang_z = self.pid.update(error, dt=DT)
        v_left, v_right = self.drive.twist_to_wheels(lin_x, ang_z)

        # Unicycle integration.
        self.x += lin_x * math.cos(self.theta) * DT
        self.y += lin_x * math.sin(self.theta) * DT
        self.theta += ang_z * DT
        self.t += DT
        self._odom_dist += abs(lin_x) * DT   # odometry: distance traveled

        self.telemetry.append((self.t, error, lin_x, ang_z, v_left, v_right,
                               self.x, self.y, self.theta))

        # Lap tracking: use TRAVELED distance (odometry), which is monotonic
        # and immune to the closest-point snapping at the start/end seam.
        # A lap completes when traveled distance crosses track_len (plus a
        # small tolerance for the first partial lap).
        if self._odom_dist >= (self.lap + 1) * self.track_len - 0.1:
            self._close_lap()

        # Cross-track error (distance from the track centerline).
        _, dist = closest_point_on_path(self.track, self.x, self.y)
        self._cross_track.append(dist)

    def _close_lap(self):
        lap_t = self.t - self.lap_start_t
        # Only scan the telemetry rows recorded during THIS lap (O(lap) not O(n^2)).
        rows = [r for r in self.telemetry[self.lap_start_idx:] if r[0] <= self.t]
        errors = [abs(r[1]) for r in rows if r[1] is not None]
        speeds = [r[2] for r in rows]
        losses = sum(1 for r in rows if r[1] is None)  # frames where the line was lost
        tracking = sum(1 for e in errors if e < LAP_ERROR_THRESHOLD) / max(1, len(errors))
        # Cross-track error for this lap: distance from the centerline (m).
        ct = self._cross_track[self.lap_start_idx:len(rows) + self.lap_start_idx]
        self.lap_metrics.append({
            "lap": self.lap + 1,
            "time_s": round(lap_t, 3),
            "dist_m": round(self.track_len, 3),
            "mean_err": round(float(np.mean(errors)) if errors else 0.0, 4),
            "max_err": round(float(np.max(errors)) if errors else 0.0, 4),
            "mean_speed": round(float(np.mean(speeds)) if speeds else 0.0, 4),
            "tracking": round(tracking, 4),
            "line_losses": losses,
            "mean_cte_m": round(float(np.mean(ct)) if ct else 0.0, 4),
            "max_cte_m": round(float(np.max(ct)) if ct else 0.0, 4),
        })
        self.lap += 1
        self.lap_start_t = self.t
        self.lap_start_idx = len(self.telemetry)

    def run(self, laps: int = 3, max_time: float = MAX_SIM_S):
        target = laps
        while self.lap < target and self.t < max_time:
            self.step()
        return self.lap_metrics


def plot_results(sim: LineFollowerSim, out_dir: str):
    tel = np.array(sim.telemetry)
    t = tel[:, 0]
    err = tel[:, 1]
    lin = tel[:, 2]
    ang = tel[:, 3]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, err, color="#1f77b4", lw=1.2)
    axes[0].axhline(0, color="gray", lw=0.6)
    axes[0].axhline(0.35, color="orange", ls="--", lw=0.8, label="tracking band ±0.35")
    axes[0].axhline(-0.35, color="orange", ls="--", lw=0.8)
    axes[0].set_ylabel("line error")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title("Line-follower telemetry — headless 2D simulator")

    axes[1].plot(t, lin, color="green", lw=1.2, label="linear_x")
    axes[1].plot(t, ang, color="red", lw=1.0, label="angular_z")
    axes[1].set_ylabel("cmd_vel")
    axes[1].legend(loc="upper right", fontsize=8)

    axes[2].plot(t, tel[:, 4], color="purple", lw=1.0, label="v_left")
    axes[2].plot(t, tel[:, 5], color="darkorange", lw=1.0, label="v_right")
    axes[2].set_ylabel("wheel speed (rad/s)")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "telemetry.png"), dpi=110)
    plt.close(fig)

    # Track + robot path overlay.
    fig2, ax2 = plt.subplots(figsize=(7, 6))
    ax2.plot(sim.track[:, 0], sim.track[:, 1], color="black", lw=3, label="track centerline")
    ax2.plot(tel[:, 6], tel[:, 7], color="#1f77b4", lw=1.2, alpha=0.8, label="robot path")
    ax2.scatter([tel[0, 6]], [tel[0, 7]], color="green", s=40, label="start")
    ax2.set_aspect("equal")
    ax2.set_title("Robot path over stadium track")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(os.path.join(out_dir, "track_path.png"), dpi=110)
    plt.close(fig2)


def main():
    ap = argparse.ArgumentParser(description="Headless 2D line-follower simulator")
    ap.add_argument("--laps", type=int, default=3, help="laps to complete")
    ap.add_argument("--seed", type=int, default=0, help="RNG seed (noise)")
    ap.add_argument("--out", default="results", help="output directory")
    ap.add_argument("--noise", type=float, default=0.0,
                    help="std of sensor noise added to the error (0 = clean)")
    ap.add_argument("--speed", type=float, default=BASE_LINEAR,
                    help="base linear speed in m/s (default 0.12)")
    ap.add_argument("--slip", type=float, default=0.0,
                    help="wheel slip factor (0 = none, 0.1 = 10% at max speed)")
    ap.add_argument("--track", type=str, default="stadium",
                    choices=["stadium", "figure8", "hairpin"],
                    help="track layout (default stadium)")
    ap.add_argument("--max-time", type=float, default=MAX_SIM_S,
                    help="max simulation seconds")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    sim = LineFollowerSim(seed=args.seed, base_linear_speed=args.speed,
                          sensor_noise=args.noise, wheel_slip=args.slip,
                          track_kind=args.track)
    metrics = sim.run(laps=args.laps, max_time=args.max_time)

    print(f"track length: {sim.track_len:.2f} m")
    print(f"simulated {len(metrics)} lap(s), {sim.t:.1f} s of driving")
    print(f"{'lap':>4s} {'time_s':>8s} {'dist_m':>7s} {'mean_err':>9s} {'max_err':>8s} "
          f"{'mean_spd':>9s} {'tracking':>9s} {'mean_cte':>9s} {'losses':>7s}")
    for m in metrics:
        print(f"{m['lap']:>4d} {m['time_s']:>8.2f} {m['dist_m']:>7.2f} "
              f"{m['mean_err']:>9.4f} {m['max_err']:>8.4f} {m['mean_speed']:>9.4f} "
              f"{m['tracking']:>9.3f} {m['mean_cte_m']:>9.4f} {m['line_losses']:>7d}")

    plot_results(sim, out_dir)
    print(f"\nplots written to {out_dir}/telemetry.png and {out_dir}/track_path.png")

    # Exit code: 0 if every lap tracked well.
    ok = all(m["tracking"] >= 0.9 for m in metrics) and len(metrics) >= 1
    print("RESULT:", "PASS — robot tracked the line for all laps" if ok else "FAIL — tracking degraded")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
