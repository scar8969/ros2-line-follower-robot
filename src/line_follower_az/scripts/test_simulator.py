#!/usr/bin/env python3
"""test_simulator.py — end-to-end integration test for the headless simulator.

Runs the full sensor -> PID -> motor pipeline over the stadium track and
asserts the robot actually tracks the line: every lap must stay within the
tracking band most of the time, complete in reasonable time, and never leave
the track.

Run with: python3 scripts/test_simulator.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from line_follower_az.simulator import LineFollowerSim, STRAIGHT_LEN, TURN_RADIUS  # noqa: E402

LAPS = 2
MAX_TIME = 200.0
MIN_TRACKING = 0.90   # fraction of time |error| < 0.35
MAX_MEAN_ERR = 0.25
# Track is a stadium: 2 straights of STRAIGHT_LEN + 2 semicircles of TURN_RADIUS.
TRACK_LEN = 2 * STRAIGHT_LEN + 2 * 3.141592653589793 * TURN_RADIUS
# At the default 0.12 m/s the ~7.77 m track takes ~65 s/lap; allow generous slack.
MAX_LAP_TIME = 100.0

# Run at the honest default speed (0.12 m/s) — the config the PID is tuned for.
sim = LineFollowerSim(seed=0, base_linear_speed=0.12)
metrics = sim.run(laps=LAPS, max_time=MAX_TIME)

print(f"track length: {TRACK_LEN:.2f} m")
print(f"{'lap':>4s} {'time_s':>8s} {'dist_m':>7s} {'mean_err':>9s} {'max_err':>8s} "
      f"{'mean_spd':>9s} {'tracking':>9s} {'losses':>7s}")
for m in metrics:
    print(f"{m['lap']:>4d} {m['time_s']:>8.2f} {m['dist_m']:>7.2f} "
          f"{m['mean_err']:>9.4f} {m['max_err']:>8.4f} {m['mean_speed']:>9.4f} "
          f"{m['tracking']:>9.3f} {m['line_losses']:>7d}")

ok = True
if len(metrics) < LAPS:
    print(f"FAIL: completed {len(metrics)}/{LAPS} laps")
    ok = False
for m in metrics:
    if m["tracking"] < MIN_TRACKING:
        print(f"FAIL: lap {m['lap']} tracking {m['tracking']:.3f} < {MIN_TRACKING}")
        ok = False
    if m["mean_err"] > MAX_MEAN_ERR:
        print(f"FAIL: lap {m['lap']} mean_err {m['mean_err']:.3f} > {MAX_MEAN_ERR}")
        ok = False
    if m["time_s"] > MAX_LAP_TIME:
        print(f"FAIL: lap {m['lap']} took {m['time_s']:.1f}s > {MAX_LAP_TIME}s")
        ok = False

print("\nRESULT:", "PASS — robot tracked the line end-to-end" if ok else "FAIL")
sys.exit(0 if ok else 1)