#!/usr/bin/env python3
"""auto_tune.py — grid-search PID gains over the headless simulator.

Sweeps Kp/Ki/Kd (and optionally base_linear_speed) over the 2D simulator,
scores each combination by lap tracking quality, and writes:
  * results/auto_tune_report.txt  — ranked table
  * results/auto_tune_heatmap.png — Kp x Kd heatmap (best Ki)
  * results/auto_tune_best.json   — best gains + score

Usage:
    python3 scripts/auto_tune.py                 # default grid
    python3 scripts/auto_tune.py --laps 1 --fast # small grid, quick smoke
"""

import argparse
import itertools
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from line_follower_az.simulator import LineFollowerSim

DEFAULT_GRID = {
    "Kp": [0.4, 0.6, 0.8, 1.0, 1.2],
    "Ki": [0.0, 0.01, 0.02, 0.05],
    "Kd": [0.05, 0.1, 0.15, 0.2],
}

FAST_GRID = {
    "Kp": [0.6, 0.8, 1.0],
    "Ki": [0.0, 0.02],
    "Kd": [0.1, 0.15],
}


def score_run(metrics):
    """Score a lap set: higher is better, dominated by tracking quality."""
    if not metrics:
        return 0.0
    lap = metrics[0]
    # tracking in [0,1], mean_err in [0,~1], time in s
    tracking = lap["tracking"]
    mean_err = lap["mean_err"]
    time_s = lap["time_s"]
    # Penalize: low tracking, high mean error, slow laps.
    return tracking * 100.0 - mean_err * 20.0 - min(time_s / 10.0, 10.0)


def main():
    ap = argparse.ArgumentParser(description="Grid-search PID gains over the headless simulator")
    ap.add_argument("--laps", type=int, default=1, help="laps per evaluation")
    ap.add_argument("--max-time", type=float, default=120.0, help="max sim seconds per eval")
    ap.add_argument("--fast", action="store_true", help="use a small grid (smoke test)")
    ap.add_argument("--out", default="results", help="output directory")
    args = ap.parse_args()

    grid = FAST_GRID if args.fast else DEFAULT_GRID
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    combos = list(itertools.product(grid["Kp"], grid["Ki"], grid["Kd"]))
    print(f"grid: {len(combos)} combos x {args.laps} lap(s) each")

    results = []
    for i, (kp, ki, kd) in enumerate(combos):
        sim = LineFollowerSim(seed=0, base_linear_speed=0.12)
        # Override gains on the sim's controller.
        sim.pid.Kp, sim.pid.Ki, sim.pid.Kd = kp, ki, kd
        metrics = sim.run(laps=args.laps, max_time=args.max_time)
        score = score_run(metrics)
        results.append((score, kp, ki, kd, metrics))
        if (i + 1) % 10 == 0 or i == len(combos) - 1:
            print(f"  [{i+1}/{len(combos)}] best so far: {max(r[0] for r in results):.2f}")

    results.sort(key=lambda r: r[0], reverse=True)

    # ---- report ----
    report_path = os.path.join(out_dir, "auto_tune_report.txt")
    with open(report_path, "w") as f:
        f.write("PID auto-tune report (headless simulator)\n")
        f.write("=" * 60 + "\n")
        f.write(f"{'rank':>4s} {'score':>7s} {'Kp':>6s} {'Ki':>6s} {'Kd':>6s} "
                f"{'tracking':>9s} {'mean_err':>9s} {'time_s':>7s}\n")
        for rank, (score, kp, ki, kd, metrics) in enumerate(results, 1):
            m = metrics[0] if metrics else {}
            f.write(f"{rank:>4d} {score:>7.2f} {kp:>6.2f} {ki:>6.3f} {kd:>6.2f} "
                    f"{m.get('tracking', 0):>9.3f} {m.get('mean_err', 0):>9.3f} "
                    f"{m.get('time_s', 0):>7.1f}\n")
    print(f"report: {report_path}")

    # ---- heatmap (Kp x Kd, best Ki per cell) ----
    # results tuples are (score, kp, ki, kd, metrics)
    kps = sorted(set(r[1] for r in results))
    kds = sorted(set(r[3] for r in results))
    heat = np.full((len(kds), len(kps)), np.nan)
    for score, kp, ki, kd, _ in results:
        i, j = kds.index(kd), kps.index(kp)
        if np.isnan(heat[i, j]) or score > heat[i, j]:
            heat[i, j] = score

    fig, ax = plt.subplots(figsize=(7, 5))
    im = ax.imshow(heat, cmap="viridis", aspect="auto")
    ax.set_xticks(range(len(kps)), [f"{k:.1f}" for k in kps])
    ax.set_yticks(range(len(kds)), [f"{k:.2f}" for k in kds])
    ax.set_xlabel("Kp")
    ax.set_ylabel("Kd")
    ax.set_title("PID auto-tune score heatmap (best Ki per cell)")
    for i in range(len(kds)):
        for j in range(len(kps)):
            if not np.isnan(heat[i, j]):
                ax.text(j, i, f"{heat[i, j]:.0f}", ha="center", va="center", fontsize=8, color="white")
    fig.colorbar(im, label="score")
    fig.tight_layout()
    heat_path = os.path.join(out_dir, "auto_tune_heatmap.png")
    fig.savefig(heat_path, dpi=110)
    plt.close(fig)
    print(f"heatmap: {heat_path}")

    # ---- best json ----
    best = results[0]
    # results tuples are (score, kp, ki, kd, metrics)
    best_metrics = best[4]
    best_json = {
        "Kp": best[1], "Ki": best[2], "Kd": best[3],
        "score": round(best[0], 2),
        "tracking": best_metrics[0]["tracking"] if best_metrics else 0,
        "mean_err": best_metrics[0]["mean_err"] if best_metrics else 0,
        "lap_time_s": best_metrics[0]["time_s"] if best_metrics else 0,
    }
    json_path = os.path.join(out_dir, "auto_tune_best.json")
    with open(json_path, "w") as f:
        json.dump(best_json, f, indent=2)
    print(f"best: {json_path}")
    print(json.dumps(best_json, indent=2))


if __name__ == "__main__":
    main()
