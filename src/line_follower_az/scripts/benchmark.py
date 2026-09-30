#!/usr/bin/env python3
"""benchmark.py — run the controller on every track layout and compare.

Runs the headless simulator on stadium, figure8, and hairpin tracks, then
writes a ranked benchmark table to results/benchmark_report.txt and a
per-track path plot to results/benchmark_paths.png.

Usage:
    python3 scripts/benchmark.py                 # 1 lap per track
    python3 scripts/benchmark.py --laps 2        # 2 laps per track
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from line_follower_az.simulator import LineFollowerSim

TRACKS = ["stadium", "figure8", "hairpin"]


def main():
    ap = argparse.ArgumentParser(description="Benchmark the line follower across track layouts")
    ap.add_argument("--laps", type=int, default=1, help="laps per track")
    ap.add_argument("--max-time", type=float, default=200.0, help="max sim seconds per track")
    ap.add_argument("--out", default="results", help="output directory")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    rows = []
    sims = {}
    for kind in TRACKS:
        sim = LineFollowerSim(seed=0, track_kind=kind)
        metrics = sim.run(laps=args.laps, max_time=args.max_time)
        sims[kind] = sim
        if metrics:
            m = metrics[0]
            rows.append({
                "track": kind,
                "len_m": round(sim.track_len, 2),
                "time_s": m["time_s"],
                "tracking": m["tracking"],
                "mean_err": m["mean_err"],
                "max_err": m["max_err"],
                "mean_cte_m": m["mean_cte_m"],
                "max_cte_m": m["max_cte_m"],
                "losses": m["line_losses"],
            })
        else:
            rows.append({"track": kind, "len_m": round(sim.track_len, 2),
                         "time_s": 0, "tracking": 0.0, "mean_err": 0.0,
                         "max_err": 0.0, "mean_cte_m": 0.0, "max_cte_m": 0.0,
                         "losses": -1})

    # ---- report ----
    report_path = os.path.join(out_dir, "benchmark_report.txt")
    with open(report_path, "w") as f:
        f.write("Line-follower benchmark (headless simulator)\n")
        f.write("=" * 72 + "\n")
        f.write(f"{'track':>10s} {'len_m':>7s} {'time_s':>8s} {'tracking':>9s} "
                f"{'mean_err':>9s} {'max_err':>8s} {'mean_cte':>9s} {'max_cte':>8s} {'losses':>7s}\n")
        for r in rows:
            f.write(f"{r['track']:>10s} {r['len_m']:>7.2f} {r['time_s']:>8.1f} "
                    f"{r['tracking']:>9.3f} {r['mean_err']:>9.4f} {r['max_err']:>8.4f} "
                    f"{r['mean_cte_m']:>9.4f} {r['max_cte_m']:>8.4f} {r['losses']:>7d}\n")
    print(f"report: {report_path}")

    # ---- console ----
    print(f"{'track':>10s} {'len_m':>7s} {'time_s':>8s} {'tracking':>9s} "
          f"{'mean_err':>9s} {'max_cte':>9s} {'losses':>7s}")
    for r in rows:
        print(f"{r['track']:>10s} {r['len_m']:>7.2f} {r['time_s']:>8.1f} "
              f"{r['tracking']:>9.3f} {r['mean_err']:>9.4f} {r['mean_cte_m']:>9.4f} {r['losses']:>7d}")

    # ---- path plot ----
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, kind in zip(axes, TRACKS):
        sim = sims[kind]
        tel = np.array(sim.telemetry)
        ax.plot(sim.track[:, 0], sim.track[:, 1], color="black", lw=2, label="track")
        if tel.size:
            ax.plot(tel[:, 6], tel[:, 7], color="#1f77b4", lw=1.2, alpha=0.8, label="robot")
        ax.set_aspect("equal")
        ax.set_title(kind)
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    paths_path = os.path.join(out_dir, "benchmark_paths.png")
    fig.savefig(paths_path, dpi=110)
    plt.close(fig)
    print(f"paths: {paths_path}")

    # Exit 0 if every track tracked at least 80% of the time.
    ok = all(r["tracking"] >= 0.8 for r in rows)
    print("RESULT:", "PASS — all tracks tracked" if ok else "FAIL — some tracks degraded")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
