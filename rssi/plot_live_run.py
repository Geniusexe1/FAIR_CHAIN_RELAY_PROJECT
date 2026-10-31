#!/usr/bin/env python3
"""Plot a live run recorded by rssi/ros/live_logger.sh on the hospital GT map.

    .venv/bin/python rssi/plot_live_run.py rssi/out/live_runs/run6_P0m45_fixes.csv [coordinator.log]

Draws each robot's ground-truth path (Gazebo poses), start / end markers, the GCS, the
closest approach between any two robots, and -- if the coordinator log is given -- every
DEPLOY decision's relay targets. Writes <csv>.png next to the CSV.
"""
import csv, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap                      # noqa: E402
from phase1_heatmap import walls_overlay             # noqa: E402

GT = os.path.join(os.path.dirname(HERE), os.environ.get("FAIR_GT", "hospital_world/gt/hospital_gt"))   # FAIR_GT=hospital_world/unist/gt/unist_gt
ROBOTS = {"head": ("chain_robot (client)", "#d62728", "D"),
          "relay1": ("relay1", "#1f77b4", "o"), "relay2": ("relay2", "#2ca02c", "o")}


def main():
    path = sys.argv[1]
    log = sys.argv[2] if len(sys.argv) > 2 else None
    rows = list(csv.DictReader(open(path)))
    g = GridMap.load(GT + ".npy", GT + ".json")
    e = g.extent()
    wide = (e[1] - e[0]) > (e[3] - e[2])
    fig, ax = plt.subplots(figsize=(15, 6.2) if wide else (8, 13))
    ax.imshow(np.where(g.free, 1.0, np.nan), origin="lower", extent=g.extent(), cmap="Greys", vmin=0, vmax=4)
    walls_overlay(ax, g)
    stats = []
    for key, (label, col, mk) in ROBOTS.items():
        pts = np.array([(float(r[f"{key}_x"]), float(r[f"{key}_y"])) for r in rows
                        if r.get(f"{key}_x") not in (None, "")])
        if not len(pts):
            continue
        d = float(np.hypot(*np.diff(pts, axis=0).T).sum())
        stats.append(f"{label}: {d:.1f} m")
        ax.plot(pts[:, 0], pts[:, 1], "-", color=col, lw=2, alpha=0.85, label=f"{label} ({d:.1f} m)")
        ax.plot(*pts[0], mk, ms=9, mfc="white", mec=col, mew=2, zorder=6)
        ax.plot(*pts[-1], mk, ms=10, color=col, mec="k", zorder=7)
    gcs = tuple(float(v) for v in os.environ.get("FAIR_GCS", "0,14").split(","))   # FAIR_GCS=1.5,-6.1 (UNIST)
    ax.plot(*gcs, "*", ms=16, mfc="white", mec="k", zorder=8, label="GCS")
    # 0.00 m = two robots logged at the identical position: a logger glitch (one sample), not contact
    seps = [(float(r["min_sep_m"]), r) for r in rows if r.get("min_sep_m") not in ("", "nan", None)
            and float(r["min_sep_m"]) > 0.01]
    if seps:
        s, r = min(seps, key=lambda x: x[0])
        stats.append(f"closest robots {s:.2f} m ({r['closest_pair']} at {r['wall_time']})")
    if log and os.path.exists(log):
        for line in open(log):
            if "DEPLOY" not in line or "targets" not in line:
                continue
            for name, x, y in re.findall(r"(relay\d) \((-?[\d.]+), (-?[\d.]+)\)", line):
                ax.plot(float(x), float(y), "x", ms=11, mew=2.5, color=ROBOTS[name][1], zorder=9)
        ax.plot([], [], "kx", ms=10, mew=2.5, label="relay targets of DEPLOY decisions")
    rtf = [float(r["rtf"]) for r in rows if r.get("rtf")]
    last = rows[-1]
    ax.set_title(f"{os.path.basename(path)}  |  {rows[0]['wall_time']} - {last['wall_time']} wall, "
                 f"RTF {np.mean(rtf):.2f}\n" + "  |  ".join(stats) +
                 f"\ndecisions {last['decisions']}: client-only {last['client_only']}, deploy {last['deploy']}, "
                 f"skip {last['skip']}", fontsize=8.5)
    ax.legend(loc="lower left", fontsize=8)
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    fig.tight_layout()
    out = os.path.splitext(path)[0] + ".png"
    fig.savefig(out, dpi=90)
    print("wrote", out, "|", "  ".join(stats))


if __name__ == "__main__":
    main()
