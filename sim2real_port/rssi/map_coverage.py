#!/usr/bin/env python3
"""How much of the floor the client's own map covers, from the coordinator's /map snapshots.

    .venv/bin/python rssi/map_coverage.py rssi/out/live_runs/<run>_maps --gt hospital_world/unist/gt/unist_gt \\
        --start -0.5 -6 [--all]

Coverage = reachable floor cells (GT free, >= 0.45 m from walls, connected to the start: the
same set explore_sim.py scores) that the live map marks free. Prints the latest snapshot, or
every snapshot with --all (coverage over time).
"""
import argparse, glob, os, sys
import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap               # noqa: E402
from explore_sim import geodesic_distance     # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("maps"); ap.add_argument("--gt", required=True)
    ap.add_argument("--start", nargs=2, type=float, required=True, help="client spawn, world m")
    ap.add_argument("--clearance", type=float, default=0.45)
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    gt = a.gt if os.path.isabs(a.gt) else os.path.join(os.path.dirname(HERE), a.gt)
    g = GridMap.load(gt + ".npy", gt + ".json")
    reach = np.isfinite(geodesic_distance(g, g.world_to_cell(*a.start))) & \
        (distance_transform_edt(g.free) * g.res >= a.clearance)
    H, W = g.shape
    xs = g.origin[0] + (np.arange(W) + 0.5) * g.res
    ys = g.origin[1] + (np.arange(H) + 0.5) * g.res
    files = sorted(glob.glob(os.path.join(a.maps, "map_*.npz")))
    for f in (files if a.all else files[-1:]):
        z = np.load(f)
        G, res = z["grid"], float(z["res"])
        ox, oy = z["origin"] + z["off"]
        cc = np.floor((xs - ox) / res).astype(int); rr = np.floor((ys - oy) / res).astype(int)
        inside = ((rr >= 0) & (rr < G.shape[0]))[:, None] & ((cc >= 0) & (cc < G.shape[1]))[None, :]
        v = G[np.clip(rr, 0, G.shape[0] - 1)[:, None], np.clip(cc, 0, G.shape[1] - 1)[None, :]]
        known = inside & (v >= 0) & (v < 50)
        print(f"{os.path.basename(f)}  coverage {100 * (known & reach).sum() / reach.sum():5.1f}% of reachable floor")


if __name__ == "__main__":
    main()
