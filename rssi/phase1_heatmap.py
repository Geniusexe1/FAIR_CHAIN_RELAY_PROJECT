#!/usr/bin/env python3
"""Phase 1 -- RSSI heatmap for a single GCS on the ground-truth map, plus checks.

    .venv/bin/python rssi/phase1_heatmap.py --gcs 0 12
    .venv/bin/python rssi/phase1_heatmap.py --gcs 0 12 --P0 -40 --n 2.0 --alpha 20

Writes rssi/out/phase1_<tag>_heatmap.png   (the deliverable)
       rssi/out/phase1_<tag>_diagnostics.png (dE, detour, LOS, raw-FMM error)
       rssi/out/phase1_<tag>_rssi.npy        (dBm per cell, NaN in obstacles)
and prints PASS/FAIL for each property the model must satisfy.
"""
import argparse, os, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap, PropagationParams, rssi_map, rssi_from_distances  # noqa: E402

ROOT = os.path.dirname(HERE)
GT = os.path.join(ROOT, "hospital_world", "gt", "hospital_gt")


def walls_overlay(ax, grid):
    cmap = ListedColormap([(0, 0, 0, 0), (0.08, 0.08, 0.1, 1.0)])
    ax.imshow(grid.occ, origin="lower", extent=grid.extent(), cmap=cmap,
              interpolation="nearest", zorder=3)


def mark(ax, xy, label="GCS"):
    ax.plot(*xy, marker="*", ms=16, mfc="white", mec="black", mew=1.2, zorder=5)
    ax.annotate(label, xy, xytext=(6, 6), textcoords="offset points", fontsize=9,
                fontweight="bold", color="black", zorder=6,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcs", nargs=2, type=float, default=[0.0, 12.0], metavar=("X", "Y"),
                    help="GCS position, WORLD metres (Gazebo frame)")
    ap.add_argument("--P0", type=float, default=PropagationParams.P0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--floor", type=float, default=None, help="colour-scale floor, dBm (default P0-60)")
    ap.add_argument("--tag", default=None)
    a = ap.parse_args()
    if a.floor is None:
        a.floor = a.P0 - 60.0

    p = PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha)
    grid = GridMap.load(GT + ".npy", GT + ".json")
    tag = a.tag or f"gcs_{a.gcs[0]:g}_{a.gcs[1]:g}".replace("-", "m")
    out = os.path.join(HERE, "out"); os.makedirs(out, exist_ok=True)

    t0 = time.time()
    P, f = rssi_map(grid, tuple(a.gcs), p)
    dt = time.time() - t0
    np.save(os.path.join(out, f"phase1_{tag}_rssi.npy"), P)

    free = grid.free
    reach = free & np.isfinite(f.dS)
    print(f"map {grid.shape[1]}x{grid.shape[0]} cells @ {grid.res} m | GCS world {tuple(a.gcs)} "
          f"-> cell {f.src_rc} | P0={p.P0} n={p.n} alpha={p.alpha} | computed in {dt:.1f} s")
    print(f"free cells {free.sum()} | reachable {reach.sum()} | LOS {f.los.sum()} "
          f"({100*f.los.sum()/max(reach.sum(),1):.1f}% of reachable) | "
          f"unreachable free {(free & ~np.isfinite(f.dS)).sum()}")
    finite = np.isfinite(P)
    print(f"RSSI range {np.nanmin(np.where(finite,P,np.nan)):.1f} .. {np.nanmax(P):.1f} dBm")

    # ------------------------------------------------------------------ checks
    results = []
    def check(name, ok, detail):
        results.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    print("\nmodel checks")
    # 1. reference point: in LOS at ~1 m the model must give P0
    ring = reach & f.los & (np.abs(f.dE - 1.0) < 0.05)
    check("RSSI at 1 m in LOS equals P0", ring.any() and abs(np.nanmean(P[ring]) - p.P0) < 0.5,
          f"{np.nanmean(P[ring]):.2f} dBm vs P0 {p.P0}" if ring.any() else "no 1 m LOS ring")

    # 2. LOS => no detour, exactly
    check("dbar == 0 wherever there is line of sight", np.all(f.dbar[f.los & reach] == 0),
          f"max dbar in LOS = {f.dbar[f.los & reach].max():.4f} m")

    # 3. a detour is never shorter than the straight line
    fin = reach & np.isfinite(f.dS)
    check("dS >= dE everywhere", np.all(f.dS[fin] >= f.dE[fin] - 1e-9),
          f"min(dS-dE) = {(f.dS[fin]-f.dE[fin]).min():.4f} m")

    # 4. in LOS the model collapses to free-space path loss
    fs = p.P0 - 10 * p.n * np.log10(np.maximum(f.dE, p.d_min))
    m = reach & f.los
    check("LOS RSSI equals free-space path loss", np.allclose(P[m], fs[m]),
          f"max |P - Pfs| = {np.abs(P[m]-fs[m]).max():.2e} dB")

    # 5. walls must cost signal: same distance band, NLOS weaker than LOS
    band = reach & (f.dE > 6) & (f.dE < 8)
    los_b, nlos_b = band & f.los, band & ~f.los
    if los_b.any() and nlos_b.any():
        d = np.nanmean(P[los_b]) - np.nanmean(P[nlos_b])
        check("NLOS weaker than LOS at the same distance (6-8 m)", d > 0,
              f"LOS {np.nanmean(P[los_b]):.1f} vs NLOS {np.nanmean(P[nlos_b]):.1f} dBm  (walls cost {d:.1f} dB)")
    else:
        check("NLOS weaker than LOS at the same distance (6-8 m)", False, "band has no LOS or no NLOS cells")

    # 6. more detour => more loss (monotone in dbar at fixed dE)
    dvals = np.array([0.0, 1.0, 5.0, 20.0])
    pv = rssi_from_distances(np.full(4, 10.0), dvals, p)
    check("RSSI strictly decreases with detour at fixed dE", np.all(np.diff(pv) < 0),
          "  ".join(f"dbar={d:g}m:{v:.1f}" for d, v in zip(dvals, pv)))

    # diagnostic (not a pass/fail): why the LOS override exists
    raw = f.dS_fmm_raw
    mm = m & np.isfinite(raw) & (f.dE > 2)
    rel = (raw[mm] - f.dE[mm]) / f.dE[mm]
    print(f"\n  note: raw fast-marching dS in LOS cells overshoots dE by median {100*np.median(rel):.2f}%, "
          f"p99 {100*np.percentile(rel,99):.2f}% -- without the LOS override that would read as "
          f"a fake detour of up to {np.percentile((raw[mm]-f.dE[mm]),99):.2f} m")

    # ------------------------------------------------------------------- plots
    vmax = p.P0
    show = np.where(free & ~np.isfinite(P), a.floor, P)       # unreachable -> floor colour
    fig, ax = plt.subplots(figsize=(7.5, 13))
    im = ax.imshow(show, origin="lower", extent=grid.extent(), cmap="turbo",
                   vmin=a.floor, vmax=vmax, interpolation="nearest")
    walls_overlay(ax, grid); mark(ax, a.gcs)
    cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02); cb.set_label("RSSI (dBm)")
    ax.set_title(f"RSSI from GCS at world ({a.gcs[0]:g}, {a.gcs[1]:g})\n"
                 f"P0={p.P0} dBm, n={p.n}, alpha={p.alpha}", fontsize=11)
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    fig.tight_layout()
    hp = os.path.join(out, f"phase1_{tag}_heatmap.png"); fig.savefig(hp, dpi=110); plt.close(fig)

    fig, axs = plt.subplots(1, 4, figsize=(20, 11))
    panels = [
        (np.where(reach, f.dE, np.nan), "viridis", "dE  straight-line (m)", None),
        (np.where(reach, f.dbar, np.nan), "magma", "dbar = |dE - dS|  detour (m)", None),
        (np.where(free, f.los.astype(float), np.nan), "Greys_r", "line of sight (white = visible)", (0, 1)),
        (np.where(mm, raw - f.dE, np.nan), "inferno",
         "raw FMM error in LOS (m)\n(what the LOS override removes)", None),
    ]
    for ax, (img, cm, title, lim) in zip(axs, panels):
        kw = dict(vmin=lim[0], vmax=lim[1]) if lim else {}
        im = ax.imshow(img, origin="lower", extent=grid.extent(), cmap=cm, interpolation="nearest", **kw)
        walls_overlay(ax, grid); mark(ax, a.gcs, "")
        ax.set_title(title, fontsize=10); ax.set_xticks([]); ax.set_yticks([])
        fig.colorbar(im, ax=ax, fraction=0.04, pad=0.01)
    fig.tight_layout()
    dp = os.path.join(out, f"phase1_{tag}_diagnostics.png"); fig.savefig(dp, dpi=90); plt.close(fig)

    print(f"\nwrote {os.path.relpath(hp, ROOT)}\n      {os.path.relpath(dp, ROOT)}")
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
