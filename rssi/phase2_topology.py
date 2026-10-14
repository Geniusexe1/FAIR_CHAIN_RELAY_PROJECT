#!/usr/bin/env python3
"""Phase 2 -- dynamic GCS + robot topology with injected noise, link visualisation.

    .venv/bin/python rssi/phase2_topology.py

Scenario: the head robot drives from beside the GCS to a far ward along the
shortest free-space route; relay1 and relay2 trail along the same route at 2/3
and 1/3 of the head's progress. Every frame recomputes the noisy links and the
best route from each robot back to the GCS.

Writes rssi/out/phase2_noise_check.png   noise statistics vs what was injected
       rssi/out/phase2_frame_XX.png      snapshots: client-to-GCS connectivity margin
                                         (head attached to relay1) + zero contour + links
       rssi/out/phase2_links.png         every link's RSSI over the run
       rssi/out/phase2_topology.gif      the whole run
"""
import argparse, os, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.colors import Normalize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap, PropagationParams, field_from          # noqa: E402
from network import Network, Node, NoiseParams, connectivity            # noqa: E402
from phase1_heatmap import walls_overlay                                # noqa: E402
from margin import margin_map, draw_margin, margin_checks, summary_line, CBAR_LABEL  # noqa: E402

ROOT = os.path.dirname(HERE)
GT = os.path.join(ROOT, "hospital_world", "gt", "hospital_gt")
OUT = os.path.join(HERE, "out")
COL = {"gcs": "white", "relay": "#1f77b4", "head": "#d62728"}
MARK = {"gcs": "*", "relay": "o", "head": "D"}


def route(grid, start, goal, spacing=0.25):
    """Shortest free-space route start->goal: descend goal's fast-marching field,
    then resample at even arc-length spacing."""
    d = field_from(grid, goal).dS_fmm_raw
    r, c = grid.world_to_cell(*start)
    gr = grid.world_to_cell(*goal)
    pts = [(r, c)]
    nb = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    for _ in range(grid.shape[0] * grid.shape[1]):
        if (r, c) == gr:
            break
        cand = [(d[r + dr, c + dc], r + dr, c + dc) for dr, dc in nb
                if grid.in_bounds(r + dr, c + dc) and np.isfinite(d[r + dr, c + dc])]
        best = min(cand)
        if best[0] >= d[r, c]:
            break
        r, c = best[1], best[2]; pts.append((r, c))
    xy = np.array([grid.cell_to_world(rr, cc) for rr, cc in pts])
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    s = np.arange(0, seg[-1], spacing)
    return np.c_[np.interp(s, seg, xy[:, 0]), np.interp(s, seg, xy[:, 1])], seg[-1]


GOOD_DBM = -65.0   # paper Fig. 9 legend: RSSI > -65 green, -65..-85 yellow, <= -85 red


def link_colour(rssi, thr):
    if not np.isfinite(rssi) or rssi < thr:
        return "#d62728"
    return "#2ca02c" if rssi > GOOD_DBM else "#e6b800"


def draw(ax, net, grid, links, conn, thr, cov, title, norm):
    show = np.where(np.isfinite(cov), cov, np.where(grid.free, norm.vmin, np.nan))  # sealed -> floor colour
    im = ax.imshow(show, origin="lower", extent=grid.extent(),
                   cmap="turbo", norm=norm, interpolation="nearest", alpha=0.85)
    walls_overlay(ax, grid)
    for L in links:
        a, b = net.node(L["a"]), net.node(L["b"])
        up = np.isfinite(L["rssi"]) and L["rssi"] >= thr
        xs, ys = [a.xy[0], b.xy[0]], [a.xy[1], b.xy[1]]
        ax.plot(xs, ys, color=link_colour(L["rssi"], thr), lw=2.8 if up else 1.4,
                ls="-" if up else (0, (3, 3)), zorder=6)
        mx, my = np.mean(xs), np.mean(ys)
        txt = f"{L['rssi']:.0f}" + ("" if L["los"] else "*")
        ax.text(mx, my, txt, fontsize=7.5, ha="center", va="center", zorder=8,
                color="black" if up else "0.3",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85 if up else 0.55))
    for n in net.nodes:
        ok = n.kind == "gcs" or n.name in conn
        ax.plot(*n.xy, marker=MARK[n.kind], ms=14 if n.kind == "gcs" else 9,
                mfc=COL[n.kind], mec="black" if ok else "red", mew=1.3 if ok else 2.5, zorder=9)
        ax.annotate(n.name, n.xy, xytext=(7, -3), textcoords="offset points", fontsize=8,
                    fontweight="bold", zorder=10, color="black" if ok else "red",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    ax.set_title(title, fontsize=9)
    return im


def noise_check(net, gcs, res_ok):
    clean = net.rssi_map(gcs, noisy=False)
    noisy = net.rssi_map(gcs, noisy=True)
    m = np.isfinite(clean) & np.isfinite(noisy)
    with np.errstate(invalid="ignore"):
        diff = noisy - clean
    resid = diff[m]
    S = net._shadow_of(gcs)
    lag = int(round(net.noise.corr_m / net.grid.res))
    a, b = S[:, :-lag].ravel(), S[:, lag:].ravel()
    rho = np.corrcoef(a, b)[0, 1]

    rng = np.random.default_rng(0)
    r1, r2 = net.nodes[0], net.nodes[-1]
    reads = [net.link(r1, r2, True, rng) for _ in range(4000)]
    samples = np.array([L["rssi_ab"] for L in reads])          # one direction, one reading
    fast_sd = samples.std()
    L12, L21 = net.link(r1, r2, True), net.link(r2, r1, True)
    sym = abs(L12["clean"] - L21["clean"])                     # the path-loss part is reciprocal
    weaker = all(L["rssi"] == min(L["rssi_ab"], L["rssi_ba"]) for L in reads[:200] + [L12, L21])

    print("\nnoise checks")
    def chk(name, ok, det):
        res_ok.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {det}")
    chk("shadowing residual is zero-mean", abs(resid.mean()) < 0.5, f"mean {resid.mean():+.3f} dB")
    chk("shadowing std matches sigma_shadow", abs(resid.std() - net.noise.sigma_shadow) < 0.15 * net.noise.sigma_shadow,
        f"{resid.std():.2f} dB vs {net.noise.sigma_shadow}")
    chk("shadowing correlation at corr_m ~ 1/e", 0.25 < rho < 0.5,
        f"rho({net.noise.corr_m} m) = {rho:.3f}  (target 0.368)")
    chk("fast fading std matches sigma_fast", abs(fast_sd - net.noise.sigma_fast) < 0.1 * net.noise.sigma_fast,
        f"{fast_sd:.2f} dB vs {net.noise.sigma_fast} over 4000 reads")
    chk("path loss is reciprocal (clean a->b == b->a)", sym < 1e-9, f"|diff| = {sym:.1e} dB")
    chk("a link's value is its weaker direction", weaker,
        f"{r1.name}->{r2.name} {L12['rssi_ab']:.1f} / {r2.name}->{r1.name} {L12['rssi_ba']:.1f} -> {L12['rssi']:.1f} dBm")

    fig, axs = plt.subplots(1, 4, figsize=(20, 10), gridspec_kw=dict(width_ratios=[1, 1, 1, 1.1]))
    norm = Normalize(net.prop.P0 - 60, net.prop.P0)
    g = net.grid
    for ax, img, t in [(axs[0], clean, "GCS clean (Phase 1 model)"),
                       (axs[1], noisy, f"GCS + shadowing (sigma {net.noise.sigma_shadow} dB, {net.noise.corr_m} m)")]:
        ax.imshow(img, origin="lower", extent=g.extent(), cmap="turbo", norm=norm, interpolation="nearest")
        walls_overlay(ax, g); ax.plot(*gcs.xy, "*", ms=14, mfc="white", mec="k")
        ax.set_title(t, fontsize=10); ax.set_xticks([]); ax.set_yticks([])
    im = axs[2].imshow(np.where(m, diff, np.nan), origin="lower", extent=g.extent(),
                       cmap="RdBu_r", vmin=-12, vmax=12, interpolation="nearest")
    walls_overlay(axs[2], g); axs[2].set_title("injected noise (dB) - spatially correlated", fontsize=10)
    axs[2].set_xticks([]); axs[2].set_yticks([]); fig.colorbar(im, ax=axs[2], fraction=0.04)
    ax = axs[3]
    xs = np.linspace(-16, 16, 200)
    ax.hist(resid, bins=80, density=True, alpha=0.6, label="shadowing residual")
    ax.plot(xs, np.exp(-xs**2 / (2 * net.noise.sigma_shadow**2)) / (net.noise.sigma_shadow * np.sqrt(2*np.pi)),
            "k-", label=f"N(0, {net.noise.sigma_shadow}²) target")
    ax.hist(samples - samples.mean(), bins=60, density=True, alpha=0.5, label="fast fading, one link")
    ax.plot(xs, np.exp(-xs**2 / (2 * net.noise.sigma_fast**2)) / (net.noise.sigma_fast * np.sqrt(2*np.pi)),
            "k--", label=f"N(0, {net.noise.sigma_fast}²) target")
    ax.set_xlabel("dB"); ax.legend(fontsize=8); ax.set_title("noise distributions vs injected", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "phase2_noise_check.png"), dpi=85); plt.close(fig)


def main():
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcs", nargs=2, type=float, default=[0.0, 12.0])
    ap.add_argument("--goal", nargs=2, type=float, default=[-9.0, -29.0],
                    help="where the head robot drives to (world m)")
    ap.add_argument("--threshold", type=float, default=-85.0, help="link usable at >= this, dBm")
    ap.add_argument("--frames", type=int, default=36)
    ap.add_argument("--sigma-shadow", type=float, default=NoiseParams.sigma_shadow)
    ap.add_argument("--corr", type=float, default=NoiseParams.corr_m)
    ap.add_argument("--sigma-fast", type=float, default=NoiseParams.sigma_fast)
    ap.add_argument("--P0", type=float, default=PropagationParams.P0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--tag", default="", help="suffix for output files")
    ap.add_argument("--no-gif", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    grid = GridMap.load(GT + ".npy", GT + ".json")
    if a.tag:
        OUT = os.path.join(HERE, "out", a.tag)
    os.makedirs(OUT, exist_ok=True)
    net = Network(grid, PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha),
                  NoiseParams(a.sigma_shadow, a.corr, a.sigma_fast))
    start = (a.gcs[0], a.gcs[1] - 1.0)
    path, length = route(grid, start, tuple(a.goal))
    print(f"P0 {a.P0} n {a.n} alpha {a.alpha} | head route {length:.1f} m, {len(path)} samples | threshold {a.threshold} dBm | "
          f"noise shadow {a.sigma_shadow} dB/{a.corr} m, fast {a.sigma_fast} dB")

    gcs = Node("GCS", tuple(a.gcs), "gcs")
    head = Node("head", tuple(path[0]), "head")
    r1 = Node("relay1", tuple(path[0]), "relay")
    r2 = Node("relay2", tuple(path[0]), "relay")
    net.nodes = [gcs, r2, r1, head]

    results = []
    # Phase-1 consistency: noise off, GCS alone == the Phase 1 model
    from propagation import rssi_map as p1_map
    p1, _ = p1_map(grid, gcs.xy, net.prop)
    net_clean = net.rssi_map(gcs, noisy=False)
    same = np.allclose(np.nan_to_num(p1, nan=1e9, neginf=-1e9), np.nan_to_num(net_clean, nan=1e9, neginf=-1e9))
    print(f"\n  [{'PASS' if same else 'FAIL'}] noise off reproduces Phase 1 exactly"); results.append(same)
    noise_check(net, gcs, results)

    chain = [n.name for n in net.nodes]                 # GCS > relay2 > relay1 > head, fixed
    head.xy, r1.xy, r2.xy = tuple(path[len(path) // 2]), tuple(path[len(path) // 3]), tuple(path[len(path) // 6])
    print("\nmargin checks (head at mid-route)")
    def rep(name, ok, det):
        results.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {det}")
    margin_checks(net, chain, a.threshold, rep)

    rng = np.random.default_rng(42)
    idx = np.linspace(0, len(path) - 1, a.frames).round().astype(int)
    frames, history = [], []
    t0 = time.time()
    for k, i in enumerate(idx):
        head.xy = tuple(path[i])
        r1.xy = tuple(path[int(i * 2 / 3)])
        r2.xy = tuple(path[int(i * 1 / 3)])
        links = net.links(noisy=True, rng=rng)
        conn = connectivity(links, net.nodes, a.threshold, "GCS")
        est = {(L["a"], L["b"]): L["rssi"] for L in links}      # this frame's link estimates
        cov = margin_map(net, "head", "relay1", chain, a.threshold, estimates=est)
        frames.append((k, [dict(L) for L in links], conn, cov,
                       {n.name: n.xy for n in net.nodes}, i * 0.25))
        history.append({f"{L['a']}-{L['b']}": L["rssi"] for L in links})
    print(f"\n{a.frames} frames in {time.time()-t0:.1f} s")

    # ---- link table for the last frame + per-frame connectivity summary
    print(f"\nfinal frame links (threshold {a.threshold} dBm, * = no line of sight)")
    print(f"  {'link':16} {'RSSI':>7} {'clean':>7} {'dE':>6} {'detour':>7}  state")
    for L in frames[-1][1]:
        up = L["rssi"] >= a.threshold
        print(f"  {L['a']+'-'+L['b']:16} {L['rssi']:7.1f} {L['clean']:7.1f} {L['dE']:6.1f} {L['dbar']:7.1f}  "
              f"{'UP  ' if up else 'down'}{'' if L['los'] else ' *'}")
    print("\nroute back to GCS per frame (bottleneck RSSI):")
    for k, links, conn, res, pos, s in frames[:: max(1, a.frames // 9)] + [frames[-1]]:
        h = conn.get("head")
        print(f"  frame {k:2d}  head at {s:5.1f} m  ->  " +
              (f"{' > '.join(reversed(h[1]))}   (weakest hop {h[0]:.1f} dBm)" if h else "HEAD DISCONNECTED"))
        for n in net.nodes: n.xy = pos[n.name]
        print(f"             {summary_line(res, net)}")

    # ---- per-link verification against the model, last frame
    L = frames[-1][1][0]
    a_, b_ = net.node(L["a"]), net.node(L["b"])
    recompute = net.link(a_, b_, noisy=False)["rssi"]
    ok = abs(recompute - L["clean"]) < 1e-9
    print(f"\n  [{'PASS' if ok else 'FAIL'}] link 'clean' value equals a fresh eq.(1) evaluation"); results.append(ok)

    # ---- plots
    norm = Normalize(net.prop.P0 - 60, net.prop.P0)
    for k in range(len(frames)):                       # every frame as its own PNG
        _, links, conn, res, pos, s = frames[k]
        for n in net.nodes: n.xy = pos[n.name]
        fig, ax = plt.subplots(figsize=(7.5, 13))
        im = draw_margin(ax, net, res,
                         f"frame {k}/{a.frames-1}  -  head {s:.1f} m along route, attached to relay1 (fixed)\n"
                         f"chain GCS > relay2 > relay1 > head  |  solid = zero margin, dashed = cutoff\n"
                         f"links: green > -65, yellow -65..{a.threshold:g}, red dashed < {a.threshold:g} dBm (paper Fig. 9)")
        fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02).set_label(CBAR_LABEL)
        ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, f"phase2_frame_{k:02d}.png"), dpi=100); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5))
    s = np.array([f[5] for f in frames])
    for key in history[0]:
        ax.plot(s, [h[key] for h in history], marker=".", label=key)
    ax.axhline(a.threshold, color="k", ls="--", lw=1, label=f"threshold {a.threshold:g} dBm")
    ax.set_xlabel("head distance along route (m)"); ax.set_ylabel("RSSI (dBm, noisy)")
    ax.set_title("every link over the run"); ax.grid(alpha=0.3); ax.legend(fontsize=8, ncol=2)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "phase2_links.png"), dpi=100); plt.close(fig)

    if not a.no_gif:
        fig, ax = plt.subplots(figsize=(5.2, 9))
        def upd(j):
            ax.clear()
            _, links, conn, res, pos, s = frames[j]
            for n in net.nodes: n.xy = pos[n.name]
            draw_margin(ax, net, res, f"connectivity margin - head {s:.1f} m along route")
            ax.set_xticks([]); ax.set_yticks([])
        anim = animation.FuncAnimation(fig, upd, frames=len(frames))
        anim.save(os.path.join(OUT, "phase2_topology.gif"), writer=animation.PillowWriter(fps=3), dpi=70)
        plt.close(fig)

    print(f"\nwrote {os.path.relpath(OUT, ROOT)}/phase2_noise_check.png, phase2_frame_*.png, phase2_links.png"
          + ("" if a.no_gif else ", phase2_topology.gif"))
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
