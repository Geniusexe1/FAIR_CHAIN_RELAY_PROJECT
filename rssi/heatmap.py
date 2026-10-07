#!/usr/bin/env python3
"""Place any nodes on the hospital map and render their RSSI coverage + links.

    .venv/bin/python rssi/heatmap.py --node GCS 0 12
    .venv/bin/python rssi/heatmap.py --node GCS 0 12 --node relay1 -6 -1 --node head -9 -28
    .venv/bin/python rssi/heatmap.py --node GCS 0 12 --node head -9 -28 --noise --n 3
    .venv/bin/python rssi/heatmap.py --node GCS 0 12 --per-node      # one map per node too

Positions are WORLD metres (Gazebo frame). The first node is treated as the GCS
(root for "can this node reach the GCS?"). Output: rssi/out/user/<name>.png and a
link table printed to the terminal.
"""
import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap, PropagationParams                     # noqa: E402
from network import Network, Node, NoiseParams, connectivity           # noqa: E402
from phase2_topology import draw, GT                                   # noqa: E402

KINDS = {"gcs": "gcs", "relay": "relay", "head": "head"}


def kind_of(name, first):
    if first:
        return "gcs"
    low = name.lower()
    return next((k for k in KINDS if low.startswith(k)), "relay")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", nargs=3, action="append", metavar=("NAME", "X", "Y"), required=True,
                    help="a node at world (X, Y). Repeat. First one is the GCS.")
    ap.add_argument("--P0", type=float, default=PropagationParams.P0, help="dBm at 1 m")
    ap.add_argument("--n", type=float, default=PropagationParams.n, help="path-loss exponent")
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha, help="detour weight")
    ap.add_argument("--threshold", type=float, default=-85.0, help="link usable at >= this, dBm")
    ap.add_argument("--noise", action="store_true", help="add shadowing to maps + fast fading to links")
    ap.add_argument("--sigma-shadow", type=float, default=NoiseParams.sigma_shadow)
    ap.add_argument("--corr", type=float, default=NoiseParams.corr_m)
    ap.add_argument("--sigma-fast", type=float, default=NoiseParams.sigma_fast)
    ap.add_argument("--seed", type=int, default=0, help="fast-fading seed")
    ap.add_argument("--floor", type=float, default=None, help="colour-scale floor, dBm (default P0-60)")
    ap.add_argument("--per-node", action="store_true", help="also save one map per node")
    ap.add_argument("--name", default="heatmap", help="output file name (no extension)")
    a = ap.parse_args()
    if a.floor is None:
        a.floor = a.P0 - 60.0

    grid = GridMap.load(GT + ".npy", GT + ".json")
    net = Network(grid, PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha),
                  NoiseParams(a.sigma_shadow, a.corr, a.sigma_fast, enabled=a.noise))

    for i, (name, x, y) in enumerate(a.node):
        xy = (float(x), float(y))
        r, c = grid.world_to_cell(*xy)
        if not grid.in_bounds(r, c):
            sys.exit(f"{name} at {xy} is outside the map "
                     f"(x {grid.extent()[0]:.1f}..{grid.extent()[1]:.1f}, "
                     f"y {grid.extent()[2]:.1f}..{grid.extent()[3]:.1f})")
        if grid.occ[r, c]:
            sys.exit(f"{name} at {xy} is inside a wall/obstacle - move it a little")
        net.nodes.append(Node(name, xy, kind_of(name, i == 0)))

    out = os.path.join(HERE, "out", "user"); os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(a.seed) if a.noise else None
    links = net.links(noisy=a.noise, rng=rng)
    gcs = net.nodes[0].name
    conn = connectivity(links, net.nodes, a.threshold, gcs)
    cov = net.coverage_map(noisy=a.noise)
    norm = Normalize(a.floor, a.P0)

    print(f"P0 {a.P0} dBm | n {a.n} | alpha {a.alpha} | threshold {a.threshold} dBm | "
          f"noise {'ON' if a.noise else 'off'}")
    if links:
        print(f"\n  {'link':22} {'RSSI':>7} {'dist':>6} {'detour':>7}  state")
        for L in links:
            up = np.isfinite(L["rssi"]) and L["rssi"] >= a.threshold
            print(f"  {L['a']+' - '+L['b']:22} {L['rssi']:7.1f} {L['dE']:5.1f}m {L['dbar']:6.1f}m  "
                  f"{'UP' if up else 'down'}{'' if L['los'] else '  (no line of sight)'}")
        print(f"\n  route to {gcs}:")
        for n in net.nodes[1:]:
            h = conn.get(n.name)
            print(f"    {n.name:10} " + (f"{' > '.join(reversed(h[1]))}   weakest hop {h[0]:.1f} dBm"
                                         if h else "NOT CONNECTED"))

    fig, ax = plt.subplots(figsize=(7.5, 13))
    im = draw(ax, net, grid, links, conn, a.threshold, cov,
              f"P0 {a.P0} dBm, n {a.n}, alpha {a.alpha}, noise {'on' if a.noise else 'off'}\n"
              f"colour = best RSSI from any node  |  links green > -65, yellow, red dashed < {a.threshold:g}", norm)
    fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02).set_label("RSSI (dBm)")
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    fig.tight_layout()
    path = os.path.join(out, a.name + ".png"); fig.savefig(path, dpi=100); plt.close(fig)
    print(f"\nwrote {os.path.relpath(path, os.path.dirname(HERE))}")

    if a.per_node:
        for n in net.nodes:
            P = net.rssi_map(n, noisy=a.noise)
            show = np.where(np.isfinite(P), P, np.where(grid.free, a.floor, np.nan))
            fig, ax = plt.subplots(figsize=(7.5, 13))
            draw(ax, Network(grid, net.prop, net.noise, [n]), grid, [], {}, a.threshold, show, n.name, norm)
            fig.tight_layout()
            p = os.path.join(out, f"{a.name}_{n.name}.png"); fig.savefig(p, dpi=100); plt.close(fig)
            print(f"      {os.path.relpath(p, os.path.dirname(HERE))}")


if __name__ == "__main__":
    main()
