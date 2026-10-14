"""Client-to-GCS connectivity margin map.

For a FIXED relay topology, where could the client be and still reach the GCS?
GCS and relays stay where they are; the client is placed virtually at every
cell x, keeping its CURRENT relay association (no switching to whichever node
happens to be strongest there):

    margin(x) = min( RSSI(client@x <-> attached relay),
                     weakest link on the attached relay's current path to the GCS )
                - eta

Every link uses its WEAKER direction (bidirectional connectivity).
margin >= 0  : the client at x would have an end-to-end path above threshold.
The zero contour is the predicted boundary of the connected region.

This is a score for the multi-hop path, not a measured RSSI -- a strong link to
the relay does not help if the relay itself has a weak or broken upstream path.
"""
from __future__ import annotations

import numpy as np
from matplotlib.colors import ListedColormap, TwoSlopeNorm
from matplotlib.lines import Line2D
from scipy.ndimage import distance_transform_edt

from propagation import rssi_from_distances

CBAR_LABEL = "Client-to-GCS connectivity margin (dB)"
CUTOFF_DB = -18.0      # second contour: the "strong enough" boundary, as a margin level (dB)


def upstream_path(chain: list[str], attach: str) -> list[str]:
    """Path from the attached relay back to the GCS along the chain (chain[0] = GCS)."""
    k = chain.index(attach)
    return list(reversed(chain[:k + 1]))


def _hop(net, nodes, u, v, estimates, noisy):
    """Current estimate of link u-v (weaker direction): a supplied estimate if there
    is one (measured / filtered), else the model's expectation."""
    if estimates:
        for k in ((u, v), (v, u)):
            if k in estimates:
                return float(estimates[k])
    return net.link(nodes[u], nodes[v], noisy=noisy, rng=None)["rssi"]


def margin_map(net, client: str, attach: str, chain: list[str], threshold: float,
               estimates: dict | None = None, noisy: bool = True) -> dict:
    """Margin (dB) at every cell, plus the pieces it was built from.

    The access link (virtual client at x) is always the model's EXPECTATION
    (shadowing on, no fast fading): a map is a prediction surface. Upstream hops
    and the actual client link use `estimates` {(a, b): dBm} when given, so the
    map moves when the link estimates move.
    """
    grid, nodes = net.grid, {n.name: n for n in net.nodes}
    a, c = nodes[attach], nodes[client]
    fa = net._field(a)
    clean = rssi_from_distances(fa.dE, fa.dbar, net.prop)          # reciprocal part, all cells
    relay_to_x = clean.copy()
    x_to_relay = clean.copy()
    if noisy and net.noise.enabled:
        ra, ca = grid.world_to_cell(*a.xy)
        relay_to_x = clean + net._shadow_of(a)                       # a's field, evaluated at x
        x_to_relay = clean + net._shadow_of(c)[ra, ca]               # client's field at the relay
    access = np.minimum(relay_to_x, x_to_relay)                      # weaker direction

    path = upstream_path(chain, attach)
    hops = []
    for u, v in zip(path[:-1], path[1:]):
        hops.append((u, v, _hop(net, nodes, u, v, estimates, noisy)))
    upstream = min((h[2] for h in hops), default=np.inf)            # attached to the GCS itself -> no hop
    weakest = min(hops, key=lambda h: h[2]) if hops else None

    with np.errstate(invalid="ignore"):
        m = np.minimum(access, upstream) - threshold
    m = np.where(grid.occ.astype(bool), np.nan, m)
    client_link = (attach, client, _hop(net, nodes, attach, client, estimates, noisy))
    return dict(margin=m, access=access, upstream=upstream, hops=hops, weakest=weakest,
                client_link=client_link, path=path, attach=attach, client=client,
                threshold=threshold)


def _contour_field(m, grid):
    """Walls take their nearest reachable cell's value -- only so contours run along
    the region edge instead of outlining every wall. Sealed cells (no propagation
    path) are masked, so no contour is drawn inside them."""
    hole = grid.occ.astype(bool) | ~np.isfinite(m)
    idx = distance_transform_edt(hole, return_distances=False, return_indices=True)
    sealed = grid.free & ~np.isfinite(m)
    return np.ma.masked_array(m[tuple(idx)], mask=sealed)


def draw_margin(ax, net, res: dict, title: str = "", vmin: float = -30.0, vmax: float = 10.0,
                cutoff: float | None = CUTOFF_DB):
    """Margin heatmap + zero contour + cutoff contour + actual client + current chain links.

    The scale is diverging about 0 but asymmetric: the margin can never exceed
    (weakest upstream hop - threshold), so the positive side is short.
    Cells with no propagation path at all (sealed rooms) are grey, not red."""
    grid = net.grid
    m = res["margin"]
    sealed = grid.free & ~np.isfinite(m)
    norm = TwoSlopeNorm(vmin=vmin, vcenter=0.0, vmax=vmax)
    im = ax.imshow(np.where(np.isfinite(m), m, np.nan), origin="lower", extent=grid.extent(),
                   cmap="RdYlBu", norm=norm, interpolation="nearest")
    ax.imshow(np.where(sealed, 1.0, np.nan), origin="lower", extent=grid.extent(),
              cmap=ListedColormap(["#b8b8b8"]), interpolation="nearest", zorder=2)
    # zero-margin contour = predicted boundary of the connected region
    H, W = grid.shape
    xs = grid.origin[0] + (np.arange(W) + 0.5) * grid.res
    ys = grid.origin[1] + (np.arange(H) + 0.5) * grid.res
    show = _contour_field(m, grid)
    legend = []
    for lvl, ls, lab in [(0.0, "-", "zero margin (connected boundary)"),
                         (cutoff, (0, (5, 3)), f"margin {cutoff:+g} dB cutoff" if cutoff is not None else "")]:
        if lvl is None:
            continue
        if show.min() < lvl < show.max():           # skip levels the map never crosses
            ax.contour(xs, ys, show, levels=[lvl], colors="black", linewidths=1.6,
                       linestyles=[ls], zorder=4)
        legend.append(Line2D([], [], color="black", lw=1.6, ls=ls, label=lab))
    ax.legend(handles=legend, loc="lower left", fontsize=7.5, framealpha=0.9).set_zorder(11)
    walls = ListedColormap([(0, 0, 0, 0), (0.08, 0.08, 0.1, 1.0)])
    ax.imshow(grid.occ, origin="lower", extent=grid.extent(), cmap=walls,
              interpolation="nearest", zorder=3)

    nodes = {n.name: n for n in net.nodes}
    thr = res["threshold"]
    # current communication links: the chain hops back to the GCS + client<->attached relay
    for u, v, r in res["hops"] + [res["client_link"]]:
        a, b = nodes[u], nodes[v]
        up = np.isfinite(r) and r >= thr
        col = "#2ca02c" if up and r > -65 else ("#e6b800" if up else "#d62728")
        ax.plot([a.xy[0], b.xy[0]], [a.xy[1], b.xy[1]], color=col, lw=3.0 if up else 1.6,
                ls="-" if up else (0, (3, 3)), zorder=6)
        ax.text((a.xy[0] + b.xy[0]) / 2, (a.xy[1] + b.xy[1]) / 2, f"{r:.0f}", fontsize=8,
                ha="center", va="center", zorder=8,
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
    style = {"gcs": ("*", 15, "white"), "relay": ("o", 9, "#1f77b4"), "head": ("D", 10, "#d62728")}
    for n in net.nodes:
        mk, ms, fc = style.get(n.kind, ("o", 9, "grey"))
        ax.plot(*n.xy, marker=mk, ms=ms, mfc=fc, mec="black", mew=1.3, zorder=9)
        lab = n.name + (" (client)" if n.name == res["client"] else "")
        ax.annotate(lab, n.xy, xytext=(7, -3), textcoords="offset points", fontsize=8,
                    fontweight="bold", zorder=10,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    if title:
        ax.set_title(title, fontsize=9)
    return im


def summary_line(res: dict, net, cutoff: float | None = CUTOFF_DB) -> str:
    c = res["client"]
    nodes = {n.name: n for n in net.nodes}
    r, cc = net.grid.world_to_cell(*nodes[c].xy)
    here = res["margin"][r, cc]
    w = res["weakest"]
    up = (f"upstream {' > '.join(res['path'])} weakest {w[0]}-{w[1]} {w[2]:.1f} dBm"
          if w else "attached directly to GCS")
    pct = np.nanmean(res["margin"] >= 0) * 100
    cut = "" if cutoff is None else f", >= {cutoff:+g} dB {np.nanmean(res['margin'] >= cutoff) * 100:.1f}%"
    return (f"client {c} on {res['attach']} | margin at client {here:+.1f} dB | {up} | "
            f"floor area: margin >= 0 {pct:.1f}%{cut}")


def margin_checks(net, chain: list[str], threshold: float, report) -> None:
    """Properties the margin map must satisfy. report(name, ok, detail)."""
    import copy
    nodes = {n.name: n for n in net.nodes}
    client, attach = chain[-1], chain[-2]
    res = margin_map(net, client, attach, chain, threshold)
    m, acc = res["margin"], res["access"]
    fin = np.isfinite(m)

    # 1. never better than the access link to the attached relay
    report("margin <= client<->attached-relay link - threshold", np.all(m[fin] <= acc[fin] - threshold + 1e-9),
           f"max excess {np.nanmax(m[fin] - (acc[fin] - threshold)):.2e} dB")
    # 2. never better than the weakest upstream hop
    cap = res["upstream"] - threshold
    report("margin <= weakest upstream hop - threshold", np.all(m[fin] <= cap + 1e-9),
           f"max margin {np.nanmax(m):.1f} dB, upstream cap {cap:.1f} dB ({res['weakest'][0]}-{res['weakest'][1]})"
           if res["weakest"] else "attached to the GCS")
    # 3. access uses the weaker direction
    Pa = net.rssi_map(nodes[attach], noisy=True)
    report("access link is the weaker direction", np.all(acc[fin] <= Pa[fin] + 1e-9),
           f"relay->x minus access: mean {np.nanmean(Pa[fin] - acc[fin]):.2f} dB")
    # 4. association is fixed: moving another node does not change the access part
    other = next((n for n in net.nodes if n.name not in (client, attach)), None)
    if other is not None:
        net2 = copy.copy(net); net2.nodes = [copy.copy(n) for n in net.nodes]
        net2._fields = dict(net._fields)
        net2.node(other.name).xy = net2.node(attach).xy
        acc2 = margin_map(net2, client, attach, chain, threshold)["access"]
        same = np.array_equal(np.nan_to_num(acc, nan=0, neginf=-999), np.nan_to_num(acc2, nan=0, neginf=-999))
        report("association fixed: other nodes never raise the access term", same,
               f"moved {other.name} onto {attach}: access map {'unchanged' if same else 'CHANGED'}")
    # 5. a broken upstream hop makes the whole map non-positive
    hops = {(u, v): -200.0 for u, v, _ in res["hops"][:1]}
    if hops:
        mb = margin_map(net, client, attach, chain, threshold, estimates=hops)["margin"]
        report("broken upstream hop -> no connected region", np.nanmax(mb) < 0,
               f"first hop forced to -200 dBm -> max margin {np.nanmax(mb):.1f} dB")
    # 6. attached directly to the GCS -> no upstream hop, margin is exactly that one link
    r0 = margin_map(net, client, chain[0], [chain[0], client], threshold)
    f0 = np.isfinite(r0["margin"])
    exact = np.allclose(r0["margin"][f0], r0["access"][f0] - threshold)
    report("attached to GCS -> margin = client<->GCS link - threshold", r0["weakest"] is None and exact,
           f"{f0.sum()} reachable cells, max |diff| "
           f"{np.abs(r0['margin'][f0] - (r0['access'][f0] - threshold)).max():.1e} dB")
