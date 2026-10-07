"""Multi-node RSSI network on top of propagation.py: nodes, noise, links.

Noise model (dB, added to eq. 1)
--------------------------------
  P_noisy = P_clean + S_tx(rx) + F

  S   log-normal SHADOWING. Slow and spatially correlated: two receivers close
      together see nearly the same offset, as walls and furniture would cause.
      One static field per transmitter, sigma_shadow dB, correlation length
      corr_m (Gaussian kernel: corr(d) = exp(-(d/corr_m)^2), so corr(corr_m)=1/e).
  F   FAST FADING / measurement noise. i.i.d. N(0, sigma_fast) per reading.

Shadowing fields are seeded per node name, so a run is reproducible and the same
node keeps the same field. A link a<->b uses the average of S_a(b) and S_b(a),
which keeps the link symmetric, as a physical channel is.
"""
from __future__ import annotations

import zlib
from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import gaussian_filter

from propagation import GridMap, PropagationParams, field_from, rssi_from_distances


@dataclass
class NoiseParams:
    sigma_shadow: float = 4.0   # dB
    corr_m: float = 3.0         # m
    sigma_fast: float = 2.0     # dB
    enabled: bool = True


@dataclass
class Node:
    name: str
    xy: tuple[float, float]
    kind: str = "robot"         # gcs | relay | head | robot


def shadowing_field(grid: GridMap, seed: int, sigma: float, corr_m: float) -> np.ndarray:
    """Zero-mean, std=sigma, spatially correlated Gaussian field over the grid."""
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(grid.shape)
    # filter std s gives correlation exp(-r^2 / (4 s^2)); want exp(-1) at corr_m
    s = (corr_m / 2.0) / grid.res
    f = gaussian_filter(w, sigma=s, mode="reflect")
    f -= f.mean()
    return f * (sigma / f.std())


def _seed(name: str) -> int:
    return zlib.crc32(name.encode())


@dataclass
class Network:
    grid: GridMap
    prop: PropagationParams = field(default_factory=PropagationParams)
    noise: NoiseParams = field(default_factory=NoiseParams)
    nodes: list[Node] = field(default_factory=list)
    _fields: dict = field(default_factory=dict, repr=False)
    _shadow: dict = field(default_factory=dict, repr=False)

    # -- caching: a field depends only on the node's CELL, a shadow only on its name
    def _field(self, n: Node):
        key = (n.name, self.grid.world_to_cell(*n.xy))
        if key not in self._fields:
            self._fields = {k: v for k, v in self._fields.items() if k[0] != n.name}
            self._fields[key] = field_from(self.grid, n.xy)
        return self._fields[key]

    def _shadow_of(self, n: Node):
        if n.name not in self._shadow:
            self._shadow[n.name] = shadowing_field(self.grid, _seed(n.name),
                                                   self.noise.sigma_shadow, self.noise.corr_m)
        return self._shadow[n.name]

    def node(self, name: str) -> Node:
        return next(n for n in self.nodes if n.name == name)

    # -- maps
    def rssi_map(self, n: Node, noisy: bool = True) -> np.ndarray:
        """dBm everywhere from node n. Noisy = clean + shadowing (no fast fading:
        a map is an expectation surface, not one instant)."""
        f = self._field(n)
        P = rssi_from_distances(f.dE, f.dbar, self.prop)
        if noisy and self.noise.enabled:
            P = P + self._shadow_of(n)
        return np.where(self.grid.occ.astype(bool), np.nan, P)

    def coverage_map(self, noisy: bool = True, kinds=None) -> np.ndarray:
        """Best (max) RSSI from any node -- what a receiver anywhere would latch onto."""
        sel = [n for n in self.nodes if kinds is None or n.kind in kinds]
        stack = np.stack([self.rssi_map(n, noisy) for n in sel])
        with np.errstate(invalid="ignore"):
            return np.nanmax(np.where(np.isfinite(stack), stack, -np.inf), axis=0)

    # -- links
    def link(self, a: Node, b: Node, noisy: bool = True, rng=None) -> dict:
        fa = self._field(a)
        rb, cb = self.grid.world_to_cell(*b.xy)
        dE, dbar, los = float(fa.dE[rb, cb]), float(fa.dbar[rb, cb]), bool(fa.los[rb, cb])
        clean = float(rssi_from_distances(dE, dbar, self.prop))
        P = clean
        if noisy and self.noise.enabled and np.isfinite(clean):
            ra, ca = self.grid.world_to_cell(*a.xy)
            P += 0.5 * (self._shadow_of(a)[rb, cb] + self._shadow_of(b)[ra, ca])
            if rng is not None:
                P += rng.normal(0.0, self.noise.sigma_fast)
        return dict(a=a.name, b=b.name, rssi=P, clean=clean, dE=dE, dbar=dbar, los=los)

    def links(self, noisy=True, rng=None) -> list[dict]:
        out = []
        for i, a in enumerate(self.nodes):
            for b in self.nodes[i + 1:]:
                out.append(self.link(a, b, noisy, rng))
        return out


def connectivity(links: list[dict], nodes: list[Node], threshold: float, root: str):
    """Which nodes reach `root` over links >= threshold, and each one's best route.

    Best route = widest path: maximise the weakest hop's RSSI, which is what limits a
    multi-hop radio route.
    """
    adj = {n.name: {} for n in nodes}
    for L in links:
        if np.isfinite(L["rssi"]) and L["rssi"] >= threshold:
            adj[L["a"]][L["b"]] = L["rssi"]; adj[L["b"]][L["a"]] = L["rssi"]
    best = {root: (np.inf, [root])}
    frontier = [root]
    while frontier:                                  # bottleneck Dijkstra
        u = max(frontier, key=lambda k: best[k][0]); frontier.remove(u)
        for v, w in adj[u].items():
            cand = min(best[u][0], w)
            if v not in best or cand > best[v][0]:
                best[v] = (cand, best[u][1] + [v]); frontier.append(v)
    return {k: v for k, v in best.items() if k != root}
