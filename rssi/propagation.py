"""Obstacle-aware RSSI propagation model on the ground-truth occupancy grid.

    P(a,b) = P0 - 10 n log10(dE) - alpha log10(dbar + 1)          (eq. 1)
    dbar   = |dE - dS|

    dE  Euclidean (straight-line) distance between a and b
    dS  shortest path between a and b through FREE space (around walls)
    dbar the detour the signal is forced to take: 0 with line of sight

Distances come from one source at a time: `field_from(src)` returns dE, dS and dbar
to EVERY cell of the map, so a heatmap is one call and a node-to-node link is a lookup.

Why line of sight is checked explicitly
---------------------------------------
dS comes from fast marching (scikit-fmm). Any grid-based shortest path carries a
small discretisation error (8-connected Dijkstra: up to ~8 %), so dS would come out
slightly larger than dE even in open space -- dbar would never be exactly 0 and the
heatmap would show fake attenuation ripples where the signal is in plain view.
The physics says dbar = 0 under line of sight, so LOS cells are found by ray
casting and get dS := dE exactly. Fast marching is used only for cells hidden
behind walls, which is where the detour actually exists.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
import skfmm


# ----------------------------------------------------------------------------- map
@dataclass
class GridMap:
    occ: np.ndarray            # (H, W) uint8, 1 = occupied
    res: float                 # metres per cell
    origin: np.ndarray         # world (x, y) of the grid's lower-left corner

    @classmethod
    def load(cls, npy_path: str, json_path: str) -> "GridMap":
        occ = np.load(npy_path).astype(np.uint8)
        meta = json.load(open(json_path))
        return cls(occ=occ, res=float(meta["resolution"]),
                   origin=np.asarray(meta["origin_xy"], float))

    @property
    def shape(self):
        return self.occ.shape

    @property
    def free(self) -> np.ndarray:
        return self.occ == 0

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """-> (row, col). grid[row, col]; row grows with world y."""
        c = int(np.floor((x - self.origin[0]) / self.res))
        r = int(np.floor((y - self.origin[1]) / self.res))
        return r, c

    def cell_to_world(self, r, c):
        return (self.origin[0] + (np.asarray(c) + 0.5) * self.res,
                self.origin[1] + (np.asarray(r) + 0.5) * self.res)

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1]

    def extent(self):
        """matplotlib imshow extent (origin='lower')."""
        H, W = self.shape
        return [self.origin[0], self.origin[0] + W * self.res,
                self.origin[1], self.origin[1] + H * self.res]


# --------------------------------------------------------------------- parameters
@dataclass
class PropagationParams:
    P0: float = -40.0      # dBm at 1 m
    n: float = 2.0         # path-loss exponent
    alpha: float = 20.0    # obstacle / detour attenuation weight
    d_min: float = 1.0     # dE is clamped below this: the model is referenced at 1 m,
                           # and log10(dE) < 0 would put RSSI ABOVE P0 near the source


# ------------------------------------------------------------------- geometry core
def line_of_sight_mask(grid: GridMap, src_rc: tuple[int, int],
                       step: float = 0.25) -> np.ndarray:
    """Cells visible from src (no occupied cell on the straight line).

    Ray casting: enough rays that adjacent rays are < 1 cell apart at the far edge of
    the map, each marched in `step`-cell increments until it hits an occupied cell.
    """
    H, W = grid.shape
    occ = grid.occ.astype(bool)
    r0, c0 = src_rc[0] + 0.5, src_rc[1] + 0.5
    R = np.hypot(H, W)
    n_rays = int(np.ceil(2 * np.pi * R * 1.5))
    th = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
    dr, dc = np.sin(th) * step, np.cos(th) * step

    vis = np.zeros((H, W), bool)
    alive = np.ones(n_rays, bool)
    r, c = np.full(n_rays, r0), np.full(n_rays, c0)
    for _ in range(int(np.ceil(R / step)) + 2):
        ri, ci = np.floor(r).astype(int), np.floor(c).astype(int)
        inb = (ri >= 0) & (ri < H) & (ci >= 0) & (ci < W)
        alive &= inb
        if not alive.any():
            break
        ra, ca = ri[alive], ci[alive]
        hit = occ[ra, ca]
        idx = np.flatnonzero(alive)
        alive[idx[hit]] = False           # ray stops AT the wall; wall itself not visible
        ok = ~hit
        vis[ra[ok], ca[ok]] = True
        r += dr; c += dc
    return vis


def geodesic_distance(grid: GridMap, src_rc: tuple[int, int]) -> np.ndarray:
    """Shortest free-space path length (metres) from src to every cell.

    Fast marching with obstacles masked. 4-neighbour stencil, so paths cannot slip
    diagonally between two touching wall cells. Unreachable cells -> inf.
    """
    phi = np.ones(grid.shape)
    phi[src_rc] = -1.0
    phi = np.ma.MaskedArray(phi, mask=grid.occ.astype(bool))
    d = skfmm.distance(phi, dx=grid.res)
    out = np.full(grid.shape, np.inf)
    good = ~np.ma.getmaskarray(d)
    out[good] = np.asarray(d)[good]
    # fmm measures from the zero contour (~half a cell from the source centre)
    out[good] += 0.5 * grid.res
    return out


@dataclass
class SourceField:
    src_rc: tuple[int, int]
    dE: np.ndarray        # metres
    dS: np.ndarray        # metres, inf where unreachable
    dbar: np.ndarray      # metres, inf where unreachable
    los: np.ndarray       # bool
    dS_fmm_raw: np.ndarray = field(repr=False, default=None)   # kept for diagnostics


def field_from(grid: GridMap, src_xy: tuple[float, float]) -> SourceField:
    r, c = grid.world_to_cell(*src_xy)
    if not grid.in_bounds(r, c):
        raise ValueError(f"source {src_xy} is outside the map")
    if grid.occ[r, c]:
        raise ValueError(f"source {src_xy} is inside an obstacle")

    H, W = grid.shape
    rr, cc = np.mgrid[0:H, 0:W]
    dE = grid.res * np.hypot(rr - r, cc - c)

    fmm = geodesic_distance(grid, (r, c))
    los = line_of_sight_mask(grid, (r, c))

    dS = np.where(los, dE, np.maximum(fmm, dE))   # a detour is never shorter than dE
    dS[grid.occ.astype(bool)] = np.inf
    dbar = np.abs(dE - dS)                         # inf stays inf
    return SourceField((r, c), dE, dS, dbar, los, fmm)


# ----------------------------------------------------------------------- the model
def rssi_from_distances(dE, dbar, p: PropagationParams):
    """Equation (1). Works on scalars or arrays; inf dbar -> -inf (no signal)."""
    dE = np.maximum(np.asarray(dE, float), p.d_min)
    with np.errstate(divide="ignore", invalid="ignore"):
        return p.P0 - 10.0 * p.n * np.log10(dE) - p.alpha * np.log10(np.asarray(dbar, float) + 1.0)


def rssi_map(grid: GridMap, src_xy, p: PropagationParams):
    """RSSI (dBm) at every cell from one transmitter. NaN inside obstacles."""
    f = field_from(grid, src_xy)
    P = rssi_from_distances(f.dE, f.dbar, p)
    P = np.where(grid.occ.astype(bool), np.nan, P)
    return P, f


def rssi_link(grid: GridMap, a_xy, b_xy, p: PropagationParams, field_a: SourceField = None):
    """RSSI of the link a -> b. Pass a cached field_a when querying many b's."""
    f = field_a or field_from(grid, a_xy)
    r, c = grid.world_to_cell(*b_xy)
    if not grid.in_bounds(r, c) or grid.occ[r, c]:
        return float("nan")
    return float(rssi_from_distances(f.dE[r, c], f.dbar[r, c], p))
