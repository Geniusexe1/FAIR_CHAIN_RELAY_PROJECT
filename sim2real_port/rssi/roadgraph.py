"""Road graph G(M) -- Seo et al., RA-L 2025, Sec. IV-B-1, Fig. 4(a).

The free space is thinned to its centre lines (Zhang-Suen, the paper's ref. [21]),
so planning happens along corridors instead of over every free cell.

    rg = RoadGraph.build(grid)               # cached on disk next to the GT map
    routes = rg.routes(a_xy, b_xy, k=3)      # shortest route + up to k-1 alternatives
    pts = resample(routes[0], n)             # n evenly spaced points along a route

Only cells with at least `clearance` metres to the nearest obstacle are thinned,
so every road-graph point is somewhere a robot body actually fits.
"""
from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt, convolve, label
from scipy.sparse import coo_matrix, diags
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

from propagation import GridMap

NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _prune(skel: np.ndarray, n: int) -> np.ndarray:
    """Remove spurs shorter than n cells (furniture makes many) by peeling endpoints
    n times, then growing the surviving branches back to their original extent."""
    k = np.ones((3, 3), int); k[1, 1] = 0
    s = skel.copy()
    for _ in range(n):
        deg = convolve(s.astype(int), k, mode="constant")
        s &= ~((deg <= 1) & s)
    for _ in range(n):                       # regrow along the original skeleton
        grow = convolve(s.astype(int), np.ones((3, 3), int), mode="constant") > 0
        s = skel & (s | (grow & _endpoint_neighbourhood(s, k)))
    return s


def _endpoint_neighbourhood(s, k):
    deg = convolve(s.astype(int), k, mode="constant")
    ends = (deg <= 1) & s
    return convolve(ends.astype(int), np.ones((3, 3), int), mode="constant") > 0


def _clear_line(occ: np.ndarray, a, b) -> bool:
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    rr = np.linspace(a[0], b[0], n).round().astype(int)
    cc = np.linspace(a[1], b[1], n).round().astype(int)
    inside = (rr >= 0) & (rr < occ.shape[0]) & (cc >= 0) & (cc < occ.shape[1])
    if not inside.all():                       # leaves the map: not a clear line
        return False
    return not occ[rr, cc].any()


@dataclass
class RoadGraph:
    grid: GridMap
    skel: np.ndarray              # bool (H, W): road-graph cells
    rc: np.ndarray                # (N, 2) cell of node i
    W: object                     # (N, N) sparse edge lengths, metres

    # ------------------------------------------------------------------ build
    @classmethod
    def build(cls, grid: GridMap, clearance: float = 0.45, spur_m: float = 1.0,
              cache_dir: str | None = None, drop_outside: bool = True) -> "RoadGraph":
        key = hashlib.md5(grid.occ.tobytes() + f"{clearance}{spur_m}v2".encode()).hexdigest()[:12]
        path = os.path.join(cache_dir, f"roadgraph_{key}.npz") if cache_dir else None
        if path and os.path.exists(path):
            skel = np.load(path)["skel"]
        else:
            free = grid.occ == 0
            # free space connected to the grid's border is OUTSIDE the building (the margin round
            # the outer wall): without this its long thin ring can be the biggest network (UNIST)
            flab, _ = label(free)
            outside = np.unique(np.r_[flab[0], flab[-1], flab[:, 0], flab[:, -1]])
            if drop_outside:                           # a live map has no such margin: unknown is not free
                free &= ~np.isin(flab, outside[outside > 0])
            clear = distance_transform_edt(free) * grid.res >= clearance
            skel = skeletonize(clear, method="zhang")
            skel = _prune(skel, int(round(spur_m / grid.res)))
            lab, n = label(skel, structure=np.ones((3, 3)))
            if n > 1:                                  # keep the biggest connected network
                sizes = np.bincount(lab.ravel()); sizes[0] = 0
                skel = lab == sizes.argmax()
            if path:
                os.makedirs(cache_dir, exist_ok=True)
                np.savez_compressed(path, skel=skel)
        rc = np.argwhere(skel)
        idx = -np.ones(skel.shape, int); idx[rc[:, 0], rc[:, 1]] = np.arange(len(rc))
        rows, cols, w = [], [], []
        H, Wd = skel.shape
        for dr, dc in NB8:
            r2, c2 = rc[:, 0] + dr, rc[:, 1] + dc
            ok = (r2 >= 0) & (r2 < H) & (c2 >= 0) & (c2 < Wd)
            j = np.full(len(rc), -1); j[ok] = idx[r2[ok], c2[ok]]
            m = j >= 0
            rows.append(np.flatnonzero(m)); cols.append(j[m])
            w.append(np.full(m.sum(), np.hypot(dr, dc) * grid.res))
        Wm = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))),
                        shape=(len(rc), len(rc))).tocsr()
        g = cls(grid, skel, rc, Wm)
        g._tree = cKDTree(rc)
        g._tight = distance_transform_edt(grid.occ == 0) * grid.res < 0.8 * clearance
        return g

    # ----------------------------------------------------------------- query
    def attach(self, xy) -> int:
        """Nearest road-graph node with a clear straight line to xy."""
        r, c = self.grid.world_to_cell(*xy)
        occ = self.grid.occ.astype(bool)
        for k in (16, 64, 256, len(self.rc)):
            _, ii = self._tree.query([r, c], k=min(k, len(self.rc)))
            for i in np.atleast_1d(ii):
                if _clear_line(occ, (r, c), self.rc[i]):
                    return int(i)
        return int(np.atleast_1d(self._tree.query([r, c], k=1)[1])[0])

    def _path(self, Wm, i, j):
        d, pred = dijkstra(Wm, indices=i, return_predecessors=True, min_only=False)
        if not np.isfinite(d[j]):
            return None, np.inf
        p = [j]
        while p[-1] != i:
            p.append(pred[p[-1]])
        return p[::-1], d[j]

    def routes(self, a_xy, b_xy, k: int = 3, max_ratio: float = 1.6,
               max_overlap: float = 0.7, avoid=(), avoid_r: float = 0.8, allowed=None) -> list[np.ndarray]:
        """Shortest road-graph route a -> b, plus up to k-1 genuinely different
        alternatives (edge-penalty method). Each route is an (n, 2) WORLD polyline
        starting exactly at a and ending exactly at b.

        avoid: points (other robots) to keep `avoid_r` away from. Road-graph nodes
        that close cost 10x per end of an edge (soft: the graph stays connected), and
        the straight shortcut is only taken if it clears them.
        allowed: optional bool grid (GT shape) of cells the robots may drive through, e.g. the
        space their own SLAM map already knows is free -- Nav2 cannot plan through unknown
        space, so a waypoint there is unreachable however open the corridor really is."""
        if not len(self.rc):                         # nothing explored yet that a robot fits in
            return []
        i, j = self.attach(a_xy), self.attach(b_xy)
        W0 = self.W
        avoid = [tuple(p) for p in avoid if np.hypot(p[0] - a_xy[0], p[1] - a_xy[1]) > 1e-3]
        f = np.ones(len(self.rc))
        if avoid:
            node_xy = np.array(self.grid.cell_to_world(self.rc[:, 0], self.rc[:, 1])).T
            near = np.zeros(len(self.rc), bool)
            for p in avoid:
                near |= np.hypot(node_xy[:, 0] - p[0], node_xy[:, 1] - p[1]) < avoid_r
            f *= np.where(near, 10.0, 1.0)
        if allowed is not None:
            f *= np.where(allowed[self.rc[:, 0], self.rc[:, 1]], 1.0, 1e3)   # unknown: last resort
        if (f != 1).any():
            W0 = (diags(f) @ self.W @ diags(f)).tocsr()
        Wm = W0.copy().tolil()
        out, used = [], []
        best = None
        ra, rb = self.grid.world_to_cell(*a_xy), self.grid.world_to_cell(*b_xy)
        blocked = self._tight if allowed is None else (self._tight | ~allowed)
        if _clear_line(blocked, ra, rb) and all(
                seg_dist(p, a_xy, b_xy) >= avoid_r for p in avoid):   # straight, roomy, clear of robots
            out.append(np.array([a_xy, b_xy], float))
            best = length(out[0])
            if k == 1:
                return out
        for _ in range(k * 3):
            p, L = self._path(Wm.tocsr(), i, j)
            if p is None:
                break
            true_len = float(sum(self.W[p[q], p[q + 1]] for q in range(len(p) - 1)))
            if best is None:
                best = true_len
            elif true_len > max_ratio * best and out:
                break
            s = set(p)
            if all(len(s & u) / max(1, min(len(s), len(u))) < max_overlap for u in used):
                used.append(s)
                xy = np.array(self.grid.cell_to_world(self.rc[p, 0], self.rc[p, 1])).T
                out.append(np.vstack([a_xy, xy, b_xy]))
                if len(out) == k:
                    break
            for q in range(len(p) - 1):                      # discourage reuse
                Wm[p[q], p[q + 1]] = Wm[p[q], p[q + 1]] * 4.0
                Wm[p[q + 1], p[q]] = Wm[p[q + 1], p[q]] * 4.0
        return out


def seg_dist(p, a, b) -> float:
    """Distance from point p to segment a-b."""
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1))
    return float(np.hypot(*(a + t * ab - p)))


def length(poly: np.ndarray) -> float:
    return float(np.hypot(*np.diff(poly, axis=0).T).sum()) if len(poly) > 1 else 0.0


def point_at(poly: np.ndarray, s: float) -> np.ndarray:
    """Point at arc length s along a polyline (clamped)."""
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(poly, axis=0).T))]
    s = float(np.clip(s, 0, seg[-1]))
    return np.array([np.interp(s, seg, poly[:, 0]), np.interp(s, seg, poly[:, 1])])


def resample(poly: np.ndarray, n: int) -> np.ndarray:
    """n points evenly spaced in arc length, first = start, last = end."""
    L = length(poly)
    return np.array([point_at(poly, s) for s in np.linspace(0, L, n)])


def sub_polyline(poly: np.ndarray, s0: float, s1: float, step: float = 0.25) -> np.ndarray:
    return np.array([point_at(poly, s) for s in np.r_[np.arange(s0, s1, step), s1]])
