"""Hierarchical communication relay planning -- Seo et al., RA-L 2025, Sec. IV-B.

Given the GCS, the relays, the client and the client's next mission point (here: the
next exploration frontier), decide

  1. CLIENT ONLY -- the goal is inside the client's bounded region for the current
     topology (Pr[every hop >= eta] >= theta at the goal), so only the client moves;
  2. DEPLOY      -- it is not: engage one more relay (the paper's placement for k
     relays, Sec. IV-B-1), assign robots to relay spots with the Hungarian algorithm
     (eq. 13-14), then plan every robot's trajectory sequentially with MCTS
     (eq. 16-17), starting from the robot nearest the GCS and ending with the client;
  3. SKIP        -- even with every relay engaged no feasible placement exists.

Every trajectory has t_f + 1 waypoints on the road graph, and robot r moves from
waypoint t-1 to t during interval t. All robots therefore reach waypoint t at the
same moment -- same interval, different distances (`schedule()` gives the speeds).

Network quality: the dominant-path model (eq. 1) + this project's shadowing, with
the predictive distribution N(mu, sigma_n^2) per direction. A link is usable only if
BOTH directions are (we take the weaker one), so Pr[link >= eta] = PHI(z)^2 and the
paper's constraint Pr[P >= eta] >= theta becomes
    mu >= eta + sigma_n * PHI^-1(sqrt(theta))   (= -85 + 3 * 1.108 = -81.68 dBm, Table I).
The paper uses KF-GPR for mu and sigma; we have no RSSI measurements to train it on
in simulation, so sigma is the measurement noise sigma_n.
"""
from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import norm

from propagation import GridMap, PropagationParams, field_from, rssi_from_distances
from network import Network, Node, NoiseParams
from roadgraph import RoadGraph, length, point_at, resample


@dataclass
class PlannerParams:            # paper Table I unless noted
    eta: float = -85.0          # RSSI threshold (dBm)
    theta: float = 0.75         # empirical probability threshold
    sigma: float = 3.0          # predictive std (dB) = measurement noise sigma_n
    beta: float = 2.0           # virtual-force step size (road-graph cells per dB)
    nu: float = 1.5             # virtual-force tolerance (dB)
    B_v: int = 1000             # virtual-force max iterations
    B_mcts: int = 300           # MCTS iterations
    t_f: int = 5                # rollout length = number of synchronised intervals
    C_p: float = 1.0            # UCT balancing coefficient
    n_routes: int = 3           # candidate trajectories per robot (|J|)       -- ours
    d_sep: float = 0.8          # min distance between robots at every waypoint (m) -- ours
    keep_deployed: bool = False # True: relays never move once placed; False (paper):
                                # engaged relays are re-placed for each new mission point

    @property
    def req(self) -> float:
        """Smallest predicted RSSI that satisfies Pr[P >= eta] >= theta."""
        return self.eta + self.sigma * norm.ppf(math.sqrt(self.theta))


@dataclass
class Robot:
    name: str
    xy: tuple
    kind: str = "relay"          # relay | client
    vmax: float = 0.5            # m/s


@dataclass
class Plan:
    decision: str                               # client_only | deploy | skip
    goal: tuple
    chain: list                                 # GCS, engaged relays (GCS outwards), client
    targets: dict = field(default_factory=dict) # name -> xy
    traj: dict = field(default_factory=dict)    # name -> (t_f+1, 2) waypoints
    seg: dict = field(default_factory=dict)     # name -> (t_f,) metres per interval
    hops: list = field(default_factory=list)    # per t: [(a, b, predicted dBm)]
    note: str = ""


class Predictor:
    """Expected link RSSI (weaker direction) between two named radios at two points."""

    def __init__(self, grid: GridMap, prop: PropagationParams, noise: NoiseParams, cache: int = 96):
        self.grid, self.prop = grid, prop
        self.net = Network(grid, prop, noise)
        self._f = OrderedDict()
        self.cache = cache
        # a robot hugging a wall, or a frontier goal, can sit in an obstacle cell:
        # radios there are treated as being at the nearest free cell
        from scipy.ndimage import distance_transform_edt
        _, self._near = distance_transform_edt(grid.occ.astype(bool), return_indices=True)

    def cell(self, xy):
        r, c = self.grid.world_to_cell(*xy)
        r = min(max(r, 0), self.grid.shape[0] - 1); c = min(max(c, 0), self.grid.shape[1] - 1)
        if self.grid.occ[r, c]:
            r, c = int(self._near[0][r, c]), int(self._near[1][r, c])
        return r, c

    def clean_from(self, xy) -> np.ndarray:
        rc = self.cell(xy)
        if rc in self._f:
            self._f.move_to_end(rc)
            return self._f[rc]
        f = field_from(self.grid, tuple(self.grid.cell_to_world(*rc)))
        P = rssi_from_distances(f.dE, f.dbar, self.prop).astype(np.float32)
        self._f[rc] = P
        if len(self._f) > self.cache:
            self._f.popitem(last=False)
        return P

    def shadow(self, name):
        return self.net._shadow_of(Node(name, (0, 0)))

    def link(self, a: str, a_xy, b: str, b_xy) -> float:
        ra, rb = self.cell(a_xy), self.cell(b_xy)
        clean = float(self.clean_from(a_xy)[rb])
        if not np.isfinite(clean):
            return -np.inf
        if not self.net.noise.enabled:
            return clean
        return min(clean + float(self.shadow(a)[rb]), clean + float(self.shadow(b)[ra]))

    def link_map(self, a: str, a_xy, b: str) -> np.ndarray:
        """link(a@a_xy, b@x) for every cell x -- the client's access term."""
        ra = self.cell(a_xy)
        clean = self.clean_from(a_xy).astype(float)
        if not self.net.noise.enabled:
            return clean
        return np.minimum(clean + self.shadow(a), clean + float(self.shadow(b)[ra]))


class RelayPlanner:
    def __init__(self, grid: GridMap, rg: RoadGraph, pred: Predictor, gcs_xy,
                 params: PlannerParams = PlannerParams(), seed: int = 0, gcs_name: str = "GCS"):
        self.grid, self.rg, self.pred, self.gcs = grid, rg, pred, tuple(gcs_xy)
        self.G = gcs_name                         # radio name: seeds the GCS's shadowing field
        self.allowed = None                       # optional bool grid: where robots may drive
        self.p = params
        self.rng = np.random.default_rng(seed)

    # ------------------------------------------------------------ the region
    def upstream(self, chain_xy: list) -> float:
        """Weakest predicted hop of a chain [(name, xy), ...] from the GCS."""
        return min((self.pred.link(a, ax, b, bx) for (a, ax), (b, bx) in zip(chain_xy[:-1], chain_xy[1:])),
                   default=np.inf)

    def region(self, chain_xy: list, client: str) -> np.ndarray:
        """Margin map vs the planning requirement: >= 0 where Pr[end-to-end >= eta] >= theta
        for the client attached (fixed) to the chain's last node."""
        tail, txy = chain_xy[-1]
        access = self.pred.link_map(tail, txy, client)
        return np.minimum(access, self.upstream(chain_xy)) - self.p.req

    def in_region(self, chain_xy, client, goal) -> bool:
        tail, txy = chain_xy[-1]
        return min(self.pred.link(tail, txy, client, goal), self.upstream(chain_xy)) >= self.p.req

    # ------------------------------------------------- placement (Sec. IV-B-1)
    def place(self, k: int, goal, client: str, names: list, fixed: list = ()) -> tuple:
        """k relay locations for the chain GCS -> ... -> client@goal.

        Gamma* = shortest road-graph path GCS -> goal, segmented evenly (k relays,
        k+1 intervals), then refined with the virtual force (eq. 12): each relay slides
        along Gamma* towards its weaker neighbour by beta * |P_{r-1,r} - P_{r,r+1}|.
        `fixed` = already-placed (name, xy) kept as they are (keep_deployed mode).
        Returns (positions, hops, Gamma*)."""
        routes = self.rg.routes(self.gcs, goal, k=1, allowed=self.allowed)
        if not routes:
            return None, None, None
        G = routes[0]
        L = length(G)
        res = self.grid.res
        s = np.array([(i + 1) * L / (k + 1) for i in range(k)])
        nodes = [self.G] + list(fixed and [f[0] for f in fixed]) + names + [client]

        def chain_pts(s):
            pts = [self.gcs] + [f[1] for f in fixed] + [tuple(point_at(G, si)) for si in s] + [tuple(goal)]
            return pts

        def hops(s):
            pts = chain_pts(s)
            return [self.pred.link(nodes[i], pts[i], nodes[i + 1], pts[i + 1]) for i in range(len(pts) - 1)]

        nf = len(fixed)
        for _ in range(self.p.B_v):
            h = hops(s)
            moved = 0.0
            done = True
            for r in range(k):
                prev, nxt = h[nf + r], h[nf + r + 1]          # P_{r-1,r}, P_{r,r+1}
                if not (np.isfinite(prev) and np.isfinite(nxt)):
                    d = 1.0 if not np.isfinite(nxt) else -1.0
                    diff = 10.0
                else:
                    diff = abs(prev - nxt)
                    d = 1.0 if nxt < prev else -1.0          # slide towards the weaker side
                if diff < self.p.nu:
                    continue
                done = False
                lo = s[r - 1] if r > 0 else 0.0
                hi = s[r + 1] if r < k - 1 else L
                new = float(np.clip(s[r] + d * self.p.beta * diff * res, lo + res, hi - res))
                moved = max(moved, abs(new - s[r]))
                s[r] = new
            if done or moved < res / 2:
                break
        return [tuple(point_at(G, si)) for si in s], hops(s), G

    # --------------------------------------------- assignment (eq. 13-14)
    def assign(self, relays: list, spots: list) -> dict:
        """Hungarian algorithm on road-graph travel distance -> {robot name: spot index}."""
        C = np.array([[self._travel(r.xy, sp) for sp in spots] for r in relays])
        C = np.where(np.isfinite(C), C, 1e9)
        ri, ci = linear_sum_assignment(C)
        return {relays[i].name: int(j) for i, j in zip(ri, ci)}

    def _travel(self, a, b) -> float:
        R = self.rg.routes(a, b, k=1, allowed=self.allowed)
        return length(R[0]) if R else np.inf

    # ----------------------------------------- sequential MCTS (eq. 16-17)
    def mcts(self, name: str, start, target, pred_name: str, pred_traj: np.ndarray, others=()):
        """Trajectory of `name` from start to target over t_f intervals that keeps
        Pr[P_{pred,name}(t) >= eta] >= theta at every t, the predecessor following
        pred_traj (t_f+1, 2), and stays d_sep from every other robot: `others` are
        (t_f+1, 2) trajectories (NaN rows = position unknown at that step)."""
        tf = self.p.t_f
        others = [np.asarray(o, float) for o in others]
        if math.dist(start, target) < 0.3:
            stay = np.repeat([start], tf + 1, axis=0)
            return stay, np.zeros(tf), True
        avoid = {tuple(np.round(o[k], 2)) for o in others for k in (0, -1) if np.isfinite(o[k]).all()}
        routes = self.rg.routes(start, target, k=self.p.n_routes, avoid=list(avoid), avoid_r=self.p.d_sep,
                                allowed=self.allowed)
        if not routes:
            stay = np.repeat([start], tf + 1, axis=0)
            return stay, np.zeros(tf), False
        Ls = [length(R) for R in routes]
        dhat = max(Ls)
        W = [resample(R, tf + 1) for R in routes]
        S = [np.linspace(0, L_, tf + 1) for L_ in Ls]           # arc length of each waypoint

        qcache = {}
        # never get CLOSER than d_sep; two robots that already are (a relay pushed, a creep)
        # may not approach further but must be allowed to move apart -- else every plan fails
        sep_thr = [min(self.p.d_sep, 0.95 * math.dist(start, o[0])) if np.isfinite(o[0]).all() else self.p.d_sep
                   for o in others]

        def pos(j, i):
            return tuple(start) if j is None else tuple(W[j][i])

        def remaining(j, i):
            return Ls[j] - S[j][i] if j is not None else min(Ls)

        def Q(j, i, t):
            key = (j, i, t)
            if key not in qcache:
                x = pos(j, i)
                clash = False
                if t > 0:                                      # whole interval t-1 -> t, both moving
                    # this robot's actual path in the interval: along its ROUTE (which bends
                    # through doorways), not the straight line between two waypoints
                    if j is not None and i > 0:
                        n_s = max(6, int(math.ceil((S[j][i] - S[j][i - 1]) / 0.2)) + 1)
                        mine = [tuple(point_at(routes[j], s)) for s in np.linspace(S[j][i - 1], S[j][i], n_s)]
                    else:
                        mine = [x] * 6                         # holding still
                    fs = np.linspace(0.0, 1.0, len(mine))
                    for o, thr in zip(others, sep_thr):
                        if not np.isfinite(o[t]).all():
                            continue
                        o0 = o[t - 1] if np.isfinite(o[t - 1]).all() else o[t]
                        for f, a_ in zip(fs, mine):
                            b_ = (o0[0] + f * (o[t][0] - o0[0]), o0[1] + f * (o[t][1] - o0[1]))
                            if math.dist(a_, b_) < thr:
                                clash = True; break
                        if clash:
                            break
                if self.allowed is not None and t > 0 and not clash:
                    rr, cc = self.grid.world_to_cell(*x)
                    clash = not self.allowed[rr, cc]           # robots cannot plan through unknown space
                P = -np.inf if clash else self.pred.link(pred_name, tuple(pred_traj[t]), name, x)
                qcache[key] = (1.0 - remaining(j, i) / dhat) if P >= self.p.req else -1.0
            return qcache[key]

        def children(j, i, t):
            if t >= tf:
                return []
            if j is None:                                     # not committed to a route yet
                return [(jj, 1, t + 1) for jj in range(len(W))] + [(None, 0, t + 1)]
            out = [(j, i, t + 1)]                             # hold
            if i < tf:
                out.append((j, i + 1, t + 1))                 # advance one waypoint
            return out

        root = (None, 0, 0)
        N, Wt, kids = {root: 0}, {root: 0.0}, {}
        for _ in range(self.p.B_mcts):
            path, node = [root], root
            # selection
            while node in kids and kids[node] and all(c in N for c in kids[node]):
                n_par = max(N[node], 1)
                node = max(kids[node], key=lambda c: Wt[c] / N[c] +
                           self.p.C_p * math.sqrt(2 * math.log(n_par) / N[c]))
                path.append(node)
            # expansion
            if node not in kids:
                kids[node] = children(*node)
            untried = [c for c in kids[node] if c not in N]
            if untried:
                node = untried[self.rng.integers(len(untried))]
                N[node], Wt[node] = 0, 0.0
                path.append(node)
            # simulation: random rollout to t_f, value = mean instance reward
            vals, cur = [Q(*node)], node
            while True:
                ch = children(*cur)
                if not ch:
                    break
                cur = ch[self.rng.integers(len(ch))]
                vals.append(Q(*cur))
            v = float(np.mean(vals))
            for n_ in path:                                   # back-propagation
                N[n_] += 1; Wt[n_] += v
        # extract: most visited child at every depth
        node, seq = root, [root]
        while kids.get(node):
            seen = [c for c in kids[node] if c in N]
            if not seen:
                break
            node = max(seen, key=lambda c: N[c])
            seq.append(node)
        while len(seq) < tf + 1:                              # tree not expanded that deep
            j, i, t = seq[-1]
            seq.append((j, min(i + 1, tf) if j is not None else 0, t + 1))
        traj = np.array([pos(j, i) for j, i, _ in seq])
        arc = np.array([0.0 if j is None else S[j][i] for j, i, _ in seq])
        ok = all(Q(j, i, t) > -1 for j, i, t in seq[1:])
        return traj, np.diff(arc), ok

    # ------------------------------------------------------------- top level
    # If MCTS cannot keep every step of a move above the requirement, retry with more
    # candidate routes and finer steps; a move that still cannot is NOT executed.
    LADDER = ((3, 5), (5, 5), (5, 8), (6, 12))          # (|J| routes, t_f intervals)  -- ours

    def _tries(self):
        keep = (self.p.n_routes, self.p.t_f)
        try:
            for nr, tf in self.LADDER:
                self.p.n_routes, self.p.t_f = nr, tf
                yield tf
        finally:
            self.p.n_routes, self.p.t_f = keep

    def plan(self, goal, client: Robot, relays: list[Robot], engaged: list[str]) -> Plan:
        """engaged: relay names currently in the chain, GCS outwards."""
        goal = tuple(goal)
        byname = {r.name: r for r in relays}
        chain_xy = [(self.G, self.gcs)] + [(n, byname[n].xy) for n in engaged]
        why = "no feasible chain with %d relays" % len(relays)

        d = self.p.d_sep
        blocked = [r.name for r in relays if math.dist(r.xy, goal) < d]
        if blocked:
            why = f"goal within {d} m of {', '.join(blocked)}"
        if self.in_region(chain_xy, client.name, goal) and not blocked:
            tail, txy = chain_xy[-1]
            for tf in self._tries():
                parked = [np.repeat([r.xy], tf + 1, axis=0) for r in relays]
                traj, seg, ok = self.mcts(client.name, client.xy, goal, tail, np.repeat([txy], tf + 1, axis=0),
                                          others=parked)
                if not ok:
                    continue
                plan = Plan("client_only", goal, [self.G] + engaged + [client.name], {client.name: goal})
                plan.traj[client.name], plan.seg[client.name] = traj, seg
                for r in relays:
                    plan.traj[r.name] = np.repeat([r.xy], tf + 1, axis=0); plan.seg[r.name] = np.zeros(tf)
                plan.note = f"goal inside the bounded region (t_f {tf})"
                self._fill_hops(plan, client.name)
                return plan
            why = "goal inside the region but no client trajectory keeps every step"
            # fall through: a relay may be able to cover the way there

        R = len(relays)
        # one more relay than now; with every relay already out, the paper re-places them
        ks = range(len(engaged) + 1, R + 1) if self.p.keep_deployed else range(min(len(engaged) + 1, R), R + 1)
        for k in ks:
            if self.p.keep_deployed:
                fixed = [(n, byname[n].xy) for n in engaged]
                free = [r for r in relays if r.name not in engaged]
                spots, hops, G = self.place(k - len(engaged), goal, client.name,
                                            [r.name for r in free[:k - len(engaged)]], fixed)
                pool = free
            else:
                spots, hops, G = self.place(k, goal, client.name, [r.name for r in relays[:k]])
                pool = relays
            if spots is None:
                continue
            ass = self.assign(pool, spots)
            order = sorted(ass, key=lambda n: ass[n])           # deployment sequence s_l
            if self.p.keep_deployed:
                order = engaged + order
                spots = [byname[n].xy for n in engaged] + spots
                ass = {n: i for i, n in enumerate(order)}
            # placement feasibility with the ASSIGNED names (shadowing is per radio)
            names = [self.G] + order + [client.name]
            pts = [self.gcs] + [spots[ass[n]] for n in order] + [goal]
            h = [self.pred.link(names[i], pts[i], names[i + 1], pts[i + 1]) for i in range(len(pts) - 1)]
            if min(h) < self.p.req:
                continue
            idle = [r for r in relays if r.name not in order]
            fixed_pts = [spots[ass[n]] for n in order] + [goal] + [r.xy for r in idle]
            if any(math.dist(p_, q_) < d for i_, p_ in enumerate(fixed_pts) for q_ in fixed_pts[i_ + 1:]):
                why = f"{k} relay(s): relay spots / goal / parked relays closer than {d} m"
                continue
            for tf in self._tries():
                def until1(xy):                                 # a robot that will move later:
                    o = np.full((tf + 1, 2), np.nan)            # only known for the first step
                    o[:2] = xy
                    return o
                planned = [np.repeat([r.xy], tf + 1, axis=0) for r in idle]
                plan = Plan("deploy", goal, names[:-1] + [client.name])
                prev_name, prev_traj = self.G, np.repeat([self.gcs], tf + 1, axis=0)
                all_ok = True
                for n in order:                                 # sequential, GCS outwards
                    tgt = spots[ass[n]]
                    later = [until1(byname[m].xy) for m in order if m != n and not any(
                        m == q for q in plan.traj)] + [until1(client.xy)]
                    traj, seg, ok = self.mcts(n, byname[n].xy, tgt, prev_name, prev_traj,
                                              others=planned + later)
                    planned.append(traj)
                    plan.targets[n], plan.traj[n], plan.seg[n] = tgt, traj, seg
                    prev_name, prev_traj = n, traj
                    all_ok &= ok
                    if not all_ok:
                        break
                if not all_ok:
                    continue
                traj, seg, ok = self.mcts(client.name, client.xy, goal, prev_name, prev_traj, others=planned)
                if not ok:
                    continue
                plan.targets[client.name], plan.traj[client.name], plan.seg[client.name] = goal, traj, seg
                for r in relays:
                    if r.name not in plan.traj:                 # idle relays stay put
                        plan.traj[r.name] = np.repeat([r.xy], tf + 1, axis=0); plan.seg[r.name] = np.zeros(tf)
                plan.note = (f"{k} relay(s) engaged (t_f {tf}); final hops " +
                             ", ".join(f"{v:.1f}" for v in h))
                self._fill_hops(plan, client.name)
                return plan
            why = f"{k} relay(s) could be placed but no trajectories keep every step"

        return Plan("skip", goal, [self.G] + engaged + [client.name], note=why)

    def _fill_hops(self, plan: Plan, client: str):
        for t in range(len(next(iter(plan.traj.values())))):
            row = []
            for a, b in zip(plan.chain[:-1], plan.chain[1:]):
                ax = self.gcs if a == self.G else tuple(plan.traj[a][t])
                bx = tuple(plan.traj[b][t])
                row.append((a, b, self.pred.link(a, ax, b, bx)))
            plan.hops.append(row)


def schedule(plan: Plan, vmax: dict) -> list[dict]:
    """Synchronised intervals: in interval t every robot covers its own segment and
    all arrive together. Duration = slowest robot at full speed; the rest slow down.
    Returns [{'dt': s, 'v': {name: m/s}, 'd': {name: m}}, ...]."""
    out = []
    T = len(next(iter(plan.seg.values())))
    for t in range(T):
        d = {n: float(plan.seg[n][t]) for n in plan.seg}
        dt = max((d[n] / vmax[n] for n in d if d[n] > 1e-6), default=0.0)
        v = {n: (d[n] / dt if dt > 0 else 0.0) for n in d}
        out.append(dict(dt=dt, v=v, d=d))
    return out
