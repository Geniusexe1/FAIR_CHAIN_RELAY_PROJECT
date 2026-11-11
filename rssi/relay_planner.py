"""Hierarchical communication relay planning -- Seo et al., RA-L 2025, Sec. IV-B.

Given the GCS, the relays, the client and the client's next mission point (here: the
next exploration frontier), decide

  1. CLIENT ONLY -- the goal is inside the client's bounded region for the network as
     it stands (Pr[every hop of the route >= eta] >= theta at the goal), so only the
     client moves;
  2. DEPLOY      -- it is not: place the FEWEST relays that reach it (k = 1, 2, ...; the
     paper's placement for k relays, Sec. IV-B-1), assign robots to relay spots with the
     Hungarian algorithm (eq. 13-14), then plan every robot's trajectory sequentially
     with MCTS (eq. 16-17), starting from the robot nearest the GCS, client last;
  3. SKIP        -- even with every relay placed no feasible plan exists.

Routing (ours): every robot can forward, so the client's connection is the best route
through ANY of the radios -- the fewest hops that satisfy the requirement, ties broken by
the strongest weakest hop (`route()`). A client next to the GCS talks to it directly
even when relays are out; a relay that is not on the route is idle. The paper's fixed
chain GCS -> r1 -> ... -> client is the special case where only that route works.

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
    route_ratio: float = 1.6    # candidates up to this x the shortest route   -- ours
    d_sep: float = 0.5          # min distance between robots at every waypoint (m): just over the bodies
                                # (0.35 m); closer encounters are the coordinator's right of way -- ours
    pass_clear: float = 0.9     # a robot standing still in a corridor at least 2x this wide does not
                                # block a route: Nav2 drives around it                       -- ours
    keep_deployed: bool = False # True: relays never move once placed; False (paper):
                                # relays are re-placed for each new mission point

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
    chain: list                                 # route at the goal: GCS, forwarding relays, client
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
        self.set_map(grid, rg)

    def set_map(self, grid: GridMap, rg: RoadGraph, allowed=None):
        """Where robots can go. May change while exploring; the radio model (pred) does not."""
        from scipy.ndimage import distance_transform_edt, maximum_filter
        self.grid, self.rg, self.allowed = grid, rg, allowed
        clr = distance_transform_edt(grid.free) * grid.res
        k = int(round(1.0 / grid.res)) * 2 + 1                     # corridor half-width around a
        self.clear = maximum_filter(clr, size=k)                   # point: max clearance within 1 m

    # ------------------------------------------------------------ routing
    def route(self, pos: dict, dst: str):
        """Best route GCS -> dst over the radios in pos {name: xy} (GCS included).
        Fewest hops whose weakest hop meets the requirement, ties -> strongest weakest
        hop; if none meets it, the widest route. Returns (weakest hop dBm, [GCS, ..., dst])."""
        names = [n for n in pos if n not in (self.G, dst)]
        L = {}
        def link(a, b):
            k = (a, b) if a < b else (b, a)
            if k not in L:
                L[k] = self.pred.link(a, pos[a], b, pos[b])
            return L[k]
        best = None
        from itertools import permutations
        for h in range(len(names) + 1):
            for mid in permutations(names, h):
                path = [self.G, *mid, dst]
                w = min(link(a, b) for a, b in zip(path[:-1], path[1:]))
                if best is None or w > best[0]:
                    best = (w, path)
            if best[0] >= self.p.req:                 # fewest hops that work
                break
        return best

    def widest(self, pos: dict) -> dict:
        """{name: weakest hop of the widest route GCS -> name} over the radios in pos."""
        W = {n: -np.inf for n in pos}
        W[self.G] = np.inf
        todo = set(pos)
        while todo:
            a = max(todo, key=lambda n: W[n])
            todo.discard(a)
            for b in todo:
                W[b] = max(W[b], min(W[a], self.pred.link(a, pos[a], b, pos[b])))
        return W

    def region(self, pos: dict, client: str) -> np.ndarray:
        """Margin map vs the planning requirement: >= 0 where the client, at that cell,
        has a route meeting Pr[every hop >= eta] >= theta through the radios in pos
        (GCS + relays, client excluded)."""
        W = self.widest(pos)
        m = np.full(self.grid.shape, -np.inf)
        for n, xy in pos.items():
            if np.isfinite(W[n]) or n == self.G:
                m = np.maximum(m, np.minimum(self.pred.link_map(n, xy, client), W[n]))
        return m - self.p.req

    def access(self, pos: dict, client: str, x) -> float:
        """Best-route weakest hop for the client at x through the radios in pos."""
        W = self.widest(pos)
        return max(min(W[n], self.pred.link(n, xy, client, x)) for n, xy in pos.items())

    def in_region(self, pos: dict, client: str, goal) -> bool:
        return self.access(pos, client, goal) >= self.p.req

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
    def mcts(self, name: str, start, target, anchors: dict, others=()):
        """Trajectory of `name` from start to target over t_f intervals that keeps
        Pr[route to GCS >= eta] >= theta at every t, and stays d_sep from every other
        robot: `others` are (t_f+1, 2) trajectories (NaN rows = position unknown).
        anchors {radio: (t_f+1, 2) trajectory}: the GCS and the robots already planned;
        `name` may attach to any of them (the paper: to its chain predecessor only)."""
        tf = self.p.t_f
        anchors = {n: np.asarray(tr, float) for n, tr in anchors.items()}
        Wt = [self.widest({n: tuple(tr[t]) for n, tr in anchors.items()}) for t in range(tf + 1)]

        def route_P(t, x):
            return max(min(Wt[t][n], self.pred.link(n, tuple(tr[t]), name, x)) for n, tr in anchors.items())
        # a robot already below the requirement where it stands (a relay drifted, say) may still
        # move -- as long as it never gets worse than now: recovery towards coverage  (ours)
        need = min(self.p.req, route_P(0, tuple(start)) - 0.5)
        others = [np.asarray(o, float) for o in others]

        def passable(o):                  # standing still where there is room to drive around it
            f = o[np.isfinite(o).all(axis=1)]
            if not len(f) or np.ptp(f, axis=0).max() > 0.05:
                return False
            r_, c_ = self.pred.cell(tuple(f[0]))
            return self.clear[r_, c_] >= self.p.pass_clear
        others = [o for o in others if not passable(o)]
        if math.dist(start, target) < 0.3:
            stay = np.repeat([start], tf + 1, axis=0)
            return stay, np.zeros(tf), True
        avoid = {tuple(np.round(o[k], 2)) for o in others for k in (0, -1) if np.isfinite(o[k]).all()}
        routes = self.rg.routes(start, target, k=self.p.n_routes, avoid=list(avoid), avoid_r=self.p.d_sep,
                                allowed=self.allowed, max_ratio=self.p.route_ratio)
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
                        # standing still this interval with room around it: Nav2 drives around it, as
                        # passable() allows for a robot still for the whole plan. A relay at its spot in
                        # the corridor to the south classroom walled it off (real_full_live6)  (ours)
                        if math.dist(o0, o[t]) < 0.05:
                            r_, c_ = self.pred.cell(tuple(o[t]))
                            if self.clear[r_, c_] >= self.p.pass_clear:
                                continue
                        for f, a_ in zip(fs, mine):
                            b_ = (o0[0] + f * (o[t][0] - o0[0]), o0[1] + f * (o[t][1] - o0[1]))
                            if math.dist(a_, b_) < thr:
                                clash = True; break
                        if clash:
                            break
                if self.allowed is not None and t > 0 and not clash:
                    rr, cc = self.grid.world_to_cell(*x)
                    clash = not self.allowed[rr, cc]           # robots cannot plan through unknown space
                P = -np.inf if clash else route_P(t, x)
                qcache[key] = (1.0 - remaining(j, i) / dhat) if P >= need else -1.0
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
        N, Wv, kids = {root: 0}, {root: 0.0}, {}
        for _ in range(self.p.B_mcts):
            path, node = [root], root
            # selection
            while node in kids and kids[node] and all(c in N for c in kids[node]):
                n_par = max(N[node], 1)
                node = max(kids[node], key=lambda c: Wv[c] / N[c] +
                           self.p.C_p * math.sqrt(2 * math.log(n_par) / N[c]))
                path.append(node)
            # expansion
            if node not in kids:
                kids[node] = children(*node)
            untried = [c for c in kids[node] if c not in N]
            if untried:
                node = untried[self.rng.integers(len(untried))]
                N[node], Wv[node] = 0, 0.0
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
                N[n_] += 1; Wv[n_] += v
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
        ok = all(Q(j, i, t) > -1 for j, i, t in seq[1:])
        if not ok:
            # the search is random: a long route that works only if driven straight through can
            # be missed. Check each candidate driven straight through, shortest first.  (ours)
            for j in sorted(range(len(W)), key=lambda j: Ls[j]):
                straight = [(j, t, t) for t in range(1, tf + 1)]
                if all(Q(*n_) > -1 for n_ in straight):
                    seq, ok = [root] + straight, True
                    break
        traj = np.array([pos(j, i) for j, i, _ in seq])
        arc = np.array([0.0 if j is None else S[j][i] for j, i, _ in seq])
        return traj, np.diff(arc), ok

    # ------------------------------------------------------------- top level
    # If MCTS cannot keep every step of a move above the requirement, retry with more
    # candidate routes and finer steps; a move that still cannot is NOT executed.
    # The last rung also allows long detours (up to 3x the shortest route): the way back along
    # the chain a client came by is often far longer than an uncovered shortcut.
    LADDER = ((3, 5, 1.6), (5, 5, 1.6), (5, 8, 1.6), (6, 12, 1.6), (6, 12, 3.0))   # (|J|, t_f, ratio) -- ours

    def _tries(self):
        keep = (self.p.n_routes, self.p.t_f, self.p.route_ratio)
        try:
            for nr, tf, ratio in self.LADDER:
                self.p.n_routes, self.p.t_f, self.p.route_ratio = nr, tf, ratio
                yield tf
        finally:
            self.p.n_routes, self.p.t_f, self.p.route_ratio = keep

    def plan(self, goal, client: Robot, relays: list[Robot], engaged: list[str] = (), static: dict = None) -> Plan:
        """relays: the relays the planner may move. engaged: relay names placed so far
        (only used with keep_deployed). static {name: xy}: radios that forward but do
        not move (e.g. a relay whose Nav2 is down)."""
        goal = tuple(goal)
        byname = {r.name: r for r in relays}
        static = dict(static or {})
        here = {self.G: self.gcs, **static, **{r.name: r.xy for r in relays}}   # network as it stands
        why = "no feasible plan with %d relays" % len(relays)

        d = self.p.d_sep
        blocked = [n for n, xy in here.items() if n != self.G and math.dist(xy, goal) < d]
        if blocked:
            why = f"goal within {d} m of {', '.join(blocked)}"
        inside = self.in_region(here, client.name, goal)
        # where the relays are needed: around the goal (paper) -- or, when the goal itself is
        # covered and only the way there is not, along the way: place on GCS -> client  (ours)
        cover = tuple(client.xy) if inside else goal
        if inside and not blocked:
            for tf in self._tries():
                anchors = {n: np.repeat([xy], tf + 1, axis=0) for n, xy in here.items()}
                parked = [v for n, v in anchors.items() if n != self.G]
                traj, seg, ok = self.mcts(client.name, client.xy, goal, anchors, others=parked)
                if not ok:
                    continue
                plan = Plan("client_only", goal, [], {client.name: goal})
                plan.traj[client.name], plan.seg[client.name] = traj, seg
                for n, xy in here.items():
                    if n != self.G:
                        plan.traj[n] = np.repeat([xy], tf + 1, axis=0); plan.seg[n] = np.zeros(tf)
                plan.note = f"goal inside the bounded region (t_f {tf})"
                self._fill_hops(plan, client.name)
                return plan
            why = "goal inside the region but no client trajectory keeps every step"
            # fall through: a relay may be able to cover the way there

        R = len(relays)
        engaged = [n for n in engaged if n in byname]
        # fewest relays first; keep_deployed: placed relays stay, one more is added
        ks = range(len(engaged) + 1, R + 1) if self.p.keep_deployed else range(1, R + 1)
        for k in ks:
            if self.p.keep_deployed:
                fixed = [(n, byname[n].xy) for n in engaged]
                free = [r for r in relays if r.name not in engaged]
                spots, hops, G = self.place(k - len(engaged), cover, client.name,
                                            [r.name for r in free[:k - len(engaged)]], fixed)
                pool = free
            else:
                spots, hops, G = self.place(k, cover, client.name, [r.name for r in relays[:k]])
                pool = relays
            if spots is None:
                continue
            ass = self.assign(pool, spots)
            order = sorted(ass, key=lambda n: ass[n])           # deployment sequence s_l
            if self.p.keep_deployed:
                order = engaged + order
                spots = [byname[n].xy for n in engaged] + spots
                ass = {n: i for i, n in enumerate(order)}
            # placement feasibility with the ASSIGNED names (shadowing is per radio): the
            # client at the goal must have a route through the final positions
            def at_goal(ass):
                final = {self.G: self.gcs, **static, **{r.name: r.xy for r in relays}}
                final.update({n: spots[i] for n, i in ass.items()})
                return self.route({**final, client.name: goal}, client.name)
            w_final, route_final = at_goal(ass)
            if w_final < self.p.req and not self.p.keep_deployed:
                # spots were placed for one name order and the shortest-travel assignment may swap
                # them; with per-radio shadowing the swap can cost a hop several dB (real_full_live6:
                # -79.3 placed, -84.5 swapped, the south classroom skipped for an hour). Try the rest,
                # least travel first.  (ours)
                from itertools import permutations
                alts = [dict(zip(p_, range(len(spots)))) for p_ in permutations([r.name for r in pool], len(spots))]
                alts.sort(key=lambda a_: sum(self._travel(byname[n].xy, spots[i]) for n, i in a_.items()))
                for alt in alts:
                    w_, r_ = at_goal(alt)
                    if w_ >= self.p.req:
                        ass, w_final, route_final = alt, w_, r_
                        order = sorted(ass, key=lambda n: ass[n])
                        break
            if w_final < self.p.req:
                continue
            idle = [r for r in relays if r.name not in order]
            fixed_pts = [spots[ass[n]] for n in order] + [goal] + [r.xy for r in idle] + list(static.values())
            if any(math.dist(p_, q_) < d for i_, p_ in enumerate(fixed_pts) for q_ in fixed_pts[i_ + 1:]):
                why = f"{k} relay(s): relay spots / goal / parked robots closer than {d} m"
                continue
            for tf in self._tries():
                def until1(xy):                                 # a robot that will move later:
                    o = np.full((tf + 1, 2), np.nan)            # only known for the first step
                    o[:2] = xy
                    return o
                stay = {n: np.repeat([xy], tf + 1, axis=0) for n, xy in static.items()}
                stay.update({r.name: np.repeat([r.xy], tf + 1, axis=0) for r in idle})
                anchors = {self.G: np.repeat([self.gcs], tf + 1, axis=0), **stay}
                planned = list(stay.values())
                plan = Plan("deploy", goal, [])
                all_ok = True
                for n in order:                                 # sequential, GCS outwards
                    tgt = spots[ass[n]]
                    later = [until1(byname[m].xy) for m in order if m != n and m not in plan.traj] + \
                            [until1(client.xy)]
                    traj, seg, ok = self.mcts(n, byname[n].xy, tgt, anchors, others=planned + later)
                    planned.append(traj)
                    anchors[n] = traj
                    plan.targets[n], plan.traj[n], plan.seg[n] = tgt, traj, seg
                    all_ok &= ok
                    if not all_ok:
                        break
                if not all_ok:
                    continue
                traj, seg, ok = self.mcts(client.name, client.xy, goal, anchors, others=planned)
                staged = False
                if not ok and all(math.dist(p_, client.xy) >= d for n in order for p_ in plan.traj[n]):
                    # Staged: the relays first while the client holds still, then the client through
                    # them standing at their spots. Moving together, the client tails a relay through
                    # the same doorway (a clash), and waiting it loses the relay it was using
                    # (real_full_live6: the south classroom, reachable, skipped for an hour). The client
                    # may dip below the requirement only while it stands still.  (ours)
                    fin = {n: np.repeat([plan.targets[n]], tf + 1, axis=0) for n in order}
                    traj, seg, ok = self.mcts(client.name, client.xy, goal, {**anchors, **fin},
                                              others=list(stay.values()) + list(fin.values()))
                    staged = ok
                if not ok:
                    continue
                if staged:
                    for n in order:
                        plan.traj[n] = np.vstack([plan.traj[n], fin[n][1:]])
                        plan.seg[n] = np.concatenate([plan.seg[n], np.zeros(tf)])
                    traj = np.vstack([np.repeat([client.xy], tf + 1, axis=0), traj[1:]])
                    seg = np.concatenate([np.zeros(tf), seg])
                    stay = {n: np.repeat(tr[:1], 2 * tf + 1, axis=0) for n, tr in stay.items()}
                plan.targets[client.name], plan.traj[client.name], plan.seg[client.name] = goal, traj, seg
                for n, tr in stay.items():                      # idle relays / static radios stay put
                    plan.traj[n] = tr; plan.seg[n] = np.zeros(len(seg))
                plan.note = (f"{k} relay(s) moved (t_f {tf}{', staged: client waits for the relays' if staged else ''}); "
                             f"final route {' > '.join(route_final)}, weakest hop {w_final:.1f}")
                self._fill_hops(plan, client.name)
                return plan
            why = f"{k} relay(s) could be placed but no trajectories keep every step"

        plan = Plan("skip", goal, self.route({**here, client.name: client.xy}, client.name)[1], note=why)
        return plan

    def _fill_hops(self, plan: Plan, client: str):
        """Per step: the route the network would use at that moment and its hops;
        plan.chain = the route at the goal (last step)."""
        for t in range(len(next(iter(plan.traj.values())))):
            pos = {self.G: self.gcs, **{n: tuple(tr[t]) for n, tr in plan.traj.items()}}
            _, path = self.route(pos, client)
            plan.hops.append([(a, b, self.pred.link(a, pos[a], b, pos[b])) for a, b in zip(path[:-1], path[1:])])
            plan.chain = path


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
