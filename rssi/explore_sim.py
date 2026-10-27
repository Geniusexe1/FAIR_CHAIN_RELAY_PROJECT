#!/usr/bin/env python3
"""Offline communication-aware exploration on the hospital GT map.

    .venv/bin/python rssi/explore_sim.py --P0 -30 --tag paperP0
    .venv/bin/python rssi/explore_sim.py --P0 -60.8 --tag measuredP0

The client explores frontier by frontier until nothing is left to explore. For each
frontier, nearest first:
  - inside the client's bounded region (current topology)   -> client goes alone
  - outside                                                 -> deploy one more relay
                                                               (relay_planner, the paper's method)
  - outside even with every relay engaged (and re-placed,
    unless --keep-deployed)                                 -> skip it, try the next one
Robots move in t_f synchronised intervals: in each one every robot covers its own
segment and they all arrive at their next waypoint together.

Writes rssi/out/explore/<tag>/
  frame_XXX.png   one per decision: explored map + plan | bounded-region margin map
  summary.png     coverage, chain RSSI and relay count over time
  events.csv      every decision, its timing and the predicted / simulated chain RSSI
"""
import argparse, csv, os, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, TwoSlopeNorm
from matplotlib.lines import Line2D
from scipy.ndimage import binary_dilation, distance_transform_edt, label

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap, PropagationParams, geodesic_distance, line_of_sight_mask  # noqa: E402
from network import NoiseParams                                                           # noqa: E402
from roadgraph import RoadGraph, length, point_at                                                # noqa: E402
from relay_planner import Predictor, RelayPlanner, PlannerParams, Robot, schedule         # noqa: E402
from margin import CBAR_LABEL, STRONG_DBM, _contour_field                                  # noqa: E402

GT = os.path.join(os.path.dirname(HERE), "hospital_world", "gt", "hospital_gt")
UNKNOWN, FREE, OCC = -1, 0, 1
STYLE = {"GCS": ("*", 15, "white"), "relay": ("o", 9, "#1f77b4"), "client": ("D", 10, "#d62728")}


class Explorer:
    def __init__(self, a):
        self.a = a
        self.g = GridMap.load(GT + ".npy", GT + ".json")
        self.rg = RoadGraph.build(self.g, cache_dir=os.path.join(HERE, "cache"))
        self.pred = Predictor(self.g, PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha),
                              NoiseParams(sigma_fast=a.sigma_n))
        self.pp = PlannerParams(sigma=a.sigma_n, keep_deployed=a.keep_deployed, B_mcts=a.mcts)
        self.pl = RelayPlanner(self.g, self.rg, self.pred, tuple(a.gcs), self.pp, seed=a.seed)
        self.rng = np.random.default_rng(a.seed)
        self.client = Robot("head", tuple(a.client), "client", a.v_client)
        self.relays = [Robot(f"relay{i + 1}", tuple(xy), "relay", a.v_relay)
                       for i, xy in enumerate(np.array(a.relays).reshape(-1, 2))]
        self.engaged = []
        self.known = np.full(self.g.shape, UNKNOWN, np.int8)
        self.clear = distance_transform_edt(self.g.free) * self.g.res
        reach = np.isfinite(geodesic_distance(self.g, self.g.world_to_cell(*self.client.xy)))
        self.reachable = reach & (self.clear >= a.clearance)
        self.blacklist = np.zeros(self.g.shape, bool)
        self.t = 0.0
        self.log, self.trace = [], []          # decisions; (time, coverage, chain RSSI, n relays)
        self.out = os.path.join(HERE, "out", "explore", a.tag)
        os.makedirs(self.out, exist_ok=True)

    # ------------------------------------------------------------- sensing
    def sense(self, xy):
        rc = self.g.world_to_cell(*xy)
        vis = line_of_sight_mask(self.g, rc)
        H, W = self.g.shape
        rr, cc = np.ogrid[0:H, 0:W]
        near = (rr - rc[0]) ** 2 + (cc - rc[1]) ** 2 <= (self.a.sensor / self.g.res) ** 2
        vis &= near
        self.known[vis] = FREE
        walls = binary_dilation(vis, np.ones((3, 3), bool)) & self.g.occ.astype(bool) & near
        self.known[walls] = OCC

    def coverage(self) -> float:
        return float((self.known[self.reachable] == FREE).mean())

    # ----------------------------------------------------------- frontiers
    def frontiers(self):
        free = self.known == FREE
        unk = self.known == UNKNOWN
        edge = free & binary_dilation(unk, [[0, 1, 0], [1, 1, 1], [0, 1, 0]]) & ~self.blacklist
        lab, n = label(edge, np.ones((3, 3)))
        d = geodesic_distance(self.g, self.g.world_to_cell(*self.client.xy))
        out = []
        for i in range(1, n + 1):
            cells = np.argwhere(lab == i)
            if len(cells) < self.a.min_frontier:
                continue
            ok = cells[self.clear[cells[:, 0], cells[:, 1]] >= self.a.clearance]
            if not len(ok):
                ok = cells
            cen = cells.mean(axis=0)
            r, c = ok[np.argmin(((ok - cen) ** 2).sum(1))]
            # goal: the nearest roomy, already-known free cell (frontier cells hug walls)
            win = (slice(max(r - 20, 0), r + 21), slice(max(c - 20, 0), c + 21))
            cand = np.argwhere((self.known[win] == FREE) & (self.clear[win] >= self.a.clearance))
            if len(cand):
                cand += [win[0].start, win[1].start]
                r, c = cand[np.argmin(((cand - [r, c]) ** 2).sum(1))]
            if not np.isfinite(d[r, c]) or d[r, c] < 0.5:
                self.blacklist[lab == i] = True
                continue
            out.append(dict(rc=(r, c), xy=tuple(float(v) for v in self.g.cell_to_world(r, c)),
                            dist=float(d[r, c]), size=len(cells), label=i, lab=lab))
        out.sort(key=lambda f: f["dist"])
        return out

    # ----------------------------------------------------------- execution
    def chain_rssi(self, chain, pos, noisy: bool):
        vals = []
        for a, b in zip(chain[:-1], chain[1:]):
            P = self.pred.link(a, pos[a], b, pos[b])
            if noisy:
                P = min(P + self.rng.normal(0, self.a.sigma_n), P + self.rng.normal(0, self.a.sigma_n))
            vals.append(P)
        return min(vals)

    def execute(self, plan):
        robots = {r.name: r for r in self.relays + [self.client]}
        sched = schedule(plan, {n: robots[n].vmax for n in plan.seg})
        tf = self.pp.t_f
        cr_ok = cr_n = 0
        for t in range(1, tf + 1):
            dt = sched[t - 1]["dt"]
            # sub-sample the interval: every robot moves along its road-graph leg
            legs = {}
            for n in plan.traj:
                a_, b_ = tuple(plan.traj[n][t - 1]), tuple(plan.traj[n][t])
                R = self.rg.routes(a_, b_, k=1) if np.hypot(*np.subtract(a_, b_)) > 0.05 else []
                legs[n] = R[0] if R else np.array([a_, b_])
            nsub = max(2, int(np.ceil(max(length(L) for L in legs.values()) / 0.5)))
            for k in range(1, nsub + 1):
                f = k / nsub
                pos = {"GCS": tuple(self.a.gcs)}
                for n, L in legs.items():
                    s = f * length(L)
                    pos[n] = tuple(point_at(L, s))
                for n in legs:
                    robots[n].xy = pos[n]
                if k % 2 == 0 or k == nsub:
                    self.sense(self.client.xy)
                P = self.chain_rssi(plan.chain, pos, noisy=True)
                cr_n += 1; cr_ok += P >= self.pp.eta
                self.trace.append((self.t + f * dt, self.coverage(), P, len(self.engaged)))
            self.t += dt
        return sched, cr_ok, cr_n

    # ------------------------------------------------------------- drawing
    def draw(self, k, plan, front, skipped):
        g = self.g
        fig, axs = plt.subplots(1, 2, figsize=(15, 13))
        ax = axs[0]
        img = np.where(self.known == UNKNOWN, 0.55, np.where(self.known == FREE, 1.0, 0.0))
        ax.imshow(img, origin="lower", extent=g.extent(), cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        ax.imshow(np.where(g.occ.astype(bool) & (self.known == UNKNOWN), 1.0, np.nan), origin="lower",
                  extent=g.extent(), cmap=ListedColormap(["#8a8a8a"]), interpolation="nearest")
        for f in front:
            ax.plot(*f["xy"], "o", ms=4, color="#e6b800", mec="k", mew=0.4, zorder=5)
        for xy in skipped:
            ax.plot(*xy, "x", ms=8, color="#d62728", mew=2, zorder=6)
        ax.set_title(f"decision {k}  t = {self.t:.0f} s  |  explored {100 * self.coverage():.1f}% of reachable floor\n"
                     f"{plan.decision.upper()}: {plan.note}", fontsize=9)
        ax2 = axs[1]
        names = ["GCS"] + self.engaged
        chain_xy = [("GCS", tuple(self.a.gcs))] + [(n, next(r.xy for r in self.relays if r.name == n))
                                                   for n in self.engaged]
        m = self.pl.region(chain_xy, self.client.name)
        m = np.where(g.occ.astype(bool), np.nan, m)
        norm = TwoSlopeNorm(vmin=-30, vcenter=0, vmax=25)
        im = ax2.imshow(m, origin="lower", extent=g.extent(), cmap="RdYlBu", norm=norm, interpolation="nearest")
        ax2.imshow(np.where(g.free & ~np.isfinite(m), 1.0, np.nan), origin="lower", extent=g.extent(),
                   cmap=ListedColormap(["#b8b8b8"]), interpolation="nearest")
        H, W = g.shape
        xs = g.origin[0] + (np.arange(W) + 0.5) * g.res
        ys = g.origin[1] + (np.arange(H) + 0.5) * g.res
        cf = _contour_field(m, g)
        for lvl, ls in [(0.0, "-"), (STRONG_DBM - self.pp.req, (0, (5, 3)))]:
            if cf.min() < lvl < cf.max():
                ax2.contour(xs, ys, cf, levels=[lvl], colors="black", linewidths=1.5, linestyles=[ls], zorder=4)
        ax2.imshow(g.occ, origin="lower", extent=g.extent(),
                   cmap=ListedColormap([(0, 0, 0, 0), (0.08, 0.08, 0.1, 1)]), interpolation="nearest", zorder=3)
        fig.colorbar(im, ax=ax2, fraction=0.035, pad=0.02).set_label(
            CBAR_LABEL + f"\n(relative to {self.pp.req:.1f} dBm: Pr[link >= {self.pp.eta:g}] >= {self.pp.theta})")
        ax2.set_title(f"bounded region of {self.client.name} on {names[-1]} after this step "
                      f"(solid: Pr >= theta, dashed: {STRONG_DBM:g} dBm)", fontsize=9)
        for a_ in (ax, ax2):
            self._robots(a_, plan, names)
            a_.set_xlabel("world x (m)"); a_.set_ylabel("world y (m)")
        ax.legend(handles=[Line2D([], [], color="k", lw=1, ls=":", marker="o", ms=3, label="planned trajectory, t = 0..t_f"),
                           Line2D([], [], ls="", marker="o", color="#e6b800", mec="k", label="frontier"),
                           Line2D([], [], ls="", marker="x", color="#d62728", mew=2, label="skipped (out of comm reach)"),
                           Line2D([], [], ls="", marker="*", color="#d62728", ms=12, label="goal")],
                  loc="lower left", fontsize=7.5, framealpha=0.9)
        fig.tight_layout()
        fig.savefig(os.path.join(self.out, f"frame_{k:03d}.png"), dpi=70)
        plt.close(fig)

    def _robots(self, ax, plan, names):
        gcs = tuple(self.a.gcs)
        pos = {"GCS": gcs, **{r.name: r.xy for r in self.relays + [self.client]}}
        chain = names + [self.client.name]
        for a_, b_ in zip(chain[:-1], chain[1:]):
            P = self.pred.link(a_, pos[a_], b_, pos[b_])
            col = "#2ca02c" if P > -65 else ("#e6b800" if P >= self.pp.eta else "#d62728")
            ax.plot([pos[a_][0], pos[b_][0]], [pos[a_][1], pos[b_][1]], color=col, lw=2.5, zorder=6)
        for n, tr in plan.traj.items():
            if np.ptp(tr, axis=0).max() > 0.05:
                ax.plot(tr[:, 0], tr[:, 1], ":", color="k", lw=1, marker="o", ms=3, zorder=7)
        ax.plot(*plan.goal, "*", ms=14, color="#d62728", mec="k", zorder=8)
        ax.plot(*gcs, marker="*", ms=15, mfc="white", mec="k", mew=1.3, zorder=9)
        for r in self.relays + [self.client]:
            mk, ms, fc = STYLE[r.kind]
            idle = r.kind == "relay" and r.name not in self.engaged
            ax.plot(*r.xy, marker=mk, ms=ms, mfc="white" if idle else fc, mec="k", mew=1.2, zorder=9)
            ax.annotate(r.name + (" (idle)" if idle else ""), r.xy, xytext=(6, -3), textcoords="offset points",
                        fontsize=7, zorder=10, bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.75))

    # ---------------------------------------------------------------- loop
    def run(self):
        a = self.a
        self.sense(self.client.xy)
        k = 0
        wall0 = time.time()
        with open(os.path.join(self.out, "events.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["decision", "sim_time_s", "goal_x", "goal_y", "decision_type", "chain", "relays_engaged",
                        "pred_min_hop_dBm_per_step", "interval_s", "client_m", "relay_m", "CR_sim_%",
                        "coverage_%", "plan_s", "note"])
            while k < a.max_decisions:
                front = self.frontiers()
                if not front:
                    break
                plan, skipped = None, []
                for f in front[:a.max_rank]:
                    t0 = time.time()
                    p = self.pl.plan(f["xy"], self.client, self.relays, self.engaged)
                    pt = time.time() - t0
                    if p.decision == "skip":
                        skipped.append(f["xy"])
                        self.blacklist[f["lab"] == f["label"]] = True
                        w.writerow([k, f"{self.t:.1f}", *np.round(f["xy"], 2), "skip", ">".join(p.chain),
                                    len(self.engaged), "", "", "", "", "", f"{100 * self.coverage():.1f}",
                                    f"{pt:.1f}", p.note])
                        print(f"  skip  {np.round(f['xy'], 1)}  {p.note}")
                        continue
                    plan = p
                    break
                if plan is None:
                    continue                           # all tried were blacklisted; re-detect
                if plan.decision == "deploy":
                    self.engaged = [n for n in plan.chain if n.startswith("relay")]
                sched, ok, n = self.execute(plan)
                k += 1
                cov = 100 * self.coverage()
                predmin = [round(min(h[2] for h in row), 1) for row in plan.hops]
                moved = {r.name: float(sum(plan.seg[r.name])) for r in self.relays}
                w.writerow([k, f"{self.t:.1f}", *np.round(plan.goal, 2), plan.decision, ">".join(plan.chain),
                            len(self.engaged), predmin, [round(s["dt"], 1) for s in sched],
                            f"{sum(plan.seg[self.client.name]):.1f}",
                            {n_: round(v, 1) for n_, v in moved.items()},
                            f"{100 * ok / max(n, 1):.1f}", f"{cov:.1f}", f"{pt:.1f}", plan.note])
                fh.flush()
                print(f"[{k:3d}] t={self.t:7.1f}s  {plan.decision:11s} goal {np.round(plan.goal, 1)}  "
                      f"chain {'>'.join(plan.chain)}  CR {100 * ok / max(n, 1):5.1f}%  explored {cov:5.1f}%  "
                      f"({pt:.1f}s plan)")
                self.draw(k, plan, front, skipped)
        self.summary(time.time() - wall0)

    def summary(self, wall):
        tr = np.array(self.trace) if self.trace else np.zeros((1, 4))
        cr = 100 * np.mean(tr[:, 2] >= self.pp.eta)
        fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axs[0].plot(tr[:, 0], 100 * tr[:, 1]); axs[0].set_ylabel("explored (%)"); axs[0].grid(alpha=.3)
        axs[0].set_title(f"P0 {self.a.P0} dBm  |  final coverage {100 * tr[-1, 1]:.1f}% of reachable floor  |  "
                         f"channel reliability CR {cr:.1f}%  |  {len(self.relays)} relays", fontsize=10)
        axs[1].plot(tr[:, 0], tr[:, 2], lw=0.7, label="end-to-end chain RSSI (simulated reading)")
        axs[1].axhline(self.pp.eta, color="r", ls="--", lw=1, label=f"eta {self.pp.eta:g} dBm")
        axs[1].axhline(-65, color="g", ls=":", lw=1, label="-65 dBm")
        axs[1].set_ylabel("dBm"); axs[1].legend(fontsize=8); axs[1].grid(alpha=.3)
        axs[2].step(tr[:, 0], tr[:, 3], where="post"); axs[2].set_ylabel("relays engaged")
        axs[2].set_xlabel("sim time (s)"); axs[2].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(self.out, "summary.png"), dpi=90); plt.close(fig)
        print(f"\ndone: coverage {100 * tr[-1, 1]:.1f}% | CR {cr:.1f}% | sim time {self.t:.0f} s | "
              f"relays engaged {len(self.engaged)} | wall {wall:.0f} s\nwrote {os.path.relpath(self.out)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", default="run")
    ap.add_argument("--P0", type=float, default=-30.0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--sigma-n", type=float, default=3.0)
    ap.add_argument("--gcs", nargs=2, type=float, default=[0.0, 14.0])
    ap.add_argument("--client", nargs=2, type=float, default=[0.0, 12.0])
    ap.add_argument("--relays", nargs="+", type=float, default=[2.5, 12.0, 3.5, 12.0], help="x y x y ...")
    ap.add_argument("--v-client", type=float, default=0.7, help="m/s (chain_robot DWB max_vel_x)")
    ap.add_argument("--v-relay", type=float, default=0.5, help="m/s (relay MPPI vx_max)")
    ap.add_argument("--sensor", type=float, default=8.0, help="mapping range (m)")
    ap.add_argument("--clearance", type=float, default=0.45, help="goal clearance from obstacles (m)")
    ap.add_argument("--min-frontier", type=int, default=10, help="cells")
    ap.add_argument("--max-rank", type=int, default=5, help="frontiers tried per decision (explorer's MAX_FRONTIER_RANK)")
    ap.add_argument("--max-decisions", type=int, default=400)
    ap.add_argument("--mcts", type=int, default=300)
    ap.add_argument("--keep-deployed", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    Explorer(ap.parse_args()).run()


if __name__ == "__main__":
    main()
