#!/usr/bin/env python3
"""Replay a recorded live run as a video: the client's map growing, robots, the route, its links.

    .venv/bin/python rssi/replay_live_run.py rssi/out/live_runs/<run>.csv <run>_coordinator.log \\
        --gt hospital_world/gt/hospital_gt --gcs 0 14 --P0 -45 [--maps <run>_maps] [--fps 8]

Positions are Gazebo ground truth (rssi/ros/live_logger.sh). With --maps (the coordinator's
--map-log snapshots) the background is the map chain_robot has built so far (white free, black
occupied, grey unknown; the building's true walls faint underneath) and the links are the route
the coordinator saw at that moment; without it, the GT floor plan and the last decided chain.
Each link is coloured by the model's predicted RSSI (weaker direction): green > -65,
yellow -65..-85, red < -85 dBm (paper Fig. 9). Writes <run>_replay.mp4 next to the CSV.
"""
import argparse, csv, glob, json, os, re, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from propagation import GridMap, PropagationParams          # noqa: E402
from network import NoiseParams                              # noqa: E402
from relay_planner import Predictor                          # noqa: E402
from phase1_heatmap import walls_overlay                     # noqa: E402

COL = {"head": "#d62728", "relay1": "#1f77b4", "relay2": "#2ca02c"}
CSV_KEY = {"head": "head", "relay1": "relay1", "relay2": "relay2"}


def tod(s):                         # "HH:MM:SS" -> seconds of the day
    h, m, x = (int(v) for v in s.split(":"))
    return h * 3600 + m * 60 + x


def sod(epoch):                      # epoch -> seconds of the (local) day
    t = time.localtime(int(epoch))
    return t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec


def decisions(log):
    """[(seconds of day, chain list, text, goal xy, decision)] from the coordinator's decision lines."""
    out = []
    for line in open(log):
        m = re.search(r"\[(\d{10})\.\d+\].*?goal world \(([-\d.]+), ([-\d.]+)\): (\w+)(?: \[([^\]]*)\])?", line)
        rg = re.search(r"\[(\d{10})\.\d+\].*?REGROUP after \d+ skips: client back to \(([-\d.]+), ([-\d.]+)\): "
                       r"(CLIENT_ONLY|DEPLOY) \[([^\]]*)\]", line)
        if rg:                              # the client returns to the GCS to re-plan from there
            g = (float(rg.group(2)), float(rg.group(3)))
            out.append((sod(rg.group(1)), [c.strip() for c in rg.group(5).split(">")],
                        f"REGROUP: back to ({g[0]:.1f}, {g[1]:.1f})", g, rg.group(4)))
            continue
        if not m:
            continue
        chain = [c.strip() for c in m.group(5).split(">")] if m.group(5) else None
        g = (float(m.group(2)), float(m.group(3)))
        out.append((sod(m.group(1)), chain, f"{m.group(4)} goal ({g[0]:.1f}, {g[1]:.1f})", g, m.group(4)))
    return out


def snapshots(d):
    """[(seconds of day, npz path)] of the coordinator's /map snapshots, in time order."""
    fs = sorted(glob.glob(os.path.join(d, "map_*.npz")))
    return [(sod(int(re.search(r"map_(\d+)", f).group(1))), f) for f in fs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv"); ap.add_argument("log")
    ap.add_argument("--gt", default="hospital_world/gt/hospital_gt")
    ap.add_argument("--gcs", nargs=2, type=float, default=[0.0, 14.0])
    ap.add_argument("--P0", type=float, default=-45.0)
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--maps", default="", help="the coordinator's --map-log directory")
    a = ap.parse_args()
    root = os.path.dirname(HERE)
    gt = a.gt if os.path.isabs(a.gt) else os.path.join(root, a.gt)
    g = GridMap.load(gt + ".npy", gt + ".json")
    pred = Predictor(g, PropagationParams(P0=a.P0), NoiseParams())
    rows = [r for r in csv.DictReader(open(a.csv)) if r.get("head_x")]
    dec = decisions(a.log)
    snaps = snapshots(a.maps) if a.maps else []
    gcs = tuple(a.gcs)

    e = g.extent()
    wide = (e[1] - e[0]) > (e[3] - e[2])
    fig, ax = plt.subplots(figsize=(14, 5.6) if wide else (7.5, 12.5))
    if snaps:
        from matplotlib.colors import ListedColormap
        ax.set_facecolor("#c9c9c9")                                 # unknown
        ax.imshow(np.where(g.occ.astype(bool), 1.0, np.nan), origin="lower", extent=e, zorder=1,
                  cmap=ListedColormap(["#a8a8a8"]), interpolation="nearest")   # true walls, faint
        live = ax.imshow(np.full((2, 2), np.nan), origin="lower", extent=e, zorder=2, interpolation="nearest",
                         cmap=ListedColormap(["#ffffff", "#111111"]), vmin=0, vmax=1)
        from scipy.ndimage import distance_transform_edt
        from propagation import geodesic_distance
        first = next(r for r in rows if r.get("head_x"))           # reachable floor, as explore_sim scores it
        reach = np.isfinite(geodesic_distance(g, g.world_to_cell(float(first["head_x"]), float(first["head_y"])))) \
            & (distance_transform_edt(g.free) * g.res >= 0.45)
        H_, W_ = g.shape
        gx = g.origin[0] + (np.arange(W_) + 0.5) * g.res
        gy = g.origin[1] + (np.arange(H_) + 0.5) * g.res
        state = {"snap": None}
        ax.set_xlim(e[0], e[1]); ax.set_ylim(e[2], e[3])
    else:
        ax.imshow(np.where(g.free, 1.0, np.nan), origin="lower", extent=e, cmap="Greys", vmin=0, vmax=4)
        walls_overlay(ax, g)
    goal_mk = ax.plot([], [], "*", ms=17, color="#ff7f0e", mec="k", zorder=9)[0]
    ax.plot(*gcs, "*", ms=16, mfc="white", mec="k", zorder=8)
    ax.annotate("GCS", gcs, xytext=(6, 6), textcoords="offset points", fontsize=8, fontweight="bold")
    trails = {n: ax.plot([], [], "-", color=c, lw=1.2, alpha=0.6, zorder=5)[0] for n, c in COL.items()}
    dots = {n: ax.plot([], [], "D" if n == "head" else "o", ms=9, color=c, mec="k", zorder=9)[0]
            for n, c in COL.items()}
    links = [ax.plot([], [], "-", lw=3, zorder=7)[0] for _ in range(4)]
    labels = [ax.text(0, 0, "", fontsize=8, ha="center", zorder=10,
                      bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85)) for _ in range(4)]
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], ls="", marker="D", color=COL["head"], mec="k", label="chain_robot (client)"),
                       Line2D([], [], ls="", marker="o", color=COL["relay1"], mec="k", label="relay1"),
                       Line2D([], [], ls="", marker="o", color=COL["relay2"], mec="k", label="relay2"),
                       Line2D([], [], ls="", marker="*", ms=12, color="#ff7f0e", mec="k", label="current goal"),
                       Line2D([], [], color="#2ca02c", lw=3, label="link > -65 dBm"),
                       Line2D([], [], color="#e6b800", lw=3, label="link -65..-85"),
                       Line2D([], [], color="#d62728", lw=3, label="link < -85")],
              loc="upper left", bbox_to_anchor=(1.005, 1.0), fontsize=8, framealpha=0.9)
    title = ax.set_title("", fontsize=10)
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    fig.tight_layout(rect=[0, 0, 0.88 if wide else 1, 0.94])        # room for the two-line title

    def pos(r, n):
        x, y = r.get(f"{CSV_KEY[n]}_x"), r.get(f"{CSV_KEY[n]}_y")
        return (float(x), float(y)) if x not in (None, "") else None

    def frame(i):
        r = rows[i]
        t = tod(r["wall_time"])
        past = [d for d in dec if d[0] <= t]
        chain = next((d[1] for d in reversed(past) if d[1]), ["gcs", "head"])
        what = past[-1][2] if past else "waiting for the first goal"
        acc = next((d for d in reversed(past) if d[4] in ("CLIENT_ONLY", "DEPLOY")), None)
        goal_mk.set_data(([acc[3][0]], [acc[3][1]]) if acc else ([], []))
        explored = ""
        if snaps:
            cur = [s_ for s_ in snaps if s_[0] <= t]
            if cur and cur[-1][1] != state["snap"]:
                z = np.load(cur[-1][1])
                state["snap"] = cur[-1][1]
                G = z["grid"]; res = float(z["res"]); ox, oy = (z["origin"] + z["off"])
                img = np.where(G < 0, np.nan, np.where(G >= 50, 1.0, 0.0))
                live.set_data(img)
                live.set_extent((ox, ox + G.shape[1] * res, oy, oy + G.shape[0] * res))
                cc = np.floor((gx - ox) / res).astype(int); rr = np.floor((gy - oy) / res).astype(int)
                ins = ((rr >= 0) & (rr < G.shape[0]))[:, None] & ((cc >= 0) & (cc < G.shape[1]))[None, :]
                v = G[np.clip(rr, 0, G.shape[0] - 1)[:, None], np.clip(cc, 0, G.shape[1] - 1)[None, :]]
                state["pct"] = 100 * float((ins & (v >= 0) & (v < 50) & reach).sum()) / reach.sum()
                state["route"] = json.loads(str(z["route"]))
            if state.get("route"):
                chain = state["route"]
            explored = f"   |   explored: {state.get('pct', 0.0):.0f}% of the reachable floor"
        for n in COL:
            pts = [pos(q, n) for q in rows[: i + 1] if pos(q, n)]
            if pts:
                trails[n].set_data([p[0] for p in pts], [p[1] for p in pts])
                dots[n].set_data([pts[-1][0]], [pts[-1][1]])
        names = ["GCS" if c.lower() == "gcs" else c for c in chain]
        xy = {"GCS": gcs, **{n: pos(r, n) for n in COL}}
        for k, ln in enumerate(links):
            if k < len(names) - 1 and xy.get(names[k]) and xy.get(names[k + 1]):
                a_, b_ = names[k], names[k + 1]
                P = pred.link("gcs" if a_ == "GCS" else a_, xy[a_], b_, xy[b_])
                col = "#2ca02c" if P > -65 else ("#e6b800" if P >= -85 else "#d62728")
                ln.set_data([xy[a_][0], xy[b_][0]], [xy[a_][1], xy[b_][1]]); ln.set_color(col)
                labels[k].set_position(((xy[a_][0] + xy[b_][0]) / 2, (xy[a_][1] + xy[b_][1]) / 2))
                labels[k].set_text(f"{P:.0f}")
            else:
                ln.set_data([], []); labels[k].set_text("")
        title.set_text(f"{os.path.basename(a.csv)}   {r['wall_time']}   RTF {r.get('rtf') or '-'}{explored}\n"
                       f"route {' > '.join(names)}   |   last decision: {what}")
        return list(dots.values()) + links + labels + [title]

    import imageio_ffmpeg
    plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    out = os.path.splitext(a.csv)[0] + "_replay.mp4"
    anim = animation.FuncAnimation(fig, frame, frames=len(rows), blit=False)
    anim.save(out, writer=animation.FFMpegWriter(fps=a.fps, bitrate=2400), dpi=90)
    print("wrote", out, f"({len(rows)} frames, {len(dec)} decisions)")


if __name__ == "__main__":
    main()
