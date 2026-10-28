#!/usr/bin/env python3
"""Live RSSI monitor: predicts every link between the GCS and the robots from
their CURRENT positions in the sim, using the dominant-path model (eq. 1).

    run in the fair_rssi container on ROS_DOMAIN_ID=42 -- see run_rssi_node.sh

Subscribes  <topic> (nav_msgs/Odometry) for each tracked node in nodes.yaml
Publishes
  /rssi/links                     std_msgs/String  JSON: every link, both clean and noisy
  /rssi/link/<a>__<b>             std_msgs/Float32 noisy RSSI of one link (dBm)
  /rssi/end_to_end/<node>         std_msgs/Float32 bottleneck RSSI of the best route
                                                   from <node> back to the GCS (eq. 5),
                                                   -inf if no route clears the threshold
  /rssi/chain                     std_msgs/Float32 paper eq. (5): weakest hop along the FIXED
                                                   chain in nodes.yaml order (GCS -> relays ->
                                                   client). -inf if any node is missing
  /rssi/markers                   visualization_msgs/MarkerArray  links for RViz
  /rssi/margin/at_client          std_msgs/Float32 client-to-GCS connectivity margin (dB) at
                                                   the client's actual position
  /rssi/margin/image              sensor_msgs/Image  the margin heat map (rgb8), re-rendered
                                                   only when something changed (see below)
  + <map-dir>/margin_latest.png   same image on disk

Margin map (margin.py): GCS and relays held fixed, the client placed virtually at
every cell, association FIXED (client -> --attach, default the node before it in
nodes.yaml), margin(x) = min(client@x <-> attach, weakest hop attach -> GCS) - eta,
weaker direction of every link. Zero contour = connected boundary, dashed contour
= --strong (end-to-end RSSI, dBm). Hop values are the EMA-filtered link estimates. The map is re-rendered
when a node moves more than --move-tol (0.5 m), a hop estimate moves by > --est-change dB, or
the topology (node set / client / attach) changes -- at most every --map-period (5) s.

Noise (when enabled): static spatially-correlated shadowing per transmitter plus
fresh N(0, sigma_n) per publication -- the "measurement" a radio would report.
Positions inside an obstacle cell (a robot hugging a wall) are snapped to the
nearest free cell.
"""
import argparse, json, math, os, sys, time

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

RSSI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RSSI_DIR)
from propagation import GridMap, PropagationParams              # noqa: E402
from network import Network, Node, NoiseParams, connectivity    # noqa: E402
from margin import margin_map, draw_margin, summary_line, CBAR_LABEL, STRONG_DBM  # noqa: E402

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GOOD_DBM = -65.0          # paper Fig. 9 bands


def topic_safe(s):
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in s)


class RssiMonitor(RosNode):
    def __init__(self, a):
        super().__init__("rssi_monitor")
        self.a = a
        root = os.path.dirname(RSSI_DIR)
        gt = a.gt if os.path.isabs(a.gt) else os.path.join(root, a.gt)
        self.grid = GridMap.load(gt + ".npy", gt + ".json")
        # nearest-free-cell lookup, for snapping positions that land in obstacles
        _, self.near = distance_transform_edt(self.grid.occ.astype(bool), return_indices=True)

        self.net = Network(self.grid,
                           PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha),
                           NoiseParams(a.sigma_shadow, a.corr, a.sigma_n, enabled=not a.no_noise))
        self.rng = np.random.default_rng()

        cfg = yaml.safe_load(open(a.config))
        self.spec = []                                   # (name, kind, topic|None, offset|xy)
        for i, (name, c) in enumerate(cfg.items()):
            kind = "gcs" if i == 0 else ("head" if name.startswith("head") else "relay")
            if "xy" in c:
                self.spec.append((name, kind, None, tuple(c["xy"])))
            else:
                self.spec.append((name, kind, c["topic"], tuple(c.get("offset", [0, 0]))))
        self.gcs = self.spec[0][0]
        self.pos = {n: tuple(v) for n, k, t, v in self.spec if t is None}
        self.last = {}

        for name, kind, topic, off in self.spec:
            if topic:
                self.create_subscription(Odometry, topic,
                                         lambda m, n=name, o=off: self._odom(n, o, m),
                                         qos_profile_sensor_data)
        self.pub_links = self.create_publisher(String, "/rssi/links", 10)
        self.pub_chain = self.create_publisher(Float32, "/rssi/chain", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/rssi/markers", 10)
        self.pub_link, self.pub_e2e = {}, {}
        self.pub_margin = self.create_publisher(Float32, "/rssi/margin/at_client", 10)
        self.pub_image = self.create_publisher(Image, "/rssi/margin/image", 1)
        order = [n for n, *_ in self.spec]
        self.client = a.client or order[-1]
        self.attach = a.attach or order[order.index(self.client) - 1]
        self.chain = order[:order.index(self.attach) + 1] + [self.client]
        if a.chain_topic:                # topology from the exploration coordinator
            self.create_subscription(String, a.chain_topic, self._chain, 10)
        self.est = {}                    # (a, b) -> EMA-filtered RSSI estimate (dBm)
        self.drawn = None                # state the current map was rendered from
        self.drawn_at = 0.0
        os.makedirs(a.map_dir, exist_ok=True)
        self.create_timer(1.0 / a.rate, self._tick)
        self.create_timer(a.log_every, self._log)
        self.latest = None
        self.get_logger().info(
            f"P0 {a.P0} n {a.n} alpha {a.alpha} | threshold {a.threshold} dBm | "
            f"noise {'off' if a.no_noise else f'shadow {a.sigma_shadow} dB/{a.corr} m + sigma_n {a.sigma_n} dB'} | "
            f"GCS '{self.gcs}' | tracking " +
            ", ".join(f"{n}<-{t}" for n, k, t, o in self.spec if t) +
            f" | margin map: {self.client} fixed on {self.attach}, chain {' > '.join(self.chain)}, "
            f"strong {a.strong:g} dBm -> {a.map_dir}")

    # ---------------------------------------------------------------- inputs
    def _chain(self, msg):
        chain = json.loads(msg.data)
        known = {n for n, *_ in self.spec}
        if len(chain) < 2 or not set(chain) <= known or chain == self.chain:
            return
        self.chain, self.client, self.attach = chain, chain[-1], chain[-2]
        self.get_logger().info(f"topology from {self.a.chain_topic}: {' > '.join(chain)}")

    def _odom(self, name, off, msg):
        p = msg.pose.pose.position
        self.pos[name] = (p.x + off[0], p.y + off[1])
        self.last[name] = time.time()

    def _snap(self, xy):
        r, c = self.grid.world_to_cell(*xy)
        r = min(max(r, 0), self.grid.shape[0] - 1); c = min(max(c, 0), self.grid.shape[1] - 1)
        if self.grid.occ[r, c]:
            r, c = int(self.near[0][r, c]), int(self.near[1][r, c])
            return tuple(float(v) for v in self.grid.cell_to_world(r, c)), True
        return xy, False

    # --------------------------------------------------------------- compute
    def _tick(self):
        nodes, snapped = [], []
        for name, kind, topic, _ in self.spec:
            if name not in self.pos:
                continue                                        # not heard from yet
            xy, moved = self._snap(self.pos[name])
            if moved:
                snapped.append(name)
            nodes.append(Node(name, xy, kind))
        if len(nodes) < 2 or nodes[0].name != self.gcs:
            return
        self.net.nodes = nodes
        links = self.net.links(noisy=True, rng=self.rng)
        conn = connectivity(links, nodes, self.a.threshold, self.gcs)

        for L in links:
            key = f"{topic_safe(L['a'])}__{topic_safe(L['b'])}"
            if key not in self.pub_link:
                self.pub_link[key] = self.create_publisher(Float32, f"/rssi/link/{key}", 10)
            self.pub_link[key].publish(Float32(data=float(L["rssi"])))
        for n in nodes[1:]:
            if n.name not in self.pub_e2e:
                self.pub_e2e[n.name] = self.create_publisher(
                    Float32, f"/rssi/end_to_end/{topic_safe(n.name)}", 10)
            h = conn.get(n.name)
            self.pub_e2e[n.name].publish(Float32(data=float(h[0]) if h else -math.inf))

        # paper eq. (5): end-to-end quality of the fixed chain = its weakest hop
        order = [name for name, *_ in self.spec]
        byp = {frozenset((L["a"], L["b"])): L["rssi"] for L in links}
        have = {n.name for n in nodes}
        if all(nm in have for nm in order):
            hops = [byp[frozenset((order[i], order[i + 1]))] for i in range(len(order) - 1)]
            chain_val = float(min(hops))
        else:
            hops, chain_val = [], -math.inf
        self.pub_chain.publish(Float32(data=chain_val))

        payload = {
            "stamp": time.time(),
            "params": dict(P0=self.a.P0, n=self.a.n, alpha=self.a.alpha, threshold=self.a.threshold),
            "nodes": {n.name: dict(kind=n.kind, xy=[round(v, 3) for v in n.xy],
                                   snapped=n.name in snapped,
                                   age_s=round(time.time() - self.last[n.name], 2) if n.name in self.last else None)
                      for n in nodes},
            "links": [dict(a=L["a"], b=L["b"], rssi=round(L["rssi"], 2), clean=round(L["clean"], 2),
                           dE=round(L["dE"], 2), dbar=round(L["dbar"], 2), los=L["los"],
                           up=bool(np.isfinite(L["rssi"]) and L["rssi"] >= self.a.threshold))
                      for L in links],
            "routes": {k: dict(bottleneck=round(v[0], 2), path=list(reversed(v[1])))
                       for k, v in conn.items()},
            "chain": dict(order=order, hops=[round(h, 2) for h in hops],
                          weakest=round(chain_val, 2) if np.isfinite(chain_val) else None,
                          up=bool(np.isfinite(chain_val) and chain_val >= self.a.threshold)),
        }
        self.pub_links.publish(String(data=json.dumps(payload)))
        self.pub_markers.publish(self._markers(nodes, links))
        self.latest = payload
        self._margin(nodes, links)

    # ---------------------------------------------------------- margin map
    def _margin(self, nodes, links):
        k = self.a.est_alpha
        for L in links:
            key = (L["a"], L["b"])
            if np.isfinite(L["rssi"]):
                prev = self.est.get(key)
                self.est[key] = L["rssi"] if prev is None or not np.isfinite(prev) else (1 - k) * prev + k * L["rssi"]
            else:
                self.est[key] = L["rssi"]
        have = {n.name for n in nodes}
        if not all(n in have for n in self.chain):
            return                                              # chain incomplete: nothing to map
        cells = {n.name: self.grid.world_to_cell(*n.xy) for n in nodes if n.name in self.chain}
        where = {n.name: n.xy for n in nodes if n.name in self.chain}
        hops = {key: v for key, v in self.est.items() if key[0] in self.chain and key[1] in self.chain}
        state = (tuple(self.chain), where, hops)
        now = time.time()
        if self.drawn is not None:
            topo, where0, hops0 = self.drawn
            moved = any(n not in where0 or math.dist(where[n], where0[n]) > self.a.move_tol for n in where)
            same = (topo == state[0] and not moved and
                    all(abs(hops[h] - hops0.get(h, np.inf)) <= self.a.est_change
                        or (not np.isfinite(hops[h]) and not np.isfinite(hops0.get(h, 0.0)))
                        for h in hops))
            if same or now - self.drawn_at < self.a.map_period:
                return
        res = margin_map(self.net, self.client, self.attach, self.chain, self.a.threshold, estimates=self.est)
        r, c = cells[self.client]
        self.pub_margin.publish(Float32(data=float(res["margin"][r, c])))
        fig, ax = plt.subplots(figsize=(6, 10.5), dpi=80)
        im = draw_margin(ax, self.net, res,
                         f"{time.strftime('%H:%M:%S')}  {self.client} fixed on {self.attach}  |  "
                         f"P0 {self.a.P0} n {self.a.n} alpha {self.a.alpha} eta {self.a.threshold:g}\n"
                         f"solid = zero margin, dashed = strong ({self.a.strong:g} dBm)  |  hop labels = filtered estimates (dBm)",
                         strong_dbm=self.a.strong)
        fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02).set_label(CBAR_LABEL)
        ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
        fig.tight_layout(); fig.canvas.draw()
        rgb = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
        tmp = os.path.join(self.a.map_dir, ".margin_latest.png")
        fig.savefig(tmp); os.replace(tmp, os.path.join(self.a.map_dir, "margin_latest.png"))
        plt.close(fig)
        img = Image(height=rgb.shape[0], width=rgb.shape[1], encoding="rgb8", is_bigendian=0,
                    step=rgb.shape[1] * 3, data=rgb.tobytes())
        img.header.stamp = self.get_clock().now().to_msg(); img.header.frame_id = "map"
        self.pub_image.publish(img)
        self.drawn, self.drawn_at = state, now
        self.margin_note = summary_line(res, self.net, self.a.strong)

    def _markers(self, nodes, links):
        ma = MarkerArray()
        ox, oy = self.a.marker_offset
        now = self.get_clock().now().to_msg()
        for i, L in enumerate(links):
            a = next(n for n in nodes if n.name == L["a"]); b = next(n for n in nodes if n.name == L["b"])
            m = Marker(); m.header.frame_id = self.a.marker_frame; m.header.stamp = now
            m.ns, m.id, m.type, m.action = "rssi_links", i, Marker.LINE_LIST, Marker.ADD
            m.scale.x = 0.06
            r = L["rssi"]
            m.color.r, m.color.g, m.color.b = ((0.17, 0.63, 0.17) if r > GOOD_DBM else
                                               (0.9, 0.72, 0.0) if r >= self.a.threshold else (0.84, 0.15, 0.16))
            m.color.a = 1.0 if r >= self.a.threshold else 0.5
            m.points = [Point(x=a.xy[0] + ox, y=a.xy[1] + oy, z=0.3), Point(x=b.xy[0] + ox, y=b.xy[1] + oy, z=0.3)]
            ma.markers.append(m)
            t = Marker(); t.header = m.header; t.ns, t.id = "rssi_labels", i
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position = Point(x=(a.xy[0] + b.xy[0]) / 2 + ox, y=(a.xy[1] + b.xy[1]) / 2 + oy, z=0.6)
            t.scale.z = 0.35; t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{r:.0f}"
            ma.markers.append(t)
        return ma

    def _log(self):
        p = self.latest
        if not p:
            waiting = [n for n, k, t, v in self.spec if t and n not in self.pos]
            self.get_logger().info(f"waiting for positions: {', '.join(waiting)}")
            return
        lines = ["nodes: " + "  ".join(
            f"{k}({v['xy'][0]:.2f},{v['xy'][1]:.2f}){'*snapped' if v['snapped'] else ''}"
            for k, v in p["nodes"].items())]
        for L in p["links"]:
            lines.append(f"  {L['a']+'-'+L['b']:16} {L['rssi']:7.1f} dBm (clean {L['clean']:6.1f})  "
                         f"dE {L['dE']:5.1f} m  detour {L['dbar']:4.1f} m  "
                         f"{'UP' if L['up'] else 'down'}{'' if L['los'] else '  NLOS'}")
        for k, v in p["routes"].items():
            lines.append(f"  route {k}: {' > '.join(v['path'])}   bottleneck {v['bottleneck']:.1f} dBm")
        ch = p["chain"]
        if ch["weakest"] is not None:
            lines.append(f"  CHAIN (eq.5) {' > '.join(ch['order'])}: hops {ch['hops']}  "
                         f"weakest {ch['weakest']:.1f} dBm  {'UP' if ch['up'] else 'BROKEN'}")
        if getattr(self, "margin_note", None):
            lines.append(f"  MARGIN {self.margin_note}")
        unreach = [n for n in p["nodes"] if n != self.gcs and n not in p["routes"]]
        if unreach:
            lines.append(f"  NO ROUTE TO {self.gcs}: {', '.join(unreach)}")
        self.get_logger().info("\n" + "\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default="hospital_world/gt/hospital_gt", help="ground-truth grid, repo-relative")
    ap.add_argument("--config", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "nodes.yaml"))
    ap.add_argument("--rate", type=float, default=2.0, help="Hz")
    ap.add_argument("--log-every", type=float, default=5.0, help="s between terminal tables")
    ap.add_argument("--P0", type=float, default=PropagationParams.P0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--threshold", type=float, default=-85.0)
    ap.add_argument("--sigma-shadow", type=float, default=NoiseParams.sigma_shadow)
    ap.add_argument("--corr", type=float, default=NoiseParams.corr_m)
    ap.add_argument("--sigma-n", type=float, default=NoiseParams.sigma_fast)
    ap.add_argument("--no-noise", action="store_true")
    ap.add_argument("--client", default=None, help="client node (default: last in nodes.yaml)")
    ap.add_argument("--attach", default=None, help="client's FIXED relay (default: the node before it)")
    ap.add_argument("--strong", type=float, default=STRONG_DBM, help="dashed contour: strong end-to-end RSSI, dBm")
    ap.add_argument("--chain-topic", default="/coord/chain",
                    help="follow this JSON chain (GCS ... client) for the margin map; '' = fixed")
    ap.add_argument("--map-dir", default="/live", help="where margin_latest.png is written")
    ap.add_argument("--map-period", type=float, default=5.0, help="min s between map renders")
    ap.add_argument("--move-tol", type=float, default=0.5,
                    help="re-render when a node has moved more than this (m) since the last render")
    ap.add_argument("--est-change", type=float, default=2.0,
                    help="re-render when a hop estimate moves by more than this (dB)")
    ap.add_argument("--est-alpha", type=float, default=0.1, help="EMA weight of a new link reading")
    ap.add_argument("--marker-frame", default="map")
    ap.add_argument("--marker-offset", nargs=2, type=float, default=[0.0, -12.0],
                    help="world -> marker frame. map = world - (0, 12) in this setup")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RssiMonitor(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
