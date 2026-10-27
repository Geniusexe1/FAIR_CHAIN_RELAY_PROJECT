#!/usr/bin/env python3
"""Communication-aware exploration coordinator (Seo et al., RA-L 2025, Sec. IV-B).

Sits between chain_robot's frontier explorer and Nav2. The explorer is started with
its action remapped:

    ros2 run frontier_exploration frontier_exploration_node.py \\
        --ros-args -r navigate_to_pose:=/coord/navigate_to_pose

so every frontier goal it picks comes here first. For each goal:

  inside the client's bounded region   -> client goes alone
  outside                              -> one more relay is engaged, relays placed and
                                          assigned, all trajectories planned (MCTS)
  outside with every relay engaged     -> REJECTED: the explorer automatically tries
     and no feasible placement             its next-ranked frontier

Execution is synchronised: the plan has t_f waypoints per robot; in interval t every
moving robot is sent its waypoint t, with a Nav2 speed limit chosen so that all of them
arrive together, re-adjusted every 0.5 s from each robot's remaining path length
(Nav2 feedback). Interval t+1 starts when everyone has arrived.

Subscribes  /Odometry (client, FAST-LIO, world = pose + client offset), /relayN/odom (world)
Uses        /navigate_to_pose, /relayN/navigate_to_pose   (map frame = world - (0, 12))
Publishes   /speed_limit, /relayN/speed_limit             nav2_msgs/SpeedLimit (m/s)
            /coord/chain   std_msgs/String  JSON list: GCS, engaged relays, client
                           (rssi_monitor --chain-topic follows it for the margin map)
            /coord/state   std_msgs/String  JSON: decision, plan, interval, speeds
            /coord/plan_markers  visualization_msgs/MarkerArray  planned trajectories
"""
import argparse, json, math, os, sys, threading, time

import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

RSSI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RSSI_DIR)
from propagation import GridMap, PropagationParams                               # noqa: E402
from network import NoiseParams                                                  # noqa: E402
from roadgraph import RoadGraph                                                  # noqa: E402
from relay_planner import Predictor, RelayPlanner, PlannerParams, Robot, schedule  # noqa: E402


class Coordinator(RosNode):
    def __init__(self, a):
        super().__init__("exploration_coordinator")
        self.a = a
        cb = ReentrantCallbackGroup()
        root = os.path.dirname(RSSI_DIR)
        gt = os.path.join(root, "hospital_world", "gt", "hospital_gt")
        grid = GridMap.load(gt + ".npy", gt + ".json")
        rg = RoadGraph.build(grid, cache_dir=a.cache_dir)
        pred = Predictor(grid, PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha), NoiseParams(sigma_fast=a.sigma_n))
        cfg = yaml.safe_load(open(a.config))
        names = list(cfg)
        self.gcs_xy = tuple(cfg[names[0]]["xy"])
        self.pp = PlannerParams(sigma=a.sigma_n, keep_deployed=a.keep_deployed)
        self.gcs_name = names[0]
        self.planner = RelayPlanner(grid, rg, pred, self.gcs_xy, self.pp, gcs_name=self.gcs_name)
        self.off = np.array(a.map_offset)                    # world = map + off

        self.client_name = names[-1]
        self.client = Robot(self.client_name, None, "client", a.v_client)
        self.relays = [Robot(n, None, "relay", a.v_relay) for n in names[1:-1]]
        self.robots = {r.name: r for r in self.relays + [self.client]}
        self.engaged = []
        for n in names[1:]:
            c = cfg[n]
            self.create_subscription(Odometry, c["topic"],
                                     lambda m, n=n, o=tuple(c.get("offset", [0, 0])): self._odom(n, o, m),
                                     qos_profile_sensor_data, callback_group=cb)

        def ns(n):
            return "" if n == self.client_name else f"/{n}"
        self.nav = {n: ActionClient(self, NavigateToPose, f"{ns(n)}/navigate_to_pose", callback_group=cb)
                    for n in self.robots}
        self.speed = {n: self.create_publisher(SpeedLimit, f"{ns(n)}/speed_limit", 10) for n in self.robots}
        self.pub_chain = self.create_publisher(String, "/coord/chain", 10)
        self.pub_state = self.create_publisher(String, "/coord/state", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/coord/plan_markers", 10)
        self.lock = threading.Lock()
        self.server = ActionServer(self, NavigateToPose, "/coord/navigate_to_pose",
                                   execute_callback=self._execute, goal_callback=self._on_goal,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.create_timer(1.0, self._publish_chain, callback_group=cb)
        self.get_logger().info(
            f"P0 {a.P0} | req {self.pp.req:.2f} dBm (Pr[link >= {self.pp.eta}] >= {self.pp.theta}) | "
            f"client {self.client_name}, relays {[r.name for r in self.relays]} | t_f {self.pp.t_f} | "
            f"{'keep deployed' if a.keep_deployed else 're-place engaged relays (paper)'}")

    # ----------------------------------------------------------------- state
    def _odom(self, name, off, msg):
        p = msg.pose.pose.position
        self.robots[name].xy = (p.x + off[0], p.y + off[1])

    def _ready(self):
        missing = [n for n, r in self.robots.items() if r.xy is None]
        return not missing, missing

    def _publish_chain(self):
        self.pub_chain.publish(String(data=json.dumps([self.gcs_name] + self.engaged + [self.client_name])))

    def _state(self, **kw):
        self.pub_state.publish(String(data=json.dumps(kw, default=float)))

    # ------------------------------------------------------------ goal gate
    def _on_goal(self, req):
        ok, missing = self._ready()
        if not ok:
            self.get_logger().warn(f"rejecting goal: no position yet for {missing}")
            return GoalResponse.REJECT
        p = req.pose.pose.position
        goal = (p.x + self.off[0], p.y + self.off[1])
        t0 = time.time()
        with self.lock:
            plan = self.planner.plan(goal, self.client, self.relays, self.engaged)
        dt = time.time() - t0
        self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): {plan.decision.upper()} "
                               f"[{' > '.join(plan.chain)}] {plan.note} ({dt:.1f} s)")
        self._state(event="decision", goal=goal, decision=plan.decision, chain=plan.chain, note=plan.note)
        if plan.decision == "skip":
            return GoalResponse.REJECT                        # explorer tries the next rank
        self._last_plan = plan
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------ execution
    def _pose(self, xy, nxt):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = xy[0] - self.off[0], xy[1] - self.off[1]
        yaw = math.atan2(nxt[1] - xy[1], nxt[0] - xy[0]) if nxt is not None else 0.0
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def _limit(self, name, v):
        msg = SpeedLimit(percentage=False, speed_limit=float(max(v, self.a.v_min)))
        self.speed[name].publish(msg)

    def _execute(self, gh):
        plan = self._last_plan
        if plan.decision == "deploy":
            self.engaged = [n for n in plan.chain if n in {r.name for r in self.relays}]
        self._markers(plan)
        sched = schedule(plan, {n: self.robots[n].vmax for n in plan.seg})
        tf = self.pp.t_f
        for t in range(1, tf + 1):
            movers = [n for n in plan.seg if plan.seg[n][t - 1] > 0.05]
            if not movers:
                continue
            fb, done, handles = {}, {}, {}
            for n in movers:
                if not self.nav[n].wait_for_server(timeout_sec=5.0):
                    self.get_logger().error(f"{n}: navigate_to_pose not available")
                    gh.abort(); return NavigateToPose.Result()
                self._limit(n, sched[t - 1]["v"][n])
                nxt = plan.traj[n][t + 1] if t < tf else None
                g = NavigateToPose.Goal(pose=self._pose(plan.traj[n][t], nxt))
                fut = self.nav[n].send_goal_async(
                    g, feedback_callback=lambda f, n=n: fb.__setitem__(n, f.feedback.distance_remaining))
                while not fut.done():
                    time.sleep(0.02)
                h = fut.result()
                if not h.accepted:
                    self.get_logger().warn(f"{n}: waypoint {t} rejected by Nav2")
                    done[n] = GoalStatus.STATUS_ABORTED
                    continue
                handles[n] = h
                h.get_result_async().add_done_callback(lambda r, n=n: done.__setitem__(n, r.result().status))
            self._state(event="interval", t=t, dt=sched[t - 1]["dt"], v=sched[t - 1]["v"], movers=movers)
            t_start = time.time()
            while len(done) < len(movers):
                if gh.is_cancel_requested:
                    for h in handles.values():
                        h.cancel_goal_async()
                    gh.canceled(); return NavigateToPose.Result()
                # keep the ETAs equal: v_r = remaining_r / T_remaining
                rem = {n: fb[n] for n in movers if n in fb and n not in done}
                if rem:
                    T = max(rem[n] / self.robots[n].vmax for n in rem)
                    for n in rem:
                        self._limit(n, rem[n] / T if T > 0 else self.robots[n].vmax)
                if time.time() - t_start > self.a.interval_timeout * max(sched[t - 1]["dt"], 1.0):
                    self.get_logger().warn(f"interval {t} timed out, moving on")
                    break
                time.sleep(0.5)
            failed = [n for n, s in done.items() if s != GoalStatus.STATUS_SUCCEEDED]
            if failed:
                self.get_logger().warn(f"interval {t}: {failed} did not reach their waypoint")
        for n in self.robots:
            self._limit(n, self.robots[n].vmax)               # lift the limits
        gh.succeed()
        return NavigateToPose.Result()

    def _markers(self, plan):
        ma = MarkerArray()
        ma.markers.append(Marker(action=Marker.DELETEALL))
        cols = {self.client_name: (0.84, 0.15, 0.16)}
        for i, (n, tr) in enumerate(plan.traj.items()):
            if np.ptp(tr, axis=0).max() < 0.05:
                continue
            m = Marker(); m.header.frame_id = "map"; m.ns, m.id = "coord_plan", i
            m.type, m.action, m.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.05
            m.color.r, m.color.g, m.color.b = cols.get(n, (0.12, 0.47, 0.71)); m.color.a = 1.0
            m.points = [Point(x=x - self.off[0], y=y - self.off[1], z=0.2) for x, y in tr]
            ma.markers.append(m)
        self.pub_mk.publish(ma)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--config", default=os.path.join(here, "nodes.yaml"))
    ap.add_argument("--cache-dir", default="/tmp/fair_cache")
    ap.add_argument("--P0", type=float, default=-30.0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--sigma-n", type=float, default=3.0)
    ap.add_argument("--v-client", type=float, default=0.7)
    ap.add_argument("--v-relay", type=float, default=0.5)
    ap.add_argument("--v-min", type=float, default=0.05, help="speed-limit floor, m/s")
    ap.add_argument("--interval-timeout", type=float, default=4.0, help="x planned interval duration")
    ap.add_argument("--map-offset", nargs=2, type=float, default=[0.0, 12.0], help="world = map + offset")
    ap.add_argument("--keep-deployed", action="store_true")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Coordinator(a)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
