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
import argparse, array, copy, json, math, os, sys, threading, time

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
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        # at low real-time factor Nav2's lifecycle manager misses bond heartbeats and
        # deactivates its own nodes; goals are then rejected. RESET + STARTUP revives it.
        self.lm = {n: self.create_client(ManageLifecycleNodes, f"{ns(n)}/lifecycle_manager_navigation/manage_nodes",
                                         callback_group=cb) for n in self.robots}
        self.bt_state = {n: self.create_client(GetState, f"{ns(n)}/bt_navigator/get_state", callback_group=cb)
                         for n in self.robots}
        self.pub_chain = self.create_publisher(String, "/coord/chain", 10)
        self.pub_state = self.create_publisher(String, "/coord/state", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/coord/plan_markers", 10)
        self.lock = threading.Lock()
        self.server = ActionServer(self, NavigateToPose, "/coord/navigate_to_pose",
                                   execute_callback=self._execute, goal_callback=self._on_goal,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.create_timer(1.0, self._publish_chain, callback_group=cb)
        # station keeping: the relays creep slowly when idle (drive-plugin issue, see
        # hospital_world/CHECKPOINT.md), so an idle relay that drifts from its spot is sent back
        self.home, self.yaw, self.keeping, self.busy = {}, {}, {}, False
        for nm in names[1:-1]:
            if "park" in cfg[nm]:
                self.home[nm] = tuple(cfg[nm]["park"])            # else: first pose seen
        # The explorer reads /map (1.5 MB). Some participants cannot get a FastDDS SHM port
        # on this host ("open_and_lock_file failed") and fall back to UDP, where 1.5 MB never
        # arrives with the default socket buffers -- its is_goal_occupied() then crashes on
        # a None map. Give it /map cropped to the explored area (<~200 KB), same frame/origin.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map_msg", m), latched,
                                 callback_group=cb)
        self.pub_map = self.create_publisher(OccupancyGrid, "/coord/map", latched)
        self.create_timer(2.0, self._publish_cropped_map, callback_group=cb)
        self.failed = {}           # goal -> times the client could not follow its plan
        self.moving = set()        # robots moving in the plan being executed
        self.visited = []          # goals already reached: the explorer re-sends frontiers it
                                   # cannot clear from there, which loops forever
        self.create_timer(a.keep_period, self._station_keep, callback_group=cb)
        self.get_logger().info(
            f"P0 {a.P0} | req {self.pp.req:.2f} dBm (Pr[link >= {self.pp.eta}] >= {self.pp.theta}) | "
            f"client {self.client_name}, relays {[r.name for r in self.relays]} | t_f {self.pp.t_f} | "
            f"{'keep deployed' if a.keep_deployed else 're-place engaged relays (paper)'}")

    # ----------------------------------------------------------------- state
    def _odom(self, name, off, msg):
        p = msg.pose.pose.position
        self.robots[name].xy = (p.x + off[0], p.y + off[1])
        q = msg.pose.pose.orientation
        self.yaw[name] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if name != self.client_name and name not in self.home:
            self.home[name] = self.robots[name].xy                # parking spot = first pose

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
        near = [v for v in self.visited + [self.client.xy] if math.dist(v, goal) < self.a.visited_r]
        if near:
            self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - already visited "
                                   f"(within {self.a.visited_r} m of the client or a reached goal)")
            self._state(event="decision", goal=goal, decision="visited")
            return GoalResponse.REJECT
        key = (round(goal[0], 1), round(goal[1], 1))
        if self.failed.get(key, 0) >= self.a.max_fail:
            self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - failed "
                                   f"{self.failed[key]} times, Nav2 cannot reach it")
            return GoalResponse.REJECT
        g = self.planner.grid
        r_, c_ = g.world_to_cell(*goal)
        if not g.in_bounds(r_, c_):                           # frontier past the building's edge
            self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - outside the map")
            return GoalResponse.REJECT
        t0 = time.time()
        try:
            with self.lock:
                self.planner.allowed = self._known_free()
                plan = self.planner.plan(goal, self.client, self.relays, self.engaged)
        except Exception as e:                                # never let one goal take the node down
            self.get_logger().error(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - planner error "
                                    f"{type(e).__name__}: {e}")
            return GoalResponse.REJECT
        dt = time.time() - t0
        self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): {plan.decision.upper()} "
                               f"[{' > '.join(plan.chain)}] {plan.note} ({dt:.1f} s)"
                               + ("" if not plan.targets else " | targets " + ", ".join(
                                   f"{n} ({xy[0]:.2f}, {xy[1]:.2f})" for n, xy in plan.targets.items())))
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

    def _floor(self, name):
        return self.a.v_min if name == self.client_name else self.a.v_floor_relay

    def _limit(self, name, v):
        msg = SpeedLimit(percentage=False, speed_limit=float(max(v, self.a.v_min)))
        self.speed[name].publish(msg)

    def _call(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def _nav_state(self, n):
        if not self.bt_state[n].wait_for_service(timeout_sec=3.0):
            return None
        r = self._call(self.bt_state[n].call_async(GetState.Request()), 10.0)
        return None if r is None else r.current_state.id

    def _wait_state(self, n, want, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self._nav_state(n)
            if st in want:
                return st
            time.sleep(3.0)
        return self._nav_state(n)

    def _revive(self, n) -> bool:
        """Nav2 deactivates itself when a node misses a 4 s bond heartbeat (CPU spikes at low
        RTF). Wait for the manager to settle, STARTUP, and only if needed RESET + STARTUP."""
        active, settled = State.PRIMARY_STATE_ACTIVE, (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE,
                                                       State.PRIMARY_STATE_UNCONFIGURED)
        st = self._wait_state(n, settled, 90.0)
        if st == active:
            return True
        self.get_logger().warn(f"{n}: Nav2 is down (bt_navigator state {st}) - restarting it")
        if not self.lm[n].wait_for_service(timeout_sec=5.0):
            return False
        R = ManageLifecycleNodes.Request
        for cmds in ((R.STARTUP,), (R.RESET, R.STARTUP)):
            for cmd in cmds:
                self._call(self.lm[n].call_async(R(command=cmd)), 180.0)
            if self._wait_state(n, (active,), 120.0) == active:
                self.get_logger().info(f"{n}: Nav2 active again")
                return True
        self.get_logger().error(f"{n}: could not bring Nav2 back")
        return False

    def _send(self, n, goal, fb):
        for attempt in range(2):
            h = self._call(self.nav[n].send_goal_async(
                goal, feedback_callback=lambda f, n=n: fb.__setitem__(n, f.feedback.distance_remaining)), 10.0)
            if h is not None and h.accepted:
                return h
            if attempt == 0:
                if not self._revive(n):
                    return None
                goal.pose.header.stamp = self.get_clock().now().to_msg()
        return None

    def _known_free(self):
        """GT-grid bool mask: cells the robots' own map (/map) knows are free. Nav2 plans
        only through known space, so planner waypoints must stay inside it."""
        m = self.map_msg
        if m is None:
            return None
        g = self.planner.grid
        key = (m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
        if getattr(self, "_mask_key", None) != key:              # GT cell -> live-map cell, cached
            H, W = g.shape
            xs = g.origin[0] + (np.arange(W) + 0.5) * g.res - self.off[0]
            ys = g.origin[1] + (np.arange(H) + 0.5) * g.res - self.off[1]
            cc = np.floor((xs - key[3]) / key[2]).astype(int)
            rr = np.floor((ys - key[4]) / key[2]).astype(int)
            self._mask_idx = (np.clip(rr, 0, key[1] - 1)[:, None], np.clip(cc, 0, key[0] - 1)[None, :],
                              ((rr >= 0) & (rr < key[1]))[:, None] & ((cc >= 0) & (cc < key[0]))[None, :])
            self._mask_key = key
        live = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        r_, c_, inside = self._mask_idx
        v = live[r_, c_]
        return inside & (v >= 0) & (v < 50)

    def _publish_cropped_map(self):
        m = self.map_msg
        if m is None:
            return
        W, H = m.info.width, m.info.height
        g = np.asarray(m.data, dtype=np.int8).reshape(H, W)
        known = np.argwhere(g != -1)
        if not len(known):
            return
        pad = 20
        r0, c0 = np.maximum(known.min(0) - pad, 0)
        r1, c1 = np.minimum(known.max(0) + pad + 1, [H, W])
        crop = g[r0:r1, c0:c1]
        out = OccupancyGrid()
        out.header = m.header
        out.info = copy.deepcopy(m.info)                      # never shift the stored map's origin
        out.info.width, out.info.height = int(c1 - c0), int(r1 - r0)
        out.info.origin.position.x = m.info.origin.position.x + c0 * m.info.resolution
        out.info.origin.position.y = m.info.origin.position.y + r0 * m.info.resolution
        out.data = array.array("b", crop.ravel().tobytes())
        self.pub_map.publish(out)

    def _station_keep(self):
        for r in self.relays:                                     # also during plans, if not moving in it
            if r.xy is None or r.name not in self.home or r.name in self.keeping or r.name in self.moving:
                continue
            drift = math.dist(r.xy, self.home[r.name])
            if drift < self.a.keep_tol:
                continue
            self.get_logger().info(f"{r.name} drifted {drift:.2f} m from its spot - sending it back")
            hx, hy = self.home[r.name]
            g = NavigateToPose.Goal(pose=self._pose((hx, hy), None))
            yaw = self.yaw.get(r.name, 0.0)                       # keep the heading it parked with
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.keeping[r.name] = None
            h = self._send(r.name, g, {})                         # revives its Nav2 if it is down
            if h is None:
                self.keeping.pop(r.name, None); continue
            self.keeping[r.name] = h
            h.get_result_async().add_done_callback(lambda _r, n=r.name: self.keeping.pop(n, None))

    def _execute(self, gh):
        self.busy = True
        plan = self._last_plan
        self.moving = {n for n in plan.seg if float(np.sum(plan.seg[n])) > 0.05}
        for n, h in list(self.keeping.items()):                   # the plan overrides station keeping
            if n in self.moving:                                  # for the robots it moves
                if h is not None:
                    h.cancel_goal_async()
                self.keeping.pop(n, None)
        try:
            self._reached = False
            res = self._execute_plan(gh)
            for n, xy in plan.targets.items():                    # where each relay now belongs
                if n != self.client_name:
                    self.home[n] = tuple(xy)
            if self._reached:                                     # only goals actually reached
                self.visited.append(tuple(plan.goal))
            else:
                k = (round(plan.goal[0], 1), round(plan.goal[1], 1))
                self.failed[k] = self.failed.get(k, 0) + 1
            return res
        finally:
            self.busy = False
            self.moving = set()

    def _execute_plan(self, gh):
        plan = self._last_plan
        if plan.decision == "deploy":
            self.engaged = [n for n in plan.chain if n in {r.name for r in self.relays}]
        self._markers(plan)
        sched = schedule(plan, {n: self.robots[n].vmax for n in plan.seg})
        tf = len(next(iter(plan.seg.values())))      # per plan: the ladder may refine t_f
        for t in range(1, tf + 1):
            movers = [n for n in plan.seg if plan.seg[n][t - 1] > 0.05]
            if not movers:
                continue
            fb, done, handles = {}, {}, {}
            dt = sched[t - 1]["dt"]
            # Same arrival, different distance. Below its floor speed a robot cannot move at all
            # (relay drive deadband: 0.10 m/s stalls, CHECKPOINT.md), so a robot whose synced
            # speed would be under the floor drives AT the floor and starts later instead.
            start, speed = {}, {}
            for n in movers:
                v, fl = sched[t - 1]["v"][n], self._floor(n)
                if 0 < v < fl:
                    speed[n], start[n] = fl, dt - plan.seg[n][t - 1] / fl
                else:
                    speed[n], start[n] = v, 0.0

            def launch(n):
                if not self.nav[n].wait_for_server(timeout_sec=5.0):
                    self.get_logger().error(f"{n}: navigate_to_pose not available")
                    done[n] = GoalStatus.STATUS_ABORTED
                    return
                self._limit(n, speed[n])
                nxt = plan.traj[n][t + 1] if t < tf else None
                g = NavigateToPose.Goal(pose=self._pose(plan.traj[n][t], nxt))
                h = self._send(n, g, fb)
                if h is None:
                    self.get_logger().warn(f"{n}: waypoint {t} rejected by Nav2 (also after a restart)")
                    done[n] = GoalStatus.STATUS_ABORTED
                    return
                handles[n] = h
                h.get_result_async().add_done_callback(lambda r, n=n: done.__setitem__(n, r.result().status))

            t_start = self.get_clock().now()                  # SIM time: plans are in sim seconds
            waiting = sorted((n for n in movers if start[n] > 0), key=lambda n: start[n])
            for n in movers:
                if start[n] <= 0:
                    launch(n)
            self._state(event="interval", t=t, dt=dt, v=speed, start_delay=start, movers=movers)
            while len(done) < len(movers):
                if gh.is_cancel_requested:
                    for h in handles.values():
                        h.cancel_goal_async()
                    gh.canceled(); return NavigateToPose.Result()
                el = (self.get_clock().now() - t_start).nanoseconds / 1e9
                while waiting and el >= start[waiting[0]]:
                    launch(waiting.pop(0))                    # delayed start: arrives with the rest
                # keep the ETAs equal: v_r = remaining_r / T_remaining, never below the floor
                rem = {n: fb[n] for n in movers if n in fb and n not in done}
                if rem:
                    T = max(rem[n] / self.robots[n].vmax for n in rem)
                    for n in rem:
                        self._limit(n, max(rem[n] / T if T > 0 else self.robots[n].vmax, self._floor(n)))
                # slack: a relay needs time to plan and turn (Ackermann) before it moves at all
                if (self.get_clock().now() - t_start).nanoseconds / 1e9 > \
                        self.a.interval_timeout * max(sched[t - 1]["dt"], 1.0) + self.a.interval_slack:
                    self.get_logger().warn(f"interval {t} timed out, moving on")
                    break
                time.sleep(0.5)
            failed = [n for n, s in done.items() if s != GoalStatus.STATUS_SUCCEEDED]
            if failed:
                self.get_logger().warn(f"interval {t}: {failed} did not reach their waypoint")
            if self.client_name in failed or self.client_name in [n for n in movers if n not in done]:
                # the client could not follow the plan: report it, the explorer re-plans
                for h in handles.values():
                    h.cancel_goal_async()
                for n in self.robots:
                    self._limit(n, self.robots[n].vmax)
                self._state(event="aborted", t=t, failed=failed)
                gh.abort()
                return NavigateToPose.Result()
        for n in self.robots:
            self._limit(n, self.robots[n].vmax)               # lift the limits
        self._reached = True
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
    ap.add_argument("--v-min", type=float, default=0.05, help="client speed-limit floor, m/s")
    ap.add_argument("--v-floor-relay", type=float, default=0.18,
                    help="relay speed floor, m/s: the drive stalls at 0.10 m/s, moves at 0.15")
    ap.add_argument("--interval-timeout", type=float, default=4.0, help="x planned interval duration")
    ap.add_argument("--interval-slack", type=float, default=30.0, help="+ sim seconds per interval (start-up)")
    ap.add_argument("--map-offset", nargs=2, type=float, default=[0.0, 12.0], help="world = map + offset")
    ap.add_argument("--keep-deployed", action="store_true")
    ap.add_argument("--max-fail", type=int, default=2, help="reject a goal after this many failed attempts")
    ap.add_argument("--visited-r", type=float, default=0.6, help="reject goals this close to a reached one (m)")
    ap.add_argument("--keep-tol", type=float, default=0.3, help="station keeping: drift (m) that triggers a return")
    ap.add_argument("--keep-period", type=float, default=10.0, help="station keeping check period (s)")
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
