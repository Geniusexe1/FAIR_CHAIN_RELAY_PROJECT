#!/usr/bin/env python3
"""Communication-aware exploration coordinator (Seo et al., RA-L 2025, Sec. IV-B).

Sits between chain_robot's frontier explorer and Nav2. The explorer is started with
its action remapped:

    ros2 run frontier_exploration frontier_exploration_node.py \\
        --ros-args -r navigate_to_pose:=/coord/navigate_to_pose

so every frontier goal it picks comes here first. For each goal:

  inside the client's bounded region   -> client goes alone (relays stay where they are
                                          and forward if they are on the best route)
  outside                              -> the fewest relays that reach it are placed and
                                          assigned, all trajectories planned (MCTS)
  outside even with every relay placed -> REJECTED: the explorer automatically tries
                                          its next-ranked frontier; a goal skipped twice
                                          is rejected at once until a relay moves

Execution is synchronised: the plan has t_f waypoints per robot; in interval t every
moving robot is sent its waypoint t, with a Nav2 speed limit chosen so that all of them
arrive together, re-adjusted every 0.5 s from each robot's remaining path length
(Nav2 feedback). Interval t+1 starts when everyone has arrived.

Subscribes  /Odometry (client, FAST-LIO, world = pose + client offset), /relayN/odom (world)
Uses        /navigate_to_pose, /relayN/navigate_to_pose   (map frame = world - (0, 12))
Publishes   /speed_limit, /relayN/speed_limit             nav2_msgs/SpeedLimit (m/s)
            /coord/chain   std_msgs/String  JSON list: the CURRENT best route GCS, ..., client
                           (fewest hops meeting the requirement; rssi_monitor --chain-topic
                           follows it for the margin map)
Writes      --map-log DIR: the client's /map every --map-log-period s (npz) for the replay
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
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
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
from relay_planner import Plan, Predictor, RelayPlanner, PlannerParams, Robot, schedule  # noqa: E402
from scipy.ndimage import binary_closing, binary_dilation, distance_transform_edt, find_objects, label  # noqa: E402


class Coordinator(RosNode):
    def __init__(self, a):
        super().__init__("exploration_coordinator")
        self.a = a
        cb = ReentrantCallbackGroup()
        root = os.path.dirname(RSSI_DIR)
        # the prebuilt map is for the radio model ONLY (the real robots measure signal instead).
        # Goals and relay positions come from what has been explored: see _explored()
        # the radio can have its own grid: signal passes over a desk that blocks a robot (real_env)
        rgt = a.radio_gt or a.gt
        rgt = rgt if os.path.isabs(rgt) else os.path.join(root, rgt)
        radio = GridMap.load(rgt + ".npy", rgt + ".json")
        grid = GridMap(occ=np.ones_like(radio.occ), res=radio.res, origin=radio.origin)   # nothing explored yet
        rg = RoadGraph.build(grid)
        pred = Predictor(radio, PropagationParams(P0=a.P0, n=a.n, alpha=a.alpha), NoiseParams(sigma_fast=a.sigma_n))
        cfg = yaml.safe_load(open(a.config))
        names = list(cfg)
        self.gcs_xy = tuple(cfg[names[0]]["xy"])
        self.next_own = 0.0                                  # sim time of the next try at a goal of our own
        self._nav_msg = None                                 # the map msg the planner's road graph is from
        self.dist_reach = None                               # m to the explored floor the client is on
        self.pp = PlannerParams(sigma=a.sigma_n, keep_deployed=a.keep_deployed, d_sep=a.d_sep,
                                 pass_clear=a.pass_clear)
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
        # chain_robot cannot reverse or turn in place (DWB min_vel_x 0.2) and its BT does not
        # back up: nose-first at a dead end it is stuck for good. The watchdog frees it (_free_client).
        self.backup = ActionClient(self, BackUp, "/backup", callback_group=cb)
        self.client_cmd = self.create_publisher(Twist, "/cmd_vel", 10)      # _free_client only
        self.relay_cmd = {}                                                  # _free_relay only
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
        self.park, self.seen = {}, set()
        for nm in names[1:-1]:
            if "park" in cfg[nm]:
                self.home[nm] = self.park[nm] = tuple(cfg[nm]["park"])   # else: first pose seen
        # The explorer reads /map (1.5 MB). Some participants cannot get a FastDDS SHM port
        # on this host ("open_and_lock_file failed") and fall back to UDP, where 1.5 MB never
        # arrives with the default socket buffers -- its is_goal_occupied() then crashes on
        # a None map. Give it /map cropped to the explored area (<~200 KB), same frame/origin.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, a.map_topic, lambda m: setattr(self, "map_msg", m), latched,
                                 callback_group=cb)
        self.pub_map = self.create_publisher(OccupancyGrid, "/coord/map", latched)
        self.create_timer(2.0, self._publish_cropped_map, callback_group=cb)
        self.failed = {}           # goal -> times the client could not follow its plan
        self.last_accept = None    # sim time of the last goal that got through the gate
        self.moving = set()        # robots moving in the plan being executed
        self.visited = []          # goals already reached: the explorer re-sends frontiers it
                                   # cannot clear from there, which loops forever
        self.create_timer(a.keep_period, self._station_keep, callback_group=cb)
        self.keep_wait = {}        # relay -> sim time before which a failed return is not tried again
        self.keep_fails = {}       # relay -> returns failed in a row
        self.unused_since = {}     # relay -> sim time since which the route has not used it
        self.relay_state = {}      # relay -> PARKED / DEPLOYING / RELAYING / RETURNING / HOLDING
        self.state_log = open(a.state_log, "a", buffering=1) if a.state_log else None
        self.create_timer(1.0, self._update_relay_states, callback_group=cb)
        self.skipped = {}          # goal key -> (times skipped, robot positions + sim time at the last skip)
        self.misses = {}           # relay -> consecutive plans it did not follow
        self.benched = {}          # relay -> sim time until which it is not moved (static radio)
        self.route = [self.gcs_name, self.client_name]
        self.skip_streak = 0       # goals skipped in a row
        # where the client regroups: its spawn point next to the GCS (= its odometry offset)
        self.regroup_xy = tuple(cfg[self.client_name].get("regroup", cfg[self.client_name].get("offset", self.gcs_xy)))
        self.regrouping = False
        if a.map_log:
            os.makedirs(a.map_log, exist_ok=True)
            self.create_timer(a.map_log_period, self._log_map, callback_group=cb)
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
        if name != self.client_name and name not in self.seen:
            self.seen.add(name)                                   # first pose: a relay already out
            self.home[name] = self.robots[name].xy                # (coordinator restarted) holds its
                                                                  # spot instead of driving to park

    def _ready(self):
        missing = [n for n, r in self.robots.items() if r.xy is None]
        return not missing, missing

    def _publish_chain(self):
        """The route the mesh would use right now (live positions)."""
        if self._ready()[0] and self.lock.acquire(blocking=False):   # never wait on a planning goal
            try:
                pos = {self.gcs_name: self.gcs_xy, **{n: r.xy for n, r in self.robots.items()}}
                self.route = self.planner.route(pos, self.client_name)[1]
            except Exception as e:
                self.get_logger().warn(f"route: {type(e).__name__}: {e}")
            finally:
                self.lock.release()
        self.pub_chain.publish(String(data=json.dumps(self.route)))

    def _log_map(self):
        m = self.map_msg
        if m is None:
            return
        g = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        np.savez_compressed(os.path.join(self.a.map_log, f"map_{time.time():.0f}.npz"), grid=g,
                            res=m.info.resolution, origin=(m.info.origin.position.x, m.info.origin.position.y),
                            off=self.off, wall=time.time(), route=json.dumps(self.route),
                            pos=json.dumps({n: r.xy for n, r in self.robots.items()}))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

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
        # every frontier left is one we remember as failed or visited: nothing moves and the explorer
        # re-sends them for ever (big_u_nav2_P45: 4700 rejections at 84%). Forget them and try again.
        now = self._now()
        if self.last_accept is None:
            self.last_accept = now
        if now - self.last_accept > self.a.reject_reset and (self.failed or self.visited):
            self.get_logger().warn(f"no goal accepted for {now - self.last_accept:.0f} s - forgetting "
                                   f"{len(self.failed)} failed and {len(self.visited)} visited goals")
            self.failed.clear(); self.visited.clear()
            self.last_accept = now
        # the explorer only ever sends its top ranks: when those are all frontiers nobody can reach (seen
        # through a gap in a wall) the real ones further down never get a turn (real_full_live: 58%, an hour
        # stuck on 5 phantoms, a whole classroom at rank 24 and below). Accept and go to a real one instead.
        if now - self.last_accept > self.a.own_goal_after and now >= self.next_own:
            self.next_own = now + self.a.own_goal_every
            if self._own_goal(goal):
                return GoalResponse.ACCEPT
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
        sk = self.skipped.get(key)
        # a goal skipped again and again is not re-planned (each plan costs seconds) until the
        # situation changed: some robot moved, or a cool-down passed -- never forever (deadlock)
        if sk and sk[0] >= self.a.max_skip and self._now() - sk[2] < self.a.skip_cooldown and all(
                math.dist(r.xy, sk[1][n]) < 1.5 for n, r in self.robots.items()):
            self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - skipped {sk[0]} times, "
                                   f"nothing has moved since")
            return GoalResponse.REJECT
        t0 = time.time()
        try:
            with self.lock:
                if not self._explored():
                    self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - no map yet")
                    return GoalResponse.REJECT
                g = self.planner.grid
                r_, c_ = g.world_to_cell(*goal)
                # a frontier the client's explored floor does not lead to: seen through a gap, or past the
                # building's edge (real_nav2: its last 15 min went on a goal out over the balcony edge)
                if not g.in_bounds(r_, c_) or self.dist_reach[r_, c_] > self.a.goal_off_floor:
                    self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - "
                                           f"not connected to the explored floor")
                    return GoalResponse.REJECT
                roomy = self._roomy_goal(goal)
                if roomy is None:
                    self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - "
                                           f"no room for the chain within {self.a.goal_snap} m")
                    self._state(event="decision", goal=goal, decision="tight")
                    return GoalResponse.REJECT
                if roomy != goal:
                    rk = (round(roomy[0], 1), round(roomy[1], 1))
                    if self.failed.get(rk, 0) >= self.a.max_fail or any(
                            math.dist(v, roomy) < self.a.visited_r for v in self.visited + [self.client.xy]):
                        self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - its roomy "
                                               f"spot ({roomy[0]:.2f}, {roomy[1]:.2f}) is visited or failed")
                        self._state(event="decision", goal=goal, decision="visited")
                        return GoalResponse.REJECT
                    self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): too tight, "
                                           f"using ({roomy[0]:.2f}, {roomy[1]:.2f})")
                    goal = roomy
                now = self._now()
                movable = [r for r in self.relays if self.benched.get(r.name, -1) < now]
                static = {r.name: r.xy for r in self.relays if r not in movable}
                plan = self.planner.plan(goal, self.client, movable, self.engaged, static=static)
                off = [n for n, xy in plan.targets.items() if n != self.client_name and not self._is_explored(xy)]
                if off:                                       # never send a relay somewhere nobody has been
                    plan = Plan("skip", goal, [], {}, note=f"{', '.join(off)} would leave the explored map")
        except Exception as e:                                # never let one goal take the node down
            import traceback
            self.get_logger().error(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): REJECTED - planner error "
                                    f"{type(e).__name__}: {e}\n{traceback.format_exc()}")
            return GoalResponse.REJECT
        dt = time.time() - t0
        self.get_logger().info(f"goal world ({goal[0]:.2f}, {goal[1]:.2f}): {plan.decision.upper()} "
                               f"[{' > '.join(plan.chain)}] {plan.note} ({dt:.1f} s)"
                               + ("" if not plan.targets else " | targets " + ", ".join(
                                   f"{n} ({xy[0]:.2f}, {xy[1]:.2f})" for n, xy in plan.targets.items())))
        self._state(event="decision", goal=goal, decision=plan.decision, chain=plan.chain, note=plan.note)
        self.regrouping = False
        if plan.decision == "skip":
            self.skipped[key] = (self.skipped.get(key, (0,))[0] + 1, {n: r.xy for n, r in self.robots.items()},
                                 self._now())
            self.skip_streak += 1
            # Every frontier left is out of reach FROM HERE (client at the end of a long chain):
            # regroup -- the client comes back to the GCS through the network it has; relays off
            # the new route then return to park, and the frontiers are planned again from there.
            if self.skip_streak >= self.a.regroup_after and math.dist(self.client.xy, self.regroup_xy) > 3.0:
                with self.lock:
                    rp = self.planner.plan(self.regroup_xy, self.client, movable, self.engaged, static=static)
                self.get_logger().info(f"REGROUP after {self.skip_streak} skips: client back to "
                                       f"({self.regroup_xy[0]:.2f}, {self.regroup_xy[1]:.2f}): {rp.decision.upper()} "
                                       f"[{' > '.join(rp.chain)}] {rp.note}")
                if rp.decision != "skip":
                    self.skip_streak = 0
                    self.regrouping = True
                    self._last_plan = rp
                    self._state(event="decision", goal=self.regroup_xy, decision="regroup", chain=rp.chain)
                    return GoalResponse.ACCEPT                # the explorer re-picks once the client is back
            return GoalResponse.REJECT                        # explorer tries the next rank
        self.skip_streak = 0
        self.skipped.pop(key, None)
        self._drop_unused(plan)
        self._drop_short(plan)
        self._last_plan = plan
        return GoalResponse.ACCEPT

    def _own_goal(self, sent):
        """Plan for a frontier on the explored floor instead of the one the explorer sent. True if one is on."""
        with self.lock:
            if not self._explored():
                return False
            now = self._now()
            movable = [r for r in self.relays if self.benched.get(r.name, -1) < now]
            static = {r.name: r.xy for r in self.relays if r not in movable}
            tried = 0
            for xy in frontier_goals(self.planner.grid, self.planner.allowed, self.unknown, self.dist_reach == 0,
                                     self.client.xy):
                key = (round(xy[0], 1), round(xy[1], 1))
                sk = self.skipped.get(key)
                if any(math.dist(v, xy) < self.a.visited_r for v in self.visited + [self.client.xy]) or \
                        self.failed.get(key, 0) >= self.a.max_fail or \
                        (sk and sk[0] >= self.a.max_skip and now - sk[2] < self.a.skip_cooldown):
                    continue
                if tried == self.a.own_goal_tries:
                    break
                tried += 1
                plan = self.planner.plan(xy, self.client, movable, self.engaged, static=static)
                if plan.decision != "skip" and all(self._is_explored(p) for n, p in plan.targets.items()
                                                   if n != self.client_name):
                    self.get_logger().info(f"OWN GOAL instead of ({sent[0]:.2f}, {sent[1]:.2f}): "
                                           f"goal world ({xy[0]:.2f}, {xy[1]:.2f}): {plan.decision.upper()} "
                                           f"[{' > '.join(plan.chain)}] {plan.note}")
                    self._state(event="decision", goal=xy, decision=plan.decision, chain=plan.chain, note=plan.note)
                    self.regrouping = False
                    self.skip_streak = 0
                    self._drop_unused(plan)
                    self._drop_short(plan)
                    self._last_plan = plan
                    return True
                self.skipped[key] = (sk[0] + 1 if sk else 1, {n: r.xy for n, r in self.robots.items()}, now)
        return False

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

    def _is_explored(self, xy):
        g = self.planner.grid
        r_, c_ = g.world_to_cell(*xy)
        return g.in_bounds(r_, c_) and bool(self.planner.allowed[r_, c_])

    def _explored(self):
        """Point the planner at the robots' own map as it stands. False until there is one."""
        m = self.map_msg
        if m is None:
            return False
        if m is self._nav_msg:
            return True
        free, unknown = self._known_free()
        # a scan leaves specks of unknown in open floor, which would cut the roads. Occupied stays occupied.
        free = free | (binary_closing(free, iterations=self.a.fill_cells) & unknown)
        g = self.planner.grid
        grid = GridMap(occ=(~free).astype(np.uint8), res=g.res, origin=g.origin)
        self.planner.set_map(grid, RoadGraph.build(grid, drop_outside=False), allowed=free)
        self.unknown = unknown & ~free
        lab, _ = label(free)
        d, (nr, nc) = distance_transform_edt(~free, return_indices=True)
        r_, c_ = g.world_to_cell(*self.client.xy)
        r_, c_ = min(max(r_, 0), g.shape[0] - 1), min(max(c_, 0), g.shape[1] - 1)
        self.dist_reach = distance_transform_edt(lab != lab[nr[r_, c_], nc[r_, c_]]) * g.res
        # unknown does not count against a goal's room, or every frontier would be too tight
        roomy = (self.dist_reach == 0) & (distance_transform_edt(free | unknown) * g.res >= self.a.goal_clear)
        self.roomy_any = bool(roomy.any())
        self.roomy_d, self.roomy_idx = distance_transform_edt(~roomy, return_indices=True)
        self._nav_msg = m
        return True

    def _roomy_goal(self, goal):
        """The goal, or the nearest cell with room around it for the chain's body. None if there is none near:
        between desks the body wedges and the head has no legal move left (contact_A3: a whole test lost)."""
        g = self.planner.grid
        r_, c_ = g.world_to_cell(*goal)
        if not self.roomy_any or self.roomy_d[r_, c_] * g.res > self.a.goal_snap:
            return None
        if self.roomy_d[r_, c_] == 0:
            return goal
        p = g.cell_to_world(self.roomy_idx[0][r_, c_], self.roomy_idx[1][r_, c_])
        return (float(p[0]), float(p[1]))

    def _known_free(self):
        """On the planner's lattice: cells the robots' own map (/map) knows are free, and cells it has
        not seen. Nav2 plans only through known space, so planner waypoints must stay inside it."""
        m = self.map_msg
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
        return inside & (v >= 0) & (v < 50), ~inside | (v < 0)

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

    def _update_relay_states(self):
        """Where each relay belongs and what it is doing, from the route the mesh uses right now. A relay the
        route has not used for --idle-after goes back to its parking spot, however the last plan ended."""
        if not self._ready()[0]:
            return
        now = self._now()
        for r in self.relays:
            n = r.name
            park = self.park.get(n)
            if n in self.route:
                self.unused_since.pop(n, None)
                if n not in self.moving and n in self.keeping and self.home.get(n) == park:
                    h = self.keeping.pop(n)                       # needed again on its way back: stay
                    if h is not None:
                        h.cancel_goal_async()
                if n not in self.moving and self.home.get(n) == park:
                    self.home[n] = r.xy
            elif n not in self.moving:
                since = self.unused_since.setdefault(n, now)
                if park and now - since >= self.a.idle_after and self.benched.get(n, -1) < now:
                    self.home[n] = park
            at_park = park is not None and math.dist(r.xy, park) < self.a.keep_tol
            if n in self.moving:
                st = "DEPLOYING"
            elif n in self.route:
                st = "RELAYING"
            elif at_park and n not in self.keeping:
                st = "PARKED"
            elif self.home.get(n) == park and park is not None:
                st = "RETURNING"
            else:
                st = "HOLDING"
            if st != self.relay_state.get(n):
                self.get_logger().info(f"{n}: {self.relay_state.get(n, 'start')} -> {st} at ({r.xy[0]:.2f}, {r.xy[1]:.2f})")
                self.relay_state[n] = st
                self._state(event="relay_state", robot=n, state=st, xy=r.xy)
        if self.state_log:
            self.state_log.write(json.dumps(dict(t=time.time(), sim=now, route=self.route, state=self.relay_state,
                                                 xy={r.name: r.xy for r in self.relays})) + "\n")

    def _keep_done(self, n, res):
        self.keeping.pop(n, None)
        if res.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.keep_fails[n] = 0
        else:                                                     # a relay that cannot get home stands still
            self.keep_fails[n] = min(self.keep_fails.get(n, 0) + 1, 4)   # rather than shuffle (real_full: 25 min)
            self.keep_wait[n] = self._now() + self.a.keep_wait * 2 ** (self.keep_fails[n] - 1)

    def _drop_unused(self, plan):
        """A relay no step of the plan routes through stays where it is. plan() places relays with the client
        exactly at the goal but the trajectory ends on the road graph next to it; on a marginal link the
        direct route then works and the relay would drive out for nothing (v3_u_full: 3 of 8 deployments)."""
        used = {n for step in plan.hops for hop in step for n in hop[:2]}
        for n in [n for n in plan.targets if n != self.client_name and n not in used]:
            self.get_logger().info(f"{n} is not on the route at any step - not moving it")
            del plan.targets[n]
            plan.traj[n] = np.repeat([self.robots[n].xy], len(plan.seg[n]) + 1, axis=0)
            plan.seg[n] = np.zeros(len(plan.seg[n]))

    def _drop_short(self, plan):
        """A car does not shift by 0.4 m: with a 0.7 m turning radius it shuffles at it until the plan times out
        (real_full_live2: relay1 300 s at a goal 0.41 m away). If the link holds from where it stands, it stays."""
        pts = {self.gcs_name: self.gcs_xy, **{n: tuple(tr[-1]) for n, tr in plan.traj.items()}}
        for n in [n for n in plan.targets if n != self.client_name]:
            if float(np.sum(plan.seg[n])) >= self.a.relay_min_move:
                continue
            trial = {**pts, n: self.robots[n].xy}
            with self.lock:
                worst = min(self.planner.pred.link(a, trial.get(a, self.robots[a].xy if a in self.robots else None),
                                                   b, trial.get(b, self.robots[b].xy if b in self.robots else None))
                            for a, b in zip(plan.chain, plan.chain[1:]))
            if worst >= self.pp.req:
                self.get_logger().info(f"{n}: a {float(np.sum(plan.seg[n])):.2f} m move is not worth it, "
                                       f"weakest hop {worst:.1f} from where it is - not moving it")
                del plan.targets[n]
                plan.traj[n] = np.repeat([self.robots[n].xy], len(plan.seg[n]) + 1, axis=0)
                plan.seg[n] = np.zeros(len(plan.seg[n]))
                pts[n] = self.robots[n].xy

    def _waypoint(self, plan, n, t, d):
        """Where to send n during interval t. The trajectories come in hops of 0.2-0.9 m: stopping at each
        is more than a car with a 0.7 m turning radius can do (real_full_live2: relay1 an hour by the door, relay2
        20 goals for 3 m), so a relay aims at the first waypoint a good stretch ahead and drives through the ones
        before it. It still counts as having reached them one by one and the others wait for it."""
        if n == self.client_name:
            return t
        last = max(i + 1 for i, v in enumerate(plan.seg[n]) if v > 0.05)
        k = t
        while k < last and math.dist(self.robots[n].xy, plan.traj[n][k]) < self.a.relay_lookahead:
            k += 1
        return max(k, d["k"]) if d and d["st"] in (None, GoalStatus.STATUS_SUCCEEDED) else k

    def _station_keep(self):
        for r in self.relays:                                     # also during plans, if not moving in it
            if r.xy is None or r.name not in self.home or r.name in self.keeping or r.name in self.moving:
                continue
            drift = math.dist(r.xy, self.home[r.name])
            # keep_tol well above Nav2's arrival tolerance (0.25 m): at 0.3 m a car that cannot move sideways
            # was re-sent for ever, each time ending up to 0.5 m from the spot again (v3_u_nav2: 58 re-sends)
            # parked: tighter, a relay creeping backwards reached a corner inside 1 m of its spot (real_full)
            tol = self.a.park_tol if self.home[r.name] == self.park.get(r.name) else self.a.keep_tol
            if drift < tol or self.keep_wait.get(r.name, -1) > self._now():
                continue
            why = "going back to its parking spot" if self.home[r.name] == self.park.get(r.name) else "sending it back"
            self.get_logger().info(f"{r.name} {drift:.2f} m from its spot - {why}")
            hx, hy = self.home[r.name]
            g = NavigateToPose.Goal(pose=self._pose((hx, hy), None))
            yaw = self.yaw.get(r.name, 0.0)                       # keep the heading it parked with
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.keeping[r.name] = None
            h = self._send(r.name, g, {})                         # revives its Nav2 if it is down
            if h is None:
                self.keeping.pop(r.name, None); continue
            self.keeping[r.name] = h
            h.get_result_async().add_done_callback(lambda res, n=r.name: self._keep_done(n, res))

    def _execute(self, gh):
        self.busy = True
        self.last_accept = self._now()
        plan = self._last_plan
        self.moving = {n for n in plan.seg if float(np.sum(plan.seg[n])) > 0.05}
        for n, h in list(self.keeping.items()):                   # the plan overrides station keeping
            if n in self.moving:                                  # for the robots it moves
                if h is not None:
                    h.cancel_goal_async()
                self.keeping.pop(n, None)
        try:
            self._reached, self._client_failed, self._missed = False, False, set()
            res = self._execute_plan(gh)
            for n in self.moving - {self.client_name}:           # a relay that keeps missing its
                if n in self._missed:                             # waypoints is benched for a while:
                    self.misses[n] = self.misses.get(n, 0) + 1    # it forwards where it stands
                    if self.misses[n] >= self.a.bench_after:
                        self.benched[n] = self._now() + self.a.bench_time
                        self.misses[n] = 0
                        self.get_logger().warn(f"{n} missed its waypoints {self.a.bench_after} plans in a row - "
                                               f"not moving it for {self.a.bench_time:.0f} s (it still forwards)")
                else:
                    self.misses[n] = 0
            for n, xy in plan.targets.items():                    # where each relay now belongs:
                if n != self.client_name:                         # its target if the plan finished,
                    self.home[n] = tuple(xy) if self._reached else self.robots[n].xy   # else where it is
            # A finished plan is not a reached goal: the client's trajectory may end short of it (t_f cap,
            # signal held at every step). Marking it visited dropped the south corridor and classroom
            # (real_full_live7: "succeeded" 6 m short, then a 0 m plan "succeeded" in 9 ms).  (ours)
            end = plan.traj[self.client_name][-1] if self.client_name in plan.traj else plan.goal
            arrived = math.dist(end, plan.goal) < self.a.visited_r
            progress = float(np.sum(plan.seg.get(self.client_name, [0.0])))
            if self._reached and not self.regrouping and arrived:  # only frontier goals actually reached
                self.visited.append(tuple(plan.goal))
            elif self._reached and not self.regrouping and progress < 0.3:   # a plan that cannot bring the
                k = (round(plan.goal[0], 1), round(plan.goal[1], 1))        # client closer counts as a failure,
                self.failed[k] = self.failed.get(k, 0) + 1                  # or the explorer re-sends it forever
                self.get_logger().info(f"goal world ({plan.goal[0]:.2f}, {plan.goal[1]:.2f}): plan moved the client "
                                       f"{progress:.2f} m - counted as failed ({self.failed[k]})")
            elif self._reached and not self.regrouping:
                self.get_logger().info(f"goal world ({plan.goal[0]:.2f}, {plan.goal[1]:.2f}): plan ended "
                                       f"{math.dist(end, plan.goal):.1f} m short - not marked visited")
            elif self._client_failed:                             # a lagging relay is not the goal's fault
                k = (round(plan.goal[0], 1), round(plan.goal[1], 1))
                self.failed[k] = self.failed.get(k, 0) + 1
            return res
        finally:
            self.busy = False
            self.moving = set()

    def _execute_plan(self, gh):
        plan = self._last_plan
        self.engaged = [n for n in plan.chain if n in {r.name for r in self.relays}]
        self._markers(plan)
        sched = schedule(plan, {n: self.robots[n].vmax for n in plan.seg})
        tf = len(next(iter(plan.seg.values())))      # per plan: the ladder may refine t_f
        fb, drive = {}, {}                           # drive: a relay's goal, it can outlast an interval
        for t in range(1, tf + 1):
            movers = [n for n in plan.seg if plan.seg[n][t - 1] > 0.05]
            if not movers:
                continue
            done, handles = {}, {n: d["h"] for n, d in drive.items() if d["st"] is None}
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
                d = drive.get(n)
                k = self._waypoint(plan, n, t, d)
                if d and d["k"] == k:
                    if d["st"] == GoalStatus.STATUS_SUCCEEDED:
                        done[n] = d["st"]
                        return
                    if d["st"] is None and handles.get(n) is d["h"]:   # already on its way
                        return
                nxt = plan.traj[n][k + 1] if k < tf else None
                if n != self.client_name and (nxt is None or math.dist(nxt, plan.traj[n][k]) < 0.05):
                    nxt = 2 * np.asarray(plan.traj[n][k]) - plan.traj[n][k - 1]   # arrive the way it is going:
                g = NavigateToPose.Goal(pose=self._pose(plan.traj[n][k], nxt))    # yaw 0 had it plan a loop to get there
                fb.pop(n, None)
                h = self._send(n, g, fb)
                if h is None:
                    self.get_logger().warn(f"{n}: waypoint {t} rejected by Nav2 (also after a restart)")
                    done[n] = GoalStatus.STATUS_ABORTED
                    return
                handles[n] = h
                if n != self.client_name:
                    d = drive[n] = {"h": h, "k": k, "st": None}
                    h.get_result_async().add_done_callback(lambda r, d=d: d.__setitem__("st", r.result().status))
                    return
                h.get_result_async().add_done_callback(            # ignore a goal replaced by the watchdog
                    lambda r, n=n, h=h: done.__setitem__(n, r.result().status) if handles.get(n) is h else None)

            t_start = self.get_clock().now()                  # SIM time: plans are in sim seconds
            watch = {"xy": self.client.xy, "t": 0.0, "n": 0, "from": self.client.xy}  # client stuck watchdog
            prog = {"best": math.inf, "t": 0.0}               # client progress along its Nav2 path

            def nav_gave_up():                                # aborted without getting anywhere: as good as stuck
                return self.client_name in handles and done.get(self.client_name) == GoalStatus.STATUS_ABORTED \
                    and watch["n"] < self.a.stuck_tries and math.dist(self.client.xy, watch["from"]) < 0.3
            retries = {}                                      # relay waypoint re-sends
            held, seps, no_hold = {}, {}, {}                  # right of way (_give_way)
            waiting = sorted((n for n in movers if start[n] > 0), key=lambda n: start[n])
            for n in movers:
                if start[n] <= 0:
                    launch(n)
            self._state(event="interval", t=t, dt=dt, v=speed, start_delay=start, movers=movers)
            while len(done) < len(movers) or nav_gave_up():
                if gh.is_cancel_requested:
                    for h in handles.values():
                        h.cancel_goal_async()
                    gh.canceled(); return NavigateToPose.Result()
                el = (self.get_clock().now() - t_start).nanoseconds / 1e9
                for n in movers:                              # how far along the relays are
                    d = drive.get(n)
                    if n == self.client_name or n in done or d is None or handles.get(n) is not d["h"]:
                        continue
                    if d["st"] is not None:
                        done[n] = d["st"]
                    elif t < d["k"] or t < max(i + 1 for i, v in enumerate(plan.seg[n]) if v > 0.05):
                        ahead = float(np.sum(plan.seg[n][t:d["k"]]))
                        if math.dist(self.robots[n].xy, plan.traj[n][t]) < self.a.relay_arrive or \
                                0 < fb.get(n, 0) <= ahead + self.a.relay_arrive:
                            done[n] = GoalStatus.STATUS_SUCCEEDED
                # a relay's Nav2 aborting ("Failed to make progress": Ackermann manoeuvring at a low
                # speed cap) is usually transient -- re-send its waypoint before giving up the plan
                for n in [n for n, st in done.items() if st != GoalStatus.STATUS_SUCCEEDED
                          and n != self.client_name and retries.get(n, 0) < self.a.relay_retries]:
                    retries[n] = retries.get(n, 0) + 1
                    self.get_logger().info(f"interval {t}: {n} aborted - re-sending its waypoint ({retries[n]})")
                    self._free_relay(n)
                    del done[n]
                    launch(n)
                while waiting and el >= start[waiting[0]]:
                    launch(waiting.pop(0))                    # delayed start: arrives with the rest
                # keep the ETAs equal: v_r = remaining_r / T_remaining, never below the floor
                rem = {n: max(fb[n] - float(np.sum(plan.seg[n][t:drive[n]["k"]])) if n in drive else fb[n], 0.0)
                       for n in movers if n in fb and n not in done and n not in held}
                if rem:
                    T = max(rem[n] / self.robots[n].vmax for n in rem)
                    for n in rem:
                        self._limit(n, max(rem[n] / T if T > 0 else self.robots[n].vmax, self._floor(n)))
                self._give_way(t, el, movers, done, handles, held, seps, no_hold, launch)
                # runtime guard, last resort: the plan assumes every robot starts when told; a relay
                # that has not (slow Ackermann start-up) can sit on the client's path (UNIST run 2: 0.27 m)
                cxy = self.client.xy
                close = [r.name for r in self.relays    # a relay standing still is Nav2's to drive around: guarding
                         if r.name in movers and cxy and r.xy and math.dist(cxy, r.xy) < self.a.guard]   # it aborted 257 plans
                if close:
                    self.get_logger().warn(f"interval {t}: client within {self.a.guard} m of {close} - stopping the plan")
                    for h in handles.values():
                        h.cancel_goal_async()
                    for n in self.robots:
                        self._limit(n, self.robots[n].vmax)
                    self._state(event="aborted", t=t, reason="proximity", robots=close)
                    gh.abort()
                    return NavigateToPose.Result()
                # client stuck (no progress although it has a waypoint, or Nav2 gave up where it stands): free it, resend
                if self.client_name in handles:
                    if math.dist(self.client.xy, watch["xy"]) > 0.15:
                        watch["xy"], watch["t"] = self.client.xy, el
                    stalled = self.client_name not in done and el - watch["t"] > self.a.stuck_s
                    if (stalled and watch["n"] < self.a.stuck_tries) or nav_gave_up():
                        watch["n"] += 1
                        h = handles.pop(self.client_name)
                        done.pop(self.client_name, None)
                        self.get_logger().warn(f"interval {t}: client stuck at ({self.client.xy[0]:.2f}, "
                                               f"{self.client.xy[1]:.2f}) - freeing it ({watch['n']})")
                        self._call(h.cancel_goal_async(), 10.0)
                        self._free_client()
                        launch(self.client_name)
                        watch["xy"], watch["from"] = self.client.xy, self.client.xy
                        watch["t"] = (self.get_clock().now() - t_start).nanoseconds / 1e9
                        prog["best"], prog["t"] = math.inf, watch["t"]   # a fresh path
                    elif stalled:                                 # nothing freed it: not this goal
                        self.get_logger().warn(f"interval {t}: client still stuck after {watch['n']} tries - giving up")
                        break
                    # Moving is not progress: shuffling between desk rows kept the watchdog above quiet
                    # for 4 min until the chain wedged and FAST-LIO diverged (real_full_live7). No metre
                    # gained on its path for progress_s: stop pushing, the goal counts as failed.  (ours)
                    rem_c = fb.get(self.client_name, 0.0)
                    if rem_c <= 0:                                # no feedback yet: just (re)sent
                        prog["t"] = el
                    elif rem_c < prog["best"] - 0.2:
                        prog["best"], prog["t"] = rem_c, el
                    # a client that has not moved at all is the stuck watchdog's (it frees it): stopping it
                    # first left the head on a desk for 50 min, re-planned every 30 s (real_full_live13)
                    if self.client_name not in done and el - prog["t"] > self.a.progress_s and rem_c > 0.4 \
                            and el - watch["t"] < self.a.progress_s:      # not its final turn; it moved
                        self.get_logger().warn(f"interval {t}: client gained nothing on its path in "
                                               f"{self.a.progress_s:.0f} s (at {self.client.xy[0]:.2f}, "
                                               f"{self.client.xy[1]:.2f}) - stopping the plan")
                        for h in handles.values():
                            h.cancel_goal_async()
                        for n in self.robots:
                            self._limit(n, self.robots[n].vmax)
                        self._client_failed, self._missed = True, {self.client_name}
                        self._state(event="aborted", t=t, reason="no_progress")
                        gh.abort()
                        return NavigateToPose.Result()
                # slack: a relay needs time to plan and turn (Ackermann) before it moves at all
                if (self.get_clock().now() - t_start).nanoseconds / 1e9 > \
                        self.a.interval_timeout * max(sched[t - 1]["dt"], 1.0) + self.a.interval_slack:
                    self.get_logger().warn(f"interval {t} timed out")
                    break
                time.sleep(0.5)
            failed = [n for n, s in done.items() if s != GoalStatus.STATUS_SUCCEEDED]
            missing = failed + [n for n in movers if n not in done]
            for n in [n for n in missing if n != self.client_name and
                      math.dist(self.robots[n].xy, plan.traj[n][t]) < self.a.relay_arrive]:
                self.get_logger().info(f"interval {t}: {n} is close enough to its waypoint - going on")
                missing.remove(n)
                if n in handles and n not in done:
                    handles[n].cancel_goal_async()
            if missing:
                # Synchronised chain: nobody goes on to waypoint t+1 while someone has not
                # reached waypoint t -- a client running ahead of lagging relays is exactly
                # how the chain breaks. Stop everyone; the explorer asks again and the plan
                # is rebuilt from where the robots really are.
                self.get_logger().warn(f"interval {t}: {missing} did not reach their waypoint - stopping the plan")
                for h in handles.values():
                    h.cancel_goal_async()
                for n in self.robots:
                    self._limit(n, self.robots[n].vmax)
                self._client_failed = self.client_name in missing   # only that counts against the goal
                self._missed = set(missing)
                self._state(event="aborted", t=t, failed=missing)
                gh.abort()
                return NavigateToPose.Result()
        for n in self.robots:
            self._limit(n, self.robots[n].vmax)               # lift the limits
        self._reached = True
        gh.succeed()
        return NavigateToPose.Result()

    def _give_way(self, t, el, movers, done, handles, held, seps, no_hold, launch):
        """Right of way between robots driving at each other: the client first, then relay1, relay2...
        The one with less priority stops until the other has passed, and its Nav2 and the other's
        steer around whatever stands still. A hold is bounded: two robots blocking a corridor must
        not wait for each other for ever."""
        rank = sorted(movers, key=lambda n: (n != self.client_name, n))
        driving = [n for n in rank if n in handles and n not in done]
        for n in list(held):
            m, t0, sep0 = held[n]
            sep = math.dist(self.robots[n].xy, self.robots[m].xy)
            if m in done or sep > max(self.a.yield_r, sep0) + 0.2 or el - t0 > self.a.yield_max:
                self.get_logger().info(f"interval {t}: {n} goes on ({m} at {sep:.2f} m, held {el - t0:.0f} s)")
                del held[n]
                no_hold[n] = el + self.a.yield_max              # gets its turn: no second hold straight away
                launch(n)
        for i, n in enumerate(rank):
            if n == self.client_name or n not in driving or n in held or el < no_hold.get(n, 0.0):
                continue
            for m in rank[:i]:
                if m not in driving or m in held:
                    continue
                sep = math.dist(self.robots[n].xy, self.robots[m].xy)
                closing = sep < seps.get((n, m), np.inf)
                seps[(n, m)] = sep
                if sep < self.a.yield_r and closing:
                    self.get_logger().info(f"interval {t}: {n} gives way to {m} ({sep:.2f} m)")
                    handles.pop(n).cancel_goal_async()
                    held[n] = (m, el, sep)
                    self._state(event="give_way", t=t, robot=n, to=m, sep=round(sep, 2))
                    break

    def _free_client(self):
        """A folded chain stays folded under reverse and spin, and Nav2 refuses every move inside its
        lethal band; driving forward and turning at once opens the fold. These commands skip Nav2's
        collision check, so each is used only where the explored map shows clear floor."""
        x, y = self.client.xy
        th = self.yaw[self.client_name]
        self._explored()

        def clear(px, py):
            return self._is_explored((x + px * math.cos(th) - py * math.sin(th), y + px * math.sin(th) + py * math.cos(th)))
        room = {}
        for sg in (1.0, -1.0):                            # free length along each arc, 0.4 m radius
            room[sg] = 0
            for i in range(1, 17):
                a = i * 0.2
                if not clear(0.4 * math.sin(a), sg * 0.4 * (1 - math.cos(a))):
                    break
                room[sg] = i
        sg = 1.0 if room[1.0] >= room[-1.0] else -1.0
        if room[sg] >= 5:
            self.get_logger().info(f"freeing the client: forward arc, turn {sg:+.0f}")
            self._drive_client(0.2, sg * 0.5, 8.0)
        elif all(clear(-d * 0.05, 0.0) for d in range(2, 13)):
            self.get_logger().info("freeing the client: straight back")
            self._drive_client(-0.2, 0.0, 6.0)
        else:
            self.get_logger().info("freeing the client: no clear floor on the map, asking Nav2 to back up")
            self._back_up(0.6)

    def _free_relay(self, n):
        """A relay stopped beside a door jamb starts inside its Nav2's lethal band: every plan aborts
        ("Starting point in lethal space") and re-sending the waypoint aborts again (real_full_live12:
        relay1 at the west classroom door, benched, an hour lost). Drive it straight along its heading,
        at most 0.5 m forward or back, to the roomiest explored floor; nothing if none is roomier."""
        th = self.yaw.get(n)
        if th is None or not self._explored():
            return
        g = self.planner.grid
        clr = distance_transform_edt(self.planner.allowed) * g.res
        x, y = self.robots[n].xy

        def at(d):
            q = (x + d * math.cos(th), y + d * math.sin(th))
            r_, c_ = g.world_to_cell(*q)
            return float(clr[r_, c_]) if g.in_bounds(r_, c_) and self._is_explored(q) else -1.0
        c0, best = at(0.0), (0.0, at(0.0))
        for sg in (1.0, -1.0):
            for i in range(1, 11):
                c = at(sg * 0.05 * i)
                if c < 0:
                    break                                 # unexplored or off the floor: not past it
                if c > best[1] + 0.02:
                    best = (sg * 0.05 * i, c)
        d, c = best
        if c - c0 < 0.1:
            return
        self.get_logger().info(f"freeing {n}: {'forward' if d > 0 else 'back'} {abs(d):.2f} m "
                               f"(clearance {c0:.2f} -> {c:.2f} m)")
        if n not in self.relay_cmd:
            self.relay_cmd[n] = self.create_publisher(Twist, f"/{n}/cmd_vel", 10)
        self._drive(self.relay_cmd[n], math.copysign(0.15, d), 0.0, abs(d) / 0.15)

    def _drive_client(self, v, w, secs):
        self._drive(self.client_cmd, v, w, secs)

    def _drive(self, pub, v, w, secs):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds / 1e9 < secs:
            pub.publish(m)
            time.sleep(0.05)
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(0.05)

    def _back_up(self, dist):
        if not self.backup.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/backup not available")
            return
        g = BackUp.Goal(speed=0.15)
        g.target.x = float(dist)
        g.time_allowance.sec = 30
        h = self._call(self.backup.send_goal_async(g), 10.0)
        if h is not None and h.accepted:
            self._call(h.get_result_async(), 60.0)

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


def frontier_goals(g, free, unknown, reach, xy, min_cells=8, clear=0.6, snap=1.0):
    """Goals on the explored floor where it meets unknown, best first: for each stretch of floor touching
    unknown, the roomy cell closest to its middle. Long stretches and near ones first: a doorway into an
    unseen room beats a speck beside a desk. clear: the client jams in the aisles between desks (real_full_live2:
    20 min at a goal 0.3 m from one). Unknown behind a wall does not count: the two must touch, and walls are
    occupied."""
    edge = reach & binary_dilation(unknown, structure=np.ones((3, 3)))
    lab, n = label(edge, structure=np.ones((3, 3)))
    roomy = reach & (distance_transform_edt(free) * g.res >= clear)
    if not n or not roomy.any():
        return []
    d, (nr, nc) = distance_transform_edt(~roomy, return_indices=True)
    out = []
    for i, cells in enumerate(find_objects(lab), 1):
        rc = np.argwhere(lab[cells] == i) + [cells[0].start, cells[1].start]
        if len(rc) < min_cells:
            continue
        r, c = rc[np.argmin(((rc - rc.mean(0)) ** 2).sum(1))]
        if d[r, c] * g.res <= snap:
            p = g.cell_to_world(nr[r, c], nc[r, c])
            out.append((len(rc) * g.res / (math.dist(p, xy) + 2.0), float(p[0]), float(p[1])))
    return [(x, y) for _, x, y in sorted(out, reverse=True)]


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--gt", default="hospital_world/gt/hospital_gt", help="ground-truth grid, repo-relative")
    ap.add_argument("--config", default=os.path.join(here, "nodes.yaml"))
    ap.add_argument("--cache-dir", default="/tmp/fair_cache")
    ap.add_argument("--P0", type=float, default=-30.0)
    ap.add_argument("--n", type=float, default=PropagationParams.n)
    ap.add_argument("--alpha", type=float, default=PropagationParams.alpha)
    ap.add_argument("--sigma-n", type=float, default=3.0)
    ap.add_argument("--v-client", type=float, default=0.5, help="chain_robot DWB max_vel_x (m/s)")
    ap.add_argument("--v-relay", type=float, default=0.5)
    # DWB samples no velocity at all below min_vel_x / min_speed_xy (0.2): a speed limit under
    # that freezes chain_robot ("No valid trajectories out of 0") -- it looked wedged on walls
    ap.add_argument("--v-min", type=float, default=0.22, help="client speed-limit floor, m/s (DWB min 0.2)")
    ap.add_argument("--relay-retries", type=int, default=2, help="re-sends of an aborted relay waypoint")
    ap.add_argument("--v-floor-relay", type=float, default=0.25,
                    help="relay speed floor, m/s: the drive stalls at 0.10 m/s, moves at 0.15")
    ap.add_argument("--interval-timeout", type=float, default=6.0, help="x planned interval duration")
    ap.add_argument("--interval-slack", type=float, default=90.0, help="+ sim seconds per interval (start-up)")
    ap.add_argument("--map-offset", nargs=2, type=float, default=[0.0, 12.0], help="world = map + offset")
    ap.add_argument("--keep-deployed", action="store_true")
    ap.add_argument("--d-sep", type=float, default=PlannerParams.d_sep, help="planned robot separation (m)")
    ap.add_argument("--yield-r", type=float, default=0.8, help="a relay closing within this of a robot with right of way stops (m)")
    ap.add_argument("--yield-max", type=float, default=30.0, help="longest such stop (sim s)")
    ap.add_argument("--guard", type=float, default=0.38,
                    help="stop the plan if the client comes this close to a relay (m): near contact; passing a parked relay in a corridor is fine")
    ap.add_argument("--goal-off-floor", type=float, default=0.5,
                    help="reject goals further than this from the explored floor the client is on (m)")
    ap.add_argument("--pass-clear", type=float, default=0.6,
                    help="a robot standing in a corridor 2x this wide does not block routes (m); the planner's 0.9 "
                         "walled off real_env's 1.46 m corridor behind a relay (real_full_live5: an hour at 57%%)")
    ap.add_argument("--goal-clear", type=float, default=0.6, help="room a client goal needs from known obstacles (m)")
    ap.add_argument("--goal-snap", type=float, default=1.0, help="how far a too-tight goal may move to get that (m)")
    ap.add_argument("--own-goal-after", type=float, default=60.0,
                    help="sim s without an accepted goal -> plan for a frontier of our own")
    ap.add_argument("--own-goal-every", type=float, default=20.0, help="... at most this often (sim s)")
    ap.add_argument("--own-goal-tries", type=int, default=3, help="... trying this many frontiers (each plan costs s)")
    ap.add_argument("--fill-cells", type=int, default=3, help="close unknown specks up to 2x this wide in the map")
    ap.add_argument("--max-fail", type=int, default=2, help="reject a goal after this many failed attempts")
    ap.add_argument("--reject-reset", type=float, default=120.0, help="sim s without an accepted goal -> forget failed and visited goals")
    ap.add_argument("--map-topic", default="/map_mesh", help="the client's map; on the mesh, the throttled one")
    ap.add_argument("--stuck-s", type=float, default=40.0, help="sim s without client progress -> free it")
    ap.add_argument("--progress-s", type=float, default=30.0,
                    help="sim s without 0.2 m gained on the client's Nav2 path -> stop the plan")
    ap.add_argument("--stuck-tries", type=int, default=6, help="tries at freeing the client per interval before giving up the plan")
    ap.add_argument("--max-skip", type=int, default=2,
                    help="reject a goal skipped this many times at once, until a robot moves >= 1.5 m ...")
    ap.add_argument("--skip-cooldown", type=float, default=180.0, help="... or this many sim s passed")
    ap.add_argument("--regroup-after", type=int, default=6, help="skips in a row -> client regroups at the GCS")
    ap.add_argument("--bench-after", type=int, default=5, help="plans a relay may miss in a row before benching")
    ap.add_argument("--bench-time", type=float, default=300.0, help="sim s a benched relay is not moved")
    ap.add_argument("--map-log", default="", help="directory for /map snapshots (replay video)")
    ap.add_argument("--map-log-period", type=float, default=5.0, help="sim s between snapshots")
    ap.add_argument("--visited-r", type=float, default=0.6, help="reject goals this close to a reached one (m)")
    ap.add_argument("--relay-lookahead", type=float, default=1.5, help="a relay aims at the first waypoint this far off")
    ap.add_argument("--relay-arrive", type=float, default=0.6, help="this close counts as past a relay waypoint (m)")
    ap.add_argument("--relay-min-move", type=float, default=0.6, help="a relay's shorter moves are dropped if the link holds")
    ap.add_argument("--keep-tol", type=float, default=1.0, help="station keeping: drift (m) that triggers a return")
    ap.add_argument("--park-tol", type=float, default=0.6, help="the same for a relay at its parking spot (m)")
    ap.add_argument("--keep-wait", type=float, default=60.0, help="sim s before a failed return is tried again, doubled per failure up to 8 x")
    ap.add_argument("--idle-after", type=float, default=20.0, help="sim s off the route -> relay goes back to park")
    ap.add_argument("--state-log", default="", help="JSON lines, each second: every relay's state, the route")
    ap.add_argument("--radio-gt", default="", help="grid for the radio model if not --gt, repo-relative")
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
