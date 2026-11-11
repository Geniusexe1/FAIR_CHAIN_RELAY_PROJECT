#!/usr/bin/env python3
"""Relay driving bench on the real map: one relay at a time drives fixed legs, the other stays parked.

    python3 relay_bench.py <results.jsonl> <label> <gt.npy> <gt.json>     (in safe_robot, domain 42)

Publishes the ground-truth grid as the relays' map, so the bench does not depend on exploration.
A leg counts as done when Nav2 reports success inside the time limit.
"""
import json, math, sys, time
import numpy as np, rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry

SPAWN = (0.4, -0.2)     # map frame = chain_robot's spawn
LIMIT = 150.0           # sim s per leg
SPEED = 0.25            # the relay floor speed the coordinator most often sets
LEGS = [("corner out", (0.3, 0.2)), ("through door", (2.4, 0.4)), ("corridor south", (2.4, -5.0)),
        ("u-turn north", (2.4, -1.2)), ("back through door", (0.3, -0.8)), ("east aisle south", (0.2, -6.0)),
        ("into row aisle", (-4.0, -3.9)), ("back along aisle", (0.2, -3.9)), ("home", None)]


class Bench(Node):
    def __init__(self, out, label, gt, meta):
        super().__init__("relay_bench", parameter_overrides=[])
        self.out, self.label = open(out, "a", buffering=1), label
        self.xy = {}
        for n in ("relay1", "relay2"):
            self.create_subscription(Odometry, f"/{n}/odom", lambda m, n=n: self.xy.__setitem__(n, (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
        self.nav = {n: ActionClient(self, NavigateToPose, f"/{n}/navigate_to_pose") for n in ("relay1", "relay2")}
        self.lim = {n: self.create_publisher(SpeedLimit, f"/{n}/speed_limit", 10) for n in ("relay1", "relay2")}
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, "/map_mesh", q)
        g = OccupancyGrid()
        g.header.frame_id = "map"
        occ = np.load(gt)
        g.info.resolution = float(json.load(open(meta))["resolution"])
        g.info.height, g.info.width = occ.shape
        o = json.load(open(meta))["origin_xy"]
        g.info.origin.position.x, g.info.origin.position.y = o[0] - SPAWN[0], o[1] - SPAWN[1]
        g.info.origin.orientation.w = 1.0
        g.data = (occ.astype(np.int8) * 100).ravel().tolist()
        self.grid = g
        self.create_timer(5.0, self._repub)
        self._repub()

    def _repub(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)

    def spin_until(self, fut, timeout):
        t = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.done()

    def wait_pose(self, n):
        while n not in self.xy:
            rclpy.spin_once(self, timeout_sec=0.2)

    def leg(self, n, name, xy, trial):
        self.wait_pose(n)
        s = SpeedLimit(); s.speed_limit = SPEED; s.percentage = False
        self.lim[n].publish(s)
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = xy[0] - SPAWN[0], xy[1] - SPAWN[1]
        g.pose.pose.orientation.w = 1.0
        start = self.xy[n]
        if not self.nav[n].wait_for_server(timeout_sec=30):
            self.out.write(json.dumps(dict(label=self.label, relay=n, leg=name, trial=trial, result="no server")) + "\n"); return
        h = self.nav[n].send_goal_async(g)
        self.spin_until(h, 30)
        gh = h.result() if h.done() else None
        if gh is None or not gh.accepted:
            self.out.write(json.dumps(dict(label=self.label, relay=n, leg=name, trial=trial, result="rejected")) + "\n"); return
        res = gh.get_result_async()
        t0, c0 = time.time(), self.get_clock().now()
        length, prev = 0.0, self.xy[n]
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            length += math.dist(prev, self.xy[n]); prev = self.xy[n]
            if (self.get_clock().now() - c0).nanoseconds / 1e9 > LIMIT or time.time() - t0 > LIMIT * 4:
                c = gh.cancel_goal_async(); self.spin_until(c, 15)
                break
        status = res.result().status if res.done() else -1
        d = dict(label=self.label, relay=n, leg=name, trial=trial, result="ok" if status == GoalStatus.STATUS_SUCCEEDED else ("timeout" if status == -1 or status == GoalStatus.STATUS_CANCELED else "aborted"),
                 sim_s=round((self.get_clock().now() - c0).nanoseconds / 1e9, 1), wall_s=round(time.time() - t0, 1), path_m=round(length, 2), straight_m=round(math.dist(start, xy), 2), end_err_m=round(math.dist(self.xy[n], xy), 2), t=time.time())
        self.out.write(json.dumps(d) + "\n"); print(d, flush=True)


def main():
    out, label, gt, meta = sys.argv[1:5]
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    b = Bench(out, label, gt, meta)
    t = time.time()
    while time.time() - t < 10:
        rclpy.spin_once(b, timeout_sec=0.2)
    homes = {"relay1": (-0.4, 2.6), "relay2": (0.5, 2.6)}
    for n in ("relay2", "relay1"):
        for name, xy in LEGS:
            b.leg(n, name, xy or homes[n], 1)
    print("BENCH_DONE", label, flush=True)


if __name__ == "__main__":
    main()
