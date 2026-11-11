#!/usr/bin/env python3
"""/map for everyone across the mesh, at a rate the radios can carry: /map_mesh every PERIOD seconds.

    python3 map_throttle.py [period_s]     (default 4, wall time)

The mapping node publishes ~345 KB about twice a second: 5.5 Mbit/s per subscriber, to the GCS and each
relay. Bursts of loss followed. Robots on this host keep reading /map itself.
"""
import sys
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapThrottle(Node):
    def __init__(self, period):
        super().__init__("map_throttle")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.last = None
        self.pub = self.create_publisher(OccupancyGrid, "/map_mesh", latched)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "last", m), latched)
        self.create_timer(period, self.tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def tick(self):
        if self.last is not None:
            self.pub.publish(self.last)
            self.last = None


def main():
    rclpy.init()
    rclpy.spin(MapThrottle(float(sys.argv[1]) if len(sys.argv) > 1 else 4.0))


if __name__ == "__main__":
    main()
