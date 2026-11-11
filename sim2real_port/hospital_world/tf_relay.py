#!/usr/bin/env python3
"""One relay's own TF: the frames of this relay from the global /tf, for its Nav2 on this NUC only.

    python3 tf_relay.py relay1

Every Nav2 process listens to TF. On the global topic that makes ~38 reliable subscribers across the
radios for all robots' transforms: ~20,000 DDS heartbeats/s and the mesh collapses. Here one best-effort
subscription crosses the mesh and Nav2 reads /<ns>/tf and /<ns>/tf_static locally.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self, ns):
        super().__init__("tf_relay", namespace=ns)
        self.prefix = ns + "/"
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TFMessage, f"/{ns}/tf", QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_static = self.create_publisher(TFMessage, f"/{ns}/tf_static", latched)
        self.create_subscription(TFMessage, "/tf", self.on_tf, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TFMessage, "/tf_static", self.on_static, latched)
        self.static = {}

    def mine(self, msg):
        return [t for t in msg.transforms if t.child_frame_id.startswith(self.prefix)]

    def on_tf(self, msg):
        ts = self.mine(msg)
        if ts:
            self.pub.publish(TFMessage(transforms=ts))

    def on_static(self, msg):
        for t in self.mine(msg):
            self.static[t.child_frame_id] = t
        if self.static:                                   # latched: each message replaces the last
            self.pub_static.publish(TFMessage(transforms=list(self.static.values())))


def main():
    rclpy.init(args=sys.argv[2:])
    rclpy.spin(TfRelay(sys.argv[1]))


if __name__ == "__main__":
    main()
