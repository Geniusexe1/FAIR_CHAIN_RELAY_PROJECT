#!/usr/bin/env python3
"""Sim only: a stopped relay stays stopped. A real car stands still with no command; in Gazebo the
Ackermann model creeps about 1.7 mm/s with its wheel PID at zero, whatever the gains or friction
(relays crept 3 m in an hour, real_full_live2). After HOLD_S with no non-zero command each relay's
pose is held; the first non-zero command lets go. Uses the world's libgazebo_ros_state.so."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState

HOLD_S = 3.0     # wall seconds
PERIOD = 0.5


class Brake(Node):
    def __init__(self, names):
        super().__init__("parking_brake")
        self.names = names
        self.moved = {n: time.time() for n in names}
        self.held = {n: None for n in names}
        self.get = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.put = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        for n in names:
            self.create_subscription(Twist, f"/{n}/cmd_vel", lambda m, n=n: self.cmd(n, m), qos_profile_sensor_data)
        self.create_timer(PERIOD, self.tick)

    def cmd(self, n, m):
        if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3:
            self.moved[n] = time.time()
            if self.held[n] is not None:
                self.held[n] = None
                self.get_logger().info(f"{n}: released")

    def tick(self):
        for n in self.names:
            if time.time() - self.moved[n] < HOLD_S:
                continue
            if self.held[n] is None:
                self.get.call_async(GetEntityState.Request(name=n, reference_frame="world")).add_done_callback(
                    lambda f, n=n: self.got(n, f))
            else:
                r = SetEntityState.Request()
                r.state.name, r.state.reference_frame, r.state.pose = n, "world", self.held[n]
                self.put.call_async(r)

    def got(self, n, f):
        res = f.result()
        if res is not None and res.success and time.time() - self.moved[n] >= HOLD_S and self.held[n] is None:
            self.held[n] = res.state.pose
            p = res.state.pose.position
            self.get_logger().info(f"{n}: held at ({p.x:.2f}, {p.y:.2f})")


def main():
    rclpy.init()
    n = Brake(sys.argv[1:])
    if not n.put.wait_for_service(timeout_sec=30.0):
        n.get_logger().warn("no /gazebo/set_entity_state in this world: no brake")
        return
    rclpy.spin(n)


if __name__ == "__main__":
    main()
