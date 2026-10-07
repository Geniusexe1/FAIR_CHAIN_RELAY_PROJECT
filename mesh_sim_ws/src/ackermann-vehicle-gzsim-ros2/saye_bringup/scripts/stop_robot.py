#!/usr/bin/env python3
"""Cancel a robot's navigation and bring it to a halt.

  ros2 run saye_bringup stop_robot.py robot2

Cancels any active NavigateToPose / FollowWaypoints goal, then publishes zero
velocity. Cancelling alone is not enough: the last non-zero command can leave
the car coasting, so we explicitly zero cmd_vel afterwards.
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.srv import CancelGoal


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else 'robot2'
    rclpy.init()
    node = Node('stop_robot',
                parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])

    for action_type, name in ((NavigateToPose, 'navigate_to_pose'),
                              (FollowWaypoints, 'follow_waypoints')):
        cli = ActionClient(node, action_type, f'/{ns}/{name}')
        if cli.wait_for_server(timeout_sec=3.0):
            # cancel every goal this server currently has
            from action_msgs.srv import CancelGoal
            srv = node.create_client(CancelGoal, f'/{ns}/{name}/_action/cancel_goal')
            if srv.wait_for_service(timeout_sec=3.0):
                req = CancelGoal.Request()          # empty = cancel all goals
                f = srv.call_async(req)
                rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
                node.get_logger().info(f'cancelled goals on /{ns}/{name}')
        cli.destroy()

    pub = node.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
    stop = Twist()
    for _ in range(20):                      # outlast the controller's last command
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.get_logger().info(f'{ns} stopped')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
