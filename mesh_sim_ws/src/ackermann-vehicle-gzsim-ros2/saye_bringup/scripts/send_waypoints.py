#!/usr/bin/env python3
"""Send navigation goals to either robot.

  ros2 run saye_bringup send_waypoints.py robot1 3.0 0.0
  ros2 run saye_bringup send_waypoints.py robot2 3.0 0.0  6.0 2.0  6.0 -2.0

One x,y pair  -> NavigateToPose  (drive there, avoiding obstacles)
Several pairs -> FollowWaypoints (visit each in order)

Coordinates are metres in the **map** frame (not odom).

Nav2 handles obstacle avoidance itself: the local costmap is built from
/<ns>/scan and the controller steers around whatever it sees.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from tf2_ros import Buffer, TransformListener

# If a result comes back faster than this with no feedback, the goal-response
# handshake failed and the "status" is meaningless -- see _sanity_check().
IMPLAUSIBLY_FAST = 1.0


def make_pose(node, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'          # global frame, shared by both robots
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    if len(sys.argv) < 4 or len(sys.argv) % 2 != 0:
        print(__doc__)
        sys.exit(1)

    ns = sys.argv[1]
    coords = [float(v) for v in sys.argv[2:]]
    pts = list(zip(coords[0::2], coords[1::2]))

    rclpy.init()
    node = Node('send_waypoints',
                parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    log = node.get_logger()

    if len(pts) == 1:
        client = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose')
        action, goal = 'navigate_to_pose', NavigateToPose.Goal()
        goal.pose = make_pose(node, *pts[0])
    else:
        client = ActionClient(node, FollowWaypoints, f'/{ns}/follow_waypoints')
        action, goal = 'follow_waypoints', FollowWaypoints.Goal()
        goal.poses = [make_pose(node, x, y) for x, y in pts]

    log.info(f'waiting for /{ns}/{action} ...')
    if not client.wait_for_server(timeout_sec=20.0):
        log.error(f'/{ns}/{action} not available - is Nav2 running for {ns}?')
        sys.exit(2)

    # wait_for_server() returns as soon as the ACTION exists, but the feedback
    # and result topics may still be completing discovery. Sending immediately
    # made the server's goal-response time out ("Failed to send goal response"),
    # after which the result future resolved instantly with a meaningless
    # status=4 -- i.e. it reported SUCCESS while the robot never moved.
    time.sleep(1.5)

    # Nav2's NavigateToPose feedback carries current_pose (already in the map
    # frame) and distance_remaining. Using it avoids depending on this process
    # receiving the full TF tree, which proved unreliable in a fresh container
    # (it saw SLAM's map->odom but not Gazebo's odom->base_link).
    feedback_count = {'n': 0}
    fb_state = {'pose': None, 'dist': None}

    def on_feedback(msg):
        feedback_count['n'] += 1
        fb = msg.feedback
        p = getattr(fb, 'current_pose', None)
        if p is not None:
            fb_state['pose'] = (round(p.pose.position.x, 3), round(p.pose.position.y, 3))
        d = getattr(fb, 'distance_remaining', None)
        if d is not None:
            fb_state['dist'] = round(float(d), 3)

    log.info(f'sending {len(pts)} waypoint(s) to {ns}: {pts}')
    t0 = time.time()
    fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    handle = fut.result()
    if handle is None:
        log.error('no goal response from server (handshake timed out)')
        sys.exit(3)
    if not handle.accepted:
        log.error('goal REJECTED')
        sys.exit(3)

    log.info('goal accepted, navigating...')
    res_fut = handle.get_result_async()

    # DO NOT trust the action status alone. On this setup the result response is
    # frequently not delivered (server logs "Failed to send goal response"), and
    # the future then either hangs or resolves with a meaningless status=4.
    # The ground truth is where the robot actually IS, in the goal's own frame.
    tf_buf = Buffer()
    # spin_thread=False on purpose. spin_thread=True gives the listener its OWN
    # executor for this node while the loop below also calls rclpy.spin_once() on
    # it. rclpy does not support one node in two executors: the /tf callbacks stop
    # being serviced, every lookup_transform raises, and the loop then spins out
    # its full deadline reporting nothing -- with the robot parked on the goal.
    # One executor (this loop, at 5 Hz) drives both the action feedback and TF.
    TransformListener(tf_buf, node, spin_thread=False)
    target = pts[-1]
    last_d = None
    final_xy = None
    # WALL-clock budget. The sim runs well below real time (RTF ~0.27 measured on
    # the hospital world), so 300 s wall was only ~80 s of sim time -- enough for a
    # straight run but not for a multi-point Ackermann maneuver, which then reported
    # a false TIMEOUT while the robot was still legitimately converging.
    deadline = time.time() + 900.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        # 1st choice: TF. It is the GROUND TRUTH and, unlike feedback, it keeps
        # updating after the goal ends. Preferring feedback here was a bug: Nav2
        # stops publishing feedback the moment the goal completes, so the last
        # value seen is often the pose from early in the drive. The robot then
        # arrives, `fb_state['pose']` stays frozen far from the target, the
        # distance never drops below the threshold, and this loop spun out the
        # full 300 s reporting nothing -- while Nav2 had already logged
        # "Goal succeeded". That made a working goal look like a dead one.
        try:
            tr = tf_buf.lookup_transform('map', f'{ns}/base_link',
                                         rclpy.time.Time()).transform.translation
            final_xy = (round(tr.x, 3), round(tr.y, 3))
            last_d = math.hypot(tr.x - target[0], tr.y - target[1])
        except Exception:
            # 2nd choice: Nav2 feedback, if this process lacks the full TF tree.
            if fb_state['pose'] is not None:
                final_xy = fb_state['pose']
                last_d = math.hypot(final_xy[0] - target[0], final_xy[1] - target[1])
        # 0.30 was far too loose and MASKED the controller's real accuracy: it broke
        # the moment the robot first came within 30 cm, reported that first sample as
        # the final result, and never saw the convergence that followed. Measured
        # properly, the car settles to 2-4 cm (approach: 0.59 -> 0.051 -> 0.114
        # overshoot -> 0.087 -> 0.064 -> 0.040 -> 0.021). Now matches the goal
        # checker's xy_goal_tolerance so the number reported is the number achieved.
        if last_d is not None and last_d < 0.08:
            break
        if res_fut.done() and (time.time() - t0) > IMPLAUSIBLY_FAST:
            break
    else:
        log.error('TIMEOUT after 900s')

    elapsed = time.time() - t0
    result = res_fut.result() if res_fut.done() else None
    status = result.status if result is not None else 'no-result-response'

    # A real navigation always produces feedback and takes real time.
    if elapsed < IMPLAUSIBLY_FAST and feedback_count['n'] == 0:
        log.error(f'BOGUS RESULT: returned status={status} after {elapsed:.2f}s '
                  f'with zero feedback -- the goal was never executed. '
                  f'Verify with: ros2 run tf2_ros tf2_echo map {ns}/base_link')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(4)

    if last_d is None:
        log.error(f'could not measure position (no map->{ns}/base_link transform)')
    else:
        verdict = 'ARRIVED' if last_d < 0.08 else 'DID NOT ARRIVE'
        log.info(f'{verdict}: at {final_xy}, goal {target}, off by {last_d:.3f} m '
                 f'| {elapsed:.1f}s | action status={status} '
                 f'(status is unreliable here - trust the distance)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
