"""Nav2 for ONE robot: obstacle avoidance + waypoint following.

Uses nav2_bringup's navigation_launch.py (planner, controller, costmaps,
bt_navigator, waypoint_follower). Deliberately NOT bringup_launch.py, because
that also starts map_server + AMCL -- we already get map->odom from SLAM, and
two things publishing that transform fight each other.

Obstacle avoidance comes from the costmaps' obstacle/voxel layers consuming
/<ns>/scan. Waypoints come from the /<ns>/navigate_to_pose action (single) and
/<ns>/follow_waypoints (a list).

    ros2 launch saye_bringup nav_robot.launch.py            # uses $ROBOT_NS
    ros2 launch saye_bringup nav_robot.launch.py namespace:=robot1
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.conditions import IfCondition


def _setup(context, *args, **kwargs):
    """Deferred so the RESOLVED `namespace` drives everything.

    `os.environ['ROBOT_NS']` must not be used to build frames or params here.
    It is only the DEFAULT for the `namespace` argument; reading it directly
    meant `namespace:=robot2` still produced a `map -> robot1/odom` static
    transform and a params file substituted with `robot1`. On a single NUC that
    put a second publisher on an edge slam_toolbox already owned -- the exact
    two-publishers-on-one-TF-edge failure this stack keeps hitting.
    """
    pkg_bringup = get_package_share_directory('saye_bringup')
    ns = LaunchConfiguration('namespace').perform(context)

    # Same ${NS} substitution trick used for the SDF and slam params.
    with open(os.path.join(pkg_bringup, 'config', 'nav2_params.yaml.in')) as f:
        text = f.read().replace('${NS}', ns)
    fd, params = tempfile.mkstemp(prefix=f'{ns}_nav2_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(text)

    pkg_nav2 = get_package_share_directory('nav2_bringup')

    return [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_map_odom',
            arguments=[
                '--x', LaunchConfiguration('map_odom_x'),
                '--y', LaunchConfiguration('map_odom_y'),
                '--z', '0',
                '--yaw', LaunchConfiguration('map_odom_yaw'),
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', ns + '/odom',
            ],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('static_map_odom')),
        ),

        # nav2_bringup's navigation_launch.py (Humble) re-roots the params under
        # `namespace` via RewrittenYaml(root_key=...) but does NOT put the nodes
        # in that namespace -- there is no PushRosNamespace in its GroupAction.
        # The nodes then come up as /controller_server while their parameters sit
        # under `robot2:`, nothing matches, and it dies with the misleading
        # "No critics defined for FollowPath". Pushing the namespace ourselves
        # makes the node FQNs line up with the rewritten param tree.
        GroupAction([
            PushRosNamespace(ns),
            # navigation_launch.py hard-remaps ('/tf' -> 'tf'), which under a
            # namespace resolves to /<ns>/tf. Our design keeps ONE GLOBAL /tf
            # separated by frame prefix (see NAMESPACING.md A5), so point the
            # nav2 nodes back at the global topics.
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'namespace': ns,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'params_file': params,
                    'use_composition': 'False',
                }.items(),
            ),
        ]),
    ]


def generate_launch_description():
    # Only the DEFAULT comes from the environment; everything else reads the
    # resolved `namespace` argument inside _setup().
    ns_default = os.environ.get('ROBOT_NS', 'robot1')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=ns_default),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # Only ONE robot can run SLAM, because /map is global and shared. The other
        # robot still needs a map-><ns>/odom link, or its tree roots at <ns>/odom
        # instead of map.
        #
        # IDENTITY is correct here, and the offsets below default to zero.
        # Gazebo's OdometryPublisher does NOT report a spawn-relative pose -- it
        # publishes the model's WORLD pose, so <ns>/odom is already coincident with
        # the world origin no matter where the model was spawned. Verified: with
        # robot2 spawned at y=2.0, /robot2/odom and the gz world pose agree to 16
        # significant figures. Setting these to the spawn pose double-counts it and
        # puts the robot 2 m off in the map.
        #
        # The offsets exist only for the case where the MAP origin is deliberately
        # not the world origin. Stays default-false so it never fights slam_toolbox,
        # which owns this edge for the mapping robot.
        DeclareLaunchArgument('static_map_odom', default_value='false',
                              description='Publish a static map-><ns>/odom (use when this robot has no SLAM)'),
        DeclareLaunchArgument('map_odom_x', default_value='0.0',
                              description='map-><ns>/odom x offset; 0 unless the map origin is not the world origin'),
        DeclareLaunchArgument('map_odom_y', default_value='0.0',
                              description='map-><ns>/odom y offset; 0 unless the map origin is not the world origin'),
        DeclareLaunchArgument('map_odom_yaw', default_value='0.0',
                              description='map-><ns>/odom yaw offset; 0 unless the map origin is not the world origin'),

        OpaqueFunction(function=_setup),
    ])
