"""Two Ackermann robots in ONE Gazebo world (senior's Phase 2).

Step A: Gazebo server+client launched exactly once.
Step B: per-robot spawn included twice, each in its own namespace.
Step C: distinct spawn coordinates so they don't interpenetrate and explode.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from robot_spawn import spawn_robot_group, clock_bridge_node, rviz_config_for  # noqa: E402


def generate_launch_description():
    pkg_description = get_package_share_directory('saye_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Step A -- Gazebo ONCE.
    # GZ_HEADLESS=1 runs the server only (no GUI). Needed on PC2, which has no
    # display, and for verifying a launch over SSH.
    _world = os.path.join(pkg_description, 'worlds', 'saye_world.sdf')
    # '-r' = run immediately. Without it Gazebo starts PAUSED: the clock stays at 0,
    # no odometry is published, and cmd_vel does nothing -- with no error anywhere.
    _gz_args = ('-s -r ' + _world) if os.environ.get('GZ_HEADLESS') else ('-r ' + _world)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': _gz_args}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value=('false' if os.environ.get('GZ_HEADLESS') else 'true'), description='Open RViz.'),

        gz_sim,

        # A4 -- exactly one /clock, outside both robot namespaces.
        clock_bridge_node(),

        # Step B + C -- two robots, separated in space and in namespace.
        # Offset along Y, NOT X: these are Ackermann cars that drive along +X,
        # so an X offset puts robot2 directly in robot1's path (verified -- it
        # got rear-ended and pushed 0.24 m).
        spawn_robot_group('robot1', x=0.0, y=0.0),
        spawn_robot_group('robot2', x=0.0, y=2.0),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_for('robot1')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
