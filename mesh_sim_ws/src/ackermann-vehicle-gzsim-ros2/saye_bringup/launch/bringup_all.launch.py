"""Everything, in one launch: Gazebo + both robots + SLAM + a Nav2 stack each.

    ros2 launch saye_bringup bringup_all.launch.py

This is the SINGLE-MACHINE convenience launcher. It runs both robots' Nav2
stacks locally, which is fine for development but is not the two-NUC layout --
there, nuc1 runs this with `nav:=robot1` and nuc2 runs `nav_robot.launch.py`
on its own.

Ordering matters and is why this file exists at all:

  1. Gazebo + both robots      t=0
  2. slam_toolbox              t=slam_delay   needs /clock and <ns>/scan
  3. Nav2 for the SLAM robot   t=nav_delay    needs map -> <ns>/odom from SLAM
  4. Nav2 for the other robot  +nav_stagger   staggered: two Nav2 bringups at
                                              once starve each other's lifecycle
                                              activation and time out
  5. RViz                      t=rviz_delay

Launching all of it at t=0 leaves the costmaps stuck on "Checking transform"
and the lifecycle manager aborting on get_state timeouts.

Arguments:
    slam:=true|false        run slam_toolbox at all
    slam_robot:=robot1      which robot maps; the other gets the static map->odom
    nav:=both|robot1|robot2|none
    rviz:=true|false
    preflight:=true|false   refuse to start if the domain is already busy
    slam_delay / nav_delay / nav_stagger / rviz_delay   seconds, for slow machines
"""
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from robot_spawn import rviz_config_for  # noqa: E402

ROBOTS = ('robot1', 'robot2')

# Runs in its own interpreter so rclpy's context never touches the launch
# process. Reports what is ALREADY on the domain before we add to it.
_PREFLIGHT = r'''
import rclpy, time
rclpy.init()
n = rclpy.create_node('saye_preflight')
# Node discovery is NOT reliable from a single sample -- a fresh participant
# routinely sees an incomplete graph for the first few seconds. A false "clean"
# here is the dangerous direction (it lets a SECOND stack onto the domain), so
# sample repeatedly and take the union.
clocks, names = 0, set()
for _ in range(4):
    time.sleep(1.0)
    clocks = max(clocks, len(n.get_publishers_info_by_topic('/clock')))
    names |= {x[0] for x in n.get_node_names_and_namespaces()}
print(f'{clocks} {len([x for x in names if x == "slam_toolbox"])} '
      f'{len([x for x in names if x == "parameter_bridge"])}')
n.destroy_node(); rclpy.shutdown()
'''


def _preflight():
    """Return (clock_publishers, slam_nodes, bridges) already on the domain."""
    try:
        out = subprocess.run([sys.executable, '-c', _PREFLIGHT],
                             capture_output=True, text=True, timeout=30)
        return tuple(int(v) for v in out.stdout.strip().split()[:3])
    except Exception:
        return (0, 0, 0)   # never block a launch because the check itself broke


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def secs(name):
        return float(arg(name))

    slam_robot = arg('slam_robot')
    if slam_robot not in ROBOTS:
        raise RuntimeError(f'slam_robot must be one of {ROBOTS}, got {slam_robot!r}')

    nav = arg('nav')
    if nav not in ('both', 'robot1', 'robot2', 'none'):
        raise RuntimeError(f"nav must be both|robot1|robot2|none, got {nav!r}")
    nav_for = list(ROBOTS) if nav == 'both' else ([] if nav == 'none' else [nav])
    # SLAM robot first: it publishes map -> odom, which the other robot's
    # costmaps want to be able to resolve as early as possible.
    nav_for.sort(key=lambda r: r != slam_robot)

    use_slam = arg('slam').lower() in ('true', '1', 'yes')

    if arg('preflight').lower() in ('true', '1', 'yes'):
        clocks, slams, bridges = _preflight()
        if clocks or slams or bridges:
            raise RuntimeError(
                '\n'
                '  DOMAIN ALREADY BUSY -- refusing to launch a second stack.\n'
                f'    /clock publishers : {clocks}   (expected 0)\n'
                f'    slam_toolbox nodes: {slams}   (expected 0)\n'
                f'    parameter_bridge  : {bridges}   (expected 0)\n'
                '\n'
                '  Two Gazebos means two /clock streams. Subscribers interleave\n'
                '  them, time appears to run backwards, and tf2 responds by\n'
                '  clearing its whole buffer several times a second -- SLAM drops\n'
                '  every scan, the controller loses its transforms, RViz blinks.\n'
                '  Two slam_toolbox nodes fight over map -> <ns>/odom the same way.\n'
                '\n'
                '  Clear it:  docker restart nuc1\n'
                '  Override:  preflight:=false\n')

    pkg = get_package_share_directory('saye_bringup')
    inc = lambda f, a: IncludeLaunchDescription(                    # noqa: E731
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', f)),
        launch_arguments=a.items())

    actions = [
        LogInfo(msg=f'[bringup_all] gazebo + {ROBOTS[0]} + {ROBOTS[1]}'
                    f' | slam={slam_robot if use_slam else "off"} | nav={nav}'),
        # RViz is launched here at the end, not by multi_vehicle, so it comes up
        # after Nav2 rather than against an empty TF tree.
        inc('multi_vehicle.launch.py', {'rviz': 'false'}),
    ]

    if use_slam:
        actions.append(TimerAction(period=secs('slam_delay'), actions=[
            LogInfo(msg=f'[bringup_all] slam_toolbox ({slam_robot})'),
            inc('slam.launch.py', {'use_sim_time': 'true'}),
        ]))

    for i, ns in enumerate(nav_for):
        # Only the robot WITHOUT slam gets the static map->odom; two publishers
        # on that edge is the original bug this whole stack was fixed for.
        static = 'false' if (use_slam and ns == slam_robot) else 'true'
        actions.append(TimerAction(
            period=secs('nav_delay') + i * secs('nav_stagger'),
            actions=[
                LogInfo(msg=f'[bringup_all] nav2 {ns} (static_map_odom={static})'),
                inc('nav_robot.launch.py',
                    {'namespace': ns, 'static_map_odom': static}),
            ]))

    if arg('rviz').lower() in ('true', '1', 'yes'):
        actions.append(TimerAction(period=secs('rviz_delay'), actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', rviz_config_for(slam_robot)],
                 parameters=[{'use_sim_time': True}],
                 output='log'),
        ]))

    return actions


def generate_launch_description():
    headless = bool(os.environ.get('GZ_HEADLESS'))
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Run slam_toolbox.'),
        DeclareLaunchArgument('slam_robot', default_value='robot1',
                              description='Which robot maps. The other gets the static map-><ns>/odom.'),
        DeclareLaunchArgument('nav', default_value='both',
                              description='Which robots get a Nav2 stack: both|robot1|robot2|none.'),
        DeclareLaunchArgument('rviz', default_value=('false' if headless else 'true'),
                              description='Open RViz once the stack is up.'),
        DeclareLaunchArgument('preflight', default_value='true',
                              description='Refuse to launch if the domain already has a stack on it.'),

        # Generous by default: a cold Intel iGPU spends 30-60 s compiling shaders
        # on the first Gazebo run. Shrink them on a warm machine.
        DeclareLaunchArgument('slam_delay', default_value='15.0'),
        DeclareLaunchArgument('nav_delay', default_value='25.0'),
        DeclareLaunchArgument('nav_stagger', default_value='12.0'),
        DeclareLaunchArgument('rviz_delay', default_value='20.0'),

        OpaqueFunction(function=_setup),
    ])
