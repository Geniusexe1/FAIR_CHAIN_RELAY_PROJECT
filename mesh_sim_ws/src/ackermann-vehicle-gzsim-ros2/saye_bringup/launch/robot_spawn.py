"""Shared per-robot spawn logic for single- and multi-robot launches.

Exists because this repo has no xacro/urdf and no ros2_control, so the usual
namespacing mechanisms do not apply. See DOCKER_SETUP.md / NAMESPACING.md.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def _substitute(src_path, ns, suffix):
    """Read a template, replace ${NS}, write to a temp file, return its path."""
    with open(src_path, 'r') as f:
        text = f.read().replace('${NS}', ns)
    fd, path = tempfile.mkstemp(prefix=f'{ns}_', suffix=suffix)
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return path


def robot_description_for(ns):
    """SDF text with ${NS} resolved.

    A1: plain SDF has no <arg> mechanism, so we substitute in Python. The
    original launch file already read this file into a string, so this adds
    no new tooling.
    """
    pkg_desc = get_package_share_directory('saye_description')
    sdf = os.path.join(pkg_desc, 'models', 'saye', 'model.sdf')
    with open(sdf, 'r') as f:
        return f.read().replace('${NS}', ns)


def rviz_config_for(ns):
    """Per-robot RViz config (A7: saye.rviz hardcodes odom / saye frames).

    Fixed Frame is `map`, NOT <ns>/odom: map is the root of the tree, so it is
    the only frame from which BOTH robots are reachable. Pinning it to one
    robot's odom leaves the other robot untransformable and invisible.
    """
    pkg_bringup = get_package_share_directory('saye_bringup')
    src = os.path.join(pkg_bringup, 'rviz', 'saye.rviz')
    with open(src, 'r') as f:
        text = (f.read()
                .replace('Fixed Frame: odom', 'Fixed Frame: map')
                .replace('Reference Frame: saye', f'Reference Frame: {ns}/base_link')
                .replace('Target Frame: saye', f'Target Frame: {ns}/base_link'))
    fd, path = tempfile.mkstemp(prefix=f'{ns}_', suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return path


def spawn_robot_group(ns, x=0.0, y=0.0, z=0.35):
    """Everything belonging to ONE robot, inside its own namespace.

    Deliberately does NOT bridge /clock -- there is a single Gazebo, so /clock
    is bridged once globally by the caller (A4). Namespacing it stalls every
    use_sim_time node with no error message.
    """
    pkg_bringup = get_package_share_directory('saye_bringup')
    bridge_cfg = _substitute(
        os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml.in'), ns, '.yaml')

    return GroupAction([
        PushRosNamespace(ns),

        # A5 source #1: link/joint TF. frame_prefix handles base_link + all
        # 6 wheel links. It does NOT touch the Gazebo-published frames -- those
        # are prefixed in model.sdf instead.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description_for(ns),
                'frame_prefix': ns + '/',
            }],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            parameters=[{
                'config_file': bridge_cfg,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # A6: '/robot_description' was absolute upstream, which breaks under a
        # namespace. Relative name resolves to /<ns>/robot_description.
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', ns,
                '-allow_renaming', 'true',
                '-x', str(x), '-y', str(y), '-z', str(z),
            ],
        ),
    ])


def clock_bridge_node():
    """The single global /clock bridge (A4)."""
    pkg_bringup = get_package_share_directory('saye_bringup')
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'clock_bridge.yaml'),
        }],
        output='screen',
    )
