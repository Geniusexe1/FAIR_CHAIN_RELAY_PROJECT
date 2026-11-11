import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Namespace lets each NUC publish under its own prefix, e.g. /nuc1/mesh_matrix.
    # Without it both NUCs publish to the SAME /mesh_matrix with their own (possibly
    # different) set of reachable radios, so a subscriber sees interleaved matrices
    # of differing dimensions with no way to tell which host observed what.
    # Defaults to the MESH_NS env var, or '' to preserve the original behaviour.
    default_ns = os.environ.get('MESH_NS', '')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=default_ns,
            description="Topic/node namespace for this host (e.g. nuc1). Empty = global."),

        Node(
            package='rajant_mesh',
            executable='weakest_link',
            name='weakest_link',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='rajant_mesh',
            executable='mesh_matrix',
            name='mesh_matrix',
            namespace=namespace,
            output='screen'
        ),
    ])
