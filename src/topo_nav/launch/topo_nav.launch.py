from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('topo_nav')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'topo_nav.yaml']),
        description='Path to topo_nav_node parameter YAML file'
    )

    topo_nav_node = Node(
        package='topo_nav',
        executable='topo_nav_node',
        name='topo_nav_node',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        topo_nav_node,
    ])
