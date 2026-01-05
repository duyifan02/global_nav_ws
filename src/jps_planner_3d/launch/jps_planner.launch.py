from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('jps_planner_3d')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'jps_planner.yaml']),
        description='Path to YAML file with jps_planner_node parameters'
    )

    node = Node(
        package='jps_planner_3d',
        executable='jps_planner_node',
        name='jps_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        node,
    ])
