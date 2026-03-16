"""
Combined launch: topo_nav_node + room_visualizer

Usage:
    ros2 launch topo_nav topo_nav_with_viz.launch.py

Optional args:
    config:=<yaml_path>       override parameter YAML (default: config/topo_nav.yaml)
    publish_rate_hz:=2.0      room_visualizer update rate

Services (with namespace topo_nav):
    /topo_nav/set_voxel_map
    /topo_nav/load_pcd_map
    /topo_nav/build_topo_map
    /topo_nav/get_room_labels
    /topo_nav/plan_hierarchical
    /topo_nav/get_distance_field

RViz topics published by room_visualizer:
    /topo_nav/room_cloud     (room-coloured PointCloud2)
    /topo_nav/floor_cloud    (floor-coloured PointCloud2)
    /topo_nav/persist_cloud  (clearance heatmap PointCloud2)
"""

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

    rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='2.0',
        description='room_visualizer publish rate (Hz)'
    )

    topo_nav_node = Node(
        package='topo_nav',
        executable='topo_nav_node',
        name='topo_nav_node',
        namespace='topo_nav',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    room_visualizer = Node(
        package='topo_nav',
        executable='room_visualizer',
        name='room_visualizer',
        output='screen',
        parameters=[{'publish_rate_hz': LaunchConfiguration('publish_rate_hz')}]
    )

    return LaunchDescription([
        config_arg,
        rate_arg,
        topo_nav_node,
        room_visualizer,
    ])
