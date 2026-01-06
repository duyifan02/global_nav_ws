from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    npy_path = LaunchConfiguration('npy_path')
    resolution = LaunchConfiguration('resolution')
    free_space_margin = LaunchConfiguration('free_space_margin')
    origin = LaunchConfiguration('origin')

    return LaunchDescription([
        DeclareLaunchArgument('npy_path', default_value='', description='Path to occupancy .npy (N,3) indices or world points'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='Voxel resolution (meters)'),
        DeclareLaunchArgument('free_space_margin', default_value='2.0', description='Free space margin (meters)'),
        DeclareLaunchArgument('origin', default_value='[0.0, 0.0, 0.0]', description='Origin for index->world conversion'),

        Node(
            package='jps_planner_3d',
            executable='jps_planner_node',
            name='jps_planner_node',
            output='screen',
        ),

        Node(
            package='occmap_tools',
            executable='npy_map_loader',
            name='npy_map_loader',
            output='screen',
            parameters=[{
                'npy_path': npy_path,
                'resolution': resolution,
                'free_space_margin': free_space_margin,
                'origin': origin,
                'points_are_indices': False,
                'call_set_map': True,
                'publish_cloud': True,
            }],
        ),
    ])
