#!/usr/bin/env python3
"""
Launch file to visualize distance field from JPS planner.

Starts both JPS planner node and distance field visualizer.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    connectivity_height_threshold_arg = DeclareLaunchArgument(
        'connectivity_height_threshold',
        default_value='0.6',
        description='BFS connectivity height threshold (meters)'
    )
    
    surface_clearance_height_arg = DeclareLaunchArgument(
        'surface_clearance_height',
        default_value='0.8',
        description='Surface clearance height requirement (meters)'
    )
    
    distance_field_max_distance_arg = DeclareLaunchArgument(
        'distance_field_max_distance',
        default_value='10',
        description='Maximum distance in distance field (grid units)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='1.0',
        description='Visualization update rate (Hz)'
    )
    
    jps_planner_node = Node(
        package='jps_planner_3d',
        executable='jps_planner_node',
        name='jps_planner_node',
        output='screen',
        parameters=[{
            'connectivity_height_threshold': LaunchConfiguration('connectivity_height_threshold'),
            'surface_clearance_height': LaunchConfiguration('surface_clearance_height'),
            'distance_field_max_distance': LaunchConfiguration('distance_field_max_distance'),
        }]
    )
    
    distance_field_visualizer_node = Node(
        package='occmap_tools',
        executable='distance_field_visualizer',
        name='distance_field_visualizer',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )
    
    return LaunchDescription([
        connectivity_height_threshold_arg,
        surface_clearance_height_arg,
        distance_field_max_distance_arg,
        update_rate_arg,
        jps_planner_node,
        distance_field_visualizer_node,
    ])
