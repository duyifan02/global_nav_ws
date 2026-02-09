from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to load Matterport3D OBJ mesh, voxelize it, and start the JPS planner.
    
    Usage:
        ros2 launch occmap_tools load_obj_mesh.launch.py obj_path:=/path/to/mesh.obj
        ros2 launch occmap_tools load_obj_mesh.launch.py obj_path:=/path/to/mesh.obj resolution:=0.1
    """
    
    # Declare launch arguments
    obj_path_arg = DeclareLaunchArgument(
        'obj_path',
        default_value='',
        description='Path to OBJ mesh file (required)'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Voxel resolution in meters'
    )
    
    auto_center_arg = DeclareLaunchArgument(
        'auto_center',
        default_value='true',
        description='Auto-center mesh at origin'
    )
    
    free_space_margin_arg = DeclareLaunchArgument(
        'free_space_margin',
        default_value='2.0',
        description='Free space margin around obstacles in meters'
    )
    
    max_service_points_arg = DeclareLaunchArgument(
        'max_service_points',
        default_value='5000000',
        description='Maximum number of points to send to service'
    )
    
    truncate_arg = DeclareLaunchArgument(
        'truncate_service_points',
        default_value='true',
        description='Truncate points if exceeding max_service_points'
    )

    # JPS Planner Node
    jps_planner_node = Node(
        package='jps_planner_3d',
        executable='jps_planner_node',
        name='jps_planner_node',
        output='screen',
    )

    # OBJ Mesh Loader Node
    obj_loader_node = Node(
        package='occmap_tools',
        executable='obj_mesh_loader',
        name='obj_mesh_loader',
        output='screen',
        parameters=[{
            'obj_path': LaunchConfiguration('obj_path'),
            'resolution': LaunchConfiguration('resolution'),
            'auto_center': LaunchConfiguration('auto_center'),
            'free_space_margin': LaunchConfiguration('free_space_margin'),
            'max_service_points': LaunchConfiguration('max_service_points'),
            'truncate_service_points': LaunchConfiguration('truncate_service_points'),
            'publish_cloud': True,
            'cloud_topic': 'occupied_voxels',
            'frame_id': 'map',
        }]
    )

    return LaunchDescription([
        obj_path_arg,
        resolution_arg,
        auto_center_arg,
        free_space_margin_arg,
        max_service_points_arg,
        truncate_arg,
        jps_planner_node,
        obj_loader_node,
    ])
