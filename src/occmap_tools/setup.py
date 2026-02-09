from setuptools import find_packages, setup

package_name = 'occmap_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/test_jps_with_npy.launch.py',
            'launch/load_obj_mesh.launch.py',
            'launch/visualize_distance_field.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Load occupancy map from .npy, call set_voxel_map, and visualize for JPS planner.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'npy_map_loader = occmap_tools.npy_map_loader:main',
            'obj_mesh_loader = occmap_tools.obj_mesh_loader:main',
            'plan_path_client = occmap_tools.plan_path_client:main',
            'distance_field_visualizer = occmap_tools.distance_field_visualizer:main',
        ],
    },
)
