from setuptools import find_packages, setup

package_name = 'topo_nav_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/topo_nav']),
        ('share/topo_nav', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Python tools for topo_nav: room visualiser and plan client.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'room_visualizer = topo_nav_tools.room_visualizer:main',
            'hierarchical_plan_client = topo_nav_tools.hierarchical_plan_client:main',
        ],
    },
)
