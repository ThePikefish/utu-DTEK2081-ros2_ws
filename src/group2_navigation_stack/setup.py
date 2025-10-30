from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'group2_navigation_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*.py'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds','*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaoxia',
    maintainer_email='xiaoxia@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grid_mapper = group2_navigation_stack.grid_mapper:main',
            'waypoint_follower = group2_navigation_stack.waypoint_follower:main',
            'dijkstra_path = group2_navigation_stack.dijkstra_path:main',
            'a_star_path = group2_navigation_stack.a_star_path:main',
        ],
    },
)
