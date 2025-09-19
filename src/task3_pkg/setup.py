from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task3_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
    glob(os.path.join('launch', '*launch.[pxy][y|a]ml'))),
        (os.path.join('share', package_name, 'img_data'),
    glob(os.path.join('img_data', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hauki',
    maintainer_email='kristo.jonsson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_static_broadcaster= task3_pkg.tf_static_broadcaster:main',
            'tf_broadcast_scanner= task3_pkg.tf_broadcast_scanner:main',
            'tf_listener_transform= task3_pkg.tf_listener_transform:main',
            'tf_listener_revolutions= task3_pkg.tf_listener_revolutions:main',
        ],
    },
)
