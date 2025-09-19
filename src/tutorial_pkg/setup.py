from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tutorial_pkg'

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
            'tf_broadcaster= tutorial_pkg.tf_broadcaster:main',
            'tf_listener= tutorial_pkg.tf_listener:main',
        ],
    },
)
