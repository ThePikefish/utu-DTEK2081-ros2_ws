from setuptools import find_packages, setup

package_name = 'lab5_feature_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'lidar_plotter = lab5_feature_detector.Task1_lidar_plot:main',
            'feature_extracter = lab5_feature_detector.Task1_feature_detector:main',
        ],
    },
)
