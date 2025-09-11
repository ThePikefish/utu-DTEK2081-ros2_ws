from setuptools import find_packages, setup

package_name = 'service_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'example_interfaces'],
    zip_safe=True,
    maintainer='hauki',
    maintainer_email='hauki@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_two_ints_client = service_example.add_two_ints_client:main',
            'add_two_ints_server = service_example.add_two_ints_server:main',
            'calculate_distance_client = service_example.calculate_distance_client:main',
            'calculate_distance_server = service_example.calculate_distance_server:main',
            'add_two_ints_client_params = service_example.add_two_ints_client_params:main',
        ],
    },
)
