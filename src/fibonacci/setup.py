from setuptools import find_packages, setup

package_name = 'fibonacci'

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
    maintainer_email='hauki@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_server = fibonacci.fibonacci_server:main',
            'fibonacci_client = fibonacci.fibonacci_client:main',
            'fibonacci_server_cancel = fibonacci.fibonacci_server_cancel:main',
            'fibonacci_client_cancel = fibonacci.fibonacci_client_cancel:main',
        ],
    },
)
