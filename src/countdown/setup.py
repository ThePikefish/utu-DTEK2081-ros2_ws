from setuptools import find_packages, setup

package_name = 'countdown'

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
    maintainer='xiaoxia',
    maintainer_email='xiaoxia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'countdown_server = countdown.countdown_server:main',
            'countdown_client = countdown.countdown_client:main',
            'countdown_server_cancel = countdown.countdown_server_cancel:main',
            'countdown_client_cancel = countdown.countdown_client_cancel:main',
        ],
    },
)
