from setuptools import find_packages, setup

package_name = 'custom_pinterface'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = custom_pinterface.publisher:main',
            'subscriber = custom_pinterface.subscriber:main',
        ],
    },
)
