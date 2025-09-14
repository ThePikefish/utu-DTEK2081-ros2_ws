from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_terminals = LaunchConfiguration('use_terminals')


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_terminals',
            default_value='0',
            description='Launch each node in a separate terminal (1=yes, 0=no)'
        ),


        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'pose_package', 'publisher'],
            name='publisher_terminal',
            output='screen'
        ),


        Node(
            condition=UnlessCondition(use_terminals),
            package='pose_package',
            executable='publisher',
            name='simple_publisher',
            output='screen'
        ),


        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'pose_package', 'subscriber'],
            name='subscriber_terminal',
            output='screen'
        ),


        Node(
            condition=UnlessCondition(use_terminals),
            package='pose_package',
            executable='subscriber',
            name='simple_subscriber',
            output='screen'
        )
    ])
