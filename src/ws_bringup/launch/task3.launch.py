import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('service_example') # TODO: your package name for launchs
    
    # Path to the parameter file
    param_file = LaunchConfiguration('params_file') 
    use_terminals = LaunchConfiguration('use_terminals')
    
    # Declare the launch argument for parameter file
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/ws/src/service_example/config/params.yaml',
        # default_value=os.path.join(bringup_dir, 'config', 'params.yaml'), # TODO: your yaml file name goes here
        description='Full path to the ROS2 parameters file to use'
    )

    declare_use_terminals_cmd = DeclareLaunchArgument(
        'use_terminals',
        default_value='0',
        description='Launch each node in a separate terminal (1=yes, 0=no)'
    )
    

    # Service server node
    service_server_node = Node(
        condition=UnlessCondition(use_terminals),
        package='service_example',
        executable='calculate_distance_server',
        name='calculate_distance_server',
        output='screen'
    )

    # Service client node
    service_client_node = Node(
        condition=UnlessCondition(use_terminals),
        package='service_example',
        executable='calculate_distance_client',
        name='calculate_distance_client_node',
        output='screen',
        parameters=[param_file]
    )


    # Service server node (terminal)
    service_server_node_terminal = ExecuteProcess(
        condition=IfCondition(use_terminals),
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'service_example', 'calculate_distance_server'],
        name='calculate_distance_server',
        output='screen'
    )

    # Service client node (terminal)
    service_client_node_terminal = ExecuteProcess(
        condition=IfCondition(use_terminals),
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'service_example', 'calculate_distance_client', '--ros-args', '--params-file', param_file],
        name='calculate_distance_client_node',
        output='screen',
        #parameters=[param_file]
    )
    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_terminals_cmd)
    ld.add_action(service_server_node)
    ld.add_action(service_client_node)
    ld.add_action(service_server_node_terminal)
    ld.add_action(service_client_node_terminal)
    
    return ld

