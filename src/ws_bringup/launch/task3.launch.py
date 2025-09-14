import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('service_example') # TODO: your package name for launchs
    
    # Path to the parameter file
    param_file = LaunchConfiguration('params_file') 
    
    # Declare the launch argument for parameter file
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/ws/src/service_example/config/params.yaml',
        # default_value=os.path.join(bringup_dir, 'config', 'params.yaml'), # TODO: your yaml file name goes here
        description='Full path to the ROS2 parameters file to use'
    )
    
    # Service server node
    service_server_node = Node(
        package='service_example',
        executable='calculate_distance_server',
        name='calculate_distance_server',
        output='screen'
    )
    
    
    # Service client node
    service_client_node = Node(
        package='service_example',
        executable='calculate_distance_client',
        name='calculate_distance_client_node',
        output='screen',
        parameters=[param_file]
    )
    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_params_file_cmd)
    ld.add_action(service_server_node)
    ld.add_action(service_client_node)
    
    return ld

