from launch import LaunchDescription
from launch_ros.actions import Node

# exactly this name!
def generate_launch_description():

    #instantiate a LaunchDescription object
    ld = LaunchDescription()
    
    # Talker node
    first_node = Node(
        package='pose_package',
        executable='publisher',
        name='pose_publisher',
        output='screen'
    )

    # Listener node
    second_node = Node(
        package='pose_package',
        executable='subscriber',
        name='pose_subscriber',
        output='screen'
    )


    # launch description
    ld.add_action(first_node)
    ld.add_action(second_node)

    return ld