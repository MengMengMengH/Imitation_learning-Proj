from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  
    ld = LaunchDescription()

    # define the node to be launched

    interpolation_node = Node(
        package='mpc_interpolation',
        executable='mpc_interpolation_node',
        output='screen',
        name='mpc_interpolation_node',
    )

    ld.add_action(interpolation_node)

    return ld

