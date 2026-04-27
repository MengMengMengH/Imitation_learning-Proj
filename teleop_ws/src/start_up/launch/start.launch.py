from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():  
    ld = LaunchDescription()

    # define the node to be launched
    config_path = os.path.join(
            get_package_share_directory('mpc_interpolation'),
            'config',
            'mpc_weight.yaml'
        )

    interpolation_node = Node(
        package='mpc_interpolation',
        executable='mpc_interpolation_node',
        output='screen',
        name='interpolation_node',
        parameters=[config_path]
    )

    IK_node = Node(
        package='arm_tele_mujo',
        executable='arm_tele_real',
        output='screen',
        name="arm_tele_real"
    )

    force_node = Node(
        package='force_srv',
        executable='force_srv',
        output='screen',
        name='six_dim_force'
    )

    camera_node = Node(
        package='cam_srv',
        executable='wrist_cam_capture',
        output='screen',
        name='camera'
    )

    ld.add_action(IK_node)
    ld.add_action(interpolation_node)
    ld.add_action(force_node)
    ld.add_action(camera_node)

    return ld

