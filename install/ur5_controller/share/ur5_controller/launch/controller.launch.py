from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Define the path to the controller parameters file
    
    ur5_controller_path = os.path.join(
        get_package_share_directory('ur5_controller'), 'config', 'ur5_controller.yaml'
    )

    # Define the controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ur5_controller_path],
        output='screen'
    )

    # Define the joint state broadcaster spawner node with a delay
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Define the position controller spawner node with a delay
    position_controller_spawner= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ])
