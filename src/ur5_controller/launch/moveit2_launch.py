from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths to the URDF and SRDF files
    urdf_file_path = "/home/om/ros2_ws/src/ur_description/urdf/ur5.urdf"
    srdf_file_path = "/home/om/ros2_ws/src/ur_description/srdf/ur5.srdf"

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_file_path,
            description='Full path to the robot URDF file'
        ),
        DeclareLaunchArgument(
            'srdf_file',
            default_value=srdf_file_path,
            description='Full path to the robot SRDF file'
        ),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('urdf_file')])}]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'robot_description': Command(['cat ', LaunchConfiguration('urdf_file')])},
                {'robot_description_semantic': Command(['cat ', LaunchConfiguration('srdf_file')])}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='ur5_controller',
            executable='moveit_cartesian_planner',
            name='moveit_cartesian_planner',
            output='screen',
            parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('urdf_file')])}]
        ),
    ])
