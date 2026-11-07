from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('x500_control'), 'config', 'scan.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(
            package='x500_control',
            executable='offboard_scan',
            name='offboard_scan',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='x500_control',
            executable='rover_investigator',
            name='rover_investigator',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
