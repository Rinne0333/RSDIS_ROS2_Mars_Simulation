from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('x500_control'), 'config', 'scan.yaml'])
    dem = PathJoinSubstitution([
        FindPackageShare('x500_control'), 'config', 'terrain_dem.csv'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument(
            'ignore_blobs', default_value='false',
            description='true: keep scanning, never publish /x500/poi (only /x500/blob_world and logs)'),
        Node(
            package='x500_control',
            executable='offboard_scan',
            name='offboard_scan',
            output='screen',
            parameters=[LaunchConfiguration('config'), {'dem_file': dem}],
        ),
        Node(
            package='x500_control',
            executable='lander_detector',
            name='lander_detector',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='x500_control',
            executable='poi_locator',
            name='poi_locator',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'dem_file': dem,
                'ignore_blobs': ParameterValue(LaunchConfiguration('ignore_blobs'), value_type=bool),
            }],
        ),
        Node(
            package='x500_control',
            executable='rover_investigator',
            name='rover_investigator',
            output='screen',
            parameters=[LaunchConfiguration('config'), {'dem_file': dem}],
        ),
    ])
