import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spaceros_models = os.path.join(
        get_package_share_directory('spaceros_gz_demos'), 'models')
    px4_home = os.path.expanduser(os.environ.get('PX4_HOME', '~/PX4-Autopilot'))
    px4_models = os.path.join(px4_home, 'Tools/simulation/gz/models')
    px4_worlds = os.path.join(px4_home, 'Tools/simulation/gz/worlds')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = ':'.join(
        p for p in (px4_models, px4_worlds, spaceros_models, existing) if p)

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument(
            'world_file',
            description='Path to the world file to load into Gazebo'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world_file')],
                'on_exit_shutdown': 'True',
            }.items(),
        ),
    ])