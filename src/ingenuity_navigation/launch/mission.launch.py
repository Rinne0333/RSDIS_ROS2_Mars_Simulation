"""Launch opcional para ler mission.yaml e definir A->B por argumentos.

Exemplo:
  ros2 launch ingenuity_navigation mission.launch.py \\
    arm:=true goal_dx:=3.0 goal_dy:=1.0 flight_height:=2.0 \
    rotate_before_navigation:=true

O Gazebo deve estar aberto separadamente. Este launch nao abre mars.launch.xml.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("ingenuity_navigation")) / "config" / "mission.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="false"),
        DeclareLaunchArgument("goal_dx", default_value="3.0"),
        DeclareLaunchArgument("goal_dy", default_value="0.0"),
        DeclareLaunchArgument("flight_height", default_value="2.0"),
        DeclareLaunchArgument("rotate_before_navigation", default_value="true"),
        Node(
            package="ingenuity_navigation",
            executable="navigation_node",
            name="ingenuity_navigation",
            output="screen",
            parameters=[
                str(config),
                {
                    "arm": ParameterValue(LaunchConfiguration("arm"), value_type=bool),
                    "goal_dx": ParameterValue(LaunchConfiguration("goal_dx"), value_type=float),
                    "goal_dy": ParameterValue(LaunchConfiguration("goal_dy"), value_type=float),
                    "flight_height": ParameterValue(
                        LaunchConfiguration("flight_height"), value_type=float),
                    "rotate_before_navigation": ParameterValue(
                        LaunchConfiguration("rotate_before_navigation"), value_type=bool),
                },
            ],
        ),
    ])
