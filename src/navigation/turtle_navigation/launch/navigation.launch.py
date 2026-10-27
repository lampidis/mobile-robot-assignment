"""Launch the controller and the public action server skeleton."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_turtlesim = LaunchConfiguration("start_turtlesim")
    controller_config = str(
        Path(get_package_share_directory("turtle_navigation")) / "config" / "navigation.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_turtlesim",
                default_value="true",
                description="Start the Turtlesim window with the navigation nodes.",
            ),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                output="screen",
                condition=IfCondition(start_turtlesim),
            ),
            Node(
                package="turtle_navigation",
                executable="navigation_action_server",
                output="screen",
            ),
            Node(
                package="turtle_navigation",
                executable="navigation_controller",
                output="screen",
                parameters=[controller_config],
            ),
        ]
    )
