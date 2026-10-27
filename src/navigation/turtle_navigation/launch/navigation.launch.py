"""Launch the navigation nodes; motion behavior is added in later steps."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_turtlesim = LaunchConfiguration("start_turtlesim")

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
            ),
        ]
    )
