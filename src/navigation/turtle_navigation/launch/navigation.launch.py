"""Launch the public navigation action server, controller, and optional Turtlesim."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_turtlesim = LaunchConfiguration("start_turtlesim")
    demo_goal = LaunchConfiguration("demo_goal")
    navigation_config = str(
        Path(get_package_share_directory("turtle_navigation")) / "config" / "navigation.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_turtlesim",
                default_value="true",
                description="Start the Turtlesim window with the navigation nodes.",
            ),
            DeclareLaunchArgument(
                "demo_goal",
                default_value="false",
                description="Send one example public goal after startup.",
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
                parameters=[navigation_config],
            ),
            Node(
                package="turtle_navigation",
                executable="navigation_controller",
                output="screen",
                parameters=[navigation_config],
            ),
            # The normal launch remains interactive. The demo option sends a
            # public action goal so a fresh launch visibly moves the turtle.
            TimerAction(
                period=3.0,
                condition=IfCondition(demo_goal),
                actions=[ExecuteProcess(
                    cmd=["ros2", "action", "send_goal", "/navigate_turtle",
                         "turtle_nav_interfaces/action/NavigateTurtle",
                         "{x: 8.0, y: 5.5}", "--feedback"],
                    output="screen",
                )],
            ),
        ]
    )
