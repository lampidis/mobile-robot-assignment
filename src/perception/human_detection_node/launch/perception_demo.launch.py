"""Start the perception pipeline, RViz, and looping rosbag playback together."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Add the recorded inputs and visualization to the normal pipeline launch."""
    share = Path(get_package_share_directory('human_detection_node'))
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='/workspace/src/perception/data/rosbag/person_walking'),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open the saved RViz display.'),
        # Reuse the same nodes as the headless tests and manual pipeline run.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch' / 'perception.launch.py'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'config.rviz')], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
        # Give detector and RViz subscriptions time to appear before the bag
        # starts. Looping keeps the display active after the recording ends.
        TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--rate',
                 LaunchConfiguration('rate'), '--loop', '--disable-keyboard-controls'],
            output='screen',
        )]),
    ])
