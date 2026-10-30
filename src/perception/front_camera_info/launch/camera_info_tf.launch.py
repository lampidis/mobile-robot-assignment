"""Launch calibrated CameraInfo and the supplied fixed sensor transforms."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Locate installed assets and start the two camera support nodes."""
    share = Path(get_package_share_directory('front_camera_info'))
    return LaunchDescription([
        # robot_state_publisher reads the supplied URDF and publishes its
        # fixed joints, including lidar_link -> camera optical frame.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share / 'urdf' / 'front_sensors.urdf').read_text()}]),
        Node(package='front_camera_info', executable='camera_info_node'),
    ])
