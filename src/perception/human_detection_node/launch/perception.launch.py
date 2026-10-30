"""Start all perception nodes; play the ROS bag in a separate shell."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Wire calibration, TF, person detection, and LiDAR location together."""
    camera = Path(get_package_share_directory('front_camera_info'))
    calibration = str(camera / 'config' / 'camera_calibration.yaml')
    return LaunchDescription([
        # The default checkpoint is bundled in the image at build time.
        DeclareLaunchArgument('model_path', default_value='/opt/models/yolo26n.pt'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (camera / 'urdf' / 'front_sensors.urdf').read_text()}]),
        Node(package='front_camera_info', executable='camera_info_node',
             parameters=[{'calibration_file': calibration}]),
        Node(package='human_detection_node', executable='human_detector',
             parameters=[{'model_path': LaunchConfiguration('model_path'),
                          'confidence': LaunchConfiguration('confidence')}]),
        Node(package='human_detection_node', executable='human_locator',
             parameters=[{'calibration_file': calibration}]),
    ])
