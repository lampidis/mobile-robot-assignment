"""Publish one calibrated CameraInfo with each incoming image header."""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoNode(Node):
    """Attach the saved calibration to each bag image's original header."""

    def __init__(self):
        super().__init__('front_camera_info')
        # Resolve the YAML from the installed ROS package, so launch works
        # after colcon build without depending on the shell's current folder.
        default = str(Path(get_package_share_directory('front_camera_info')) / 'config' / 'camera_calibration.yaml')
        path = Path(self.declare_parameter('calibration_file', default).value)
        data = yaml.safe_load(path.read_text())
        if data['distortion_model'] != 'equidistant':
            raise ValueError('expected equidistant fisheye calibration')
        # Calibration values are constant for this recording. Only the header
        # changes from one image to the next.
        self.template = CameraInfo()
        self.template.width = int(data['image_width'])
        self.template.height = int(data['image_height'])
        self.template.distortion_model = data['distortion_model']
        self.template.k = [float(v) for v in data['camera_matrix']['data']]
        self.template.d = [float(v) for v in data['distortion_coefficients']['data']]
        self.template.r = [float(v) for v in data['rectification_matrix']['data']]
        self.template.p = [float(v) for v in data['projection_matrix']['data']]
        if (len(self.template.k), len(self.template.d), len(self.template.r), len(self.template.p)) != (9, 4, 9, 12):
            raise ValueError('invalid calibration matrix lengths')
        self.publisher = self.create_publisher(CameraInfo, '/m20/camera/front/camera_info', qos_profile_sensor_data)
        self.subscription = self.create_subscription(Image, '/m20/camera/front/image_raw', self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'loaded calibration from {path}')

    def on_image(self, image):
        """Publish CameraInfo with the same stamp and frame as this image."""
        if (image.width, image.height) != (self.template.width, self.template.height):
            self.get_logger().error('image dimensions differ from calibration; skipping CameraInfo')
            return
        info = CameraInfo()
        # Matching headers let later nodes associate CameraInfo with an image
        # even when bag playback delivers messages at a reduced rate.
        info.header = image.header
        info.width = self.template.width
        info.height = self.template.height
        info.distortion_model = self.template.distortion_model
        info.k = self.template.k
        info.d = self.template.d
        info.r = self.template.r
        info.p = self.template.p
        self.publisher.publish(info)


def main():
    """Start the ROS node and release its resources on shutdown."""
    rclpy.init()
    node = CameraInfoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
