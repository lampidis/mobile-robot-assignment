#!/usr/bin/env python3
"""Exercise one perception node's ROS topic contract per invocation."""

import argparse
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray
from geometry_msgs.msg import PointStamped


def wait_for(executor, condition, seconds=8):
    """Spin actual ROS callbacks until one expected event or a short deadline."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False


def image(stamp, width=1280, height=720, encoding='rgb8'):
    """Make a small synthetic image; only the detector needs pixel bytes."""
    msg = Image()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'front_wide_angle_camera_frame'
    msg.width, msg.height, msg.encoding = width, height, encoding
    msg.step = width * 3
    msg.data = bytes(msg.step * height)
    return msg


def camera_case(executor, client):
    """Check matching CameraInfo headers and rejection of wrong dimensions."""
    from front_camera_info.camera_info_node import CameraInfoNode

    node = CameraInfoNode()
    executor.add_node(node)
    seen = []
    client.create_subscription(CameraInfo, '/m20/camera/front/camera_info',
                               seen.append, qos_profile_sensor_data)
    publisher = client.create_publisher(Image, '/m20/camera/front/image_raw', qos_profile_sensor_data)
    if not wait_for(executor, lambda: publisher.get_subscription_count() > 0 and
                    node.publisher.get_subscription_count() > 0):
        raise AssertionError('CameraInfo topic endpoints did not connect')
    publisher.publish(image(101))
    if not wait_for(executor, lambda: len(seen) == 1):
        raise AssertionError('no CameraInfo for a valid image')
    info = seen[0]
    assert (info.header.stamp.sec, info.header.frame_id) == (101, 'front_wide_angle_camera_frame')
    assert (info.width, info.height, info.distortion_model, len(info.d)) == (1280, 720, 'equidistant', 4)
    # An image with the wrong size must not reuse this calibration.
    publisher.publish(image(102, width=640, height=360))
    assert not wait_for(executor, lambda: len(seen) > 1, seconds=0.6)
    print('PASS CameraInfo preserves the image header and rejects wrong dimensions')


def detector_case(executor, client):
    """Check both detector outputs, including an empty-person frame and bad input."""
    from human_detection_node.detector import Detector

    node = Detector()
    executor.add_node(node)
    boxes, annotated = [], []
    client.create_subscription(Detection2DArray, '/m20/camera/front/human/detections',
                               boxes.append, qos_profile_sensor_data)
    client.create_subscription(Image, '/m20/camera/front/human/image_annotated',
                               annotated.append, qos_profile_sensor_data)
    publisher = client.create_publisher(Image, '/m20/camera/front/image_raw', qos_profile_sensor_data)
    if not wait_for(executor, lambda: publisher.get_subscription_count() > 0 and
                    node.detection_pub.get_subscription_count() > 0 and
                    node.annotated_pub.get_subscription_count() > 0):
        raise AssertionError('detector topic endpoints did not connect')
    # A blank frame has no people, but still gets two matching output messages.
    publisher.publish(image(201, width=128, height=128))
    if not wait_for(executor, lambda: boxes and annotated, seconds=15):
        raise AssertionError('detector did not publish both outputs')
    assert not boxes[0].detections
    assert boxes[0].header.stamp.sec == annotated[0].header.stamp.sec == 201
    assert (annotated[0].width, annotated[0].height, annotated[0].encoding) == (128, 128, 'rgb8')
    # Invalid encoding must not produce a misleading detection for this stamp.
    publisher.publish(image(202, width=128, height=128, encoding='mono8'))
    assert not wait_for(executor, lambda: len(boxes) > 1 or len(annotated) > 1, seconds=0.6)
    print('PASS detector publishes both outputs for an empty frame and skips bad encoding')


def locator_case(executor, client):
    """Check that an unmatched detection yields an empty highlight, no 3D point."""
    from human_detection_node.locator import Locator

    node = Locator()
    executor.add_node(node)
    highlights, locations = [], []
    client.create_subscription(PointCloud2, '/human/selected_points',
                               highlights.append, qos_profile_sensor_data)
    client.create_subscription(PointStamped, '/human/location',
                               locations.append, qos_profile_sensor_data)
    publisher = client.create_publisher(Detection2DArray,
                                        '/m20/camera/front/human/detections', qos_profile_sensor_data)
    if not wait_for(executor, lambda: publisher.get_subscription_count() > 0 and
                    node.selected_pub.get_subscription_count() > 0):
        raise AssertionError('locator topic endpoints did not connect')
    detections = Detection2DArray()
    detections.header.stamp.sec = 301
    detections.header.frame_id = 'front_wide_angle_camera_frame'
    detection = Detection2D()
    detection.bbox = BoundingBox2D()
    detection.bbox.size_x = detection.bbox.size_y = 100.0
    detections.detections.append(detection)
    publisher.publish(detections)
    if not wait_for(executor, lambda: highlights, seconds=6):
        raise AssertionError('locator did not publish an empty selection')
    assert highlights[0].width == 0 and highlights[0].header.stamp.sec == 301
    assert highlights[0].header.frame_id == 'base_link_custom' and not locations
    # A frame without detections also clears the previous RViz selection.
    detections.header.stamp.sec = 302
    detections.detections.clear()
    publisher.publish(detections)
    if not wait_for(executor, lambda: len(highlights) == 2, seconds=6):
        raise AssertionError('empty detection frame did not clear selection')
    assert highlights[1].width == 0 and highlights[1].header.stamp.sec == 302
    assert not locations
    print('PASS locator clears selected points for missing cloud and empty detections')


def main():
    """Run one case in the isolated ROS domain chosen by the suite runner."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=('camera', 'detector', 'locator'))
    args = parser.parse_args()
    rclpy.init()
    client = Node('perception_topic_test')
    executor = SingleThreadedExecutor()
    executor.add_node(client)
    try:
        {'camera': camera_case, 'detector': detector_case, 'locator': locator_case}[args.case](executor, client)
    finally:
        # The executor owns callbacks; destroy test nodes before shutdown.
        for node in list(executor.get_nodes()):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
