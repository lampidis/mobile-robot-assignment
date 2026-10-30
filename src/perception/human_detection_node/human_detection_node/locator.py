"""Associate person boxes with nearby LiDAR points using calibrated fisheye projection."""

from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


def stamp_ns(stamp):
    """Convert a ROS timestamp to one integer for safe nearest-time matching."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def transform_matrix(transform):
    """Turn a TF quaternion and translation into NumPy rotation/offset."""
    # TF gives orientation as a quaternion; matrix multiplication lets us
    # transform a whole point cloud without a Python loop over each point.
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)
    translation = np.array([transform.translation.x, transform.translation.y,
                            transform.translation.z], dtype=np.float64)
    return rotation, translation


def cloud_xyz(msg, stride):
    """Read sampled XYZ values from the bag's packed PointCloud2 buffer."""
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ('x', 'y', 'z')):
        raise ValueError('cloud lacks x/y/z fields')
    if any(fields[name].datatype != 7 for name in ('x', 'y', 'z')):
        raise ValueError('cloud x/y/z must be float32')
    # PointCloud2 may have extra fields between XYZ points. The structured
    # dtype respects field offsets and point_step without copying all fields.
    end = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [end + 'f4'] * 3,
                      'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    packed = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    sampled = packed[::stride]
    return np.column_stack((sampled['x'], sampled['y'], sampled['z']))


def select_points(pixels, optical, base, box, floor_height, min_points=6):
    """Return the nearest supported non-floor points inside one person box."""
    # The inner box reduces returns from nearby furniture at its edges.
    x1 = box.center.position.x - 0.4 * box.size_x
    x2 = box.center.position.x + 0.4 * box.size_x
    y1 = box.center.position.y - 0.4 * box.size_y
    y2 = box.center.position.y + 0.4 * box.size_y
    inside = ((pixels[:, 0] >= x1) & (pixels[:, 0] <= x2) &
              (pixels[:, 1] >= y1) & (pixels[:, 1] <= y2) &
              (base[:, 2] > floor_height + 0.15))
    candidate = optical[inside]
    candidate_base = base[inside]
    if len(candidate) < min_points:
        return np.empty((0, 3))
    # A 0.5 m depth window groups one surface without letting a distant wall
    # dominate the person's point estimate. Skip isolated foreground returns,
    # but decline a location if only a much farther surface has support.
    depths = candidate[:, 2]
    nearest_depth = float(np.min(depths))
    for start in np.sort(depths):
        if start > nearest_depth + 1.0:
            break
        group = (depths >= start) & (depths < start + 0.5)
        if group.sum() >= min_points:
            return candidate_base[group]
    return np.empty((0, 3))


def select_location(pixels, optical, base, box, floor_height, min_points=6):
    """Estimate a person-surface location from the selected LiDAR points."""
    points = select_points(pixels, optical, base, box, floor_height, min_points)
    return np.median(points, axis=0) if len(points) else None


def selected_cloud(header, points):
    """Pack selected robot-frame XYZ values for the separate RViz display."""
    cloud = PointCloud2()
    # The cloud's time belongs to the camera detection, while its coordinates
    # have already been transformed into the robot base frame.
    cloud.header.stamp = header.stamp
    cloud.header.frame_id = 'base_link_custom'
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = np.asarray(points, dtype='<f4').reshape(-1, 3).tobytes()
    cloud.is_dense = True
    return cloud


class Locator(Node):
    """Match delayed person detections to clouds and publish 3D points."""

    def __init__(self):
        super().__init__('human_locator')
        default = str(Path(get_package_share_directory('front_camera_info')) / 'config' / 'camera_calibration.yaml')
        path = Path(self.declare_parameter('calibration_file', default).value)
        config = yaml.safe_load(path.read_text())
        self.k = np.array(config['camera_matrix']['data'], np.float64).reshape(3, 3)
        self.d = np.array(config['distortion_coefficients']['data'], np.float64).reshape(4, 1)
        self.width = int(config['image_width'])
        self.height = int(config['image_height'])
        # The 150 ms default covers most measured bag pairs while leaving the
        # unmatched tail frames unmatched. Values are parameters for review.
        self.tolerance_ns = int(float(self.declare_parameter('max_skew_s', 0.15).value) * 1e9)
        self.retention_s = float(self.declare_parameter('cloud_retention_s', 20.0).value)
        self.wait_s = float(self.declare_parameter('association_wait_s', 2.0).value)
        self.stride = int(self.declare_parameter('cloud_stride', 4).value)
        if self.stride < 1 or self.tolerance_ns <= 0 or self.retention_s <= 0 or self.wait_s < 0:
            raise ValueError('invalid association parameters')
        # Store sampled points rather than whole 4 MB ROS messages. Arrival
        # time bounds memory; header time decides which measurements match.
        self.clouds = deque()
        self.pending = deque()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output = self.create_publisher(PointStamped, '/human/location', qos_profile_sensor_data)
        self.selected_pub = self.create_publisher(PointCloud2, '/human/selected_points', qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(PointCloud2, '/LIDAR/POINTS', self.on_cloud, qos_profile_sensor_data)
        self.detection_sub = self.create_subscription(Detection2DArray, '/m20/camera/front/human/detections', self.on_detections, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.process_pending)

    def on_cloud(self, msg):
        """Save a compact cloud with both sensor and local arrival times."""
        try:
            points = cloud_xyz(msg, self.stride)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self.clouds.append((stamp_ns(msg.header.stamp), time.monotonic(), msg.header.frame_id, points))
        self.prune()

    def on_detections(self, msg):
        """Queue each frame so an empty selection can clear the RViz display."""
        self.pending.append((time.monotonic(), msg))
        self.prune()

    def prune(self):
        """Limit cloud age and both queue lengths during long bag playback."""
        now = time.monotonic()
        while self.clouds and (now - self.clouds[0][1] > self.retention_s or len(self.clouds) > 120):
            self.clouds.popleft()
        while len(self.pending) > 120:
            self.pending.popleft()

    def process_pending(self):
        """Project the nearest cloud into each pending person's image box."""
        self.prune()
        now = time.monotonic()
        while self.pending and now - self.pending[0][0] >= self.wait_s:
            _, detections = self.pending.popleft()
            target_stamp = stamp_ns(detections.header.stamp)
            if not detections.detections or not self.clouds:
                self.selected_pub.publish(selected_cloud(detections.header, []))
                continue
            # The bag's cloud headers sometimes move backward. Search by
            # header timestamp instead of assuming the newest arrival wins.
            cloud_stamp, _, frame, points = min(self.clouds, key=lambda item: abs(item[0] - target_stamp))
            if abs(cloud_stamp - target_stamp) > self.tolerance_ns:
                self.selected_pub.publish(selected_cloud(detections.header, []))
                continue
            try:
                camera_tf = self.tf_buffer.lookup_transform('front_wide_angle_camera_frame', frame, rclpy.time.Time())
                base_tf = self.tf_buffer.lookup_transform('base_link_custom', frame, rclpy.time.Time())
            except TransformException as exc:
                self.get_logger().warning(f'TF unavailable: {exc}')
                self.selected_pub.publish(selected_cloud(detections.header, []))
                continue
            # Projection needs optical-frame coordinates; the published point
            # uses the documented robot-relative base_link_custom frame.
            camera_r, camera_t = transform_matrix(camera_tf.transform)
            base_r, base_t = transform_matrix(base_tf.transform)
            valid = np.isfinite(points).all(axis=1)
            points = points[valid].astype(np.float64)
            optical = points @ camera_r.T + camera_t
            # A point behind the camera or at an implausible depth cannot
            # support a person location in the front image.
            front = (optical[:, 2] > 0.1) & (optical[:, 2] < 30)
            points, optical = points[front], optical[front]
            if len(points) == 0:
                self.selected_pub.publish(selected_cloud(detections.header, []))
                continue
            base = points @ base_r.T + base_t
            # The recorded cloud has a dense floor near z=-0.6 m. Estimate
            # that band from nearby front points instead of hardcoding a
            # robot-specific floor height into the location rule.
            nearby = optical[:, 2] < 5.0
            if nearby.sum() < 20:
                self.selected_pub.publish(selected_cloud(detections.header, []))
                continue
            floor_height = float(np.percentile(base[nearby, 2], 20))
            projected, _ = cv2.fisheye.projectPoints(optical.reshape(-1, 1, 3),
                                                     np.zeros((3, 1)), np.zeros((3, 1)), self.k, self.d,
                                                     alpha=self.k[0, 1] / self.k[0, 0])
            pixels = projected.reshape(-1, 2)
            chosen_groups = []
            for detection in detections.detections:
                chosen = select_points(pixels, optical, base, detection.bbox, floor_height)
                if len(chosen) == 0:
                    continue
                position = np.median(chosen, axis=0)
                chosen_groups.append(chosen)
                output = PointStamped()
                # Preserve the image measurement time, not wall-clock publish
                # time, so downstream consumers can trace the estimate.
                output.header.stamp = detections.header.stamp
                output.header.frame_id = 'base_link_custom'
                output.point.x, output.point.y, output.point.z = [float(v) for v in position]
                self.output.publish(output)
            # One cloud combines all supported people in this image; an empty
            # cloud removes an old highlight when no new location was found.
            selected = np.concatenate(chosen_groups) if chosen_groups else []
            self.selected_pub.publish(selected_cloud(detections.header, selected))


def main():
    """Start the ROS location node."""
    rclpy.init()
    node = Locator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is the normal end of a manual bag playback run.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
