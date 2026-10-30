#!/usr/bin/env python3
"""Start the perception launch and sample its required outputs during bag playback."""

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import tempfile
import time
from pathlib import Path


def main():
    """Launch a short bag segment and collect output-topic evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path('src/perception/data/rosbag/person_walking'))
    parser.add_argument('--duration', type=float, default=35.0, help='wall seconds of playback')
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--start-offset', type=float, default=4.0)
    parser.add_argument('--domain', type=int, default=88)
    parser.add_argument('--output', type=Path, default=Path('out/perception/pipeline-smoke.json'))
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        parser.error('duration and rate must be positive')
    # Keep this automated run separate from a developer's manual ROS session.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from vision_msgs.msg import Detection2DArray
    from geometry_msgs.msg import PointStamped
    rclpy.init()
    node = Node('perception_pipeline_smoke')
    types = {
        '/m20/camera/front/image_raw': Image,
        '/LIDAR/POINTS': PointCloud2,
        '/m20/camera/front/camera_info': CameraInfo,
        '/m20/camera/front/human/image_annotated': Image,
        '/m20/camera/front/human/detections': Detection2DArray,
        '/human/location': PointStamped,
        '/human/selected_points': PointCloud2,
    }
    counts = {name: 0 for name in types}
    samples = {'detections_per_frame': [], 'locations': []}
    selected_summary = {'nonempty': 0, 'invalid': 0}
    # One monotonic clock lets us compare when this subscriber receives raw
    # sensor data, detections, and the selected cloud for matching headers.
    arrivals = {'images': [], 'clouds': [], 'detections': [], 'selected': []}

    def callback(name):
        """Count all messages but retain only a small review sample."""
        def on_message(msg):
            arrived_ns = time.monotonic_ns()
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            counts[name] += 1
            if name == '/m20/camera/front/image_raw':
                arrivals['images'].append((stamp_ns, arrived_ns))
            elif name == '/LIDAR/POINTS':
                arrivals['clouds'].append((stamp_ns, arrived_ns))
            elif name.endswith('/detections'):
                arrivals['detections'].append((stamp_ns, arrived_ns))
            if name.endswith('/detections') and len(samples['detections_per_frame']) < 20:
                samples['detections_per_frame'].append(len(msg.detections))
            if name == '/human/location' and len(samples['locations']) < 20:
                samples['locations'].append({
                    'frame': msg.header.frame_id,
                    'stamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                    'xyz_m': [msg.point.x, msg.point.y, msg.point.z],
                })
            if name == '/human/selected_points':
                if msg.width:
                    selected_summary['nonempty'] += 1
                    arrivals['selected'].append((stamp_ns, arrived_ns))
                if (msg.header.frame_id != 'base_link_custom' or
                        msg.row_step != msg.width * 12 or len(msg.data) != msg.row_step):
                    selected_summary['invalid'] += 1
        return on_message

    subscriptions = [node.create_subscription(typ, name, callback(name), qos_profile_sensor_data)
                     for name, typ in types.items()]
    launch = player = None
    launch_tail = player_tail = ''
    try:
        with tempfile.TemporaryFile(mode='w+t') as launch_log, tempfile.TemporaryFile(mode='w+t') as player_log:
            # Start the pipeline before playback so its subscriptions can
            # appear in the ROS graph before sensor messages arrive.
            launch = subprocess.Popen(['ros2', 'launch', 'human_detection_node', 'perception.launch.py'],
                                      stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            ready_until = time.monotonic() + 10
            while time.monotonic() < ready_until:
                rclpy.spin_once(node, timeout_sec=0.1)
                if all(node.count_publishers(name) > 0 for name in types
                       if name not in ('/m20/camera/front/image_raw', '/LIDAR/POINTS')):
                    break
            player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag.resolve()), '--rate', str(args.rate),
                                       '--start-offset', str(args.start_offset), '--disable-keyboard-controls'],
                                      stdout=player_log, stderr=subprocess.STDOUT, start_new_session=True)
            until = time.monotonic() + args.duration
            while time.monotonic() < until and player.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            # Stop only the two processes this script created. The launch
            # process forwards shutdown to its child ROS nodes.
            for process in (player, launch):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            launch_log.seek(0)
            player_log.seek(0)
            launch_tail = launch_log.read()[-4000:]
            player_tail = player_log.read()[-1000:]
    finally:
        for process in (player, launch):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()
    def lag_stats_ms(pairs):
        """Summarize signed subscriber-arrival differences in milliseconds."""
        if not pairs:
            return None
        values = sorted((end - start) / 1e6 for start, end in pairs)
        return {'count': len(values), 'median': round(statistics.median(values), 1),
                'p95': round(values[round((len(values) - 1) * 0.95)], 1)}

    by_image = dict(arrivals['images'])
    by_detection = dict(arrivals['detections'])
    image_to_detection = [(by_image[stamp], arrival) for stamp, arrival in arrivals['detections']
                          if stamp in by_image]
    detection_to_selected = [(by_detection[stamp], arrival) for stamp, arrival in arrivals['selected']
                             if stamp in by_detection]
    cloud_to_selected = []
    image_to_cloud = []
    header_skews = []
    for stamp, selected_arrival in arrivals['selected']:
        if not arrivals['clouds']:
            break
        cloud_stamp, cloud_arrival = min(arrivals['clouds'], key=lambda item: abs(item[0] - stamp))
        if abs(cloud_stamp - stamp) <= 150_000_000:
            cloud_to_selected.append((cloud_arrival, selected_arrival))
            header_skews.append((0, abs(cloud_stamp - stamp)))
            if stamp in by_image:
                image_to_cloud.append((by_image[stamp], cloud_arrival))
    timing = {'image_to_detection_ms': lag_stats_ms(image_to_detection),
              'detection_to_selected_ms': lag_stats_ms(detection_to_selected),
              'matched_cloud_to_selected_ms': lag_stats_ms(cloud_to_selected),
              'image_to_matched_cloud_ms': lag_stats_ms(image_to_cloud),
              'image_cloud_header_skew_ms': lag_stats_ms(header_skews)}
    # The report makes a short smoke run inspectable without needing a GUI.
    report = {'duration_s': args.duration, 'rate': args.rate, 'start_offset_s': args.start_offset,
              'ros_domain_id': args.domain, 'counts': counts, 'samples': samples,
              'selected_points': selected_summary, 'arrival_timing': timing,
              'launch_log_tail': launch_tail, 'player_log_tail': player_tail}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    # This sample window should include a LiDAR-supported person. A positive
    # count alone is insufficient if a frame or coordinate is invalid.
    valid_locations = (counts['/human/location'] > 0 and
                       all(item['frame'] == 'base_link_custom' and
                           all(math.isfinite(value) for value in item['xyz_m'])
                           for item in samples['locations']))
    valid_selection = selected_summary['nonempty'] > 0 and selected_summary['invalid'] == 0
    return 0 if (counts['/m20/camera/front/camera_info'] and
                 counts['/m20/camera/front/human/detections'] and
                 valid_locations and valid_selection) else 1


if __name__ == '__main__':
    raise SystemExit(main())
