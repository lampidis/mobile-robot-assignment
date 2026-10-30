#!/usr/bin/env python3
"""Play the supplied bag once and record messages received by ROS subscribers."""

import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml


def parse_args():
    """Read playback settings; the default speed is gentle on large clouds."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True, help="ROS 2 bag directory")
    parser.add_argument("--rate", type=float, default=0.5, help="playback speed (default: 0.5)")
    parser.add_argument("--output", type=Path, required=True, help="JSON report path")
    parser.add_argument("--domain", type=int, default=87, help="isolated ROS_DOMAIN_ID (default: 87)")
    parser.add_argument("--timeout", type=float, help="wall-clock timeout; default is bag duration/rate + 30 s")
    parser.add_argument("--drain", type=float, default=2.0, help="seconds to receive queued messages after playback")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.drain < 0:
        parser.error("--drain must be nonnegative")
    if not 0 <= args.domain <= 232:
        parser.error("--domain must be between 0 and 232")
    return args


def run(args):
    """Replay one full bag and count messages observed by ROS subscribers."""
    metadata_path = args.bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"missing metadata: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
    for path in metadata["relative_file_paths"]:
        if not (args.bag / path).is_file():
            raise ValueError(f"missing database: {args.bag / path}")
    # Database counts are the baseline; best-effort transport may deliver
    # fewer messages to this subscriber even when the player exits normally.
    expected = {
        item["topic_metadata"]["name"]: item["message_count"]
        for item in metadata["topics_with_message_count"]
    }
    duration_s = metadata["duration"]["nanoseconds"] / 1_000_000_000
    timeout_s = args.timeout if args.timeout is not None else duration_s / args.rate + 30.0
    if timeout_s <= 0:
        raise ValueError("timeout must be positive")

    # Set the domain before rclpy.init() and before launching the child player
    # so both processes communicate without mixing with another ROS run.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosbag2_interfaces.srv import Resume
    from sensor_msgs.msg import Image, PointCloud2

    rclpy.init()
    node = Node("perception_playback_probe")
    topics = {
        "/m20/camera/front/image_raw": Image,
        "/LIDAR/POINTS": PointCloud2,
    }
    received = {
        name: {"count": 0, "first_header_ns": None, "last_header_ns": None,
               "first_arrival_s": None, "last_arrival_s": None, "frames": set()}
        for name in topics
    }
    arrivals = {name: [] for name in topics}
    complete_cloud_messages = 0
    invalid_cloud_messages = 0
    start = time.monotonic()

    def make_callback(name):
        """Bind each topic name to a lightweight count/timestamp callback."""
        def callback(msg):
            nonlocal complete_cloud_messages, invalid_cloud_messages
            item = received[name]
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            now = round(time.monotonic() - start, 3)
            # Store one monotonic arrival time per complete ROS message. The
            # points inside a PointCloud2 do not arrive as separate callbacks.
            arrivals[name].append((stamp, time.monotonic_ns()))
            if name == "/LIDAR/POINTS":
                if (msg.row_step == msg.width * msg.point_step and
                        len(msg.data) == msg.row_step * msg.height):
                    complete_cloud_messages += 1
                else:
                    invalid_cloud_messages += 1
            item["count"] += 1
            if item["first_header_ns"] is None:
                item["first_header_ns"] = stamp
                item["first_arrival_s"] = now
            item["last_header_ns"] = stamp
            item["last_arrival_s"] = now
            item["frames"].add(msg.header.frame_id)
        return callback

    subscriptions = [
        node.create_subscription(typ, name, make_callback(name), qos_profile_sensor_data)
        for name, typ in topics.items()
    ]
    player = None
    timed_out = False
    player_log = ""
    discovery_s = None
    try:
        # Allow the subscriber endpoints to enter the ROS graph before playback starts.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        with tempfile.TemporaryFile(mode="w+t") as log:
            # Pausing lets us discover both publishers before requesting the
            # first message. It reduces startup loss but cannot guarantee all
            # best-effort messages will reach this subscriber.
            player = subprocess.Popen(
                ["ros2", "bag", "play", str(args.bag.resolve()), "--rate", str(args.rate),
                 "--start-paused", "--disable-keyboard-controls"],
                stdout=log, stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + timeout_s
            resume = node.create_client(Resume, "/rosbag2_player/resume")
            while player.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if resume.service_is_ready() and all(
                    node.count_publishers(name) > 0 for name in topics
                ):
                    discovery_s = round(time.monotonic() - start, 3)
                    break
            if discovery_s is not None:
                # DDS graph discovery precedes the first message; a brief
                # extra spin gives endpoints time to finish matching.
                rclpy.spin_once(node, timeout_sec=0.5)
                future = resume.call_async(Resume.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.exception() is not None:
                    raise RuntimeError("could not resume paused bag player")
            while player.poll() is None:
                if time.monotonic() >= deadline:
                    timed_out = True
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            # Let callbacks drain after the player exits before final counts.
            if not timed_out:
                drain_until = time.monotonic() + args.drain
                while time.monotonic() < drain_until:
                    rclpy.spin_once(node, timeout_sec=0.1)
            if player.poll() is None:
                player.send_signal(signal.SIGINT)
            try:
                player.wait(timeout=5)
            except subprocess.TimeoutExpired:
                player.kill()
                player.wait()
            log.seek(0)
            player_log = log.read()[-2000:]
    finally:
        if player is not None and player.poll() is None:
            player.kill()
            player.wait()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()

    # Save evidence even when playback exits with an error or no messages;
    # the caller can inspect player_exit_code and the captured log tail.
    report = {
        "bag": str(args.bag.resolve()),
        "ros_domain_id": args.domain,
        "playback_rate": args.rate,
        "subscriber_qos": "sensor_data_best_effort",
        "bag_duration_s": round(duration_s, 3),
        "wall_duration_s": round(time.monotonic() - start, 3),
        "publisher_discovery_s": discovery_s,
        "timeout_s": round(timeout_s, 3),
        "timed_out": timed_out,
        "player_exit_code": player.returncode,
        "topics": {},
        "player_log_tail": player_log,
    }
    # Compare arrival times only for image/cloud pairs close in sensor time.
    # The closest cloud may have arrived earlier or later than the image.
    gaps_ms = []
    for image_stamp, image_arrival in arrivals["/m20/camera/front/image_raw"]:
        if not arrivals["/LIDAR/POINTS"]:
            break
        cloud_stamp, cloud_arrival = min(arrivals["/LIDAR/POINTS"],
                                       key=lambda item: abs(item[0] - image_stamp))
        if abs(cloud_stamp - image_stamp) <= 150_000_000:
            gaps_ms.append((cloud_arrival - image_arrival) / 1_000_000)
    ordered_gaps = sorted(gaps_ms)
    report["matched_cloud_arrival_after_image_ms"] = (
        {"pairs": len(ordered_gaps), "min": round(ordered_gaps[0], 1),
         "median": round(statistics.median(ordered_gaps), 1),
         "p95": round(ordered_gaps[round((len(ordered_gaps) - 1) * 0.95)], 1),
         "max": round(ordered_gaps[-1], 1)} if ordered_gaps else None)
    report["cloud_payloads"] = {"complete": complete_cloud_messages,
                                "invalid": invalid_cloud_messages}
    for name, item in received.items():
        report["topics"][name] = {
            **{key: value for key, value in item.items() if key != "frames"},
            "frame_ids": sorted(item["frames"]),
            "metadata_count": expected.get(name),
            "received_fraction": round(item["count"] / expected[name], 3) if expected.get(name) else None,
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if player.returncode == 0 and not timed_out and all(
        item["count"] > 0 for item in received.values()
    ) else 1


def main():
    """Convert expected input/runtime failures into a short CLI error."""
    args = parse_args()
    try:
        return run(args)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(f"playback probe failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
