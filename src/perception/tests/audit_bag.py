#!/usr/bin/env python3
"""Read-only audit of the supplied ROS 2 SQLite bag and its sensor headers."""

import argparse
import bisect
import json
import sqlite3
import statistics
import sys
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2


# These values come from the assignment and this supplied recording. A changed
# recording should fail clearly rather than silently produce misleading stats.
EXPECTED = {
    "/m20/camera/front/image_raw": ("sensor_msgs/msg/Image", Image, 370),
    "/LIDAR/POINTS": ("sensor_msgs/msg/PointCloud2", PointCloud2, 371),
}
CLOUD_FIELDS = {
    "x": (0, 7, 1),
    "y": (4, 7, 1),
    "z": (8, 7, 1),
    "intensity": (12, 7, 1),
    "ring": (16, 4, 1),
    "timestamp": (18, 8, 1),
}


def require(condition, message):
    """Raise one readable audit error when an input assumption is false."""
    if not condition:
        raise ValueError(message)


def percentile(values, fraction):
    """Select a value from sorted samples without another statistics package."""
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def millisecond_stats(values):
    """Summarize a list of intervals already expressed in milliseconds."""
    if not values:
        return None
    return {
        "min": round(min(values), 2),
        "median": round(statistics.median(values), 2),
        "p95": round(percentile(values, 0.95), 2),
        "max": round(max(values), 2),
    }


def audit(bag_dir):
    """Check metadata, SQLite payloads, and sensor timestamp relationships."""
    metadata_path = bag_dir / "metadata.yaml"
    require(metadata_path.is_file(), f"missing metadata: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
    require(metadata["storage_identifier"] == "sqlite3", "expected sqlite3 storage")
    paths = metadata["relative_file_paths"]
    require(paths == ["person_walking_0.db3"], f"unexpected database paths: {paths}")
    db_path = bag_dir / paths[0]
    require(db_path.is_file(), f"missing database: {db_path}")
    require(db_path.stat().st_size > 0, f"empty database: {db_path}")

    # mode=ro avoids creating a missing database or changing the supplied input.
    with sqlite3.connect(f"file:{db_path.resolve().as_posix()}?mode=ro", uri=True) as db:
        integrity = db.execute("PRAGMA quick_check").fetchone()[0]
        require(integrity == "ok", f"SQLite quick_check failed: {integrity}")
        topics = {
            name: (topic_id, type_name, serialization)
            for topic_id, name, type_name, serialization in db.execute(
                "SELECT id, name, type, serialization_format FROM topics"
            )
        }
        require(set(topics) == set(EXPECTED), f"unexpected topics: {sorted(topics)}")
        metadata_counts = {
            item["topic_metadata"]["name"]: item["message_count"]
            for item in metadata["topics_with_message_count"]
        }
        # The recorded best-effort profile determines which subscriber QoS
        # can actually receive messages during replay.
        metadata_qos = {
            item["topic_metadata"]["name"]: yaml.safe_load(
                item["topic_metadata"]["offered_qos_profiles"]
            )
            for item in metadata["topics_with_message_count"]
        }
        result = {
            "database": str(db_path),
            "database_bytes": db_path.stat().st_size,
            "sqlite_quick_check": integrity,
            "topics": {},
        }
        stamps = {}
        message_times = {}
        for name, (expected_type, message_type, expected_count) in EXPECTED.items():
            topic_id, type_name, serialization = topics[name]
            require(type_name == expected_type, f"{name}: wrong type {type_name}")
            require(serialization == "cdr", f"{name}: wrong serialization {serialization}")
            profiles = metadata_qos[name]
            require(len(profiles) == 1, f"{name}: expected one offered QoS profile")
            require(profiles[0]["reliability"] == 2, f"{name}: expected best-effort QoS")
            require(profiles[0]["durability"] == 2, f"{name}: expected volatile QoS")
            bag_times, header_times, lags_ms = [], [], []
            widths = set()
            frames = set()
            encodings = set()
            point_counts = []
            count = 0
            # Deserializing every payload catches a damaged message that
            # metadata counts and SQLite quick_check alone would miss.
            for bag_ns, payload in db.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_id,),
            ):
                msg = deserialize_message(payload, message_type)
                header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                require(header_ns > 0, f"{name}: nonpositive header timestamp")
                bag_times.append(bag_ns)
                header_times.append(header_ns)
                lags_ms.append((bag_ns - header_ns) / 1_000_000)
                frames.add(msg.header.frame_id)
                if message_type is Image:
                    widths.add((msg.width, msg.height))
                    encodings.add(msg.encoding)
                    require(msg.step == msg.width * 3, "unexpected RGB row step")
                    require(len(msg.data) == msg.step * msg.height, "image data size mismatch")
                else:
                    require(msg.height == 1, "expected unorganized cloud")
                    require(msg.point_step == 26, "unexpected cloud point_step")
                    require(msg.row_step == msg.width * msg.point_step, "cloud row_step mismatch")
                    require(len(msg.data) == msg.row_step, "cloud data size mismatch")
                    fields = {
                        field.name: (field.offset, field.datatype, field.count)
                        for field in msg.fields
                    }
                    require(fields == CLOUD_FIELDS, f"unexpected cloud fields: {fields}")
                    point_counts.append(msg.width)
                count += 1
            require(count == expected_count, f"{name}: {count} messages; expected {expected_count}")
            require(count == metadata_counts.get(name), f"{name}: metadata count mismatch")
            require(count > 0, f"{name}: no messages")
            expected_frame = (
                "front_wide_angle_camera_frame" if message_type is Image else "lidar_link"
            )
            require(frames == {expected_frame}, f"{name}: unexpected frames {frames}")
            # Bag time is when rosbag stored a message; header time is the
            # sensor's measurement time. They are deliberately reported apart.
            entry = {
                "type": type_name,
                "count": count,
                "frame_id": expected_frame,
                "offered_qos": "best_effort/volatile",
                "bag_interval_ms": millisecond_stats(
                    [(b - a) / 1_000_000 for a, b in zip(bag_times, bag_times[1:])]
                ),
                "bag_minus_header_ms": millisecond_stats(lags_ms),
                "header_backwards_in_bag_order": sum(
                    b < a for a, b in zip(header_times, header_times[1:])
                ),
            }
            if message_type is Image:
                require(widths == {(1280, 720)}, f"unexpected image sizes: {widths}")
                require(encodings == {"rgb8"}, f"unexpected encodings: {encodings}")
                entry.update({"width": 1280, "height": 720, "encoding": "rgb8"})
            else:
                entry.update({
                    "fields": sorted(CLOUD_FIELDS),
                    "point_count_min": min(point_counts),
                    "point_count_max": max(point_counts),
                })
            result["topics"][name] = entry
            stamps[name] = header_times
            message_times[name] = list(zip(header_times, bag_times))

    # Sort cloud sensor times because they are not monotonic in bag order.
    # Each image is compared to its closest cloud by sensor time.
    image_stamps = stamps["/m20/camera/front/image_raw"]
    cloud_stamps = sorted(stamps["/LIDAR/POINTS"])
    skews_ms = []
    for stamp in image_stamps:
        index = bisect.bisect_left(cloud_stamps, stamp)
        candidates = cloud_stamps[max(0, index - 1):index + 1]
        skews_ms.append(min(abs(stamp - other) for other in candidates) / 1_000_000)
    result["nearest_cloud_header_skew_ms"] = {
        **millisecond_stats(skews_ms),
        "within_50_ms": sum(value <= 50 for value in skews_ms),
        "within_100_ms": sum(value <= 100 for value in skews_ms),
        "within_150_ms": sum(value <= 150 for value in skews_ms),
        "final_six_ms": [round(value, 2) for value in skews_ms[-6:]],
    }
    # These bag timestamps determine when playback schedules the messages.
    # A negative gap means the cloud was stored before its nearest image.
    cloud_by_header = sorted(message_times["/LIDAR/POINTS"])
    cloud_headers = [header for header, _ in cloud_by_header]
    arrival_gaps_ms = []
    for image_header, image_bag_time in message_times["/m20/camera/front/image_raw"]:
        index = bisect.bisect_left(cloud_headers, image_header)
        nearest = min(cloud_by_header[max(0, index - 1):index + 1],
                      key=lambda item: abs(item[0] - image_header))
        if abs(nearest[0] - image_header) <= 150_000_000:
            arrival_gaps_ms.append((nearest[1] - image_bag_time) / 1_000_000)
    result["matched_cloud_bag_after_image_ms"] = {
        **millisecond_stats(arrival_gaps_ms),
        "matched_pairs": len(arrival_gaps_ms),
        "cloud_stored_after_image": sum(gap > 0 for gap in arrival_gaps_ms),
    }
    require(sum(item["count"] for item in result["topics"].values()) == metadata["message_count"],
            "total metadata count mismatch")
    return result


def main():
    """Print a JSON report and return a nonzero exit code on bad input."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="directory containing metadata.yaml and .db3")
    args = parser.parse_args()
    try:
        print(json.dumps(audit(args.bag), indent=2))
    except (OSError, sqlite3.Error, ValueError, KeyError) as exc:
        print(f"bag audit failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
