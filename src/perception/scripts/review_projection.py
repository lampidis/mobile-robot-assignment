#!/usr/bin/env python3
"""Save camera images with time-matched LiDAR points projected onto them."""

import argparse
import json
import sqlite3
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D

from human_detection_node.locator import cloud_xyz, select_location, stamp_ns


def lidar_to_optical(urdf_path):
    """Read the supplied fixed-joint chain and convert LiDAR XYZ into optical XYZ."""
    def rpy_matrix(rpy):
        """URDF fixed joints rotate roll, then pitch, then yaw."""
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        roll_r = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        pitch_r = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        yaw_r = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return yaw_r @ pitch_r @ roll_r

    joints = {}
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        origin = joint.find('origin').attrib
        translation = np.fromstring(origin['xyz'], sep=' ')
        rotation = rpy_matrix(np.fromstring(origin['rpy'], sep=' '))
        joints[child] = (parent, rotation, translation)

    def to_base(frame):
        """Compose fixed joints from one sensor frame back to the robot base."""
        rotation = np.eye(3)
        translation = np.zeros(3)
        while frame != 'base_link_custom':
            frame, joint_rotation, joint_translation = joints[frame]
            translation = joint_rotation @ translation + joint_translation
            rotation = joint_rotation @ rotation
        return rotation, translation

    lidar_r, lidar_t = to_base('lidar_link')
    optical_r, optical_t = to_base('front_wide_angle_camera_frame')
    # Convert a LiDAR point to the base first, then invert the optical pose.
    return optical_r.T @ lidar_r, optical_r.T @ (lidar_t - optical_t), lidar_r, lidar_t


def selected_images(db, topic, indices):
    """Read just the requested bag frames while retaining their source headers."""
    images = {}
    for index, (payload,) in enumerate(db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,))):
        if index in indices:
            images[index] = deserialize_message(payload, Image)
    if set(images) != set(indices):
        raise ValueError(f'missing image indices: {sorted(set(indices) - set(images))}')
    return images


def nearest_clouds(db, topic, images):
    """Match by sensor header time; bag arrival order is not sensor time order."""
    best = {index: (float('inf'), None) for index in images}
    for (payload,) in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,)):
        cloud = deserialize_message(payload, PointCloud2)
        cloud_time = stamp_ns(cloud.header.stamp)
        for index, image in images.items():
            skew = abs(cloud_time - stamp_ns(image.header.stamp))
            if skew < best[index][0]:
                best[index] = (skew, cloud)
    return best


def audit_all_clouds(db, topic, rotation, translation, k, d, output):
    """Measure which camera rows receive LiDAR returns across the full bag."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    row_edges = [0, 120, 240, 360, 480, 600, 720]
    clouds = []
    for index, (payload,) in enumerate(db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,))):
        cloud = deserialize_message(payload, PointCloud2)
        xyz = cloud_xyz(cloud, 1)
        optical = xyz @ rotation.T + translation
        front = np.isfinite(optical).all(axis=1) & (optical[:, 2] > 0.1)
        over_30m = int((front & (optical[:, 2] >= 30)).sum())
        optical = optical[front & (optical[:, 2] < 30)]
        pixels, _ = cv2.fisheye.projectPoints(optical.reshape(-1, 1, 3),
                                              np.zeros(3), np.zeros(3), k, d,
                                              alpha=k[0, 1] / k[0, 0])
        pixels = pixels.reshape(-1, 2)
        in_image = ((pixels[:, 0] >= 0) & (pixels[:, 0] < 1280) &
                    (pixels[:, 1] >= 0) & (pixels[:, 1] < 720))
        pixels = pixels[in_image]
        central = (pixels[:, 0] >= 400) & (pixels[:, 0] < 880)
        # Point counts describe returns, not the fraction of pixels covered.
        clouds.append({'cloud': index, 'raw_points': len(xyz), 'visible_points': len(pixels),
                       'row_counts': np.histogram(pixels[:, 1], bins=row_edges)[0].tolist(),
                       'central_top_240_fraction': round(float((pixels[central, 1] < 240).sum() /
                                                               central.sum()), 4),
                       'front_points_over_30m': over_30m})

    counts = np.sum([item['row_counts'] for item in clouds], axis=0)
    central_fractions = [item['central_top_240_fraction'] for item in clouds]
    summary = {'cloud_count': len(clouds), 'row_counts_total': counts.tolist(),
               'row_fractions_total': (counts / counts.sum()).round(4).tolist(),
               'central_top_240_fraction_min_median_max': [round(float(value), 4) for value in
                   (min(central_fractions), np.median(central_fractions), max(central_fractions))],
               'front_points_over_30m_total': sum(item['front_points_over_30m'] for item in clouds)}
    (output / 'coverage-audit.json').write_text(json.dumps({'summary': summary, 'clouds': clouds}, indent=2) + '\n')

    # This chart makes the sparse top third easy to compare with lower rows.
    labels = ['0–120 (top)', '120–240', '240–360', '360–480', '480–600', '600–720 (bottom)']
    percentages = counts / counts.sum() * 100
    figure, axis = plt.subplots(figsize=(9, 4.5))
    bars = axis.barh(labels, percentages, color=['#9ca3af'] * 2 + ['#2563eb'] * 4)
    axis.invert_yaxis()
    axis.set_xlim(0, 40)
    axis.set_xlabel('Share of projected LiDAR returns inside the camera image (%)')
    axis.set_title(f'Recorded LiDAR coverage by image row — all {len(clouds)} clouds')
    for bar, value in zip(bars, percentages):
        axis.text(value + 0.4, bar.get_y() + bar.get_height() / 2, f'{value:.1f}%', va='center')
    figure.tight_layout()
    figure.savefig(output / 'coverage-rows.png', dpi=160)
    plt.close(figure)
    return summary


def main():
    """Draw depth-coloured LiDAR projections and record per-box point counts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, nargs='+', default=[30, 184, 338])
    parser.add_argument('--output', type=Path, default=Path('out/perception/projection-review'))
    parser.add_argument('--coverage-all', action='store_true',
                        help='measure projected image-row coverage for every recorded cloud')
    args = parser.parse_args()
    if any(index < 0 for index in args.frames):
        parser.error('frame indices must be nonnegative')
    calibration = Path('src/perception/front_camera_info/config/camera_calibration.yaml')
    urdf = Path('src/perception/front_camera_info/urdf/front_sensors.urdf')
    bag = Path('src/perception/data/rosbag/person_walking/person_walking_0.db3')
    config = yaml.safe_load(calibration.read_text())
    k = np.array(config['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    d = np.array(config['distortion_coefficients']['data'], dtype=np.float64)
    rotation, translation, base_rotation, base_translation = lidar_to_optical(urdf)
    args.output.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(f'file:{bag.resolve()}?mode=ro', uri=True) as db:
        topics = dict(db.execute('SELECT name, id FROM topics'))
        if args.coverage_all:
            print(json.dumps(audit_all_clouds(db, topics['/LIDAR/POINTS'], rotation,
                                              translation, k, d, args.output), indent=2))
            return
        images = selected_images(db, topics['/m20/camera/front/image_raw'], args.frames)
        matched = nearest_clouds(db, topics['/LIDAR/POINTS'], images)
    model = YOLO('/opt/models/yolo26n.pt')
    report = []
    for index, image in images.items():
        skew_ns, cloud = matched[index]
        if cloud is None:
            raise ValueError(f'no cloud available for frame {index}')
        rgb = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
        canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
        # Use every point for this diagnostic; production may sample for speed.
        xyz = cloud_xyz(cloud, 1)
        sampled = np.arange(len(xyz)) % 4 == 0
        optical = xyz @ rotation.T + translation
        base = xyz @ base_rotation.T + base_translation
        finite = np.isfinite(optical).all(axis=1)
        front = finite & (optical[:, 2] > 0.1)
        over_30m = int((front & (optical[:, 2] >= 30)).sum())
        valid = front & (optical[:, 2] < 30)
        optical, base, sampled = optical[valid], base[valid], sampled[valid]
        sampled_nearby = sampled & (optical[:, 2] < 5)
        sampled_floor_height = float(np.percentile(base[sampled_nearby, 2], 20))
        # OpenCV takes skew separately as alpha; K[0,1] is in pixel units.
        pixels, _ = cv2.fisheye.projectPoints(optical.reshape(-1, 1, 3),
                                              np.zeros(3), np.zeros(3), k, d,
                                              alpha=k[0, 1] / k[0, 0])
        pixels = pixels.reshape(-1, 2)
        in_image = ((pixels[:, 0] >= 0) & (pixels[:, 0] < image.width) &
                    (pixels[:, 1] >= 0) & (pixels[:, 1] < image.height))
        pixels, optical, base, sampled = pixels[in_image], optical[in_image], base[in_image], sampled[in_image]
        # Count the full projected cloud by image row, before person filtering.
        # Sparse upper rows are a sensor-coverage observation, not YOLO masking.
        row_counts = np.histogram(pixels[:, 1], bins=[0, 120, 240, 360, 480, 600, 720])[0]
        central = (pixels[:, 0] >= 400) & (pixels[:, 0] < 880)
        central_top_fraction = float((pixels[central, 1] < 240).sum() / central.sum())
        # Blue is near and red is far; the legend keeps the overlay readable.
        depth_color = np.clip((optical[:, 2] - 0.5) / 4.5 * 255, 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(depth_color.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
        for pixel, color in zip(pixels.astype(int), colors):
            cv2.circle(canvas, tuple(pixel), 2, tuple(int(v) for v in color), -1)
        result = model.predict(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), classes=[0],
                               conf=0.35, imgsz=640, device='cpu', verbose=False)[0]
        boxes = []
        floor_height = float(np.percentile(base[optical[:, 2] < 5, 2], 20))
        for box in result.boxes:
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2
            bbox.center.position.y = (y1 + y2) / 2
            bbox.size_x, bbox.size_y = x2 - x1, y2 - y1
            # The runtime locator declines a cloud more than 150 ms away.
            location = (select_location(pixels[sampled], optical[sampled], base[sampled],
                                        bbox, sampled_floor_height)
                        if skew_ns <= 150_000_000 else None)
            if location is not None:
                # Project the selected 3D median back for a visual sanity check.
                lidar_point = (location - base_translation) @ base_rotation
                optical_point = lidar_point @ rotation.T + translation
                marker, _ = cv2.fisheye.projectPoints(optical_point.reshape(1, 1, 3),
                                                      np.zeros(3), np.zeros(3), k, d,
                                                      alpha=k[0, 1] / k[0, 0])
                marker_xy = tuple(int(round(v)) for v in marker.reshape(2))
            inside = ((pixels[:, 0] >= x1) & (pixels[:, 0] <= x2) &
                      (pixels[:, 1] >= y1) & (pixels[:, 1] <= y2))
            depths = optical[inside, 2]
            heights = base[inside, 2]
            raised = depths[heights > floor_height + 0.15]
            raised_pixels = pixels[inside][heights > floor_height + 0.15]
            for pixel in raised_pixels.astype(int):
                # Magenta identifies points that survive the floor filter.
                cv2.circle(canvas, tuple(pixel), 3, (255, 0, 255), -1)
            if location is not None:
                cv2.drawMarker(canvas, marker_xy, (0, 255, 255), cv2.MARKER_CROSS, 24, 3)
            # A coarse depth histogram reveals whether foreground returns
            # form one person-like group or multiple competing surfaces.
            histogram, edges = np.histogram(raised, bins=np.arange(0, 10.25, 0.25))
            bins = [[round(float(edges[i]), 2), int(count)] for i, count in enumerate(histogram) if count]
            median_pixels = [
                [round(float(edges[i]), 2),
                 [round(float(v), 1) for v in np.median(raised_pixels[(raised >= edges[i]) & (raised < edges[i + 1])], axis=0)]]
                for i, count in enumerate(histogram) if count
            ]
            boxes.append({'xyxy': [round(v, 1) for v in (x1, y1, x2, y2)],
                          'projected_points': int(inside.sum()),
                          'depth_m_p10_p50_p90': [round(float(np.percentile(depths, p)), 2)
                                                  for p in (10, 50, 90)] if len(depths) else [],
                          'height_m_p10_p50_p90': [round(float(np.percentile(heights, p)), 2)
                                                   for p in (10, 50, 90)] if len(heights) else [],
                          'selected_xyz_base_m': [round(float(v), 2) for v in location]
                                                 if location is not None else None,
                          'raised_depth_bins_m_count': bins,
                          'raised_bin_median_pixels': median_pixels})
            cv2.rectangle(canvas, (round(x1), round(y1)), (round(x2), round(y2)), (0, 255, 0), 2)
        cv2.putText(canvas, f'frame {index} | skew {skew_ns / 1e6:.1f} ms | blue near, red far',
                    (18, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imwrite(str(args.output / f'frame-{index:03d}.png'), canvas)
        nearby_heights = base[optical[:, 2] < 5, 2]
        report.append({'frame': index, 'skew_ms': round(skew_ns / 1e6, 2),
                       'raw_lidar_points': len(xyz), 'front_points_over_30m': over_30m,
                       'visible_lidar_points': len(pixels),
                       'projected_row_counts_120px': row_counts.tolist(),
                       'central_top_240_fraction': round(central_top_fraction, 4),
                       'nearby_height_m_p10_p20_p30_p50': [round(float(np.percentile(nearby_heights, p)), 2)
                                                            for p in (10, 20, 30, 50)],
                       'boxes': boxes})
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
