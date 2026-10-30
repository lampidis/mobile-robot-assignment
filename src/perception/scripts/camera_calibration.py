#!/usr/bin/env python3
"""Calibrate the supplied 9x7, 20 mm chessboard images with OpenCV fisheye."""

import argparse
import json
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import yaml

# OpenCV expects *inner* corner counts. Object coordinates use metres so the
# resulting calibration can be paired with metre-valued LiDAR points.
PATTERN = (9, 7)
SQUARE_M = 0.020


class Sample(NamedTuple):
    """One image's name and its matching 3D/2D chessboard corners."""

    name: str
    object_points: np.ndarray
    image_points: np.ndarray


def fit(samples, image_size):
    """Fit the selected images once, starting from an equidistant lens guess."""
    width, height = image_size
    # An equidistant fisheye roughly follows image radius = focal * angle.
    # If half the image spans about 90 degrees, focal is width / pi pixels.
    # This is only an initial guess: calibration optimizes all K/D values.
    focal_guess = max(image_size) / np.pi
    k = np.array([[focal_guess, 0, width / 2],
                  [0, focal_guess, height / 2],
                  [0, 0, 1]], dtype=np.float64)
    d = np.zeros((4, 1), dtype=np.float64)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    return cv2.fisheye.calibrate(
        [sample.object_points for sample in samples],
        [sample.image_points for sample in samples],
        image_size, k, d, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )


def main():
    """Read images, fit the fisheye model, and write reviewable outputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, default=Path('src/perception/data/calibration_images'))
    parser.add_argument('--output', type=Path, default=Path('src/perception/front_camera_info/config/camera_calibration.yaml'))
    parser.add_argument('--results', type=Path, default=Path('src/perception/results'))
    parser.add_argument('--exclude', nargs='*', default=['camera0_20261007162543.png'],
                        help='image basenames to exclude after review; default excludes visibly warped board')
    args = parser.parse_args()
    files = sorted(args.images.glob('*.png'))
    if not files:
        parser.error(f'no PNG images in {args.images}')
    args.results.mkdir(parents=True, exist_ok=True)
    # Every chessboard has the same flat 3D corner layout (Z=0). OpenCV pairs
    # these known points with the detected 2D image corners.
    object_grid = np.zeros((1, PATTERN[0] * PATTERN[1], 3), np.float64)
    object_grid[0, :, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_M
    # Keep each image name and its corner arrays together so exclusions cannot
    # accidentally misalign the 3D and 2D calibration inputs.
    samples, rejected = [], {}
    image_size = None
    for path in files:
        image = cv2.imread(str(path))
        if image is None:
            rejected[path.name] = 'could not decode image'
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            rejected[path.name] = f'image size {size} differs from {image_size}'
            continue
        # A calibration image is useful only if all 63 inner corners are found.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN)
        if not found:
            rejected[path.name] = '9x7 inner corners not found'
            continue
        samples.append(Sample(path.name, object_grid.copy(),
                              corners.reshape(1, -1, 2).astype(np.float64)))
    if len(samples) < 4:
        raise SystemExit(f'only {len(samples)} usable images; need at least four')
    if args.exclude:
        # The visibly warped board violates the flat-board model. Record every
        # exclusion so the final calibration is traceable to its input photos.
        for sample in samples:
            if sample.name in args.exclude:
                rejected[sample.name] = 'excluded after visual/geometric review'
        samples = [sample for sample in samples if sample.name not in args.exclude]
        if len(samples) < 4:
            raise SystemExit('fewer than four images remain after exclusions')
    rms, k, d, rvecs, tvecs = fit(samples, image_size)
    errors = {}
    for sample, rv, tv in zip(samples, rvecs, tvecs):
        # Per-image error exposes warped or poorly detected boards that a
        # single overall RMS could hide.
        projected, _ = cv2.fisheye.projectPoints(sample.object_points, rv, tv, k, d)
        errors[sample.name] = float(np.sqrt(np.mean(np.sum((projected - sample.image_points) ** 2, axis=2))))
    # ROS CameraInfo stores K, D, R, and P separately. R is identity because
    # this script calibrates one camera; P has no stereo translation column.
    p = np.hstack((k, np.zeros((3, 1))))
    calibration = {
        'image_width': image_size[0], 'image_height': image_size[1],
        'camera_name': 'front_wide_angle_camera', 'distortion_model': 'equidistant',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.reshape(-1).tolist()},
        'distortion_coefficients': {'rows': 1, 'cols': 4, 'data': d.reshape(-1).tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).reshape(-1).tolist()},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': p.reshape(-1).tolist()},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(calibration, sort_keys=False))

    # Show corners in three different accepted photos so a reviewer can see
    # that detection works across the dataset, not only in the first image.
    examples = [
        ('calibration_corners.png', samples[0]),
        ('calibration_corners_middle.png', samples[len(samples) // 2]),
        ('calibration_corners_last.png', samples[-1]),
    ]
    visual_examples = []
    for filename, sample in examples:
        image = cv2.imread(str(args.images / sample.name))
        corners = sample.image_points.reshape(-1, 1, 2).astype(np.float32)
        cv2.drawChessboardCorners(image, PATTERN, corners, True)
        output_path = args.results / filename
        if not cv2.imwrite(str(output_path), image):
            raise SystemExit(f'could not save visual example: {output_path}')
        visual_examples.append({'image': sample.name, 'output': str(output_path)})

    report = {'rms_pixels': float(rms),
              'initial_focal_pixels': float(max(image_size) / np.pi),
              'initialization': 'equidistant 180-degree field-of-view guess',
              'image_size': list(image_size),
              'accepted': [sample.name for sample in samples],
              'rejected': rejected, 'per_image_rms_pixels': errors,
              'worst_images': sorted(errors, key=errors.get, reverse=True)[:5],
              'visual_examples': visual_examples}
    (args.results / 'calibration_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
