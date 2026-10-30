#!/usr/bin/env python3
"""Check that simple depth selection resists floor and distant background."""

import unittest

import numpy as np
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D

from human_detection_node.locator import select_location, selected_cloud


class LocationSelectionTest(unittest.TestCase):
    """Use small synthetic clouds with known foreground and background."""

    def setUp(self):
        self.box = BoundingBox2D()
        self.box.center.position.x = 100.0
        self.box.center.position.y = 100.0
        self.box.size_x = 100.0
        self.box.size_y = 100.0

    def test_supported_person_beats_floor_and_wall(self):
        # Eight person returns at 2 m, floor returns, and a denser 5 m wall.
        pixels = np.tile([100.0, 100.0], (30, 1))
        optical = np.array([[2.0, 0.0, 2.0]] * 8 +
                           [[2.0, 0.0, 2.1]] * 2 +
                           [[1.0, 0.0, 1.0]] * 8 +
                           [[5.0, 0.0, 5.0]] * 12)
        base = optical.copy()
        base[10:18, 2] = -0.6  # Ground points can project inside a box.
        base[:10, 2] = 0.1
        base[18:, 2] = 0.2
        location = select_location(pixels, optical, base, self.box, -0.6)
        self.assertIsNotNone(location)
        self.assertAlmostEqual(location[0], 2.0)
        self.assertAlmostEqual(location[2], 0.1)

    def test_sparse_foreground_does_not_select_distant_wall(self):
        # Two near returns do not justify jumping to a well-sampled 5 m wall.
        pixels = np.tile([100.0, 100.0], (14, 1))
        optical = np.array([[1.0, 0.0, 1.0]] * 2 + [[5.0, 0.0, 5.0]] * 12)
        base = optical.copy()
        base[:, 2] = 0.0
        self.assertIsNone(select_location(pixels, optical, base, self.box, -0.6))

    def test_floor_only_has_no_location(self):
        pixels = np.tile([100.0, 100.0], (12, 1))
        optical = np.tile([2.0, 0.0, 2.0], (12, 1))
        base = np.tile([2.0, 0.0, -0.6], (12, 1))
        self.assertIsNone(select_location(pixels, optical, base, self.box, -0.6))

    def test_selected_cloud_keeps_points_and_measurement_time(self):
        # RViz receives the exact selected points in the robot base frame.
        header = Header()
        header.stamp.sec = 123
        points = np.array([[1.0, 2.0, 0.1], [1.2, 2.1, 0.2]])
        cloud = selected_cloud(header, points)
        self.assertEqual(cloud.header.frame_id, 'base_link_custom')
        self.assertEqual(cloud.header.stamp.sec, 123)
        self.assertEqual(cloud.width, 2)
        self.assertEqual(cloud.point_step, 12)
        self.assertEqual([field.name for field in cloud.fields], ['x', 'y', 'z'])
        np.testing.assert_allclose(np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3), points)
        self.assertEqual(selected_cloud(header, []).width, 0)


if __name__ == '__main__':
    unittest.main()
