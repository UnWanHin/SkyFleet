"""几何纯函数测试。遮挡判定的正确性完全依赖本模块。"""
import unittest

from skyfleet.world.geometry import (distance_2d, distance_3d,
                                     segment_intersects_rect)


class TestSegmentRect(unittest.TestCase):
    def test_passes_straight_through(self):
        self.assertTrue(segment_intersects_rect((0, 5), (10, 5), (4, 0, 2, 10)))

    def test_clearly_avoids(self):
        self.assertFalse(segment_intersects_rect((0, 20), (10, 20), (4, 0, 2, 10)))

    def test_crosses_corner_region(self):
        self.assertTrue(segment_intersects_rect((0, 0), (10, 10), (4, 4, 2, 2)))

    def test_fully_inside(self):
        self.assertTrue(segment_intersects_rect((4.5, 4.5), (5.5, 5.5), (4, 4, 2, 2)))

    def test_grazes_edge_is_intersection(self):
        self.assertTrue(segment_intersects_rect((0, 10), (10, 10), (4, 0, 2, 10)))

    def test_just_misses(self):
        self.assertFalse(segment_intersects_rect((0, 10.001), (10, 10.001), (4, 0, 2, 10)))

    def test_zero_length_segment_inside(self):
        self.assertTrue(segment_intersects_rect((5, 5), (5, 5), (4, 4, 2, 2)))

    def test_zero_length_segment_outside(self):
        self.assertFalse(segment_intersects_rect((0, 0), (0, 0), (4, 4, 2, 2)))

    def test_vertical_segment_crossing(self):
        self.assertTrue(segment_intersects_rect((5, -5), (5, 15), (4, 0, 2, 10)))

    def test_segment_ending_inside(self):
        self.assertTrue(segment_intersects_rect((0, 5), (5, 5), (4, 0, 2, 10)))

    def test_parallel_above_no_touch(self):
        self.assertFalse(segment_intersects_rect((0, 30), (10, 30), (4, 0, 2, 10)))


class TestDistance(unittest.TestCase):
    def test_2d(self):
        self.assertAlmostEqual(distance_2d((0, 0), (3, 4)), 5.0)

    def test_2d_zero(self):
        self.assertEqual(distance_2d((1, 1), (1, 1)), 0.0)

    def test_3d(self):
        self.assertAlmostEqual(distance_3d((0, 0, 0), (1, 2, 2)), 3.0)

    def test_3d_vertical(self):
        self.assertAlmostEqual(distance_3d((0, 0, 10), (0, 0, 60)), 50.0)


if __name__ == "__main__":
    unittest.main()
