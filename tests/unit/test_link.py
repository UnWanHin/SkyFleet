"""可插拔链路模型测试。

核心命题（报告 §3.1）：链路质量必须随位置、建筑遮挡和飞行高度变化，
不能使用固定丢包率。飞行高度高于楼高时不应判遮挡。
"""
import unittest

from skyfleet.protocol.identity import Role
from skyfleet.world.building import Building
from skyfleet.world.link import GeometricOcclusionLinkModel
from skyfleet.world.node import Node

WALL = Building("B1", (40.0, 0.0, 20.0, 100.0), height=80.0)


def node(nid, x, y, z):
    return Node(nid, Role.SLAVE, (x, y, z), speed=10.0, energy=100.0)


def model(buildings=(), max_range=1000.0, min_quality=0.05):
    return GeometricOcclusionLinkModel(buildings, max_range=max_range,
                                       min_quality=min_quality)


class TestOcclusion(unittest.TestCase):
    def test_clear_line_is_not_obstructed(self):
        q = model([WALL]).evaluate(node("A", 0, 50, 100), node("B", 100, 50, 100), 0.0)
        self.assertFalse(q.obstructed)

    def test_line_through_building_is_obstructed(self):
        q = model([WALL]).evaluate(node("A", 0, 50, 50), node("B", 100, 50, 50), 0.0)
        self.assertTrue(q.obstructed)
        self.assertEqual(q.blocked_by, "B1")

    def test_flying_above_building_avoids_occlusion(self):
        q = model([WALL]).evaluate(node("A", 0, 50, 90), node("B", 100, 50, 90), 0.0)
        self.assertFalse(q.obstructed)

    def test_obstructed_quality_is_heavily_penalised(self):
        q = model([WALL]).evaluate(node("A", 0, 50, 50), node("B", 100, 50, 50), 0.0)
        self.assertLessEqual(q.quality, 0.1)

    def test_beyond_range_is_unusable(self):
        q = model([], max_range=50.0).evaluate(
            node("A", 0, 0, 50), node("B", 500, 0, 50), 0.0)
        self.assertEqual(q.quality, 0.0)

    def test_obstruction_wins_over_range(self):
        # 遮挡的惩罚优先于距离衰减：即使很近，被挡就是被挡
        q = model([WALL]).evaluate(node("A", 0, 50, 50), node("B", 100, 50, 50), 0.0)
        self.assertTrue(q.obstructed)
        self.assertEqual(q.quality, 0.05)

    def test_closer_links_are_better(self):
        m = model([])
        near = m.evaluate(node("A", 0, 0, 50), node("B", 20, 0, 50), 0.0)
        far = m.evaluate(node("A", 0, 0, 50), node("B", 400, 0, 50), 0.0)
        self.assertGreater(near.quality, far.quality)

    def test_quality_within_bounds(self):
        m = model([WALL])
        for d in (0, 10, 100, 400):
            q = m.evaluate(node("A", 0, 0, 50), node("B", d, 0, 50), 0.0).quality
            self.assertGreaterEqual(q, 0.0)
            self.assertLessEqual(q, 1.0)

    def test_quality_is_never_zero_within_range_when_clear(self):
        q = model([], max_range=1000.0).evaluate(
            node("A", 0, 0, 50), node("B", 900, 0, 50), 0.0)
        self.assertGreater(q.quality, 0.0)


class TestTrend(unittest.TestCase):
    def test_trend_is_zero_without_history(self):
        q = model([]).evaluate(node("A", 0, 0, 50), node("B", 20, 0, 50), 0.0)
        self.assertEqual(q.trend, 0.0)

    def test_trend_negative_when_quality_falls(self):
        m = model([])
        a = node("A", 0, 0, 50)
        m.evaluate(a, node("B", 20, 0, 50), 0.0)
        far = node("B", 450, 0, 50)
        q = m.evaluate(a, far, 1.0)
        q = m.evaluate(a, far, 2.0)
        self.assertLess(q.trend, 0.0)

    def test_trend_positive_when_quality_recovers(self):
        m = model([])
        a = node("A", 0, 0, 50)
        m.evaluate(a, node("B", 450, 0, 50), 0.0)
        near = node("B", 20, 0, 50)
        m.evaluate(a, near, 1.0)
        q = m.evaluate(a, near, 2.0)
        self.assertGreater(q.trend, 0.0)


class TestPluggability(unittest.TestCase):
    def test_model_satisfies_link_model_protocol(self):
        from skyfleet.world.link import LinkModel
        self.assertIsInstance(model([]), LinkModel)

    def test_custom_model_can_be_substituted(self):
        from skyfleet.world.link import LinkQuality

        class AlwaysBad:
            def evaluate(self, a, b, t):
                return LinkQuality(0.0, True, None, 0.0)

        q = AlwaysBad().evaluate(node("A", 0, 0, 50), node("B", 1, 0, 50), 0.0)
        self.assertEqual(q.quality, 0.0)


if __name__ == "__main__":
    unittest.main()
