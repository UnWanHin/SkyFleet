"""P1 位置源的测试。

定位误差与失锁的区别必须清楚：
- **漂移**是"位置不准"，但仍然是新数据；
- **失锁**是"没有新数据"，只能沿用旧定位，因此会**过期**。
报告 §2.2.6 要的是后者触发悬停。把两者混为一谈，就会在定位精度差时
误触发悬停（可用性下降），或在失锁时以为位置还新鲜（安全性下降）。
"""
import unittest

from skyfleet.kernel.config import SimConfig
from skyfleet.world.geometry import distance_3d
from skyfleet.world.node import Node, Role
from skyfleet.world.position import (
    GnssPositionSource, IdealPositionSource, build_position_source,
)


def node(node_id="S1", pos=(100.0, 100.0, 90.0)):
    return Node(node_id=node_id, role=Role.SLAVE, pos=pos, speed=10.0,
                energy=100.0)


class TestIdealSource(unittest.TestCase):
    def test_reports_the_exact_position(self):
        n = node()
        status = IdealPositionSource().report(n, 5.0)
        self.assertEqual(status.pos, n.pos)
        self.assertFalse(status.stale)
        self.assertEqual(status.age_s, 0.0)


class TestDrift(unittest.TestCase):
    def setUp(self):
        self.source = GnssPositionSource(seed=1, drift_m=5.0)

    def test_position_is_offset_but_close(self):
        n = node()
        status = self.source.report(n, 3.0)
        self.assertNotEqual(status.pos, n.pos)
        self.assertLess(distance_3d(status.pos, n.pos), 5.0 * 1.2)

    def test_drift_is_constant_inside_a_time_bin(self):
        """同一时间片内位置必须稳定：每次调用重抽等于人为制造高频噪声。"""
        n = node()
        first = self.source.report(n, 3.0).pos
        self.assertEqual(self.source.report(n, 3.5).pos, first)
        self.assertEqual(self.source.report(n, 3.99).pos, first)

    def test_drift_changes_across_bins(self):
        n = node()
        self.assertNotEqual(self.source.report(n, 3.0).pos,
                            self.source.report(n, 4.0).pos)

    def test_drift_does_not_make_position_stale(self):
        status = self.source.report(node(), 3.0)
        self.assertFalse(status.stale)
        self.assertEqual(status.age_s, 0.0)

    def test_deterministic_across_instances(self):
        n = node()
        a = GnssPositionSource(seed=1, drift_m=5.0).report(n, 3.0).pos
        b = GnssPositionSource(seed=1, drift_m=5.0).report(n, 3.0).pos
        self.assertEqual(a, b)

    def test_different_seed_differs(self):
        n = node()
        a = GnssPositionSource(seed=1, drift_m=5.0).report(n, 3.0).pos
        b = GnssPositionSource(seed=2, drift_m=5.0).report(n, 3.0).pos
        self.assertNotEqual(a, b)

    def test_zero_drift_is_exact(self):
        source = GnssPositionSource(seed=1, drift_m=0.0)
        n = node()
        self.assertEqual(source.report(n, 3.0).pos, n.pos)


class TestOutage(unittest.TestCase):
    def setUp(self):
        self.source = GnssPositionSource(
            seed=1, drift_m=0.0, outages=[("S1", 10.0, 20.0)])

    def test_position_freezes_during_the_outage(self):
        n = node()
        before = self.source.report(n, 9.0).pos
        during = self.source.report(n, 15.0).pos
        self.assertEqual(during, before)

    def test_outage_marks_the_fix_stale_with_growing_age(self):
        n = node()
        first = self.source.report(n, 10.0)
        later = self.source.report(n, 17.0)
        self.assertTrue(first.stale)
        self.assertTrue(later.stale)
        self.assertGreater(later.age_s, first.age_s)
        self.assertAlmostEqual(later.age_s, 7.0)

    def test_recovers_after_the_outage(self):
        status = self.source.report(node(), 20.0)
        self.assertFalse(status.stale)
        self.assertEqual(status.age_s, 0.0)

    def test_boundary_is_half_open(self):
        self.assertTrue(self.source.report(node(), 10.0).stale)
        self.assertFalse(self.source.report(node(), 20.0).stale)

    def test_other_nodes_are_unaffected(self):
        other = node("S2")
        self.assertFalse(self.source.report(other, 15.0).stale)

    def test_multiple_spans_for_one_node(self):
        source = GnssPositionSource(
            seed=1, outages=[("S1", 10.0, 12.0), ("S1", 30.0, 40.0)])
        self.assertTrue(source.report(node(), 11.0).stale)
        self.assertFalse(source.report(node(), 20.0).stale)
        self.assertTrue(source.report(node(), 35.0).stale)

    def test_same_time_gives_the_same_answer(self):
        n = node()
        first = self.source.report(n, 15.0)
        for _ in range(3):
            self.assertEqual(self.source.report(n, 15.0), first)


class TestConfigWiring(unittest.TestCase):
    def test_defaults_to_ideal(self):
        self.assertIsInstance(build_position_source(SimConfig()),
                              IdealPositionSource)

    def test_drift_enables_gnss(self):
        self.assertIsInstance(
            build_position_source(SimConfig(gnss_drift_m=3.0)),
            GnssPositionSource)

    def test_outage_alone_enables_gnss(self):
        source = build_position_source(
            SimConfig(gnss_outages=(("S1", 1.0, 2.0),)))
        self.assertIsInstance(source, GnssPositionSource)
        self.assertTrue(source.report(node(), 1.5).stale)

    def test_outage_triples_are_coerced_from_json(self):
        from skyfleet.kernel.config import coerce_overrides
        out = coerce_overrides({"gnss_outages": [["S1", 1, 2.5]]})
        self.assertEqual(out["gnss_outages"], (("S1", 1.0, 2.5),))


if __name__ == "__main__":
    unittest.main()
