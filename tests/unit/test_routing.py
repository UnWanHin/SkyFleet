"""路由迟滞测试。

核心命题（报告 §3.2）：不能因为一次探测结果变化就立即切换路径。
城市峡谷会造成链路质量反复抖动，无迟滞的路由会产生震荡与额外重传。
"""
import unittest

from skyfleet.protocol.routing import RoutingManager, RoutingParams

P = RoutingParams(h_up=0.75, h_down=0.35, t_hold=1.0)


class TestHysteresis(unittest.TestCase):
    def setUp(self):
        self.r = RoutingManager(P, parent=None)

    def test_no_immediate_switch_on_first_good_sample(self):
        self.assertFalse(self.r.consider("R1", 0.9, 0.0))
        self.assertIsNone(self.r.parent)

    def test_no_switch_before_hold_expires(self):
        self.r.consider("R1", 0.9, 0.0)
        self.assertFalse(self.r.consider("R1", 0.9, 0.5))
        self.assertIsNone(self.r.parent)

    def test_switches_after_hold_expires(self):
        self.r.consider("R1", 0.9, 0.0)
        self.assertTrue(self.r.consider("R1", 0.9, 1.0))
        self.assertEqual(self.r.parent, "R1")

    def test_switch_bumps_route_version(self):
        self.r.consider("R1", 0.9, 0.0)
        self.r.consider("R1", 0.9, 1.0)
        self.assertEqual(self.r.route_version, 1)
        self.assertEqual(self.r.switch_count, 1)

    def test_quality_in_hysteresis_band_does_not_switch(self):
        self.r.parent = "R1"
        self.assertFalse(self.r.consider("R2", 0.5, 0.0))
        self.assertFalse(self.r.consider("R2", 0.5, 5.0))
        self.assertEqual(self.r.parent, "R1")

    def test_candidate_reset_when_sample_degrades(self):
        self.r.consider("R1", 0.9, 0.0)
        self.r.consider("R1", 0.5, 0.5)        # 掉回迟滞带 -> 候选清零
        self.assertFalse(self.r.consider("R1", 0.9, 1.5))
        self.assertIsNone(self.r.parent)

    def test_no_switch_when_already_parent(self):
        self.r.parent = "R1"
        self.assertFalse(self.r.consider("R1", 0.95, 0.0))
        self.assertEqual(self.r.switch_count, 0)


class TestDegradedParent(unittest.TestCase):
    def setUp(self):
        self.r = RoutingManager(P, parent="R1")

    def test_leave_degraded_parent_after_hold(self):
        self.assertFalse(self.r.consider("R1", 0.1, 0.0))
        self.assertTrue(self.r.consider("R1", 0.1, 1.0))
        self.assertIsNone(self.r.parent)

    def test_parent_recovering_stays(self):
        self.r.consider("R1", 0.1, 0.0)
        self.r.consider("R1", 0.9, 0.5)      # 恢复 -> 候选清零
        self.assertFalse(self.r.consider("R1", 0.1, 1.0))
        self.assertEqual(self.r.parent, "R1")


class TestFlapping(unittest.TestCase):
    def test_quick_reversal_counts_as_flap(self):
        r = RoutingManager(P, parent=None)
        r.consider("R1", 0.9, 0.0)
        r.consider("R1", 0.9, 1.0)           # 切到 R1
        r.consider("R1", 0.1, 2.0)
        r.consider("R1", 0.1, 3.0)           # 3s 内切走 -> 记一次震荡
        self.assertEqual(r.switch_count, 2)
        self.assertEqual(r.flap_count, 1)

    def test_slow_switch_is_not_a_flap(self):
        r = RoutingManager(P, parent=None, flap_window=1.0)
        r.consider("R1", 0.9, 0.0)
        r.consider("R1", 0.9, 1.0)
        r.consider("R1", 0.1, 50.0)
        r.consider("R1", 0.1, 51.0)
        self.assertEqual(r.flap_count, 0)


if __name__ == "__main__":
    unittest.main()
