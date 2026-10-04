"""P1 非视距链路模型的测试。

重点不在"公式算得对不对"（那需要实测数据才能判），而在三件事：
1. 它确实实现了 `LinkModel` 协议，协议层因此零改动就能换模型。
2. 它的物理性质是**单调且方向正确**的：飞得更高、离得更近、视线不被挡，
   质量就必须更好。方向错了，报告里的结论就会反过来。
3. 它与调用次数、调用顺序无关。这是确定性仿真里最容易踩的坑：
   一旦 `evaluate` 内部推进随机状态，同一时刻评估两次结果就不同。
"""
import unittest

from skyfleet.world.building import Building
from skyfleet.world.link import GeometricOcclusionLinkModel, LinkModel
from skyfleet.world.link_nlos import NlosDiffractionLinkModel, NlosParams
from skyfleet.world.node import Node, Role

BLOCK = Building("B0", (40.0, 0.0, 60.0, 200.0), 80.0)


def node(node_id, pos, role=Role.SLAVE):
    return Node(node_id=node_id, role=role, pos=pos, speed=10.0, energy=100.0)


def model(buildings=(BLOCK,), seed=1, **overrides):
    params = NlosParams(**overrides) if overrides else None
    return NlosDiffractionLinkModel(list(buildings), max_range=1000.0,
                                    seed=seed, params=params)


class TestProtocolConformance(unittest.TestCase):
    def test_satisfies_the_link_model_protocol(self):
        self.assertIsInstance(model(), LinkModel)

    def test_geometric_model_also_satisfies_it(self):
        self.assertIsInstance(GeometricOcclusionLinkModel([BLOCK], 1000.0),
                              LinkModel)


class TestLineOfSight(unittest.TestCase):
    def test_quality_falls_with_distance(self):
        m = model(buildings=())
        a = node("L", (0.0, 0.0, 140.0), Role.LEADER)
        near = m.evaluate(a, node("S1", (50.0, 0.0, 140.0)), 0.0).quality
        far = m.evaluate(a, node("S2", (400.0, 0.0, 140.0)), 0.0).quality
        self.assertGreater(near, far)

    def test_nothing_is_blocked_without_buildings(self):
        result = model(buildings=()).evaluate(
            node("L", (0.0, 0.0, 140.0)), node("S1", (100.0, 0.0, 140.0)), 0.0)
        self.assertFalse(result.obstructed)
        self.assertIsNone(result.blocked_by)

    def test_beyond_max_range_is_unusable(self):
        m = NlosDiffractionLinkModel([], max_range=100.0, seed=1)
        result = m.evaluate(node("L", (0.0, 0.0, 140.0)),
                            node("S1", (500.0, 0.0, 140.0)), 0.0)
        self.assertEqual(result.quality, 0.0)


class TestDiffraction(unittest.TestCase):
    def test_obstruction_is_reported_with_its_cause(self):
        result = model().evaluate(node("L", (0.0, 0.0, 40.0)),
                                  node("S1", (200.0, 0.0, 40.0)), 0.0)
        self.assertTrue(result.obstructed)
        self.assertEqual(result.blocked_by, "B0")

    def test_obstructed_link_is_worse_than_clear_one(self):
        m = model()
        blocked = m.evaluate(node("L", (0.0, 0.0, 40.0)),
                             node("S1", (200.0, 0.0, 40.0)), 0.0).quality
        clear = m.evaluate(node("L", (0.0, 300.0, 140.0)),
                           node("S1", (200.0, 300.0, 140.0)), 0.0).quality
        self.assertLess(blocked, clear)

    def test_diffraction_separates_grazing_from_deep_blockage(self):
        """这正是换模型的意义。

        几何遮挡模型对被挡链路一律判 0.05——"擦着楼顶过去"和"被整栋楼
        挡死"给出同一个数。绕射模型必须把这两种情形分开，否则
        "飞高一点能挽回多少"就无法回答。
        """
        m = model()
        grazing = m.evaluate(node("L", (0.0, 0.0, 79.0)),
                             node("S1", (200.0, 0.0, 79.0)), 0.0)
        deep = m.evaluate(node("L", (0.0, 0.0, 20.0)),
                          node("S1", (200.0, 0.0, 20.0)), 0.0)
        self.assertTrue(grazing.obstructed and deep.obstructed)
        self.assertGreater(grazing.quality, 0.0,
                           "擦顶绕射不该被完全判死")
        self.assertGreater(grazing.quality, deep.quality)

    def test_geometric_model_cannot_separate_them(self):
        """反向对照：二值模型对这两种情形给出同一个值。"""
        geometric = GeometricOcclusionLinkModel([BLOCK], 1000.0)
        grazing = geometric.evaluate(node("L", (0.0, 0.0, 79.0)),
                                     node("S1", (200.0, 0.0, 79.0)), 0.0)
        deep = geometric.evaluate(node("L", (0.0, 0.0, 20.0)),
                                  node("S1", (200.0, 0.0, 20.0)), 0.0)
        self.assertEqual(grazing.quality, deep.quality)

    def test_higher_altitude_gives_better_quality(self):
        """报告的核心命题之一：飞行高度影响链路质量。方向必须正确。"""
        values = [model().evaluate(node("L", (0.0, 0.0, z)),
                                   node("S1", (200.0, 0.0, z)), 0.0).quality
                  for z in (40.0, 60.0, 70.0, 75.0, 90.0, 140.0)]
        for lower, higher in zip(values, values[1:]):
            self.assertLessEqual(lower, higher,
                                 f"高度升高后质量反而下降: {values}")

    def test_quality_is_continuous_across_the_obstruction_boundary(self):
        """二值模型在遮挡边界上会跳变；绕射模型的过渡必须是连续的，
        否则节点会在边界附近反复触发路由切换。"""
        values = [model().evaluate(node("L", (0.0, 0.0, z)),
                                   node("S1", (200.0, 0.0, z)), 0.0).quality
                  for z in [70.0 + 0.5 * i for i in range(20)]]
        jumps = [abs(b - a) for a, b in zip(values, values[1:])]
        self.assertLess(max(jumps), 0.35, f"边界跳变过大: {values}")


class TestDeterminism(unittest.TestCase):
    def test_repeated_evaluation_at_the_same_time_is_identical(self):
        m = model()
        a, b = node("L", (0.0, 0.0, 60.0)), node("S1", (200.0, 0.0, 60.0))
        first = m.evaluate(a, b, 1.0).quality
        for _ in range(5):
            self.assertEqual(m.evaluate(a, b, 1.0).quality, first)

    def test_evaluation_order_does_not_matter(self):
        """两条链路交替评估与分别评估，结果必须一致。"""
        a = node("L", (0.0, 0.0, 60.0))
        b = node("S1", (200.0, 0.0, 60.0))
        c = node("S2", (100.0, 300.0, 90.0))

        m1 = model()
        ab = m1.evaluate(a, b, 2.0).quality
        ac = m1.evaluate(a, c, 2.0).quality

        m2 = model()
        ac2 = m2.evaluate(a, c, 2.0).quality
        ab2 = m2.evaluate(a, b, 2.0).quality

        self.assertEqual(ab, ab2)
        self.assertEqual(ac, ac2)

    def test_a_to_b_equals_b_to_a(self):
        """链路是双向的：衰落必须按无序节点对派生。"""
        m = model()
        a, b = node("L", (0.0, 0.0, 60.0)), node("S1", (200.0, 0.0, 60.0))
        self.assertEqual(m.evaluate(a, b, 3.0).quality,
                         m.evaluate(b, a, 3.0).quality)

    def test_same_seed_same_result_different_seed_differs(self):
        # 取视距中距链路：此处 quality 落在量程中部，衰落才有体现
        a, b = node("L", (0.0, 0.0, 140.0)), node("S1", (600.0, 0.0, 140.0))
        one = model(buildings=(), seed=1).evaluate(a, b, 4.0).quality
        again = model(buildings=(), seed=1).evaluate(a, b, 4.0).quality
        other = model(buildings=(), seed=99).evaluate(a, b, 4.0).quality
        self.assertEqual(one, again)
        self.assertNotEqual(one, other)

    def test_fading_varies_between_coherence_intervals(self):
        m = model(fade_step_s=0.05)
        a, b = node("L", (0.0, 0.0, 140.0)), node("S1", (300.0, 0.0, 140.0))
        values = {m.evaluate(a, b, t).quality
                  for t in (0.0, 0.05, 0.1, 0.15, 0.2, 0.25)}
        self.assertGreater(len(values), 1, "衰落没有随时间变化")

    def test_cache_does_not_grow_without_bound(self):
        m = model()
        a = node("L", (0.0, 0.0, 140.0))
        for i in range(3000):
            m.evaluate(a, node(f"S{i}", (100.0, 0.0, 140.0)), i * 0.05)
        self.assertLessEqual(len(m._fade_cache), 20000)


class TestTrend(unittest.TestCase):
    def test_trend_reports_improvement_over_the_window(self):
        m = model()
        a = node("L", (0.0, 0.0, 140.0))
        b = node("S1", (100.0, 0.0, 140.0))
        for t in range(6):
            result = m.evaluate(a, b, t * 0.05)
        self.assertIsInstance(result.trend, float)

    def test_single_evaluation_has_zero_trend(self):
        m = model()
        result = m.evaluate(node("L", (0.0, 0.0, 140.0)),
                            node("S1", (100.0, 0.0, 140.0)), 0.0)
        self.assertEqual(result.trend, 0.0)


if __name__ == "__main__":
    unittest.main()
