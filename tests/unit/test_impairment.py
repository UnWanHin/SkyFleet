"""P1 损伤源的测试。

损伤源的作用是"让链路变差"，但**变差的方式**必须精确：
只在时间窗内、只在影响范围内、只在障碍物确实挡住视线时。
少一个条件，仿真就会在没人干扰的时候丢包——这种错误看起来像
"协议不健壮"，实际是注入器写错了，极难排查。
"""
import unittest

from skyfleet.kernel.config import SimConfig
from skyfleet.world.building import Building
from skyfleet.world.impairment import (
    ImpairedLinkModel, Interference, TemporaryObstacle,
    build_impairments, wrap_with_impairments,
)
from skyfleet.world.link import GeometricOcclusionLinkModel, LinkModel
from skyfleet.world.link_nlos import NlosDiffractionLinkModel
from skyfleet.world.node import Node, Role

BLOCK = Building("B0", (40.0, 0.0, 60.0, 200.0), 80.0)


def node(node_id, pos, role=Role.SLAVE):
    return Node(node_id=node_id, role=role, pos=pos, speed=10.0, energy=100.0)


def inner_model(buildings=(), kind="geometric"):
    if kind == "nlos":
        return NlosDiffractionLinkModel(list(buildings), max_range=1000.0,
                                        seed=1)
    return GeometricOcclusionLinkModel(list(buildings), max_range=1000.0)


class TestProtocolConformance(unittest.TestCase):
    def test_decorator_is_still_a_link_model(self):
        model = ImpairedLinkModel(inner_model())
        self.assertIsInstance(model, LinkModel)

    def test_wraps_both_inner_models(self):
        for kind in ("geometric", "nlos"):
            model = ImpairedLinkModel(inner_model(kind=kind))
            self.assertIsInstance(model, LinkModel)


class TestInterference(unittest.TestCase):
    def setUp(self):
        self.a = node("L", (0.0, 0.0, 140.0), Role.LEADER)
        self.b = node("S1", (200.0, 0.0, 140.0))
        self.jammer = Interference(id="J1", center=(100.0, 0.0),
                                   radius_m=150.0, t_start=10.0, t_end=20.0,
                                   extra_loss_db=10.0)

    def model(self, **kwargs):
        return ImpairedLinkModel(inner_model(), interferences=[self.jammer],
                                 **kwargs)

    def test_no_effect_before_the_window(self):
        base = inner_model().evaluate(self.a, self.b, 5.0).quality
        impaired = self.model().evaluate(self.a, self.b, 5.0).quality
        self.assertEqual(base, impaired)

    def test_no_effect_after_the_window(self):
        base = inner_model().evaluate(self.a, self.b, 25.0).quality
        impaired = self.model().evaluate(self.a, self.b, 25.0).quality
        self.assertEqual(base, impaired)

    def test_degrades_inside_the_window(self):
        base = inner_model().evaluate(self.a, self.b, 15.0).quality
        impaired = self.model().evaluate(self.a, self.b, 15.0).quality
        self.assertLess(impaired, base)

    def test_window_is_half_open(self):
        model = self.model()
        self.assertTrue(self.jammer.active(10.0))
        self.assertFalse(self.jammer.active(20.0))

    def test_no_effect_outside_the_radius(self):
        far_a = node("L2", (0.0, 900.0, 140.0))
        far_b = node("S9", (200.0, 900.0, 140.0))
        base = inner_model().evaluate(far_a, far_b, 15.0).quality
        impaired = self.model().evaluate(far_a, far_b, 15.0).quality
        self.assertEqual(base, impaired)

    def test_affects_a_link_with_only_one_endpoint_inside(self):
        """干扰抬高的是接收端噪声底，只要有一端在范围内就该受影响。"""
        model = self.model()
        self.assertEqual(model.extra_loss_db(self.a, self.b, 15.0), 10.0)

    def test_loss_maps_through_the_models_own_dynamic_range(self):
        model = self.model()
        # 10 dB 的损伤在 20 dB 量程上应当恰好削掉一半 quality
        base = inner_model().evaluate(self.a, self.b, 15.0).quality
        impaired = model.evaluate(self.a, self.b, 15.0).quality
        self.assertAlmostEqual(base - impaired, 0.5, places=6)

    def test_two_sources_accumulate(self):
        second = Interference(id="J2", center=(100.0, 0.0), radius_m=150.0,
                              t_start=10.0, t_end=20.0, extra_loss_db=5.0)
        model = ImpairedLinkModel(inner_model(),
                                  interferences=[self.jammer, second])
        self.assertEqual(model.extra_loss_db(self.a, self.b, 15.0), 15.0)

    def test_quality_never_goes_negative(self):
        huge = Interference(id="J3", center=(100.0, 0.0), radius_m=500.0,
                            t_start=0.0, t_end=1e9, extra_loss_db=999.0)
        model = ImpairedLinkModel(inner_model(), interferences=[huge])
        self.assertEqual(model.evaluate(self.a, self.b, 5.0).quality, 0.0)


class TestTemporaryObstacle(unittest.TestCase):
    def setUp(self):
        self.a = node("L", (0.0, 0.0, 40.0), Role.LEADER)
        self.b = node("S1", (200.0, 0.0, 40.0))
        self.crane = TemporaryObstacle(
            id="T1", rect=(90.0, -50.0, 20.0, 100.0), height=120.0,
            t_start=30.0, t_end=60.0)

    def model(self):
        return ImpairedLinkModel(inner_model(), obstacles=[self.crane])

    def test_blocks_only_inside_its_window(self):
        model = self.model()
        self.assertTrue(self.crane.blocks(self.a, self.b, 40.0))
        self.assertFalse(self.crane.blocks(self.a, self.b, 10.0))
        self.assertFalse(self.crane.blocks(self.a, self.b, 70.0))

    def test_obstruction_is_reported_with_its_cause(self):
        result = self.model().evaluate(self.a, self.b, 40.0)
        self.assertTrue(result.obstructed)
        self.assertEqual(result.blocked_by, "T1")

    def test_quality_drops_while_it_stands(self):
        model = self.model()
        before = model.evaluate(self.a, self.b, 10.0).quality
        during = model.evaluate(self.a, self.b, 40.0).quality
        after = model.evaluate(self.a, self.b, 70.0).quality
        self.assertLess(during, before)
        self.assertEqual(before, after)

    def test_ignored_when_lower_than_both_endpoints(self):
        low = TemporaryObstacle(id="T2", rect=(90.0, -50.0, 20.0, 100.0),
                                height=30.0, t_start=0.0, t_end=1e9)
        self.assertFalse(low.blocks(self.a, self.b, 10.0))

    def test_decorator_preserves_the_inner_models_own_obstruction(self):
        model = ImpairedLinkModel(inner_model([BLOCK]),
                                  obstacles=[self.crane])
        result = model.evaluate(node("L", (0.0, 0.0, 40.0)),
                                node("S1", (200.0, 0.0, 40.0)), 5.0)
        self.assertTrue(result.obstructed)
        self.assertEqual(result.blocked_by, "B0")


class TestDeterminism(unittest.TestCase):
    def test_repeat_evaluation_is_identical(self):
        jammer = Interference(id="J", center=(100.0, 0.0), radius_m=500.0,
                              t_start=0.0, t_end=1e9, extra_loss_db=8.0)
        model = ImpairedLinkModel(inner_model(kind="nlos"),
                                  interferences=[jammer])
        a, b = node("L", (0.0, 0.0, 100.0)), node("S1", (300.0, 0.0, 100.0))
        first = model.evaluate(a, b, 7.0).quality
        for _ in range(4):
            self.assertEqual(model.evaluate(a, b, 7.0).quality, first)

    def test_order_independence(self):
        jammer = Interference(id="J", center=(100.0, 0.0), radius_m=500.0,
                              t_start=0.0, t_end=1e9, extra_loss_db=8.0)
        a = node("L", (0.0, 0.0, 100.0))
        b = node("S1", (300.0, 0.0, 100.0))
        c = node("S2", (100.0, 400.0, 100.0))

        m1 = ImpairedLinkModel(inner_model(kind="nlos"),
                               interferences=[jammer])
        ab = m1.evaluate(a, b, 9.0).quality
        ac = m1.evaluate(a, c, 9.0).quality

        m2 = ImpairedLinkModel(inner_model(kind="nlos"),
                               interferences=[jammer])
        ac2 = m2.evaluate(a, c, 9.0).quality
        ab2 = m2.evaluate(a, b, 9.0).quality
        self.assertEqual((ab, ac), (ab2, ac2))


class TestConfigWiring(unittest.TestCase):
    def test_empty_config_returns_the_inner_model_untouched(self):
        config = SimConfig()
        base = inner_model()
        self.assertIs(wrap_with_impairments(base, config), base)

    def test_parses_interferences_and_obstacles(self):
        config = SimConfig(interferences=[
            {"id": "J1", "x": 100.0, "y": 100.0, "radius": 200.0,
             "t_start": 5.0, "t_end": 50.0, "extra_loss_db": 15.0},
        ], obstacles=[
            {"id": "T1", "x": 10.0, "y": 10.0, "w": 20.0, "h": 20.0,
             "height": 100.0, "t_start": 0.0, "t_end": 30.0},
        ])
        interferences, obstacles = build_impairments(config)
        self.assertEqual(len(interferences), 1)
        self.assertEqual(interferences[0].radius_m, 200.0)
        self.assertEqual(interferences[0].extra_loss_db, 15.0)
        self.assertEqual(len(obstacles), 1)
        self.assertEqual(obstacles[0].rect, (10.0, 10.0, 20.0, 20.0))

    def test_defaults_time_window_to_open_ended(self):
        config = SimConfig(interferences=[
            {"id": "J", "x": 0.0, "y": 0.0, "radius": 10.0}])
        interferences, _ = build_impairments(config)
        self.assertEqual(interferences[0].t_start, 0.0)
        self.assertTrue(interferences[0].active(1e9))

    def test_wrapping_produces_a_decorator_when_configured(self):
        config = SimConfig(interferences=[
            {"id": "J", "x": 0.0, "y": 0.0, "radius": 10.0}])
        wrapped = wrap_with_impairments(inner_model(), config)
        self.assertIsInstance(wrapped, ImpairedLinkModel)


if __name__ == "__main__":
    unittest.main()
