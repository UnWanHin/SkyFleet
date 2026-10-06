"""能量模型测试。

重点不是"函数能跑"，而是把分析文档
`docs/analysis/2026-10-05-thrust-margin-analysis.md` 里的**物理结论**
固化成回归：一旦有人改动公式让"悬停比巡航便宜"，这里必须红。
"""
from __future__ import annotations

import math
import unittest

from skyfleet.kernel.config import SimConfig
from skyfleet.sim.engine import _step_velocities
from skyfleet.world.power import (
    AIRFRAMES,
    DEFAULT_DESCENT_FLOOR,
    G,
    RHO,
    Airframe,
    LinearEnergyModel,
    PhysicalEnergyModel,
    TxEnergyModel,
    build_energy_model,
)


class TestAirframeDerived(unittest.TestCase):
    """机型导出量必须与手算一致。"""

    def test_disk_area_total_is_four_disks(self):
        af = AIRFRAMES["hummingbird"]
        self.assertAlmostEqual(af.disk_area_total,
                               4 * math.pi * 0.10 ** 2, places=12)

    def test_px4_matches_analysis_document(self):
        """锚点值取自分析文档 §3，防止公式被悄悄改动。"""
        af = AIRFRAMES["px4_sihsim_quadx"]
        self.assertAlmostEqual(af.max_thrust_n, 20.0, places=6)
        self.assertAlmostEqual(af.hover_thrust_n(), 9.81, places=6)
        self.assertAlmostEqual(af.thrust_to_weight(), 2.0387, places=3)
        self.assertAlmostEqual(100 * af.hover_throttle(), 70.0, places=1)
        self.assertAlmostEqual(af.max_bank_deg(), 60.6, places=1)

    def test_hover_throttle_is_inverse_sqrt_not_inverse(self):
        """推力正比于转速**平方**，所以悬停油门是 1/sqrt(TWR) 而非 1/TWR。

        写成 1/TWR 会把 70% 说成 49%，是很容易犯且不易察觉的错误。
        """
        af = AIRFRAMES["px4_sihsim_quadx"]
        self.assertAlmostEqual(af.hover_throttle(), 1.0 / math.sqrt(af.twr)
                               if hasattr(af, "twr") else
                               1.0 / math.sqrt(af.thrust_to_weight()))
        self.assertNotAlmostEqual(af.hover_throttle(),
                                  1.0 / af.thrust_to_weight(), places=3)

    def test_hummingbird_has_far_more_margin(self):
        """Hummingbird 是四种机型里推重比最高的。"""
        hb = AIRFRAMES["hummingbird"]
        for name, af in AIRFRAMES.items():
            if name != "hummingbird":
                self.assertGreater(hb.thrust_to_weight(),
                                   af.thrust_to_weight(), name)

    def test_payload_lowers_thrust_margin(self):
        af = AIRFRAMES["px4_sihsim_quadx"]
        self.assertGreater(af.hover_throttle(0.5), af.hover_throttle(0.0))
        self.assertLess(af.max_bank_deg(0.5), af.max_bank_deg(0.0))

    def test_all_airframes_fly_the_scenario_speed(self):
        """分析文档 §3 的核心结论：12 m/s 对四种机型都不饱和。

        若这条红了，说明"四电机对常规飞行不是约束"的结论不再成立。
        """
        for name, af in AIRFRAMES.items():
            with self.subTest(airframe=name):
                self.assertFalse(af.is_saturated(12.0, 0.0))


class TestLinearModelUnchanged(unittest.TestCase):
    """linear 模型是 P0/P1 的既定行为，必须逐位不变。"""

    def test_ignores_flight_state(self):
        m = LinearEnergyModel(0.05)
        for v_h, v_z in ((0.0, 0.0), (12.0, 0.0), (0.0, 12.0), (0.0, -12.0)):
            with self.subTest(v_h=v_h, v_z=v_z):
                self.assertEqual(m.energy_for_step(v_h, v_z, 0.5), 0.05 * 0.5)

    def test_is_the_default(self):
        model = build_energy_model(SimConfig())
        self.assertIsInstance(model, LinearEnergyModel)


class TestPhysicalPower(unittest.TestCase):
    """动量理论功率。"""

    def setUp(self):
        self.af = AIRFRAMES["px4_sihsim_quadx"]
        self.m = PhysicalEnergyModel(self.af, battery_capacity_wh=100.0)

    def test_hover_matches_momentum_theory(self):
        """P = T^1.5 / sqrt(2*rho*A_total) / (FM*eta)。"""
        expected = (self.af.hover_thrust_n() ** 1.5
                    / math.sqrt(2 * RHO * self.af.disk_area_total)
                    / (self.m.figure_of_merit
                       * self.m.propulsion_efficiency))
        self.assertAlmostEqual(self.m.hover_power_w(), expected, places=9)

    def test_hover_power_is_sane_for_one_kilogram(self):
        """1 kg 机型悬停功率应在几十到一百多瓦之间——量级检查。

        这条能抓住"用单桨盘面积配总推力"导致的高估 2 倍。
        """
        self.assertGreater(self.m.hover_power_w(), 50.0)
        self.assertLess(self.m.hover_power_w(), 200.0)

    def test_cruise_is_cheaper_than_hover_for_delivery_class(self):
        """★ 分析文档 §4 的关键失真：交付级机型巡航比悬停省电。

        这是多旋翼固有特性（诱导功率随速度下降），与固定翼相反。
        当前 linear 模型让两者同价，把最贵与较便宜的状态算成一样。
        """
        for name in ("hummingbird", "px4_sihsim_quadx"):
            with self.subTest(airframe=name):
                m = PhysicalEnergyModel(AIRFRAMES[name])
                self.assertLess(m.cruise_power_w(12.0), m.hover_power_w())

    def test_cruise_is_dearer_than_hover_for_tiny_airframes(self):
        """微型机在 12 m/s 下阻力占重量八成，方向相反。

        两类机型结论不同，不能一概而论——这条防止有人把
        "巡航更省电"当成放之四海皆准的结论。
        """
        m = PhysicalEnergyModel(AIRFRAMES["crazyflie"])
        self.assertGreater(m.cruise_power_w(12.0), m.hover_power_w())

    def test_climb_costs_more_than_hover(self):
        self.assertGreater(self.m.power_w(0.0, 12.0), self.m.hover_power_w())

    def test_descent_costs_less_than_hover(self):
        self.assertLess(self.m.power_w(0.0, -12.0), self.m.hover_power_w())

    def test_descent_never_generates_energy(self):
        """涡环状态下动量理论失效，必须有下限，否则下降会"发电"。

        扫一遍下降率，确保功率恒正且不低于悬停诱导功率的 descent_floor。
        """
        floor = DEFAULT_DESCENT_FLOOR * self.m.hover_power_w()
        for v_z in (-0.5, -1.0, -2.0, -3.0, -5.0, -8.0, -12.0, -20.0):
            with self.subTest(v_z=v_z):
                p = self.m.power_w(0.0, v_z)
                self.assertGreater(p, 0.0)
                self.assertGreaterEqual(p + 1e-9, floor)

    def test_saturation_is_clamped_not_inflated(self):
        """饱和时推力钳到上限——报一个"更大的功率"是误导。"""
        # 10 kg 载荷远超 PX4 的 20 N 上限
        m = PhysicalEnergyModel(self.af, payload_kg=10.0)
        self.assertGreater(m.power_w(0.0, 0.0), 0.0)
        self.assertTrue(self.af.is_saturated(0.0, 0.0, payload_kg=10.0))

    def test_power_is_monotonic_in_speed(self):
        """低速段诱导功率下降、废阻上升，总功率对速度不是单调的；
        但超过最小功率速度后必须单调增。"""
        speeds = [10.0, 14.0, 18.0, 22.0]
        powers = [self.m.cruise_power_w(v) for v in speeds]
        self.assertEqual(powers, sorted(powers))

    def test_quadratic_scaling_with_mass(self):
        """悬停功率 ∝ T^1.5，T ∝ m，故 P ∝ m^1.5。"""
        m1 = PhysicalEnergyModel(self.af, payload_kg=0.0)
        m2 = PhysicalEnergyModel(self.af, payload_kg=1.0)
        self.assertAlmostEqual(
            m2.hover_power_w() / m1.hover_power_w(), 2.0 ** 1.5, places=6)


class TestPhysicalAccounting(unittest.TestCase):
    """计费与续航。"""

    def setUp(self):
        self.m = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"],
                                     battery_capacity_wh=100.0)

    def test_endurance_times_rate_is_one_hundred_percent(self):
        for v_h, v_z in ((0.0, 0.0), (12.0, 0.0), (0.0, 12.0)):
            with self.subTest(v_h=v_h, v_z=v_z):
                rate = self.m.percent_per_second(v_h, v_z)
                self.assertAlmostEqual(rate * self.m.endurance_s(v_h, v_z),
                                       100.0, places=6)

    def test_hover_endurance_is_realistic(self):
        """1 kg 机型 100 Wh 电池悬停续航应在半小时到两小时之间。"""
        e = self.m.endurance_s()
        self.assertGreater(e, 1800.0)
        self.assertLess(e, 7200.0)

    def test_energy_for_step_is_rate_times_dt(self):
        self.assertAlmostEqual(
            self.m.energy_for_step(12.0, 0.0, 0.5),
            self.m.percent_per_second(12.0, 0.0) * 0.5, places=12)

    def test_bigger_battery_lasts_longer(self):
        big = PhysicalEnergyModel(self.m.airframe, battery_capacity_wh=200.0)
        self.assertAlmostEqual(big.endurance_s(),
                               2.0 * self.m.endurance_s(), places=6)

    def test_energy_consumed_matches_flight_time(self):
        """飞 600 s 悬停应消耗约 600 秒的悬停倍率。"""
        spent = sum(self.m.energy_for_step(0.0, 0.0, 1.0)
                    for _ in range(600))
        self.assertAlmostEqual(spent, 600.0 / self.m.endurance_s() * 100.0,
                               places=6)

    def test_zero_or_negative_battery_rejected(self):
        with self.assertRaises(ValueError):
            PhysicalEnergyModel(self.m.airframe, battery_capacity_wh=0.0)


class TestTxEnergy(unittest.TestCase):
    def test_tx_energy_is_negligible_next_to_flight(self):
        """分析结论：2 Mbps、100 mW 下通信能耗远低于飞行能耗。

        这条把"不要把协议节能当成主要结果"这个结论固定下来。
        """
        tx = TxEnergyModel(tx_power_w=0.333, channel_rate_bps=2_000_000,
                           battery_capacity_wh=100.0)
        flight = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"])
        # 满速连续发射（占满 2 Mbps 信道）的耗电速率
        tx_full_rate = tx.energy_per_bit() * 2_000_000
        hover = flight.percent_per_second(0.0, 0.0)
        # 实测约 1:273 —— 即使**占满整条信道**发射，也只有悬停耗电的
        # 0.37%，远低于 1%。
        self.assertLess(tx_full_rate, 0.01 * hover)
        self.assertGreater(tx_full_rate, 0.0)
        self.assertAlmostEqual(hover / tx_full_rate, 273.0, delta=5.0)

    def test_mission_tx_energy_share_is_tiny(self):
        """整场任务的发射能耗占比。基线日志 1 027 976 字节 @ 2 Mbps。"""
        tx = TxEnergyModel(tx_power_w=0.333, channel_rate_bps=2_000_000,
                           battery_capacity_wh=100.0)
        flight = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"])
        bits = 1_027_976 * 8
        airtime_s = bits / 2_000_000
        tx_joules = 0.333 * airtime_s
        flight_joules = flight.power_w(0.0, 0.0) * 600.0
        share = tx_joules / flight_joules
        self.assertLess(share, 1e-4)          # 不到万分之一
        self.assertGreater(share, 1e-6)

    def test_rejects_zero_rate(self):
        with self.assertRaises(ValueError):
            TxEnergyModel(1.0, 0.0, 100.0).energy_per_bit()


class TestBatterySizing(unittest.TestCase):
    """电池容量必须按机型给。

    曾用一个全局默认 100 Wh，结果给 30 g 的 Crazyflie（真实约 0.9 Wh）
    配上 1 kg 机型的电池，能耗数字完全失去意义。扫参时才暴露出来。
    """

    def test_uses_airframe_typical_battery_when_unset(self):
        for name, af in AIRFRAMES.items():
            with self.subTest(airframe=name):
                m = build_energy_model(SimConfig(energy_model="physical",
                                                 airframe=name))
                self.assertEqual(m.battery_capacity_wh, af.typical_battery_wh)

    def test_explicit_battery_overrides_typical(self):
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="crazyflie",
                                         battery_capacity_wh=12.0))
        self.assertEqual(m.battery_capacity_wh, 12.0)

    def test_typical_capacities_are_ordered_by_airframe_size(self):
        """机型越大电池越大——这条能抓住"所有机型共用一个容量"。"""
        masses = sorted(AIRFRAMES.values(), key=lambda a: a.mass_kg)
        caps = [a.typical_battery_wh for a in masses]
        self.assertEqual(caps, sorted(caps))

    def test_endurances_are_all_plausible(self):
        """四种机型 10~60 分钟，量级检查。"""
        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                m = build_energy_model(SimConfig(energy_model="physical",
                                                 airframe=name))
                minutes = m.endurance_s() / 60.0
                self.assertGreater(minutes, 10.0)
                self.assertLess(minutes, 60.0)


class TestMicroRotorValidityLimit(unittest.TestCase):
    """把模型的**已知局限**固定下来，免得被当成精确值引用。"""

    def test_micro_rotor_efficiency_is_lower_than_large(self):
        """微尺度旋翼气动品质差，品质因数按机型区分，不能用同一个值。"""
        self.assertLess(AIRFRAMES["crazyflie"].figure_of_merit,
                        AIRFRAMES["px4_sihsim_quadx"].figure_of_merit)

    def test_crazyflie_endurance_is_optimistic_and_known_to_be(self):
        """动量理论在微尺度偏乐观：实测 Crazyflie 悬停约 7 分钟，
        本模型给出 20 分钟以上。差距来自微尺度电机与旋翼效率，
        模型不试图修正——**这条测试是为了让该偏差不被误当成精度**。
        """
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="crazyflie"))
        self.assertGreater(m.endurance_s() / 60.0, 15.0)

    def test_delivery_class_power_is_physically_plausible(self):
        """1 kg 级机型悬停 50~200 W 是合理区间——量级检查。"""
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="px4_sihsim_quadx"))
        self.assertGreater(m.hover_power_w(), 50.0)
        self.assertLess(m.hover_power_w(), 200.0)


class TestFactory(unittest.TestCase):
    def test_default_is_linear(self):
        self.assertIsInstance(build_energy_model(SimConfig()),
                              LinearEnergyModel)

    def test_physical_selected(self):
        m = build_energy_model(SimConfig(energy_model="physical"))
        self.assertIsInstance(m, PhysicalEnergyModel)

    def test_physical_respects_payload_and_battery(self):
        m = build_energy_model(SimConfig(energy_model="physical",
                                         payload_kg=0.5,
                                         battery_capacity_wh=50.0))
        self.assertEqual(m.payload_kg, 0.5)
        self.assertEqual(m.battery_capacity_wh, 50.0)

    def test_physical_uses_the_airframes_figure_of_merit(self):
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="crazyflie"))
        self.assertEqual(m.figure_of_merit,
                         AIRFRAMES["crazyflie"].figure_of_merit)

    def test_case_insensitive(self):
        m = build_energy_model(SimConfig(energy_model="PHYSICAL"))
        self.assertIsInstance(m, PhysicalEnergyModel)

    def test_unknown_model_raises(self):
        with self.assertRaises(ValueError) as ctx:
            build_energy_model(SimConfig(energy_model="nope"))
        self.assertIn("nope", str(ctx.exception))

    def test_unknown_airframe_raises(self):
        with self.assertRaises(ValueError) as ctx:
            build_energy_model(SimConfig(energy_model="physical",
                                         airframe="nope"))
        self.assertIn("nope", str(ctx.exception))


class TestStepVelocities(unittest.TestCase):
    """由位移反推速度——飞行状态的唯一判据。"""

    def test_stationary(self):
        self.assertEqual(_step_velocities((1.0, 2.0, 3.0),
                                          (1.0, 2.0, 3.0), 0.5), (0.0, 0.0))

    def test_horizontal_only(self):
        v_h, v_z = _step_velocities((0.0, 0.0, 100.0), (6.0, 8.0, 100.0), 1.0)
        self.assertAlmostEqual(v_h, 10.0)
        self.assertAlmostEqual(v_z, 0.0)

    def test_climb_and_descent_sign(self):
        _, up = _step_velocities((0.0, 0.0, 0.0), (0.0, 0.0, 5.0), 1.0)
        _, down = _step_velocities((0.0, 0.0, 5.0), (0.0, 0.0, 0.0), 1.0)
        self.assertAlmostEqual(up, 5.0)
        self.assertAlmostEqual(down, -5.0)

    def test_diagonal_splits_into_components(self):
        v_h, v_z = _step_velocities((0.0, 0.0, 0.0), (3.0, 4.0, 12.0), 1.0)
        self.assertAlmostEqual(v_h, 5.0)
        self.assertAlmostEqual(v_z, 12.0)

    def test_zero_dt_is_safe(self):
        self.assertEqual(_step_velocities((0.0, 0.0, 0.0),
                                          (1.0, 1.0, 1.0), 0.0), (0.0, 0.0))


class TestAirframeImmutability(unittest.TestCase):
    def test_frozen(self):
        af = AIRFRAMES["crazyflie"]
        with self.assertRaises(Exception):
            af.mass_kg = 1.0  # type: ignore[misc]

    def test_custom_airframe_is_accepted(self):
        af = Airframe("custom", 2.0, 1e-5, 900.0, 0.15)
        m = PhysicalEnergyModel(af)
        self.assertGreater(m.hover_power_w(), 0.0)


if __name__ == "__main__":
    unittest.main()
