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
    DEFAULT_CD_A_COEFF,
    LEGACY_TX_PER_BIT,
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

    def test_cruise_is_cheaper_than_hover_for_every_airframe(self):
        """★ 修正旋翼半径后，"微型机巡航反而更贵"这个结论**不再成立**。

        早前版本用的是被高估 2 倍的旋翼半径（把桨直径当半径），
        盘面积大 4 倍，废阻项（∝R^2）被放大，于是算出 Crazyflie
        巡航功率是悬停的 2.46 倍，并据此写下"微型机方向相反"的结论。

        修正半径后四种机型一致：12 m/s 巡航功率均为悬停的 0.55~0.77 倍。
        那条旧结论是半径 bug 的**假象**，不是微尺度物理。
        """
        for name in ("crazyflie", "crazyflie_brushless", "hummingbird",
                     "px4_sihsim_quadx"):
            m = PhysicalEnergyModel(AIRFRAMES[name])
            with self.subTest(airframe=name):
                ratio = m.cruise_power_w(12.0) / m.hover_power_w()
                self.assertLess(ratio, 1.0)
                self.assertGreater(ratio, 0.4)

    def test_climb_costs_more_than_hover(self):
        self.assertGreater(self.m.power_w(0.0, 12.0), self.m.hover_power_w())

    def test_descent_costs_less_than_hover(self):
        self.assertLess(self.m.power_w(0.0, -12.0), self.m.hover_power_w())

    def test_descent_power_is_positive_and_below_hover(self):
        """下降功率必须为正，且不应高于悬停。

        旧实现用 `descent_floor` 砍在**诱导**功率上，理由是"否则下降会
        发电"——该理由不成立（旧拼接式恒有 v_z+v_i >= 0）。现在下限落在
        **电气**功率上，理由改为"固定桨距电调不能回馈电能"。
        注意涡环区（真实功率**高于**悬停）未建模，见物理模型文档。
        """
        hover = self.m.hover_power_w()
        for v_z in (-0.5, -1.0, -2.0, -3.0, -5.0, -8.0):
            with self.subTest(v_z=v_z):
                p = self.m.power_w(0.0, v_z)
                self.assertGreater(p, 0.0)
                self.assertLess(p, hover)
        # 快速下降进入风车刹车区，诱导功率可以为负；固定桨距电调不能
        # 回馈电能，故电气功率被压到恰好 0——这是**正确**的边界行为，
        # 不是缺陷。代价是快速下降在模型里变成"免费"，属已知限制。
        for v_z in (-12.0, -20.0):
            with self.subTest(v_z=v_z):
                self.assertEqual(self.m.power_w(0.0, v_z), 0.0)

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


class TestTxEnergyIsPartOfTheModel(unittest.TestCase):
    """发送耗电必须**与飞行模型同源**，不能各用各的单位。

    原先引擎固定用 `energy_tx_per_bit`（线性模型的任意单位），
    而物理飞行模型用瓦特——两者单位不一致，只是量级小到看不出来。
    这里把"每个模型自己定义自己的发送能耗口径"钉住。
    """

    def test_linear_reports_the_configured_tx_cost(self):
        m = LinearEnergyModel(0.05, tx_per_bit=5.0e-7)
        self.assertEqual(m.tx_energy_per_bit(), 5.0e-7)

    def test_legacy_constant_matches_the_config_default(self):
        """`LEGACY_TX_PER_BIT` 与 `SimConfig.energy_tx_per_bit` 是同一件事的
        两处声明。两者一旦漂移，golden digest 会以一种很难查的方式改变
        （或该变而不变）。
        """
        from skyfleet.world.power import LEGACY_TX_PER_BIT
        self.assertEqual(LEGACY_TX_PER_BIT, SimConfig().energy_tx_per_bit)

    def test_linear_default_tx_keeps_legacy_value(self):
        """默认值必须等于既有 config 值，否则 golden digest 会变。"""
        self.assertEqual(LinearEnergyModel(0.05).tx_energy_per_bit(), 5.0e-7)

    def test_physical_tx_cost_is_physical(self):
        """物理模型：每比特能耗 = 发射功率 / 信道速率，再换算成百分比电量。"""
        m = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"],
                                battery_capacity_wh=50.0,
                                tx_power_w=0.333,
                                channel_rate_bps=2_000_000)
        expected = (0.333 / 2_000_000) / 3600.0 / 50.0 * 100.0
        self.assertAlmostEqual(m.tx_energy_per_bit(), expected, places=15)

    def test_physical_tx_cost_is_far_below_the_legacy_constant(self):
        """物理口径比旧常量小得多——旧常量是无依据的假设值。"""
        m = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"],
                                battery_capacity_wh=50.0)
        self.assertLess(m.tx_energy_per_bit(), 1e-10)

    def test_both_models_expose_the_same_interface(self):
        """引擎只调 tx_energy_per_bit()，两个模型都必须提供。"""
        for m in (LinearEnergyModel(0.05),
                  PhysicalEnergyModel(AIRFRAMES["crazyflie"])):
            with self.subTest(model=type(m).__name__):
                self.assertGreater(m.tx_energy_per_bit(), 0.0)

    def test_physical_tx_scales_with_battery_capacity(self):
        small = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"],
                                    battery_capacity_wh=25.0)
        big = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"],
                                  battery_capacity_wh=50.0)
        self.assertAlmostEqual(small.tx_energy_per_bit(),
                               2.0 * big.tx_energy_per_bit(), places=15)


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
        # 实测约 1:546 —— 即使**占满整条信道**发射，也只有悬停耗电的
        # 0.18%，远低于 1%。
        #
        # 这个比值随飞行功率变化：旋翼半径修正（0.047->0.023）使悬停
        # 功率翻倍，比值也就从 273 变成 546。**不要**在不理解来源的
        # 情况下改这个数；它由 hover_power_w 与 TxEnergyModel 共同决定。
        self.assertLess(tx_full_rate, 0.01 * hover)
        self.assertGreater(tx_full_rate, 0.0)
        self.assertGreater(hover / tx_full_rate, 100.0)
        self.assertAlmostEqual(hover / tx_full_rate, 546.0, delta=10.0)

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
        """动量理论在微尺度仍偏乐观，但修正半径后偏差大幅缩小。

        实测 Crazyflie 悬停约 7 分钟。半径修正前（0.047，把桨直径当半径）
        本模型给出约 23 分钟——偏乐观 3.3 倍，且其中约 2 倍来自几何错误
        而非真实的微尺度损失。修正后（0.023）给出约 11 分钟，偏乐观
        约 1.6 倍，剩下的才是真正的微尺度电机/旋翼效率损失。

        **这条测试是为了让该偏差不被误当成精度。**
        """
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="crazyflie"))
        minutes = m.endurance_s() / 60.0
        self.assertGreater(minutes, 8.0)
        self.assertLess(minutes, 15.0)
        # 相对实测 7 分钟的乐观倍数应在 1.2~2.2 之间
        self.assertGreater(minutes / 7.0, 1.2)
        self.assertLess(minutes / 7.0, 2.2)

    def test_delivery_class_power_is_physically_plausible(self):
        """1 kg 级机型悬停 50~200 W 是合理区间——量级检查。"""
        m = build_energy_model(SimConfig(energy_model="physical",
                                         airframe="px4_sihsim_quadx"))
        self.assertGreater(m.hover_power_w(), 50.0)
        self.assertLess(m.hover_power_w(), 200.0)


class TestPhysicsErrorsFoundInReview(unittest.TestCase):
    """评审发现的物理错误。每一条都先做过独立数值验证。

    这些测试的作用不是"函数能跑"，而是让这四处错误**不能悄悄回来**。
    """

    def test_micro_rotor_radius_is_geometrically_possible(self):
        """★ 原先把 46 mm 桨**直径**当成了半径。

        RotorPy 对 Crazyflie 只声明 `d = 0.043`（臂长），没有 rotor_radius。
        半径 0.047 m 意味着 94 mm 桨装在 43 mm 臂上——桨叶会穿过机架，
        几何上不可能。真实桨径约 46 mm，故半径约 0.023 m。
        """
        arm_length_m = 0.043      # RotorPy crazyflie_params.py 声明值
        for name in ("crazyflie", "crazyflie_brushless"):
            with self.subTest(airframe=name):
                af = AIRFRAMES[name]
                self.assertLess(
                    af.rotor_radius_m, arm_length_m,
                    f"{name} 的旋翼半径 {af.rotor_radius_m} 不小于臂长 "
                    f"{arm_length_m}，桨叶会互相重叠")
                self.assertAlmostEqual(af.rotor_radius_m, 0.023, places=3)

    def test_corrected_radius_puts_hover_power_in_measured_order(self):
        """修正半径后悬停功率应落在实测 6~8 W 的量级附近（原本 2.33 W）。"""
        m = PhysicalEnergyModel(AIRFRAMES["crazyflie"])
        self.assertGreater(m.hover_power_w(), 3.5)
        self.assertLess(m.hover_power_w(), 9.0)

    def test_direct_construction_uses_the_airframe_figure_of_merit(self):
        """品质因数必须默认取机型自带值。

        原默认值是全局 0.65，于是所有直接构造（含分析文档与 6 个测试）
        都给 Crazyflie 用了 0.65 而不是 0.35，机型区分形同虚设。
        """
        af = AIRFRAMES["crazyflie"]
        m = PhysicalEnergyModel(af)
        self.assertEqual(m.figure_of_merit, af.figure_of_merit)
        self.assertNotAlmostEqual(m.figure_of_merit, 0.65, places=3)

    def test_explicit_figure_of_merit_still_overrides(self):
        m = PhysicalEnergyModel(AIRFRAMES["crazyflie"], figure_of_merit=0.9)
        self.assertEqual(m.figure_of_merit, 0.9)

    def test_parasite_power_goes_through_the_efficiency_chain(self):
        """废阻功率也是电机必须输出的机械功率，必须除以推进效率。

        品质因数只描述旋翼，不作用于废阻——但推进效率描述的是
        电机+电调，废阻同样经过它。原实现把废阻整个排在效率链之外。
        判据：推进效率减半，**全部**机械功率都应翻倍。
        """
        af = AIRFRAMES["px4_sihsim_quadx"]
        full = PhysicalEnergyModel(af, propulsion_efficiency=1.0)
        half = PhysicalEnergyModel(af, propulsion_efficiency=0.5)
        for v_h in (5.0, 12.0, 20.0):
            with self.subTest(v_h=v_h):
                self.assertAlmostEqual(half.cruise_power_w(v_h),
                                       2.0 * full.cruise_power_w(v_h),
                                       places=6)

    def test_is_saturated_uses_the_thrust_magnitude_not_a_scalar_sum(self):
        """必需推力是 sqrt((mg)^2+D^2)，不是 mg+D。

        mg+D >= hypot(mg,D)，所以标量加法会**高估**饱和：PX4 从约
        30 m/s 就报饱和，而真实（推力/倾角）上限在约 40 m/s。
        """
        af = AIRFRAMES["px4_sihsim_quadx"]
        for v_h in range(0, 60, 2):
            v = float(v_h)
            drag = 0.5 * RHO * DEFAULT_CD_A_COEFF * af.rotor_radius_m ** 2 * v ** 2
            expected = math.hypot(af.hover_thrust_n(0.0),
                                  drag) > af.max_thrust_n
            with self.subTest(v_h=v_h):
                self.assertEqual(af.is_saturated(v, 0.0), expected)

    def test_is_saturated_and_power_w_agree_on_the_same_state(self):
        """两处对同一个状态必须给出一致的判断。"""
        af = AIRFRAMES["px4_sihsim_quadx"]
        m = PhysicalEnergyModel(af)
        drag = 0.5 * RHO * DEFAULT_CD_A_COEFF * af.rotor_radius_m ** 2 * 42.0 ** 2
        self.assertGreater(math.hypot(af.hover_thrust_n(0.0), drag),
                           af.max_thrust_n)
        self.assertTrue(af.is_saturated(42.0, 0.0))


class TestExactGlauertInflow(unittest.TestCase):
    """前飞 + 轴向的**联合**入流必须解同一个方程，不能用两个闭式解拼接。

    原实现把前飞闭式解 v_i_fwd 代进轴向闭式解，而两者来自**不同**的方程
    （`v_i(v_z+v_i)=v_i_fwd^2` 与 `v_i_fwd*sqrt(v_h^2+v_i_fwd^2)=v_hov^2`），
    因此只在两个极限上精确，中间偏小 8~18%。正确的联合关系是隐式的
    Glauert 式：`v_i * sqrt(v_h^2 + (v_z+v_i)^2) = v_hov^2`。

    验收标准就是两个极限必须与各自的闭式解吻合到机器精度。
    """

    def setUp(self):
        self.m = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"])

    def _thrust(self, v_h: float) -> float:
        """模型在 v_h 处实际使用的推力（含废阻，并按最大推力钳位）。

        注意**不能**用悬停推力代替：v_h=18 m/s 时废阻已有 3.7 N，
        推力 9.81 -> 10.49 N，由它算出的 v_hov^2 差 7%，会让断言失败。
        这是测试自身曾经写错的地方。
        """
        af = self.m.airframe
        drag = 0.5 * RHO * DEFAULT_CD_A_COEFF * af.rotor_radius_m ** 2 * v_h ** 2
        return min(math.hypot(af.hover_thrust_n(0.0), drag), af.max_thrust_n)

    def _vhov2(self, v_h: float) -> float:
        af = self.m.airframe
        return self._thrust(v_h) / (2.0 * RHO * af.disk_area_total)

    def _vhov(self, v_h: float = 0.0) -> float:
        return math.sqrt(self._vhov2(v_h))

    def test_reduces_to_the_axial_closed_form_when_not_moving_forward(self):
        """v_h=0 时必须还原轴向闭式解 v_i = -v_z/2 + sqrt((v_z/2)^2+v_hov^2)。"""
        vhov = self._vhov(0.0)
        for v_z in (0.0, 0.5, 2.0, 5.0, 12.0):
            with self.subTest(v_z=v_z):
                vi = self.m.induced_velocity(0.0, v_z)
                expected = -v_z / 2.0 + math.sqrt((v_z / 2.0) ** 2 + vhov ** 2)
                self.assertAlmostEqual(vi, expected, places=9)

    def test_reduces_to_the_forward_closed_form_when_not_climbing(self):
        """v_z=0 时必须还原前飞闭式解。"""
        for v_h in (0.0, 3.0, 12.0, 25.0, 40.0):
            with self.subTest(v_h=v_h):
                vhov = self._vhov(v_h)
                vi = self.m.induced_velocity(v_h, 0.0)
                expected = math.sqrt(
                    (math.sqrt(v_h ** 4 + 4.0 * vhov ** 4) - v_h ** 2) / 2.0)
                self.assertAlmostEqual(vi, expected, places=9)

    def test_satisfies_the_glauert_relation_everywhere(self):
        """中间区域必须真的满足 Glauert 隐式方程，而不是近似满足。"""
        for v_h in (2.0, 6.0, 12.0, 18.0):
            for v_z in (-1.0, 0.0, 1.0, 3.0, 6.0):
                with self.subTest(v_h=v_h, v_z=v_z):
                    vi = self.m.induced_velocity(v_h, v_z)
                    lhs = vi * math.hypot(v_h, v_z + vi)
                    self.assertAlmostEqual(lhs, self._vhov2(v_h), places=9)

    def test_exceeds_the_old_hybrid_when_climbing_and_moving_forward(self):
        """旧的拼接式在爬升+前飞时**偏小**，真实入流更大（诱导功率更高）。"""
        for v_h, v_z in ((12.0, 2.0), (20.0, 3.0), (10.0, 10.0)):
            vhov = self._vhov(v_h)
            vi_fwd = math.sqrt(
                (math.sqrt(v_h ** 4 + 4.0 * vhov ** 4) - v_h ** 2) / 2.0)
            hybrid = -v_z / 2.0 + math.sqrt((v_z / 2.0) ** 2 + vi_fwd ** 2)
            exact = self.m.induced_velocity(v_h, v_z)
            with self.subTest(v_h=v_h, v_z=v_z):
                self.assertGreater(exact, hybrid)
                self.assertGreater((exact - hybrid) / hybrid, 0.05)

    def test_hover_is_unchanged_by_the_joint_solution(self):
        """悬停处两个解必须完全相同——否则这次修改会悄悄挪动所有基线。"""
        self.assertAlmostEqual(self.m.induced_velocity(0.0, 0.0),
                               self._vhov(0.0), places=12)


class TestNonRegenerativeDescent(unittest.TestCase):
    """下降：动量理论允许负诱导功率（风车刹车），但电池不可回充。

    原实现的下限注释说"否则下降会算出发电"——**这句话是错的**：
    原来的拼接式在数学上恒有 v_z+v_i >= 0，根本不可能为负。
    改成精确 Glauert 之后负值才真的会出现，下限也才真正需要，
    但它应当落在**电气**功率上（固定桨距电调不能回馈电能），
    而不是砍在诱导功率上。
    """

    def setUp(self):
        self.m = PhysicalEnergyModel(AIRFRAMES["px4_sihsim_quadx"])

    def test_the_old_hybrid_could_never_have_gone_negative(self):
        """把"旧注释是错的"这件事钉住：拼接式恒有 v_z+v_i >= 0。"""
        af = self.m.airframe
        vhov = math.sqrt(af.hover_thrust_n(0.0)
                         / (2.0 * RHO * af.disk_area_total))
        for v_h in (0.0, 5.0, 12.0, 20.0):
            for v_z in [x / 4.0 for x in range(-80, 1)]:
                vi_fwd = math.sqrt(
                    (math.sqrt(v_h ** 4 + 4.0 * vhov ** 4) - v_h ** 2) / 2.0)
                hybrid = -v_z / 2.0 + math.sqrt((v_z / 2.0) ** 2 + vi_fwd ** 2)
                with self.subTest(v_h=v_h, v_z=v_z):
                    self.assertGreaterEqual(v_z + hybrid, -1e-12)

    def test_fast_descent_can_produce_negative_induced_power(self):
        """精确解在快速下降时确实给出负诱导功率——这正是下限存在的理由。"""
        neg = [self.m.induced_velocity(6.0, v_z) * (v_z
               + self.m.induced_velocity(6.0, v_z))
               for v_z in (-8.0, -12.0, -16.0)]
        self.assertTrue(any(p < 0.0 for p in neg),
                        "精确解在任何下降率下都不为负，下限就没有存在理由")

    def test_electrical_power_is_never_negative(self):
        """固定桨距电调不能回馈电能，电气功率必须有 0 下限。"""
        for v_h in (0.0, 5.0, 12.0, 20.0):
            for v_z in [x / 2.0 for x in range(-60, 21)]:
                with self.subTest(v_h=v_h, v_z=v_z):
                    self.assertGreaterEqual(self.m.power_w(v_h, v_z), 0.0)


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


class TestVortexRingStateIsQuantified(unittest.TestCase):
    """★ 跳线测试：把「涡环状态未建模」从模糊限制变成可执行触发条件。

    涡环状态（VRS，大致 `-2*v_hov < v_z < -0.5*v_hov`）内动量理论失效，
    且真实功率**高于**悬停，而模型因 `max(p_electric, 0.0)` 在带内给出
    约 0——偏低。

    实测（PX4 物理模型，600 s 基线）：该带内的能耗只占总飞行能耗的
    0.0144%。即使极端假设带内真实功率是模型的 3 倍，总能耗误差也只有
    0.029%，远小于模型本身已知的其他偏差。因此**刻意不实现**涡环建模：
    要有意义地实现它需要随下降率变化的经验公式，而本场景不覆盖该工况，
    无从标定——顺手发明物理正是本项目反复避免的错误。

    本测试的作用是**触发条件**：若将来有人调整下降速度、航点结构或机型，
    使涡环带能耗占比越过阈值，它会失败并提示「此处必须实现涡环建模」。
    在此之前，「已知限制」是零风险的。
    """

    #: 占比上限。实测 0.0144%，留约 7 倍余裕，避免因无关的数值微调而
    #: 误报；但一旦工况真的改变（例如下降速度降到涡环带内），必然触发。
    MAX_BAND_ENERGY_SHARE = 0.001     # 0.1%

    def _band(self) -> tuple[float, float]:
        af = AIRFRAMES["px4_sihsim_quadx"]
        v_hov = math.sqrt(af.hover_thrust_n(0.0) / (2.0 * RHO * af.disk_area_total))
        return -2.0 * v_hov, -0.5 * v_hov

    def test_vortex_ring_band_is_negligible_in_the_baseline(self):
        from dataclasses import replace
        from pathlib import Path
        import tempfile

        from skyfleet.kernel.config import load_config
        from skyfleet.sim.engine import Engine
        from skyfleet.world.power import PhysicalEnergyModel

        lo, hi = self._band()
        samples: list[tuple[float, float]] = []
        original = PhysicalEnergyModel.energy_for_step

        def spy(model, v_h, v_z, dt):
            energy = original(model, v_h, v_z, dt)
            samples.append((energy, v_z))
            return energy

        PhysicalEnergyModel.energy_for_step = spy
        try:
            with tempfile.TemporaryDirectory() as tmp:
                cfg = replace(load_config(Path("skyfleet/scenarios/baseline_small.json")),
                              log_path=str(Path(tmp) / "vrs.jsonl"),
                              energy_model="physical",
                              airframe="px4_sihsim_quadx")
                Engine(cfg).run()
        finally:
            PhysicalEnergyModel.energy_for_step = original

        self.assertGreater(len(samples), 1000, "取样太少，测试会空转")
        total = sum(e for e, _ in samples)
        self.assertGreater(total, 0.0, "总能耗为 0，前提不成立")
        in_band = [e for e, v_z in samples if lo < v_z < hi]
        share = sum(in_band) / total
        self.assertLess(
            share, self.MAX_BAND_ENERGY_SHARE,
            f"涡环带内能耗占比 {share:.6%} 超过阈值 "
            f"{self.MAX_BAND_ENERGY_SHARE:.2%}（带 {lo:.2f}..{hi:.2f} m/s，"
            f"{len(in_band)} 个取样点）。\n"
            f"这说明工况已改变，**必须实现涡环建模**——"
            f"见分析文档 §13。")

    def test_the_band_is_still_entered_at_all(self):
        """反向上界：确认带确实会被进入。

        若这条失败，说明场景**完全**不经过涡环带（例如下降速度全部
        在带外），那么「不建模」的决定更加没有风险——但这也是一个
        应当被知道的事实变化，所以一并钉住。
        """
        lo, hi = self._band()
        self.assertLess(lo, hi)
        # PX4 的带约为 -9.03 .. -2.26 m/s
        self.assertAlmostEqual(lo, -9.03, delta=0.1)
        self.assertAlmostEqual(hi, -2.26, delta=0.1)


if __name__ == "__main__":
    unittest.main()
