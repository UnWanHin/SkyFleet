"""调参模块的测试。

**为什么调参也要测**：`sweep()` 一次只改一个参数，如果 `dataclasses.replace`
用错或指标算错，"某参数没影响"这种结论就可能是假的——而假的"没影响"会让
人错过真正的瓶颈。开发时就遇到过：在 z 轴上扫 k_R 三行完全相同，一度以为
扫描坏了，实际是纯升降不涉及姿态，物理上就该没影响。
"""
from __future__ import annotations

import unittest

import numpy as np

from flightlab.control import Gains
from flightlab.tuning import Metrics, measure_step, sweep


class TestMeasureStep(unittest.TestCase):
    def test_a_well_tuned_step_converges(self):
        """默认增益必须收敛：稳态误差进厘米级、无明显超调。"""
        m = measure_step(Gains(), axis="z", distance=1.0, duration=6.0)
        self.assertLess(m.steady_err_m, 0.05,
                        f"稳态误差 {m.steady_err_m:.4f} m 偏大")
        self.assertLess(m.overshoot_pct, 10.0,
                        f"超调 {m.overshoot_pct:.1f}% 偏大")
        self.assertGreater(m.rise_s, 0.05, "上升时间过小，指标可能算错")

    def test_horizontal_step_also_converges(self):
        m = measure_step(Gains(), axis="x", distance=1.0, duration=6.0)
        self.assertLess(m.steady_err_m, 0.05)
        self.assertLess(m.overshoot_pct, 15.0)

    def test_saturation_is_reported_not_hidden(self):
        """饱和必须能看出来——它意味着控制器已失去部分权限。"""
        # 一步要求 10 m，远超 TWR 与限幅
        m = measure_step(Gains(), axis="z", distance=10.0, duration=3.0)
        self.assertGreaterEqual(m.saturated_ratio, 0.0)
        self.assertLessEqual(m.saturated_ratio, 1.0)


class TestSweep(unittest.TestCase):
    def test_sweep_changes_exactly_one_parameter(self):
        """★ 扫描必须只动一个参数。

        同时改两个，指标变好时无法判断是谁起作用，也无法判断有没有互相抵消。
        """
        base = Gains()
        results = sweep("kp_pos", [0.9, 1.562], axis="x", duration=4.0,
                        base=base)
        self.assertEqual(len(results), 2)
        # base 本身不能被修改
        self.assertAlmostEqual(base.kp_pos, 1.562)

    def test_a_stiffer_position_gain_rises_faster(self):
        """★ 位置环刚度提高，上升时间必须变短。

        这是"扫描确实在起作用"的物理证据：如果三行完全一样，说明参数
        没被真正传下去。
        """
        results = sweep("kp_pos", [0.9, 3.0], axis="x", duration=6.0)
        soft, stiff = results[0][1], results[1][1]
        self.assertLess(stiff.rise_s, soft.rise_s,
                        f"kp_pos 3.0 上升 {stiff.rise_s:.3f}s 不短于 "
                        f"0.9 的 {soft.rise_s:.3f}s——参数没生效")

    def test_attitude_gains_are_inert_on_a_vertical_step(self):
        """★ 纯升降不倾斜，k_R 必须**没有**影响。

        这条测试钉住一个容易误判的现象：在 z 轴上扫 k_R 会得到三行完全
        相同的数字。那不是扫描坏了，是物理如此。把它写成测试，避免将来
        有人看到"没差别"就去改扫描代码。
        """
        results = sweep("k_R", [0.006, 0.023], axis="z", duration=4.0)
        a, b = results[0][1], results[1][1]
        self.assertAlmostEqual(a.rise_s, b.rise_s, places=3)

    def test_more_attitude_damping_is_not_slower(self):
        """★ 加大 k_ω 是纯收益：更快且更平滑，不是速度换平稳。

        按 ζ=0.9 反推的 0.00138 只是个起点；扫描发现提到 0.0028 后超调归
        零、调节时间缩短，而且完整航线上最大角速度反而降低 29%。

        这条测试钉住"不会因为加大阻尼而变慢"这个反直觉结论——如果将来
        有人凭直觉把 k_ω 调回去，这里会红。
        """
        loose = sweep("k_omega", [0.00138, 0.0028], axis="x", duration=6.0)
        self.assertLessEqual(loose[1][1].settling_s, loose[0][1].settling_s,
                             "加大阻尼后调节时间变长了，与实测结论矛盾")
        self.assertLessEqual(loose[1][1].overshoot_pct,
                             loose[0][1].overshoot_pct)

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(AttributeError):
            sweep("no_such_gain", [1.0])


if __name__ == "__main__":
    unittest.main()
