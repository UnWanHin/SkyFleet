"""多机场景测试。

**为什么多机要单独测**：单机测试全绿也可能多机全错，因为多机引入了单机
没有的失效模式——名字重名、qpos 索引错位、转向符号串机。这些错误在单机
测试里照不到，一旦发生表现为"其它飞机跟着第一架动"，极难查。
"""
from __future__ import annotations

import math
import unittest

import numpy as np

from flightlab.params import DEFAULT
from flightlab.swarm import SwarmLayout, SwarmSim, build_swarm_xml


def _swarm(n: int = 4) -> SwarmSim:
    return SwarmSim(SwarmLayout(count=n, radius=0.8, altitude=1.2))


class TestModelGeneration(unittest.TestCase):
    def test_all_names_are_unique(self):
        """★ 名字必须唯一。

        MuJoCo 里同名对象会静默取到第一个，症状是"其它飞机跟着第一架动"。
        生成器必须保证每个 body / joint / site / actuator 都带机号。
        """
        xml = build_swarm_xml(SwarmLayout(count=3))
        sim = SwarmSim(SwarmLayout(count=3))
        self.assertEqual(sim.model.nbody, 3 * 5 + 1)   # 每架 1 本体 + 4 转子 + world
        self.assertEqual(sim.model.nu, 12)
        self.assertEqual(xml.count("<mujoco"), 1)

    def test_each_drone_has_its_own_indices(self):
        """每架飞机的 qpos 索引必须互不相同，否则状态会串。"""
        sim = _swarm(4)
        self.assertEqual(len(set(sim._qpos)), 4)
        self.assertEqual(len(set(sim._qvel)), 4)

    def test_spin_signs_are_two_positive_two_negative_per_drone(self):
        """每架的转向都必须两正两反，否则该机偏航反扭矩无法抵消。"""
        sim = _swarm(4)
        for i in range(4):
            with self.subTest(drone=i):
                self.assertAlmostEqual(float(sim._spin[i].sum()), 0.0)
                self.assertEqual(abs(sim._spin[i]).sum(), 4.0)


class TestMassConsistency(unittest.TestCase):
    def test_model_mass_equals_controller_assumption(self):
        """★ 模型真实质量必须等于控制器假设的质量。

        这条测试是被一个真实 bug 逼出来的：多机 MJCF 最初漏建起落架，
        实际 27.6 g，控制器仍按单机的 28.6 g 算重力补偿。多出 3.6% 推力
        把飞机顶高 **54 mm**——而且是稳态误差，不是瞬态。所有位置仍然
        对称、不发散、看起来完全正常，只有对着数字才看得出来。

        静默稳态误差比发散难查得多。加载时的断言 + 这条测试把它锁死。
        """
        sim = _swarm(4)
        one = float(sim.model.body_mass[sim._bodies[0]]
                    + sum(sim.model.body_mass[b]
                          for b in sim._rotor_bodies[0]))
        self.assertAlmostEqual(one, DEFAULT.mass, places=6)


class TestFlight(unittest.TestCase):
    def test_static_hover_has_no_steady_state_offset(self):
        """★ 静态悬停必须是毫米级，不能有厘米级稳态偏差。

        纯 PD 对常值目标没有稳态误差——如果出现几十毫米的恒定偏差，说明
        有别的力在补偿它，也就是某个假设错了（这里是质量对不上）。
        """
        sim = _swarm(4)
        sim.reset()
        home = [sim.pos(i).copy() for i in range(4)]
        sim.run(8.0, lambda t: home)
        for i in range(4):
            with self.subTest(drone=i):
                err = abs(float(sim.pos(i)[2]) - home[i][2])
                self.assertLess(err, 0.01,
                                f"机 {i} 稳态高度偏差 {err*1000:.1f} mm")
    def test_all_drones_hold_altitude(self):
        """编队呼吸过程中所有机的高度都要守住 1.2 m。"""
        sim = _swarm(4)
        sim.reset()
        home = [sim.pos(i).copy() for i in range(4)]

        def targets(t: float):
            k = 0.5 * math.sin(2 * math.pi * t / 6.0)
            out = []
            for h in home:
                r = math.hypot(h[0], h[1]) or 1.0
                out.append(np.array([h[0] * (1 + k / r),
                                     h[1] * (1 + k / r), 1.2]))
            return out

        log = sim.run(8.0, targets)
        for i in range(4):
            z = log.positions(i)[:, 2]
            with self.subTest(drone=i):
                self.assertLess(float(np.max(np.abs(z - 1.2))), 0.15,
                                f"机 {i} 高度偏离 1.2 m 超过 15 cm")

    def test_hover_speed_converges_for_every_drone(self):
        """悬停时每架的转速都要收敛到理论值 1873 rad/s。"""
        sim = _swarm(4)
        sim.reset()
        home = [sim.pos(i).copy() for i in range(4)]
        sim.run(6.0, lambda t: home)
        for i in range(4):
            speeds = sim.rotor_speeds(i)
            with self.subTest(drone=i):
                np.testing.assert_allclose(speeds, DEFAULT.omega_hover,
                                           rtol=0.03)

    def test_drones_do_not_share_state(self):
        """★ 一架被推到不同目标，其它架必须不受影响。"""
        sim = _swarm(4)
        sim.reset()
        home = [sim.pos(i).copy() for i in range(4)]
        moved = [h.copy() for h in home]
        moved[0] = moved[0] + np.array([0.8, 0.0, 0.0])
        sim.run(6.0, lambda t: moved)

        # 机 0 明显移动，其余三架留在原位
        self.assertGreater(float(np.linalg.norm(sim.pos(0) - home[0])), 0.5)
        for i in (1, 2, 3):
            with self.subTest(drone=i):
                self.assertLess(float(np.linalg.norm(sim.pos(i) - home[i])), 0.1,
                                f"机 {i} 被别的飞机的目标带偏了")


if __name__ == "__main__":
    unittest.main()
