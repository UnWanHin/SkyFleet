"""物理接线的符号约定测试。

存在原因：符号错误不会让程序崩溃，只会让飞行器**安静地飞反**。开发过程
中这一点让四个症状依次暴露（推力重复累加、反扭矩同向叠加、转速环自激、
阻力矩加能量），每修一处就冒出下一处，因为没有任何一处集中约束符号。

所以这里不测"能飞"，只测**接线本身是否正确**：每条物理约束单独一条用例，
失败时能立刻指出是哪一根线接反了。
"""
from __future__ import annotations

import math
import unittest

import mujoco
import numpy as np

from flightlab.params import DEFAULT
from flightlab.sim import QuadrotorSim


def _fresh(z: float = 1.0, rotor_speed: float = 0.0) -> QuadrotorSim:
    sim = QuadrotorSim()
    sim.reset(pos=(0.0, 0.0, z), rotor_speed=rotor_speed)
    return sim


class TestSpinConvention(unittest.TestCase):
    def test_rotors_are_two_positive_two_negative(self):
        """X 型布局必须两正两反，否则偏航反扭矩无法抵消。"""
        sim = _fresh()
        s = sim._spin_sign
        self.assertEqual(abs(s).sum(), 4.0)
        self.assertAlmostEqual(float(s.sum()), 0.0)

    def test_spin_signs_come_from_the_xml_axes(self):
        """转向必须由关节轴推出，不能硬编码——否则与 XML 脱节。"""
        sim = _fresh()
        for i in range(4):
            jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"motor{i}")
            axis_z = sim.model.jnt_axis[jid][2]
            self.assertEqual(sim._spin_sign[i],
                             1.0 if axis_z >= 0 else -1.0)

    def test_measured_speed_is_always_positive(self):
        """转速以**大小**返回。带符号的值送进 ESC 会让误差凭空翻倍。"""
        sim = _fresh(rotor_speed=DEFAULT.omega_hover)
        self.assertTrue(np.all(sim.rotor_speeds() >= 0.0),
                        f"读回负转速 {sim.rotor_speeds()}，转速环会自激")
        np.testing.assert_allclose(sim.rotor_speeds(), DEFAULT.omega_hover,
                                   rtol=1e-9)


class TestThrustWiring(unittest.TestCase):
    def test_thrust_points_along_body_up(self):
        sim = _fresh(z=1.0, rotor_speed=DEFAULT.omega_hover)
        sim.reset(pos=(0.0, 0.0, 1.0), yaw=0.0,
                  rotor_speed=DEFAULT.omega_hover)
        before = sim.vel.copy()
        sim.step(np.array([0.0, 0.0, 1.0]), 0.0)
        # 悬停转速下净加速度应接近 0（推力恰好抵重力）
        accel = (sim.vel - before) / sim.dt
        self.assertLess(abs(accel[2]), 0.15,
                        f"悬停转速下垂直加速度 {accel[2]:.3f} m/s² 偏离 0")

    def test_hover_thrust_balances_weight(self):
        sim = _fresh(rotor_speed=DEFAULT.omega_hover)
        f = DEFAULT.k_f * sim.rotor_speeds() ** 2
        self.assertAlmostEqual(float(f.sum()), DEFAULT.mass * DEFAULT.g,
                               places=4)


class TestDragNeverAddsEnergy(unittest.TestCase):
    """★ 阻力矩必须在任何转向下都**减小**转速大小。

    开发中这条被接反过：轴反向的转子 qvel 为正、世界系角速度为负，
    按 qvel 的符号算阻力等于给转子**加**能量，转速一路冲到 6755 rad/s
    （超过物理上限 3004），表现为"电机疯了"。
    """

    def test_drag_decelerates_whichever_way_the_rotor_turns(self):
        for i in range(4):
            with self.subTest(rotor=i):
                sim = _fresh(z=2.0, rotor_speed=600.0)
                sim.data.ctrl[:] = 0.0          # 不给电机力矩
                w0 = abs(sim.data.qvel[sim._dofs[i]])
                for _ in range(50):
                    # 只跑气动接线 + 物理积分，绕开控制器——
                    # 走 step() 会被 ESC 重新供能，测不出阻力
                    sim.data.ctrl[:] = 0.0
                    sim._apply_aerodynamics()
                    mujoco.mj_step(sim.model, sim.data)
                w1 = abs(sim.data.qvel[sim._dofs[i]])
                self.assertLess(w1, w0,
                                f"rotor{i} 无动力时转速从 {w0:.1f} 升到 "
                                f"{w1:.1f}——阻力矩在加能量")

    def test_rotor_speed_never_exceeds_physical_limit(self):
        sim = _fresh(z=2.0, rotor_speed=0.0)
        sim.data.ctrl[:] = DEFAULT.torque_max      # 全油门
        for _ in range(1500):
            sim.data.ctrl[:] = DEFAULT.torque_max
            sim._apply_aerodynamics()
            mujoco.mj_step(sim.model, sim.data)
        peak = float(np.max(sim.rotor_speeds()))
        self.assertLess(peak, DEFAULT.omega_max * 1.10,
                        f"转速 {peak:.0f} 超过物理上限 "
                        f"{DEFAULT.omega_max:.0f}")


class TestTorqueWiring(unittest.TestCase):
    def test_equal_rotor_speeds_produce_no_net_body_yaw(self):
        """四桨同速时，电机反扭矩必须净为零。"""
        sim = _fresh(rotor_speed=DEFAULT.omega_hover)
        tau = DEFAULT.k_m * DEFAULT.omega_hover ** 2
        sim.data.ctrl[:] = tau
        yaw = sum(-sim._spin_sign[i] * tau for i in range(4))
        self.assertAlmostEqual(yaw, 0.0, places=12)

    def test_mixer_inverts_cleanly(self):
        """混控矩阵可逆，且悬停解为四桨均分。"""
        sim = _fresh()
        f, sat = sim.controller.mix(DEFAULT.mass * DEFAULT.g, np.zeros(3))
        self.assertFalse(sat)
        np.testing.assert_allclose(f, DEFAULT.hover_thrust_per_rotor,
                                   rtol=1e-9)

    def test_yaw_command_accelerates_opposite_pairs(self):
        """偏航指令必须让两对转子反向增减，且总推力守恒。"""
        sim = _fresh()
        base = DEFAULT.mass * DEFAULT.g
        f, _ = sim.controller.mix(base, np.array([0.0, 0.0, 1e-3]))
        self.assertAlmostEqual(float(f.sum()), base, places=6)
        self.assertGreater(abs(f[0] - f[2]), 1e-4,
                           "偏航指令没有产生差动")

    def test_yaw_mixer_sign_matches_physics(self):
        """★ 混控的偏航符号必须与机体实际受力**同号**。

        机体力矩来自转子阻力反作用：CCW(0/1) 给 −z，CW(2/3) 给 +z，
        因此 τ_z = c(f₂+f₃−f₀−f₁)。写反会变成正反馈，1e-6 的数值扰动
        会被放大到发散——症状是"悬停时莫名自旋"，但每一行代码看起来都对。

        这里用**物理反推**验证：给混控一个 +τz，算出四桨推力，再按物理
        公式算机体应得力矩，两者必须同号且同量级。
        """
        sim = _fresh()
        base = DEFAULT.mass * DEFAULT.g
        tau_z = 1e-3
        f, _ = sim.controller.mix(base, np.array([0.0, 0.0, tau_z]))
        # 物理：CCW 转子给机身 −k_m ω²，CW 转子给 +k_m ω²
        c = DEFAULT.k_m / DEFAULT.k_f
        physical = c * (f[2] + f[3] - f[0] - f[1])
        self.assertAlmostEqual(physical, tau_z, places=9)
        self.assertGreater(f[2] + f[3], f[0] + f[1],
                           "正偏航指令应让 CW 那对转得更快")

    def test_positive_yaw_torque_turns_the_body_positive(self):
        """★ 端到端：给正偏航力矩，机身偏航角速度必须变正。

        这是上一条的物理验证——绕开一切推导，直接看仿真结果。
        """
        sim = _fresh(z=5.0, rotor_speed=DEFAULT.omega_hover)
        for _ in range(300):
            f, _ = sim.controller.mix(DEFAULT.mass * DEFAULT.g,
                                      np.array([0.0, 0.0, 2e-3]))
            sim.data.ctrl[:] = DEFAULT.k_m * (f / DEFAULT.k_f)
            sim._apply_aerodynamics()
            mujoco.mj_step(sim.model, sim.data)
        self.assertGreater(float(sim.omega[2]), 0.0,
                           f"正偏航力矩得到负角速度 {sim.omega[2]:.4f}——"
                           "混控符号与物理相反")


if __name__ == "__main__":
    unittest.main()
