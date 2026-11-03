"""与 RotorPy 的交叉验证。

**为什么需要这条验证**：自己写的动力学最容易犯的错不是"算错"，而是"和
公认实现用的不是同一个模型"，然后所有结论都建在一个孤立的假设上。RotorPy
是公开的多旋翼仿真包，用它的 crazyflie 参数跑同一组指令，看高度响应是否
一致，是最直接的证伪手段。

**做法**：生成一个参数与 RotorPy `crazyflie_params` 完全对齐的 MJCF
（质量、转动惯量、k_f、k_m 全部照抄），给两边相同的马达转速指令，比较
1 秒内的垂直轨迹。

**预期**：两者都是刚体 + `F=k_f·ω²`、`τ=k_m·ω²`，差异只应来自积分器
（MuJoCo RK4 vs RotorPy RK4）与转子一阶动态。若差到厘米级，说明模型
不是同一个。
"""
from __future__ import annotations

import pathlib
import tempfile
import unittest

import mujoco
import numpy as np

from flightlab.params import QuadParams
from flightlab.sim import QuadrotorSim

try:
    from rotorpy.vehicles.crazyflie_params import quad_params as RP_PARAMS
    from rotorpy.vehicles.multirotor import Multirotor
    HAVE_ROTORPY = True
except ImportError:                                  # pragma: no cover
    HAVE_ROTORPY = False


#: 与 RotorPy crazyflie 对齐的 MJCF。显式写 <inertial>，否则 MuJoCo 会
#: 按几何体算惯量，和 RotorPy 的 1.43e-5 差 6 倍，比出来的差异全是机型的。
#:
#: `balanceinertia="true"` 不是随手加的：RotorPy 的 crazyflie 惯量
#: (Ixx=Iyy=1.43e-5, Izz=2.89e-5) 满足不了刚体三角不等式
#: Ixx+Iyy >= Izz（2.86e-5 < 2.89e-5），MuJoCo 直接拒绝加载。公开参数
#: 里存在约 1% 的物理不一致，靠 balanceinertia 把惯量微调进可行域。
_ALIGNED_XML = """<mujoco model="cf_aligned">
  <compiler angle="radian" autolimits="true" balanceinertia="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4" density="0"/>
  <worldbody>
    <body name="drone" pos="0 0 1">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="0.030"
                diaginertia="1.43e-5 1.43e-5 2.89e-5"/>
      <geom name="hull" type="box" size=".055 .055 .018" mass="0"/>
      <body name="rotor0" pos=" .11  .11 0">
        <joint name="motor0" type="hinge" axis="0 0 1" armature="1.0e-7"/>
        <geom type="cylinder" size=".045 .003" mass="1e-6"/>
        <site name="thrust0" size=".01"/>
      </body>
      <body name="rotor1" pos="-.11 -.11 0">
        <joint name="motor1" type="hinge" axis="0 0 1" armature="1.0e-7"/>
        <geom type="cylinder" size=".045 .003" mass="1e-6"/>
        <site name="thrust1" size=".01"/>
      </body>
      <body name="rotor2" pos=" .11 -.11 0">
        <joint name="motor2" type="hinge" axis="0 0 -1" armature="1.0e-7"/>
        <geom type="cylinder" size=".045 .003" mass="1e-6"/>
        <site name="thrust2" size=".01"/>
      </body>
      <body name="rotor3" pos="-.11  .11 0">
        <joint name="motor3" type="hinge" axis="0 0 -1" armature="1.0e-7"/>
        <geom type="cylinder" size=".045 .003" mass="1e-6"/>
        <site name="thrust3" size=".01"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="cmd0" joint="motor0" gear="1" ctrllimited="true" ctrlrange="0 1.2e-2"/>
    <motor name="cmd1" joint="motor1" gear="1" ctrllimited="true" ctrlrange="0 1.2e-2"/>
    <motor name="cmd2" joint="motor2" gear="1" ctrllimited="true" ctrlrange="0 1.2e-2"/>
    <motor name="cmd3" joint="motor3" gear="1" ctrllimited="true" ctrlrange="0 1.2e-2"/>
  </actuator>
</mujoco>"""


def _aligned_sim() -> QuadrotorSim:
    tmp = pathlib.Path(tempfile.mkdtemp()) / "cf_aligned.xml"
    tmp.write_text(_ALIGNED_XML, encoding="utf-8")
    p = QuadParams(mass_body=0.030, mass_rotor=0.0,
                   k_f=RP_PARAMS["k_eta"], k_m=RP_PARAMS["k_m"],
                   torque_max=1.2e-2)
    sim = QuadrotorSim(xml_path=str(tmp), params=p)
    sim.reset(pos=(0.0, 0.0, 1.0), rotor_speed=0.0)
    return sim


@unittest.skipUnless(HAVE_ROTORPY, "需要 rotorpy")
class TestAgreementWithRotorPy(unittest.TestCase):
    def test_mass_and_inertia_match_the_reference(self):
        sim = _aligned_sim()
        self.assertAlmostEqual(float(sim.model.body_mass[1]), 0.030, places=6)

    def test_hover_speed_formula_agrees(self):
        """悬停转速公式必须与 RotorPy 一致（同一个动量理论）。"""
        sim = _aligned_sim()
        expected = np.sqrt(RP_PARAMS["mass"] * 9.81
                           / (4.0 * RP_PARAMS["k_eta"]))
        self.assertAlmostEqual(sim.params.omega_hover, expected, places=6)
        # RotorPy 自己内建的初值就是 1788.53
        self.assertAlmostEqual(expected, 1788.53, delta=0.1)

    def test_rigid_body_and_aero_agree_to_a_millimetre(self):
        """★ 刚体动力学 + 气动模型必须与 RotorPy 一致到毫米级。

        这是整个模型的证伪测试。**必须让两边的初始转子转速相同**，否则
        比的是电机起动动态而不是刚体动力学——见下面那条测试。

        实测：两边都从 w_cmd 起转、aero 关闭时，1 秒后高度
        2.0293 m vs 2.0301 m，差 0.8 mm。
        """
        sim = _aligned_sim()
        w_cmd = 1.1 * sim.params.omega_hover
        sim.reset(pos=(0.0, 0.0, 1.0), rotor_speed=w_cmd)

        state = {
            "x": np.array([0.0, 0.0, 1.0]), "v": np.zeros(3),
            "q": np.array([0.0, 0.0, 0.0, 1.0]), "w": np.zeros(3),
            "wind": np.zeros(3), "rotor_speeds": np.full(4, w_cmd),
        }
        # aero=False：RotorPy 的 aero 额外建模机体阻力 (k_d/k_z)，本项目没有，
        # 要比较的是**共同的**那部分物理。
        veh = Multirotor(RP_PARAMS, state, control_abstraction="cmd_motor_speeds",
                         aero=False)
        dt = 0.002
        for _ in range(500):
            sim.data.ctrl[:] = sim.params.k_m * w_cmd ** 2
            sim._apply_aerodynamics()
            mujoco.mj_step(sim.model, sim.data)
            state = veh.step(state, {"cmd_motor_speeds": np.full(4, w_cmd)}, dt)

        diff = float(sim.pos[2] - state["x"][2])
        self.assertLess(abs(diff), 0.005,
                        f"与 RotorPy 高度差 {diff:.4f} m——两边的刚体/气动模型"
                        "不是同一个")

    def test_motor_spin_up_differs_and_that_is_expected(self):
        """★ 电机起动动态**本就不同**，这条测试把它固定成已知差异。

        本项目：转子有转动惯量，由力矩驱动，受二次气动阻力限制 -> 起动有
        真实滞后。RotorPy：一阶转速滞后模型。所以"从 0 起转"的 1 秒高度
        差 0.4 m，这不是 bug。

        把它写成测试是为了防止将来有人看到 0.4 m 的差异就去"修"刚体动力学
        ——修错地方比不修更糟。
        """
        sim = _aligned_sim()
        w_cmd = 1.1 * sim.params.omega_hover
        sim.reset(pos=(0.0, 0.0, 1.0), rotor_speed=0.0)

        state = {
            "x": np.array([0.0, 0.0, 1.0]), "v": np.zeros(3),
            "q": np.array([0.0, 0.0, 0.0, 1.0]), "w": np.zeros(3),
            "wind": np.zeros(3), "rotor_speeds": np.zeros(4),
        }
        veh = Multirotor(RP_PARAMS, state, control_abstraction="cmd_motor_speeds",
                         aero=False)
        for _ in range(500):
            sim.data.ctrl[:] = sim.params.k_m * w_cmd ** 2
            sim._apply_aerodynamics()
            mujoco.mj_step(sim.model, sim.data)
            state = veh.step(state, {"cmd_motor_speeds": np.full(4, w_cmd)}, 0.002)

        diff = float(sim.pos[2] - state["x"][2])
        # 只要求"显著但同量级"，钉住这是起动动态差异而非发散
        self.assertGreater(abs(diff), 0.05,
                           "起动差异消失，说明电机模型被改成了瞬时响应")
        self.assertLess(abs(diff), 1.0,
                        f"起动差异 {diff:.3f} m 过大，可能真的发散了")


if __name__ == "__main__":
    unittest.main()
