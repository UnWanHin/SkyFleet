"""多机 3D 场景：N 架四旋翼同时在 MuJoCo 里闭环飞行。

**为什么要多机**：单机证明的是"飞控写对了"，多机才看得出这是**集群**仿真。
原报告的场景是 1 主 + 8 从 + 2 中继，如果只能画一架，交付物和课题是脱节的。

**性能约束（实测得出）**：瓶颈不在 MuJoCo 的物理积分，而在 Python 里逐机
跑控制器。每步要为 N 架各算一次串级控制 + 混控 + 转速环，所以 N 不能无脑
往上堆。当前实现 1 架约 25 µs/步，2 ms 步长下单帧有 80 倍余量——但那是
单线程串行的账，N=11 时余量降到 7 倍左右，仍可用。

**与 skyfleet 的关系**：这一层只管"飞机怎么飞"。通信协议、路由、投递确认
仍在 `skyfleet/`，两边通过航点而不是 import 耦合——保持解耦才能各自单独测。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from flightlab.control import CascadeController, ESCController, Gains
from flightlab.params import DEFAULT as DEFAULT_PARAMS
from flightlab.params import QuadParams


@dataclass(frozen=True)
class SwarmLayout:
    """多机的初始站位。"""

    count: int = 4
    #: 站位围成的圆半径 m。太小会相互气流干扰（本模型未建模），
    #: 太大会跑出相机视野，取 0.8 m 是可视性与间距的折中。
    radius: float = 0.8
    #: 初始高度 m
    altitude: float = 1.2


def build_swarm_xml(layout: SwarmLayout) -> str:
    """生成 N 架四旋翼的 MJCF。

    每架的 body / joint / site 名字都带编号后缀，避免重名——MuJoCo 里同名
    对象会静默取到第一个，症状是"其它飞机跟着第一架动"，极难查。
    """
    robots = []
    for i in range(layout.count):
        ang = 2.0 * math.pi * i / layout.count
        x = layout.radius * math.cos(ang)
        y = layout.radius * math.sin(ang)
        rotors = []
        # X 型布局，两正两反绕向；轴反向的两台使偏航反扭矩抵消
        positions = [(0.11, 0.11, 1), (-0.11, -0.11, 1),
                     (0.11, -0.11, -1), (-0.11, 0.11, -1)]
        for j, (rx, ry, sign) in enumerate(positions):
            rotors.append(f"""
        <body name="d{i}_rotor{j}" pos="{rx} {ry} 0">
          <joint name="d{i}_motor{j}" type="hinge" axis="0 0 {sign}"
                 armature="1.0e-7" damping="2.0e-8" frictionloss="1.0e-9"/>
          <geom type="cylinder" size=".045 .003" mass="0.0004" rgba=".20 .75 .45 .85"/>
          <site name="d{i}_thrust{j}" size=".01"/>
        </body>""")
        robots.append(f"""
    <body name="d{i}" pos="{x:.4f} {y:.4f} {layout.altitude}">
      <freejoint name="d{i}_root"/>
      <geom type="box" size=".055 .055 .018" mass="0.020" rgba=".85 .25 .15 1"/>
      <geom type="capsule" fromto=".035 .035 -.018  .035 .035 -.055" size=".004" mass="0.0005" rgba=".10 .10 .12 1"/>
      <geom type="capsule" fromto="-.035 -.035 -.018  -.035 -.035 -.055" size=".004" mass="0.0005" rgba=".10 .10 .12 1"/>
      <geom type="capsule" fromto="0 0 0  .11  .11 0" size=".006" mass="0.0015" rgba=".15 .15 .17 1"/>
      <geom type="capsule" fromto="0 0 0 -.11 -.11 0" size=".006" mass="0.0015" rgba=".15 .15 .17 1"/>
      <geom type="capsule" fromto="0 0 0  .11 -.11 0" size=".006" mass="0.0015" rgba=".15 .15 .17 1"/>
      <geom type="capsule" fromto="0 0 0 -.11  .11 0" size=".006" mass="0.0015" rgba=".15 .15 .17 1"/>{''.join(rotors)}
    </body>""")

    actuators = []
    for i in range(layout.count):
        for j in range(4):
            actuators.append(
                f'    <motor name="d{i}_cmd{j}" joint="d{i}_motor{j}" gear="1" '
                f'ctrllimited="true" ctrlrange="0 1.2e-2"/>')

    return f"""<mujoco model="swarm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4" density="0"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1=".25 .35 .5" rgb2=".75 .82 .9" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".32 .36 .40" rgb2=".26 .30 .34"/>
    <material name="ground" texture="grid" texrepeat="40 40" reflectance=".05"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true" castshadow="true"
           diffuse=".8 .8 .8"/>
    <geom name="ground" type="plane" size="80 80 .1" material="ground"/>
{''.join(robots)}
  </worldbody>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
</mujoco>"""


@dataclass
class DroneState:
    """单机的状态快照。"""

    pos: np.ndarray
    vel: np.ndarray
    R: np.ndarray
    omega: np.ndarray
    rotor_speed: np.ndarray
    target: np.ndarray
    saturated: bool


@dataclass
class SwarmLog:
    times: list[float] = field(default_factory=list)
    frames: list[list[DroneState]] = field(default_factory=list)

    def positions(self, index: int) -> np.ndarray:
        return np.array([f[index].pos for f in self.frames])


class SwarmSim:
    """N 架四旋翼共用一个 MuJoCo 场景，各自独立闭环。"""

    def __init__(self, layout: SwarmLayout | None = None,
                 params: QuadParams = DEFAULT_PARAMS,
                 gains: Gains | None = None) -> None:
        self.layout = layout or SwarmLayout()
        self.params = params
        self.gains = gains or Gains()
        xml = build_swarm_xml(self.layout)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.n = self.layout.count
        self._resolve()
        self._assert_mass_matches_params()
        self.controllers = [CascadeController(params, self.gains)
                            for _ in range(self.n)]
        self.escs = [ESCController(params, self.gains) for _ in range(self.n)]
        self.log = SwarmLog()

    def _resolve(self) -> None:
        m = self.model
        self._bodies = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"d{i}")
                        for i in range(self.n)]
        self._qpos = [m.jnt_qposadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, f"d{i}_root")] for i in range(self.n)]
        self._qvel = [m.jnt_dofadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, f"d{i}_root")] for i in range(self.n)]
        self._dofs = [[m.jnt_dofadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, f"d{i}_motor{j}")] for j in range(4)]
            for i in range(self.n)]
        self._rotor_bodies = [[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, f"d{i}_rotor{j}") for j in range(4)]
            for i in range(self.n)]
        self._sites = [[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_SITE, f"d{i}_thrust{j}") for j in range(4)]
            for i in range(self.n)]
        self._ctrl = [[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"d{i}_cmd{j}") for j in range(4)]
            for i in range(self.n)]
        #: 转向符号，由关节轴决定，不硬编码
        self._spin = np.array([
            [1.0 if m.jnt_axis[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT, f"d{i}_motor{j}")][2] >= 0 else -1.0
             for j in range(4)] for i in range(self.n)])

    def _assert_mass_matches_params(self) -> None:
        """模型真实质量必须等于控制器假设的质量。

        **这条检查是被一个真实 bug 逼出来的**：多机 MJCF 最初漏建起落架，
        实际 27.6 g，而控制器仍按单机的 28.6 g 算重力补偿。多出 3.6% 的
        推力把飞机顶高 54 mm（稳态误差，不是瞬态），而所有位置仍然对称、
        不发散、看起来完全正常。

        静默稳态误差比发散难查得多——发散会崩，这个只会悄悄偏。所以把它
        变成加载时就抛异常。
        """
        # 只统计第 0 架（本体 + 4 转子），不是整个场景的总质量
        model_mass = float(self.model.body_mass[self._bodies[0]]
                           + sum(self.model.body_mass[b]
                                 for b in self._rotor_bodies[0]))
        if abs(model_mass - self.params.mass) > 1e-5:
            raise ValueError(
                f"模型质量 {model_mass*1000:.2f} g 与控制器假设的 "
                f"{self.params.mass*1000:.2f} g 不一致（差 "
                f"{(model_mass-self.params.mass)*1000:+.2f} g）。"
                "控制器用它算重力补偿，不一致会产生静默稳态高度偏差。")

    # ------------------------------------------------------------ 状态

    def pos(self, i: int) -> np.ndarray:
        return self.data.qpos[self._qpos[i]:self._qpos[i] + 3].copy()

    def vel(self, i: int) -> np.ndarray:
        return self.data.qvel[self._qvel[i]:self._qvel[i] + 3].copy()

    def R(self, i: int) -> np.ndarray:
        q = self.data.qpos[self._qpos[i] + 3:self._qpos[i] + 7]
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, q)
        return mat.reshape(3, 3)

    def omega(self, i: int) -> np.ndarray:
        return self.data.qvel[self._qvel[i] + 3:self._qvel[i] + 6].copy()

    def rotor_speeds(self, i: int) -> np.ndarray:
        return np.array([self.data.qvel[d] for d in self._dofs[i]])

    # ------------------------------------------------------------ 复位

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for i in range(self.n):
            for d in self._dofs[i]:
                self.data.qvel[d] = self.params.omega_hover
            self.escs[i].reset()
        self.log = SwarmLog()
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------ 一步

    def step(self, targets: list[np.ndarray], t: float) -> list[DroneState]:
        # ---- 控制（逐机）----
        for i in range(self.n):
            speeds_des, diag = self.controllers[i].compute(
                self.pos(i), self.vel(i), self.R(i), self.omega(i),
                np.asarray(targets[i], dtype=float), np.zeros(3), 0.0)
            tau = self.escs[i].compute(self.rotor_speeds(i), speeds_des,
                                       self.dt)
            for j in range(4):
                self.data.ctrl[self._ctrl[i][j]] = tau[j]

        # ---- 气动接线（全部机体一次写完再积分）----
        self.data.qfrc_applied[:] = 0.0
        for i in range(self.n):
            thrust_dir = self.R(i)[:, 2]
            speeds = self.rotor_speeds(i)
            for j in range(4):
                point = self.data.site_xpos[self._sites[i][j]].copy()
                force = thrust_dir * self.params.k_f * speeds[j] ** 2
                mujoco.mj_applyFT(self.model, self.data, force, np.zeros(3),
                                  point, self._bodies[i],
                                  self.data.qfrc_applied)
                qvel = self.data.qvel[self._dofs[i][j]]
                omega_z = qvel * self._spin[i][j]
                drag = -math.copysign(1.0, omega_z) * self.params.k_m * qvel ** 2
                mujoco.mj_applyFT(self.model, self.data, np.zeros(3),
                                  np.array([0.0, 0.0, drag]), point,
                                  self._rotor_bodies[i][j],
                                  self.data.qfrc_applied)
        mujoco.mj_step(self.model, self.data)

        return [DroneState(pos=self.pos(i), vel=self.vel(i), R=self.R(i),
                           omega=self.omega(i),
                           rotor_speed=self.rotor_speeds(i),
                           target=np.asarray(targets[i], float),
                           saturated=False) for i in range(self.n)]

    def run(self, duration: float, targets_fn, *, record_every: int = 5
            ) -> SwarmLog:
        steps = int(round(duration / self.dt))
        t = 0.0
        for k in range(steps):
            states = self.step(targets_fn(t), t)
            if k % record_every == 0:
                self.log.times.append(t)
                self.log.frames.append(states)
            t += self.dt
        return self.log
