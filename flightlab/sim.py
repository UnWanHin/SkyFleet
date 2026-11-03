"""MuJoCo 里的四旋翼闭环仿真。

**为什么用 MuJoCo 而不是自己积分**：刚体动力学、接触、关节约束交给成熟
引擎，本项目只负责"旋翼气动 + 控制"这两块真正需要自己写的东西。这也是
Gazebo + PX4 的分工方式（引擎管物理，控制器管飞行）。

**MuJoCo 没有旋翼气动**：它不会因为转子转动而产生升力。这是引擎的已知
边界，不是本项目的简化。因此推力按 `F = k_f·ω²` 由**转子实际角速度**
算出，用 `mj_applyFT` 作用在转子位置上，反扭矩同样按 `k_m·ω²` 加上。
关键在于：推力不是指令值，而是转速的函数——所以电机响应滞后会真实地
体现在飞行上。

坐标系约定：
* 世界系 z 轴向上，重力 `[0, 0, -g]`；
* 机体系 z 轴指向推力方向；
* 姿态用旋转矩阵 R（列向量为机体系各轴在世界系的表示）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from flightlab.control import (CascadeController, ESCController, Gains,
                               desired_attitude)
from flightlab.params import DEFAULT as DEFAULT_PARAMS
from flightlab.params import QuadParams

#: 控制器频率 Hz。姿态环比位置环快一个量级，是串级控制的标准做法：
#: 内环必须足够快，才能在外环看来"瞬时"跟随。
CTRL_HZ_POS = 100.0
CTRL_HZ_ATT = 500.0


@dataclass
class Step:
    """一步的状态快照，用于画图与分析。"""

    t: float
    pos: np.ndarray
    vel: np.ndarray
    R: np.ndarray
    omega: np.ndarray
    rotor_speed: np.ndarray
    rotor_thrust: np.ndarray
    target: np.ndarray
    torque: np.ndarray
    saturated: bool


@dataclass
class FlightLog:
    steps: list[Step] = field(default_factory=list)

    def positions(self) -> np.ndarray:
        return np.array([s.pos for s in self.steps])

    def times(self) -> np.ndarray:
        return np.array([s.t for s in self.steps])


class QuadrotorSim:
    """四旋翼闭环仿真器。"""

    def __init__(self, xml_path: str = "flightlab/quadrotor.xml",
                 params: QuadParams = DEFAULT_PARAMS,
                 gains: Gains | None = None) -> None:
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.params = params
        self.gains = gains or Gains()
        self.controller = CascadeController(params, self.gains)
        self.esc = ESCController(params, self.gains)
        self.dt = self.model.opt.timestep
        self._resolve_ids()
        #: 转子转向符号，由 XML 里的关节轴决定（+1 轴为 +z，-1 轴为 -z）。
        #: X 型布局必须两正两反：转向相同时四个电机的反扭矩会**同向叠加**，
        #: 机身被持续偏航，姿态环只能靠差动推力硬扛，最终对称饱和。
        self._spin = self._read_spin_signs()
        self._spin_sign = self._spin
        self.log = FlightLog()

    def _read_spin_signs(self) -> np.ndarray:
        """从关节轴读出转向符号，**不硬编码**——硬编码会和 XML 脱节。"""
        signs = []
        for i in range(4):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"motor{i}")
            axis = self.model.jnt_axis[jid]
            signs.append(1.0 if axis[2] >= 0 else -1.0)
        arr = np.array(signs)
        # X 型四旋翼必须两正两反，否则偏航反扭矩无法抵消
        if abs(arr.sum()) > 1e-9 or abs(arr).sum() != 4.0:
            raise ValueError(
                f"转子转向必须两正两反（实际 {arr.tolist()}）。"
                "同向会让四个电机的反扭矩叠加，机身持续自旋。")
        return arr

    def _resolve_ids(self) -> None:
        m = self.model
        self._drone_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "drone")
        self._site_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE,
                                            f"thrust{i}") for i in range(4)]
        self._dofs = [m.jnt_dofadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, f"motor{i}")] for i in range(4)]
        self._rotor_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                             f"rotor{i}") for i in range(4)]

    # ------------------------------------------------------------ 状态

    @property
    def pos(self) -> np.ndarray:
        return self.data.qpos[0:3].copy()

    @property
    def vel(self) -> np.ndarray:
        return self.data.qvel[0:3].copy()

    @property
    def R(self) -> np.ndarray:
        q = self.data.qpos[3:7]
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, q)
        return mat.reshape(3, 3)

    @property
    def omega(self) -> np.ndarray:
        return self.data.qvel[3:6].copy()

    def rotor_speeds(self) -> np.ndarray:
        """各转子转速大小 rad/s，**恒为正值**。

        为什么 qvel 本身就是正值：转向已经由**关节轴**编码（转子 2/3 的
        轴是 -z），所以正向 qvel 对 0/1 号表示逆时针、对 2/3 号表示顺时针。
        转向由 `_spin_sign` 承担，这里只管大小。

        ★ 这一条被搞错过两次，值得记下来：曾经「修」成返回
        `spin_sign * qvel`，于是另外又去把 `reset` 的初速改成带符号，
        两个改动一起做，症状互相掩盖——转子一开机反向空转，电机力矩把它
        推回正方向，机身被反扭矩带着自旋。**一次只改一个变量**不是形式主义。
        """
        return np.array([self.data.qvel[d] for d in self._dofs])

    def rotor_positions(self) -> np.ndarray:
        return np.array([self.data.site_xpos[s] for s in self._site_ids])

    # ------------------------------------------------------------ 复位

    def reset(self, pos=(0.0, 0.0, 0.05), yaw: float = 0.0,
              rotor_speed: float | None = None) -> None:
        """复位到给定位置与偏航角。

        `rotor_speed` 给定时转子以此初速启动，用于"空中起航"的测试；
        默认 0 表示从地面静止开始，能看出起飞过程本身。
        """
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = np.asarray(pos, dtype=float)
        self.data.qpos[3:7] = np.array([math.cos(yaw / 2), 0.0, 0.0,
                                        math.sin(yaw / 2)])
        w = self.params.omega_hover if rotor_speed is None else rotor_speed
        # 初速一律取正：关节轴已编码转向，负初速等于让转子反向空转
        for d in self._dofs:
            self.data.qvel[d] = w
        self.esc.reset()
        self.log = FlightLog()
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------ 一步

    def step(self, target: np.ndarray, t: float,
             v_target: np.ndarray | None = None,
             yaw_target: float = 0.0) -> Step:
        """推进一个物理步长（含控制解算）。"""
        speeds_des, diag = self.controller.compute(
            self.pos, self.vel, self.R, self.omega,
            np.asarray(target, dtype=float),
            np.zeros(3) if v_target is None else np.asarray(v_target, float),
            yaw_target)
        omega = self.rotor_speeds()
        torque = self.esc.compute(omega, speeds_des, self.dt)
        self.data.ctrl[:] = torque
        forces = self._apply_aerodynamics()
        mujoco.mj_step(self.model, self.data)

        snap = Step(t=t, pos=self.pos, vel=self.vel, R=self.R,
                    omega=self.omega, rotor_speed=omega, rotor_thrust=forces,
                    target=np.asarray(target, float).copy(),
                    torque=torque, saturated=diag["saturated"])
        return snap

    # ------------------------------------------------------------ 气动接线

    def _apply_aerodynamics(self) -> np.ndarray:
        """按**当前**转子转速施加推力与阻力矩。与控制回路完全解耦。

        拆成独立方法是为了能被单独测试：符号错误不会让程序崩溃，只会让
        飞行器安静地飞反，所以必须能在不跑控制器的情况下验证接线。

        返回四桨推力，供日志记录。
        """
        p = self.params
        thrust_dir = self.R[:, 2]
        # 推力按**实际**转速算，不是按指令算——这是电机动态能体现出来的原因
        forces = p.k_f * self.rotor_speeds() ** 2

        # ★ 必须每步清零 qfrc_applied：mj_applyFT 是**累加**语义，而
        # mj_step 不会自动清零。漏掉这一行，推力会一步步叠加，表现为
        # 重力远大于推力却仍在加速上升——物理上不可能，但数据看起来
        # 只是"发散"，很容易被误判成增益太猛。
        self.data.qfrc_applied[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

        for i, site in enumerate(self._site_ids):
            point = self.data.site_xpos[site].copy()
            mujoco.mj_applyFT(self.model, self.data, thrust_dir * forces[i],
                              np.zeros(3), point, self._drone_id,
                              self.data.qfrc_applied)
            # 转子上的气动阻力矩：始终与**世界系角速度**反向，把转速限制在
            # ω_max = sqrt(τ_max/k_m)。
            #
            # 符号必须由世界系角速度决定，不能由关节 qvel 决定：轴反向的
            # 转子 qvel 为正时世界系角速度为负，按 qvel 取号等于给转子
            # **加**能量，转速会冲到物理上限之外（实测 6755 vs 上限 3004）。
            #
            # 阻力作用在空气上，因此**不对机身**产生反作用——机身在偏航上
            # 只受电机反扭矩，那个由 MuJoCo 的 hinge 关节自动施加，两正两
            # 反的转子使其在悬停时抵消。
            qvel = self.data.qvel[self._dofs[i]]
            omega_z = qvel * self._spin_sign[i]      # 世界系 z 分量
            drag = -math.copysign(1.0, omega_z) * p.k_m * qvel ** 2
            mujoco.mj_applyFT(self.model, self.data, np.zeros(3),
                              np.array([0.0, 0.0, drag]), point,
                              self._rotor_ids[i], self.data.qfrc_applied)
        return forces

    # ------------------------------------------------------------ 闭环运行

    def run(self, duration: float, target, *, yaw_target: float = 0.0,
            record_every: int = 5, v_target=None) -> FlightLog:
        """闭环跑 `duration` 秒。`target` 可以是常向量或 `f(t) -> 向量`。"""
        steps = int(round(duration / self.dt))
        t = 0.0
        for k in range(steps):
            tgt = target(t) if callable(target) else target
            snap = self.step(tgt, t, v_target, yaw_target)
            if k % record_every == 0:
                self.log.steps.append(snap)
            t += self.dt
        return self.log

    # ------------------------------------------------------------ 指标

    def step_response(self, axis: int = 0) -> dict[str, float]:
        """从日志算阶跃响应指标。用于**调参对比**，不是摆设。

        上升时间、超调、稳态误差、调节时间都是"这组增益好不好"的直接
        依据；只报"能飞"等于没调。
        """
        if not self.log.steps:
            return {}
        pos = self.log.positions()
        t = self.log.times()
        final = float(np.mean(pos[-20:, axis]))
        start = float(pos[0, axis])
        span = final - start
        if abs(span) < 1e-9:
            return {"error": 0.0}
        norm = (pos[:, axis] - start) / span
        # 上升时间：10% -> 90%
        i10 = int(np.argmax(norm >= 0.1)) if np.any(norm >= 0.1) else 0
        i90 = int(np.argmax(norm >= 0.9)) if np.any(norm >= 0.9) else len(norm) - 1
        rise = float(t[i90] - t[i10])
        overshoot = float(max(0.0, (norm.max() - 1.0) * 100.0))
        # 调节时间：最后一次进入 ±2% 带的时刻
        inside = np.abs(norm - 1.0) <= 0.02
        if inside.all():
            settle = 0.0
        else:
            bad = np.where(~inside)[0]
            settle = float(t[bad[-1]] - t[0]) if len(bad) else 0.0
        return {"rise_s": rise, "overshoot_pct": overshoot,
                "settling_s": settle,
                "steady_error_m": float(pos[-20:, axis].mean() - final)}
