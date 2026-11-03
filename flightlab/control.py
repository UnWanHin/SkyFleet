"""四旋翼串级控制：位置环 -> 姿态环 -> 混控 -> 电机转速环。

**为什么是串级**：四旋翼是欠驱动系统——四个电机只能产生沿机体 z 轴的
合推力与三个力矩，共四个输入，却要控制六维位姿。因此位置不能直接控制，
只能通过**倾斜机身**来产生水平加速度。这个物理事实决定了控制结构必须
是"位置环算期望姿态、姿态环算力矩"，而不是两个独立 PID。

层级与本模块的对应：

    PositionController   位置 PD -> 期望推力矢量（世界系）
        ↓
    desired_attitude     推力方向 + 期望偏航 -> 期望旋转矩阵 R_des
        ↓
    AttitudeController   几何 SE3 控制 -> 期望力矩（机体系）
        ↓
    mixer                总推力 + 三轴力矩 -> 四个电机推力
        ↓
    motor_speeds         推力 -> 期望转速
        ↓
    ESCController        转速误差 -> 电机力矩（MuJoCo 里是真的关节力矩）

**参考**：姿态环采用几何 SE3 控制律（Lee, Leok & McClamroch 的
`e_R = ½(R_desᵀR − RᵀR_des)` 形式），与 RotorPy 的 `SE3Control` 同一套
思路。这里自己实现是为了让**每一级都能单独测、单独调**，而不是把
控制器当黑箱。差异：RotorPy 直接输出电机转速，这里多了一层带惯量的
转速环，能看出电机响应滞后。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from flightlab.params import QuadParams


# ---------------------------------------------------------------- 工具

def vee(m: np.ndarray) -> np.ndarray:
    """SO(3) 反对称矩阵的逆映射。"""
    return np.array([m[2, 1] - m[1, 2],
                     m[0, 2] - m[2, 0],
                     m[1, 0] - m[0, 1]]) * 0.5


def rotation_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数 (w, x, y, z)，供 MuJoCo 写状态用。"""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def desired_attitude(thrust_vec: np.ndarray, yaw: float) -> np.ndarray:
    """由期望推力方向 + 期望偏航角构造期望旋转矩阵。

    列向量 b1/b2/b3 分别是机体系 x/y/z 轴在世界系的表示。
    b3 必须对齐推力方向——四旋翼只能沿机体 z 轴推——其余两轴由偏航
    约束补足。这就是"欠驱动"在控制器里的具体体现。
    """
    norm = np.linalg.norm(thrust_vec)
    if norm < 1e-9:
        # 推力为零时姿态无定义，退化为当前偏航的水平姿态
        b3 = np.array([0.0, 0.0, 1.0])
    else:
        b3 = thrust_vec / norm
    c1 = np.array([math.cos(yaw), math.sin(yaw), 0.0])   # 期望机头方向
    b2 = np.cross(b3, c1)
    n2 = np.linalg.norm(b2)
    if n2 < 1e-6:            # 机头与推力共线（垂直指向），偏航不可定义
        c1 = np.array([1.0, 0.0, 0.0])
        b2 = np.cross(b3, c1)
        n2 = np.linalg.norm(b2)
    b2 = b2 / n2
    b1 = np.cross(b2, b3)
    return np.column_stack((b1, b2, b3))


# ---------------------------------------------------------------- 参数

@dataclass(frozen=True)
class Gains:
    """全部可调的增益。默认值经调参得到，见 `docs/flight/tuning.md`。"""

    # 位置环（世界系，输出期望加速度）
    #:
    #: ★ 增益一律**按带宽反推**，不凭手感。PD 写成 a = kd(kp·e − v)，
    #: 于是 ωn = √(kp·kd)、ζ = kd/(2√(kp·kd))。取 ωn=2.5、ζ=0.8 得
    #: kp=1.56、kd=4.0。
    #:
    #: 早先取 kp=6、kd=4.2（ωn=5.0、ζ=0.42）的后果：1.5 m 的航点一步就
    #: 要求 12.6 m/s² 的水平加速度，即 **52° 倾角**，姿态环在大误差下饱和，
    #: 飞行器翻转到 162° 并飞出去 238 m。
    kp_pos: float = 1.562
    kd_pos: float = 4.0
    #: 期望速度限幅 m/s。这个值直接决定最大倾角，不只是"安全限幅"。
    v_max: float = 1.5
    #: 水平加速度指令限幅 m/s²。tan⁻¹(3/9.81)=17°，留足姿态环的跟踪余量，
    #: 也保证所需总推力远低于上限。
    a_max_horizontal: float = 3.0

    # 姿态环（机体系，输出力矩）
    #: ωn=15，由 I_yy≈5.1e-5 kg·m² 反推：k_R=I·ωn²、k_ω=2ζI·ωn。
    #:
    #: k_ω 取 ζ≈1.8（比临界阻尼更硬）不是笔误，是扫描实测的结果：
    #: 按 ζ=0.9 算出的 0.00138 在阶跃上超调 0.9%、调节 1.588 s；
    #: 提到 0.0028 后超调降到 0.0%、调节 1.482 s，而且完整航线上**最大
    #: 角速度反而从 3.69 降到 2.62 rad/s**——阻尼足，姿态不用来回甩。
    #: 也就是说这是"更快 + 更平滑"的纯收益，不是速度换平稳。
    k_R: float = 0.0115
    k_omega: float = 0.0028

    # 电机转速环（ESC）
    #: 转速环比例增益 N·m/(rad/s)。由于下面加了**前馈**，这个值只负责
    #: 动态修正，不再承担提供稳态力矩的职责，因此可以取小。
    #:
    #: 历史坑：没有前馈时只能靠加大 kp 来消稳态误差，而 kp 一大，几十
    #: rad/s 的误差就把力矩打满，转速环退化成 bang-bang；姿态环（ωn=15）
    #: 与这个非线性环节组成极限环，振幅从 5 rad/s 一路涨到 172 rad/s。
    #: 增益再调也调不出来——问题在**结构**，不在数值。
    kp_motor: float = 5.0e-5
    #: 电机力矩变化率上限 N·m/s。电调不是理想力矩源，但也不能太慢：
    #: 取 0.3 时满力矩建立时间 40 ms，与姿态环 67 ms 的时间常数匹配。
    torque_rate_max: float = 3.0e-1

    def describe(self) -> dict[str, float]:
        return {
            "kp_pos": self.kp_pos, "kd_pos": self.kd_pos,
            "v_max": self.v_max, "a_max_h": self.a_max_horizontal,
            "k_R": self.k_R,
            "k_omega": self.k_omega, "kp_motor": self.kp_motor,
        }


# ---------------------------------------------------------------- 控制器

class CascadeController:
    """完整串级控制器。一次 `compute()` 给出四个电机的目标转速。"""

    def __init__(self, params: QuadParams, gains: Gains | None = None) -> None:
        self.p = params
        self.gains = gains or Gains()
        # 混控矩阵：把 (总推力, τx, τy, τz) 映射到四个电机推力
        #
        # ★ 偏航行的符号容易想反，推导如下：机身受到的偏航力矩是**转子气动
        # 阻力的反作用**。CCW 转子（0/1）阻力朝 −z，机身感受 −k_m·ω²；
        # CW 转子（2/3）阻力朝 +z，机身感受 +k_m·ω²。于是
        #     τ_z = k_m(ω₂²+ω₃²) − k_m(ω₀²+ω₁²) = c(f₂+f₃−f₀−f₁)
        # 也就是说**要往正方向偏航，得让 CW 那对转得更快**。写成
        # c(f₀+f₁−f₂−f₃) 会得到与物理相反的符号，姿态环变成正反馈：
        # 一个 1e-6 rad/s 的数值扰动会被放大到 2.5 rad/s 直至发散。
        L, c = params.arm_xy, params.k_m / params.k_f
        self._mixer = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [L, -L, -L, L],
            [-L, L, -L, L],
            [-c, -c, c, c],
        ])
        self._mixer_inv = np.linalg.inv(self._mixer)

    # ---- 位置环 ----
    def desired_acceleration(self, p: np.ndarray, v: np.ndarray,
                             p_des: np.ndarray, v_des: np.ndarray
                             ) -> tuple[np.ndarray, np.ndarray]:
        g = self.gains
        e_p = p_des - p
        # 速度指令限幅：误差大时如果直接乘 kp，会要求一个远超 TWR 的
        # 倾角，控制器输出饱和、积分(此处无)不起作用，表现为"撞天花板"。
        v_ref = v_des + g.kp_pos * e_p
        speed = np.linalg.norm(v_ref)
        if speed > g.v_max:
            v_ref = v_ref / speed * g.v_max
        a = g.kd_pos * (v_ref - v)
        # 水平加速度限幅：这是**姿态参考可实现性**的保证，不是保守起见。
        # 不限幅时位置环会一步要求几十度倾角，姿态环追不上而饱和。
        a_h = a[:2]
        norm = float(np.linalg.norm(a_h))
        if norm > g.a_max_horizontal:
            a = a.copy()
            a[:2] = a_h / norm * g.a_max_horizontal
        return a, v_ref

    # ---- 姿态环 ----
    def attitude_torque(self, R: np.ndarray, omega: np.ndarray,
                        R_des: np.ndarray) -> np.ndarray:
        g = self.gains
        e_R = vee(R_des.T @ R - R.T @ R_des)
        e_omega = omega.copy()          # 期望角速度取 0（悬停/慢速跟踪）
        return -g.k_R * e_R - g.k_omega * e_omega

    # ---- 混控 ----
    def mix(self, total_thrust: float, torque: np.ndarray
            ) -> tuple[np.ndarray, bool]:
        """返回 (每桨推力, 是否饱和)。

        饱和判定必须**显式返回**而不是静默钳位：推力打到上下限时控制器
        已经失去一部分权限，报告里"为什么跟不上"要能查到这一位。
        """
        wrench = np.array([total_thrust, torque[0], torque[1], torque[2]])
        f = self._mixer_inv @ wrench
        lo, hi = 0.0, self.p.thrust_max_per_rotor
        saturated = bool(np.any(f < lo) or np.any(f > hi))
        return np.clip(f, lo, hi), saturated

    def motor_speeds(self, rotor_thrust: np.ndarray) -> np.ndarray:
        """每桨推力 -> 期望转速。推力与转速是**平方**关系。"""
        return np.sqrt(np.maximum(rotor_thrust, 0.0) / self.p.k_f)

    def compute(self, p, v, R, omega, p_des, v_des, yaw_des=0.0
                ) -> tuple[np.ndarray, dict]:
        """一次完整串级解算。返回 (四个期望转速, 诊断信息)。"""
        a, v_ref = self.desired_acceleration(p, v, p_des, v_des)
        # 期望推力矢量：抵消重力 + 提供所需加速度
        thrust_vec = self.p.mass * (a + np.array([0.0, 0.0, self.p.g]))
        R_des = desired_attitude(thrust_vec, yaw_des)
        # 总推力是期望推力矢量在**当前**机体 z 轴上的投影——机身没转到
        # 位时只能出到这个分量，这正是欠驱动的表现。
        b3 = R[:, 2]
        total = float(np.dot(thrust_vec, b3))
        total = max(total, 0.0)
        torque = self.attitude_torque(R, omega, R_des)
        rotor_thrust, saturated = self.mix(total, torque)
        speeds = self.motor_speeds(rotor_thrust)
        diag = {
            "R_des": R_des, "thrust_vec": thrust_vec, "total": total,
            "torque": torque, "rotor_thrust": rotor_thrust,
            "saturated": saturated, "v_ref": v_ref,
        }
        return speeds, diag


class ESCController:
    """电机转速环：转速误差 -> 关节力矩。

    仿真里这四个力矩**真的**作用在带转动惯量的转子关节上，由 MuJoCo
    积分。因此能看到转速上升的滞后、力矩速率限制带来的响应延迟——这些
    在"直接设推力"的简化模型里是看不到的。
    """

    def __init__(self, params: QuadParams, gains: Gains) -> None:
        self.p = params
        self.gains = gains
        self._last_torque = np.zeros(4)

    def reset(self) -> None:
        self._last_torque = np.zeros(4)

    def compute(self, omega_actual: np.ndarray, omega_des: np.ndarray,
                dt: float) -> np.ndarray:
        g = self.gains
        # 前馈：维持该转速所需的稳态力矩就是气动阻力矩 k_m·ω²。
        # 直接算出来，转速环就只剩"修正"职责，不必用高增益去硬顶稳态误差。
        feedforward = self.p.k_m * omega_des ** 2
        torque = feedforward + g.kp_motor * (omega_des - omega_actual)
        # 电调不是理想力矩源：力矩有变化率上限
        max_step = g.torque_rate_max * dt
        torque = np.clip(torque,
                         self._last_torque - max_step,
                         self._last_torque + max_step)
        torque = np.clip(torque, 0.0, self.p.torque_max)
        self._last_torque = torque
        return torque
