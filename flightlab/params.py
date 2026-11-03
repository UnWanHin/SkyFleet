"""四旋翼物理参数（与 quadrotor.xml 严格对应）。

所有数值都能追溯到模型文件或明确标注为假设，不留"魔法数字"：
报告里写"推力余裕 2.57 倍"必须能当场算出 2.57 从哪来。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class QuadParams:
    # ---- 几何（读自 XML）----
    arm_xy: float = 0.11          # 电机在机体系 x/y 的偏移 m
    rotor_radius: float = 0.045   # 桨半径 m

    # ---- 质量（读自 XML）----
    mass_body: float = 0.027      # 机身（含机臂、起落架）kg
    mass_rotor: float = 0.0004    # 单个转子 kg

    # ---- 气动 ----
    #: 推力系数 T = k_f * ω² 。选值依据：让 TWR 落在小型四旋翼常见区间。
    k_f: float = 2.0e-8           # N/(rad/s)^2
    #: 反扭矩系数 τ_drag = k_m * ω²，k_m/k_f 有长度量纲（≈0.0666 m）。
    k_m: float = 1.33e-9          # N·m/(rad/s)^2
    #: 电机最大力矩。决定可达转速上限 ω_max = sqrt(τ_max/k_m)。
    torque_max: float = 1.2e-2    # N·m

    g: float = 9.81

    @property
    def mass(self) -> float:
        """整机质量：机身 + 四个转子。"""
        return self.mass_body + 4.0 * self.mass_rotor

    @property
    def arm_length(self) -> float:
        """电机到机体中心的距离 m。"""
        return math.hypot(self.arm_xy, self.arm_xy)

    @property
    def omega_hover(self) -> float:
        """单电机悬停转速 rad/s。"""
        return math.sqrt(self.hover_thrust_per_rotor / self.k_f)

    @property
    def hover_thrust_per_rotor(self) -> float:
        return self.mass * self.g / 4.0

    @property
    def omega_max(self) -> float:
        """力矩上限对应的稳态转速（气动阻力与电机力矩平衡处）。"""
        return math.sqrt(self.torque_max / self.k_m)

    @property
    def thrust_max_per_rotor(self) -> float:
        return self.k_f * self.omega_max ** 2

    @property
    def thrust_to_weight(self) -> float:
        """推重比。小于 1 表示根本飞不起来。"""
        return 4.0 * self.thrust_max_per_rotor / (self.mass * self.g)

    @property
    def hover_throttle(self) -> float:
        """悬停时的电机力矩占最大力矩的比例（不是转速比例）。"""
        return self.k_m * self.omega_hover ** 2 / self.torque_max

    def summary(self) -> dict[str, float]:
        return {
            "质量 kg": self.mass,
            "臂长 m": self.arm_length,
            "悬停转速 rad/s": self.omega_hover,
            "最大转速 rad/s": self.omega_max,
            "单桨最大推力 N": self.thrust_max_per_rotor,
            "推重比": self.thrust_to_weight,
            "悬停油门": self.hover_throttle,
        }


DEFAULT = QuadParams()
