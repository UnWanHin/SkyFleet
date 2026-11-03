"""遥测：把仿真的真实状态打包成指令台能读的一帧。

**为什么单独成一个模块**：遥测是「仿真 → 外部」的唯一出口。把它写进 sim
或 cli 里，外部一旦要读数据就得依赖仿真内部字段名，改一个属性名就断。
这里定义一个稳定的帧结构，仿真怎么改都不影响消费方。

**内环与遥测的关系**：内环（读转速 → ESC 算力矩 → 再积分）在每一物理步里
跑，这里只是**抽取**某一时刻的结果。遥测频率（20 Hz）远低于物理步频
（500 Hz），所以它是采样，不是控制回路的一部分。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from flightlab.control import rotation_to_quat_wxyz
from flightlab.params import QuadParams


@dataclass(frozen=True)
class TelemetryFrame:
    """一帧遥测。

    字段全部是**物理量**，不是内部索引：让指令台无需知道 mujoco 的
    qpos/qvel 布局。
    """

    t: float
    #: 世界坐标位置 m
    pos: np.ndarray
    #: 机体姿态，wxyz 顺序的四元数
    quat: np.ndarray
    #: 机体角速度 rad/s（机体系）
    omega: np.ndarray
    #: 四个电机的实际转速 rad/s
    rotor_speeds: np.ndarray
    #: 四个电机的实际推力 N
    thrust: np.ndarray
    #: 当前飞行目标 m
    target: np.ndarray
    #: 飞行模式：TRACK / HOLD / RETURN
    mode: str = "TRACK"

    @property
    def thrust_total(self) -> float:
        return float(self.thrust.sum())

    @property
    def tilt_deg(self) -> float:
        """机体 z 轴与竖直方向的夹角，度。"""
        # 四元数 wxyz → 旋转矩阵第三列
        w, x, y, z = self.quat
        b3z = 1.0 - 2.0 * (x * x + y * y)
        return float(np.degrees(np.arccos(np.clip(b3z, -1.0, 1.0))))

    def as_row(self, params: QuadParams) -> str:
        """压成一行给终端显示。"""
        p = self.pos
        sp = self.rotor_speeds
        return (f"t={self.t:6.2f}s  "
                f"pos=({p[0]:6.3f},{p[1]:6.3f},{p[2]:6.3f})  "
                f"tilt={self.tilt_deg:5.1f}°  "
                f"ω=({sp[0]:4.0f},{sp[1]:4.0f},{sp[2]:4.0f},{sp[3]:4.0f})  "
                f"F={self.thrust_total*1000:6.1f}mN  "
                f"{self.mode}")


def sample_telemetry(sim, t: float, *, target=None, mode: str = "TRACK"
                     ) -> TelemetryFrame:
    """从仿真抽一帧遥测。

    `target` 不传时用仿真的当前位置填充——指令台在 HOLD 模式下不该看到
    一个会漂走的目标值。
    """
    speeds = sim.rotor_speeds()
    thrust = sim.params.k_f * speeds ** 2
    return TelemetryFrame(
        t=float(t),
        pos=np.asarray(sim.pos, dtype=float).copy(),
        quat=rotation_to_quat_wxyz(np.asarray(sim.R, dtype=float)),
        omega=np.asarray(sim.omega, dtype=float).copy(),
        rotor_speeds=np.asarray(speeds, dtype=float).copy(),
        thrust=np.asarray(thrust, dtype=float).copy(),
        target=(np.asarray(sim.pos, dtype=float).copy() if target is None
                else np.asarray(target, dtype=float).copy()),
        mode=mode,
    )
