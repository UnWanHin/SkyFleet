"""调参：把"这组增益好不好"变成可量化的数字。

**为什么需要专门的模块**：控制器的增益有六个，互相耦合，靠肉眼看着"好像
还行"是最不可靠的判断方式。这里对每组增益跑同一条阶跃指令，输出上升时间、
超调量、调节时间、稳态误差，让"调参"变成可比对的表格而不是感觉。

**为什么不自动搜索最优**：真实工程的增益受执行器饱和、噪声、抗扰能力约束，
单纯最小化调节时间会得到一组在数值仿真里好看、在实物上一碰就振的增益。
所以这里只做**受控对比**：一次改一个参数，看指标往哪走。这正是调参时
人做的事，只是把观察变成了数字。
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import numpy as np

from flightlab.control import Gains
from flightlab.params import QuadParams
from flightlab.params import DEFAULT as DEFAULT_PARAMS
from flightlab.sim import QuadrotorSim


@dataclass(frozen=True)
class Metrics:
    """一条阶跃响应的四项指标。"""

    rise_s: float          # 10% -> 90%
    overshoot_pct: float   # 超过终值的百分比
    settling_s: float      # 最后一次离开 ±2% 带的时刻
    steady_err_m: float    # 末段均值与目标的偏差
    saturated_ratio: float # 饱和步数占比，暴露"靠打满力矩硬顶"

    def as_row(self) -> str:
        return (f"{self.rise_s:7.3f} {self.overshoot_pct:9.1f} "
                f"{self.settling_s:9.3f} {self.steady_err_m:11.4f} "
                f"{self.saturated_ratio*100:8.1f}%")


def measure_step(gains: Gains, *, axis: str = "z", distance: float = 1.0,
                 duration: float = 6.0, params: QuadParams = DEFAULT_PARAMS,
                 settle_band: float = 0.02) -> Metrics:
    """跑一次阶跃并算指标。

    `axis` 取 "z" 是纯升降（考验推力通道），"x" 是水平（考验姿态环与
    欠驱动耦合）——两者对增益的敏感度完全不同，必须分开看。
    """
    sim = QuadrotorSim(params=params, gains=gains)
    start = np.array([0.0, 0.0, 1.0])
    sim.reset(pos=start, rotor_speed=params.omega_hover)
    target = start.copy()
    target[{"x": 0, "y": 1, "z": 2}[axis]] += distance

    steps = int(round(duration / sim.dt))
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    values = np.empty(steps)
    saturated = 0
    for k in range(steps):
        snap = sim.step(target, k * sim.dt)
        values[k] = snap.pos[idx]
        saturated += int(snap.saturated)

    t = np.arange(steps) * sim.dt
    final = float(values[-int(steps * 0.05):].mean())
    span = final - start[idx]
    if abs(span) < 1e-9:
        return Metrics(0.0, 0.0, 0.0, 0.0, 0.0)
    norm = (values - start[idx]) / span

    i10 = int(np.argmax(norm >= 0.1)) if np.any(norm >= 0.1) else 0
    i90 = int(np.argmax(norm >= 0.9)) if np.any(norm >= 0.9) else steps - 1
    rise = float(t[i90] - t[i10])
    overshoot = float(max(0.0, (norm.max() - 1.0) * 100.0))

    inside = np.abs(norm - 1.0) <= settle_band
    if inside.all():
        settle = 0.0
    else:
        bad = np.where(~inside)[0]
        settle = float(t[bad[-1]]) if len(bad) else 0.0

    return Metrics(rise_s=rise, overshoot_pct=overshoot, settling_s=settle,
                   steady_err_m=abs(final - target[idx]),
                   saturated_ratio=saturated / steps)


def sweep(parameter: str, values: list[float], *,
          axis: str = "z", distance: float = 1.0, duration: float = 6.0,
          base: Gains | None = None) -> list[tuple[float, Metrics]]:
    """固定其他参数，只改一个，返回 [(取值, 指标)]。

    一次只改一个变量是调参的基本纪律：同时改两个参数，指标变好时你无法
    判断是哪个起了作用，也无法判断有没有互相抵消。
    """
    base = base or Gains()
    if not hasattr(base, parameter):
        raise AttributeError(f"Gains 没有字段 {parameter!r}")
    out: list[tuple[float, Metrics]] = []
    for v in values:
        g = dataclasses.replace(base, **{parameter: v})
        out.append((v, measure_step(g, axis=axis, distance=distance,
                                    duration=duration)))
    return out


def format_sweep(parameter: str, results: list[tuple[float, Metrics]]) -> str:
    header = (f"  {parameter:14s} {'上升s':>7} {'超调%':>9} "
              f"{'调节s':>9} {'稳态误差m':>11} {'饱和占比':>9}")
    lines = [f"参数 {parameter}（其余固定）", header]
    for value, m in results:
        lines.append(f"  {value:<14.6g} {m.as_row()}")
    return "\n".join(lines)


#: 默认扫描计划。每个参数扫三个值，覆盖"偏软 / 当前值 / 偏硬"。
#:
#: 默认用 **x 轴**（水平机动）而不是 z 轴，这是一次实测得出的结论：
#: 纯升降不需要倾斜机身，姿态环完全不参与，k_R / k_omega 扫出来的三行
#: **一模一样**——那样扫了等于没扫。水平机动才会把姿态环暴露出来。
DEFAULT_SWEEPS: dict[str, list[float]] = {
    "k_R": [0.006, 0.0115, 0.023],
    "k_omega": [0.0007, 0.00138, 0.0028],
    "kp_pos": [0.9, 1.562, 3.0],
    "kd_pos": [2.4, 4.0, 6.4],
    "v_max": [0.8, 1.5, 2.5],
    "a_max_horizontal": [1.5, 3.0, 6.0],
}


def run_default_study(axis: str = "x") -> str:
    """跑完整默认扫描并返回 Markdown 表格。"""
    chunks = [f"阶跃方向：{axis} 轴，距离 1.0 m。", ""]
    for name, values in DEFAULT_SWEEPS.items():
        results = sweep(name, values, axis=axis)
        best = min(results, key=lambda r: (r[1].settling_s,
                                           r[1].overshoot_pct))
        chunks.append(f"```\n{format_sweep(name, results)}\n```")
        chunks.append(f"调节时间最短：{name} = {best[0]:.6g}"
                      f"（{best[1].settling_s:.3f} s，"
                      f"超调 {best[1].overshoot_pct:.1f}%）")
        chunks.append("")
    return "\n".join(chunks)
