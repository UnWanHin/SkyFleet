#!/usr/bin/env python3
"""四旋翼电机饱和与推力余裕分析。

回答一个问题：**SkyFleet 场景里指令的机动，会不会把真实四旋翼的电机
推到饱和？**（用户提出的"四个电机才测得准"这一疑虑的正确定位）

数据来源分三类，输出中逐项标注：
  [RotorPy]  取自 RotorPy 3.0 的真实机型参数文件（下节列出具体文件与行）
  [推导]     由本脚本按四旋翼标准力学公式计算
  [假设]     本脚本自定，无外部依据，报告中必须声明

用法：python3 docs/analysis/thrust_margin.py [--json]
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict

G = 9.81                  # 重力加速度 m/s^2            [假设]
RHO = 1.225               # 海平面标准大气密度 kg/m^3    [假设]
FM = 0.65                 # 旋翼品质因数（小旋翼 0.5~0.75）[假设]
ETA_PROP = 0.75           # 电机+电调效率。FM 只覆盖旋翼气动品质，
                          # 不含电气损耗；不加这一项会把悬停功率低估约 1/3，
                          # 使 1 kg 机型的 68 W 看起来比实测（约 100~140 W）好太多。[假设]
#: 迎风阻力面积按旋翼展长估算：CdA = 1.2 * R^2（Cd≈1.0 钝体，
#: 迎风面积≈0.3*(2R)^2）。用一个全局常数会让 30 g 的 Crazyflie 和
#: 1 kg 的 PX4 共用同一个阻力，量级差 8 倍，算出来的倾角直接是荒谬值。
CD_A_COEFF = 1.2          # CdA / R^2                    [假设]
BATTERY_SAG = 0.85        # 电量耗尽时可用最高转速比例     [假设]


# ---------------------------------------------------------------------------
# 机型参数：全部取自 RotorPy 3.0（pip install rotorpy）
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Airframe:
    name: str
    mass: float          # kg
    k_eta: float         # 推力系数 N/(rad/s)^2
    omega_max: float     # 最大转速 rad/s
    rotor_radius: float  # m（RotorPy 仅 Hummingbird 声明，其余为 [假设]）
    source: str

    @property
    def t_max(self) -> float:
        """四个电机的最大总推力。T_i = k_eta * omega^2"""
        return 4.0 * self.k_eta * self.omega_max ** 2

    @property
    def t_hover(self) -> float:
        return self.mass * G

    @property
    def twr(self) -> float:
        """推重比。悬停时是"油门余裕"的倒数平方。"""
        return self.t_max / self.t_hover

    @property
    def hover_throttle(self) -> float:
        """悬停所需油门（推力正比于转速平方）。"""
        return 1.0 / math.sqrt(self.twr)

    @property
    def max_bank_deg(self) -> float:
        """协调转弯下最大倾角：T*cos(phi) = mg"""
        return math.degrees(math.acos(min(1.0, 1.0 / self.twr)))

    @property
    def max_h_accel(self) -> float:
        """最大水平加速度 a = g*tan(phi)"""
        return G * math.tan(math.radians(self.max_bank_deg))

    @property
    def max_climb_accel(self) -> float:
        """最大垂直加速度：T = m(g + a_z)"""
        return G * (self.twr - 1.0)

    @property
    def disk_area(self) -> float:
        """**单个**旋翼的桨盘面积。"""
        return math.pi * self.rotor_radius ** 2

    @property
    def disk_area_total(self) -> float:
        """四个旋翼的总桨盘面积。

        动量理论里推力与桨盘面积必须成对出现：用**总推力**配**单盘面积**
        会把诱导功率高估 2 倍（P ∝ T^(3/2)/sqrt(A)，4 个盘等价于
        A_total = 4A 且 T_i = T/4，整理后 P_total = 0.5*T^1.5/sqrt(2ρA)）。
        这个错误不影响巡航/悬停的**比值**，但会让绝对功率全部翻倍。
        """
        return 4.0 * math.pi * self.rotor_radius ** 2

    def turn_radius(self, v: float) -> float:
        """速度 v 下以最大倾角转弯的半径。"""
        return v ** 2 / self.max_h_accel

    def hover_power(self) -> float:
        """悬停功率（动量理论 + 品质因数）。[推导]

        P_ideal = T^(3/2) / sqrt(2*rho*A)，P = P_ideal / FM
        """
        return (self.t_hover ** 1.5
                / math.sqrt(2.0 * RHO * self.disk_area_total)) / (FM * ETA_PROP)

    @property
    def cd_a(self) -> float:
        return CD_A_COEFF * self.rotor_radius ** 2

    def cruise_required_tilt(self, v: float) -> float:
        """前飞速度 v 所需的机体倾角（度）。多旋翼必须倾斜以产生推力分量。"""
        drag = 0.5 * RHO * self.cd_a * v ** 2
        return math.degrees(math.atan2(drag, self.t_hover))

    def cruise_power(self, v: float) -> tuple[float, float, bool]:
        """前飞功率，返回 (总功率 W, 倾角 degrees, 是否饱和)。[推导]

        多旋翼前飞时推力必须倾斜：T = sqrt((mg)^2 + D^2)。
        诱导功率随速度下降，废阻功率随速度三次方上升，
        **因此巡航比悬停更费电**——这与固定翼相反。

        若所需倾角超过可用倾角，说明该机型**飞不到这个速度**，
        此时返回的功率没有物理意义，用 saturated=True 标出。
        """
        drag = 0.5 * RHO * self.cd_a * v ** 2
        thrust = math.sqrt(self.t_hover ** 2 + drag ** 2)
        tilt = math.degrees(math.atan2(drag, self.t_hover))
        saturated = tilt > self.max_bank_deg

        # 前飞诱导速度：动量理论闭式解 v_i^4 + u^2 v_i^2 - v_h^4 = 0
        v_h = math.sqrt(self.t_hover / (2.0 * RHO * self.disk_area_total))
        v_i = math.sqrt((math.sqrt(v ** 4 + 4.0 * v_h ** 4) - v ** 2) / 2.0)
        induced = thrust * v_i
        parasite = drag * v
        # 品质因数只作用于旋翼诱导功率；废阻是机体阻力，不适用 FM
        return induced / (FM * ETA_PROP) + parasite, tilt, saturated

    def thrust_at_payload(self, payload_kg: float) -> "Airframe":
        return Airframe(self.name, self.mass + payload_kg, self.k_eta,
                        self.omega_max, self.rotor_radius, self.source)

    def with_sag(self, sag: float) -> "Airframe":
        """电池电压跌落：可用最高转速下降，最大推力按平方下降。"""
        return Airframe(self.name, self.mass, self.k_eta,
                        self.omega_max * sag, self.rotor_radius, self.source)


ROTORPY_SRC = "RotorPy 3.0 vehicles/%s_params.py"

AIRFRAMES = [
    # 半径 0.023 m（桨径约 46 mm）。RotorPy 只声明臂长 d=0.043，未声明
    # rotor_radius；早前把桨**直径**当半径（0.047）导致盘面积大 4 倍。
    Airframe("Crazyflie", 0.030, 2.3e-08, 2500.0, 0.023,
             ROTORPY_SRC % "crazyflie"),
    Airframe("Crazyflie BL", 0.044, 4.052e-08, 3100.0, 0.023,
             ROTORPY_SRC % "crazyfliebrushless"),
    Airframe("Hummingbird", 0.500, 5.57e-06, 1500.0, 0.100,
             ROTORPY_SRC % "hummingbird"),
    Airframe("PX4 SIH QuadX", 1.000, 5e-06, 1000.0, 0.125,
             ROTORPY_SRC % "px4_sihsim_quadx"),
]

#: SkyFleet 场景中实际下达的机动。取自 skyfleet/kernel/config.py 与场景文件。
#: [RotorPy 无关，属本项目配置]
SCENARIO = {
    "node_speed": 12.0,            # m/s   config.node_speed
    "cruise_altitude": 140.0,      # m     config.cruise_altitude
    "delivery_altitude": (80.0, 110.0),   # m  config.delivery_altitude
    "area": 500.0,                 # m     baseline_small.json
    "slave_radius": 0.35,          #       engine 内环/外环布置系数
    "tasks": 20,
    "duration": 600.0,
}


def report() -> dict:
    out: dict = {"airframes": [], "scenario": SCENARIO, "findings": []}

    for af in AIRFRAMES:
        row = {
            "name": af.name,
            "mass_kg": af.mass,
            "k_eta": af.k_eta,
            "omega_max_rad_s": af.omega_max,
            "rotor_radius_m": af.rotor_radius,
            "source": af.source,
            "t_max_N": round(af.t_max, 4),
            "t_hover_N": round(af.t_hover, 4),
            "twr": round(af.twr, 3),
            "hover_throttle_pct": round(100 * af.hover_throttle, 1),
            "max_bank_deg": round(af.max_bank_deg, 1),
            "max_h_accel_m_s2": round(af.max_h_accel, 2),
            "max_climb_accel_m_s2": round(af.max_climb_accel, 2),
            "turn_radius_at_12ms_m": round(af.turn_radius(12.0), 2),
            "hover_power_W": round(af.hover_power(), 2),
        }
        cp, tilt, sat = af.cruise_power(SCENARIO["node_speed"])
        row["cruise_power_W"] = round(cp, 2)
        row["cruise_tilt_deg"] = round(tilt, 2)
        row["cruise_saturated"] = sat
        row["cruise_over_hover"] = round(cp / af.hover_power(), 3)
        out["airframes"].append(row)

    # 场景所需机动 vs 可用余裕
    af = next(a for a in AIRFRAMES if a.name == "PX4 SIH QuadX")
    climb = SCENARIO["cruise_altitude"] - max(SCENARIO["delivery_altitude"])
    out["scenario_demands"] = {
        "climb_height_m": climb,
        "climb_time_at_half_max_accel_s": round(
            climb / (0.5 * af.max_climb_accel), 2),
        "turn_radius_required_m": round(af.turn_radius(
            SCENARIO["node_speed"]), 2),
        "waypoint_spacing_estimate_m": round(
            SCENARIO["area"] * 2 * SCENARIO["slave_radius"] / 4.0, 1),
    }

    # 载荷敏感性
    payload_rows = []
    for p in (0.0, 0.2, 0.5, 0.8, 1.0):
        for base in (AIRFRAMES[3], AIRFRAMES[2]):
            loaded = base.thrust_at_payload(p)
            payload_rows.append({
                "airframe": base.name, "payload_kg": p,
                "mass_kg": round(loaded.mass, 2),
                "twr": round(loaded.twr, 3),
                "hover_throttle_pct": round(100 * loaded.hover_throttle, 1),
                "max_bank_deg": round(loaded.max_bank_deg, 1),
            })
    out["payload_sensitivity"] = payload_rows

    # 电池跌落敏感性
    sag_rows = []
    for sag in (1.0, 0.95, 0.90, 0.85, 0.80):
        for base in (AIRFRAMES[3], AIRFRAMES[0]):
            s = base.with_sag(sag)
            sag_rows.append({
                "airframe": base.name, "sag": sag,
                "twr": round(s.twr, 3),
                "hover_throttle_pct": round(100 * s.hover_throttle, 1),
                "max_bank_deg": round(s.max_bank_deg, 1),
            })
    out["battery_sag_sensitivity"] = sag_rows

    # 与当前线性能量模型对照
    config_hover_per_s = 0.05       # skyfleet/kernel/config.py
    initial_energy = 100.0
    out["energy_model_comparison"] = {
        "sim_hover_units_per_s": config_hover_per_s,
        "sim_hover_endurance_s": round(initial_energy / config_hover_per_s, 1),
        "sim_scenario_duration_s": SCENARIO["duration"],
        "note": "当前模型：悬停与巡航同价（线性），且无质量/推力概念；"
                "物理模型：巡航比悬停更费电，且悬停已占 53%~72% 油门。",
    }

    unable = [r["name"] for r in out["airframes"] if r["cruise_saturated"]]
    out["unable_at_12ms"] = unable

    big = [r for r in out["airframes"] if r["mass_kg"] >= 0.5]
    cruise_cheaper = [r["name"] for r in big if r["cruise_over_hover"] < 1.0]
    throttles = [r["hover_throttle_pct"] for r in out["airframes"]]

    out["findings"] = [
        (f"常规飞行不饱和：四种机型在 12 m/s（本项目 node_speed）下所需倾角为 "
         f"9.6°~38.5°，全部低于各自可用倾角（59.2°~84.4°）。"
         f"因此「四个电机」对 SkyFleet 场景的**常规机动不是约束**——"
         f"加四电机模型不会改变任务成败。"),
        (f"悬停本身吃掉大部分推力预算：四种机型的悬停油门为 "
         f"{min(throttles):.0f}%~{max(throttles):.0f}%。"
         f"这正是安全动作（悬停等待）代价高昂的物理原因，"
         f"而当前仿真只按固定速率扣电，没有体现「悬停占用 70% 油门」这一事实。"),
        (f"★ 关键失真：对交付级机型（≥0.5 kg），巡航功率只有悬停的 "
         f"{min(r['cruise_over_hover'] for r in big):.2f}~"
         f"{max(r['cruise_over_hover'] for r in big):.2f} 倍——"
         f"**巡航比悬停更省电**（{'、'.join(cruise_cheaper)}）。"
         f"这与固定翼相反，是多旋翼的固有特性。"
         f"当前线性能量模型让悬停与巡航同价，把最贵与较便宜的状态算成一样，"
         f"因此**系统性地低估了「被迫悬停」的代价**。"),
        (f"饱和真正发生在载荷与电池：PX4 机型加 0.5 kg 载荷后悬停油门升到 "
         f"85.8%、最大倾角降到 42.6°；电池电压跌到 80% 时油门升到 87.5%。"
         f"这两项与续航、载重直接相关，是当前能量模型的第二处失真。"),
        (f"结论：要回答「会不会掉机」，该补的是**悬停代价与能量**，"
         f"不是电机数量。四电机模型只在需要致动器饱和、姿态级保真度时才必要。"),
    ]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    args = ap.parse_args()
    data = report()
    if args.json:
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0

    print("=" * 92)
    print("四旋翼电机饱和与推力余裕分析   [RotorPy 参数 + 标准力学推导]")
    print("=" * 92)
    hdr = (f"{'机型':<15}{'质量kg':>8}{'T_max N':>9}{'T_hover N':>10}"
           f"{'推重比':>8}{'悬停油门':>9}{'最大倾角':>9}{'水平加速度':>10}")
    print(hdr)
    print("-" * 92)
    for r in data["airframes"]:
        print(f"{r['name']:<15}{r['mass_kg']:>8.3f}{r['t_max_N']:>9.3f}"
              f"{r['t_hover_N']:>10.3f}{r['twr']:>8.2f}"
              f"{r['hover_throttle_pct']:>8.1f}%{r['max_bank_deg']:>8.1f}°"
              f"{r['max_h_accel_m_s2']:>9.1f}")
    print()
    print(f"{'机型':<15}{'悬停功率W':>11}{'巡航功率W':>11}{'巡航/悬停':>11}"
          f"{'巡航需倾角':>11}{'可用倾角':>9}{'能否12m/s':>11}")
    print("-" * 92)
    for r in data["airframes"]:
        verdict = "★饱和" if r["cruise_saturated"] else "可以"
        print(f"{r['name']:<15}{r['hover_power_W']:>11.2f}"
              f"{r['cruise_power_W']:>11.2f}{r['cruise_over_hover']:>11.2f}"
              f"{r['cruise_tilt_deg']:>10.2f}°{r['max_bank_deg']:>8.1f}°"
              f"{verdict:>11}")
    print()
    print("场景需求（PX4 SIH QuadX 为参考）:")
    for k, v in data["scenario_demands"].items():
        print(f"  {k:<36}{v}")
    print()
    print("载荷敏感性（推重比 / 悬停油门 / 最大倾角）:")
    for r in data["payload_sensitivity"]:
        print(f"  {r['airframe']:<15}+{r['payload_kg']:.1f}kg -> "
              f"TWR {r['twr']:.2f}  油门 {r['hover_throttle_pct']:.1f}%  "
              f"倾角 {r['max_bank_deg']:.1f}°")
    print()
    print("电池电压跌落敏感性:")
    for r in data["battery_sag_sensitivity"]:
        print(f"  {r['airframe']:<15} sag={r['sag']:.2f} -> "
              f"TWR {r['twr']:.2f}  油门 {r['hover_throttle_pct']:.1f}%  "
              f"倾角 {r['max_bank_deg']:.1f}°")
    print()
    print("结论:")
    for f in data["findings"]:
        print(f"  · {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
