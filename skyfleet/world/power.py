"""能量模型：把飞行状态映射为功率消耗。

背景见 `docs/analysis/2026-10-05-thrust-margin-analysis.md`。要点：

引擎原先对每个存活节点**无条件**扣 `energy_hover_per_s * dt`，与它在
巡航、悬停、爬升还是下降无关——那是一个时钟，不是能量模型。物理上
**悬停才是最贵的工作状态**（对交付级机型，巡航功率只有悬停的
0.60~0.65 倍），把两者算成同价会系统性低估"被迫悬停"的代价。

本模块提供两种可替换的实现：

- `LinearEnergyModel` —— 既有行为，逐位不变，作为默认值保留
- `PhysicalEnergyModel` —— 四旋翼动量理论，按状态计费

**默认仍是 `linear`**：换成物理模型会改变所有已发表的扫描结果与
golden digest，那应当是一次有意识的重新基线，而不是顺手改掉。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

#: 重力加速度 m/s^2
G = 9.81
#: 海平面标准大气密度 kg/m^3
RHO = 1.225

#: 旋翼品质因数。小旋翼典型 0.5~0.75。
DEFAULT_FIGURE_OF_MERIT = 0.65
#: 电机 + 电调效率。品质因数只覆盖旋翼气动品质，不含电气损耗；
#: 不加这一项会把悬停功率低估约 1/3。
DEFAULT_PROPULSION_EFFICIENCY = 0.75
#: 迎风阻力面积系数：CdA = coeff * R^2（Cd≈1.0 钝体，迎风面积≈0.3*(2R)^2）
DEFAULT_CD_A_COEFF = 1.2

#: 下降时诱导功率的下限（相对悬停诱导功率）。
#: 动量理论在涡环状态（-2*v_h < v_z < 0）失效，不加下限会出现
#: "下降反而发电"的荒谬结果。0.3 是保守取值。
DEFAULT_DESCENT_FLOOR = 0.30


@dataclass(frozen=True)
class Airframe:
    """四旋翼机型参数。

    质量与推力系数取自 RotorPy 3.0 的机型文件（可 `pip install rotorpy`
    复核），`rotor_radius_m` 除 Hummingbird 外 RotorPy 未声明，本处为
    **假设值**——因此这些机型的功率绝对值不可直接引用，只有比值可靠。
    """

    name: str
    mass_kg: float
    k_eta: float          # 推力系数 N/(rad/s)^2
    omega_max_rad_s: float
    rotor_radius_m: float
    num_rotors: int = 4
    #: 典型电池容量 Wh。**必须按机型给**：给 30 g 的 Crazyflie 配
    #: 100 Wh 电池（真实约 0.9 Wh）会让它的能耗数字完全失去意义。
    typical_battery_wh: float = 50.0
    #: 旋翼品质因数。微型旋翼（Re < 10^4）的气动品质远低于大旋翼，
    #: 用单一大旋翼的值会高估小机型的效率数倍。
    figure_of_merit: float = 0.65

    @property
    def disk_area_total(self) -> float:
        """总桨盘面积。诱导功率必须用**总**盘面积配总推力，
        否则会高估 2 倍（P ∝ T^1.5/sqrt(A)）。"""
        return self.num_rotors * math.pi * self.rotor_radius_m ** 2

    @property
    def max_thrust_n(self) -> float:
        return self.num_rotors * self.k_eta * self.omega_max_rad_s ** 2

    def hover_thrust_n(self, payload_kg: float = 0.0) -> float:
        return (self.mass_kg + payload_kg) * G

    def thrust_to_weight(self, payload_kg: float = 0.0) -> float:
        return self.max_thrust_n / self.hover_thrust_n(payload_kg)

    def hover_throttle(self, payload_kg: float = 0.0) -> float:
        """悬停油门。推力正比于转速**平方**，故为 1/sqrt(TWR)。"""
        return 1.0 / math.sqrt(self.thrust_to_weight(payload_kg))

    def max_bank_deg(self, payload_kg: float = 0.0) -> float:
        """协调转弯可用最大倾角：T*cos(phi) = mg。"""
        return math.degrees(math.acos(
            min(1.0, 1.0 / self.thrust_to_weight(payload_kg))))

    def required_tilt_deg(self, v_h: float, payload_kg: float = 0.0,
                          cd_a_coeff: float = DEFAULT_CD_A_COEFF) -> float:
        """前飞速度 v_h 所需的机体倾角。"""
        drag = self._drag_n(v_h, cd_a_coeff)
        return math.degrees(math.atan2(drag, self.hover_thrust_n(payload_kg)))

    def _drag_n(self, v_h: float, cd_a_coeff: float) -> float:
        return 0.5 * RHO * cd_a_coeff * self.rotor_radius_m ** 2 * v_h ** 2

    def is_saturated(self, v_h: float, v_z: float,
                     payload_kg: float = 0.0,
                     cd_a_coeff: float = DEFAULT_CD_A_COEFF) -> bool:
        """该速度是否超出了可用推力（倾角超限或爬升加速度不足）。

        饱和时功率不再是"更大的数"，而是**根本达不到这个状态**；
        物理模型据此把功率钳到最大推力对应的值，避免输出没有物理
        意义的数字。
        """
        if self.required_tilt_deg(v_h, payload_kg, cd_a_coeff) > \
                self.max_bank_deg(payload_kg):
            return True
        # 爬升需要的推力：T = m(g + a_z)；这里只看准稳态的爬升率代价
        thrust_needed = self.hover_thrust_n(payload_kg) + \
            self._drag_n(v_h, cd_a_coeff)
        return thrust_needed > self.max_thrust_n


#: RotorPy 3.0 机型。radius 标注 (假设) 的为本文档补值，非 RotorPy 声明。
AIRFRAMES: dict[str, Airframe] = {
    # 微型机：电池与品质因数都是 [假设]。动量理论在这一尺度上**已知
    # 偏乐观**——实测 Crazyflie 悬停约 7 分钟、约 6~8 W，本模型给出
    # 的悬停功率低于此，差距来自微尺度电机与旋翼效率，模型不试图修正。
    # 这类机型可用于对比**趋势**，功率绝对值不可引用。
    "crazyflie": Airframe("Crazyflie", 0.030, 2.3e-08, 2500.0, 0.047,
                          typical_battery_wh=0.9, figure_of_merit=0.35),
    "crazyflie_brushless": Airframe(
        "Crazyflie Brushless", 0.044, 4.052e-08, 3100.0, 0.047,
        typical_battery_wh=1.9, figure_of_merit=0.40),
    # 交付级：本模型适用的尺度
    "hummingbird": Airframe("Hummingbird", 0.500, 5.57e-06, 1500.0, 0.100,
                            typical_battery_wh=24.0, figure_of_merit=0.65),
    "px4_sihsim_quadx": Airframe(
        "PX4 SIH QuadX", 1.000, 5e-06, 1000.0, 0.125,
        typical_battery_wh=50.0, figure_of_merit=0.65),
}


class EnergyModel(Protocol):
    """按飞行状态计费。"""

    def energy_for_step(self, v_h: float, v_z: float, dt: float) -> float:
        """返回 dt 秒内消耗的电量（引擎内部单位）。"""
        ...


class LinearEnergyModel:
    """既有模型：与飞行状态无关的恒定扣费。

    **逐位保留**，作为默认值——换成物理模型会改变所有已发表数字。
    """

    def __init__(self, hover_per_s: float) -> None:
        self.hover_per_s = hover_per_s

    def energy_for_step(self, v_h: float, v_z: float, dt: float) -> float:
        return self.hover_per_s * dt


class PhysicalEnergyModel:
    """四旋翼动量理论功率模型。

    功率由三部分构成：

    - **诱导功率** `P_i = T * (v_z + v_i)`：产生推力必须向下加速气流。
      前飞使诱导速度下降，爬升使其上升。
    - **废阻功率** `P_p = D * v_h`：克服机体阻力。
    - 两者之和除以 `品质因数 * 动力系统效率` 得到电功率。

    推力按准稳态取 `T = sqrt((mg)^2 + D^2)`——多旋翼前飞必须倾斜，
    这是"巡航比悬停省电"的根源。
    """

    def __init__(self, airframe: Airframe, battery_capacity_wh: float = 100.0,
                 payload_kg: float = 0.0,
                 figure_of_merit: float = DEFAULT_FIGURE_OF_MERIT,
                 propulsion_efficiency: float = DEFAULT_PROPULSION_EFFICIENCY,
                 cd_a_coeff: float = DEFAULT_CD_A_COEFF,
                 descent_floor: float = DEFAULT_DESCENT_FLOOR) -> None:
        if battery_capacity_wh <= 0.0:
            raise ValueError("battery_capacity_wh 必须为正")
        self.airframe = airframe
        self.battery_capacity_wh = battery_capacity_wh
        self.payload_kg = payload_kg
        self.figure_of_merit = figure_of_merit
        self.propulsion_efficiency = propulsion_efficiency
        self.cd_a_coeff = cd_a_coeff
        self.descent_floor = descent_floor

    # ---------- 功率 ----------

    def _induced_fwd(self, thrust: float, drag: float, v_h: float) -> float:
        """前飞诱导速度：动量理论闭式解 v_i^4 + u^2 v_i^2 - v_h^4 = 0。"""
        v_hov = math.sqrt(thrust / (2.0 * RHO
                                    * self.airframe.disk_area_total))
        return math.sqrt((math.sqrt(v_h ** 4 + 4.0 * v_hov ** 4) - v_h ** 2)
                         / 2.0)

    def power_w(self, v_h: float, v_z: float) -> float:
        """给定水平速度与前向爬升率，返回电功率（瓦）。"""
        af = self.airframe
        drag = 0.5 * RHO * self.cd_a_coeff * af.rotor_radius_m ** 2 * v_h ** 2
        weight = af.hover_thrust_n(self.payload_kg)
        thrust = math.sqrt(weight ** 2 + drag ** 2)

        # 饱和时钳到最大推力：此时该状态物理上无法维持，
        # 报一个"更大的功率"是误导，钳位并让调用方看见。
        if thrust > af.max_thrust_n:
            thrust = af.max_thrust_n

        v_i_fwd = self._induced_fwd(thrust, drag, v_h)
        # 轴向（爬升/下降）修正：v_i^2 + v_z*v_i - v_h^2 = 0
        v_i = -v_z / 2.0 + math.sqrt((v_z / 2.0) ** 2 + v_i_fwd ** 2)
        p_induced = thrust * (v_z + v_i)

        if v_z < 0.0:
            # 涡环状态（-2*v_h < v_z < 0）内动量理论失效；设下限，
            # 否则下降会算出"发电"。
            v_hov = math.sqrt(thrust / (2.0 * RHO * af.disk_area_total))
            p_induced = max(p_induced, self.descent_floor * thrust * v_hov)

        p_parasite = drag * v_h
        # 品质因数只作用于旋翼诱导功率；废阻是机体阻力，不适用
        return (p_induced / (self.figure_of_merit * self.propulsion_efficiency)
                + p_parasite)

    def hover_power_w(self) -> float:
        return self.power_w(0.0, 0.0)

    def cruise_power_w(self, v_h: float) -> float:
        return self.power_w(v_h, 0.0)

    # ---------- 计费 ----------

    def percent_per_second(self, v_h: float, v_z: float) -> float:
        """每秒消耗的百分比电量。"""
        joules_per_s = self.power_w(v_h, v_z)
        return joules_per_s / 3600.0 / self.battery_capacity_wh * 100.0

    def energy_for_step(self, v_h: float, v_z: float, dt: float) -> float:
        return self.percent_per_second(v_h, v_z) * dt

    def endurance_s(self, v_h: float = 0.0, v_z: float = 0.0) -> float:
        """该状态下的理论续航时间。"""
        rate = self.percent_per_second(v_h, v_z)
        if rate <= 0.0:
            return float("inf")
        return 100.0 / rate


class TxEnergyModel:
    """发送耗电。与飞行模型无关，两者相加构成总能耗。

    物理量级很小：2 Mbps、100 mW 电台下，整场任务的总发射时间约 4 s，
    耗电远低于飞行。保留可配置是为了让"通信能耗可忽略"这个结论
    可以被复算，而不是靠断言。
    """

    def __init__(self, tx_power_w: float, channel_rate_bps: float,
                 battery_capacity_wh: float) -> None:
        self.tx_power_w = tx_power_w
        self.channel_rate_bps = channel_rate_bps
        self.battery_capacity_wh = battery_capacity_wh

    def energy_per_bit(self) -> float:
        if self.channel_rate_bps <= 0.0:
            raise ValueError("channel_rate_bps 必须为正")
        joules_per_bit = self.tx_power_w / self.channel_rate_bps
        return joules_per_bit / 3600.0 / self.battery_capacity_wh * 100.0


def build_energy_model(config) -> EnergyModel:
    """按配置构造飞行能量模型。

    工厂放在这里而不是引擎里：引擎只声明"我要一个能量模型"，
    与 `build_link_model` 同一套路。
    """
    kind = (getattr(config, "energy_model", "linear") or "linear").lower()
    if kind == "linear":
        return LinearEnergyModel(config.energy_hover_per_s)
    if kind == "physical":
        name = (getattr(config, "airframe", "px4_sihsim_quadx")
                or "px4_sihsim_quadx").lower()
        if name not in AIRFRAMES:
            raise ValueError(
                f"未知机型 {config.airframe!r}，可选 {' / '.join(AIRFRAMES)}")
        airframe = AIRFRAMES[name]
        # config 未指定容量时用该机型的典型值：一个全局默认值会给
        # 30 g 的 Crazyflie 配上 100 Wh 电池，让它的能耗数字失去意义。
        capacity = getattr(config, "battery_capacity_wh", None)
        if capacity is None:
            capacity = airframe.typical_battery_wh
        return PhysicalEnergyModel(
            airframe, battery_capacity_wh=capacity,
            payload_kg=getattr(config, "payload_kg", 0.0),
            figure_of_merit=airframe.figure_of_merit)
    raise ValueError(f"未知能量模型 {config.energy_model!r}，"
                     f"可选 linear / physical")
