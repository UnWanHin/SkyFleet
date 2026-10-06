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

#: 线性模型的发送能耗系数（每 bit）。这是 P0 起沿用的**无实测依据**假设值，
#: 保留为默认以维持逐位兼容；物理模型不再使用它。
LEGACY_TX_PER_BIT = 5.0e-7
#: 电台发射功率 W。[假设] 100 mW 射频输出、功放效率约 30%。
DEFAULT_TX_POWER_W = 0.333

#: 联合入流求解的收敛设置。Glauert 式是隐式的，用二分求解；
#: 80 次二分把区间压到 2^-80，远优于双精度的分辨能力。
GLAUERT_BISECT_ITERS = 80
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
        """该速度是否超出了可用推力。

        所需推力是**矢量大小** `sqrt((mg)^2 + D^2)`，不是标量和 `mg + D`。
        标量和恒大于等于矢量大小，因此用它判断会高估饱和：PX4 从约
        30 m/s 就报饱和，而真实的推力/倾角上限在约 40 m/s。

        惯用的倾角判据与本判据**完全等价**：
        `atan2(D, mg) > acos(mg/T_max)  <=>  hypot(mg, D) > T_max`。

        `v_z` 对准稳态所需推力没有影响（爬升靠功率而非额外推力），
        故不参与判断，仅为与 `power_w` 保持同一接口。
        """
        thrust_needed = math.hypot(self.hover_thrust_n(payload_kg),
                                   self._drag_n(v_h, cd_a_coeff))
        return thrust_needed > self.max_thrust_n


#: RotorPy 3.0 机型。mass / k_eta / omega_max 均为 RotorPy 声明值；
#: rotor_radius 只有 Hummingbird 由 RotorPy 声明，其余为本文档补值。
AIRFRAMES: dict[str, Airframe] = {
    # 微型机：旋翼半径、电池与品质因数都是 [假设]。动量理论在这一尺度上
    # **已知偏乐观**——实测 Crazyflie 悬停约 7 分钟、约 6~8 W，本模型
    # 修正半径后给出约 4.8 W、约 11 分钟，仍偏乐观但已回到同一量级。
    # 这类机型可用于对比**趋势**，功率绝对值不可引用。
    #
    # 半径取值依据：RotorPy 对 Crazyflie 只声明 `d = 0.043`（臂长），
    # **没有** rotor_radius。半径 0.047 m 等于 94 mm 桨径，装在 43 mm
    # 臂上桨叶会穿过机架——几何上不可能。真实桨径约 46 mm，故取 0.023 m。
    # （早前版本把桨**直径**当成了半径，旋翼盘面积因此大了 4 倍、
    #   诱导功率低了约 2 倍。）
    "crazyflie": Airframe("Crazyflie", 0.030, 2.3e-08, 2500.0, 0.023,
                          typical_battery_wh=0.9, figure_of_merit=0.35),
    "crazyflie_brushless": Airframe(
        "Crazyflie Brushless", 0.044, 4.052e-08, 3100.0, 0.023,
        typical_battery_wh=1.9, figure_of_merit=0.40),
    # 交付级：本模型适用的尺度
    "hummingbird": Airframe("Hummingbird", 0.500, 5.57e-06, 1500.0, 0.100,
                            typical_battery_wh=24.0, figure_of_merit=0.65),
    "px4_sihsim_quadx": Airframe(
        "PX4 SIH QuadX", 1.000, 5e-06, 1000.0, 0.125,
        typical_battery_wh=50.0, figure_of_merit=0.65),
}


class EnergyModel(Protocol):
    """按飞行状态计费，并定义自己的发送能耗口径。

    发送能耗之所以放进协议：引擎原先固定用 `energy_tx_per_bit`（线性模型的
    任意单位），而物理飞行模型用瓦特，两者单位并不一致——只是量级小到
    看不出来。让每个模型自己回答"1 bit 值多少电量"，单位才不会分叉。
    """

    def energy_for_step(self, v_h: float, v_z: float, dt: float) -> float:
        """返回 dt 秒内消耗的电量（引擎内部单位）。"""
        ...

    def tx_energy_per_bit(self) -> float:
        """发送 1 bit 消耗的电量（同一内部单位）。"""
        ...


class LinearEnergyModel:
    """既有模型：与飞行状态无关的恒定扣费。

    **逐位保留**，作为默认值——换成物理模型会改变所有已发表数字。
    """

    def __init__(self, hover_per_s: float,
                 tx_per_bit: float = LEGACY_TX_PER_BIT) -> None:
        self.hover_per_s = hover_per_s
        self.tx_per_bit = tx_per_bit

    def energy_for_step(self, v_h: float, v_z: float, dt: float) -> float:
        return self.hover_per_s * dt

    def tx_energy_per_bit(self) -> float:
        """沿用既有无依据假设值——默认必须等于 config 的历史值，
        否则 golden digest 会变。"""
        return self.tx_per_bit


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

    def __init__(self, airframe: Airframe,
                 battery_capacity_wh: float | None = None,
                 payload_kg: float = 0.0,
                 figure_of_merit: float | None = None,
                 propulsion_efficiency: float = DEFAULT_PROPULSION_EFFICIENCY,
                 cd_a_coeff: float = DEFAULT_CD_A_COEFF,
                 tx_power_w: float = DEFAULT_TX_POWER_W,
                 channel_rate_bps: float = 2_000_000.0) -> None:
        # 默认取机型自带的典型电池容量。全局默认 100 Wh 会让微尺度机型
        # 得到荒唐的续航（Crazyflie 装 100 Wh 能飞 38 小时）。
        if battery_capacity_wh is None:
            battery_capacity_wh = airframe.typical_battery_wh
        if battery_capacity_wh <= 0.0:
            raise ValueError("battery_capacity_wh 必须为正")
        # 默认取机型自带值：全局默认 0.65 会让微尺度机型失去区分度
        # （Crazyflie 应为 0.35）。
        if figure_of_merit is None:
            figure_of_merit = airframe.figure_of_merit
        if not (0.0 < figure_of_merit <= 1.0):
            raise ValueError(f"figure_of_merit 必须在 (0,1]，得到 {figure_of_merit}")
        if not (0.0 < propulsion_efficiency <= 1.0):
            raise ValueError(
                f"propulsion_efficiency 必须在 (0,1]，得到 {propulsion_efficiency}")
        if tx_power_w < 0.0:
            raise ValueError(
                f"tx_power_w 不能为负（会让耗电量算成负值、变相充电）：{tx_power_w}")
        if channel_rate_bps <= 0.0:
            raise ValueError(f"channel_rate_bps 必须为正：{channel_rate_bps}")
        self.airframe = airframe
        self.battery_capacity_wh = battery_capacity_wh
        self.payload_kg = payload_kg
        self.figure_of_merit = figure_of_merit
        self.propulsion_efficiency = propulsion_efficiency
        self.cd_a_coeff = cd_a_coeff
        self.tx = TxEnergyModel(tx_power_w, channel_rate_bps,
                                battery_capacity_wh)

    # ---------- 功率 ----------

    def induced_velocity(self, v_h: float, v_z: float) -> float:
        """滑流诱导速度：联合（前飞 + 轴向）动量理论的**精确解**。

        隐式 Glauert 关系：

            v_i * sqrt(v_h^2 + (v_z + v_i)^2) = v_hov^2,
            v_hov^2 = T / (2 rho A)

        该式没有闭式解，用二分法求解。两个极限会自动还原成常见闭式：

          * v_h = 0 → v_i = -v_z/2 + sqrt((v_z/2)^2 + v_hov^2)   （轴向）
          * v_z = 0 → v_i = sqrt((sqrt(v_h^4+4 v_hov^4) - v_h^2)/2)（前飞）

        **不要**再用"把前飞闭式解代进轴向闭式解"的拼接法：两者来自不同的
        方程，拼接后只在上述两个极限处精确，爬升+前飞时低估 8~18%。
        """
        thrust, _ = self._thrust_and_drag(v_h)
        return self._glauert_vi(v_h, v_z, thrust)

    def _thrust_and_drag(self, v_h: float) -> tuple[float, float]:
        """准稳态所需推力与其矢量分量。推力按最大推力钳位。"""
        af = self.airframe
        drag = 0.5 * RHO * self.cd_a_coeff * af.rotor_radius_m ** 2 * v_h ** 2
        # 所需推力是矢量大小；饱和时钳到上限（注意这是**静默**钳位，
        # 调用方若需要知道状态不可达，应另查 `is_saturated`）。
        thrust = math.hypot(af.hover_thrust_n(self.payload_kg), drag)
        return min(thrust, af.max_thrust_n), drag

    def _glauert_vi(self, v_h: float, v_z: float, thrust: float) -> float:
        v_hov2 = thrust / (2.0 * RHO * self.airframe.disk_area_total)
        if v_hov2 <= 0.0:
            return 0.0
        scale = math.sqrt(v_hov2) + abs(v_z) + 1.0
        # 左端 f(0) = -v_hov^2 < 0 恒成立；右端取足够大保证 f > 0。
        lo, hi = 0.0, scale
        for _ in range(200):
            if self._glauert_residual(v_h, v_z, hi, v_hov2) > 0.0:
                break
            hi *= 2.0
        else:                                    # pragma: no cover - 兜底
            return math.sqrt(v_hov2)
        for _ in range(GLAUERT_BISECT_ITERS):
            mid = 0.5 * (lo + hi)
            if self._glauert_residual(v_h, v_z, mid, v_hov2) > 0.0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    @staticmethod
    def _glauert_residual(v_h: float, v_z: float, v_i: float,
                          v_hov2: float) -> float:
        return v_i * math.hypot(v_h, v_z + v_i) - v_hov2

    def power_w(self, v_h: float, v_z: float) -> float:
        """给定水平速度与垂直速度，返回电功率（瓦）。

        **饱和状态是静默钳位的**：所需推力超过最大推力时，这里按最大推力
        计算并返回一个有限的功率值，调用方无法从返回值看出该状态其实
        不可达。需要判断可达性请另行调用 `is_saturated`。
        """
        thrust, drag = self._thrust_and_drag(v_h)
        v_i = self._glauert_vi(v_h, v_z, thrust)
        p_induced = thrust * (v_z + v_i)

        p_parasite = drag * v_h
        # 效率链分两段，不能混为一谈：
        #   品质因数 FM  描述**旋翼**（理想诱导功率 / 实际轴功率），
        #                故只作用于诱导功率，不作用于机体废阻；
        #   推进效率 eta 描述**电机+电调**（轴功率 / 电功率），
        #                诱导功率与废阻都要经过它——两者都是电机必须
        #                输出的机械功率。
        # 早前版本把废阻整个排在效率链之外，12 m/s 时低估 12.1%、
        # 20 m/s 时低估 26.2%。
        p_electric = (p_induced / self.figure_of_merit + p_parasite) / \
            self.propulsion_efficiency

        # 快速下降时 v_z 主导，诱导功率可以为负（风车刹车状态从气流中
        # 取回能量）。固定桨距电调**不能回馈电能**，故电气侧下限为 0。
        # 注意：涡环状态（大致 -2*v_hov < v_z < -0.5*v_hov）内动量理论
        # 失效且真实功率**高于**悬停，本模型未建模该区间——下降段的
        # 功率因此可能偏低，这是已知限制。
        return max(p_electric, 0.0)

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

    def tx_energy_per_bit(self) -> float:
        """发送 1 bit 的耗电。与飞行模型**同一单位**（百分比电量）。"""
        return self.tx.energy_per_bit()

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
        return LinearEnergyModel(
            config.energy_hover_per_s,
            tx_per_bit=getattr(config, "energy_tx_per_bit", LEGACY_TX_PER_BIT))
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
            figure_of_merit=airframe.figure_of_merit,
            tx_power_w=getattr(config, "tx_power_w", DEFAULT_TX_POWER_W),
            channel_rate_bps=getattr(config, "channel_rate_bps", 2_000_000.0))
    raise ValueError(f"未知能量模型 {config.energy_model!r}，"
                     f"可选 linear / physical")
