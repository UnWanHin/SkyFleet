"""配置：全部可调参数集中于此。

硬性约束：状态机门限、路由迟滞门限、信道参数一律来自配置文件，
不得硬编码——否则报告要求的"阈值标定"无法进行。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, fields, replace
from pathlib import Path

SCENARIOS_DIR = Path(__file__).resolve().parent.parent / "scenarios"


def baseline_path() -> str:
    """基线场景配置路径。"""
    return str(SCENARIOS_DIR / "baseline_small.json")


@dataclass
class SimConfig:
    # 运行
    seed: int = 1
    duration: float = 600.0
    log_path: str = "out/logs/baseline.jsonl"

    # 世界
    area: float = 500.0
    blocks: int = 3
    gap: float = 20.0
    #: 楼高带。高于街道、低于巡航高度，是"城市峡谷"的来源。
    height_range: tuple[float, float] = (50.0, 110.0)
    #: 节点初始所在高度带（高于多数楼顶）
    flight_altitude: tuple[float, float] = (110.0, 160.0)
    #: 巡航高度：在两个任务点之间转场时使用，高于全部楼顶
    cruise_altitude: float = 140.0
    #: 取餐点/投递点的高度带：位于街道峡谷内，低于楼顶。
    #: "高空巡航 + 低空投递"让遮挡成为**每次投递都会短暂发生**的现象，
    #: 而不是"整段航程全遮挡"或"完全无遮挡"的二值结果——后者既不符合
    #: 城市峡谷的物理，也让链路劣化与恢复的分析无从做起。
    delivery_altitude: tuple[float, float] = (80.0, 110.0)
    node_speed: float = 12.0

    # 集群
    leader_id: str = "L"
    slaves: int = 8
    relays: int = 2
    tasks: int = 20
    initial_energy: float = 100.0
    min_energy_for_task: float = 20.0

    #: 链路模型："geometric"（P0 几何遮挡，二值）或 "nlos"
    #: （P1 绕射 + 快衰落）。协议层不感知差异——这正是 §9 的保险栓。
    link_model: str = "geometric"

    #: 环境随机种子。为 None 时退回 seed（保持既有场景行为不变）。
    #: 显式指定后，**楼群布局与初始站位只由它决定**，与 seed 无关；
    #: 于是扫参的重复实验可以在同一座城市里比较协议差异（共同随机数）。
    #: 不这样做的话，"参数效应"里会混进"城市布局效应"，而后者大得多。
    env_seed: int | None = None

    #: 故障注入：[[node_id, 静默起始时刻], ...]。
    #: 报告 R0-R3 要求"失效检测 -> 判定 -> 重派 -> 恢复"这条链可复现，
    #: 因此把它做成**场景文件的一部分**，而不是只存在于测试代码里——
    #: 答辩现场能一键跑出来，才有说服力。
    kills: tuple[tuple[str, float], ...] = ()
    #: 故障恢复：[[node_id, 重新入网时刻], ...]
    revives: tuple[tuple[str, float], ...] = ()

    # ===== 场景自述（不参与仿真，只为报告与命令行阅读）=====
    #: 场景名。留空时由文件名推断。
    name: str = ""
    #: 场景说明：这是什么场景、想验证什么。
    description: str = ""

    # ===== P1 注入接口（§9）=====
    #: 背景干扰源：[{"id","x","y","radius","t_start","t_end","extra_loss_db"}]
    interferences: tuple[dict, ...] = ()
    #: 临时障碍（高架/塔吊）：[{"id","x","y","w","h","height","t_start","t_end"}]
    obstacles: tuple[dict, ...] = ()
    #: 禁飞区：[{"id","x","y","w","h","z_min","z_max"}]
    geofences: tuple[dict, ...] = ()
    #: GNSS 定位漂移幅度（米）。0 表示理想定位。
    gnss_drift_m: float = 0.0
    #: GNSS 失锁时段：[[node_id, 起始, 结束], ...]
    gnss_outages: tuple[tuple[str, float, float], ...] = ()
    #: 对抗注入：[{"kind": ..., "t": ...}, ...]，见 sim/adversary.py
    adversary: tuple[dict, ...] = ()

    # 协议节奏
    heartbeat_period: float = 1.0
    telemetry_period: float = 0.5
    motion_period: float = 0.1
    queue_depth: int = 200
    #: 每次周期发送的时钟漂移抖动（秒，±）。
    #: 没有它，各节点的发送时刻是严格周期的固定相位，彼此几乎永不重叠——
    #: 于是"信道竞争"这个报告要求观察的现象会退化成恒等于 0。
    #: 加入漂移后到达过程接近随机的 ALOHA 型竞争。
    access_jitter: float = 0.05

    # 信道
    channel_rate_bps: float = 2_000_000
    interference_range_m: float = 300.0
    max_link_range: float = 800.0
    loss_threshold: float = 0.5
    #: 直连质量低于该值时才考虑改走中继。
    #: 中继是"直连不够用时的补线"，不是"略微更好就换"的优选：若只要
    #: 中继比直连好一点就切过去，每一次心跳都会多占用一跳的空口时间，
    #: 信道负载凭空翻倍，冲突率随之被自己推高——那是协议自己制造的拥塞。
    relay_direct_threshold: float = 0.6
    retry_interval: float = 1.0
    #: 信道时隙长度。竞争以时隙为单位判定：同一时隙内互相在干扰范围内的
    #: 并发发送全部冲突。没有时隙，浮点时刻下冲突概率为零。
    slot_duration: float = 0.002
    #: 投递确认的有效期。比 R2 的默认 1 s 长：投递确认是业务闭环，
    #: 值得用更长的时间窗换取"最终一定送达或被判定失败"。
    confirm_deadline: float = 5.0

    # 限流
    q0_rate: float = 5.0
    q0_burst: float = 10.0
    q1_rate: float = 10.0
    q1_burst: float = 20.0

    # 成员状态机门限
    n_degraded: int = 2
    n_suspect: int = 4
    t_fail: float = 3.0
    q_degraded: float = 0.4

    # 路由迟滞门限
    h_up: float = 0.75
    h_down: float = 0.35
    t_hold: float = 1.0
    flap_window: float = 5.0

    # 去重
    dedup_ttl: float = 30.0
    dedup_capacity: int = 5000

    # 能耗（假设值，见文档 §14.2 开放问题 3）
    energy_tx_per_bit: float = 5.0e-7
    energy_hover_per_s: float = 0.05

    #: 飞行能量模型："linear"（与飞行状态无关的恒定扣费，P0/P1 行为）
    #: 或 "physical"（四旋翼动量理论，按悬停/巡航/爬升/下降分别计费）。
    #: 默认 linear 是**刻意的**：换成 physical 会改变所有已发表的扫描
    #: 结果与 golden digest，那应当是有意识的重新基线，不是顺手改掉。
    #: 分析见 docs/analysis/2026-10-05-thrust-margin-analysis.md
    energy_model: str = "linear"
    #: physical 模型使用的机型，名字取自 skyfleet/world/power.AIRFRAMES
    airframe: str = "px4_sihsim_quadx"
    #: 载荷（含投递物），kg
    payload_kg: float = 0.0
    #: 电池容量，Wh。physical 模型把瓦特换算成百分比电量的基准。
    #: None 表示采用该机型的典型容量（见 power.AIRFRAMES）——不要用一个
    #: 全局默认值，否则会给 30 g 机型配上 1 kg 机型的电池。
    battery_capacity_wh: float | None = None

    # 可见性
    telemetry_visibility: float = 300.0

    @property
    def total_nodes(self) -> int:
        return 1 + self.slaves + self.relays


_TUPLE_FIELDS = ("height_range", "flight_altitude", "delivery_altitude")

#: 这些字段是"列表的列表"，需要把内层也转成元组
_PAIR_LIST_FIELDS = ("kills", "revives")

#: 三元组列表（节点, 起始, 结束）
_TRIPLE_LIST_FIELDS = ("gnss_outages",)

#: 字典列表，原样保留但转成元组
_DICT_LIST_FIELDS = ("interferences", "obstacles", "geofences", "adversary")


def load_config(path: str | Path) -> SimConfig:
    """加载场景配置。未知字段报错，缺失字段用默认值。"""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(str(path))

    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"配置文件不是合法 JSON: {path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ValueError(f"配置文件顶层必须是对象: {path}")

    try:
        raw = coerce_overrides(raw)
    except ValueError as exc:
        raise ValueError(f"配置文件含无效字段: {path}: {exc}") from exc

    return SimConfig(**raw)


def coerce_overrides(raw: dict) -> dict:
    """把外部传入的参数值规范化成 SimConfig 能接受的类型。

    配置文件和扫参规格走的是**同一条**规范化路径：如果扫参自己再实现一遍，
    两边对 tuple 的处理迟早会漂开，届时扫出来的结果和手跑的结果不一致，
    而这类不一致极难察觉。未知键一律报错——扫参时把 `slot_duration`
    拼成 `slot_duraton`，静默忽略会让整轮扫描的结论都是假的。
    """
    known = {f.name for f in fields(SimConfig)}
    unknown = sorted(set(raw) - known)
    if unknown:
        raise ValueError(f"未知字段 {unknown}")

    out = dict(raw)
    for key in _TUPLE_FIELDS:
        if out.get(key) is not None:
            out[key] = tuple(float(v) for v in out[key])
    for key in _PAIR_LIST_FIELDS:
        if out.get(key) is not None:
            out[key] = tuple((str(n), float(t)) for n, t in out[key])
    for key in _TRIPLE_LIST_FIELDS:
        if out.get(key) is not None:
            out[key] = tuple((str(n), float(a), float(b))
                             for n, a, b in out[key])
    for key in _DICT_LIST_FIELDS:
        if out.get(key) is not None:
            out[key] = tuple(dict(item) for item in out[key])
    return out


def apply_overrides(config: SimConfig, seed: int | None = None,
                    duration: float | None = None) -> SimConfig:
    """命令行覆盖。None 表示保持原值。"""
    changes: dict[str, object] = {}
    if seed is not None:
        changes["seed"] = seed
    if duration is not None:
        changes["duration"] = duration
    if not changes:
        return replace(config)
    return replace(config, **changes)
