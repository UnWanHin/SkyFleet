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

    #: 故障注入：[[node_id, 静默起始时刻], ...]。
    #: 报告 R0-R3 要求"失效检测 -> 判定 -> 重派 -> 恢复"这条链可复现，
    #: 因此把它做成**场景文件的一部分**，而不是只存在于测试代码里——
    #: 答辩现场能一键跑出来，才有说服力。
    kills: tuple[tuple[str, float], ...] = ()
    #: 故障恢复：[[node_id, 重新入网时刻], ...]
    revives: tuple[tuple[str, float], ...] = ()

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

    # 可见性
    telemetry_visibility: float = 300.0

    @property
    def total_nodes(self) -> int:
        return 1 + self.slaves + self.relays


_TUPLE_FIELDS = ("height_range", "flight_altitude", "delivery_altitude")

#: 这些字段是"列表的列表"，需要把内层也转成元组
_PAIR_LIST_FIELDS = ("kills", "revives")


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

    known = {f.name for f in fields(SimConfig)}
    unknown = sorted(set(raw) - known)
    if unknown:
        raise ValueError(f"配置文件含未知字段 {unknown}: {path}")

    for key in _TUPLE_FIELDS:
        if key in raw:
            raw[key] = tuple(raw[key])
    for key in _PAIR_LIST_FIELDS:
        if key in raw:
            raw[key] = tuple((str(n), float(t)) for n, t in raw[key])

    return SimConfig(**raw)


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
