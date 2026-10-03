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
    height_range: tuple[float, float] = (30.0, 120.0)
    flight_altitude: tuple[float, float] = (30.0, 120.0)
    node_speed: float = 12.0

    # 集群
    leader_id: str = "L"
    slaves: int = 8
    relays: int = 2
    tasks: int = 20
    initial_energy: float = 100.0
    min_energy_for_task: float = 20.0

    # 协议节奏
    heartbeat_period: float = 1.0
    telemetry_period: float = 0.5
    motion_period: float = 0.1
    queue_depth: int = 200

    # 信道
    channel_rate_bps: float = 2_000_000
    interference_range_m: float = 300.0
    max_link_range: float = 800.0
    loss_threshold: float = 0.5
    retry_interval: float = 1.0

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


_TUPLE_FIELDS = ("height_range", "flight_altitude")


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
