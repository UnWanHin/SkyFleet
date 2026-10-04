"""扫参规格：定义"扫什么、扫几轮、看哪些指标"。

规格用 JSON 落盘而不是命令行参数，因为标定结果要写进报告：
"这组数字是哪个规格跑出来的"必须能被别人原样重跑。
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from skyfleet.kernel.config import SimConfig, coerce_overrides
from skyfleet.kernel.rng import derive_seed

#: 默认关注的指标（点号路径，指向 compute_metrics() 返回的嵌套字典）。
#: 覆盖报告第 6 章的九项，外加几项用于归因（转发次数、路由切换）。
DEFAULT_METRICS = (
    "tasks.completion_rate",
    "delivery_confirm.success_rate",
    "latency_ms.Q1.mean",
    "latency_ms.Q1.p95",
    "latency_ms.Q2.mean",
    "latency_ms.Q2.p95",
    "latency_ms.Q3.mean",
    "latency_ms.Q3.p95",
    "loss.control_rate",
    "loss.data_rate",
    "drop_reasons.COLLISION",
    "drop_reasons.OCCLUDED",
    "heartbeat.delivery_rate",
    "retransmission.total",
    "retransmission.expired",
    "route_switches",
    "forwarded",
    "recovery.mean_s",
    "recovery.recovered",
    "overhead.total_bytes",
    "overhead.bytes_per_delivered_task",
    "energy.min_final",
)

_SPEC_KEYS = {"name", "base", "seed", "grid", "repeats", "duration",
              "metrics", "description", "jobs"}


def expand_grid(grid: dict[str, Any]) -> list[dict[str, Any]]:
    """展开参数笛卡尔积。

    **键按字典序排序**，不按插入顺序：否则同一份规格里调换两个参数的书写
    次序就会产生不同的行顺序，并行跑完拼回去时对不上号。
    """
    if not grid:
        return [{}]

    keys = sorted(grid)
    for key in keys:
        values = grid[key]
        if isinstance(values, (str, bytes)) or not hasattr(values, "__iter__"):
            raise ValueError(f"参数 {key} 的取值必须是列表，得到 {values!r}")

    value_lists = [list(grid[k]) for k in keys]
    if any(len(v) == 0 for v in value_lists):
        return []
    return [dict(zip(keys, combo))
            for combo in itertools.product(*value_lists)]


def run_seed(master_seed: int, combo_index: int, repeat: int) -> int:
    """每个 (参数组合, 重复序号) 一个独立的种子。

    派生而不共用主种子：共用会让各次实验的随机流完全相关，
    参数之间的差异被同一批随机数掩盖，敏感度分析就白做了。

    结果掩到 63 位：派生出来的是 64 位无符号数，若不收窄，
    一旦有人把种子写进 JSON 或喂给只接受有符号 64 位的下游就会溢出。
    """
    raw = derive_seed(master_seed, f"sweep:{combo_index}:{repeat}")
    return raw & ((1 << 63) - 1)


def get_path(data: Any, path: str) -> Any:
    """按点号路径取值；任一层缺失都返回 None（而不是抛异常）。

    指标缺失是**正常情形**：某个参数组合下可能一个节点都没恢复，
    `recovery.mean_s` 自然没有样本。返回 None 让统计层去丢弃它。
    """
    cur = data
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def coerce_params(params: dict[str, Any]) -> dict[str, Any]:
    """规范化一组参数覆盖值，并拒绝拼错的键名。"""
    return coerce_overrides(params)


def apply_params(config: SimConfig, params: dict[str, Any]) -> SimConfig:
    """在一份配置上应用参数覆盖，返回新对象。"""
    return replace(config, **coerce_params(params))


@dataclass(frozen=True)
class SweepSpec:
    name: str
    base: str
    seed: int = 1
    repeats: int = 1
    grid: dict[str, Any] = None            # type: ignore[assignment]
    duration: float | None = None
    metrics: tuple[str, ...] = DEFAULT_METRICS
    description: str = ""

    def __post_init__(self) -> None:
        if self.grid is None:
            object.__setattr__(self, "grid", {})

    @classmethod
    def from_json(cls, path: str | Path) -> "SweepSpec":
        import json

        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(str(path))
        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"扫参规格不是合法 JSON: {path}: {exc}") from exc
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "SweepSpec":
        unknown = sorted(set(raw) - _SPEC_KEYS)
        if unknown:
            raise ValueError(f"扫参规格含未知字段 {unknown}")
        for required in ("name", "base"):
            if required not in raw:
                raise ValueError(f"扫参规格缺少字段 {required}")

        repeats = int(raw.get("repeats", 1))
        if repeats < 1:
            raise ValueError(f"repeats 必须 >= 1，得到 {repeats}")

        metrics = tuple(raw.get("metrics") or DEFAULT_METRICS)
        if not metrics:
            raise ValueError("metrics 不能为空")

        return cls(
            name=str(raw["name"]),
            base=str(raw["base"]),
            seed=int(raw.get("seed", 1)),
            repeats=repeats,
            grid=dict(raw.get("grid") or {}),
            duration=(float(raw["duration"])
                      if raw.get("duration") is not None else None),
            metrics=metrics,
            description=str(raw.get("description", "")),
        )

    def combos(self) -> list[dict[str, Any]]:
        return expand_grid(self.grid)

    def run_count(self) -> int:
        return len(self.combos()) * self.repeats

    def to_dict(self) -> dict[str, Any]:
        """回写成 JSON，便于把"实际用了什么规格"存档。"""
        return {
            "name": self.name,
            "base": self.base,
            "seed": self.seed,
            "repeats": self.repeats,
            "grid": {k: list(v) for k, v in sorted(self.grid.items())},
            "duration": self.duration,
            "metrics": list(self.metrics),
            "description": self.description,
        }


def load_base_config(spec: SweepSpec) -> SimConfig:
    """载入扫描的基准场景。相对路径按项目根解析。"""
    from skyfleet.kernel.config import load_config

    base = Path(spec.base)
    if not base.is_absolute():
        root = Path(__file__).resolve().parents[2]
        candidate = root / spec.base
        base = candidate if candidate.exists() else base
    return load_config(base)
