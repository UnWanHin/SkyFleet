"""扫参执行：跑网格、抽取指标、汇总成可写进报告的表。

三件事必须成立，否则标定结果不能用：
1. **可复现**：同一份规格跑两次，逐个数字相同。
2. **与并发无关**：`jobs=1` 和 `jobs=8` 结果一致（结果按固定键序拼装）。
3. **规格可存档**：把"实际用的规格"原样写出来，别人能重跑。

第三条容易被忽略但最要命：报告的表格若无法对应到一份确切的规格，
那些数字就只是截图。
"""
from __future__ import annotations

import csv
import dataclasses
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.metrics import compute_metrics
from skyfleet.study.spec import (
    SweepSpec, apply_params, get_path, load_base_config, run_seed,
)
from skyfleet.study.stats import aggregate


@dataclass(frozen=True)
class RunRecord:
    combo_index: int
    repeat: int
    seed: int
    params: dict[str, Any]
    metrics: dict[str, float | None]
    log_path: str


@dataclass
class SweepResult:
    spec: SweepSpec
    combos: list[dict[str, Any]]
    records: list[RunRecord] = field(default_factory=list)

    # ---------- 输出 ----------

    def param_names(self) -> list[str]:
        return sorted(self.combos[0]) if self.combos else []

    def metric_names(self) -> list[str]:
        return list(self.spec.metrics)

    def write_csv(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        names = self.param_names()
        header = (["combo", "repeat", "seed"] + names
                  + [f"m:{m}" for m in self.metric_names()])
        with path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(header)
            for record in self.records:
                row: list[Any] = [record.combo_index, record.repeat, record.seed]
                row += [record.params.get(n) for n in names]
                row += [record.metrics.get(m) for m in self.metric_names()]
                writer.writerow(row)
        return path

    def write_spec(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = self.spec.to_dict()
        payload["run_count"] = len(self.records)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
        return path

    def summary(self) -> list[dict[str, Any]]:
        """按参数组合汇总：每个指标的均值与 95% 置信区间。"""
        by_combo: dict[int, list[RunRecord]] = {}
        for record in self.records:
            by_combo.setdefault(record.combo_index, []).append(record)

        rows: list[dict[str, Any]] = []
        for index in sorted(by_combo):
            group = by_combo[index]
            rows.append({
                "combo": index,
                "params": dict(group[0].params),
                "repeats": len(group),
                "seeds": [r.seed for r in group],
                "metrics": {
                    name: aggregate([r.metrics.get(name) for r in group])
                    for name in self.metric_names()
                },
            })
        return rows

    def write_markdown(self, path: str | Path) -> Path:
        """生成可直接贴进报告的汇总表。

        缺失指标写 "—"，绝不打印 nan：读者会把 nan 当成一个数值，
        进而以为"这项测出来是 nan"，而真相是"这次实验没有这项样本"。
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        names = self.param_names()
        lines: list[str] = []
        lines.append(f"# 扫参结果：{self.spec.name}")
        lines.append("")
        if self.spec.description:
            lines.append(self.spec.description)
            lines.append("")
        lines.append(f"- 基准场景：`{self.spec.base}`")
        lines.append(f"- 主种子：{self.spec.seed}，每个组合重复 "
                     f"{self.spec.repeats} 次，共 {len(self.records)} 次运行")
        lines.append(f"- 参数网格：`{json.dumps(self.spec.grid, ensure_ascii=False)}`")
        lines.append("")

        header = ["组合"] + names + ["重复"]
        header += [f"{m}（均值±95%CI）" for m in self.metric_names()]
        lines.append("| " + " | ".join(header) + " |")
        lines.append("|" + "---|" * len(header))

        for row in self.summary():
            cells = [str(row["combo"])]
            cells += [_num(row["params"].get(n)) for n in names]
            cells.append(str(row["repeats"]))
            for name in self.metric_names():
                stats = row["metrics"][name]
                cells.append(_mean_ci(stats))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")
        lines.append("> 区间为双尾 95% t 置信区间。`—` 表示该组合下没有该指标的"
                     "样本（例如没有节点恢复时的平均恢复时间），不是 0。")
        lines.append("")

        path.write_text("\n".join(lines), encoding="utf-8")
        return path


def _num(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def _mean_ci(stats: dict[str, Any]) -> str:
    if stats.get("n", 0) == 0 or stats.get("mean") is None:
        return "—"
    mean = stats["mean"]
    if stats.get("ci_low") is None:
        return f"{mean:.4g}"
    half = (stats["ci_high"] - stats["ci_low"]) / 2.0
    return f"{mean:.4g} ± {half:.2g}"


# ---------- 执行 ----------


def _prepare_config(base: SimConfig, spec: SweepSpec,
                    params: dict[str, Any], seed: int,
                    log_path: Path) -> SimConfig:
    config = apply_params(base, params)
    changes: dict[str, Any] = {"seed": seed, "log_path": str(log_path)}
    if spec.duration is not None:
        changes["duration"] = spec.duration
    return dataclasses.replace(config, **changes)


def run_one(base: SimConfig, spec: SweepSpec, combo_index: int,
            repeat: int, params: dict[str, Any], log_dir: Path,
            keep_logs: bool = True) -> RunRecord:
    """跑一次实验并抽取指标。这是并行单元，必须是纯函数式输入输出。"""
    from skyfleet.sim.engine import Engine

    seed = run_seed(spec.seed, combo_index, repeat)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"c{combo_index:03d}_r{repeat:02d}.jsonl"

    config = _prepare_config(base, spec, params, seed, log_path)
    Engine(config).run()

    raw = compute_metrics(log_path)
    metrics = {name: _as_number(get_path(raw, name))
               for name in spec.metrics}

    if not keep_logs:
        log_path.unlink(missing_ok=True)

    return RunRecord(combo_index=combo_index, repeat=repeat, seed=seed,
                     params=dict(params), metrics=metrics,
                     log_path=str(log_path))


def _as_number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return float(value) if isinstance(value, bool) else None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _worker(payload: dict[str, Any]) -> RunRecord:
    """ProcessPoolExecutor 的入口，必须是模块级可 pickle 的函数。"""
    return run_one(base=payload["base"], spec=payload["spec"],
                   combo_index=payload["combo_index"],
                   repeat=payload["repeat"], params=payload["params"],
                   log_dir=Path(payload["log_dir"]),
                   keep_logs=payload["keep_logs"])


def run_sweep(spec: SweepSpec, out_dir: str | Path, jobs: int = 1,
              keep_logs: bool = True, verbose: bool = False) -> SweepResult:
    """按规格跑完整轮扫描。

    `jobs > 1` 时用进程池。结果**按 (组合序号, 重复序号) 排序后**再返回，
    因此并发度不影响输出内容，只影响耗时。
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir = out_dir / "logs"

    base = load_base_config(spec)
    combos = spec.combos()
    result = SweepResult(spec=spec, combos=combos)

    tasks = [(index, repeat, params)
             for index, params in enumerate(combos)
             for repeat in range(spec.repeats)]

    if jobs <= 1:
        records = []
        for done, (index, repeat, params) in enumerate(tasks, start=1):
            records.append(run_one(base, spec, index, repeat, params,
                                   log_dir, keep_logs))
            if verbose:
                print(f"  [{done}/{len(tasks)}] combo {index} repeat {repeat}",
                      file=sys.stderr)
    else:
        payloads = [{"base": base, "spec": spec, "combo_index": index,
                     "repeat": repeat, "params": params,
                     "log_dir": str(log_dir), "keep_logs": keep_logs}
                    for index, repeat, params in tasks]
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            records = list(pool.map(_worker, payloads))

    # 排序是并发安全的保证：map 保序，但显式排序让"保序"不再依赖实现细节
    result.records = sorted(records, key=lambda r: (r.combo_index, r.repeat))
    return result
