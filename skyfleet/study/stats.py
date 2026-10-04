"""统计汇总：只依赖标准库。

多次重复实验后要报告的不是一个数，而是"均值 ± 置信区间"。
用 t 分布而不是正态：重复次数通常只有 3~10 次，正态近似会低估区间，
把噪声说成结论。
"""
from __future__ import annotations

import math
import statistics
from typing import Any, Iterable, Sequence

#: 双尾 95% t 分布临界值（自由度 1~30），超出范围用正态近似 1.96。
_T95 = (
    12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228,
    2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086,
    2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042,
)
_NORMAL_95 = 1.96


def t_critical(df: int) -> float:
    """双尾 95% 置信区间的 t 临界值。"""
    if df < 1:
        df = 1
    if df > len(_T95):
        return _NORMAL_95
    return _T95[df - 1]


def aggregate(values: Iterable[float | None]) -> dict[str, Any]:
    """把同一参数组合下的多次重复汇总成均值与置信区间。

    `None` 表示"该次实验没有这个指标的样本"（例如没有任何节点恢复时的
    平均恢复时间）。它必须被**丢弃**而不是当作 0——否则"没有故障"会被
    平均成"恢复时间为 0"，结论正好反过来。
    """
    xs = [float(v) for v in values if v is not None]
    n = len(xs)
    if n == 0:
        return {"n": 0, "mean": None, "stdev": None,
                "ci_low": None, "ci_high": None, "min": None, "max": None}

    mean = sum(xs) / n
    if n == 1:
        return {"n": 1, "mean": mean, "stdev": None,
                "ci_low": mean, "ci_high": mean,
                "min": xs[0], "max": xs[0]}

    stdev = statistics.stdev(xs)
    half = t_critical(n - 1) * stdev / math.sqrt(n)
    return {"n": n, "mean": mean, "stdev": stdev,
            "ci_low": mean - half, "ci_high": mean + half,
            "min": min(xs), "max": max(xs)}


def relative_spread(stats: dict[str, Any]) -> float | None:
    """半宽相对均值的比例，用于判断"这个差异是否值得当结论"。"""
    mean, low, high = stats.get("mean"), stats.get("ci_low"), stats.get("ci_high")
    if mean is None or low is None or high is None or mean == 0:
        return None
    return (high - low) / 2.0 / abs(mean)


def welch_significant(a: Sequence[float], b: Sequence[float]) -> bool:
    """Welch t 检验的粗略判据：两个样本的 95% 区间是否分离。

    这里不追求严格的 p 值——重复次数少，p 值本身也不可靠。报告里要回答的
    只是"差异有没有大到值得解释"，区间分离是个够用且不易被误读的判据。
    """
    sa, sb = aggregate(a), aggregate(b)
    if sa["n"] < 2 or sb["n"] < 2:
        return sa["mean"] != sb["mean"]
    return sa["ci_high"] < sb["ci_low"] or sb["ci_high"] < sa["ci_low"]
