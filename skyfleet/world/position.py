"""P1 位置源：理想定位 vs GNSS 漂移与失锁（规格 §9 的 `PositionSource`）。

报告 §2.2.6 要求"位置过期时执行悬停"——但 P0 里没有任何东西会产生
"过期位置"，`safety_decision(position_stale=...)` 这个分支从未被触发过。
本模块补上这个来源，使该分支成为可观测、可测的行为：

- **漂移**：定位误差按 (节点, 时间片) 确定性派生，同一时间片内不变。
  若每次调用都重新抽，位置会高频抖动——那不是定位误差，是噪声源。
- **失锁**：失锁期间节点只能上报**最后一次有效定位**，且该位置随时间变旧。
  这正是"过期"的定义，也是触发悬停的依据。

位置是纯函数于 (节点, t) 的，不推进随机状态：`report` 可能被多次调用
（组包、日志、安全检查），推进式实现会让同一时刻给出不同答案。
"""
from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass
from typing import Protocol, Sequence

from skyfleet.world.node import Node, Pos


@dataclass(frozen=True)
class GnssStatus:
    """一次定位汇报的结果。"""

    pos: Pos
    stale: bool
    #: 该定位自取得以来的时长（秒）。理想定位恒为 0。
    age_s: float = 0.0


class PositionSource(Protocol):
    """位置源接口。P1 的邻居相对定位只需实现同一签名。"""

    def report(self, node: Node, t: float) -> GnssStatus: ...


class IdealPositionSource:
    """P0 行为：定位永远准确、永不过期。"""

    def report(self, node: Node, t: float) -> GnssStatus:
        return GnssStatus(pos=node.pos, stale=False, age_s=0.0)


class GnssPositionSource:
    """带漂移与失锁的 GNSS 定位源。"""

    def __init__(self, seed: int, drift_m: float = 0.0,
                 outages: Sequence[tuple[str, float, float]] = (),
                 drift_step_s: float = 1.0) -> None:
        self.seed = seed
        self.drift_m = float(drift_m)
        self.drift_step_s = max(1e-6, float(drift_step_s))
        self.outages: dict[str, list[tuple[float, float]]] = {}
        for node_id, start, end in outages:
            self.outages.setdefault(str(node_id), []).append(
                (float(start), float(end)))
        for spans in self.outages.values():
            spans.sort()

    # ---------- 失锁 ----------

    def outage_covering(self, node_id: str, t: float) -> tuple[float, float] | None:
        for start, end in self.outages.get(node_id, ()):
            if start <= t < end:
                return start, end
        return None

    def last_fix_time(self, node_id: str, t: float) -> float:
        """最后一次有效定位的时刻：即当前失锁时段的起点，否则就是 t。"""
        span = self.outage_covering(node_id, t)
        return span[0] if span is not None else t

    # ---------- 汇报 ----------

    def report(self, node: Node, t: float) -> GnssStatus:
        span = self.outage_covering(node.node_id, t)
        if span is not None:
            start, _ = span
            # 失锁期间只能拿最后一次有效定位，位置冻结、年龄增长
            return GnssStatus(pos=self._fix(node, start), stale=True,
                              age_s=t - start)
        return GnssStatus(pos=self._fix(node, t), stale=False, age_s=0.0)

    def _fix(self, node: Node, t: float) -> Pos:
        """t 时刻的定位结果（含漂移）。"""
        if self.drift_m <= 0.0:
            return node.pos
        index = int(math.floor(t / self.drift_step_s))
        rng = random.Random(self._stream(node.node_id, index))
        # 水平漂移为主，垂直方向小一个量级（GNSS 高程精度天然更差，
        # 但这里保守取小，避免由定位误差引入虚假的高度收益）
        dx = rng.uniform(-1.0, 1.0) * self.drift_m
        dy = rng.uniform(-1.0, 1.0) * self.drift_m
        dz = rng.uniform(-1.0, 1.0) * self.drift_m * 0.1
        return (node.pos[0] + dx, node.pos[1] + dy, node.pos[2] + dz)

    def _stream(self, node_id: str, index: int) -> int:
        raw = f"gnss:{self.seed}:{node_id}:{index}".encode("utf-8")
        return int.from_bytes(hashlib.blake2b(raw, digest_size=8).digest(),
                              "big")


def build_position_source(config) -> PositionSource:
    """按配置构造位置源；无漂移无失锁时退回理想定位。"""
    drift = float(getattr(config, "gnss_drift_m", 0.0) or 0.0)
    outages = tuple(getattr(config, "gnss_outages", ()) or ())
    if drift <= 0.0 and not outages:
        return IdealPositionSource()
    return GnssPositionSource(seed=config.seed, drift_m=drift,
                              outages=outages)
