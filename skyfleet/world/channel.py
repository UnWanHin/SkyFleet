"""共享无线信道。

报告 §3.1 明确否定"固定丢包率"：若每条链路只掷一个固定概率的骰子，
"信道竞争、队列长度、拥塞"这些报告要求观察的现象根本不会产生，
因为没有任何东西在竞争。

本模块把信道建模为**共享资源**，并采用"同一时刻统一结算"的方式处理竞争：
同一虚拟时刻注册的多个发送意图会被一起判定，互相干扰者全部冲突。
这需要 Scheduler.on_timestamp_end 在每个时刻结束时回调 flush()。

确定性来自两点：
1. 冲突判定基于时间重叠与几何距离，不掷骰子；
2. 随机数只用于链路质量抽签，且来自独立随机流。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from skyfleet.observe.events import DropReason
from skyfleet.world.geometry import distance_3d

Pos = tuple[float, float, float]

#: 光速，用于计算传播时延
PROP_SPEED = 299_792_458.0


@dataclass
class TxAttempt:
    """一次发送尝试。ok/reason 可能被 flush() 改写（冲突是事后结算的）。"""

    ok: bool
    reason: DropReason | None
    t_access: float     # 等待信道空闲的时间
    t_tx: float         # 实际传输时间 = size_bits / rate
    t_prop: float       # 传播时延
    t_end: float
    t_start: float = 0.0
    src_id: str = ""
    src_pos: Pos = (0.0, 0.0, 0.0)
    msg_id: str = ""


class Channel:
    """单次占用的共享信道。"""

    def __init__(self, rate_bps: float, interference_range_m: float,
                 rng: Any, prop_speed: float = PROP_SPEED,
                 loss_threshold: float = 0.5) -> None:
        self.rate_bps = rate_bps
        self.interference_range_m = interference_range_m
        self.rng = rng
        self.prop_speed = prop_speed
        self.loss_threshold = loss_threshold
        self._busy_until = 0.0
        self._batch: list[TxAttempt] = []

    def busy_until(self) -> float:
        return self._busy_until

    def attempt(self, msg_id: str, src_id: str, src_pos: Pos, dst_id: str,
                dst_pos: Pos, t: float, link_quality: float,
                size_bits: int) -> TxAttempt:
        """注册一次发送尝试。

        结果中的 ok/reason 在 flush() 之后才最终确定——因为同一时刻的
        并发发送必须一起判定冲突。调用方应等到时刻结束再使用结果。
        """
        duration = size_bits / self.rate_bps
        prop = distance_3d(src_pos, dst_pos) / self.prop_speed

        if link_quality <= 0.0:
            return TxAttempt(ok=False, reason=DropReason.OCCLUDED,
                             t_access=0.0, t_tx=duration, t_prop=prop,
                             t_end=t + duration, t_start=t, src_id=src_id,
                             src_pos=src_pos, msg_id=msg_id)

        if self._busy_until > t:
            t_start = self._busy_until
            t_access = t_start - t
        else:
            t_start = t
            t_access = 0.0

        attempt = TxAttempt(ok=True, reason=None, t_access=t_access,
                            t_tx=duration, t_prop=prop, t_end=t_start + duration,
                            t_start=t_start, src_id=src_id, src_pos=src_pos,
                            msg_id=msg_id)

        # 链路质量抽签：仅在质量明显偏低时才掷骰子，
        # 这样高质量链路的行为是完全确定的。
        if link_quality < self.loss_threshold and self.rng.random() > link_quality:
            attempt.ok = False
            attempt.reason = DropReason.OCCLUDED

        self._batch.append(attempt)
        return attempt

    def flush(self) -> list[TxAttempt]:
        """结算本时刻的全部发送尝试。互相干扰者全部冲突。"""
        batch, self._batch = self._batch, []
        for i, attempt in enumerate(batch):
            if not attempt.ok:
                continue
            for other in batch[i + 1:]:
                if not other.ok:
                    continue
                if (self._overlaps(attempt, other)
                        and self._interferes(attempt, other)):
                    attempt.ok = False
                    attempt.reason = DropReason.COLLISION
                    other.ok = False
                    other.reason = DropReason.COLLISION

        for attempt in batch:
            if attempt.ok:
                self._busy_until = max(self._busy_until, attempt.t_end)
        return batch

    def _overlaps(self, a: TxAttempt, b: TxAttempt) -> bool:
        return a.t_start < b.t_end and b.t_start < a.t_end

    def _interferes(self, a: TxAttempt, b: TxAttempt) -> bool:
        if a.src_id == b.src_id:
            return False
        return distance_3d(a.src_pos, b.src_pos) <= self.interference_range_m
