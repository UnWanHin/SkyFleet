"""共享无线信道（时隙化 ALOHA）。

报告 §3.1 明确否定"固定丢包率"：若每条链路只掷一个固定概率的骰子，
"信道竞争、队列长度、拥塞"这些报告要求观察的现象根本不会产生，
因为没有任何东西在竞争。

本模块把信道建模为**共享资源**：发送以时隙为单位，落在同一时隙内、
且彼此处于干扰范围的两台发送方互相冲突。

为什么必须时隙化：若只在"完全相同的浮点时刻"才判定冲突，那么两个
独立随机时刻相等的概率为零——冲突数会恒等于 0，"信道竞争"这一整块
分析就变成了空话。时隙把连续时间离散成竞争单位，冲突才成为可观测、
可统计、可归因的现象。

确定性来自三点：
1. 冲突判定只依赖时隙编号与几何距离，不掷骰子；
2. 结算顺序由时隙编号与登记顺序决定，与字典遍历无关；
3. 随机数只用于链路质量抽签，且来自独立随机流。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from skyfleet.observe.events import DropReason
from skyfleet.world.geometry import distance_3d

Pos = tuple[float, float, float]

#: 光速，用于计算传播时延
PROP_SPEED = 299_792_458.0

#: 默认时隙长度（秒）。2 Mbps 下最长的控制消息约 0.3 ms，
#: 2 ms 时隙足以容纳一次发送，同时把同隙并发压到较低水平——
#: 时隙越长，同隙竞争者越多，冲突率越高。
DEFAULT_SLOT = 0.002

#: 时隙结算事件的微小偏移。
#: 结算必须发生在该时隙**所有**发送意图都登记完之后；把结算安排在
#: 时隙边界之后 1 ns，就能确保它排在所有恰好落在边界上的事件之后，
#: 而 1 ns 远小于任何一次发送的时长（≥128 µs），不影响物理量。
TICK_EPSILON = 1e-9


def reception_ok(quality: float, loss_threshold: float, rng: Any) -> bool:
    """给定链路质量，判定一次接收是否成功。

    同一条规则同时用于单播（在信道里判定）与广播（在交付时按接收端判定），
    保证两种传播方式的口径一致。
    """
    if quality <= 0.0:
        return False
    if quality < loss_threshold and rng.random() > quality:
        return False
    return True


@dataclass
class TxAttempt:
    """一次发送尝试。ok/reason 在所在时隙关闭时才最终确定。"""

    ok: bool
    reason: DropReason | None
    t_access: float     # 等到本时隙边界的时间
    t_tx: float         # 实际传输时间 = size_bits / rate
    t_prop: float       # 传播时延
    t_end: float
    t_start: float = 0.0
    src_id: str = ""
    src_pos: Pos = (0.0, 0.0, 0.0)
    msg_id: str = ""
    tx_id: int = 0
    slot: int = 0


class Channel:
    """时隙化共享信道。"""

    def __init__(self, rate_bps: float, interference_range_m: float,
                 rng: Any, prop_speed: float = PROP_SPEED,
                 loss_threshold: float = 0.5,
                 slot_duration: float = DEFAULT_SLOT) -> None:
        self.rate_bps = rate_bps
        self.interference_range_m = interference_range_m
        self.rng = rng
        self.prop_speed = prop_speed
        self.loss_threshold = loss_threshold
        self.slot_duration = slot_duration
        self._busy_until = 0.0
        self._slots: dict[int, list[TxAttempt]] = {}
        self._tx_seq = 0

    def slot_of(self, t: float) -> int:
        """t 所归属的发送时隙。

        发送总是等到**下一个**时隙边界，因此时隙号向上取整。
        这一点是因果性的底线：如果把发送时刻回退到当前时隙的起点，
        一条在时隙末尾产生的消息就会"在产生之前"被发出去，
        端到端时延会算出负值，虚拟时钟也会倒退。
        """
        return int(math.ceil(t / self.slot_duration - 1e-9))

    def slot_start(self, slot: int) -> float:
        return slot * self.slot_duration

    def slot_containing(self, t: float) -> int:
        """物理上包含时刻 t 的时隙编号（用于判断一次发送覆盖了哪些时隙）。"""
        return int(math.floor(t / self.slot_duration + 1e-9))

    def tick_time(self, slot: int) -> float:
        """该时隙的结算时刻。"""
        return slot * self.slot_duration + TICK_EPSILON

    def busy_until(self) -> float:
        return self._busy_until

    def attempt(self, msg_id: str, src_id: str, src_pos: Pos, dst_id: str,
                dst_pos: Pos, t: float, link_quality: float,
                size_bits: int) -> TxAttempt:
        """登记一次发送尝试。结果在所在时隙关闭后才最终确定。"""
        duration = size_bits / self.rate_bps
        prop = distance_3d(src_pos, dst_pos) / self.prop_speed
        slot = self.slot_of(t)
        t_start = self.slot_start(slot)

        self._tx_seq += 1
        attempt = TxAttempt(
            ok=True, reason=None,
            t_access=max(0.0, t_start - t), t_tx=duration, t_prop=prop,
            t_end=t_start + duration, t_start=t_start, src_id=src_id,
            src_pos=src_pos, msg_id=msg_id, tx_id=self._tx_seq, slot=slot)

        if link_quality <= 0.0:
            attempt.ok = False
            attempt.reason = DropReason.OCCLUDED
        elif (link_quality < self.loss_threshold
                and self.rng.random() > link_quality):
            # 链路质量抽签：仅在质量明显偏低时才掷骰子，
            # 这样高质量链路的行为是完全确定的。
            attempt.ok = False
            attempt.reason = DropReason.OCCLUDED

        # 一次发送可能跨多个时隙（长消息）；它应参与每个所覆盖时隙的竞争
        last_slot = self.slot_containing(attempt.t_end - 1e-12)
        for covered in range(slot, max(slot, last_slot) + 1):
            self._slots.setdefault(covered, []).append(attempt)
        return attempt

    def active_slots(self) -> list[int]:
        """尚未结算的时隙编号。"""
        return sorted(self._slots)

    def flush(self, t: float | None = None,
              force: bool = False) -> list[TxAttempt]:
        """结算已经关闭的时隙。

        时隙 k 在时刻 t 关闭的条件是 slot_of(t) > k——此刻起不可能
        再有发送落入该时隙。t=None 或 force=True 时结算全部。
        """
        if t is None or force:
            closed = sorted(self._slots)
        else:
            # 时隙 k 一旦 t 越过它的边界就已关闭：
            # 归属时隙 k 的发送必然满足 t' <= k*τ，不可能再来新的。
            closed = [slot for slot in sorted(self._slots)
                      if slot * self.slot_duration < t]

        resolved: list[TxAttempt] = []
        seen: set[int] = set()
        for slot in closed:
            for attempt in self._resolve(self._slots.pop(slot)):
                if attempt.tx_id not in seen:
                    seen.add(attempt.tx_id)
                    resolved.append(attempt)
        return resolved

    def _resolve(self, batch: list[TxAttempt]) -> list[TxAttempt]:
        """同一时隙内互相干扰的发送全部冲突。"""
        for index, attempt in enumerate(batch):
            if not attempt.ok:
                continue
            for other in batch[index + 1:]:
                if not other.ok:
                    continue
                if self._interferes(attempt, other):
                    attempt.ok = False
                    attempt.reason = DropReason.COLLISION
                    other.ok = False
                    other.reason = DropReason.COLLISION

        for attempt in batch:
            if attempt.ok:
                self._busy_until = max(self._busy_until, attempt.t_end)
        return batch

    def _interferes(self, a: TxAttempt, b: TxAttempt) -> bool:
        if a.src_id == b.src_id:
            return False
        return distance_3d(a.src_pos, b.src_pos) <= self.interference_range_m
