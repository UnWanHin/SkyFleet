"""版本、任期与消息顺序的接受规则。

全部是**无副作用纯函数 + 显式状态对象**，因为这是最容易出错、
也最容易测试的一块：旧任期、旧任务、旧会话、重复操作都不能触发错误动作。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

from skyfleet.protocol.message import Msg


class AcceptDecision(enum.Enum):
    """消息接受判定结果。非 ACCEPT 即为丢弃原因。"""

    ACCEPT = "ACCEPT"
    STALE_EPOCH = "STALE_EPOCH"
    STALE_SESSION = "STALE_SESSION"
    STALE_VERSION = "STALE_VERSION"
    STALE_SEQ = "STALE_SEQ"
    DEDUP = "DEDUP"


@dataclass
class LocalView:
    """接收端的本地版本视图。"""

    leader_epoch: int = 0
    mission_version: int = 0
    route_version: int = 0
    sessions: set[str] = field(default_factory=set)
    #: (src, leader_epoch, mission_version) -> 已见最大 seq
    last_seq: dict[tuple[str, int, int], int] = field(default_factory=dict)


class DedupTable:
    """operation_id 去重表，带 TTL 与容量上限。

    容量上限是必须的：长时间仿真下去重表会无界增长。
    """

    def __init__(self, ttl: float, capacity: int) -> None:
        self.ttl = ttl
        self.capacity = capacity
        self._expires: dict[str, float] = {}

    def seen(self, op_id: str, t: float) -> bool:
        """若该操作在有效期内已出现过返回 True，否则登记并返回 False。"""
        expire_at = self._expires.get(op_id)
        if expire_at is not None and t < expire_at:
            return True
        self._expires[op_id] = t + self.ttl
        if len(self._expires) > self.capacity:
            oldest = min(self._expires, key=lambda k: self._expires[k])
            del self._expires[oldest]
        return False

    def prune(self, t: float) -> None:
        """清理已过期条目。"""
        for key in [k for k, exp in self._expires.items() if t >= exp]:
            del self._expires[key]

    def __len__(self) -> int:
        return len(self._expires)


def accept(msg: Msg, view: LocalView, dedup: DedupTable,
           t: float) -> AcceptDecision:
    """判定是否接受一条消息。

    规则顺序不可调换：任期 -> 会话 -> 任务版本 -> 去重 -> 序列。
    旧任期的消息即使会话未知，也必须报 STALE_EPOCH。
    """
    if msg.leader_epoch < view.leader_epoch:
        return AcceptDecision.STALE_EPOCH
    if view.sessions and msg.session_id not in view.sessions:
        return AcceptDecision.STALE_SESSION
    if msg.mission_version < view.mission_version:
        return AcceptDecision.STALE_VERSION
    if msg.operation_id is not None and dedup.seen(msg.operation_id, t):
        return AcceptDecision.DEDUP

    key = (msg.src, msg.leader_epoch, msg.mission_version)
    previous = view.last_seq.get(key)
    if previous is not None and msg.seq <= previous:
        return AcceptDecision.STALE_SEQ

    view.leader_epoch = max(view.leader_epoch, msg.leader_epoch)
    view.mission_version = max(view.mission_version, msg.mission_version)
    view.last_seq[key] = msg.seq
    return AcceptDecision.ACCEPT
