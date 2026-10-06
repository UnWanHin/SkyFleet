"""版本、任期与消息顺序的接受规则。

全部是**无副作用纯函数 + 显式状态对象**，因为这是最容易出错、
也最容易测试的一块：旧任期、旧任务、旧会话、重复操作都不能触发错误动作。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

from skyfleet.protocol.message import MISSION_SCOPED, Msg


class AcceptDecision(enum.Enum):
    """消息接受判定结果。非 ACCEPT 即为丢弃原因。"""

    ACCEPT = "ACCEPT"
    STALE_EPOCH = "STALE_EPOCH"
    STALE_SESSION = "STALE_SESSION"
    STALE_VERSION = "STALE_VERSION"
    STALE_SEQ = "STALE_SEQ"
    #: 序号跃变超出滑动窗口。**必须与 STALE_SEQ 分开**：混用会让
    #: "有人冒充" 和 "我自己掉线" 在日志里长得一模一样，攻击不可观测。
    SEQ_OUT_OF_WINDOW = "SEQ_OUT_OF_WINDOW"
    DEDUP = "DEDUP"


@dataclass
class LocalView:
    """接收端的本地版本视图。"""

    leader_epoch: int = 0
    mission_version: int = 0
    route_version: int = 0
    sessions: set[str] = field(default_factory=set)
    #: (源, 消息类型, 任期, 任务版本) -> 已见最大 seq
    last_seq: dict[tuple[str, str, int, int], int] = field(default_factory=dict)


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


#: 序号滑动窗口的默认宽度。
#:
#: 必须大于"正常乱序能拉开的距离"，否则会把真实的乱序包误判为攻击。
#: 发送端每个周期递增 1，乱序距离受在途包数限制；64 远大于实测的最大
#: 乱序距离，又远小于冒充者的跃变尺度（10^7 量级）。
DEFAULT_SEQ_WINDOW = 64


def accept(msg: Msg, view: LocalView, dedup: DedupTable,
           t: float, seq_window: int = DEFAULT_SEQ_WINDOW) -> AcceptDecision:
    """判定是否接受一条消息。

    规则顺序不可调换：任期 -> 会话 -> 任务版本 -> 去重 -> 序列。
    旧任期的消息即使会话未知，也必须报 STALE_EPOCH。
    """
    if msg.leader_epoch < view.leader_epoch:
        return AcceptDecision.STALE_EPOCH
    if view.sessions and msg.session_id not in view.sessions:
        return AcceptDecision.STALE_SESSION
    if (msg.msg_type in MISSION_SCOPED
            and msg.mission_version < view.mission_version):
        return AcceptDecision.STALE_VERSION
    if msg.operation_id is not None and dedup.seen(msg.operation_id, t):
        return AcceptDecision.DEDUP

    # 序列号只在**同一个源的同一条消息流**内可比。心跳与遥测共用一个
    # 发送序号，但它们互不排序；若把 msg_type 排除在键外，一条乱序到达的
    # 遥测就会把紧随其后的心跳误判为过期，凭空造出一次"心跳丢失"。
    key = (msg.src, msg.msg_type, msg.leader_epoch, msg.mission_version)
    # 序号必须落在**滑动窗口**内，而不是无上界的单调比较。
    #
    # 原实现只有 `last_seq[key] = msg.seq`，没有任何上界。冒充者发送一个
    # seq = 10^7 的包就能把 last_seq 推到 10^7，此后所有真实心跳（seq
    # 只有几十）都满足 seq <= previous 而被判 STALE_SEQ——**单封包 DoS**。
    #
    # 关键：越窗时**不推进窗口**。否则冒充者只要连发 last_seq + W 就能
    # 把窗口一步步推到任意远处，DoS 依旧成立。
    previous = view.last_seq.get(key)
    if previous is not None:
        if msg.seq <= previous:
            return AcceptDecision.STALE_SEQ
        if msg.seq > previous + seq_window:
            return AcceptDecision.SEQ_OUT_OF_WINDOW

    view.leader_epoch = max(view.leader_epoch, msg.leader_epoch)
    view.mission_version = max(view.mission_version, msg.mission_version)
    view.last_seq[key] = msg.seq
    return AcceptDecision.ACCEPT
