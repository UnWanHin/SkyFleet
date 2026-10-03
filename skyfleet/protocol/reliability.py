"""R0-R3 可靠性策略。

关键命题（报告表 4-1「可靠重传与信息时效」）：重传不是无条件的。
超过消息 deadline 后必须停止重传并记 DEADLINE_MISSED，
否则重传只会把已经失效的控制包继续灌进信道。
"""
from __future__ import annotations

from dataclasses import dataclass

from skyfleet.protocol.message import Msg, Rel

#: 各可靠性等级的最大重传次数（报告表 2-3）
MAX_RETRIES: dict[Rel, int] = {
    Rel.R0: 0,
    Rel.R1: 2,
    Rel.R2: 3,
    Rel.R3: 2,
}


def max_retries(rel: Rel) -> int:
    return MAX_RETRIES[rel]


@dataclass
class _PendingTx:
    msg: Msg
    attempts: int = 0
    acked: bool = False
    next_retry_at: float = 0.0


class ReliabilityManager:
    """追踪需要 ACK 的在途消息，并决定何时重传、何时放弃。"""

    def __init__(self, retry_interval: float = 1.0) -> None:
        self.retry_interval = retry_interval
        self._pending: dict[str, _PendingTx] = {}

    def register(self, msg: Msg, t: float) -> bool:
        """登记一条需要确认的消息。R0 不追踪（不重传、允许覆盖）。"""
        if msg.reliability == Rel.R0:
            return False
        self._pending[msg.msg_id] = _PendingTx(
            msg=msg, next_retry_at=t + self.retry_interval)
        return True

    def ack(self, msg_id: str) -> bool:
        tx = self._pending.get(msg_id)
        if tx is None or tx.acked:
            return False
        tx.acked = True
        return True

    def due_retries(self, t: float) -> list[Msg]:
        """到期的重传。调用即推进该消息的下次重传时刻，避免重复触发。"""
        due: list[Msg] = []
        for tx in self._pending.values():
            if tx.acked:
                continue
            if tx.attempts >= max_retries(tx.msg.reliability):
                continue
            if t < tx.next_retry_at:
                continue
            if t > tx.msg.deadline:
                continue
            tx.next_retry_at = t + self.retry_interval
            due.append(tx.msg)
        return due

    def on_retry_sent(self, msg_id: str, t: float) -> None:
        tx = self._pending.get(msg_id)
        if tx is None:
            return
        tx.attempts += 1
        tx.next_retry_at = t + self.retry_interval

    def exhausted(self, msg_id: str) -> bool:
        tx = self._pending.get(msg_id)
        if tx is None:
            return False
        return tx.attempts >= max_retries(tx.msg.reliability)

    def pending_count(self) -> int:
        return sum(1 for tx in self._pending.values() if not tx.acked)

    def expired(self, t: float) -> list[Msg]:
        """超过有效期且未确认的消息——引擎据此记 DEADLINE_MISSED。"""
        return [tx.msg for tx in self._pending.values()
                if not tx.acked and t > tx.msg.deadline]

    def forget(self, msg_id: str) -> None:
        self._pending.pop(msg_id, None)
