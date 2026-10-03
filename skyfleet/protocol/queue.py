"""发送队列与高优先级限流。

报告 §2.2.5 同时要求两件相互拉扯的事：
- 消息分级，高优先级先走（严格优先级）
- 对高优先级消息设速率限制，防止异常节点灌爆信道

严格优先级的已知代价是低优先级饿死，因此 Q0/Q1 必须配 token bucket。
"""
from __future__ import annotations

from collections import deque

from skyfleet.observe.events import DropReason
from skyfleet.protocol.message import Msg, Qos, Rel


class TokenBucket:
    """令牌桶。用于给高优先级消息设速率上限。"""

    def __init__(self, rate_per_s: float, burst: float) -> None:
        self.rate = rate_per_s
        self.burst = burst
        self._tokens = burst
        self._last = 0.0

    def allow(self, t: float) -> bool:
        elapsed = max(0.0, t - self._last)
        self._tokens = min(self.burst, self._tokens + elapsed * self.rate)
        self._last = t
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False


class SendQueue:
    """每个 QoS 等级一条 FIFO；R0 消息按 (类型, 源) 做最新值覆盖。"""

    def __init__(self, depth_limit: int, rng=None) -> None:
        self.depth_limit = depth_limit
        self.rng = rng
        self._queues: dict[Qos, deque[Msg]] = {q: deque() for q in Qos}
        #: (msg_type, src) -> 当前在队的 R0 消息，用于覆盖旧值
        self._latest_r0: dict[tuple[str, str], Msg] = {}

    def enqueue(self, msg: Msg, t: float) -> DropReason | None:
        """入队。返回 None 表示成功，否则返回丢弃原因。"""
        if msg.reliability == Rel.R0:
            key = (msg.msg_type, msg.src)
            previous = self._latest_r0.get(key)
            if previous is not None:
                self._drop_from_queue(previous)
                self._queues[msg.qos].append(msg)
                self._latest_r0[key] = msg
                return None

        if self.size() >= self.depth_limit:
            return DropReason.QUEUE_FULL

        self._queues[msg.qos].append(msg)
        if msg.reliability == Rel.R0:
            self._latest_r0[(msg.msg_type, msg.src)] = msg
        return None

    def dequeue(self, t: float) -> Msg | None:
        """按严格优先级取出下一条消息。"""
        for qos in Qos:
            queue = self._queues[qos]
            if queue:
                msg = queue.popleft()
                if msg.reliability == Rel.R0:
                    self._latest_r0.pop((msg.msg_type, msg.src), None)
                return msg
        return None

    def size(self, qos: Qos | None = None) -> int:
        if qos is None:
            return sum(len(q) for q in self._queues.values())
        return len(self._queues[qos])

    def sample(self) -> dict[str, int]:
        """供 QUEUE_SAMPLE 事件使用。"""
        return {q.name: len(self._queues[q]) for q in Qos}

    def _drop_from_queue(self, msg: Msg) -> None:
        try:
            self._queues[msg.qos].remove(msg)
        except ValueError:  # pragma: no cover - 理论上不会发生
            pass


class RateLimiter:
    """只对 Q0/Q1 限流；其余等级一律放行。"""

    def __init__(self, q0_rate: float, q0_burst: float,
                 q1_rate: float, q1_burst: float) -> None:
        self._buckets = {
            Qos.Q0: TokenBucket(q0_rate, q0_burst),
            Qos.Q1: TokenBucket(q1_rate, q1_burst),
        }

    def allow(self, msg: Msg, t: float) -> bool:
        bucket = self._buckets.get(msg.qos)
        if bucket is None:
            return True
        return bucket.allow(t)
