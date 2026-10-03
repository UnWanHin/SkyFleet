"""离散事件调度器。

单线程、确定性、不使用真实睡眠。这是整个仿真器的心脏。
"""
from __future__ import annotations

import heapq
from typing import Any, Callable

from skyfleet.kernel.event import Event

Handler = Callable[[Event], None]


class Scheduler:
    """以 (t, tie) 为键的事件优先队列。"""

    def __init__(self) -> None:
        self._heap: list[tuple[float, int, Event]] = []
        self._tie = 0
        self._now = 0.0
        self._handlers: dict[str, list[Handler]] = {}

    @property
    def now(self) -> float:
        return self._now

    def subscribe(self, kind: str, handler: Handler) -> None:
        """注册某类事件的处理器。同类型可注册多个，按注册顺序调用。"""
        self._handlers.setdefault(kind, []).append(handler)

    def schedule_at(self, t: float, kind: str, payload: Any = None) -> Event:
        """在绝对时刻 t 安排事件。"""
        self._tie += 1
        ev = Event(t=t, kind=kind, payload=payload, tie=self._tie)
        heapq.heappush(self._heap, (ev.t, ev.tie, ev))
        return ev

    def schedule(self, delay: float, kind: str, payload: Any = None) -> Event:
        """在当前时刻之后 delay 秒安排事件。负延迟按 0 处理。"""
        return self.schedule_at(self._now + max(0.0, delay), kind, payload)

    def cancel(self, event: Event) -> bool:
        """取消事件。已取消过则返回 False。"""
        if event.cancelled:
            return False
        event.cancelled = True
        return True

    def pending(self) -> int:
        """堆中剩余事件数（含已取消）。"""
        return len(self._heap)

    def run(self, until: float | None = None) -> None:
        """事件循环。until 为虚拟时间上界（含该时刻的事件不执行）。"""
        while self._heap:
            t, _, ev = self._heap[0]
            if until is not None and t > until:
                break
            heapq.heappop(self._heap)
            if ev.cancelled:
                continue
            self._now = t
            for handler in self._handlers.get(ev.kind, ()):
                handler(ev)
