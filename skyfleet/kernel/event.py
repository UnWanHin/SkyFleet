"""仿真事件对象。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class Event:
    """调度器里的事件。

    tie 是单调递增序号，用于让同一时刻的事件有确定顺序；
    否则堆的内部顺序会造成两次运行结果不一致。
    """

    t: float
    kind: str
    payload: Any = None
    tie: int = 0
    cancelled: bool = False

    def sort_key(self) -> tuple[float, int]:
        return (self.t, self.tie)

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return f"Event(t={self.t:.6f}, kind={self.kind!r}, tie={self.tie})"
