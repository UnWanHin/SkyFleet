"""测试替身。"""
from __future__ import annotations


class FakeTransport:
    """记录发送意图，不做任何信道建模。"""

    def __init__(self) -> None:
        self.sent: list[tuple[object, str, float]] = []

    def transmit(self, msg, sender, t: float) -> None:
        self.sent.append((msg, sender.node.node_id, t))

    def messages_of_type(self, msg_type: str) -> list:
        return [m for m, _, _ in self.sent if m.msg_type == msg_type]

    def clear(self) -> None:
        self.sent.clear()
