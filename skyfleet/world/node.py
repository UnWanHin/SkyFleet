"""节点状态。

能量被建模为一等状态：报告要求"能耗趋势"指标，并要求区分
"节点在线"与"节点可用"——低电量节点仍在线，但不适合承担转发或配送。
"""
from __future__ import annotations

from dataclasses import dataclass, field

from skyfleet.protocol.identity import Role

Pos = tuple[float, float, float]


@dataclass
class Node:
    node_id: str
    role: Role
    pos: Pos
    speed: float
    energy: float
    session_id: str = ""
    #: 累计发送/转发字节数，用于"单位任务通信开销"
    bytes_sent: int = 0
    bytes_forwarded: int = 0
    energy_spent: float = field(default=0.0)

    def consume(self, amount: float) -> None:
        """消耗能量，下界为 0。"""
        spent = min(self.energy, amount)
        self.energy -= spent
        self.energy_spent += spent

    def is_useful(self, min_energy: float) -> bool:
        """是否仍有能力承担任务（区别于"是否在线"）。"""
        return self.energy >= min_energy
