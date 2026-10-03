"""投递确认的幂等账本。

报告要求"保证重复到达时动作只执行一次"（表 2-2 operation_id 的作用）。
这里以 operation_id 为键，重复确认被拒绝并计数——该计数本身就是
P1 重放攻击测试的对照基线。
"""
from __future__ import annotations


class DeliveryLedger:
    """记录已执行的投递确认。"""

    def __init__(self) -> None:
        self._confirmed: set[str] = set()
        self.confirm_count = 0
        self.duplicate_count = 0

    def confirm(self, op_id: str, t: float) -> bool:
        """首次确认返回 True 并执行；重复确认返回 False。"""
        if op_id in self._confirmed:
            self.duplicate_count += 1
            return False
        self._confirmed.add(op_id)
        self.confirm_count += 1
        return True

    def is_confirmed(self, op_id: str) -> bool:
        return op_id in self._confirmed

    def __len__(self) -> int:
        return len(self._confirmed)
