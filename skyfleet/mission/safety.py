"""本地安全动作。

报告 §2.2.6：当主节点或通信关系不可用时，节点仍需执行悬停、返航、降速、
等待或其他预先定义的安全动作——也就是说这些决策**不能依赖主节点**。

优先级：禁飞区 > 低电量 > 位置过期 > 失联。
"""
from __future__ import annotations

import enum


class SafetyAction(str, enum.Enum):
    NONE = "NONE"
    HOVER = "HOVER"
    RETURN = "RETURN"
    DESCEND = "DESCEND"
    HOLD = "HOLD"


def safety_decision(link_lost: bool = False, leader_lost: bool = False,
                    low_battery: bool = False, position_stale: bool = False,
                    geofence: bool = False) -> SafetyAction:
    """按优先级给出本地安全动作。"""
    if geofence:
        return SafetyAction.HOLD
    if low_battery:
        return SafetyAction.RETURN
    if position_stale:
        return SafetyAction.HOVER
    if link_lost or leader_lost:
        return SafetyAction.HOVER
    return SafetyAction.NONE
