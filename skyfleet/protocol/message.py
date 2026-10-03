"""统一消息格式。

字段对应报告 §2.2.4：源、目的、会话、任期、任务版本、路由版本、序列、
优先级、有效期、跳数、TTL、完整性校验。
"""
from __future__ import annotations

import enum
import itertools
import math
from dataclasses import dataclass
from typing import Any


class Qos(enum.IntEnum):
    """优先级等级（报告 §2.2.5）。数值越小越紧急。"""

    Q0 = 0   # 紧急控制与避障
    Q1 = 1   # 主节点切换、任务、投递确认
    Q2 = 2   # 成员与通信关系
    Q3 = 3   # 普通遥测与位置状态
    Q4 = 4   # 大数据业务


class Rel(enum.IntEnum):
    """可靠性等级（报告表 2-3）。"""

    R0 = 0   # 不重传，最新值覆盖
    R1 = 1   # 有限次重传
    R2 = 2   # ACK + 超时重传 + 去重
    R3 = 3   # 高优先级 + 重复发送 + 备用路径


#: 消息类型 -> (优先级, 可靠性等级)
MSG_TYPES: dict[str, tuple[Qos, Rel]] = {
    "EMERGENCY": (Qos.Q0, Rel.R3),
    "TASK_ASSIGN": (Qos.Q1, Rel.R2),
    "DELIVERY_CONFIRM": (Qos.Q1, Rel.R2),
    "ACK": (Qos.Q1, Rel.R2),
    "HEARTBEAT": (Qos.Q2, Rel.R1),
    "MEMBER_UPDATE": (Qos.Q2, Rel.R1),
    "ROUTE_UPDATE": (Qos.Q2, Rel.R1),
    "JOIN_REQUEST": (Qos.Q2, Rel.R1),
    "TELEMETRY": (Qos.Q3, Rel.R0),
    "BULK": (Qos.Q4, Rel.R0),
}

#: 各可靠性等级的默认有效期（秒）。R0 允许最新值覆盖，故永不过期。
DEADLINE_BY_REL: dict[Rel, float] = {
    Rel.R0: math.inf,
    Rel.R1: 2.0,
    Rel.R2: 1.0,
    Rel.R3: 0.05,
}

_DEFAULT_TTL = 8
_msg_counter = itertools.count(1)


@dataclass(frozen=True)
class Msg:
    """不可变消息。用 Msg.new() 构造，它会按消息类型填好 qos/rel/deadline。"""

    msg_id: str
    msg_type: str
    src: str
    dst: str
    session_id: str
    leader_epoch: int
    mission_version: int
    route_version: int
    seq: int
    operation_id: str | None
    qos: Qos
    reliability: Rel
    t_created: float
    deadline: float
    hop: int
    ttl: int
    payload: dict[str, Any]
    checksum: str

    @classmethod
    def new(cls, msg_type: str, src: str, dst: str, t: float,
            **overrides: Any) -> "Msg":
        """按消息类型构造消息。未知类型抛 KeyError。"""
        if msg_type not in MSG_TYPES:
            raise KeyError(msg_type)
        qos, rel = MSG_TYPES[msg_type]
        fields: dict[str, Any] = {
            "msg_id": f"m{next(_msg_counter)}",
            "msg_type": msg_type,
            "src": src,
            "dst": dst,
            "session_id": "",
            "leader_epoch": 0,
            "mission_version": 0,
            "route_version": 0,
            "seq": 0,
            "operation_id": None,
            "qos": qos,
            "reliability": rel,
            "t_created": t,
            "deadline": t + DEADLINE_BY_REL[rel],
            "hop": 0,
            "ttl": _DEFAULT_TTL,
            "payload": {},
            "checksum": "",           # P0 占位；P1 接管完整性
        }
        fields.update(overrides)
        return cls(**fields)

    @property
    def size_bits(self) -> int:
        """估算消息大小，用于信道占用时间与通信开销统计。"""
        base = 256                                    # 头部固定开销
        return base + 64 * len(self.payload)
