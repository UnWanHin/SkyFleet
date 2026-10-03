"""身份、会话、角色与租约。

核心约束（报告 §2.2.2）：node_id 不随网络位置变化；每次重连生成新 session_id；
权限租约到期自动失效，不需要显式撤销消息。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass


class Role(str, enum.Enum):
    LEADER = "LEADER"
    SLAVE = "SLAVE"
    RELAY = "RELAY"
    STANDBY = "STANDBY"


@dataclass(frozen=True)
class Lease:
    """权限凭证：节点、角色、任期、权限范围、有效期。"""

    node_id: str
    role: Role
    epoch: int
    scope: str
    expire_at: float

    def valid(self, t: float) -> bool:
        return t < self.expire_at


class IdentityRegistry:
    """节点长期身份与短期会话的登记处。"""

    def __init__(self) -> None:
        self._joins: dict[str, int] = {}
        self._sessions: dict[str, str] = {}
        self._leases: dict[str, Lease] = {}

    def join(self, node_id: str, t: float) -> str:
        """建立一次新会话，返回新的 session_id。旧的同节点会话立即失效。"""
        count = self._joins.get(node_id, 0) + 1
        self._joins[node_id] = count
        session_id = f"{node_id}#{count}"
        self._sessions[node_id] = session_id
        return session_id

    def current_session(self, node_id: str) -> str | None:
        return self._sessions.get(node_id)

    def is_valid_session(self, node_id: str, session_id: str) -> bool:
        return self._sessions.get(node_id) == session_id

    def grant_lease(self, node_id: str, role: Role, epoch: int, scope: str,
                    t: float, ttl: float) -> Lease:
        lease = Lease(node_id, role, epoch, scope, t + ttl)
        self._leases[node_id] = lease
        return lease

    def active_leases(self, t: float) -> list[Lease]:
        return [lease for lease in self._leases.values() if lease.valid(t)]

    def lease_for(self, node_id: str, t: float) -> Lease | None:
        lease = self._leases.get(node_id)
        if lease is None or not lease.valid(t):
            return None
        return lease
