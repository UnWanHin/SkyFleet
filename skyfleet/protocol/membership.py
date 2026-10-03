"""带迟滞的成员状态机。

这是全案最核心的机制。报告 §2.2.1 要求"不能把单次丢包直接等同于无人机故障"，
做法是把判定门限参数化，并引入三路证据：

1. 直接心跳（miss_count）
2. 邻居/中继的间接探测（抑制误判）
3. 链路质量趋势

城市遮挡常造成"主节点看不到从节点，但从节点之间仍可互通"，
缺少第 2 路证据就会把一次遮挡误判为坠机并触发无谓的任务重派。
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass


class MemberStatus(enum.Enum):
    ONLINE = "ONLINE"
    DEGRADED = "DEGRADED"
    SUSPECT = "SUSPECT"
    FAILED = "FAILED"
    LEFT = "LEFT"
    REJOINING = "REJOINING"


@dataclass(frozen=True)
class MembershipParams:
    """判定门限。全部来自配置，不得硬编码。"""

    n_degraded: int = 2
    n_suspect: int = 4
    t_fail: float = 3.0
    q_degraded: float = 0.4


@dataclass
class MemberRecord:
    node_id: str
    session_id: str
    status: MemberStatus = MemberStatus.ONLINE
    miss_count: int = 0
    last_seen: float = 0.0
    last_evidence_at: float = 0.0
    status_since: float = 0.0
    heartbeat_seq: int = 0
    link_quality: float = 1.0
    energy: float = math.inf


class MembershipView:
    """主节点或任意节点持有的成员视图。"""

    def __init__(self, params: MembershipParams) -> None:
        self.params = params
        self._records: dict[str, MemberRecord] = {}

    # ---------- 查询 ----------

    def status(self, node_id: str) -> MemberStatus | None:
        rec = self._records.get(node_id)
        return rec.status if rec else None

    def record(self, node_id: str) -> MemberRecord | None:
        return self._records.get(node_id)

    def members(self, statuses: set[MemberStatus] | None = None) -> list[str]:
        if statuses is None:
            return sorted(self._records)
        return sorted(n for n, r in self._records.items() if r.status in statuses)

    def available(self, min_energy: float) -> list[str]:
        """区分"在线"与"可用"：低电量节点仍在线，但不适合承担任务。"""
        return sorted(n for n, r in self._records.items()
                      if r.status is MemberStatus.ONLINE and r.energy >= min_energy)

    # ---------- 事件 ----------

    def on_join(self, node_id: str, session_id: str, t: float) -> MemberRecord:
        rec = MemberRecord(node_id=node_id, session_id=session_id,
                           status=MemberStatus.ONLINE, last_seen=t,
                           last_evidence_at=t, status_since=t)
        self._records[node_id] = rec
        return rec

    def on_heartbeat(self, node_id: str, session_id: str, t: float,
                     link_quality: float, heartbeat_seq: int) -> MemberStatus:
        rec = self._records.get(node_id)
        if rec is None:
            rec = self.on_join(node_id, session_id, t)
        rec.session_id = session_id
        rec.miss_count = 0
        rec.last_seen = t
        rec.last_evidence_at = t
        rec.heartbeat_seq = heartbeat_seq
        rec.link_quality = link_quality
        target = (MemberStatus.DEGRADED if link_quality < self.params.q_degraded
                  else MemberStatus.ONLINE)
        self._transition(rec, target, t)
        return rec.status

    def on_miss(self, node_id: str, t: float) -> MemberStatus | None:
        rec = self._records.get(node_id)
        if rec is None:
            return None
        if rec.status in (MemberStatus.FAILED, MemberStatus.LEFT):
            return rec.status

        rec.miss_count += 1
        recent_evidence = (t - rec.last_evidence_at) < self.params.t_fail
        if (rec.miss_count >= self.params.n_suspect
                and not recent_evidence
                and rec.status is not MemberStatus.SUSPECT):
            self._transition(rec, MemberStatus.SUSPECT, t)
        elif (rec.miss_count >= self.params.n_degraded
                and rec.status is MemberStatus.ONLINE):
            self._transition(rec, MemberStatus.DEGRADED, t)
        return rec.status

    def on_indirect(self, node_id: str, t: float) -> None:
        """邻居或中继报告"曾见到该节点"——刷新证据，抑制误判。"""
        rec = self._records.get(node_id)
        if rec is None:
            return
        rec.last_evidence_at = t
        if rec.status is MemberStatus.SUSPECT:
            self._transition(rec, MemberStatus.DEGRADED, t)

    def on_leave(self, node_id: str, t: float) -> None:
        rec = self._records.get(node_id)
        if rec is not None:
            self._transition(rec, MemberStatus.LEFT, t)

    def set_energy(self, node_id: str, energy: float) -> None:
        rec = self._records.get(node_id)
        if rec is not None:
            rec.energy = energy

    def tick(self, t: float) -> list[tuple[str, MemberStatus]]:
        """把停留过久的 SUSPECT 转为 FAILED，返回本次发生的状态变化。"""
        changes: list[tuple[str, MemberStatus]] = []
        for node_id, rec in self._records.items():
            if (rec.status is MemberStatus.SUSPECT
                    and t - rec.status_since >= self.params.t_fail):
                self._transition(rec, MemberStatus.FAILED, t)
                changes.append((node_id, rec.status))
        return changes

    # ---------- 内部 ----------

    def _transition(self, rec: MemberRecord, status: MemberStatus,
                    t: float) -> None:
        if rec.status is status:
            return
        rec.status = status
        rec.status_since = t
        if status in (MemberStatus.FAILED, MemberStatus.LEFT):
            rec.miss_count = 0
