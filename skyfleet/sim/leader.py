"""主节点任务编排。

职责：生成任务、按可用成员分配、成员失效时重派并递增 mission_version。
mission_version 递增是关键——它让在途的旧指派消息自然失效，
这就是报告表 4-1"旧任务不能触发错误动作"的落地方式。
"""
from __future__ import annotations

from typing import Any

from skyfleet.kernel.config import SimConfig
from skyfleet.mission.task import TaskBoard, TaskState
from skyfleet.observe.events import EventKind
from skyfleet.protocol.identity import IdentityRegistry
from skyfleet.protocol.membership import MembershipView
from skyfleet.sim.agent import NodeAgent

Pos = tuple[float, float, float]


class LeaderController:
    def __init__(self, config: SimConfig, board: TaskBoard,
                 members: MembershipView, agent: NodeAgent,
                 identity: IdentityRegistry, log: Any) -> None:
        self.config = config
        self.board = board
        self.members = members
        self.agent = agent
        self.identity = identity
        self.log = log
        self.mission_version = 0
        #: task_id -> node_id
        self.assignments: dict[str, str] = {}

    # ---------- 场景生成 ----------

    def create_tasks(self, t: float) -> int:
        """生成配置数量的配送任务。取餐点/投递点由索引确定性推导。"""
        for i in range(self.config.tasks):
            task_id = f"T{i + 1:03d}"
            origin = self._point(i, salt=1)
            destination = self._point(i, salt=6)
            deadline = t + 300.0
            self.board.create(task_id, origin, destination, deadline, t)
            self.log.write(EventKind.TASK_CREATE, t, task_id=task_id,
                           origin=list(origin), destination=list(destination),
                           deadline=deadline)
        return self.config.tasks

    def _point(self, index: int, salt: int) -> Pos:
        """由索引确定性推导一个点（不使用随机流，保证场景可复现）。"""
        area = self.config.area
        x = ((index * 73 + salt * 29) % 97) / 97.0 * area
        y = ((index * 41 + salt * 17) % 89) / 89.0 * area
        low, high = self.config.flight_altitude
        z = low + ((index * 13 + salt) % 7) / 7.0 * (high - low)
        return (x, y, z)

    # ---------- 分配 ----------

    def fresh_members(self) -> list[str]:
        """有能力承担任务的成员（区别于"仅仅在线"）。"""
        return [n for n in self.members.available(self.config.min_energy_for_task)
                if n != self.config.leader_id]

    def assign_pending(self, t: float) -> int:
        """把 CREATED / REASSIGNED 的任务轮流分配给可用成员。"""
        available = self.fresh_members()
        if not available:
            return 0

        count = 0
        for task in self.board.all():
            if task.state not in (TaskState.CREATED, TaskState.REASSIGNED):
                continue
            node_id = available[count % len(available)]
            if not self.board.assign(task.task_id, node_id, t):
                continue
            self.assignments[task.task_id] = node_id
            self.agent.submit(self._assign_msg(task, node_id, t), t)
            self.log.write(EventKind.TASK_ASSIGN, t, task_id=task.task_id,
                           to_node=node_id,
                           mission_version=self.mission_version)
            count += 1
        return count

    def _assign_msg(self, task, node_id: str, t: float):
        from skyfleet.protocol.message import Msg
        return Msg.new(
            "TASK_ASSIGN", src=self.config.leader_id, dst=node_id, t=t,
            msg_id=self.agent.new_msg_id(), seq=self.agent.next_seq(),
            session_id=self.agent.session_id,
            leader_epoch=self.agent.leader_epoch,
            mission_version=self.mission_version,
            operation_id=task.task_id,
            payload={"task_id": task.task_id,
                     "origin": list(task.origin),
                     "destination": list(task.destination),
                     "deadline": task.deadline,
                     "priority": task.priority})

    # ---------- 失效与重派 ----------

    def on_member_failed(self, node_id: str, t: float) -> int:
        """成员失效：其名下未完成任务全部作废并等待重派。"""
        affected = sorted(tid for tid, n in self.assignments.items()
                          if n == node_id)
        if not affected:
            return 0

        reassigned = 0
        for task_id in affected:
            task = self.board.get(task_id)
            if task is None or task.state in (TaskState.DELIVERED,
                                              TaskState.EXPIRED):
                self.assignments.pop(task_id, None)
                continue
            self.board.transition(task_id, TaskState.FAILED, t)
            if not self.board.transition(task_id, TaskState.REASSIGNED, t):
                continue
            task.assignee = None
            self.assignments.pop(task_id, None)
            reassigned += 1

        if reassigned:
            self.mission_version += 1
            for task_id in affected:
                task = self.board.get(task_id)
                if task is not None and task.state is TaskState.REASSIGNED:
                    self.log.write(EventKind.TASK_REASSIGN, t,
                                   task_id=task_id, from_node=node_id,
                                   mission_version=self.mission_version)
        return reassigned

    def mark_delivered(self, task_id: str, t: float) -> bool:
        if not self.board.transition(task_id, TaskState.DELIVERED, t):
            return False
        self.assignments.pop(task_id, None)
        return True
