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
                 identity: IdentityRegistry, log: Any,
                 busy_nodes: frozenset[str] = frozenset(),
                 grounded_nodes: set[str] | None = None) -> None:
        self.config = config
        self.board = board
        self.members = members
        self.agent = agent
        self.identity = identity
        self.log = log
        #: 不承担配送任务的节点（中继）。中继是**通信基础设施**：一旦
        #: 让它去送货，它就会离开站位、飞进楼群低空，既不再提供中继覆盖，
        #: 又把自己变成最容易被遮挡的节点——中继的价值恰好被抹掉。
        self.busy_nodes = busy_nodes
        #: 已迫降/坠机、不可再承担配送任务的节点。由引擎持有并原地更新，
        #: 因此这里保存的是**同一个可变集合**而不是副本。
        self.grounded_nodes = (grounded_nodes if grounded_nodes is not None
                               else set())
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
        """由索引确定性推导一个点（不使用随机流，保证场景可复现）。

        高度取自**投递高度带**：取餐点与投递点都在街道峡谷内，
        因此"下降投递"这一航段必然穿过楼群。
        """
        area = self.config.area
        x = ((index * 73 + salt * 29) % 97) / 97.0 * area
        y = ((index * 41 + salt * 17) % 89) / 89.0 * area
        low, high = self.config.delivery_altitude
        z = low + ((index * 13 + salt) % 7) / 7.0 * (high - low)
        return (x, y, z)

    # ---------- 分配 ----------

    def fresh_members(self) -> list[str]:
        """有能力承担任务的成员（区别于"仅仅在线"）。

        中继被排除在外：它承担的是转发，不是配送。详见 busy_nodes 的说明。
        """
        return [n for n in self.members.available(self.config.min_energy_for_task)
                if n != self.config.leader_id
                and n not in self.busy_nodes
                and n not in self.grounded_nodes]

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

    def mark_failed(self, task_id: str, t: float) -> bool:
        """任务执行失败（例如投递确认始终无法送达）——作废并等待重派。"""
        task = self.board.get(task_id)
        if task is None or task.state in (TaskState.DELIVERED,
                                          TaskState.EXPIRED):
            return False
        if not self.board.transition(task_id, TaskState.FAILED, t):
            return False
        if not self.board.transition(task_id, TaskState.REASSIGNED, t):
            return False
        task.assignee = None
        self.assignments.pop(task_id, None)
        self.mission_version += 1
        return True
