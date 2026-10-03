"""任务生命周期。

状态机与允许的转移显式列在 ALLOWED 里：报告要求识别"过期任务"与"失败任务"，
并且非法转移必须被拒绝，而不是静默接受。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

Pos = tuple[float, float, float]


class TaskState(str, enum.Enum):
    CREATED = "CREATED"
    ASSIGNED = "ASSIGNED"
    ENROUTE = "ENROUTE"
    PICKED = "PICKED"
    DELIVERED = "DELIVERED"
    FAILED = "FAILED"
    EXPIRED = "EXPIRED"
    REASSIGNED = "REASSIGNED"


ALLOWED: dict[TaskState, frozenset[TaskState]] = {
    TaskState.CREATED: frozenset({TaskState.ASSIGNED, TaskState.EXPIRED}),
    TaskState.ASSIGNED: frozenset({TaskState.ENROUTE, TaskState.FAILED,
                                   TaskState.REASSIGNED}),
    TaskState.ENROUTE: frozenset({TaskState.PICKED, TaskState.FAILED}),
    TaskState.PICKED: frozenset({TaskState.DELIVERED, TaskState.FAILED}),
    TaskState.DELIVERED: frozenset(),
    TaskState.FAILED: frozenset({TaskState.REASSIGNED}),
    TaskState.EXPIRED: frozenset(),
    TaskState.REASSIGNED: frozenset({TaskState.ASSIGNED, TaskState.EXPIRED}),
}

ASSIGNABLE_FROM = frozenset({TaskState.CREATED, TaskState.REASSIGNED})


@dataclass
class Task:
    task_id: str
    origin: Pos
    destination: Pos
    deadline: float
    priority: int = 1
    state: TaskState = TaskState.CREATED
    assignee: str | None = None
    created_at: float = 0.0
    delivered_at: float | None = None
    attempts: int = 0


class TaskBoard:
    """任务集合与状态转移。"""

    def __init__(self) -> None:
        self._tasks: dict[str, Task] = {}

    def create(self, task_id: str, origin: Pos, destination: Pos,
               deadline: float, t: float, priority: int = 1) -> Task:
        task = Task(task_id=task_id, origin=origin, destination=destination,
                    deadline=deadline, priority=priority, created_at=t)
        self._tasks[task_id] = task
        return task

    def get(self, task_id: str) -> Task | None:
        return self._tasks.get(task_id)

    def all(self) -> list[Task]:
        return list(self._tasks.values())

    def transition(self, task_id: str, new_state: TaskState, t: float) -> bool:
        task = self._tasks.get(task_id)
        if task is None:
            return False
        if new_state not in ALLOWED[task.state]:
            return False
        task.state = new_state
        if new_state is TaskState.DELIVERED:
            task.delivered_at = t
        return True

    def assign(self, task_id: str, node_id: str, t: float) -> bool:
        task = self._tasks.get(task_id)
        if task is None or task.state not in ASSIGNABLE_FROM:
            return False
        task.assignee = node_id
        task.attempts += 1
        task.state = TaskState.ASSIGNED
        return True

    def counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for task in self._tasks.values():
            counts[task.state.value] = counts.get(task.state.value, 0) + 1
        return counts

    def completion_rate(self) -> float:
        if not self._tasks:
            return 0.0
        delivered = sum(1 for t in self._tasks.values()
                        if t.state is TaskState.DELIVERED)
        return delivered / len(self._tasks)
