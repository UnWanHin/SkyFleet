"""从事件日志重建回放时间线。

**只读日志**。SCENARIO_INIT 事件已把世界描述（区域、楼群、节点初始位姿）
写进日志，因此一份日志文件就足以重建当时的画面，不需要附带配置文件。
"""
from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

from skyfleet.observe.log import read_log

#: 回放内嵌的消息事件上限——避免把整条消息流塞进 HTML。
MESSAGE_CAP = 20_000

REPLAY_MESSAGE_KINDS = frozenset({
    "MSG_DROP", "MSG_DUP", "MSG_EXPIRE", "MSG_FORWARD",
    "TASK_CREATE", "TASK_ASSIGN", "TASK_REASSIGN", "TASK_PICKUP",
    "TASK_DELIVER", "STATUS_CHANGE", "OCCLUSION_ENTER", "OCCLUSION_EXIT",
    "HEARTBEAT_MISS", "SAFETY_ACTION", "NODE_JOIN", "NODE_LEAVE",
})


def build_timeline(log_path: str | Path) -> dict[str, Any]:
    events = list(read_log(log_path))

    world: dict[str, Any] = {"area": 0.0, "duration": 0.0, "seed": 0,
                             "leader_id": "", "buildings": []}
    nodes: dict[str, dict[str, Any]] = {}
    tasks: dict[str, dict[str, Any]] = {}
    messages: list[list[Any]] = []

    for event in events:
        kind = event["kind"]
        t = event["t"]

        if kind == "SCENARIO_INIT":
            world["area"] = event.get("area", 0.0)
            world["duration"] = event.get("duration", 0.0)
            world["seed"] = event.get("seed", 0)
            world["leader_id"] = event.get("leader_id", "")
            world["buildings"] = event.get("buildings", [])
            for node_id, role, pos in event.get("nodes", []):
                nodes[node_id] = {
                    "role": role, "session": "", "samples": [[0.0, *pos]],
                    "status": [[0.0, "ONLINE"]], "energy": [],
                }
            continue

        if kind == "NODE_JOIN":
            node = nodes.setdefault(event["node_id"],
                                    {"role": "?", "samples": [],
                                     "status": [], "energy": []})
            node["session"] = event.get("session_id", "")
            continue

        if kind == "ENERGY_SAMPLE":
            node = nodes.get(event["node_id"])
            if node is not None:
                pos = event.get("pos")
                if pos:
                    node["samples"].append([t, *pos])
                node["energy"].append([t, event.get("energy", 0.0)])
            continue

        if kind == "STATUS_CHANGE":
            node = nodes.get(event["node_id"])
            if node is not None:
                node["status"].append([t, event["status"]])
            messages.append([t, kind, event.get("node_id", ""),
                             event.get("status", "")])
            continue

        if kind == "TASK_CREATE":
            tasks[event["task_id"]] = {
                "origin": event["origin"], "destination": event["destination"],
                "created": t, "assigned": None, "assignee": None,
                "picked": None, "delivered": None, "state": "CREATED",
            }
        elif kind == "TASK_ASSIGN":
            task = tasks.get(event["task_id"])
            if task is not None:
                task["assigned"] = t
                task["assignee"] = event.get("to_node")
                task["state"] = "ASSIGNED"
        elif kind == "TASK_PICKUP":
            task = tasks.get(event["task_id"])
            if task is not None:
                task["picked"] = t
                task["state"] = "PICKED"
        elif kind == "TASK_DELIVER":
            task = tasks.get(event["task_id"])
            if task is not None:
                task["delivered"] = t
                task["state"] = "DELIVERED"
        elif kind == "TASK_REASSIGN":
            task = tasks.get(event["task_id"])
            if task is not None:
                task["state"] = "REASSIGNED"

        if kind in REPLAY_MESSAGE_KINDS and len(messages) < MESSAGE_CAP:
            messages.append([t, kind, event.get("node_id", msg_src(event)),
                             event.get("reason") or event.get("msg_type") or
                             event.get("task_id") or event.get("status") or ""])

    return {
        **world,
        "nodes": nodes,
        "tasks": tasks,
        "events": messages,
        "counts": _counts(events),
    }


def msg_src(event: dict[str, Any]) -> str:
    return event.get("src", "")


def _counts(events: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)
    for event in events:
        counts[event["kind"]] += 1
    return dict(counts)
