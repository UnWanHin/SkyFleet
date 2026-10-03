"""从事件日志重算评价指标。

设计约束：**只读日志，不读仿真内存**。指标定义变化时不必重跑仿真，
并且指标与回放共用同一份数据，不会出现两套口径。
"""
from __future__ import annotations

import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from skyfleet.observe.log import read_log

CONTROL_QOS = ("Q0", "Q1", "Q2")
DATA_QOS = ("Q3", "Q4")


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return ordered[int(position)]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _summary(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p95": None,
                "p99": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "max": max(values),
    }


def compute_metrics(log_path: str | Path) -> dict[str, Any]:
    events = list(read_log(log_path))

    created: set[str] = set()
    delivered: set[str] = set()
    confirm_tx = confirm_rx = 0
    retries = 0
    latency: dict[str, list[float]] = defaultdict(list)
    tx_by_qos: Counter = Counter()
    rx_by_qos: Counter = Counter()
    drop_by_qos: Counter = Counter()
    drop_reasons: Counter = Counter()
    drop_by_reason_qos: Counter = Counter()
    bytes_tx = 0
    bytes_control = bytes_data = 0
    max_route_version = 0
    retries_per_msg: Counter = Counter()
    energy: dict[str, float] = {}
    energy_start: dict[str, float] = {}
    status_events: dict[str, list[tuple[float, str]]] = defaultdict(list)
    heartbeat_tx = heartbeat_rx = 0
    forwarded = 0
    expired = 0

    for event in events:
        kind = event["kind"]
        t = event["t"]

        if kind == "TASK_CREATE":
            created.add(event["task_id"])
        elif kind == "TASK_DELIVER":
            delivered.add(event["task_id"])
        elif kind == "MSG_TX":
            qos = event.get("qos", "Q?")
            tx_by_qos[qos] += 1
            bits = event.get("bits", 0)
            bytes_tx += bits // 8
            if qos in CONTROL_QOS:
                bytes_control += bits // 8
            else:
                bytes_data += bits // 8
            max_route_version = max(max_route_version,
                                    event.get("route_version", 0))
            if event.get("is_retry"):
                retries += 1
                retries_per_msg[event["msg_id"]] += 1
            if event.get("msg_type") == "DELIVERY_CONFIRM":
                confirm_tx += 1
            elif event.get("msg_type") == "HEARTBEAT":
                heartbeat_tx += 1
        elif kind == "MSG_RX":
            qos = event.get("qos", "Q?")
            rx_by_qos[qos] += 1
            if event.get("e2e") is not None:
                latency[qos].append(event["e2e"] * 1000.0)
            if event.get("msg_type") == "DELIVERY_CONFIRM":
                confirm_rx += 1
            elif event.get("msg_type") == "HEARTBEAT":
                heartbeat_rx += 1
        elif kind in ("MSG_DROP", "MSG_DUP"):
            reason = event.get("reason", "UNKNOWN")
            drop_reasons[reason] += 1
            drop_by_qos[event.get("qos", "Q?")] += 1
            drop_by_reason_qos[(reason, event.get("qos", "Q?"))] += 1
        elif kind == "MSG_FORWARD":
            forwarded += 1
        elif kind == "MSG_EXPIRE":
            expired += 1
        elif kind == "STATUS_CHANGE":
            status_events[event["node_id"]].append((t, event["status"]))
        elif kind == "ENERGY_SAMPLE":
            node_id = event["node_id"]
            energy_start.setdefault(node_id, event["energy"])
            energy[node_id] = event["energy"]

    recoveries: list[float] = []
    unrecovered: list[str] = []
    for node_id, changes in status_events.items():
        failed_at = next((t for t, s in changes if s == "FAILED"), None)
        if failed_at is None:
            continue
        back = next((t for t, s in changes
                     if t > failed_at and s in ("ONLINE", "REJOINING")), None)
        if back is None:
            unrecovered.append(node_id)
        else:
            recoveries.append(back - failed_at)

    def loss_rate(qos_group: tuple[str, ...]) -> float | None:
        sent = sum(tx_by_qos[q] for q in qos_group)
        lost = sum(drop_by_qos[q] for q in qos_group)
        return (lost / sent) if sent else None

    return {
        "events": len(events),
        "duration": max((e["t"] for e in events), default=0.0),
        "tasks": {
            "created": len(created),
            "delivered": len(delivered),
            "completion_rate": (len(delivered) / len(created)
                                if created else 0.0),
        },
        "delivery_confirm": {
            "sent": confirm_tx,
            "received": confirm_rx,
            "success_rate": (confirm_rx / confirm_tx) if confirm_tx else None,
        },
        "latency_ms": {qos: _summary(values)
                       for qos, values in sorted(latency.items())},
        "loss": {
            "control_rate": loss_rate(CONTROL_QOS),
            "data_rate": loss_rate(DATA_QOS),
            "tx_by_qos": dict(tx_by_qos),
            "rx_by_qos": dict(rx_by_qos),
            "drop_by_qos": dict(drop_by_qos),
        },
        "drop_reasons": dict(drop_reasons.most_common()),
        "drop_by_reason_qos": {f"{r}/{q}": c
                               for (r, q), c in
                               sorted(drop_by_reason_qos.items())},
        "retransmission": {
            "total": retries,
            "expired": expired,
            "max_per_message": max(retries_per_msg.values(), default=0),
        },
        "heartbeat": {
            "sent": heartbeat_tx,
            "received": heartbeat_rx,
            "delivery_rate": (heartbeat_rx / heartbeat_tx
                              if heartbeat_tx else None),
        },
        "recovery": {
            "recovered": len(recoveries),
            "unrecovered": sorted(unrecovered),
            "mean_s": (sum(recoveries) / len(recoveries)
                       if recoveries else None),
            "max_s": max(recoveries) if recoveries else None,
        },
        "route_switches": max_route_version,
        "forwarded": forwarded,
        "overhead": {
            "total_bytes": bytes_tx,
            "control_bytes": bytes_control,
            "data_bytes": bytes_data,
            "bytes_per_delivered_task": (bytes_tx / len(delivered)
                                         if delivered else None),
        },
        "energy": {
            "start": energy_start,
            "final": energy,
            "min_final": min(energy.values()) if energy else None,
        },
    }


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def format_report(metrics: dict[str, Any]) -> str:
    """把指标字典排版成可直接贴进报告的文本。"""
    lines: list[str] = []
    add = lines.append

    add("=" * 62)
    add("SkyFleet 仿真评价指标")
    add("=" * 62)
    add(f"事件总数        : {metrics['events']}")
    add(f"仿真时长        : {_fmt(metrics['duration'])} s")

    tasks = metrics["tasks"]
    add("")
    add("-- 任务完成情况 --")
    add(f"任务总数        : {tasks['created']}")
    add(f"已投递          : {tasks['delivered']}")
    add(f"完成率          : {_fmt(tasks['completion_rate'] * 100, 1)} %")

    confirm = metrics["delivery_confirm"]
    add("")
    add("-- 投递确认 --")
    add(f"发送 / 接收     : {confirm['sent']} / {confirm['received']}")
    rate = confirm["success_rate"]
    add(f"确认成功率      : {_fmt(rate * 100 if rate is not None else None, 1)} %")

    add("")
    add("-- 端到端时延（毫秒，按优先级）--")
    add(f"{'等级':<6}{'样本':>7}{'均值':>11}{'P50':>11}"
        f"{'P95':>11}{'P99':>11}{'最大':>11}")
    for qos, summary in metrics["latency_ms"].items():
        add(f"{qos:<6}{summary['count']:>7}{_fmt(summary['mean']):>11}"
            f"{_fmt(summary['p50']):>11}{_fmt(summary['p95']):>11}"
            f"{_fmt(summary['p99']):>11}{_fmt(summary['max']):>11}")

    loss = metrics["loss"]
    add("")
    add("-- 控制消息丢失率 --")
    add(f"控制类 Q0-Q2    : {_fmt(loss['control_rate'] * 100 if loss['control_rate'] is not None else None, 2)} %")
    add(f"数据类 Q3-Q4    : {_fmt(loss['data_rate'] * 100 if loss['data_rate'] is not None else None, 2)} %")

    hb = metrics["heartbeat"]
    add("")
    add("-- 心跳可达性 --")
    add(f"发送 / 接收     : {hb['sent']} / {hb['received']}")
    add(f"送达率          : {_fmt(hb['delivery_rate'] * 100 if hb['delivery_rate'] is not None else None, 2)} %")

    recovery = metrics["recovery"]
    add("")
    add("-- 失效恢复 --")
    add(f"已恢复节点      : {recovery['recovered']}")
    add(f"平均恢复时间    : {_fmt(recovery['mean_s'])} s")
    add(f"最长恢复时间    : {_fmt(recovery['max_s'])} s")
    if recovery["unrecovered"]:
        add(f"未恢复节点      : {', '.join(recovery['unrecovered'])}")

    add("")
    add("-- 重传与路由 --")
    add(f"重传次数        : {metrics['retransmission']['total']}")
    add(f"超时作废        : {metrics['retransmission']['expired']}")
    add(f"单消息最大重传  : {metrics['retransmission']['max_per_message']}")
    add(f"中继转发次数    : {metrics['forwarded']}")
    add(f"路由切换次数    : {metrics['route_switches']}")

    overhead = metrics["overhead"]
    add("")
    add("-- 通信开销 --")
    add(f"总字节          : {overhead['total_bytes']}")
    add(f"其中控制 / 数据 : {overhead['control_bytes']} / {overhead['data_bytes']}")
    add(f"单次投递开销    : {_fmt(overhead['bytes_per_delivered_task'], 1)} 字节")

    add("")
    add("-- 丢弃原因分布 --")
    if not metrics["drop_reasons"]:
        add("（无丢弃）")
    for reason, count in metrics["drop_reasons"].items():
        add(f"{reason:<16}: {count}")

    add("")
    add("-- 电量（末值，初始 {})--".format(
        _fmt(next(iter(metrics['energy']['start'].values()), None), 1)))
    for node_id, value in sorted(metrics["energy"]["final"].items()):
        add(f"{node_id:<8}: {_fmt(value, 2)}")

    return "\n".join(lines)
