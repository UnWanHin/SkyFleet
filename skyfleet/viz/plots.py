"""从事件日志生成报告用图表。

图与指标同源：都从日志重算。每张图同时输出位图（PNG，300 dpi 级别）
与矢量（SVG），便于直接插入报告并按需缩放。
"""
from __future__ import annotations

import os
import re
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any

# matplotlib 需要一个可写缓存目录；默认的 ~/.config 在受限环境下可能不可写
os.environ.setdefault("MPLCONFIGDIR",
                      os.path.join(tempfile.gettempdir(), "skyfleet-mpl"))

import matplotlib                                       # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                          # noqa: E402
from matplotlib import font_manager                      # noqa: E402

from skyfleet.observe.log import read_log                # noqa: E402
from skyfleet.observe.metrics import compute_metrics     # noqa: E402
from skyfleet.observe.replay import build_timeline       # noqa: E402

_CJK_CANDIDATES = ("Noto Sans CJK SC", "Noto Sans CJK TC", "Noto Sans CJK JP",
                   "Source Han Sans SC", "WenQuanYi Zen Hei",
                   "WenQuanYi Micro Hei", "Droid Sans Fallback")

STATUS_ORDER = ("ONLINE", "DEGRADED", "SUSPECT", "FAILED", "LEFT",
                "REJOINING")
STATUS_LEVEL = {name: len(STATUS_ORDER) - 1 - i
                for i, name in enumerate(STATUS_ORDER)}

ROLE_COLOUR = {"LEADER": "#c0392b", "SLAVE": "#2471a3", "RELAY": "#1e8449"}

STATE_COLOUR = {"DELIVERED": "#1e8449", "PICKED": "#2471a3",
                "ASSIGNED": "#b9770e", "CREATED": "#7f8c8d",
                "REASSIGNED": "#8e44ad", "FAILED": "#c0392b",
                "EXPIRED": "#566573"}


def _use_cjk_font() -> bool:
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in _CJK_CANDIDATES:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def _configure_svg_text() -> None:
    """让 SVG 保留**可搜索、可编辑的文字**而不是字形轮廓路径，
    并让导出的 SVG 位元级可重现。

    matplotlib 默认把文字转成路径，好处是任何机器上显示都一致，坏处是
    导出的 SVG 里一个字都搜不到，也无法在 Illustrator/Inkscape 里改轴标签。
    作为报告插图，可编辑比"永不缺字体"更重要。

    `svg.hashsalt` 也必须固定：不固定时 matplotlib 用对象的**内存地址**
    生成元素 id（`id="m086001c3a6"`），于是同一份数据每次画出来的 SVG
    都不同——图能重现但文件不能，diff 里全是噪声。
    """
    plt.rcParams["svg.fonttype"] = "none"
    plt.rcParams["svg.hashsalt"] = "skyfleet"


CJK = _use_cjk_font()
_configure_svg_text()


def L(zh: str, en: str) -> str:
    """有中文字体就用中文标签，否则退回英文，避免出现方块字。"""
    return zh if CJK else en


def _save(fig, out_dir: Path, name: str) -> list[str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    for suffix, kwargs in ((".png", {"dpi": 220}), (".svg", {})):
        path = out_dir / f"{name}{suffix}"
        fig.savefig(path, bbox_inches="tight", **kwargs)
        if suffix == ".svg":
            _normalise_svg_metadata(path)
        written.append(str(path))
    plt.close(fig)
    return written


def _normalise_svg_metadata(path: Path) -> None:
    """把 SVG 元数据里的导出时刻固定下来。

    matplotlib 会把"导出时刻"写进 `<dc:date>`。它是唯一让同一份数据
    两次导出产生不同文件的东西：固定 `svg.hashsalt` 之后，元素 id 已经
    稳定，只剩下这一行时间戳。仿真本身是确定性的，图也不该例外——
    需要时间信息时看日志里的仿真时刻，而不是看文件是几点画出来的。
    """
    text = path.read_text(encoding="utf-8")
    normalised = re.sub(r"<dc:date>[^<]*</dc:date>",
                        "<dc:date>1970-01-01T00:00:00</dc:date>", text)
    if normalised != text:
        path.write_text(normalised, encoding="utf-8")


# ---------------------------------------------------------------- 图表

def _plot_task_completion(timeline: dict[str, Any], out: Path) -> list[str]:
    counts = Counter(task["state"] for task in timeline["tasks"].values())
    states = [s for s in STATE_COLOUR if counts.get(s)]
    values = [counts[s] for s in states]

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    if states:
        ax.bar(states, values,
               color=[STATE_COLOUR[s] for s in states], width=0.6)
        for x, v in zip(states, values):
            ax.text(x, v, str(v), ha="center", va="bottom", fontsize=10)
    else:
        ax.text(0.5, 0.5, L("无任务", "no tasks"), ha="center",
                transform=ax.transAxes)
    ax.set_ylabel(L("任务数", "tasks"))
    ax.set_title(L("任务最终状态分布", "Task final-state distribution"))
    ax.grid(axis="y", alpha=0.3)
    return _save(fig, out, "task_completion")


def _plot_latency_by_qos(metrics: dict[str, Any], out: Path) -> list[str]:
    latency = metrics["latency_ms"]
    qoses = sorted(latency)
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    if qoses:
        width = 0.26
        xs = range(len(qoses))
        for offset, key, colour in ((-width, "p50", "#5dade2"),
                                    (0.0, "p95", "#f5b041"),
                                    (width, "p99", "#cb4335")):
            values = [latency[q][key] or 0.0 for q in qoses]
            ax.bar([x + offset for x in xs], values, width=width,
                   label=key.upper(), color=colour)
        ax.set_xticks(list(xs))
        ax.set_xticklabels(qoses)
        ax.legend()
    else:
        ax.text(0.5, 0.5, L("无接收样本", "no samples"), ha="center",
                transform=ax.transAxes)
    ax.set_ylabel(L("端到端时延 (ms)", "end-to-end delay (ms)"))
    ax.set_xlabel(L("优先级等级", "QoS class"))
    ax.set_title(L("端到端时延分位数", "End-to-end delay percentiles"))
    ax.grid(axis="y", alpha=0.3)
    return _save(fig, out, "latency_by_qos")


def _plot_delay_breakdown(log_path: str | Path, out: Path) -> list[str]:
    segments = ("t_sample_wait", "t_queue", "t_access", "t_tx", "t_prop")
    labels = {
        "t_sample_wait": L("采样等待", "sample wait"),
        "t_queue": L("排队", "queue"),
        "t_access": L("接入等待", "channel access"),
        "t_tx": L("传输", "transmit"),
        "t_prop": L("传播", "propagate"),
    }
    totals: dict[str, dict[str, float]] = {}
    counts: Counter = Counter()
    for event in read_log(log_path):
        if event["kind"] != "MSG_RX":
            continue
        qos = event.get("qos", "Q?")
        counts[qos] += 1
        bucket = totals.setdefault(qos, {s: 0.0 for s in segments})
        for segment in segments:
            bucket[segment] += event.get(segment, 0.0) or 0.0

    qoses = sorted(totals)
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    if qoses:
        means = {q: [totals[q][s] / counts[q] * 1000.0 for s in segments]
                 for q in qoses}
        bottoms = [0.0] * len(qoses)
        colours = ("#aeb6bf", "#5dade2", "#f5b041", "#48c9b0", "#af7ac5")
        for index, segment in enumerate(segments):
            values = [means[q][index] for q in qoses]
            ax.bar(qoses, values, bottom=bottoms, label=labels[segment],
                   color=colours[index])
            bottoms = [b + v for b, v in zip(bottoms, values)]
        for x, total in zip(qoses, bottoms):
            ax.text(x, total, f"{total:.3f}", ha="center", va="bottom",
                    fontsize=9)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, L("无接收样本", "no samples"), ha="center",
                transform=ax.transAxes)
    ax.set_ylabel(L("平均时延 (ms)", "mean delay (ms)"))
    ax.set_xlabel(L("优先级等级", "QoS class"))
    ax.set_title(L("端到端时延六段分解", "Six-segment delay decomposition"))
    ax.grid(axis="y", alpha=0.3)
    return _save(fig, out, "delay_breakdown")


def _plot_drop_reasons(metrics: dict[str, Any], out: Path) -> list[str]:
    reasons = metrics["drop_reasons"]
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    if reasons:
        names = list(reasons)
        values = [reasons[n] for n in names]
        ax.barh(names[::-1], values[::-1], color="#c0392b")
        for y, v in enumerate(values[::-1]):
            ax.text(v, y, f" {v}", va="center", fontsize=9)
    else:
        ax.text(0.5, 0.5, L("无丢弃", "no drops"), ha="center",
                transform=ax.transAxes)
    ax.set_xlabel(L("丢弃次数", "drops"))
    ax.set_title(L("消息丢弃原因分布", "Message drop reasons"))
    ax.grid(axis="x", alpha=0.3)
    return _save(fig, out, "drop_reasons")


def _plot_member_status(timeline: dict[str, Any], out: Path) -> list[str]:
    duration = timeline["duration"] or 1.0
    nodes = sorted(timeline["nodes"])
    fig, ax = plt.subplots(figsize=(7.2, 0.6 * len(nodes) + 2.0))
    for index, node_id in enumerate(nodes):
        history = timeline["nodes"][node_id]["status"] or [[0.0, "ONLINE"]]
        times = [h[0] for h in history] + [duration]
        levels = [STATUS_LEVEL.get(h[1], 0) for h in history]
        levels.append(levels[-1])
        ax.step(times, [index + (lv / 6.0) for lv in levels], where="post",
                linewidth=1.6, color="#2471a3")
    ax.set_yticks(range(len(nodes)))
    ax.set_yticklabels(nodes)
    ax.set_ylim(-0.5, len(nodes) - 0.5)
    ax.set_xlim(0, duration)
    ax.set_xlabel(L("仿真时间 (s)", "simulation time (s)"))
    ax.set_title(L("成员状态时间线（纵向偏移表示状态等级）",
                   "Member status timeline (vertical offset = status level)"))
    ax.grid(alpha=0.3)
    return _save(fig, out, "member_status_timeline")


def _plot_energy(timeline: dict[str, Any], out: Path) -> list[str]:
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    drew = False
    for node_id in sorted(timeline["nodes"]):
        series = timeline["nodes"][node_id]["energy"]
        if not series:
            continue
        drew = True
        role = timeline["nodes"][node_id]["role"]
        ax.plot([p[0] for p in series], [p[1] for p in series],
                label=node_id, linewidth=1.4,
                color=ROLE_COLOUR.get(role, "#555555"))
    if not drew:
        ax.text(0.5, 0.5, L("无采样", "no samples"), ha="center",
                transform=ax.transAxes)
    ax.set_xlabel(L("仿真时间 (s)", "simulation time (s)"))
    ax.set_ylabel(L("剩余电量 (%)", "remaining energy (%)"))
    ax.set_title(L("节点电量变化", "Node energy over time"))
    ax.legend(fontsize=8, ncols=2)
    ax.grid(alpha=0.3)
    return _save(fig, out, "energy")


def _plot_world(timeline: dict[str, Any], out: Path) -> list[str]:
    area = timeline["area"] or 1.0
    fig, ax = plt.subplots(figsize=(6.6, 6.0))

    heights = [b[4] for b in timeline["buildings"]] or [1.0]
    low, high = min(heights), max(heights)
    span = (high - low) or 1.0
    for x, y, w, h, height in timeline["buildings"]:
        shade = 0.85 - 0.55 * (height - low) / span
        ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=str(shade),
                                   edgecolor="#566573", linewidth=0.8))

    for node_id in sorted(timeline["nodes"]):
        node = timeline["nodes"][node_id]
        colour = ROLE_COLOUR.get(node["role"], "#555555")
        if len(node["samples"]) > 1:
            ax.plot([s[1] for s in node["samples"]],
                    [s[2] for s in node["samples"]], linewidth=0.9,
                    color=colour, alpha=0.55)
        if node["samples"]:
            last = node["samples"][-1]
            ax.plot(last[1], last[2], marker="^", markersize=7, color=colour)
            ax.annotate(node_id, (last[1], last[2]), fontsize=8,
                        xytext=(4, 4), textcoords="offset points")

    for task in timeline["tasks"].values():
        ax.plot(task["origin"][0], task["origin"][1], marker="o",
                markersize=4, color="#b9770e")
        ax.plot(task["destination"][0], task["destination"][1], marker="s",
                markersize=4, color="#1e8449")
        ax.annotate("", xy=(task["destination"][0], task["destination"][1]),
                    xytext=(task["origin"][0], task["origin"][1]),
                    arrowprops=dict(arrowstyle="->", color="#b9770e", lw=0.7,
                                    alpha=0.5))

    ax.set_xlim(0, area)
    ax.set_ylim(0, area)
    ax.set_aspect("equal")
    ax.set_xlabel(L("东向 (m)", "east (m)"))
    ax.set_ylabel(L("北向 (m)", "north (m)"))
    ax.set_title(L("楼群、节点轨迹与配送任务",
                   "Buildings, node trajectories and delivery tasks"))
    ax.grid(alpha=0.2)
    return _save(fig, out, "world_map")


def render_all(log_path: str | Path, out_dir: str | Path) -> list[str]:
    """生成全部图表，返回写出的文件路径列表。"""
    path = Path(log_path)
    if not path.exists():
        raise FileNotFoundError(str(log_path))

    timeline = build_timeline(path)
    metrics = compute_metrics(path)
    out = Path(out_dir)

    written: list[str] = []
    written += _plot_task_completion(timeline, out)
    written += _plot_latency_by_qos(metrics, out)
    written += _plot_delay_breakdown(path, out)
    written += _plot_drop_reasons(metrics, out)
    written += _plot_member_status(timeline, out)
    written += _plot_energy(timeline, out)
    written += _plot_world(timeline, out)
    return written
