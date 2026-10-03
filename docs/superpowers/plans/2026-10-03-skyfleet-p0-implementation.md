# SkyFleet P0 通信仿真 —— 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 実装一个纯 Python 标准库的离散事件通信仿真器，跑通报告 P0 的通信闭环，输出 JSONL 事件日志、九项指标、报告用 PNG 图表与单文件 HTML 回放。

**Architecture:** 五层单向依赖（kernel → world → protocol → mission）+ 横切观测层，外加 `sim/` 装配层与 `viz/` 离线可视化。仿真器只写日志，绝不知道可视化的存在。

**Tech Stack:** Python 3.14.7 标准库（`heapq`/`random`/`hashlib`/`json`/`dataclasses`/`enum`/`statistics`/`math`/`collections`/`argparse`）+ 测试用标准库 `unittest`。可视化额外用 `.venv/` 里的 matplotlib 与原生 JS。

**Spec:** `docs/superpowers/specs/2026-10-03-uav-swarm-comms-sim-design.md`

## Global Constraints

- 仿真核心（`skyfleet/`）**零第三方依赖**，只用标准库。
- 测试用**标准库 `unittest`**，运行方式 `python3 -m unittest discover -s tests -t .`，不引入 pytest。
- **确定性**：相同 `master_seed` 两次运行，日志规范化后 SHA-256 必须一致。
- 随机种子的派生必须用 `hashlib.blake2b`，**禁止**使用内建 `hash()`（受 `PYTHONHASHSEED` 影响）。
- 所有门限参数必须来自配置，不得硬编码。
- 事件时间戳必须带单调 `tie` 序号，保证同刻事件顺序确定。
- `skyfleet/` 下不得 import `viz/`；`viz/` 只能读日志文件。
- 基线场景：1 主 + 8 从 + 2 中继 = 11 节点，500 m × 500 m，20 任务，600 s。
- 丢弃原因必须落在 `DropReason` 闭环枚举内，日志不得出现自由文本原因。
- 每完成一个 Task 提交一次 git commit。

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `skyfleet/kernel/rng.py` | 稳定种子派生、独立随机流注册表 |
| `skyfleet/kernel/event.py` | 事件对象与排序键 |
| `skyfleet/kernel/scheduler.py` | 事件循环、时钟、订阅分发 |
| `skyfleet/kernel/config.py` | 配置数据类、加载与覆盖 |
| `skyfleet/world/geometry.py` | 点/线段/矩形相交、距离（纯函数） |
| `skyfleet/world/building.py` | 楼块与楼群生成 |
| `skyfleet/world/node.py` | 节点静态与运动状态、能量 |
| `skyfleet/world/mobility.py` | 航点移动 |
| `skyfleet/world/link.py` | `LinkModel` 接口 + 几何遮挡实现 |
| `skyfleet/world/channel.py` | 共享信道：占用、等待、冲突 |
| `skyfleet/protocol/message.py` | `Msg`、`Qos`、`Rel`、消息类型表 |
| `skyfleet/protocol/version.py` | 版本/任期/序列接受规则、去重表（纯函数） |
| `skyfleet/protocol/identity.py` | node_id/session_id/role/lease |
| `skyfleet/protocol/membership.py` | 带迟滞的成员状态机 |
| `skyfleet/protocol/queue.py` | Q0–Q4 优先队列、token bucket 限流、R0 最新值覆盖 |
| `skyfleet/protocol/reliability.py` | R0–R3 策略、ACK、重传、截止期 |
| `skyfleet/protocol/routing.py` | 路由迟滞与切换计数 |
| `skyfleet/mission/task.py` | 任务状态机 |
| `skyfleet/mission/delivery.py` | 投递确认与幂等 |
| `skyfleet/mission/safety.py` | 本地安全动作 |
| `skyfleet/observe/events.py` | `EventKind`、`DropReason`、日志事件结构 |
| `skyfleet/observe/log.py` | JSONL 写入/读取/规范化摘要 |
| `skyfleet/observe/metrics.py` | 九项指标从日志重算 |
| `skyfleet/observe/replay.py` | 从日志重建时间线 |
| `skyfleet/sim/agent.py` | 单节点协议代理 |
| `skyfleet/sim/leader.py` | 主节点任务编排 |
| `skyfleet/sim/engine.py` | 装配世界/信道/代理并跑事件循环 |
| `skyfleet/cli.py` | `run` / `metrics` / `plot` / `replay` / `all` |
| `skyfleet/scenarios/baseline_small.json` | 基线场景与全部门限参数 |
| `skyfleet/viz/plots.py` | matplotlib 图表 |
| `skyfleet/viz/replay_html.py` | 单文件 HTML 回放 |
| `tests/unit/**` | 单元测试 |
| `tests/integration/**` | 完整事件链集成测试 |
| `tests/determinism/**` | 确定性测试 |

---

## Task 1: 项目骨架与独立随机源

**Files:**
- Create: `skyfleet/__init__.py`, `skyfleet/kernel/__init__.py`, `skyfleet/world/__init__.py`, `skyfleet/protocol/__init__.py`, `skyfleet/mission/__init__.py`, `skyfleet/observe/__init__.py`, `skyfleet/sim/__init__.py`, `skyfleet/viz/__init__.py`
- Create: `skyfleet/kernel/rng.py`
- Test: `tests/unit/test_rng.py`
- Create: `.gitignore`, `tests/__init__.py`, `tests/unit/__init__.py`

**Interfaces:**
- Produces: `derive_seed(master_seed: int, stream_name: str) -> int`；`RngRegistry(master_seed: int)` 带 `.get(name: str) -> random.Random`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_rng.py
import unittest
from skyfleet.kernel.rng import derive_seed, RngRegistry

class TestDeriveSeed(unittest.TestCase):
    def test_same_inputs_give_same_seed(self):
        self.assertEqual(derive_seed(42, "channel:collision"),
                         derive_seed(42, "channel:collision"))

    def test_different_names_give_different_seeds(self):
        self.assertNotEqual(derive_seed(42, "a"), derive_seed(42, "b"))

    def test_different_master_gives_different_seed(self):
        self.assertNotEqual(derive_seed(1, "a"), derive_seed(2, "a"))

    def test_seed_is_stable_across_processes(self):
        # 钉死具体数值：若改用内建 hash()，此断言会因 PYTHONHASHSEED 而失败
        self.assertEqual(derive_seed(0, "channel:collision"), 12036629166246477061)

class TestRngRegistry(unittest.TestCase):
    def test_same_name_returns_same_instance(self):
        r = RngRegistry(7)
        self.assertIs(r.get("x"), r.get("x"))

    def test_independent_streams_do_not_interleave(self):
        a = RngRegistry(7).get("x")
        b = RngRegistry(7)
        b.get("y").random()          # 消耗另一条流
        self.assertEqual(a.random(), RngRegistry(7).get("x").random())

    def test_reproducible_sequence(self):
        self.assertEqual([RngRegistry(3).get("s").random() for _ in range(1)],
                         [RngRegistry(3).get("s").random() for _ in range(1)])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_rng -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'skyfleet.kernel.rng'`

- [ ] **Step 3: Write minimal implementation**

```python
# skyfleet/kernel/rng.py
"""独立随机流。

关键：绝不用内建 hash() 派生种子——字符串 hash() 受 PYTHONHASHSEED 影响，
逐进程变化，会直接破坏仿真的可复现性。"""
from __future__ import annotations

import hashlib
import random


def derive_seed(master_seed: int, stream_name: str) -> int:
    digest = hashlib.blake2b(f"{master_seed}:{stream_name}".encode("utf-8"),
                             digest_size=8).digest()
    return int.from_bytes(digest, "big")


class RngRegistry:
    """每个随机来源一条独立流。

    若共用一条流，增删节点会使后续所有抽样整体位移，导致 A/B 对比失效。
    """

    def __init__(self, master_seed: int) -> None:
        self.master_seed = master_seed
        self._streams: dict[str, random.Random] = {}

    def get(self, name: str) -> random.Random:
        if name not in self._streams:
            self._streams[name] = random.Random(derive_seed(self.master_seed, name))
        return self._streams[name]
```

**注意**：`test_seed_is_stable_across_processes` 的具体数值需在首次运行时用
`derive_seed(0, "channel:collision")` 打印后回填，并在**两个不同 PYTHONHASHSEED** 下验证一致。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_rng -v`
Expected: PASS（7 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(kernel): 项目骨架与独立随机流"
```

---

## Task 2: 事件与调度器

**Files:**
- Create: `skyfleet/kernel/event.py`, `skyfleet/kernel/scheduler.py`
- Test: `tests/unit/test_scheduler.py`

**Interfaces:**
- Consumes: 无
- Produces: `Event(t: float, kind: str, payload: Any, tie: int)`；`Scheduler()` 带 `.now: float`、`.schedule_at(t, kind, payload) -> Event`、`.schedule(delay, kind, payload) -> Event`、`.cancel(event)`、`.subscribe(kind, handler)`、`.run(until: float | None)`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_scheduler.py
import unittest
from skyfleet.kernel.scheduler import Scheduler

class TestScheduler(unittest.TestCase):
    def test_runs_events_in_time_order(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.payload))
        s.schedule_at(2.0, "e", "b")
        s.schedule_at(1.0, "e", "a")
        s.schedule_at(3.0, "e", "c")
        s.run()
        self.assertEqual(seen, ["a", "b", "c"])

    def test_same_timestamp_uses_insertion_order(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.payload))
        for tag in ("first", "second", "third"):
            s.schedule_at(1.0, "e", tag)
        s.run()
        self.assertEqual(seen, ["first", "second", "third"])

    def test_clock_advances_to_current_event(self):
        s, stamps = Scheduler(), []
        s.subscribe("e", lambda ev: stamps.append(s.now))
        s.schedule_at(5.0, "e")
        s.run()
        self.assertEqual(stamps, [5.0])

    def test_schedule_is_relative_to_current_time(self):
        # 注意：schedule_at 只是入堆，不推进时钟
        s, seen = Scheduler(), []
        s.subscribe("a", lambda ev: s.schedule(2.5, "b"))
        s.subscribe("b", lambda ev: seen.append(ev.t))
        s.schedule_at(10.0, "a")
        s.run()
        self.assertEqual(seen, [12.5])

    def test_run_until_stops_early(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.t))
        s.schedule_at(1.0, "e"); s.schedule_at(9.0, "e")
        s.run(until=5.0)
        self.assertEqual(seen, [1.0])

    def test_cancel_prevents_handler(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(1))
        ev = s.schedule_at(1.0, "e")
        self.assertTrue(s.cancel(ev))
        s.run()
        self.assertEqual(seen, [])

    def test_two_runs_of_same_schedule_are_identical(self):
        def build():
            s, seen = Scheduler(), []
            s.subscribe("e", lambda ev: seen.append(ev.payload))
            for i in (3, 1, 2, 1):
                s.schedule_at(float(i), "e", i)
            s.run()
            return seen
        self.assertEqual(build(), build())
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_scheduler -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'skyfleet.kernel.scheduler'`

- [ ] **Step 3: Write minimal implementation**

```python
# skyfleet/kernel/event.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Event:
    t: float
    kind: str
    payload: Any = None
    tie: int = 0          # 单调序号，保证同刻事件顺序确定
    cancelled: bool = False

    def sort_key(self) -> tuple[float, int]:
        return (self.t, self.tie)
```

```python
# skyfleet/kernel/scheduler.py
from __future__ import annotations

import heapq
from typing import Any, Callable

from skyfleet.kernel.event import Event

Handler = Callable[[Event], None]


class Scheduler:
    """离散事件调度器：单线程、确定性。

    不使用多线程、不使用真实睡眠。
    """

    def __init__(self) -> None:
        self._heap: list[tuple[float, int, Event]] = []
        self._tie = 0
        self._now = 0.0
        self._handlers: dict[str, list[Handler]] = {}

    @property
    def now(self) -> float:
        return self._now

    def subscribe(self, kind: str, handler: Handler) -> None:
        self._handlers.setdefault(kind, []).append(handler)

    def schedule_at(self, t: float, kind: str, payload: Any = None) -> Event:
        self._tie += 1
        ev = Event(t=t, kind=kind, payload=payload, tie=self._tie)
        heapq.heappush(self._heap, (ev.t, ev.tie, ev))
        return ev

    def schedule(self, delay: float, kind: str, payload: Any = None) -> Event:
        return self.schedule_at(self._now + max(0.0, delay), kind, payload)

    def cancel(self, event: Event) -> bool:
        if event.cancelled:
            return False
        event.cancelled = True
        return True

    def run(self, until: float | None = None) -> None:
        while self._heap:
            t, _, ev = self._heap[0]
            if until is not None and t > until:
                break
            heapq.heappop(self._heap)
            if ev.cancelled:
                continue
            self._now = t
            for handler in self._handlers.get(ev.kind, ()):
                handler(ev)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_scheduler -v`
Expected: PASS（7 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(kernel): 离散事件调度器与确定性事件排序"
```

---

## Task 3: 几何与遮挡判定基础

**Files:**
- Create: `skyfleet/world/geometry.py`
- Test: `tests/unit/test_geometry.py`

**Interfaces:**
- Produces: `segment_intersects_rect(p1, p2, rect: tuple[float,float,float,float]) -> bool`（rect 为 `(x, y, w, h)`）；`distance_2d(a, b)`；`distance_3d(a, b)`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_geometry.py
import unittest
from skyfleet.world.geometry import segment_intersects_rect, distance_2d, distance_3d

class TestSegmentRect(unittest.TestCase):
    def test_passes_straight_through(self):
        self.assertTrue(segment_intersects_rect((0, 5), (10, 5), (4, 0, 2, 10)))

    def test_clearly_avoids(self):
        self.assertFalse(segment_intersects_rect((0, 20), (10, 20), (4, 0, 2, 10)))

    def test_crosses_corner_region(self):
        self.assertTrue(segment_intersects_rect((0, 0), (10, 10), (4, 4, 2, 2)))

    def test_fully_inside(self):
        self.assertTrue(segment_intersects_rect((4.5, 4.5), (5.5, 5.5), (4, 4, 2, 2)))

    def test_grazes_edge_is_intersection(self):
        self.assertTrue(segment_intersects_rect((0, 10), (10, 10), (4, 0, 2, 10)))

    def test_just_misses(self):
        self.assertFalse(segment_intersects_rect((0, 10.001), (10, 10.001), (4, 0, 2, 10)))

    def test_zero_length_segment_inside(self):
        self.assertTrue(segment_intersects_rect((5, 5), (5, 5), (4, 4, 2, 2)))

class TestDistance(unittest.TestCase):
    def test_2d(self):
        self.assertAlmostEqual(distance_2d((0, 0), (3, 4)), 5.0)

    def test_3d(self):
        self.assertAlmostEqual(distance_3d((0, 0, 0), (1, 2, 2)), 3.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_geometry -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'skyfleet.world.geometry'`

- [ ] **Step 3: Write minimal implementation**

```python
# skyfleet/world/geometry.py
"""几何纯函数。遮挡判定的正确性完全依赖本模块，是单测重点。"""
from __future__ import annotations

import math

Point = tuple[float, float]
Rect = tuple[float, float, float, float]   # (x, y, w, h)


def _inside(p: Point, r: Rect) -> bool:
    x, y, w, h = r
    return x <= p[0] <= x + w and y <= p[1] <= y + h


def _orient(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    d1 = _orient(p3, p4, p1)
    d2 = _orient(p3, p4, p2)
    d3 = _orient(p1, p2, p3)
    d4 = _orient(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))


def segment_intersects_rect(p1: Point, p2: Point, rect: Rect) -> bool:
    """线段与轴对齐矩形是否相交（含边界接触、含完全包含）。"""
    if _inside(p1, rect) or _inside(p2, rect):
        return True
    x, y, w, h = rect
    corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    for i in range(4):
        if _segments_intersect(p1, p2, corners[i], corners[(i + 1) % 4]):
            return True
    return False


def distance_2d(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance_3d(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_geometry -v`
Expected: PASS（9 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(world): 几何相交与距离纯函数"
```

---

## Task 4: 消息格式与封闭枚举

**Files:**
- Create: `skyfleet/observe/events.py`, `skyfleet/protocol/message.py`
- Test: `tests/unit/test_message.py`

**Interfaces:**
- Produces:
  - `DropReason`（12 个成员，见 spec §4.3）、`EventKind`（spec §4.3 列表）
  - `Qos` = `Q0..Q4`，`Rel` = `R0..R3`（`enum.IntEnum`，可比较大小）
  - `Msg` frozen dataclass，字段见 spec §6.4
  - `MSG_TYPES: dict[str, tuple[Qos, Rel]]`，含 `HEARTBEAT`/`TELEMETRY`/`MEMBER_UPDATE`/`ROUTE_UPDATE`/`TASK_ASSIGN`/`DELIVERY_CONFIRM`/`EMERGENCY`/`JOIN_REQUEST`/`ACK`
  - `Msg.new(msg_type, src, dst, t, **kw) -> Msg` 工厂，自动填充 qos/rel/msg_id

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_message.py
import unittest
from skyfleet.protocol.message import Msg, Qos, Rel, MSG_TYPES
from skyfleet.observe.events import DropReason

class TestQosRelOrdering(unittest.TestCase):
    def test_q0_is_most_urgent(self):
        self.assertLess(Qos.Q0, Qos.Q3)
        self.assertLess(Qos.Q1, Qos.Q4)

    def test_drop_reason_is_closed_set(self):
        self.assertEqual(len(DropReason), 12)
        self.assertEqual(DropReason.OCCLUDED.value, "OCCLUDED")

class TestMsgFactory(unittest.TestCase):
    def test_new_fills_qos_and_rel_from_type(self):
        m = Msg.new("TELEMETRY", src="S1", dst="LEADER", t=0.0)
        self.assertIs(m.qos, Qos.Q3)
        self.assertIs(m.reliability, Rel.R0)

    def test_emergency_is_q0_r3(self):
        m = Msg.new("EMERGENCY", src="S1", dst="LEADER", t=1.0)
        self.assertIs(m.qos, Qos.Q0)
        self.assertIs(m.reliability, Rel.R3)

    def test_ack_is_r2(self):
        self.assertIs(MSG_TYPES["ACK"][1], Rel.R2)

    def test_msg_id_is_unique(self):
        a = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        b = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertNotEqual(a.msg_id, b.msg_id)

    def test_is_frozen(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        with self.assertRaises(Exception):
            m.seq = 99

    def test_defaults(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertEqual(m.hop, 0)
        self.assertEqual(m.leader_epoch, 0)
        self.assertIsNone(m.operation_id)
        self.assertEqual(m.payload, {})

    def test_deadline_defaults_to_never(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertEqual(m.deadline, float("inf"))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_message -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

`skyfleet/observe/events.py`：定义 `DropReason`（`OCCLUDED, COLLISION, QUEUE_FULL, TTL_EXPIRED,
DEADLINE_MISSED, DEDUP, STALE_EPOCH, STALE_SESSION, STALE_VERSION, NO_ROUTE, RATE_LIMITED,
UNAUTHORIZED`，共 12 个）与 `EventKind`（`NODE_JOIN, NODE_LEAVE, NODE_REJOIN, ROLE_CHANGE,
STATUS_CHANGE, HEARTBEAT_TX, HEARTBEAT_MISS, MSG_TX, MSG_RX, MSG_FORWARD, MSG_DROP, MSG_DUP,
MSG_EXPIRE, CHANNEL_BUSY, OCCLUSION_ENTER, OCCLUSION_EXIT, TASK_CREATE, TASK_ASSIGN, TASK_PICKUP,
TASK_DELIVER, TASK_FAIL, TASK_REASSIGN, SAFETY_ACTION, QUEUE_SAMPLE, ENERGY_SAMPLE`），
均继承 `str, enum.Enum`，便于 JSON 序列化。

`skyfleet/protocol/message.py`：按 spec §6.4 定义 frozen `Msg`，用 `itertools.count` 生成
`msg_id`（`f"m{next(counter)}"`），`Msg.new` 从 `MSG_TYPES` 取 qos/rel 并按消息类型填入默认
deadline 偏移（R3 → `t + 0.05`，R2 → `t + 1.0`，R1 → `t + 2.0`，R0 → `inf`）。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_message -v`
Expected: PASS（9 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): 消息格式与封闭枚举"
```

---

## Task 5: 版本、任期与去重（纯函数）

**Files:**
- Create: `skyfleet/protocol/version.py`
- Test: `tests/unit/test_version.py`

**Interfaces:**
- Consumes: `Msg`（Task 4）、`DropReason`（Task 4）
- Produces: `AcceptDecision` 枚举（`ACCEPT, STALE_EPOCH, STALE_SESSION, STALE_VERSION, STALE_SEQ, DEDUP`）；`DedupTable(ttl: float, capacity: int)` 带 `.seen(op_id, t) -> bool`、`.prune(t)`；`LocalView` 带 `leader_epoch, mission_version, route_version, sessions: set[str], last_seq: dict[tuple[str,int,int], int]`；`accept(msg, view, dedup, t) -> AcceptDecision`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_version.py
import unittest
from skyfleet.protocol.message import Msg
from skyfleet.protocol.version import AcceptDecision, DedupTable, LocalView, accept

def msg(**kw):
    base = dict(msg_type="TASK_ASSIGN", src="L", dst="S1", t=0.0)
    base.update(kw)
    return Msg.new(**base)

class TestAcceptRules(unittest.TestCase):
    def setUp(self):
        self.view = LocalView(leader_epoch=2, mission_version=5,
                              route_version=1, sessions={"L#1"})
        self.dedup = DedupTable(ttl=10.0, capacity=100)

    def test_accepts_fresh_message(self):
        m = msg(leader_epoch=2, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_old_epoch(self):
        m = msg(leader_epoch=1, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_accepts_newer_epoch(self):
        m = msg(leader_epoch=3, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_unknown_session(self):
        m = msg(leader_epoch=2, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SESSION)

    def test_rejects_old_mission_version(self):
        m = msg(leader_epoch=2, mission_version=4, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_VERSION)

    def test_epoch_beats_session_check(self):
        # 旧任期的消息即使 session 也不认识，仍应报 STALE_EPOCH
        m = msg(leader_epoch=1, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_rejects_duplicate_operation(self):
        m = msg(leader_epoch=2, mission_version=5, seq=1, session_id="L#1",
                operation_id="op-7")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)
        self.assertIs(accept(m, self.view, self.dedup, 0.1), AcceptDecision.DEDUP)

    def test_rejects_stale_seq(self):
        self.view.last_seq[("L", 2, 5)] = 9
        m = msg(leader_epoch=2, mission_version=5, seq=9, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SEQ)

    def test_accepts_higher_seq(self):
        self.view.last_seq[("L", 2, 5)] = 9
        m = msg(leader_epoch=2, mission_version=5, seq=10, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_accept_updates_view(self):
        m = msg(leader_epoch=4, mission_version=7, seq=3, session_id="L#1")
        accept(m, self.view, self.dedup, 0.0)
        self.assertEqual(self.view.leader_epoch, 4)
        self.assertEqual(self.view.mission_version, 7)
        self.assertEqual(self.view.last_seq[("L", 4, 7)], 3)

class TestDedupTable(unittest.TestCase):
    def test_expires_after_ttl(self):
        d = DedupTable(ttl=5.0, capacity=10)
        self.assertFalse(d.seen("op", 0.0))
        self.assertTrue(d.seen("op", 4.9))
        self.assertFalse(d.seen("op", 5.1))

    def test_capacity_is_bounded(self):
        d = DedupTable(ttl=1000.0, capacity=3)
        for i in range(50):
            d.seen(f"op{i}", float(i))
        self.assertLessEqual(len(d), 3)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_version -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

按 spec §6.3 的六条规则实现。规则顺序必须是：epoch → session → mission_version →
dedup → seq，因为 `test_epoch_beats_session_check` 依赖该优先级。
`DedupTable` 内部用 `dict[str, float]` 存 `op_id -> 过期时刻`，超出 `capacity` 时按过期时刻
淘汰最旧的；`seen()` 命中未过期则返回 `True`。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_version -v`
Expected: PASS（12 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): 版本任期与去重接受规则"
```

---

## Task 6: 身份、会话与租约

**Files:**
- Create: `skyfleet/protocol/identity.py`
- Test: `tests/unit/test_identity.py`

**Interfaces:**
- Produces: `Role` 枚举（`LEADER, SLAVE, RELAY, STANDBY`）；`Lease(node_id, role, epoch, scope, expire_at)` 带 `.valid(t) -> bool`；`IdentityRegistry()` 带 `.join(node_id, t) -> str`（返回新 `session_id`）、`.current_session(node_id) -> str | None`、`.is_valid_session(node_id, session_id) -> bool`、`.grant_lease(node_id, role, epoch, scope, t, ttl) -> Lease`、`.active_leases(t) -> list[Lease]`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_identity.py
import unittest
from skyfleet.protocol.identity import IdentityRegistry, Lease, Role

class TestJoin(unittest.TestCase):
    def test_first_join_creates_session(self):
        r = IdentityRegistry()
        self.assertEqual(r.join("S1", 0.0), "S1#1")

    def test_rejoin_creates_new_session(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        self.assertEqual(r.join("S1", 5.0), "S1#2")

    def test_old_session_becomes_invalid(self):
        r = IdentityRegistry()
        old = r.join("S1", 0.0)
        r.join("S1", 5.0)
        self.assertFalse(r.is_valid_session("S1", old))

    def test_current_session_is_latest(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        new = r.join("S1", 5.0)
        self.assertEqual(r.current_session("S1"), new)

    def test_unknown_node_has_no_session(self):
        self.assertIsNone(IdentityRegistry().current_session("X"))

    def test_sessions_are_independent_between_nodes(self):
        r = IdentityRegistry()
        r.join("S1", 0.0); r.join("S2", 0.0)
        self.assertEqual(r.current_session("S1"), "S1#1")
        self.assertEqual(r.current_session("S2"), "S2#1")

class TestLease(unittest.TestCase):
    def test_valid_before_expiry(self):
        self.assertTrue(Lease("L", Role.LEADER, 0, "all", 10.0).valid(9.9))

    def test_invalid_at_and_after_expiry(self):
        self.assertFalse(Lease("L", Role.LEADER, 0, "all", 10.0).valid(10.0))

    def test_grant_and_query_active(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        self.assertEqual(len(r.active_leases(4.0)), 1)

    def test_expired_lease_not_active(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        self.assertEqual(r.active_leases(5.0), [])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_identity -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

`IdentityRegistry` 维护 `node_id -> (join_counter, session_id)` 与 `node_id -> Lease`。
`Lease.valid(t)` 为 `t < expire_at`（到期即失效，不需显式撤销——对应 spec §6.1）。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_identity -v`
Expected: PASS（10 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): 身份会话与租约"
```

---

## Task 7: 成员状态机（带迟滞）

**Files:**
- Create: `skyfleet/protocol/membership.py`
- Test: `tests/unit/test_membership.py`

**Interfaces:**
- Consumes: `DropReason`（Task 4）
- Produces: `MemberStatus` 枚举（`ONLINE, DEGRADED, SUSPECT, FAILED, LEFT, REJOINING`）；`MembershipParams(n_degraded, n_suspect, t_fail, q_degraded)`；`MemberRecord`；`MembershipView(params)` 带 `.on_heartbeat(node_id, session_id, t, link_quality, heartbeat_seq)`、`.on_miss(node_id, t) -> MemberStatus`、`.on_indirect(node_id, t)`、`.on_leave(node_id, t)`、`.on_join(node_id, session_id, t)`、`.tick(t) -> list[tuple[str, MemberStatus]]`、`.status(node_id) -> MemberStatus`、`.members(statuses=None) -> list[str]`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_membership.py
import unittest
from skyfleet.protocol.membership import (MemberStatus, MembershipParams,
                                          MembershipView)

P = MembershipParams(n_degraded=2, n_suspect=4, t_fail=3.0, q_degraded=0.4)

class TestHysteresis(unittest.TestCase):
    def setUp(self):
        self.v = MembershipView(P)
        self.v.on_join("S1", "S1#1", 0.0)

    def test_join_starts_online(self):
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_single_miss_does_not_change_status(self):
        self.v.on_miss("S1", 1.0)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_two_misses_go_degraded_not_failed(self):
        self.v.on_miss("S1", 1.0); self.v.on_miss("S1", 2.0)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_degraded_does_not_fail_a_member(self):
        for i in range(1, 4):
            self.v.on_miss("S1", float(i))
        self.assertNotEqual(self.v.status("S1"), MemberStatus.FAILED)

    def test_low_quality_triggers_degraded_without_misses(self):
        self.v.on_heartbeat("S1", "S1#1", 0.1, 0.1, 1)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_sustained_misses_go_suspect(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.assertIs(self.v.status("S1"), MemberStatus.SUSPECT)

    def test_indirect_evidence_prevents_suspect(self):
        for i in range(1, 4):
            self.v.on_miss("S1", float(i))
        self.v.on_indirect("S1", 3.5)
        self.v.on_miss("S1", 4.0)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_suspect_fails_after_t_fail(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.tick(6.9)          # 距上次证据不足 t_fail
        self.assertIs(self.v.status("S1"), MemberStatus.SUSPECT)
        self.v.tick(7.1)
        self.assertIs(self.v.status("S1"), MemberStatus.FAILED)

    def test_heartbeat_recovers_to_online(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.on_heartbeat("S1", "S1#1", 5.0, 0.9, 2)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_leave_sets_left(self):
        self.v.on_leave("S1", 1.0)
        self.assertIs(self.v.status("S1"), MemberStatus.LEFT)

    def test_rejoin_from_failed_goes_online(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.tick(9.0)
        self.assertIs(self.v.status("S1"), MemberStatus.FAILED)
        self.v.on_join("S1", "S1#2", 10.0)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_members_filter_by_status(self):
        self.v.on_join("S2", "S2#1", 0.0)
        self.v.on_leave("S2", 1.0)
        self.assertEqual(self.v.members(statuses={MemberStatus.ONLINE}), ["S1"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_membership -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

严格实现 spec §6.2 的转移表。要点：
- `on_miss` 递增 `miss_count`；`>= n_degraded` 且当前 `ONLINE` → `DEGRADED`；
  `>= n_suspect` **且** `last_evidence_at` 距今无间接证据 → `SUSPECT`。
- 间接证据通过 `on_indirect` 刷新 `last_evidence_at`，从而抑制 `SUSPECT`。
- `tick(t)` 把处于 `SUSPECT` 且 `t - last_evidence_at >= t_fail` 的记录转 `FAILED`，
  返回本次发生变化的 `(node_id, status)` 列表。
- 任何有效心跳把状态拉回 `ONLINE` 并清零 `miss_count`（低质量心跳只到 `DEGRADED`）。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_membership -v`
Expected: PASS（13 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): 带迟滞的成员状态机"
```

---

## Task 8: 优先队列与限流

**Files:**
- Create: `skyfleet/protocol/queue.py`
- Test: `tests/unit/test_queue.py`

**Interfaces:**
- Consumes: `Msg`, `Qos`, `Rel`（Task 4）
- Produces: `TokenBucket(rate_per_s: float, burst: float)` 带 `.allow(t) -> bool`；`SendQueue(depth_limit: int, rng)` 带 `.enqueue(msg, t) -> DropReason | None`、`.dequeue(t) -> Msg | None`、`.size(qos=None) -> int`、`.sample() -> dict`；`RateLimiter(q0_rate, q0_burst, q1_rate, q1_burst)` 带 `.allow(msg, t) -> bool`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_queue.py
import unittest
from skyfleet.protocol.message import Msg, Qos
from skyfleet.protocol.queue import RateLimiter, SendQueue, TokenBucket

def m(qos, src="S1", t=0.0, seq=1):
    msg = Msg.new("TELEMETRY", src=src, dst="L", t=t, seq=seq)
    return msg

class TestTokenBucket(unittest.TestCase):
    def test_allows_up_to_burst(self):
        b = TokenBucket(rate_per_s=10.0, burst=3.0)
        self.assertEqual([b.allow(0.0) for _ in range(4)], [True, True, True, False])

    def test_refills_over_time(self):
        b = TokenBucket(rate_per_s=10.0, burst=3.0)
        for _ in range(3):
            b.allow(0.0)
        self.assertFalse(b.allow(0.0))
        self.assertTrue(b.allow(0.2))     # 补充 2 个

class TestSendQueue(unittest.TestCase):
    def test_strict_priority_dequeue(self):
        q = SendQueue(depth_limit=100, rng=None)
        q.enqueue(m(Qos.Q3, seq=1), 0.0)
        q.enqueue(m(Qos.Q0, seq=2), 0.0)
        q.enqueue(m(Qos.Q1, seq=3), 0.0)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q0)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q1)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q3)

    def test_fifo_within_same_qos(self):
        q = SendQueue(depth_limit=100, rng=None)
        q.enqueue(m(Qos.Q2, seq=1), 0.0)
        q.enqueue(m(Qos.Q2, seq=2), 0.0)
        self.assertEqual(q.dequeue(0.0).seq, 1)
        self.assertEqual(q.dequeue(0.0).seq, 2)

    def test_queue_full_returns_reason(self):
        q = SendQueue(depth_limit=1, rng=None)
        self.assertIsNone(q.enqueue(m(Qos.Q3, seq=1), 0.0))
        from skyfleet.observe.events import DropReason
        self.assertIs(q.enqueue(m(Qos.Q3, seq=2), 0.0), DropReason.QUEUE_FULL)

    def test_r0_latest_value_overwrites_same_source(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q3, seq=1), 0.0)
        q.enqueue(m(Qos.Q3, seq=2), 0.0)
        self.assertEqual(q.size(), 1)
        self.assertEqual(q.dequeue(0.0).seq, 2)

    def test_empty_dequeue_is_none(self):
        self.assertIsNone(SendQueue(depth_limit=1, rng=None).dequeue(0.0))

    def test_size_by_qos(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q0, seq=1), 0.0)
        q.enqueue(m(Qos.Q3, seq=2), 0.0)
        self.assertEqual(q.size(Qos.Q0), 1)
        self.assertEqual(q.size(), 2)

class TestRateLimiter(unittest.TestCase):
    def test_telemetry_is_never_rate_limited(self):
        rl = RateLimiter(q0_rate=1.0, q0_burst=1.0, q1_rate=1.0, q1_burst=1.0)
        self.assertTrue(all(rl.allow(m(Qos.Q3, seq=i), 0.0) for i in range(1, 20)))

    def test_emergency_burst_then_blocked(self):
        rl = RateLimiter(q0_rate=1.0, q0_burst=2.0, q1_rate=1.0, q1_burst=2.0)
        self.assertTrue(rl.allow(m(Qos.Q0, seq=1), 0.0))
        self.assertTrue(rl.allow(m(Qos.Q0, seq=2), 0.0))
        self.assertFalse(rl.allow(m(Qos.Q0, seq=3), 0.0))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_queue -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

`SendQueue` 每条 QoS 一条 `collections.deque`。R0 入队时按 `(msg_type, src)` 覆盖同键旧消息
（spec §6.5「最新值覆盖旧值」）。`depth_limit` 为所有队列总长上限。
`RateLimiter` 对 Q0/Q1 各持一个 `TokenBucket`，其他等级一律放行。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_queue -v`
Expected: PASS（11 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): 优先队列与高优先级限流"
```

---

## Task 9: 可靠性策略与截止期

**Files:**
- Create: `skyfleet/protocol/reliability.py`
- Test: `tests/unit/test_reliability.py`

**Interfaces:**
- Consumes: `Msg`, `Rel`（Task 4）
- Produces: `RetransmitPolicy`（`Rel -> int` 最大重传次数：R0→0, R1→2, R2→3, R3→2）；
  `ReliabilityManager()` 带 `.register(msg, t)`、`.ack(msg_id) -> bool`、`.due_retries(t) -> list[Msg]`、
  `.on_retry_sent(msg_id, t)`、`.exhausted(msg_id) -> bool`、`.pending_count() -> int`、
  `.expired(t) -> list[Msg]`（超过 `deadline` 且尚未 ACK）

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_reliability.py
import unittest
from skyfleet.protocol.message import Msg, Rel
from skyfleet.protocol.reliability import ReliabilityManager, max_retries

def mk(rel, t=0.0, deadline=None):
    m = Msg.new("DELIVERY_CONFIRM", src="S1", dst="L", t=t)
    if deadline is not None:
        m = Msg(**{**m.__dict__, "deadline": deadline, "reliability": rel})
    else:
        m = Msg(**{**m.__dict__, "reliability": rel})
    return m

class TestPolicy(unittest.TestCase):
    def test_max_retries_by_level(self):
        self.assertEqual(max_retries(Rel.R0), 0)
        self.assertEqual(max_retries(Rel.R1), 2)
        self.assertEqual(max_retries(Rel.R2), 3)
        self.assertEqual(max_retries(Rel.R3), 2)

class TestManager(unittest.TestCase):
    def test_ack_marks_done(self):
        rm = ReliabilityManager()
        m = mk(Rel.R2, deadline=10.0)
        rm.register(m, 0.0)
        self.assertTrue(rm.ack(m.msg_id))

    def test_ack_unknown_message_is_false(self):
        self.assertFalse(ReliabilityManager().ack("nope"))

    def test_retry_due_after_timeout(self):
        rm = ReliabilityManager(retry_interval=1.0)
        m = mk(Rel.R2, deadline=10.0)
        rm.register(m, 0.0)
        self.assertEqual(rm.due_retries(0.5), [])
        self.assertEqual([x.msg_id for x in rm.due_retries(1.5)], [m.msg_id])

    def test_acked_message_never_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        m = mk(Rel.R2, deadline=10.0)
        rm.register(m, 0.0); rm.ack(m.msg_id)
        self.assertEqual(rm.due_retries(5.0), [])

    def test_no_retry_past_deadline(self):
        rm = ReliabilityManager(retry_interval=1.0)
        m = mk(Rel.R2, deadline=1.2)
        rm.register(m, 0.0)
        self.assertEqual(rm.due_retries(2.0), [])
        self.assertTrue(rm.expired(2.0))

    def test_r0_never_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        m = mk(Rel.R0, deadline=10.0)
        rm.register(m, 0.0)
        self.assertEqual(rm.due_retries(5.0), [])

    def test_exhausted_after_max_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        m = mk(Rel.R2, deadline=100.0)
        rm.register(m, 0.0)
        for i in range(3):
            rm.on_retry_sent(m.msg_id, float(i) + 1.0)
        self.assertTrue(rm.exhausted(m.msg_id))

    def test_pending_count(self):
        rm = ReliabilityManager()
        a, b = mk(Rel.R1, deadline=9.0), mk(Rel.R1, deadline=9.0)
        rm.register(a, 0.0); rm.register(b, 0.0)
        rm.ack(a.msg_id)
        self.assertEqual(rm.pending_count(), 1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_reliability -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

`register` 只对 `Rel != R0` 的消息登记。`due_retries(t)` 返回 `t >= next_retry_at` **且**
`t <= msg.deadline` **且**未 ACK 且未耗尽的消息，并推进 `next_retry_at`。
`expired(t)` 返回超期未 ACK 的消息（引擎据此记 `DEADLINE_MISSED`）。
核心可验证命题：**重传受 deadline 约束**（spec §6.5）。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_reliability -v`
Expected: PASS（11 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(protocol): R0-R3 可靠性策略与截止期约束"
```

---

## Task 10: 楼块、节点与运动

**Files:**
- Create: `skyfleet/world/building.py`, `skyfleet/world/node.py`, `skyfleet/world/mobility.py`
- Test: `tests/unit/test_world.py`

**Interfaces:**
- Consumes: `geometry`（Task 3）
- Produces: `Building(id, rect, height)`；`grid_buildings(area, blocks, gap, height_range, rng) -> list[Building]`；
  `Node(node_id, role, pos, speed, energy)` 带 `.pos`、`.move_to(target, dt)`、`.consume(amount)`；
  `WaypointMover(node, speed)` 带 `.set_route(waypoints)`、`.step(dt) -> bool`（返回是否到达）、`.arrived`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_world.py
import unittest
from skyfleet.kernel.rng import RngRegistry
from skyfleet.protocol.identity import Role
from skyfleet.world.building import Building, grid_buildings
from skyfleet.world.mobility import WaypointMover
from skyfleet.world.node import Node

class TestGridBuildings(unittest.TestCase):
    def test_generates_expected_count(self):
        bs = grid_buildings(area=500.0, blocks=3, gap=20.0,
                            height_range=(30.0, 120.0), rng=RngRegistry(1).get("b"))
        self.assertEqual(len(bs), 9)

    def test_buildings_stay_inside_area(self):
        for b in grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(1).get("b")):
            x, y, w, h = b.rect
            self.assertGreaterEqual(x, 0.0)
            self.assertLessEqual(x + w, 500.0)
            self.assertGreaterEqual(y, 0.0)
            self.assertLessEqual(y + h, 500.0)

    def test_heights_within_range(self):
        for b in grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(1).get("b")):
            self.assertGreaterEqual(b.height, 30.0)
            self.assertLessEqual(b.height, 120.0)

    def test_reproducible(self):
        a = grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(5).get("b"))
        b = grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(5).get("b"))
        self.assertEqual([x.height for x in a], [x.height for x in b])

class TestNode(unittest.TestCase):
    def test_consume_reduces_energy(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        n.consume(30.0)
        self.assertAlmostEqual(n.energy, 70.0)

    def test_energy_floors_at_zero(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=5.0)
        n.consume(50.0)
        self.assertEqual(n.energy, 0.0)

class TestWaypointMover(unittest.TestCase):
    def test_moves_toward_waypoint(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(100.0, 0.0, 50.0)])
        mv.step(1.0)
        self.assertAlmostEqual(n.pos[0], 10.0)

    def test_arrives_and_advances(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(10.0, 0.0, 50.0), (20.0, 0.0, 50.0)])
        self.assertTrue(mv.step(1.0))
        self.assertAlmostEqual(n.pos[0], 10.0)

    def test_reports_arrived_when_route_exhausted(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=100.0, energy=100.0)
        mv = WaypointMover(n, speed=100.0)
        mv.set_route([(10.0, 0.0, 50.0)])
        mv.step(1.0)
        self.assertTrue(mv.arrived)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_world -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

`grid_buildings` 把 `area` 切成 `blocks × blocks` 网格，每格留 `gap`，楼高由传入 `rng` 抽样。
`WaypointMover.step(dt)` 按 `speed * dt` 前进，不超过剩余距离；到达一个航点即推进到下一个，
返回本次是否到达；航点耗尽置 `arrived = True`。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_world -v`
Expected: PASS（10 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(world): 楼群节点与航点运动"
```

---

## Task 11: 链路模型（可插拔 + 几何遮挡）

**Files:**
- Create: `skyfleet/world/link.py`
- Test: `tests/unit/test_link.py`

**Interfaces:**
- Consumes: `geometry`（Task 3）、`Building`（Task 10）
- Produces: `LinkQuality(quality: float, obstructed: bool, blocked_by: str | None, trend: float)`；
  `LinkModel`（`typing.Protocol`，方法 `evaluate(a, b, t) -> LinkQuality`）；
  `GeometricOcclusionLinkModel(buildings, max_range, min_quality, trend_window)`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_link.py
import unittest
from skyfleet.protocol.identity import Role
from skyfleet.world.building import Building
from skyfleet.world.link import GeometricOcclusionLinkModel
from skyfleet.world.node import Node

def node(nid, x, y, z):
    return Node(nid, Role.SLAVE, (x, y, z), speed=10.0, energy=100.0)

WALL = Building("B1", (40.0, 0.0, 20.0, 100.0), height=80.0)

class TestOcclusion(unittest.TestCase):
    def test_clear_line_is_not_obstructed(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=1000.0, min_quality=0.05)
        q = m.evaluate(node("A", 0, 50, 100), node("B", 100, 50, 100), 0.0)
        self.assertFalse(q.obstructed)

    def test_line_through_building_is_obstructed(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=1000.0, min_quality=0.05)
        q = m.evaluate(node("A", 0, 50, 50), node("B", 100, 50, 50), 0.0)
        self.assertTrue(q.obstructed)
        self.assertEqual(q.blocked_by, "B1")

    def test_flying_above_building_avoids_occlusion(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=1000.0, min_quality=0.05)
        q = m.evaluate(node("A", 0, 50, 90), node("B", 100, 50, 90), 0.0)
        self.assertFalse(q.obstructed)

    def test_obstructed_quality_is_heavily_penalised(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=1000.0, min_quality=0.05)
        q = m.evaluate(node("A", 0, 50, 50), node("B", 100, 50, 50), 0.0)
        self.assertLessEqual(q.quality, 0.1)

    def test_beyond_range_is_unusable(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=50.0, min_quality=0.05)
        self.assertEqual(m.evaluate(node("A", 0, 0, 50), node("B", 500, 0, 50), 0.0).quality, 0.0)

    def test_closer_links_are_better(self):
        m = GeometricOcclusionLinkModel([], max_range=1000.0, min_quality=0.05)
        near = m.evaluate(node("A", 0, 0, 50), node("B", 20, 0, 50), 0.0)
        far = m.evaluate(node("A", 0, 0, 50), node("B", 400, 0, 50), 0.0)
        self.assertGreater(near.quality, far.quality)

    def test_quality_within_bounds(self):
        m = GeometricOcclusionLinkModel([WALL], max_range=1000.0, min_quality=0.05)
        for d in (0, 10, 100, 400):
            q = m.evaluate(node("A", 0, 0, 50), node("B", d, 0, 50), 0.0).quality
            self.assertGreaterEqual(q, 0.0)
            self.assertLessEqual(q, 1.0)

class TestTrend(unittest.TestCase):
    def test_trend_is_zero_without_history(self):
        m = GeometricOcclusionLinkModel([], max_range=1000.0, min_quality=0.05)
        self.assertEqual(m.evaluate(node("A", 0, 0, 50), node("B", 20, 0, 50), 0.0).trend, 0.0)

    def test_trend_negative_when_quality_falls(self):
        m = GeometricOcclusionLinkModel([], max_range=1000.0, min_quality=0.05)
        a, b = node("A", 0, 0, 50), node("B", 20, 0, 50)
        m.evaluate(a, b, 0.0)                       # 近距，质量高
        far = node("B", 450, 0, 50)
        for t in (1.0, 2.0):
            q = m.evaluate(a, far, t)               # 变远，质量下降
        self.assertLess(q.trend, 0.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_link -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

判定遮挡：线段与楼块矩形相交**且** `building.height >= min(a.z, b.z)` 才算遮挡
（`test_flying_above_building_avoids_occlusion` 依赖此规则）。遮挡时 `quality = min_quality`。
通畅时 `quality = max(min_quality, 1 - dist/max_range)`；`dist > max_range` 时 `quality = 0.0`。
`trend` 为按 `(a.node_id, b.node_id)` 维护的滑动窗口首末斜率，无历史时为 `0.0`。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_link -v`
Expected: PASS（9 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(world): 可插拔链路模型与几何遮挡"
```

---

## Task 12: 共享信道（竞争与冲突）

**Files:**
- Create: `skyfleet/world/channel.py`
- Test: `tests/unit/test_channel.py`

**Interfaces:**
- Consumes: `Msg`（Task 4）、`DropReason`（Task 4）、`LinkQuality`（Task 11）
- Produces: `TxAttempt(ok: bool, reason: DropReason | None, t_access: float, t_tx: float, t_prop: float, t_end: float)`；
  `Channel(rate_bps, interference_range_m, rng, prop_speed=299792458.0)` 带
  `.attempt(msg_id, src_id, src_pos, dst_id, dst_pos, t, link_quality, size_bits) -> TxAttempt`、
  `.busy_until() -> float`、`.active_count(t) -> int`、`.reset()`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_channel.py
import unittest
from skyfleet.kernel.rng import RngRegistry
from skyfleet.observe.events import DropReason
from skyfleet.world.channel import Channel

def mk():
    return Channel(rate_bps=2_000_000, interference_range_m=300.0,
                   rng=RngRegistry(1).get("channel:collision"))

P1, P2 = (0.0, 0.0, 50.0), (100.0, 0.0, 50.0)

class TestBasics(unittest.TestCase):
    def test_free_channel_succeeds_without_waiting(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        self.assertTrue(a.ok)
        self.assertEqual(a.t_access, 0.0)

    def test_tx_time_is_size_over_rate(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertAlmostEqual(a.t_tx, 1.0)

    def test_propagation_delay_is_computed(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        self.assertAlmostEqual(a.t_prop, 100.0 / 299792458.0, places=12)

    def test_busy_until_advances_after_success(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertAlmostEqual(c.busy_until(), a.t_end)

    def test_zero_quality_cannot_transmit(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.0, 512)
        self.assertFalse(a.ok)
        self.assertIsNotNone(a.reason)

class TestContention(unittest.TestCase):
    def test_second_sender_waits_for_busy_channel(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)   # 占用 0.0-1.0
        a = c.attempt("m2", "S2", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertGreater(a.t_access, 0.0)

    def test_simultaneous_contenders_collide(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)   # busy 到 1.0
        a = c.attempt("m2", "S2", P1, "L", P2, 0.0, 0.9, 2_000_000)  # 等到 1.0
        b = c.attempt("m3", "S3", (0.0, 10.0, 50.0), "L", P2, 0.0, 0.9, 2_000_000)
        self.assertIs(a.reason, DropReason.COLLISION)
        self.assertFalse(a.ok)

    def test_distant_sender_does_not_interfere(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        far = (5000.0, 5000.0, 50.0)
        a = c.attempt("m2", "S2", far, "L", (5100.0, 5000.0, 50.0), 0.0, 0.9, 2_000_000)
        self.assertIsNot(a.reason, DropReason.COLLISION)

    def test_active_count_drops_after_transmission_ends(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertEqual(c.active_count(0.5), 1)
        self.assertEqual(c.active_count(1.5), 0)

class TestDeterminism(unittest.TestCase):
    def test_same_inputs_same_outcome(self):
        def run():
            c = mk()
            return [c.attempt(f"m{i}", f"S{i}", P1, "L", P2, 0.0, 0.9, 512)
                    for i in range(3)]
        a, b = run(), run()
        self.assertEqual([x.ok for x in a], [x.ok for x in b])
        self.assertEqual([x.reason for x in a], [x.reason for x in b])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.unit.test_channel -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

模型（对应 spec §5.5）：信道是**单次占用的共享资源**。
1. `t_start = max(t, busy_until)`，`t_access = t_start - t`
2. 检查 `[t_start, t_start + t_tx)` 是否与已登记的在途传输重叠；若重叠且双方
   **发送方相互在干扰范围内** → `COLLISION`，`ok=False`
3. `link_quality <= 0` → `OCCLUDED`
4. `link_quality < 0.5` → 以 `interference_rng` 抽签，失败记 `LINK_DEGRADED` 语义下的
   `OCCLUDED`（用 `DropReason.OCCLUDED`）
5. 成功则登记在途传输，推进 `busy_until = max(busy_until, t_start + t_tx)`，
   返回含 `t_prop = distance_3d/ prop_speed` 的 `TxAttempt`

冲突判定必须**基于时间重叠**（确定性），而非随机抽签——否则两次运行结果不一致。

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.unit.test_channel -v`
Expected: PASS（13 tests）

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(world): 共享信道竞争与冲突模型"
```

---

## Task 13–23（摘要）

以下任务沿用同一 TDD 节奏（写测试 → 验证失败 → 最小实现 → 验证通过 → 提交），
文件、接口与验收测试如下。

| # | 文件 | 关键接口 | 验收测试 |
|---|---|---|---|
| 13 | `protocol/routing.py` | `RoutingParams(h_up, h_down, t_hold)`；`RoutingManager`：`.consider(node_id, quality, t) -> bool`、`.route_version`、`.switch_count`、`.flap_count` | `test_routing.py`：迟滞内不切换、持续改善超 `t_hold` 才切换、短窗回切计入 flap |
| 14 | `kernel/config.py` + `scenarios/baseline_small.json` | `SimConfig` dataclass（含全部门限）；`load_config(path)`；`apply_overrides(cfg, seed, duration)` | `test_config.py`：加载成功、缺字段报错、覆盖生效、所有门限可从 JSON 读到 |
| 15 | `mission/task.py`、`delivery.py`、`safety.py` | `TaskState` 枚举；`Task`；`DeliveryLedger`：`.confirm(op_id, t) -> bool`（幂等）；`SafetyAction` 枚举；`safety_decision(link_lost, leader_lost, low_battery, pos_stale) -> SafetyAction` | `test_mission.py`：合法转移、非法转移被拒、重复确认只生效一次、各触发条件给对应动作 |
| 16 | `observe/log.py` | `EventLog(path)`：`.write(kind, t, **fields)`、`.close()`；`read_log(path) -> Iterator[dict]`；`normalised_digest(path) -> str`（sha256） | `test_log.py`：写入可读回、字段完整、摘要对同一内容稳定、对内容变化敏感 |
| 17 | `sim/agent.py` | `NodeAgent(node, cfg, registry, rngs, log, link_model)`：`.on_heartbeat_tick(t)`、`.on_telemetry_tick(t)`、`.send(msg, t)`、`.receive(msg, t, quality)`、`.queue`、`.membership` | `test_agent.py`：心跳产生 `HEARTBEAT_TX`、遥测按周期产生、R0 不重传、R2 收到 ACK 后停止重传 |
| 18 | `sim/leader.py` | `LeaderController`：`.create_tasks(n, t)`、`.assign(task, node_id, t)`、`.on_member_failed(node_id, t)`、`.mission_version` | `test_leader.py`：建 20 任务、指派递增 `mission_version`、成员失败触发重派 |
| 19 | `sim/engine.py` | `Engine(cfg)`：`.build()`、`.run() -> Path`（返回日志路径）、`.nodes`、`.leader` | `test_engine.py`：可跑完 600s、日志非空、11 节点、任务被指派 |
| 20 | `cli.py` | `main(argv)`：子命令 `run` / `metrics` / `plot` / `replay` / `all` | `test_cli.py`：`run` 产出日志路径、`metrics` 打印九项、参数解析正确 |
| 21 | `observe/metrics.py` | `compute_metrics(log_path) -> dict`；`format_report(m) -> str` | `test_metrics.py`：用**构造的小日志**断言每项指标数值精确（不依赖跑仿真） |
| 22 | `observe/replay.py`、`viz/plots.py`、`viz/replay_html.py` | `Replay.from_log(path)`：`.timeline()`、`.at(t)`；`make_figures(log_path, out_dir)`；`make_replay_html(log_path, out_path)` | `test_replay.py`：时间线单调、`at(t)` 返回正确快照；`test_viz.py`：产出文件非空且 HTML 自包含（含 `<canvas`，无外链 `src="http`） |
| 23 | `tests/integration/`、`tests/determinism/` | — | **集成**：遮挡→DEGRADED→（无佐证）→SUSPECT→FAILED→重派→`mission_version` 递增→旧指派 `STALE_VERSION`；**确定性**：同 seed 两次运行 `normalised_digest` 相等 |

**Task 21 的指标测试必须用构造日志**，不能跑完整仿真——否则测试既慢又脆弱，且无法断言
精确数值。构造日志的辅助函数放在 `tests/support/logfixtures.py`。

---

## Self-Review

**Spec 覆盖检查**

| Spec 章节 | 对应任务 |
|---|---|
| §4.1 时间与事件模型 | Task 2 |
| §4.2 独立随机源 | Task 1 |
| §4.3 事件与原因码 | Task 4 |
| §4.4 配置 | Task 14 |
| §5.1 几何 | Task 3 |
| §5.2 楼块 | Task 10 |
| §5.3 节点与运动 | Task 10 |
| §5.4 链路模型接口 | Task 11 |
| §5.5 共享信道 | Task 12 |
| §6.1 身份/会话/角色 | Task 6 |
| §6.2 成员状态机 | Task 7 |
| §6.3 版本与顺序 | Task 5 |
| §6.4 消息格式 | Task 4 |
| §6.5 可靠性策略 | Task 9 |
| §6.6 队列与限流 | Task 8 |
| §6.7 路由迟滞 | Task 13 |
| §7.1–7.4 业务层 | Task 15 |
| §8.1 事件日志 | Task 16 |
| §8.2–8.4 指标 | Task 21 |
| §8.3 时延六段 | Task 12（分段计时）+ Task 21（汇总） |
| §8.5 回放与确定性 | Task 22（回放）+ Task 23（确定性） |
| §10.1 可视化方案 | Task 22 |
| §10.2 目录结构 | 全部 |

无遗漏。

**占位扫描**：无 TBD / TODO / "相似于 Task N" 式引用。

**类型一致性**：`Msg.new` 全篇统一；`DropReason` 在 Task 4 定义、Task 5/8/12 消费，名称一致；
`LinkQuality` 在 Task 11 定义、Task 12/17 消费，字段名一致；`MemberStatus`/`MembershipParams`
在 Task 7 定义并在 Task 17/18 消费，名称一致；调度器 API（`schedule_at`/`schedule`/`subscribe`/
`run`）在 Task 2 定义，Task 17–19 一律沿用，无别名。

---

## 附：实现验收记录（2026-10-04）

Task 1–23 全部完成。测试命令与结果：

```
python3 -m unittest discover -s tests -t .          # 系统 Python，跳过 5 个 matplotlib 测试
.venv/bin/python -m unittest discover -s tests -t . # 含可视化，335 passed
```

端到端：

```
.venv/bin/python -m skyfleet.cli all --out out                              # 基线
.venv/bin/python -m skyfleet.cli all \
    --config skyfleet/scenarios/stress_canyon.json --out out                # 压力对照
```

确定性验收：同一 `master_seed` 连续两次运行的规范化日志 SHA-256 一致
（基线种子 1：`6f40ceea3386dcea521e700cb4f0da0452a080a0a50ff6d09500ab1e195ce76f`）。

**与计划的偏差**：实现过程中发现 11 处**模型/协议层面的缺陷**（不是调参），
逐条记录在规格文档 §15，含理由、修正方式与实测数据。其中影响最大的三条：

1. §15.1 信道未时隙化 → 冲突数恒为 0，"信道竞争"分析会整体失效；
2. §15.8 投递确认未等 ACK → 任务完成率被钉死在"每节点恰好一个"（10/20）；
3. §15.9 全程低空飞行 → 遮挡变成网络分区，而非可观测的劣化。

对应的测试改动：`tests/unit/test_channel.py` 重写为时隙模型；
`tests/unit/test_version.py` 的 `last_seq` 键由 3 元组改为 4 元组；
`tests/unit/test_agent.py` 新增周期消息不重传、R2 登记重传两条。
