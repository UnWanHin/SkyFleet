"""离散事件仿真引擎：把各部件装配成一个可运行的集群。

装配顺序（也是事件的因果顺序）：
    世界与节点 -> 代理 -> 信道 -> 周期性事件 -> 主节点编排

三条贯穿全案的设计约束在这里落地：
1. **确定性**：所有随机性来自 RngRegistry 的具名独立流；周期性事件带
   确定性抖动，避免所有节点在同一虚拟时刻齐发（那会让"冲突"变成必然，
   而不是一个可观察、可归因的现象）。
2. **共享信道**：发送意图在时刻结束时统一结算，同刻并发且互相在
   干扰范围内的发送全部判为 COLLISION。
3. **唯一事实来源**：引擎只写日志，不算指标。指标由 observe/metrics.py
   从日志重算。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

from skyfleet.kernel.config import SimConfig
from skyfleet.kernel.event import Event
from skyfleet.kernel.rng import RngRegistry
from skyfleet.kernel.scheduler import Scheduler
from skyfleet.mission.delivery import DeliveryLedger
from skyfleet.mission.safety import SafetyAction, safety_decision
from skyfleet.mission.task import TaskBoard, TaskState
from skyfleet.observe.events import DropReason, EventKind
from skyfleet.observe.log import EventLog
from skyfleet.protocol.identity import IdentityRegistry, Role
from skyfleet.protocol.message import Msg
from skyfleet.protocol.version import AcceptDecision
from skyfleet.sim.agent import NodeAgent
from skyfleet.sim.leader import LeaderController
from skyfleet.world.building import grid_buildings
from skyfleet.world.channel import Channel
from skyfleet.world.link import GeometricOcclusionLinkModel
from skyfleet.world.mobility import WaypointMover
from skyfleet.world.node import Node

# 事件类型（调度器键，非日志枚举）
EV_HEARTBEAT = "hb"
EV_TELEMETRY = "tlm"
EV_MOTION = "motion"
EV_LEADER_CHECK = "leader_check"
EV_RETRY = "retry"
EV_RX = "rx"
EV_KILL = "kill"

LEASE_TTL = 1_000_000.0


@dataclass
class _Tx:
    msg: Msg
    sender: NodeAgent
    attempt: Any
    quality: float
    next_hop: str


@dataclass
class _Rx:
    msg: Msg
    receiver: NodeAgent
    quality: float
    attempt: Any


class Engine:
    def __init__(self, config: SimConfig) -> None:
        self.config = config
        self.scheduler = Scheduler()
        self.rngs = RngRegistry(config.seed)
        self.channel = Channel(config.channel_rate_bps,
                               config.interference_range_m,
                               self.rngs.get("channel:collision"),
                               loss_threshold=config.loss_threshold)
        self.identity = IdentityRegistry()
        self.board = TaskBoard()
        self.delivery = DeliveryLedger()

        self.buildings: list[Any] = []
        self.link_model: GeometricOcclusionLinkModel | None = None
        self.nodes: dict[str, Node] = {}
        self.agents: dict[str, NodeAgent] = {}
        self.movers: dict[str, WaypointMover] = {}
        self.leader: LeaderController | None = None
        self.log: EventLog | None = None

        self._built = False
        self._kills: list[tuple[str, float]] = []
        self._tx_batch: list[_Tx] = []

        # 任务执行簿记
        self._node_task_queue: dict[str, list[str]] = {}
        self._active_task: dict[str, str] = {}
        self._task_leg: dict[str, int] = {}
        self._known_tasks: set[str] = set()

        # 观测簿记
        self._occluded: dict[str, bool] = {}
        self._last_safety: dict[str, SafetyAction] = {}
        self._sample_countdown = 0

    # ================= 构建 =================

    def build(self) -> "Engine":
        cfg = self.config

        self.buildings = grid_buildings(cfg.area, cfg.blocks, cfg.gap,
                                        cfg.height_range,
                                        self.rngs.get("env:buildings"))
        self.link_model = GeometricOcclusionLinkModel(
            self.buildings, max_range=cfg.max_link_range)

        self._place_nodes()

        for node in self.nodes.values():
            agent = NodeAgent(node, cfg, self.identity, self.rngs, None, self)
            self.agents[node.node_id] = agent
            self.movers[node.node_id] = WaypointMover(node, node.speed)

        self.leader = LeaderController(
            cfg, self.board, self.agents[cfg.leader_id].membership,
            self.agents[cfg.leader_id], self.identity, None)

        self._built = True
        return self

    def _open_log(self) -> None:
        """日志在 run() 时才打开：build() 是可离线调用的纯装配步骤，
        不应留下未关闭的文件句柄。"""
        self.log = EventLog(self.config.log_path, append=False)
        for agent in self.agents.values():
            agent.log = self.log
        assert self.leader is not None
        self.leader.log = self.log

    def _place_nodes(self) -> None:
        cfg = self.config
        rng = self.rngs.get("env:placement")
        area = cfg.area
        low, high = cfg.flight_altitude

        self.nodes[cfg.leader_id] = Node(
            cfg.leader_id, Role.LEADER, (area / 2, area / 2, (low + high) / 2),
            speed=0.0, energy=cfg.initial_energy)

        ids = ([f"S{i + 1}" for i in range(cfg.slaves)]
               + [f"R{i + 1}" for i in range(cfg.relays)])
        for index, node_id in enumerate(ids):
            role = Role.SLAVE if node_id.startswith("S") else Role.RELAY
            angle = 2.0 * math.pi * index / len(ids)
            radius = area * 0.35
            x = area / 2 + radius * math.cos(angle) + rng.uniform(-15.0, 15.0)
            y = area / 2 + radius * math.sin(angle) + rng.uniform(-15.0, 15.0)
            z = rng.uniform(low, high)
            self.nodes[node_id] = Node(
                node_id, role,
                (min(max(x, 0.0), area), min(max(y, 0.0), area), z),
                speed=cfg.node_speed, energy=cfg.initial_energy)

    @property
    def relay_ids(self) -> list[str]:
        return [n for n, node in self.nodes.items()
                if node.role is Role.RELAY]

    # ================= 运行 =================

    def schedule_kill(self, node_id: str, t: float) -> None:
        """故障注入：让某节点在 t 时刻起彻底静默。"""
        self._kills.append((node_id, t))

    def run(self) -> str:
        if not self._built:
            self.build()
        self._open_log()

        self.scheduler.on_timestamp_end = self._end_timestamp
        self._subscribe()
        self._start()

        self.scheduler.run(until=self.config.duration)
        assert self.log is not None
        self.log.close()
        return self.config.log_path

    def _subscribe(self) -> None:
        self.scheduler.subscribe(EV_HEARTBEAT, self._on_heartbeat)
        self.scheduler.subscribe(EV_TELEMETRY, self._on_telemetry)
        self.scheduler.subscribe(EV_MOTION, self._on_motion)
        self.scheduler.subscribe(EV_LEADER_CHECK, self._on_leader_check)
        self.scheduler.subscribe(EV_RETRY, self._on_retry)
        self.scheduler.subscribe(EV_RX, self._on_rx)
        self.scheduler.subscribe(EV_KILL, self._on_kill)

    def _start(self) -> None:
        cfg = self.config
        assert self.log is not None
        leader_agent = self.agents[cfg.leader_id]

        for node_id, node in self.nodes.items():
            agent = self.agents[node_id]
            agent.session_id = self.identity.join(node_id, 0.0)
            self.identity.grant_lease(node_id, node.role, 0, "cluster", 0.0,
                                      LEASE_TTL)
            self.log.write(EventKind.NODE_JOIN, 0.0, node_id=node_id,
                           role=node.role.value, session_id=agent.session_id,
                           pos=list(node.pos))

        # 初始视图：集群已建立，主节点认识所有成员会话，成员认识主节点
        leader_agent.view.sessions.add(leader_agent.session_id)
        for node_id in self.nodes:
            if node_id == cfg.leader_id:
                continue
            session = self.identity.current_session(node_id) or ""
            leader_agent.view.sessions.add(session)
            agent = self.agents[node_id]
            agent.view.sessions.add(leader_agent.session_id)
            agent.last_heard[cfg.leader_id] = 0.0
            leader_agent.membership.on_join(node_id, session, 0.0)
            leader_agent.membership.set_energy(node_id, cfg.initial_energy)
            self._node_task_queue[node_id] = []

        self.leader.create_tasks(0.0)
        self.wire_leader_handlers()

        for node_id in self.nodes:
            self.scheduler.schedule_at(self._jitter(node_id, "hb")
                                       * cfg.heartbeat_period,
                                       EV_HEARTBEAT, self.agents[node_id])
            self.scheduler.schedule_at(self._jitter(node_id, "tlm")
                                       * cfg.telemetry_period,
                                       EV_TELEMETRY, self.agents[node_id])

        self.scheduler.schedule_at(0.0, EV_MOTION)
        self.scheduler.schedule_at(cfg.heartbeat_period, EV_LEADER_CHECK)
        # 重传扫描按节点错开：全局同步扫描会让所有节点的重传在同一时刻齐发，
        # 制造出与协议设计无关的"羊群冲突"。
        for node_id, agent in self.agents.items():
            self.scheduler.schedule_at(
                self._jitter(node_id, "retry") * cfg.retry_interval
                + cfg.retry_interval, EV_RETRY, agent)
        for node_id, t in self._kills:
            self.scheduler.schedule_at(t, EV_KILL, node_id)

    def _jitter(self, node_id: str, stream: str) -> float:
        """[0,1) 抖动。没有它，所有节点会在同一虚拟时刻齐发，
        冲突就不再是"竞争现象"而是必然结果。"""
        return self.rngs.get(f"node:{node_id}:{stream}").random()

    # ================= 周期性事件 =================

    def _on_heartbeat(self, ev: Event) -> None:
        agent: NodeAgent = ev.payload
        if not agent.alive:
            return
        t = ev.t
        if agent.node.role is Role.LEADER:
            for member in list(agent.membership.members()):
                if member != agent.node.node_id:
                    agent.on_heartbeat_tick(t, dst=member)
        else:
            agent.on_heartbeat_tick(t)
        assert self.log is not None
        self.log.write(EventKind.HEARTBEAT_TX, t, node_id=agent.node.node_id)
        self.scheduler.schedule_at(t + self.config.heartbeat_period,
                                   EV_HEARTBEAT, agent)

    def _on_telemetry(self, ev: Event) -> None:
        agent: NodeAgent = ev.payload
        if not agent.alive:
            return
        t = ev.t
        agent.on_telemetry_tick(t)
        if agent.node.node_id != self.config.leader_id:
            self._observe_link(agent, t)
        self.scheduler.schedule_at(t + self.config.telemetry_period,
                                   EV_TELEMETRY, agent)

    def _observe_link(self, agent: NodeAgent, t: float) -> None:
        """评估到主节点与各中继的链路，驱动路由迟滞与遮挡事件。"""
        assert self.link_model is not None and self.log is not None
        leader_node = self.nodes[self.config.leader_id]

        to_leader = self.link_model.evaluate(agent.node, leader_node, t)
        best_id, best_quality = self.config.leader_id, to_leader.quality

        for relay_id in self.relay_ids:
            relay = self.nodes[relay_id]
            if relay_id == agent.node.node_id or not self.agents[relay_id].alive:
                continue
            quality = self.link_model.evaluate(agent.node, relay, t).quality
            if quality > best_quality + 0.05:
                best_id, best_quality = relay_id, quality

        agent.routing.consider(best_id, best_quality, t)

        obstructed = to_leader.obstructed
        was = self._occluded.get(agent.node.node_id)
        if was is not None and was != obstructed:
            self.log.write(
                EventKind.OCCLUSION_ENTER if obstructed
                else EventKind.OCCLUSION_EXIT,
                t, node_id=agent.node.node_id,
                blocked_by=to_leader.blocked_by,
                quality=round(to_leader.quality, 6))
        self._occluded[agent.node.node_id] = obstructed

    def _on_motion(self, ev: Event) -> None:
        cfg = self.config
        t = ev.t
        dt = cfg.motion_period

        for node_id, mover in self.movers.items():
            agent = self.agents[node_id]
            if not agent.alive or mover is None:
                continue
            agent.spend_energy(cfg.energy_hover_per_s * dt)
            if mover.arrived or mover.step(dt):
                self._advance_mission(node_id, t)

        self._sample_countdown += 1
        if self._sample_countdown * dt >= 5.0:
            self._sample_countdown = 0
            self._write_samples(t)

        self.scheduler.schedule_at(t + dt, EV_MOTION)

    def _write_samples(self, t: float) -> None:
        assert self.log is not None
        for node_id, agent in self.agents.items():
            self.log.write(EventKind.ENERGY_SAMPLE, t, node_id=node_id,
                           energy=round(agent.node.energy, 6))
            self.log.write(EventKind.QUEUE_SAMPLE, t, node_id=node_id,
                           **agent.queue.sample())

    # ================= 任务执行 =================

    def _advance_mission(self, node_id: str, t: float) -> None:
        assert self.log is not None
        task_id = self._active_task.get(node_id)
        if task_id is None:
            self._start_next_task(node_id, t)
            return

        task = self.board.get(task_id)
        if task is None or task.state in (TaskState.DELIVERED,
                                          TaskState.FAILED,
                                          TaskState.EXPIRED):
            self._finish_local_task(node_id)
            return

        if self._task_leg.get(node_id, 0) == 0:
            self.board.transition(task_id, TaskState.PICKED, t)
            self.log.write(EventKind.TASK_PICKUP, t, task_id=task_id,
                           node_id=node_id)
            self._task_leg[node_id] = 1
            self.movers[node_id].set_route([task.destination])
        else:
            self._send_delivery_confirm(node_id, task, t)
            self._finish_local_task(node_id)

    def _start_next_task(self, node_id: str, t: float) -> None:
        queue = self._node_task_queue.get(node_id) or []
        while queue:
            task_id = queue.pop(0)
            task = self.board.get(task_id)
            if task is None or task.state is not TaskState.ASSIGNED:
                continue
            if not self.board.transition(task_id, TaskState.ENROUTE, t):
                continue
            self._active_task[node_id] = task_id
            self._task_leg[node_id] = 0
            self.movers[node_id].set_route([task.origin])
            return

    def _finish_local_task(self, node_id: str) -> None:
        task_id = self._active_task.pop(node_id, None)
        if task_id is not None:
            self._known_tasks.discard(task_id)
        self._task_leg.pop(node_id, None)
        self.movers[node_id].set_route([])

    def _send_delivery_confirm(self, node_id: str, task, t: float) -> None:
        agent = self.agents[node_id]
        msg = Msg.new("DELIVERY_CONFIRM", src=node_id,
                      dst=self.config.leader_id,
                      t=t, msg_id=agent.new_msg_id(), seq=agent.next_seq(),
                      session_id=agent.session_id,
                      leader_epoch=agent.leader_epoch,
                      mission_version=self.leader.mission_version,
                      route_version=agent.routing.route_version,
                      operation_id=task.task_id,
                      payload={"task_id": task.task_id,
                               "delivered_at": t})
        agent.submit(msg, t)

    def _handle_delivery_confirm(self, msg: Msg, t: float,
                                 quality: float) -> None:
        assert self.log is not None
        task_id = msg.operation_id or msg.payload.get("task_id")
        if task_id is None:
            return
        if not self.delivery.confirm(task_id, t):
            return                        # 重复确认：只计数，不执行动作
        if self.leader.mark_delivered(task_id, t):
            self._known_tasks.discard(task_id)
            self.log.write(EventKind.TASK_DELIVER, t, task_id=task_id,
                           node_id=msg.src,
                           mission_version=self.leader.mission_version)

    # ================= 主节点巡检 =================

    def _on_leader_check(self, ev: Event) -> None:
        cfg = self.config
        t = ev.t
        assert self.log is not None and self.leader is not None
        agent = self.agents[cfg.leader_id]
        members = [n for n in agent.membership.members()
                   if n != cfg.leader_id]

        for node_id in members:
            record = agent.membership.record(node_id)
            if record is None or record.status in ("FAILED", "LEFT"):
                continue
            before = record.status
            if record.last_seen < t - cfg.heartbeat_period - 1e-9:
                agent.membership.on_miss(node_id, t)
                self.log.write(EventKind.HEARTBEAT_MISS, t, node_id=node_id,
                               miss_count=record.miss_count)
            self._log_status(node_id, before, record.status, t)

        before_status = {n: agent.membership.status(n) for n in members}
        for node_id, status in agent.membership.tick(t):
            self._log_status(node_id, before_status.get(node_id), status, t)
            if status.value == "FAILED":
                self._handle_failed_member(node_id, t)

        self.leader.assign_pending(t)
        self._sync_assignments()
        self._check_safety(t)

        self.scheduler.schedule_at(t + cfg.heartbeat_period, EV_LEADER_CHECK)

    def _handle_failed_member(self, node_id: str, t: float) -> None:
        assert self.leader is not None and self.log is not None
        before = dict(self.leader.assignments)
        count = self.leader.on_member_failed(node_id, t)
        if count:
            affected = {tid for tid, n in before.items() if n == node_id}
            self._node_task_queue.pop(node_id, None)
            active = self._active_task.pop(node_id, None)
            if active is not None:
                affected.add(active)
            self._task_leg.pop(node_id, None)
            self._known_tasks -= affected
            self.movers[node_id].set_route([])

    def _sync_assignments(self) -> None:
        assert self.leader is not None
        for task_id, node_id in self.leader.assignments.items():
            if task_id in self._known_tasks:
                continue
            self._known_tasks.add(task_id)
            self._node_task_queue.setdefault(node_id, []).append(task_id)

    def _check_safety(self, t: float) -> None:
        """本地安全动作：不依赖主节点的兜底行为。"""
        assert self.log is not None
        cfg = self.config
        for node_id, agent in self.agents.items():
            if node_id == cfg.leader_id or not agent.alive:
                continue
            leader_seen = agent.last_heard.get(cfg.leader_id)
            action = safety_decision(
                leader_lost=(leader_seen is None
                             or t - leader_seen > 2.0 * cfg.heartbeat_period),
                low_battery=agent.energy_is_low(),
            )
            if self._last_safety.get(node_id) != action:
                self._last_safety[node_id] = action
                self.log.write(EventKind.SAFETY_ACTION, t, node_id=node_id,
                               action=action.value)

    def _log_status(self, node_id: str, before: Any, after: Any,
                    t: float) -> None:
        if before is None or before == after:
            return
        assert self.log is not None
        self.log.write(EventKind.STATUS_CHANGE, t, node_id=node_id,
                       status=after.value if hasattr(after, "value") else after,
                       previous=(before.value if hasattr(before, "value")
                                 else before))

    # ================= 发送与接收 =================

    def transmit(self, msg: Msg, sender: NodeAgent, t: float) -> None:
        assert self.link_model is not None
        next_hop = sender.next_hop(msg)
        target = self.nodes.get(next_hop)
        if target is None:
            assert self.log is not None
            self.log.write(EventKind.MSG_DROP, t, node_id=sender.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           reason=DropReason.NO_ROUTE)
            return

        quality = self.link_model.evaluate(sender.node, target, t).quality
        attempt = self.channel.attempt(msg.msg_id, sender.node.node_id,
                                       sender.node.pos, next_hop, target.pos,
                                       t, quality, msg.size_bits)
        sender.spend_energy(self.config.energy_tx_per_bit * msg.size_bits)
        self._tx_batch.append(_Tx(msg=msg, sender=sender, attempt=attempt,
                                  quality=quality, next_hop=next_hop))

    def _end_timestamp(self, t: float) -> None:
        """时刻结束：统一结算信道竞争，再决定投递或丢弃。"""
        self.channel.flush()
        batch, self._tx_batch = self._tx_batch, []
        assert self.log is not None

        for record in batch:
            msg, sender, attempt = record.msg, record.sender, record.attempt
            node_id = sender.node.node_id

            if not attempt.ok:
                self.log.write(EventKind.MSG_DROP, t, node_id=node_id,
                               msg_id=msg.msg_id, msg_type=msg.msg_type,
                               qos=msg.qos.name,
                               reason=attempt.reason or DropReason.COLLISION)
                continue

            self.log.write(
                EventKind.MSG_TX, t, node_id=node_id, msg_id=msg.msg_id,
                msg_type=msg.msg_type, dst=msg.dst,
                next_hop=record.next_hop, qos=msg.qos.name,
                rel=msg.reliability.name, bits=msg.size_bits,
                hop=msg.hop, route_version=msg.route_version,
                mission_version=msg.mission_version,
                is_retry=bool(sender.reliability.exhausted(msg.msg_id)),
                t_access=round(attempt.t_access, 9),
                t_tx=round(attempt.t_tx, 9))

            receiver = self.agents.get(record.next_hop)
            if receiver is None:
                continue
            self.scheduler.schedule_at(
                attempt.t_end + attempt.t_prop, EV_RX,
                _Rx(msg=msg, receiver=receiver, quality=record.quality,
                    attempt=attempt))

    def _on_rx(self, ev: Event) -> None:
        payload: _Rx = ev.payload
        msg, receiver, t = payload.msg, payload.receiver, ev.t
        assert self.log is not None

        if not receiver.alive:
            return

        # 中继转发：目的不是本机，只做转发，不触发本机协议动作
        if msg.dst != receiver.node.node_id:
            self._forward(msg, receiver, t)
            return

        before = receiver.membership.status(msg.src)
        decision = receiver.receive(msg, t, payload.quality)
        after = receiver.membership.status(msg.src)
        if receiver.node.role is Role.LEADER:
            self._log_status(msg.src, before, after, t)

        if decision is not AcceptDecision.ACCEPT:
            return

        created = msg.t_created
        attempt = payload.attempt
        t_enqueue = receiver.enqueue_time(msg.msg_id)
        t_dequeue = receiver.dequeue_time(msg.msg_id)
        self.log.write(
            EventKind.MSG_RX, t, node_id=receiver.node.node_id,
            msg_id=msg.msg_id, msg_type=msg.msg_type, src=msg.src,
            qos=msg.qos.name, hop=msg.hop,
            e2e=round(t - created, 9),
            t_sample_wait=round(t_enqueue - created, 9),
            t_queue=round(t_dequeue - t_enqueue, 9),
            t_access=round(attempt.t_access, 9),
            t_tx=round(attempt.t_tx, 9),
            t_prop=round(attempt.t_prop, 9))

    def _forward(self, msg: Msg, relay: NodeAgent, t: float) -> None:
        assert self.log is not None
        if msg.ttl <= 1:
            self.log.write(EventKind.MSG_DROP, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           reason=DropReason.TTL_EXPIRED)
            return
        forwarded = replace(msg, hop=msg.hop + 1, ttl=msg.ttl - 1)
        if relay.forward(forwarded, t):
            self.log.write(EventKind.MSG_FORWARD, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, dst=msg.dst, hop=forwarded.hop)
        else:
            self.log.write(EventKind.MSG_DROP, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           reason=DropReason.QUEUE_FULL)

    # ================= 重传与故障注入 =================

    def _on_retry(self, ev: Event) -> None:
        t = ev.t
        agent: NodeAgent = ev.payload
        assert self.log is not None
        if agent.alive:
            for msg in agent.reliability.due_retries(t):
                agent.reliability.on_retry_sent(msg.msg_id, t)
                agent.forward(msg, t)
            for msg in agent.reliability.expired(t):
                self.log.write(EventKind.MSG_EXPIRE, t,
                               node_id=agent.node.node_id, msg_id=msg.msg_id,
                               msg_type=msg.msg_type,
                               reason=DropReason.DEADLINE_MISSED)
                agent.reliability.forget(msg.msg_id)
        self.scheduler.schedule_at(t + self.config.retry_interval, EV_RETRY,
                                   agent)

    def _on_kill(self, ev: Event) -> None:
        node_id = ev.payload
        agent = self.agents.get(node_id)
        if agent is None:
            return
        agent.alive = False
        assert self.log is not None
        self.log.write(EventKind.NODE_LEAVE, ev.t, node_id=node_id,
                       cause="SILENT")

    # ================= 供主节点处理器接线 =================

    def wire_leader_handlers(self) -> None:
        self.agents[self.config.leader_id].handlers[
            "DELIVERY_CONFIRM"] = self._handle_delivery_confirm
