"""单节点协议代理。

这是装配层的第一块：把身份、成员、队列、可靠性、路由这些纯逻辑部件
绑到一个节点上，并接上一条 Transport（引擎或测试替身）。

代理自身不做信道建模——那是 Channel 的职责；代理只负责"我想发这条消息"。
"""
from __future__ import annotations

import math
from typing import Any, Callable, Protocol

from skyfleet.kernel.config import SimConfig
from skyfleet.kernel.rng import RngRegistry
from skyfleet.observe.events import DropReason, EventKind
from skyfleet.protocol.identity import IdentityRegistry
from skyfleet.protocol.message import Msg, Rel

#: 时效性指令：过了有效期就不该再被执行。心跳/遥测/ACK 迟到不影响
#: 正确性，一律拒绝反而平白降低可达性，因此不在此列。
TIMELY_ORDERS = frozenset({"TASK_ASSIGN", "TASK_REASSIGN"})
from skyfleet.protocol.membership import MembershipParams, MembershipView
from skyfleet.protocol.queue import RateLimiter, SendQueue
from skyfleet.protocol.reliability import ReliabilityManager
from skyfleet.protocol.routing import RoutingManager, RoutingParams
from skyfleet.protocol.version import AcceptDecision, DedupTable, LocalView, accept
from skyfleet.world.node import Node


class Transport(Protocol):
    """发送通道。引擎实现它；测试用 FakeTransport。"""

    def transmit(self, msg: Msg, sender: "NodeAgent", t: float) -> None: ...


class NodeAgent:
    """一个节点的协议状态机。"""

    def __init__(self, node: Node, config: SimConfig,
                 identity: IdentityRegistry, rngs: RngRegistry,
                 log: Any, transport: Transport) -> None:
        self.node = node
        self.config = config
        self.identity = identity
        self.rngs = rngs
        self.log = log
        self.transport = transport

        self.session_id = ""
        self.leader_epoch = 0
        self.mission_version = 0
        self.alive = True

        self.queue = SendQueue(depth_limit=config.queue_depth,
                               rng=rngs.get(f"node:{node.node_id}:queue"))
        self.reliability = ReliabilityManager(config.retry_interval)
        self.rate_limiter = RateLimiter(config.q0_rate, config.q0_burst,
                                        config.q1_rate, config.q1_burst)
        self.routing = RoutingManager(
            RoutingParams(config.h_up, config.h_down, config.t_hold),
            flap_window=config.flap_window)
        self.membership = MembershipView(MembershipParams(
            config.n_degraded, config.n_suspect, config.t_fail,
            config.q_degraded))
        self.view = LocalView()
        self.dedup = DedupTable(config.dedup_ttl, config.dedup_capacity)
        #: msg_id 级去重表：识别"同一条消息的重传副本"。
        #: 没有它，重传副本会以 STALE_SEQ 的名义混进"过期"统计，
        #: 把"去重"和"过期"两个完全不同的现象混为一谈。
        self.seen_msg_ids = DedupTable(config.dedup_ttl, config.dedup_capacity)

        #: node_id -> 最近一次直接收到其心跳的时刻（用于邻居间接探测）
        self.last_heard: dict[str, float] = {}
        #: 消息类型 -> 业务层处理器（仅装配层会注册）
        self.handlers: dict[str, Callable[[Msg, float, float], None]] = {}

        self._seq = 0
        self._msg_seq = 0
        self._enqueue_t: dict[str, float] = {}
        self._dequeue_t: dict[str, float] = {}

    def new_msg_id(self) -> str:
        """节点本地生成的确定性消息 ID。

        不能用 message.py 里的模块级计数器：它跨仿真运行持续递增，
        会让"同种子两次运行产生同一日志摘要"这条核心验收直接失败。
        """
        self._msg_seq += 1
        return f"{self.node.node_id}:{self._msg_seq}"

    # ---------- 供引擎查询的计时信息（六段时延的第一、二段） ----------

    def enqueue_time(self, msg_id: str) -> float | None:
        """本节点为该消息入队的时刻；从未入队则返回 None。

        必须是 None 而不是 0.0：接收广播或转发副本的节点从未在本机排队，
        若用 0.0 兜底，"采样等待 = 入队时刻 - 生成时刻"会被算成
        "0 - 生成时刻"，六段时延分解里就会出现几十万毫秒的负值。
        """
        return self._enqueue_t.get(msg_id)

    def dequeue_time(self, msg_id: str) -> float | None:
        return self._dequeue_t.get(msg_id)

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    # ---------- 周期性流量 ----------

    def on_heartbeat_tick(self, t: float, dst: str | None = None) -> bool:
        if not self.alive:
            return False
        # 目的地址始终是**逻辑**目的地（主节点）；中继由 next_hop 决定。
        # 若把中继写进 dst，中继就会把它当作本机业务消息处理，
        # 从而被会话/版本校验挡住——那样中继转发永远不可能成功。
        target = dst if dst is not None else self.config.leader_id
        return self.submit(self._heartbeat(t, target), t)

    def on_telemetry_tick(self, t: float) -> bool:
        if not self.alive:
            return False
        return self.submit(self._telemetry(t), t)

    def _heartbeat(self, t: float, dst: str) -> Msg:
        return Msg.new(
            "HEARTBEAT", src=self.node.node_id, dst=dst, t=t,
            msg_id=self.new_msg_id(),
            seq=self.next_seq(), session_id=self.session_id,
            leader_epoch=self.leader_epoch,
            mission_version=self.mission_version,
            route_version=self.routing.route_version,
            payload={"pos": list(self.node.pos), "energy": self.node.energy,
                     "nearby": self.nearby(t)})

    def _telemetry(self, t: float) -> Msg:
        return Msg.new(
            "TELEMETRY", src=self.node.node_id, dst=self.config.leader_id,
            t=t, msg_id=self.new_msg_id(), seq=self.next_seq(),
            session_id=self.session_id,
            leader_epoch=self.leader_epoch,
            mission_version=self.mission_version,
            route_version=self.routing.route_version,
            payload={"pos": list(self.node.pos), "energy": self.node.energy})

    def nearby(self, t: float) -> list[str]:
        """最近仍能直接听到的节点——这是"邻居间接探测"的证据来源。

        必须基于**实际收到的心跳**，不能用几何距离：否则一个已经静默的
        节点仍会被邻居列为"看得见"，导致失联判定永远无法升级。
        """
        window = 2.0 * self.config.heartbeat_period
        return sorted(n for n, seen in self.last_heard.items()
                      if t - seen <= window)

    # ---------- 发送 ----------

    def submit(self, msg: Msg, t: float) -> bool:
        """限流 -> 入队 -> 登记重传 -> 立即尝试发送一条。"""
        if not self.alive:
            return False

        if not self.rate_limiter.allow(msg, t):
            self.log.write(EventKind.MSG_DROP, t, node_id=self.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           qos=msg.qos.name, dst=msg.dst,
                           reason=DropReason.RATE_LIMITED)
            return False

        self._enqueue_t[msg.msg_id] = t
        reason = self.queue.enqueue(msg, t)
        if reason is not None:
            self.log.write(EventKind.MSG_DROP, t, node_id=self.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           qos=msg.qos.name, reason=reason, dst=msg.dst)
            return False

        if self._needs_ack(msg):
            self.reliability.register(msg, t)
        self.drain(t)
        return True

    def _needs_ack(self, msg: Msg) -> bool:
        """只有 R2/R3 才登记重传。

        R1 是**周期性状态**（心跳、成员/路由更新）。它由周期本身提供重复，
        再叠加一次无条件重传只会把已经过时的状态重新灌进信道：一次心跳
        会变成三次发送，而接收端只能把它们全部丢弃。限次重传的能力
        （MAX_RETRIES[R1]）保留，但由显式请求触发，不再默认施加于周期流量。
        """
        return msg.reliability >= Rel.R2

    def drain(self, t: float) -> Msg | None:
        """取出一条消息交给 Transport。"""
        msg = self.queue.dequeue(t)
        if msg is None:
            return None
        self._dequeue_t[msg.msg_id] = t
        self.transport.transmit(msg, self, t)
        return msg

    def forward(self, msg: Msg, t: float) -> bool:
        """中继转发：只排队发送，不登记本机重传（重传责任在源节点）。"""
        if not self.alive:
            return False
        self._enqueue_t[msg.msg_id] = t
        if self.queue.enqueue(msg, t) is not None:
            return False
        self.drain(t)
        return True

    def upstream(self) -> str:
        """当前上游：有中继则走中继，否则直连主节点。"""
        return self.routing.parent or self.config.leader_id

    def next_hop(self, msg: Msg) -> str:
        """物理下一跳。逻辑目的地为 msg.dst，可能经由中继。"""
        if msg.dst == self.config.leader_id and self.routing.parent:
            return self.routing.parent
        return msg.dst

    # ---------- 接收 ----------

    def receive(self, msg: Msg, t: float, quality: float = 1.0) -> AcceptDecision:
        if self.seen_msg_ids.seen(msg.msg_id, t):
            # 重传副本仍要确认：去重抑制的是**动作**，不是**确认**。
            # 若此处不回 ACK，源端会一直重传到有效期结束，而接收端其实
            # 早就处理过了。
            self._ack_if_needed(msg, t)
            self.log.write(EventKind.MSG_DUP, t, node_id=self.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           src=msg.src, reason=DropReason.DEDUP)
            return AcceptDecision.DEDUP

        # 有效期必须由**接收端**把关。此前 deadline 只有发送端在用（重传
        # 超时），接收端从不检查，于是延迟到达或被人为注入的过期指令照样
        # 会被执行——"过期控制消息"正是 §9 要求防的一类攻击。
        # 只对时效性指令生效：心跳/遥测/ACK 迟到不影响正确性，
        # 一律拒绝反而会平白降低可达性。
        if msg.msg_type in TIMELY_ORDERS and msg.deadline < t:
            self.log.write(EventKind.MSG_DROP, t, node_id=self.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           src=msg.src, dst=msg.dst,
                           reason=DropReason.DEADLINE_MISSED)
            return AcceptDecision.STALE_SEQ

        decision = accept(msg, self.view, self.dedup, t)

        if decision is not AcceptDecision.ACCEPT:
            if decision in (AcceptDecision.DEDUP, AcceptDecision.STALE_SEQ):
                self._ack_if_needed(msg, t)
            self.log.write(
                EventKind.MSG_DUP if decision is AcceptDecision.DEDUP
                else EventKind.MSG_DROP,
                t, node_id=self.node.node_id, msg_id=msg.msg_id,
                msg_type=msg.msg_type, src=msg.src, dst=msg.dst,
                reason=decision.value)
            return decision

        if msg.msg_type == "HEARTBEAT":
            self.last_heard[msg.src] = t
            self.membership.on_heartbeat(msg.src, msg.session_id, t, quality,
                                         msg.seq)
            energy = msg.payload.get("energy")
            if energy is not None:
                self.membership.set_energy(msg.src, energy)
            for neighbour in msg.payload.get("nearby", ()):
                self.membership.on_indirect(neighbour, t)

        if msg.msg_type == "ACK":
            self.reliability.ack(msg.payload.get("ack_for", ""))
        elif msg.reliability >= Rel.R2:
            self._send_ack(msg, t)
        handler = self.handlers.get(msg.msg_type)
        if handler is not None:
            handler(msg, t, quality)
        return decision

    def _ack_if_needed(self, msg: Msg, t: float) -> None:
        """对已处理过的 R2/R3 消息补发确认。"""
        if msg.msg_type != "ACK" and msg.reliability >= Rel.R2:
            self._send_ack(msg, t)

    def _send_ack(self, original: Msg, t: float) -> None:
        ack = Msg.new("ACK", src=self.node.node_id, dst=original.src, t=t,
                      msg_id=self.new_msg_id(), seq=self.next_seq(),
                      session_id=self.session_id,
                      leader_epoch=self.leader_epoch,
                      mission_version=self.mission_version,
                      payload={"ack_for": original.msg_id})
        self.submit(ack, t)

    # ---------- 能量 ----------

    def spend_energy(self, amount: float) -> None:
        self.node.consume(amount)
        self.membership.set_energy(self.node.node_id, self.node.energy)

    def energy_is_low(self) -> bool:
        return self.node.energy < self.config.min_energy_for_task
