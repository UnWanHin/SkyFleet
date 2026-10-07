"""离散事件仿真引擎：把各部件装配成一个可运行的集群。

装配顺序（也是事件的因果顺序）：
    世界与节点 -> 代理 -> 信道 -> 周期性事件 -> 主节点编排

三条贯穿全案的设计约束在这里落地：
1. **确定性**：所有随机性来自 RngRegistry 的具名独立流；周期性事件带
   确定性抖动与时钟漂移，避免所有节点在同一虚拟时刻齐发（那会让"冲突"
   变成必然，而不是一个可观察、可归因的现象）。
2. **共享信道**：信道是按时隙争用的共享资源，同一时隙内互相在干扰范围内
   的并发发送全部判为 COLLISION；广播是一次发送、多接收端分别判定。
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
from skyfleet.world.channel import Channel, reception_ok
from skyfleet.sim.adversary import (
    EV_ATTACK, AdversaryInjector, AttackKind, build_attack_msg,
)
from skyfleet.world.geofence import breach as geofence_breach
from skyfleet.world.geofence import build_geofences
from skyfleet.world.impairment import wrap_with_impairments
from skyfleet.world.link import LinkModel, build_link_model
from skyfleet.world.position import build_position_source
from skyfleet.world.power import build_energy_model
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
EV_REVIVE = "revive"
EV_CHANNEL = "chan"

#: 广播目的地址。主节点心跳用一条广播覆盖全员，而不是逐节点单播
#: （后者会把信道负载放大 N 倍，冲突率被严重高估）。
BROADCAST = "*"

LEASE_TTL = 1_000_000.0


def _step_velocities(before, after, dt: float) -> tuple[float, float]:
    """由一步的实际位移推出 (水平速度, 垂直速度)。

    用位移而不是"看航线还剩哪一段"来判断飞行状态：位移是精确的、
    与航线规划无关，且天然把斜向航段分解成两个分量。
    """
    if dt <= 0.0:
        return 0.0, 0.0
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    dz = after[2] - before[2]
    return math.hypot(dx, dy) / dt, dz / dt


@dataclass
class _Tx:
    msg: Msg
    sender: NodeAgent
    attempt: Any
    quality: float
    next_hop: str
    broadcast: bool = False


@dataclass
class _InjectedAttempt:
    """对抗注入消息的"伪发送记录"。

    注入消息没有真实占用信道，因此时延各段为 0；但接口必须与真实
    发送记录一致，否则接收路径要为此加分支，注入就绕开了正常路径。
    """

    t_access: float = 0.0
    t_tx: float = 0.0
    t_prop: float = 0.0
    t_start: float = 0.0
    t_end: float = 0.0
    ok: bool = True
    reason: str | None = None


@dataclass
class _Rx:
    msg: Msg
    receiver: NodeAgent
    quality: float
    attempt: Any


#: 规格 §3.2 规定的三种中继布点。
#:
#: - `ring`：内环 + 高高度。历史行为，**默认**，逐位不变。
#: - `grid`：按 ceil(sqrt(n)) 均布于任务域。用来检验「环」带来的
#:   旋转对称性是否掩盖了协议效应。
#: - `farthest`：贪心最远点。作为可达性**上界参考**，不是实用方案。
RELAY_LAYOUTS = frozenset({"ring", "grid", "farthest"})

#: `farthest` 贪心用的候选格边长。候选越密，贪心解越接近真正的最远点，
#: 但代价是 O(candidates x chosen)。9 已足够——中继数量是个位数。
_FARTHEST_CANDIDATE_GRID = 9


def _relay_base_positions(layout: str, area: float,
                          indices: list[int], total: int
                          ) -> list[tuple[float, float]]:
    """按布点给出各中继的**基准**坐标（不含抖动）。

    `indices` 是中继在全体节点中的全局序号，`total` 是全体节点数。
    `ring` 必须用这两个值复现历史公式——用「中继自身序号」均分所得
    角度与历史不同，会直接改变仿真结果（实测 golden digest 因此变化）：
    8 从机 + 2 中继时，全局公式给出 288° 与 324°，自身序号给出 0° 与
    180°，是完全不同的布点。
    """
    count = len(indices)
    if count <= 0:
        return []
    if layout == "ring":
        # 内环：与主节点同心，半径 0.18 x area。**逐字保留**历史公式。
        radius = area * 0.18
        return [(area / 2 + radius * math.cos(2.0 * math.pi * index / total),
                 area / 2 + radius * math.sin(2.0 * math.pi * index / total))
                for index in indices]
    if layout == "grid":
        # 把中继均布在任务域上（留出 20% 边距，避免贴边）。列数取
        # ceil(sqrt(n))，使网格尽量接近正方形。
        cols = math.ceil(math.sqrt(count))
        rows = math.ceil(count / cols)
        margin = area * 0.2
        span = area - 2.0 * margin
        out: list[tuple[float, float]] = []
        for i in range(count):
            r, c = divmod(i, cols)
            x = margin + (span * (c + 0.5) / cols if cols else area / 2)
            y = margin + (span * (r + 0.5) / rows if rows else area / 2)
            out.append((x, y))
        return out
    if layout == "farthest":
        # 贪心最远点：每次选距「已选点集 + 主节点」最近距离最大的候选。
        # 起点放入主节点位置，使第一个中继就落在离主节点最远处。
        center = (area / 2, area / 2)
        grid = _FARTHEST_CANDIDATE_GRID
        candidates = [(area * (i + 0.5) / grid, area * (j + 0.5) / grid)
                      for i in range(grid) for j in range(grid)]
        chosen: list[tuple[float, float]] = []
        for _ in range(count):
            best, best_d = None, -1.0
            for point in candidates:
                d = min([math.dist(point, center)]
                        + [math.dist(point, p) for p in chosen])
                if d > best_d:
                    best, best_d = point, d
            assert best is not None
            chosen.append(best)
        return chosen
    raise ValueError(f"未知 relay_layout: {layout!r}")


class Engine:
    def __init__(self, config: SimConfig) -> None:
        self.config = config
        self.scheduler = Scheduler()
        self.rngs = RngRegistry(config.seed)
        # 环境随机源与协议随机源分开：env_seed 控制楼群与初始站位，
        # seed 控制协议侧行为。标定实验靠这条隔离拿到共同随机数。
        env_seed = (config.seed if config.env_seed is None
                    else config.env_seed)
        self.env_rngs = RngRegistry(env_seed)
        self.channel = Channel(config.channel_rate_bps,
                               config.interference_range_m,
                               self.rngs.get("channel:collision"),
                               loss_threshold=config.loss_threshold,
                               slot_duration=config.slot_duration)
        self.identity = IdentityRegistry()
        self.board = TaskBoard()
        self.delivery = DeliveryLedger()

        self.buildings: list[Any] = []
        self.link_model: LinkModel | None = None
        self.nodes: dict[str, Node] = {}
        self.agents: dict[str, NodeAgent] = {}
        self.movers: dict[str, WaypointMover] = {}
        self.leader: LeaderController | None = None
        self.log: EventLog | None = None

        self._built = False
        #: **当前**主节点。可变：主节点失效后由选举切换到新当选者。
        #: 执行期一律用它，不要用 config.leader_id —— 那是**初始**主节点。
        self.leader_id: str = config.leader_id
        #: 最近一次听到主节点心跳的时刻（引擎视角，用于判失联）。
        self._last_leader_beat: float = 0.0
        self._kills: list[tuple[str, float]] = []
        self._revives: list[tuple[str, float]] = []
        self._tx_index: dict[int, _Tx] = {}
        self._slot_ticks: set[int] = set()

        # 任务执行簿记
        self._node_task_queue: dict[str, list[str]] = {}
        self._active_task: dict[str, str] = {}
        self._task_leg: dict[str, int] = {}
        self._known_tasks: set[str] = set()
        #: 等待投递确认的节点 -> (msg_id, task_id)
        self._awaiting_confirm: dict[str, tuple[str, str]] = {}

        # 观测簿记
        self._occluded: dict[str, bool] = {}
        #: node_id -> (动作, 原因)；原因参与变化检测，见 _check_safety
        self._last_safety: dict[str, tuple[SafetyAction, str]] = {}
        #: 已处理过电量耗尽的节点。用集合去重：耗尽只处置一次，
        #: 否则每 0.1 s 都会重复写事件、重复坠机。
        self._empty_batteries: set[str] = set()
        #: 已就地迫降、不再承担任务的节点（land 策略）
        self.grounded: set[str] = set()
        raw_policy = config.on_battery_empty
        if raw_policy is None:
            raw_policy = "none"
        if not isinstance(raw_policy, str):
            raise ValueError(
                f"on_battery_empty 必须是字符串，得到 {type(raw_policy).__name__}"
                f"（{raw_policy!r}）；可选 none / land / crash")
        policy = raw_policy.lower()
        if policy not in ("none", "land", "crash"):
            raise ValueError(
                f"未知的 on_battery_empty={raw_policy!r}，"
                f"可选 none / land / crash")
        self._battery_policy = policy
        self._sample_countdown = 0

    # ================= 构建 =================

    def build(self) -> "Engine":
        cfg = self.config

        self.buildings = grid_buildings(cfg.area, cfg.blocks, cfg.gap,
                                        cfg.height_range,
                                        self.env_rngs.get("env:buildings"))
        # P1：在链路模型之上按配置叠加损伤源。没有任何损伤源时
        # wrap_with_impairments 原样返回，不引入额外对象。
        self.link_model = wrap_with_impairments(
            build_link_model(cfg, self.buildings), cfg)
        self.position_source = build_position_source(cfg)
        self.energy = build_energy_model(cfg)
        self.geofences = build_geofences(cfg)
        self.adversary = AdversaryInjector.from_config(cfg)

        self._place_nodes()

        for node in self.nodes.values():
            agent = NodeAgent(node, cfg, self.identity, self.rngs, None, self)
            agent.position = self.position_source
            self.agents[node.node_id] = agent
            self.movers[node.node_id] = WaypointMover(node, node.speed)

        self.leader = LeaderController(
            cfg, self.board, self.agents[cfg.leader_id].membership,
            self.agents[cfg.leader_id], self.identity, None,
            busy_nodes=frozenset(self.relay_ids),
            grounded_nodes=self.grounded)

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
        rng = self.env_rngs.get("env:placement")
        area = cfg.area
        low, high = cfg.flight_altitude

        # 主节点在高空盘旋：既符合"以高度换覆盖"的直觉，也让遮挡主要由
        # 从节点的飞行高度决定，而不是由一个静止的低空主节点主导。
        self.nodes[cfg.leader_id] = Node(
            cfg.leader_id, Role.LEADER, (area / 2, area / 2, high),
            speed=0.0, energy=cfg.initial_energy)

        if cfg.relay_layout not in RELAY_LAYOUTS:
            # 未知布点必须报错。静默退回默认会让拼错的整轮扫参结论
            # 全部是假的——配置层对未知字段一律报错也是同一个理由。
            raise ValueError(
                f"未知 relay_layout: {cfg.relay_layout!r}，"
                f"可选 {sorted(RELAY_LAYOUTS)}")

        ids = ([f"S{i + 1}" for i in range(cfg.slaves)]
               + [f"R{i + 1}" for i in range(cfg.relays)])
        # 中继的基准坐标按布点预先算好。**只改基准，不改随机数消耗顺序**：
        # 三种布点仍然为每个节点抽同样次数的抖动，这样同一 env_seed 下
        # 三者的从节点抖动完全一致，比较才只反映「布点」这一个变量。
        relay_indices = [i for i, nid in enumerate(ids)
                         if nid.startswith("R")]
        relay_base = _relay_base_positions(
            cfg.relay_layout, area, relay_indices, len(ids))
        relay_index = 0
        for index, node_id in enumerate(ids):
            role = Role.SLAVE if node_id.startswith("S") else Role.RELAY
            # 中继一律在高空。若把中继和从节点摆在同一个高度带，两者看到的
            # 遮挡几乎一样——中继对任何一条链路都不会更好，于是永远
            # 不会被选中，中继转发这条路径实际上形同虚设。
            if role is Role.RELAY:
                bx, by = relay_base[relay_index]
                relay_index += 1
                z = high
            else:
                angle = 2.0 * math.pi * index / len(ids)
                radius = area * 0.35
                bx = area / 2 + radius * math.cos(angle)
                by = area / 2 + radius * math.sin(angle)
                z = rng.uniform(low, high)
            x = bx + rng.uniform(-15.0, 15.0)
            y = by + rng.uniform(-15.0, 15.0)
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

    def schedule_revive(self, node_id: str, t: float) -> None:
        """故障恢复：让静默的节点在 t 时刻以**新会话**重新入网。

        必须换会话：身份层用 session_id 区分重连前后，沿用旧会话会让
        主节点无法区分"迟到的旧消息"和"节点真的回来了"。
        """
        self._revives.append((node_id, t))

    def run(self) -> str:
        if not self._built:
            self.build()
        self._open_log()

        self._subscribe()
        self._start()

        self.scheduler.run(until=self.config.duration)
        # 最后一个时隙没有更晚的时刻来关闭它，收尾时必须强制结算
        for attempt in self.channel.flush(force=True):
            self._resolve_tx(attempt)
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
        self.scheduler.subscribe(EV_REVIVE, self._on_revive)
        self.scheduler.subscribe(EV_ATTACK, self._on_attack)
        self.scheduler.subscribe(EV_CHANNEL, self._on_channel)

    def _start(self) -> None:
        cfg = self.config
        assert self.log is not None
        leader_agent = self.agents[cfg.leader_id]

        self.log.write(EventKind.SCENARIO_INIT, 0.0,
                       area=cfg.area, duration=cfg.duration, seed=cfg.seed,
                       leader_id=cfg.leader_id,
                       buildings=[[b.rect[0], b.rect[1], b.rect[2], b.rect[3],
                                   b.height] for b in self.buildings],
                       nodes=[[n.node_id, n.role.value, list(n.pos)]
                              for n in self.nodes.values()])

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
        # 场景文件里的故障注入/恢复，与代码里 schedule_kill 的调用合并
        self._kills.extend(cfg.kills)
        self._revives.extend(cfg.revives)
        for node_id, t in self._kills:
            self.scheduler.schedule_at(t, EV_KILL, node_id)
        for node_id, t in self._revives:
            self.scheduler.schedule_at(t, EV_REVIVE, node_id)
        self.adversary.schedule(self.scheduler)

    def _jitter(self, node_id: str, stream: str) -> float:
        """[0,1) 抖动，用于确定各节点周期事件的初始相位。

        没有它，所有节点会在同一虚拟时刻齐发，冲突就不再是"竞争现象"
        而是必然结果。
        """
        return self.rngs.get(f"node:{node_id}:{stream}").random()

    def _drift(self, node_id: str, stream: str) -> float:
        """每次周期事件叠加的时钟漂移（±access_jitter）。"""
        jitter = self.config.access_jitter
        if jitter <= 0.0:
            return 0.0
        return self.rngs.get(f"node:{node_id}:{stream}:drift").uniform(
            -jitter, jitter)

    def _next_periodic(self, agent: NodeAgent, stream: str,
                       period: float, t: float) -> float:
        return t + period + self._drift(agent.node.node_id, stream)

    # ================= 周期性事件 =================

    def _on_heartbeat(self, ev: Event) -> None:
        agent: NodeAgent = ev.payload
        if not agent.alive:
            return
        t = ev.t
        if agent.node.node_id == self.leader_id:
            # 判据是"是不是**当前**主节点"，不是初始 Role：故障切换后
            # 新主节点的 Role 仍是 SLAVE，用角色判会让它发单播心跳，
            # 其余节点收不到，选举会被反复触发（选举风暴）。
            agent.on_heartbeat_tick(t, dst=BROADCAST)
            self._last_leader_beat = t
        else:
            agent.on_heartbeat_tick(t)
        assert self.log is not None
        self.log.write(EventKind.HEARTBEAT_TX, t, node_id=agent.node.node_id)
        self.scheduler.schedule_at(
            self._next_periodic(agent, "hb", self.config.heartbeat_period, t),
            EV_HEARTBEAT, agent)

    def _on_telemetry(self, ev: Event) -> None:
        agent: NodeAgent = ev.payload
        if not agent.alive:
            return
        t = ev.t
        agent.on_telemetry_tick(t)
        if agent.node.node_id != self.leader_id:
            self._observe_link(agent, t)
        self._maybe_elect(t)
        self.scheduler.schedule_at(
            self._next_periodic(agent, "tlm", self.config.telemetry_period, t),
            EV_TELEMETRY, agent)

    #: 主节点失联多久后触发选举（心跳周期的倍数）。
    #:
    #: 必须 > 1.5（主节点自己判成员丢失用的阈值），否则正常时钟漂移
    #: 就会触发一次选举；又要足够小，否则集群空转太久。
    ELECTION_TIMEOUT_FACTOR = 3.0

    def _maybe_elect(self, t: float) -> None:
        """主节点失联时选举新主节点。

        规则刻意做得极小：候选人按 node_id **字典序**最小者当选。理由见
        规格 §2.2——本仿真的目标是**可观测的协议行为**，不是分布式共识的
        正确性证明；规则必须简单到能在报告里用一段话说清，且能解释每一条
        新事件。要更强的保证，应先有明确的论文级需求。

        中继不参选：它们是通信基础设施，一旦成为主节点就会离开站位。
        """
        assert self.log is not None
        leader_agent = self.agents.get(self.leader_id)
        if leader_agent is not None and leader_agent.alive:
            timeout = self.ELECTION_TIMEOUT_FACTOR * self.config.heartbeat_period
            if t - self._last_leader_beat <= timeout:
                return                      # 主节点健在，无需选举
        # 候选：活着、不是中继、不在迫降/坠机集合内。
        candidates = sorted(
            nid for nid, a in self.agents.items()
            if a.alive and nid not in self.relay_ids
            and nid not in self.grounded and nid != self.leader_id)
        if not candidates:
            return                          # 全灭：无解，如实留空
        self._rebind_leader(candidates[0], t)

    def _rebind_leader(self, new_id: str, t: float) -> None:
        """把主节点职责移交给 new_id，并让全集群改指新主节点。"""
        assert self.leader is not None and self.log is not None
        old_id = self.leader_id
        new_agent = self.agents[new_id]
        # 任期 +1：这是 `STALE_EPOCH` 分支第一次**真正生效**。
        # 在此之前 leader_epoch 恒为 0、从不递增，那条分支是死代码。
        new_agent.leader_epoch = self.agents[old_id].leader_epoch + 1
        new_agent.node.role = Role.LEADER
        self.leader_id = new_id
        # 重置"最近听到主节点心跳"的时刻。
        #
        # 不做这一步会引发**选举风暴**：新主节点要到下一个心跳周期才会
        # 发心跳，而 `_last_leader_beat` 仍是旧主节点的陈值，于是每次
        # 遥测巡检都判"新主节点也失联"，反复重选。实测一次击杀产生
        # 了 3 次 LEADER_ANNOUNCE。这一行给新主节点一个心跳周期的宽限。
        self._last_leader_beat = t
        # 会话交换：新主节点用的是它**当从机时**的会话，而其他节点的
        # view.sessions 里只有旧主节点的会话，于是新主节点发出的所有
        # 任务指派都会被判 STALE_SESSION 挡下——实测未做这一步时，
        # 完成率是 0.000，日志里 6196 条 STALE_SESSION。
        #
        # 这里模拟"各节点收到 LEADER_ANNOUNCE 后接纳新主节点的会话"。
        # 刻意**不**调用 identity.join() 换会话：那会让新主节点的旧会话
        # 立即失效，它在其他节点的成员记录里会变成一次"重连"，
        # 反而制造一轮无关的失效/恢复事件。
        new_agent.view.sessions.add(new_agent.session_id)
        for agent in self.agents.values():
            # 全集群改指新主节点：否则遥测与上游选择会永远朝着一个
            # 已经失效的节点发，消息只会进黑洞。
            agent.leader_id = new_id
            agent.view.sessions.add(new_agent.session_id)
            new_agent.view.sessions.add(agent.session_id)
        old = self.leader
        self.leader = LeaderController(
            self.config, self.board, new_agent.membership, new_agent,
            self.identity, self.log,
            busy_nodes=frozenset(self.relay_ids),
            grounded_nodes=self.grounded)
        # 继承任务编排状态：新主节点必须知道哪些任务已经指派给谁，
        # 否则会把同一任务重复派给第二个节点。
        self.leader.mission_version = old.mission_version + 1
        self.leader.assignments = dict(old.assignments)
        new_agent.mission_version = self.leader.mission_version
        # 主节点专属处理器必须跟着移交。`wire_leader_handlers()` 把它
        # 注册在**初始**主节点上，故障切换后新主节点没有它，于是
        # DELIVERY_CONFIRM 会被 accept() 放行、却**没有任何处理器接手**——
        # 消息静默消失，任务永远停在“已取件”。实测：17 笔确认送达 S1，
        # `_handle_delivery_confirm` 调用次数为 0，完成率 0.000。
        self.agents[old_id].handlers.pop("DELIVERY_CONFIRM", None)
        new_agent.handlers["DELIVERY_CONFIRM"] = self._handle_delivery_confirm
        # 主节点巡检（EV_LEADER_CHECK）此前只在**初始**主节点上排程，
        # 新主节点从不会被排到，于是 `assign_pending` / `_sync_assignments`
        # / `_check_safety` 全部停摆——任务派出去却永远不会被标记完成，
        # 完成率掉到 0.000 而没有任何错误事件。必须在这里接上。
        self.scheduler.schedule_at(
            t + self.config.heartbeat_period, EV_LEADER_CHECK)
        self.log.write(EventKind.LEADER_ANNOUNCE, t, node_id=new_id,
                       old_leader=old_id,
                       leader_epoch=new_agent.leader_epoch)

    def _observe_link(self, agent: NodeAgent, t: float) -> None:
        """评估到主节点与各中继的链路，驱动路由迟滞与遮挡事件。

        中继自己**不参与**上游选择：否则 R1 选 R2、R2 选 R1 会形成转发
        环路，消息只能在 TTL 耗尽后消失——那看起来像"丢包"，实际上是
        路由设计缺陷。
        """
        assert self.link_model is not None and self.log is not None
        leader_node = self.nodes[self.leader_id]

        to_leader = self.link_model.evaluate(agent.node, leader_node, t)
        best_id, best_quality = self.leader_id, to_leader.quality

        # 只有直连不足以支撑时才把中继列为候选。详见 SimConfig 中的说明：
        # 无条件取"最优上游"会让中继承担本可直接送达的流量，信道负载翻倍。
        if (agent.node.role is not Role.RELAY
                and to_leader.quality < self.config.relay_direct_threshold):
            for relay_id in self.relay_ids:
                relay = self.nodes[relay_id]
                if (relay_id == agent.node.node_id
                        or not self.agents[relay_id].alive):
                    continue
                quality = self.link_model.evaluate(agent.node, relay,
                                                   t).quality
                if quality > best_quality + 0.1:
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
            # 按**实际位移**判定飞行状态再计费：位移是精确的，不需要
            # 从航线推测当前是在爬升还是平飞。linear 模型忽略这两个
            # 速度，因此结果与 P0/P1 逐位一致。
            before = agent.node.pos
            mover.step(dt)
            v_h, v_z = _step_velocities(before, agent.node.pos, dt)
            agent.spend_energy(self.energy.energy_for_step(v_h, v_z, dt))
            if agent.node.energy <= 0.0 and node_id not in self._empty_batteries:
                self._on_battery_empty(node_id, t)
            # 只有**整条航线走完**才算抵达任务点。航线含巡航转场点，
            # 若把"到达任意一个航点"当成抵达，无人机会在巡航高度上
            # 直接开始投递，永远不下降进入峡谷。
            if mover.arrived:
                self._advance_mission(node_id, t)

        self._sample_countdown += 1
        if self._sample_countdown * dt >= 5.0:
            self._sample_countdown = 0
            self._write_samples(t)

        self.scheduler.schedule_at(t + dt, EV_MOTION)

    def _on_battery_empty(self, node_id: str, t: float) -> None:
        """电量耗尽后的处置。

        默认 `none` 直接返回——P0/P1 就是"归零没有后果"，保持逐位兼容。
        """
        assert self.log is not None
        if self._battery_policy == "none":
            # **不登记**：none 策略下没有任何处置发生，把它记进
            # `_empty_batteries` 只会留下一个名不副实的状态。
            return
        self._empty_batteries.add(node_id)
        agent = self.agents[node_id]
        self.log.write(EventKind.BATTERY_EMPTY, t, node_id=node_id,
                       policy=self._battery_policy)
        # 两种策略都要先停住：清空航线即停止移动。随后 _advance_mission
        # 会因 arrived 而触发，因此必须同时把节点标记为不可用，
        # 否则它会立刻接到下一个任务、继续飞。
        mover = self.movers.get(node_id)
        if mover is not None:
            mover.set_route([])
        self.grounded.add(node_id)
        if self._battery_policy == "crash":
            agent.alive = False
            self.log.write(EventKind.NODE_LEAVE, t, node_id=node_id,
                           cause="BATTERY")
        # 无论迫降还是坠机，该节点名下的未完成任务都必须作废并进入重派，
        # 否则 land 会把在途/排队任务**永久搁置**：节点仍然 alive、仍然
        # 心跳，主节点的成员视图不会转 FAILED，于是任务既完不成也不会
        # 被判失败，任务完成率会无声地掉下去而没有任何事件解释原因。
        self._void_tasks_of(node_id, t)

    def _write_samples(self, t: float) -> None:
        assert self.log is not None
        for node_id, agent in self.agents.items():
            # 位置与电量一起采样：日志因此可独立重建轨迹，无需另加事件类型
            self.log.write(EventKind.ENERGY_SAMPLE, t, node_id=node_id,
                           energy=round(agent.node.energy, 6),
                           pos=[round(v, 6) for v in agent.node.pos])
            self.log.write(EventKind.QUEUE_SAMPLE, t, node_id=node_id,
                           **agent.queue.sample())

    # ================= 任务执行 =================

    def _route_to(self, point: tuple[float, float, float]
                  ) -> list[tuple[float, float, float]]:
        """规划到目标点的航线：先升到巡航高度转场，再下降进入峡谷。

        直线低空穿越整个城区会让链路**长时间**被遮挡，看起来像网络分区；
        而"高空转场 + 末端下降"让遮挡只发生在投递前后，是短暂、可恢复、
        可归因的现象——这正是报告要观察的"城市峡谷链路劣化与恢复"。
        """
        return [(point[0], point[1], self.config.cruise_altitude), point]

    def _advance_mission(self, node_id: str, t: float) -> None:
        assert self.log is not None
        if node_id in self.grounded:
            return          # 已迫降/坠机：不再推进任何任务
        if node_id in self._awaiting_confirm:
            return                       # 停在投递点，等确认被 ACK
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

        leg = self._task_leg.get(node_id, 0)
        if leg == 0:
            self.board.transition(task_id, TaskState.PICKED, t)
            self.log.write(EventKind.TASK_PICKUP, t, task_id=task_id,
                           node_id=node_id)
            self._task_leg[node_id] = 1
            self.movers[node_id].set_route(self._route_to(task.destination))
        elif leg == 1:
            # 已抵达投递点。**先爬升脱离峡谷，再上报**。
            # 街道峡谷里链路被楼群遮挡，若停在原地等确认，确认发不出去、
            # ACK 也回不来，任务会卡死直到有效期结束——投递动作本身早已
            # 完成，却因为上报路径不可用而被判失败。真实的低空配送系统
            # 同样是"投递后爬升再回传"。
            self._task_leg[node_id] = 2
            self.movers[node_id].set_route(
                [(task.destination[0], task.destination[1],
                  self.config.cruise_altitude)])
        else:
            self._send_delivery_confirm(node_id, task, t)

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
            self.movers[node_id].set_route(self._route_to(task.origin))
            return

    def _finish_local_task(self, node_id: str) -> None:
        task_id = self._active_task.pop(node_id, None)
        if task_id is not None:
            self._known_tasks.discard(task_id)
        self._task_leg.pop(node_id, None)
        self.movers[node_id].set_route([])

    def _send_delivery_confirm(self, node_id: str, task, t: float) -> None:
        """发送投递确认，并**停在这里等确认被 ACK**。

        若发完就当作完成，一次确认丢失就会让任务永远停在"已取件"：
        节点飞走了、任务没交付、主节点也不知道。任务闭环必须止于确认。
        """
        agent = self.agents[node_id]
        msg = Msg.new("DELIVERY_CONFIRM", src=node_id,
                      dst=self.leader_id,
                      t=t, msg_id=agent.new_msg_id(),
                      seq=agent.next_seq("DELIVERY_CONFIRM"),
                      session_id=agent.session_id,
                      leader_epoch=agent.leader_epoch,
                      mission_version=self.leader.mission_version,
                      route_version=agent.routing.route_version,
                      operation_id=task.task_id,
                      deadline=t + self.config.confirm_deadline,
                      payload={"task_id": task.task_id,
                               "delivered_at": t})
        self._awaiting_confirm[node_id] = (msg.msg_id, task.task_id)
        self.movers[node_id].set_route([])
        agent.submit(msg, t)

    def _make_ack_handler(self, node_id: str) -> Any:
        """为**指定节点**生成 ACK 处理器。

        必须按接收节点闭包绑定：ACK 的 msg.src 是主节点，而"谁在等这个
        ACK"只有接收方自己知道。若用 msg.src 去查等待表，键永远对不上，
        节点会永远停在投递点等一个早已到达的确认——任务完成率因此被死死
        钉在"每节点恰好一个"。
        """
        def handler(msg: Msg, t: float, quality: float) -> None:
            awaiting = self._awaiting_confirm.get(node_id)
            if (awaiting is None
                    or msg.payload.get("ack_for") != awaiting[0]):
                return
            self._awaiting_confirm.pop(node_id, None)
            self._finish_local_task(node_id)
        return handler

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
        agent = self.agents[self.leader_id]
        if not agent.alive:
            # 死掉的主节点必须立刻停止指挥，且**不再续排**。
            #
            # 这个回调绑定在引擎上，不看 agent.alive；而 _on_rx 对已死节点
            # 提前返回，于是死主节点的成员视图会**冻结**在死亡时刻，接着
            # 把仍然活着的成员逐个判成 FAILED 并重派它们的任务——那些事件
            # 在信道上从未发生过（主节点死后 MSG_TX 恒为 0），却会污染任务
            # 完成率与路由切换次数。
            #
            # 注意：这里只是让**死者**停止。接管由 `_maybe_elect` 负责
            # （P2 切片 B 补上）——它由 `_on_telemetry` 驱动，因此不依赖
            # 这个回调继续排程。
            return
        members = [n for n in agent.membership.members()
                   if n != self.leader_id]

        for node_id in members:
            record = agent.membership.record(node_id)
            if record is None or record.status in ("FAILED", "LEFT"):
                continue
            before = record.status
            # 1.5 个周期才判一次丢失：周期发送带时钟漂移，用 1.0 个周期
            # 会把正常漂移误判为丢包。
            if record.last_seen < t - 1.5 * cfg.heartbeat_period - 1e-9:
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

    def _void_tasks_of(self, node_id: str, t: float) -> None:
        """把某节点名下的未完成任务作废并交给主节点重派。

        迫降与坠机共用：两者的共同点是"这个节点再也送不了货了"，
        区别只在它是否还在网。
        """
        assert self.leader is not None and self.log is not None
        before = dict(self.leader.assignments)
        count = self.leader.on_member_failed(node_id, t)
        if not count:
            return
        affected = {tid for tid, n in before.items() if n == node_id}
        self._node_task_queue.pop(node_id, None)
        active = self._active_task.pop(node_id, None)
        if active is not None:
            affected.add(active)
        self._task_leg.pop(node_id, None)
        self._awaiting_confirm.pop(node_id, None)
        self._known_tasks -= affected
        mover = self.movers.get(node_id)
        if mover is not None:
            mover.set_route([])

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
            self._awaiting_confirm.pop(node_id, None)
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
            if node_id == self.leader_id or not agent.alive:
                continue
            leader_seen = agent.last_heard.get(self.leader_id)
            status = agent.position.report(agent.node, t)
            fence = geofence_breach(agent.node.pos, self.geofences)
            link_lost = (leader_seen is None
                         or t - leader_seen > 2.0 * cfg.heartbeat_period)
            low_battery = agent.energy_is_low()
            stale = status.stale
            # 原因也要参与变化检测：只比较动作时，"已经在悬停、随后 GNSS
            # 才失锁"这种情况不会有任何日志——恰好是最需要看见的那一类。
            cause = ("GEOFENCE" if fence is not None else
                     "LOW_BATTERY" if low_battery else
                     "POSITION_STALE" if stale else
                     "LINK_LOST" if link_lost else "NONE")
            action = safety_decision(
                leader_lost=link_lost, low_battery=low_battery,
                position_stale=stale, geofence=fence is not None,
            )
            if self._last_safety.get(node_id) != (action, cause):
                self._last_safety[node_id] = (action, cause)
                self.log.write(EventKind.SAFETY_ACTION, t, node_id=node_id,
                               action=action.value, cause=cause,
                               geofence=fence,
                               position_age=round(status.age_s, 6))

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
        broadcast = msg.dst == BROADCAST
        if broadcast:
            # 广播是**一次**发送。若按接收端复制成 N 次单播，主节点的
            # 心跳就会凭空把信道负载放大 N 倍，冲突率随之被严重高估——
            # 那是模型假象，不是协议性质。接收结果在交付时按接收端分别判定。
            next_hop, target = BROADCAST, sender.node
        else:
            next_hop = sender.next_hop(msg)
            target = self.nodes.get(next_hop)
            if target is None:
                assert self.log is not None
                self.log.write(EventKind.MSG_DROP, t,
                               node_id=sender.node.node_id, msg_id=msg.msg_id,
                               msg_type=msg.msg_type, dst=msg.dst,
                               reason=DropReason.NO_ROUTE)
                return

        quality = (1.0 if broadcast
                   else self.link_model.evaluate(sender.node, target, t).quality)
        attempt = self.channel.attempt(msg.msg_id, sender.node.node_id,
                                       sender.node.pos, next_hop, target.pos,
                                       t, quality, msg.size_bits)
        sender.spend_energy(
            self.energy.tx_energy_per_bit() * msg.size_bits)
        self._tx_index[attempt.tx_id] = _Tx(
            msg=msg, sender=sender, attempt=attempt, quality=quality,
            next_hop=next_hop, broadcast=broadcast)

        # 时隙关闭时结算竞争。发送可能跨多个时隙，取最后一个即可——
        # 一次结算会关掉它之前的所有时隙。
        last_slot = self.channel.slot_containing(attempt.t_end - 1e-12)
        if last_slot not in self._slot_ticks:
            self._slot_ticks.add(last_slot)
            self.scheduler.schedule_at(self.channel.tick_time(last_slot),
                                       EV_CHANNEL, last_slot)

    def _on_channel(self, ev: Event) -> None:
        self._slot_ticks.discard(ev.payload)
        for attempt in self.channel.flush(ev.t):
            self._resolve_tx(attempt)

    def _resolve_tx(self, attempt: Any) -> None:
        """时隙结算完毕：投递或按原因丢弃。"""
        record = self._tx_index.pop(attempt.tx_id, None)
        if record is None:
            return
        msg, sender = record.msg, record.sender
        node_id = sender.node.node_id
        t = attempt.t_start
        assert self.log is not None

        if not attempt.ok:
            self.log.write(EventKind.MSG_DROP, t, node_id=node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           qos=msg.qos.name, hop=msg.hop, dst=msg.dst,
                           reason=attempt.reason or DropReason.COLLISION)
            return

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

        if record.broadcast:
            self._deliver_broadcast(record, t)
            return

        receiver = self.agents.get(record.next_hop)
        if receiver is None:
            return
        self.scheduler.schedule_at(
            attempt.t_end + attempt.t_prop, EV_RX,
            _Rx(msg=msg, receiver=receiver, quality=record.quality,
                attempt=attempt))

    def _deliver_broadcast(self, record: _Tx, t: float) -> None:
        """广播交付：对每个接收端单独判定链路质量。

        接收端按 node_id 排序处理，随机流也按消息派生，因此判定顺序与
        结果都是确定的。
        """
        assert self.link_model is not None and self.log is not None
        msg, sender, attempt = record.msg, record.sender, record.attempt
        rng = self.rngs.get(f"rx:{msg.msg_id}")
        for node_id in sorted(self.agents):
            if node_id == sender.node.node_id:
                continue
            receiver = self.agents[node_id]
            if not receiver.alive:
                continue
            quality = self.link_model.evaluate(
                sender.node, receiver.node, attempt.t_start).quality
            if not reception_ok(quality, self.config.loss_threshold, rng):
                self.log.write(EventKind.MSG_DROP, attempt.t_start,
                               node_id=node_id, msg_id=msg.msg_id,
                               msg_type=msg.msg_type, src=msg.src,
                               dst=msg.dst, reason=DropReason.OCCLUDED,
                               stage="rx")
                continue
            self.scheduler.schedule_at(
                attempt.t_end + attempt.t_prop, EV_RX,
                _Rx(msg=msg, receiver=receiver, quality=quality,
                    attempt=attempt))

    def _on_rx(self, ev: Event) -> None:
        payload: _Rx = ev.payload
        msg, receiver, t = payload.msg, payload.receiver, ev.t
        assert self.log is not None

        if not receiver.alive:
            return

        # 广播是链路本地的：所有邻居都是接收者，收到即本地处理，绝不转发。
        # 若把广播当成"目的不是本机"，每个收到它的节点都会转发一次，
        # 转发量会凭空等于节点数乘以广播速率。
        broadcast = msg.dst == BROADCAST

        # 中继转发：目的不是本机，只做转发，不触发本机协议动作
        if not broadcast and msg.dst != receiver.node.node_id:
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
        # 采样等待与排队只有**本机产生**的消息才有意义：广播接收方与
        # 转发副本的接收方从未在本机排队，此时两段记为 0，
        # 而不是拿 0.0 去减生成时刻、算出荒谬的负时延。
        t_enqueue = receiver.enqueue_time(msg.msg_id)
        t_dequeue = receiver.dequeue_time(msg.msg_id)
        sample_wait = (t_enqueue - created) if t_enqueue is not None else 0.0
        queue_wait = ((t_dequeue - t_enqueue)
                      if (t_dequeue is not None and t_enqueue is not None)
                      else 0.0)
        self.log.write(
            EventKind.MSG_RX, t, node_id=receiver.node.node_id,
            msg_id=msg.msg_id, msg_type=msg.msg_type, src=msg.src,
            qos=msg.qos.name, hop=msg.hop,
            e2e=round(t - created, 9),
            t_sample_wait=round(sample_wait, 9),
            t_queue=round(queue_wait, 9),
            t_access=round(attempt.t_access, 9),
            t_tx=round(attempt.t_tx, 9),
            t_prop=round(attempt.t_prop, 9))

    def _forward(self, msg: Msg, relay: NodeAgent, t: float) -> None:
        assert self.log is not None
        # 只有中继角色承担转发。从节点或主节点收到"不是给自己的"消息，
        # 只能说明它不该在本次发送的接收集合里——静默丢弃，不得转发，
        # 否则共享介质上的旁听会变成一场转发风暴。
        if relay.node.role is not Role.RELAY:
            self.log.write(EventKind.MSG_DROP, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           dst=msg.dst, reason=DropReason.NO_ROUTE,
                           hop=msg.hop)
            return
        if msg.ttl <= 1:
            self.log.write(EventKind.MSG_DROP, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           dst=msg.dst, reason=DropReason.TTL_EXPIRED,
                           hop=msg.hop)
            return
        # 兜底：绝不把消息原路送回它的来源（环路防护）
        if relay.next_hop(msg) == msg.src:
            self.log.write(EventKind.MSG_DROP, t, node_id=relay.node.node_id,
                           msg_id=msg.msg_id, msg_type=msg.msg_type,
                           dst=msg.dst, reason=DropReason.NO_ROUTE,
                           hop=msg.hop)
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
                self._on_expired(agent, msg, t)
        self.scheduler.schedule_at(t + self.config.retry_interval, EV_RETRY,
                                   agent)

    def _on_expired(self, agent: NodeAgent, msg: Msg, t: float) -> None:
        """确认彻底失败：任务作废并交回主节点重派，而不是静默卡住。"""
        node_id = agent.node.node_id
        awaiting = self._awaiting_confirm.get(node_id)
        if awaiting is None or awaiting[0] != msg.msg_id:
            return
        _, task_id = awaiting
        self._awaiting_confirm.pop(node_id, None)
        self._finish_local_task(node_id)
        if self.leader.mark_failed(task_id, t):
            assert self.log is not None
            self.log.write(EventKind.TASK_FAIL, t, task_id=task_id,
                           node_id=node_id, reason=DropReason.DEADLINE_MISSED,
                           mission_version=self.leader.mission_version)

    def _on_kill(self, ev: Event) -> None:
        node_id = ev.payload
        agent = self.agents.get(node_id)
        if agent is None or not agent.alive:
            return          # 幂等：重复的注入不该写两条 NODE_LEAVE
        agent.alive = False
        assert self.log is not None
        self.log.write(EventKind.NODE_LEAVE, ev.t, node_id=node_id,
                       cause="SILENT")

    def _on_revive(self, ev: Event) -> None:
        """节点以新会话重新入网。"""
        node_id = ev.payload
        agent = self.agents.get(node_id)
        if agent is None or agent.alive:
            return
        agent.alive = True
        # 若这个节点是因电量耗尽而坠机的，复活前必须先"换电池"。
        # 否则它带着 0 电量复飞，而 _empty_batteries 已去重、不会再触发
        # 耗尽——"电量没有后果"这个刚修掉的缺陷会从另一扇门溜回来。
        # 只在非 none 策略下生效：none 策略的既有行为是"归零无后果"，
        # 动它会改变所有已发表数字。
        if self._battery_policy != "none" and node_id in self._empty_batteries:
            agent.node.energy = self.config.initial_energy
            self._empty_batteries.discard(node_id)
            self.grounded.discard(node_id)
        agent.session_id = self.identity.join(node_id, ev.t)
        agent.leader_epoch = self.agents[self.config.leader_id].leader_epoch
        # 重新入网必须让主节点接受新会话，否则此后该节点的消息全被挡在
        # 会话校验外——"恢复了但永远回不来"是最难查的一类错误。
        # 同时**作废旧会话**：否则重连前滞留在信道上的旧消息仍会被接受。
        leader_view = self.agents[self.config.leader_id].view
        leader_view.sessions = {
            sid for sid in leader_view.sessions
            if not sid.startswith(f"{node_id}#")}
        leader_view.sessions.add(agent.session_id)
        assert self.log is not None
        self.log.write(EventKind.NODE_JOIN, ev.t, node_id=node_id,
                       session_id=agent.session_id, cause="REVIVE")
        self.scheduler.schedule_at(
            ev.t + self._jitter(node_id, "hb") * self.config.heartbeat_period,
            EV_HEARTBEAT, agent)

    def _on_attack(self, ev: Event) -> None:
        """把对抗注入投进**正常接收路径**。

        不在这里判断"要不要拒绝"——那验的是补丁，不是协议。消息照常
        进入 NodeAgent.receive，由会话/任期/版本/去重/有效期检查决定去留，
        被拒时产生的 MSG_DROP 就是"校验在工作"的证据。
        """
        index, attack = ev.payload
        leader = self.agents.get(self.config.leader_id)
        if leader is None:
            return
        # 会话必须取自**被冒充的发送方**（消息的 src），而不是被攻击的一侧：
        # 发给 S1 的过期指令其 src 是主节点，会话就该是主节点的。
        # 用错了会话，消息会先被会话校验挡成 STALE_SESSION，
        # 过期校验根本没走到——"防护有效"与"检查没跑"看起来一模一样。
        impersonated = self.agents.get(attack.target or attack.node_id)
        if attack.kind is AttackKind.EXPIRED:
            impersonated = leader
        session_id = (impersonated.session_id if impersonated is not None
                      and impersonated.session_id else leader.session_id)
        msg = build_attack_msg(
            attack,
            leader_id=self.config.leader_id,
            session_id=session_id,
            leader_epoch=leader.leader_epoch,
            seq=10 ** 6 + index,
            t_created=ev.t,
            deadline=(ev.t + 1.0 if attack.kind is not AttackKind.EXPIRED
                      else ev.t - 1.0),
        )
        # 投递目标是**消息自己声明的地**，不是固定送给主节点：
        # 把发给 S1 的过期指令投到主节点，只会因"目的地不是我"被转发逻辑
        # 挡成 NO_ROUTE——看起来像防护成功，实际那条校验根本没被走到。
        receiver = self.agents.get(msg.dst) or leader
        if not receiver.alive:
            return

        self.adversary.injected.append(msg.msg_id)
        assert self.log is not None
        self.log.write(EventKind.ATTACK_INJECT, ev.t,
                       node_id=receiver.node.node_id,
                       msg_id=msg.msg_id, msg_type=msg.msg_type,
                       src=msg.src, dst=msg.dst,
                       attack_kind=attack.kind.value)
        self._on_rx(Event(t=ev.t, kind=EV_RX,
                          payload=_Rx(msg=msg, receiver=receiver,
                                      quality=1.0,
                                      attempt=_InjectedAttempt())))

    # ================= 供主节点处理器接线 =================

    def wire_leader_handlers(self) -> None:
        self.agents[self.config.leader_id].handlers[
            "DELIVERY_CONFIRM"] = self._handle_delivery_confirm
        for node_id in self.agents:
            if node_id != self.config.leader_id:
                self.agents[node_id].handlers["ACK"] = \
                    self._make_ack_handler(node_id)
