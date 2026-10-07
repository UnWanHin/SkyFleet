"""P1 对抗注入（规格 §9 的 `AdversaryInjector`）。

对抗注入的全部意义在于**检验已有协议校验是否真的在工作**：注入的
未授权节点、重放消息、过期控制消息都必须走正常接收路径，由协议的
会话/任期/版本/去重/有效期检查把它们挡下来。若靠"识别出攻击者后
直接丢弃"来实现，那验的不是协议，是补丁。

因此本模块只做两件事：
1. 构造出**格式合法**的恶意消息（否则被挡下来毫无意义）。
2. 通过引擎的正常接收入口投递，并记录注入事实以便对账。

支持的注入类型：
- `replay`：重放一条此前出现过的控制消息（旧 seq / 旧 msg_id）。
- `stale_epoch`：用已过期的 leader_epoch 发送控制消息。
- `expired`：投递一条有效期已过的控制消息。
- `forged_join`：未授权节点宣告加入（未知会话）。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any

from skyfleet.kernel.event import Event
from skyfleet.protocol.message import Msg, Qos, Rel


class AttackKind(str, enum.Enum):
    REPLAY = "replay"
    STALE_EPOCH = "stale_epoch"
    EXPIRED = "expired"
    FORGED_JOIN = "forged_join"


@dataclass(frozen=True)
class Attack:
    """一次对抗注入。`t` 是注入时刻，其余字段按 kind 解释。"""

    kind: AttackKind
    t: float
    #: 显式指定序号/任期，用于表达不同的攻击风格。默认值见 build_attack_msg。
    seq: int | None = None
    epoch: int | None = None
    #: 冒充/被注入的节点。replay 与 stale_epoch 应填**真实存在**的节点，
    #: 否则消息会先被会话校验挡掉，测不到序号与任期检查。
    node_id: str = "S1"
    target: str = ""
    #: 注入的消息是否携带**有效 MAC**。
    #:
    #: 默认 False —— 外部攻击者没有组密钥，伪造的消息在认证处即被拒。
    #: 置真用于模拟两类**认证防不住**的情形，使测试能证明认证确实挡住了
    #: 伪造（而不只是"注入都失败了"）：
    #:   - 抓包重放：MAC 有效，但序号/去重会挡下；
    #:   - 内部成员：持有组密钥，认证**不防**（组密钥方案的固有边界）。
    signed: bool = False

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "Attack":
        try:
            kind = AttackKind(str(raw["kind"]))
        except KeyError as exc:
            raise ValueError("对抗注入缺少 kind 字段") from exc
        except ValueError as exc:
            raise ValueError(f"未知对抗注入类型 {raw.get('kind')!r}") from exc
        return cls(kind=kind, t=float(raw.get("t", 0.0)),
                   node_id=str(raw.get("node_id", "S1")),
                   target=str(raw.get("target", "")),
                   signed=bool(raw.get("signed", False)),
                   seq=(int(raw["seq"]) if raw.get("seq") is not None
                        else None),
                   epoch=(int(raw["epoch"]) if raw.get("epoch") is not None
                          else None))


@dataclass
class InjectionRecord:
    """注入与其被拒的对照记录，用于结果对账。"""

    attack: Attack
    injected_msg_id: str
    rejected_as: str | None = None


#: 注入事件名（引擎订阅）
EV_ATTACK = "attack"

#: 注入消息统一使用的 QoS/R 等级：取 R2，使它们**有资格**进入重传与
#: 确认逻辑——这样被挡下来才说明校验有效，而不是因为等级太低被忽略。
_ATTACK_QOS = Qos.Q1
_ATTACK_REL = Rel.R2


def build_attack_msg(attack: Attack, *, leader_id: str, session_id: str,
                     leader_epoch: int, seq: int = 999_999,
                     t_created: float = 0.0,
                     deadline: float | None = None) -> Msg:
    """构造格式合法的攻击消息。

    默认接受方是主节点：未授权节点最可能尝试的就是冒充从机向主节点
    汇报，这样才会走到会话与任期校验。
    """
    dst = attack.target or leader_id
    if attack.kind is AttackKind.REPLAY:
        # 重放：沿用旧 seq，模拟"把之前抓到的包再发一遍"
        return Msg.new("HEARTBEAT", src=attack.node_id, dst=dst,
                       t=t_created, msg_id=f"replay:{attack.t}",
                       seq=(attack.seq if attack.seq is not None else 1),
                       session_id=session_id,
                       leader_epoch=(attack.epoch if attack.epoch is not None
                                     else leader_epoch),
                       payload={"replay": True},
                       qos=_ATTACK_QOS, reliability=_ATTACK_REL,
                       deadline=deadline)

    if attack.kind is AttackKind.STALE_EPOCH:
        # 不夹到 0：夹到 0 会让注入的任期与当前任期相同（甚至更高），
        # 攻击变成空操作，那条防线看起来"通过"了其实根本没被触发。
        # 用更小的数（含负数）才能真正走一遍 STALE_EPOCH 分支。
        #
        # 注：早前这里的理由是"leader_epoch 从不递增"。P2 切片 B 引入选举后
        # **已不再成立**——任期会随重新选主递增，故 `leader_epoch - 1` 仍然
        # 是一个比当前小的值，理由本身要换成上面那条。
        return Msg.new("HEARTBEAT", src=attack.node_id, dst=dst,
                       t=t_created, msg_id=f"stale:{attack.t}",
                       seq=(attack.seq if attack.seq is not None else seq),
                       session_id=session_id,
                       leader_epoch=(attack.epoch if attack.epoch is not None
                                     else leader_epoch - 1),
                       payload={"stale_epoch": True},
                       qos=_ATTACK_QOS, reliability=_ATTACK_REL,
                       deadline=deadline)

    if attack.kind is AttackKind.EXPIRED:
        # 有效期已过：构造时必须给一个过去的 deadline
        # deadline 必须**严格早于**投递时刻。取等于当下时 `deadline < t`
        # 不成立，过期校验不会触发，看起来"消息被拒了"其实是别的检查挡的。
        return Msg.new("TASK_ASSIGN", src=leader_id, dst=attack.node_id,
                       t=t_created, msg_id=f"expired:{attack.t}",
                       seq=(attack.seq if attack.seq is not None else seq),
                       session_id=session_id,
                       leader_epoch=leader_epoch,
                       payload={"task_id": "ATTACK"},
                       qos=_ATTACK_QOS, reliability=_ATTACK_REL,
                       deadline=(deadline if deadline is not None
                                 else t_created - 1.0))

    # FORGED_JOIN：未知身份宣告加入
    return Msg.new("HEARTBEAT", src=attack.node_id, dst=dst,
                   t=t_created, msg_id=f"forged:{attack.t}",
                   seq=(attack.seq if attack.seq is not None else seq),
                   session_id=f"{attack.node_id}#forged",
                   leader_epoch=leader_epoch,
                   payload={"forged": True},
                   qos=_ATTACK_QOS, reliability=_ATTACK_REL,
                   deadline=deadline)


@dataclass
class AdversaryInjector:
    """把配置里的对抗注入排进事件循环。

    只负责"何时投、投什么"；投递本身由引擎的接收路径完成。
    """

    attacks: list[Attack] = field(default_factory=list)
    injected: list[str] = field(default_factory=list)

    @classmethod
    def from_config(cls, config) -> "AdversaryInjector":
        raw = getattr(config, "adversary", ()) or ()
        return cls(attacks=[Attack.from_dict(item) for item in raw])

    def schedule(self, scheduler) -> None:
        for index, attack in enumerate(self.attacks):
            scheduler.schedule_at(attack.t, EV_ATTACK, (index, attack))

    def payload_for(self, event: Event) -> Attack:
        return event.payload[1]
