"""事件类型与丢弃原因码。

两者都是**封闭枚举**：日志中不得出现自由文本原因，新增原因必须改枚举。
这对应报告"消息…丢弃、重复和过期原因"的记录要求。
"""
from __future__ import annotations

import enum


class DropReason(str, enum.Enum):
    """消息被丢弃的原因。共 13 种。

    必须与实际写入日志的原因码一一对应：曾经 `STALE_SEQ` 会被
    `accept()` 返回并当作原因写进日志，却不在本枚举里——"封闭枚举"
    这条约束在代码层面是假的，只有测试才能发现。
    """

    OCCLUDED = "OCCLUDED"                  # 几何遮挡导致链路不可用
    COLLISION = "COLLISION"                # 共享信道竞争冲突
    QUEUE_FULL = "QUEUE_FULL"              # 本地发送队列溢出
    TTL_EXPIRED = "TTL_EXPIRED"            # 跳数或生存时间耗尽
    DEADLINE_MISSED = "DEADLINE_MISSED"    # 超过有效期，已无重传价值
    DEDUP = "DEDUP"                        # operation_id 命中去重表
    STALE_EPOCH = "STALE_EPOCH"            # 来自旧任期
    STALE_SESSION = "STALE_SESSION"        # 来自旧会话（重连前）
    STALE_VERSION = "STALE_VERSION"        # 旧 mission/route 版本
    STALE_SEQ = "STALE_SEQ"                # 序号回退（重放或乱序）
    NO_ROUTE = "NO_ROUTE"                  # 当前通信关系不可达
    RATE_LIMITED = "RATE_LIMITED"          # 高优先级配额耗尽
    UNAUTHORIZED = "UNAUTHORIZED"          # 权限校验失败（P1 注入用）


class EventKind(str, enum.Enum):
    """写入事件日志的事件类型。"""

    #: 世界描述（t=0）。有了它，一份日志就能**单独**重建全部画面，
    #: 不需要额外携带场景配置文件。
    SCENARIO_INIT = "SCENARIO_INIT"
    #: P1 对抗注入：记录"投了什么"，用于与随后的 MSG_DROP 对账
    ATTACK_INJECT = "ATTACK_INJECT"
    NODE_JOIN = "NODE_JOIN"
    NODE_LEAVE = "NODE_LEAVE"
    NODE_REJOIN = "NODE_REJOIN"
    ROLE_CHANGE = "ROLE_CHANGE"
    STATUS_CHANGE = "STATUS_CHANGE"
    HEARTBEAT_TX = "HEARTBEAT_TX"
    #: 电量耗尽。仅在 on_battery_empty != "none" 时产生——默认策略下
    #: 电量归零没有任何后果，也就没有这个事件。
    BATTERY_EMPTY = "BATTERY_EMPTY"
    HEARTBEAT_MISS = "HEARTBEAT_MISS"
    MSG_TX = "MSG_TX"
    MSG_RX = "MSG_RX"
    MSG_FORWARD = "MSG_FORWARD"
    MSG_DROP = "MSG_DROP"
    MSG_DUP = "MSG_DUP"
    MSG_EXPIRE = "MSG_EXPIRE"
    CHANNEL_BUSY = "CHANNEL_BUSY"
    OCCLUSION_ENTER = "OCCLUSION_ENTER"
    OCCLUSION_EXIT = "OCCLUSION_EXIT"
    TASK_CREATE = "TASK_CREATE"
    TASK_ASSIGN = "TASK_ASSIGN"
    TASK_PICKUP = "TASK_PICKUP"
    TASK_DELIVER = "TASK_DELIVER"
    TASK_FAIL = "TASK_FAIL"
    TASK_REASSIGN = "TASK_REASSIGN"
    SAFETY_ACTION = "SAFETY_ACTION"
    QUEUE_SAMPLE = "QUEUE_SAMPLE"
    ENERGY_SAMPLE = "ENERGY_SAMPLE"
