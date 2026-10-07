"""消息认证：HMAC-SHA256 + 全网预共享组密钥。

为什么需要它（P2 切片 A 留下的残差）：序号滑动窗口建立在
「消息确实来自它声称的发送者」之上。没有认证，这个前提不成立——
若接收端**从未见过**某条 `(src, msg_type)` 流，冒充者那个 10^7 的
大序号会被当作锚点收下，此后真实节点的正常序号全部落在窗口后面被判
`STALE_SEQ`。这不是滑动窗口的参数问题，是「无认证」的本质边界。

设计取舍见 `docs/superpowers/specs/2026-10-09-p4-message-authentication-spec.md`：

* 选 HMAC + 组密钥而非非对称签名：场景是同一集群内网，预共享组密钥是
  现实做法（MAVLink 2 的签名、组播组密钥同理），且纯标准库实现。
* **明确不防**内部成员伪造：持有组密钥即可伪造。这是组密钥方案的固有
  边界，如实记录，不假装认证解决了一切。
* **不防重放**：重放的消息 MAC 有效。防重放是序号窗口与去重表的职责
  ——两套机制各司其职，不互相越俎代庖。
"""
from __future__ import annotations

import hashlib
import hmac
import json
from typing import Any

from skyfleet.protocol.message import Msg

#: MAC 截断后的十六进制长度。128 位：伪造成功概率 2^-128，对仿真足够，
#: 且比全长 SHA256 更接近嵌入式链路上真实的带宽约束。
MAC_HEX_LEN = 32

#: 参与 MAC 计算的字段。
#:
#: `src` **必须**在内——否则冒充者换掉 src 而 MAC 依然有效，认证就完全
#: 没防住冒充，而那正是本模块存在的理由。
#:
#: `hop`/`ttl` **必须排除**：中继转发时用 `replace` 修改它们，纳入会让
#: 每一跳都验签失败，症状是「中继一转发就丢包」，看起来像链路问题。
#: `checksum` 自身当然不能在内，否则永远无法验证。
#: `deadline`/`qos`/`reliability` 由 `Msg.new` 按消息类型推导，不是
#: 独立信息，纳入只是增加计算量。
_MAC_FIELDS = (
    "msg_id", "msg_type", "src", "dst",
    "session_id", "leader_epoch", "mission_version", "route_version",
    "seq", "operation_id", "t_created", "payload",
)


def _canonical(msg: Msg) -> bytes:
    """把受保护的字段序列化成**规范**字节串。

    用 `json.dumps(sort_keys=True, separators=...)` 而不是 `repr(dict)`：
    后者的键序随插入序变化，同一条消息在两次构造后可能得到不同字节串，
    从而算出不同 MAC——而症状是"重传的副本偶尔验签失败"，极难排查。
    """
    payload: dict[str, Any] = {name: getattr(msg, name)
                               for name in _MAC_FIELDS}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"),
                      default=str).encode("utf-8")


def compute_mac(msg: Msg, key: str) -> str:
    """计算消息的消息认证码（十六进制，截断到 `MAC_HEX_LEN`）。"""
    if not isinstance(key, str):
        raise TypeError(f"认证密钥必须是字符串，得到 {type(key).__name__}")
    digest = hmac.new(key.encode("utf-8"), _canonical(msg),
                      hashlib.sha256).hexdigest()
    return digest[:MAC_HEX_LEN]


def verify_mac(msg: Msg, key: str, mac: str) -> bool:
    """常量时间比对。

    用 `hmac.compare_digest` 而不是 `==`：后者在首个不同字节处提前返回，
    比对耗时随前缀匹配长度变化，构成时序侧信道。对仿真而言这近乎吹毛
    求疵，但代价为零，且能避免读者以为项目不知道这件事。
    """
    if not mac:
        return False                    # P0 占位（空串）一律不通过
    return hmac.compare_digest(compute_mac(msg, key), mac)
