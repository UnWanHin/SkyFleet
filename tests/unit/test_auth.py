"""P4：消息认证（MAC）。

规格：docs/superpowers/specs/2026-10-09-p4-message-authentication-spec.md

测试策略：MAC 是**密码学原语**，因此这里既测"正常路径能用"，也测
"篡改必须被发现"——后者才是认证存在的理由。只测前者会写出一个
「签了名但改了字段也验得过」的假实现。
"""
from __future__ import annotations

import unittest

from skyfleet.protocol.auth import (
    MAC_HEX_LEN,
    compute_mac,
    verify_mac,
)
from skyfleet.protocol.message import Msg
from skyfleet.protocol.version import (AcceptDecision, DedupTable,
                                      LocalView, accept)

KEY = "swarm-group-key-1"
OTHER_KEY = "swarm-group-key-2"


def heartbeat(**overrides) -> Msg:
    base = dict(msg_type="HEARTBEAT", src="S1", dst="L", t=1.0, seq=7)
    base.update(overrides)
    return Msg.new(**base)


class TestMacBasics(unittest.TestCase):
    def test_deterministic(self):
        """同一消息、同一密钥必须恒得同一 MAC。

        否则重传的副本会验签失败——而重传是本仿真的核心机制之一。
        """
        a, b = heartbeat(), heartbeat()
        # msg_id 由全局计数器生成，故按字段复制而非重新构造
        from dataclasses import replace
        b = replace(a)
        self.assertEqual(compute_mac(a, KEY), compute_mac(b, KEY))

    def test_length_is_128_bits(self):
        self.assertEqual(len(compute_mac(heartbeat(), KEY)), MAC_HEX_LEN)
        self.assertEqual(MAC_HEX_LEN, 32)

    def test_different_keys_give_different_macs(self):
        msg = heartbeat()
        self.assertNotEqual(compute_mac(msg, KEY), compute_mac(msg, OTHER_KEY))

    def test_verify_accepts_a_fresh_mac(self):
        msg = heartbeat()
        self.assertTrue(verify_mac(msg, KEY, compute_mac(msg, KEY)))

    def test_verify_rejects_a_wrong_mac(self):
        self.assertFalse(verify_mac(heartbeat(), KEY, "0" * MAC_HEX_LEN))

    def test_verify_rejects_an_empty_mac(self):
        """P0 占位（空串）必须验不过——否则认证形同虚设。"""
        self.assertFalse(verify_mac(heartbeat(), KEY, ""))


class TestTamperingIsDetected(unittest.TestCase):
    """★ 逐个字段篡改都必须被发现。这是认证存在的唯一理由。"""

    def _signed(self):
        msg = heartbeat(operation_id="op-1", session_id="S1#1",
                        leader_epoch=2, mission_version=3, route_version=1)
        return msg, compute_mac(msg, KEY)

    def test_src_tampering_is_detected(self):
        """`src` 必须在覆盖范围内。

        否则冒充者换掉 src 而 MAC 依然有效，认证就完全没防住冒充——
        而那正是本切片要解决的问题。
        """
        from dataclasses import replace
        msg, mac = self._signed()
        forged = replace(msg, src="S9")
        self.assertFalse(verify_mac(forged, KEY, mac),
                         "篡改 src 后 MAC 仍然有效——认证没有防住冒充")

    def test_every_covered_field_is_detected(self):
        from dataclasses import replace
        msg, mac = self._signed()
        for field, value in (("msg_type", "TELEMETRY"),
                             ("dst", "S3"),
                             ("session_id", "S1#2"),
                             ("leader_epoch", 99),
                             ("mission_version", 99),
                             ("route_version", 99),
                             ("seq", 12345),
                             ("operation_id", "op-2"),
                             ("t_created", 42.0)):
            with self.subTest(field=field):
                forged = replace(msg, **{field: value})
                self.assertFalse(verify_mac(forged, KEY, mac),
                                 f"篡改 {field} 后 MAC 仍然有效")

    def test_payload_tampering_is_detected(self):
        from dataclasses import replace
        msg, mac = self._signed()
        forged = replace(msg, payload={"task_id": "T999", "evil": True})
        self.assertFalse(verify_mac(forged, KEY, mac),
                         "篡改 payload 后 MAC 仍然有效")

    def test_hop_and_ttl_are_not_covered(self):
        """★ `hop`/`ttl` **不得**纳入覆盖范围。

        中继转发时必然修改它们（`replace(msg, hop=..., ttl=...)`）。
        若纳入，每一跳都会验签失败——认证会把多跳转发**全部杀掉**，
        而症状是"中继一转发就丢包"，看起来像链路问题。
        """
        from dataclasses import replace
        msg, mac = self._signed()
        forwarded = replace(msg, hop=msg.hop + 1, ttl=msg.ttl - 1)
        self.assertTrue(verify_mac(forwarded, KEY, mac),
                        "hop/ttl 改变了就对不上 MAC——多跳转发会被误杀")

    def test_checksum_is_not_covered(self):
        """`checksum` 自身不得参与计算，否则永远无法验证。"""
        from dataclasses import replace
        msg, mac = self._signed()
        with_mac = replace(msg, checksum=mac)
        self.assertEqual(compute_mac(with_mac, KEY), mac)


class TestMacIsKeyedNotPlainHash(unittest.TestCase):
    def test_a_plain_digest_of_the_fields_would_not_verify(self):
        """确认用的是**带密钥**的 MAC，而不是字段的裸哈希。

        裸哈希谁都能算，等于没有认证；而它照样能通过"能验签"这类
        表面测试。
        """
        import hashlib
        import json
        from dataclasses import asdict
        msg = heartbeat()
        blob = json.dumps(asdict(msg), sort_keys=True, default=str)
        plain = hashlib.sha256(blob.encode()).hexdigest()[:MAC_HEX_LEN]
        self.assertNotEqual(compute_mac(msg, KEY), plain)
        self.assertFalse(verify_mac(msg, KEY, plain))


def _signed(msg: Msg, key: str = KEY) -> Msg:
    from dataclasses import replace
    return replace(msg, checksum=compute_mac(msg, key))


class TestFirstArrivingSpoofer(unittest.TestCase):
    """★ 本切片要关闭的残差：**先到的**冒充者。

    P2 切片 A 的滑动窗口解决的是「基准已存在」的情形：被冒充的节点一直在
    发消息，接收端有 `last_seq` 可比，冒充者那个 10^7 会越窗被拒。

    但若接收端**从未见过**该 `(src, msg_type)` 流，就没有基准，10^7 会被
    当作锚点收下，此后真实节点的正常序号全部落在窗口**后面**被判
    `STALE_SEQ`——单封包就能让一个真实节点从视野里消失。

    这不是滑动窗口的参数问题，是「无认证」的本质边界：任何基于序号、
    会话、任期的检查都建立在「消息确实来自它声称的发送者」之上。
    """

    HUGE = 10 ** 7

    def _fresh(self):
        return (LocalView(), DedupTable(ttl=10.0, capacity=100))

    def test_without_authentication_the_spoofer_wins(self):
        """残差的**正面复现**：无认证时冒充者成功锚定，真实节点被打瞎。

        这条是下面那条的对照。没有它，"认证后防住了"可能只是因为那股
        攻击本来就没生效。
        """
        view, dedup = self._fresh()
        self.assertIs(accept(heartbeat(seq=self.HUGE), view, dedup, 0.0),
                      AcceptDecision.ACCEPT,
                      "无基准时本应无法拒绝——若这里已变成越窗，说明默认"
                      "行为被改了，golden digest 也会变")
        self.assertIs(accept(heartbeat(seq=1), view, dedup, 1.0),
                      AcceptDecision.STALE_SEQ,
                      "先到的冒充者本应能打瞎接收端——这是 P2 的已知残差")

    def test_authentication_rejects_the_spoofer(self):
        """★ 认证开启后，冒充者（无密钥）被判 `BAD_AUTH`。"""
        view, dedup = self._fresh()
        self.assertIs(
            accept(heartbeat(seq=self.HUGE), view, dedup, 0.0, auth_key=KEY),
            AcceptDecision.BAD_AUTH,
            "未签名的冒充者没有被认证拒绝——残差没有关闭")

    def test_rejection_does_not_poison_the_sequence_state(self):
        """★ 残差的**功能**验证：拒绝之后，真实节点仍须被正常接受。

        这一条才是"关掉残差"的本体。只断言"多了一个 BAD_AUTH 原因码"
        是不够的——真正的危害是接收端的序号状态被污染。
        """
        view, dedup = self._fresh()
        self.assertIs(
            accept(heartbeat(seq=self.HUGE), view, dedup, 0.0, auth_key=KEY),
            AcceptDecision.BAD_AUTH)
        # 真实节点随后到达，必须一切正常
        self.assertIs(accept(_signed(heartbeat(seq=1)), view, dedup, 1.0,
                             auth_key=KEY),
                      AcceptDecision.ACCEPT,
                      "认证拒绝冒充者之后，真实消息仍被误判——序号状态被污染")
        self.assertIs(accept(_signed(heartbeat(seq=2)), view, dedup, 2.0,
                             auth_key=KEY),
                      AcceptDecision.ACCEPT)

    def test_an_insider_with_the_key_is_still_not_defended(self):
        """★ 组密钥的**固有边界**：持有密钥的内部成员仍能锚定。

        如实固定，不假装认证解决了一切。选组密钥而非非对称签名是一个
        有代价的取舍（见规格 §2），代价必须被记录，否则将来有人会以为
        "开了认证就安全了"。
        """
        view, dedup = self._fresh()
        self.assertIs(
            accept(_signed(heartbeat(seq=self.HUGE)), view, dedup, 0.0,
                   auth_key=KEY),
            AcceptDecision.ACCEPT,
            "内部人签名有效，认证**不应**拦它——若这里被拒，说明实现把"
            "认证当成了授权，那会掩盖「缺少成员管理」这个真实缺口")
        self.assertIs(accept(_signed(heartbeat(seq=1)), view, dedup, 1.0,
                             auth_key=KEY),
                      AcceptDecision.STALE_SEQ,
                      "内部人仍能打瞎接收端——这是组密钥方案的固有边界")


if __name__ == "__main__":
    unittest.main()
