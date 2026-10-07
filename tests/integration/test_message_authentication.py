"""P4：消息认证的端到端验证。

规格：docs/superpowers/specs/2026-10-09-p4-message-authentication-spec.md

本文件要证明的是**残差被关闭**，而不只是"认证代码能跑"：

| 情形 | 认证关闭 | 认证开启 |
|---|---|---|
| 外部伪造（无密钥） | 需要靠序号/会话等检查，先到的冒充者防不住 | **在认证处即被拒** |
| 抓包重放（MAC 有效） | 序号/去重 | 序号/去重（认证**不越俎代庖**）|
| 内部成员伪造 | 无法防 | **不防**（组密钥固有边界，如实记录）|

因此每条断言都针对 `MSG_DROP` 的**原因码**：原因码错了，说明挡住它的
是别的机制，那条防线其实没被验证——这是本项目对抗日志的一贯要求。
"""
from __future__ import annotations

import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.observe.log import read_log
from skyfleet.protocol.auth import compute_mac
from skyfleet.protocol.version import AcceptDecision
from skyfleet.sim.engine import Engine

SCENARIO = Path("skyfleet/scenarios/baseline_small.json")
KEY = "swarm-group-key-for-tests"

#: 冒充者使用的超大序号。接收端若**从未见过**该流，就没有基准判断它
#: 是否离谱——这正是 P2 切片 A 留下的残差。
HUGE_SEQ = 10 ** 7


def scenario(tmp: str, *, auth: bool, name: str, **overrides):
    """按是否启用认证构造配置。"""
    return replace(load_config(SCENARIO),
                   log_path=str(Path(tmp) / f"{name}.jsonl"),
                   auth_enabled=auth,
                   auth_key=KEY if auth else "",
                   **overrides)


def drops(events, reason: str) -> list[dict]:
    return [e for e in events
            if e["kind"] in ("MSG_DROP", "MSG_DUP") and e.get("reason") == reason]


class TestConfigValidation(unittest.TestCase):
    def test_enabling_auth_without_a_key_is_rejected(self):
        """开启认证却没有密钥，必须当场报错。

        这是最危险的组合：它会拿空密钥去比对，所有消息都验不过，集群
        静默瘫痪，而配置看起来"开了认证"（像是更安全了）。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "bad.jsonl"),
                          auth_enabled=True, auth_key="")
            with self.assertRaises(ValueError) as ctx:
                Engine(cfg)
        self.assertIn("auth_key", str(ctx.exception))

    def test_auth_is_off_by_default(self):
        """默认关闭：新机制默认不改变既有行为，golden digest 才守得住。"""
        cfg = load_config(SCENARIO)
        self.assertFalse(cfg.auth_enabled)
        self.assertEqual(cfg.auth_key, "")


class TestNormalTrafficSurvivesAuthentication(unittest.TestCase):
    def test_a_full_scenario_runs_with_auth_enabled(self):
        """★ 认证开启后，正常节点的消息**不得被误拒**。

        只测"攻击被挡"会漏掉一整类失效：把密钥用错、把 MAC 算错、
        或转发时重签，症状都是"攻击被挡住了"——因为**所有**消息都被
        挡住了。这条测试与下一条配对，缺一不可。
        """
        from skyfleet.observe.metrics import compute_metrics
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="ok", duration=120.0)
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
            # 指标必须在**临时目录存活期间**读——日志随目录一起消失，
            # 在 with 外面读会得到 FileNotFoundError，看起来像"认证把日志
            # 弄坏了"，其实只是测试写错了位置。
            metrics = compute_metrics(cfg.log_path)
        self.assertEqual(drops(events, "BAD_AUTH"), [],
                         "认证开启后正常消息被判 BAD_AUTH")
        self.assertTrue([e for e in events if e["kind"] == "MSG_TX"],
                        "没有任何发送")
        # 任务必须真的在推进，而不是"消息都在但什么都不发生"
        self.assertGreater(metrics["tasks"]["completion_rate"], 0.0,
                           "认证开启后任务完成率为 0——消息被静默吞掉了")

    def test_multi_hop_forwarding_survives_authentication(self):
        """★ 中继转发必须仍能通过认证。

        `hop`/`ttl` 在每一跳都会变。若它们被纳入 MAC 覆盖范围，每一跳
        都会验签失败——症状是"中继一转发就丢包"，看起来像链路问题。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="hop", duration=120.0)
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        forwarded = [e for e in events
                     if e["kind"] == "MSG_TX" and (e.get("hop") or 0) > 0]
        self.assertTrue(forwarded, "没有任何多跳转发，本测试会空转")
        # 转发过的消息不得因认证被拒
        self.assertEqual(drops(events, "BAD_AUTH"), [],
                         "多跳转发被认证误杀（hop/ttl 不应参与 MAC）")


class TestForgeryIsRejected(unittest.TestCase):
    def test_unsigned_forgery_is_rejected_as_bad_auth(self):
        """★ 外部伪造（无密钥）必须在认证处被拒。"""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="forge", duration=20.0,
                            adversary=[{"kind": "forged_join", "t": 6.0,
                                        "node_id": "S9"}])
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"]
        self.assertTrue(injected, "注入没有发生，本测试会空转")
        self.assertTrue(drops(events, "BAD_AUTH"),
                        "未签名的伪造没有被判 BAD_AUTH——认证没有生效")

    def test_a_tampered_source_is_rejected(self):
        """★ 篡改 `src` 必须被发现——那正是"冒充"的定义。

        若 `src` 不在 MAC 覆盖范围内，冒充者换掉 src 而 MAC 依然有效，
        认证就完全没防住冒充，而那正是本切片要解决的问题。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="tamper", duration=20.0,
                            adversary=[{"kind": "forged_join", "t": 6.0,
                                        "node_id": "S1"}])
            Engine(cfg).run()
        # 注入器构造的是**无** MAC 的消息；这里另外验证"带 MAC 但改了 src"
        # 也会被拒（单元测试已逐字段覆盖，此处是端到端的呼应）。
        from skyfleet.protocol.message import Msg
        from skyfleet.protocol.auth import verify_mac
        msg = Msg.new("HEARTBEAT", src="S1", dst="L", t=1.0, seq=1)
        mac = compute_mac(msg, KEY)
        self.assertTrue(verify_mac(msg, KEY, mac))
        self.assertFalse(verify_mac(replace(msg, src="S2"), KEY, mac))


class TestResidualGapIsClosed(unittest.TestCase):
    """★ 本切片的核心：无认证时防不住的「先到的冒充者」，认证后防得住。

    **为什么不在整合层面复现"锚定"**：残差的前提是「接收端从未见过该
    `(src, msg_type)` 流」。用 `forged_join` 注入时它带伪造会话，会先被
    会话检查挡下、**根本走不到序号逻辑**；用 `replay` 注入时接收端通常
    已有基准，会被判越窗而非锚定。两种情形都无法在整场景里稳定复现
    "锚定"，硬凑的断言会因"根本没走到那条分支"而**空转**。
    「拒绝后不污染序号状态」因此在 `accept()` 层面精确验证
    （tests/unit/test_auth.py 的 TestFirstArrivingSpoofer）。

    这里只断言整合层能可靠验证的部分：注入确实被判 `BAD_AUTH`——证明
    **接线是通的**（签名点、密钥传递、校验位置都对）。
    """

    def test_unsigned_injection_is_rejected_as_bad_auth(self):
        """★ 认证开启后，无密钥的注入在**认证处**即被拒。

        原因码必须是 `BAD_AUTH`：若报别的码，说明挡住它的是既有检查
        （会话/任期），认证**没有被验证到**——这与"攻击被挡"是两回事。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="gap_on", duration=20.0,
                            adversary=[{"kind": "forged_join", "t": 6.0,
                                        "node_id": "S9", "seq": HUGE_SEQ}])
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        injected = {e["msg_id"] for e in events if e["kind"] == "ATTACK_INJECT"}
        self.assertTrue(injected, "注入没有发生，本测试会空转")
        bad = drops(events, "BAD_AUTH")
        self.assertTrue(bad, "认证开启后注入没有被判 BAD_AUTH——接线没通")
        self.assertTrue(injected & {e.get("msg_id") for e in bad},
                        "BAD_AUTH 不是那条注入产生的——挡下它的是别的原因，"
                        "认证没有被验证到")

    def test_without_authentication_the_same_injection_is_not_bad_auth(self):
        """对照组：认证关闭时同一注入**不得**出现 BAD_AUTH。

        没有这条，"认证后出现了 BAD_AUTH"可能只说明这个原因码总是出现。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=False, name="gap_off", duration=20.0,
                            adversary=[{"kind": "forged_join", "t": 6.0,
                                        "node_id": "S9", "seq": HUGE_SEQ}])
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        self.assertTrue([e for e in events if e["kind"] == "ATTACK_INJECT"],
                        "注入没有发生，本测试会空转")
        self.assertEqual(drops(events, "BAD_AUTH"), [],
                         "认证关闭时不应出现 BAD_AUTH")


class TestNoStateMutationBeforeAuthentication(unittest.TestCase):
    """★ 认证必须在**任何改动接收端状态的检查**之前，而不只是在 accept() 内。

    这是实现 P4 时被探针发现的缺陷：`accept()` 内确实把认证放在最前，但
    `NodeAgent.receive()` 在调用 `accept()` **之前**还有两道检查：

      1. `seen_msg_ids.seen(msg_id)` —— 去重表，**检查时即登记**；
      2. `deadline` 过期检查。

    于是未认证的消息**在被拒绝之前就已经改动了接收端状态**。可利用的
    攻击：`msg_id` 是每节点从 1 递增的计数器（`new_msg_id()` 返回
    `f"{node_id}:{n}"`），**完全可预测**。攻击者抢先用某个即将出现的
    msg_id 发一条伪造消息，去重表就把它登记下来；随后**真实**节点用同一
    个 msg_id 发出的消息会被判 DEDUP 丢弃。

    认证开着也挡不住——因为认证根本还没跑。这正是「未经认证的消息，其
    所有字段都不可信」这句话的完整含义：`msg_id` 也是字段之一。
    """

    def _leader_with_log(self, tmp, auth):
        cfg = scenario(tmp, auth=auth, name="squat", duration=30.0)
        engine = Engine(cfg)
        engine.build()
        engine._open_log()
        return cfg, engine

    def _forged(self, msg_id, dst):
        from skyfleet.protocol.message import Msg, Qos, Rel
        return Msg.new("HEARTBEAT", src="S9", dst=dst, t=1.0,
                       msg_id=msg_id, seq=999_999,
                       session_id="S9#forged", payload={"squat": True},
                       qos=Qos.Q1, reliability=Rel.R2)

    def _legit(self, msg_id, src, session_id, key=None):
        from skyfleet.protocol.message import Msg, Qos, Rel
        msg = Msg.new("HEARTBEAT", src=src, dst="S1", t=2.0,
                      msg_id=msg_id, seq=1, session_id=session_id,
                      payload={"pos": [0, 0, 0], "energy": 100.0,
                               "nearby": []},
                      qos=Qos.Q1, reliability=Rel.R2)
        if key:
            from dataclasses import replace as _replace
            msg = _replace(msg, checksum=compute_mac(msg, key))
        return msg

    def _squat(self, tmp, auth):
        """跑一次预占，返回（被预占的 msg_id, 真实消息的判定）。"""
        cfg, engine = self._leader_with_log(tmp, auth)
        try:
            leader = engine.agents[cfg.leader_id]
            # 攻击者预测 leader 的**下一个** msg_id
            predicted = f"{cfg.leader_id}:{leader._msg_seq + 1}"
            leader.receive(self._forged(predicted, cfg.leader_id), 1.0)
            decision = leader.receive(
                self._legit(predicted, cfg.leader_id, leader.session_id,
                            KEY if auth else None), 2.0)
        finally:
            engine.log.close()
        return predicted, decision

    def test_without_authentication_the_squat_succeeds(self):
        """残差的**正面复现**：无认证时预占必然成功。

        没有认证就无法分辨伪造与真实，去重表登记谁都一样——这不是实现
        缺陷，是「无认证」的必然结果。本测试把它固定下来，同时作为下面
        那条的对照：否则"认证后挡住了"可能只是因为攻击本来就没生效。
        """
        with tempfile.TemporaryDirectory() as tmp:
            predicted, decision = self._squat(tmp, auth=False)
        self.assertIs(decision, AcceptDecision.DEDUP,
                      f"无认证时预占 {predicted} 本应成功（真实消息被误杀）")

    def test_authentication_prevents_the_squat(self):
        """★ 认证开启后，未认证的伪造**不得**占用 msg_id。

        关键是伪造必须在**去重表登记之前**就被拒——若认证仍在 `accept()`
        里（去重之后），伪造会被拒但 msg_id 已被登记，真实消息照样被误杀。
        """
        with tempfile.TemporaryDirectory() as tmp:
            predicted, decision = self._squat(tmp, auth=True)
        self.assertIsNot(
            decision, AcceptDecision.DEDUP,
            f"认证开启后，未认证的伪造仍占用了 msg_id {predicted}，真实消息"
            f"被误判为 DEDUP——认证没有排在 receive() 的状态改动之前")
        self.assertIs(decision, AcceptDecision.ACCEPT,
                      "真实消息应当被正常接受")


class TestAuthenticationDoesNotOverreach(unittest.TestCase):
    def test_a_signed_replay_is_caught_by_sequence_not_by_auth(self):
        """★ 认证**不防重放**，这是刻意的分工。

        抓包重放的消息 MAC 有效，认证会放行；挡下它的是序号窗口与去重。
        若这里报 `BAD_AUTH`，说明签名实现把"重放"也算成了伪造——那样
        序号窗口的意义就被架空了，两个机制会互相掩盖失效。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = scenario(tmp, auth=True, name="replay", duration=20.0,
                            adversary=[{"kind": "replay", "t": 6.0,
                                        "node_id": "S1", "seq": 1,
                                        "signed": True}])
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"]
        self.assertTrue(injected, "注入没有发生，本测试会空转")
        # 带有效 MAC 的重放**不应**被判认证失败
        self.assertEqual(
            [e for e in drops(events, "BAD_AUTH")
             if e.get("msg_id") in {i["msg_id"] for i in injected}], [],
            "带有效 MAC 的重放被判 BAD_AUTH——认证越界了，"
            "重放本该由序号/去重负责")
        # 且它确实被挡下了（由别的机制），否则说明它被接受了
        blocked = [e for e in events
                   if e["kind"] in ("MSG_DROP", "MSG_DUP")
                   and e.get("msg_id") in {i["msg_id"] for i in injected}]
        self.assertTrue(blocked, "带有效 MAC 的重放既没被认证拒、也没被其他"
                                 "机制拒——它被接受了")

    def test_signed_replay_passes_the_auth_check_itself(self):
        """直接验证签名层：MAC 有效的消息 `verify_mac` 必须为真。

        避免上一条测试因"注入根本没走到认证"而假通过。
        """
        from skyfleet.protocol.message import Msg
        from skyfleet.protocol.auth import verify_mac
        msg = Msg.new("HEARTBEAT", src="S1", dst="L", t=1.0, seq=1)
        self.assertTrue(verify_mac(msg, KEY, compute_mac(msg, KEY)),
                        "MAC 有效的消息验签失败——认证本身有问题")


if __name__ == "__main__":
    unittest.main()
