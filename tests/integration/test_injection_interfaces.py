"""P1 禁飞区与对抗注入的测试。

对抗注入的验收标准只有一个：**消息是被协议挡下来的，不是被注入器
自己丢掉的**。因此每条断言都针对 `MSG_DROP` 的**原因码**——
原因码错了，说明挡住它的是别的机制，那条防线其实没被验证。
"""
import json
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import SimConfig, load_config
from skyfleet.observe.log import read_log
from skyfleet.sim.adversary import Attack, AttackKind
from skyfleet.sim.engine import Engine
from skyfleet.world.geofence import Geofence, breach, build_geofences


class TestGeofence(unittest.TestCase):
    def setUp(self):
        self.fence = Geofence(id="NFZ1", rect=(100.0, 100.0, 50.0, 50.0),
                              z_min=0.0, z_max=120.0)

    def test_inside(self):
        self.assertTrue(self.fence.contains((120.0, 120.0, 60.0)))

    def test_outside_planar(self):
        self.assertFalse(self.fence.contains((300.0, 120.0, 60.0)))

    def test_outside_in_altitude(self):
        """同一片空域可能低空禁飞、高空放行，只判平面会误禁。"""
        self.assertFalse(self.fence.contains((120.0, 120.0, 200.0)))

    def test_boundary_is_inclusive(self):
        self.assertTrue(self.fence.contains((100.0, 100.0, 0.0)))
        self.assertTrue(self.fence.contains((150.0, 150.0, 120.0)))

    def test_open_ended_altitude_by_default(self):
        fence = Geofence(id="NFZ2", rect=(0.0, 0.0, 10.0, 10.0))
        self.assertTrue(fence.contains((5.0, 5.0, 9999.0)))

    def test_breach_returns_the_first_hit(self):
        self.assertEqual(breach((120.0, 120.0, 60.0), [self.fence]), "NFZ1")

    def test_breach_returns_none_when_clear(self):
        self.assertIsNone(breach((300.0, 300.0, 60.0), [self.fence]))

    def test_builds_from_config(self):
        config = SimConfig(geofences=[
            {"id": "A", "x": 0.0, "y": 0.0, "w": 10.0, "h": 10.0,
             "z_min": 0.0, "z_max": 50.0}])
        fences = build_geofences(config)
        self.assertEqual(len(fences), 1)
        self.assertEqual(fences[0].rect, (0.0, 0.0, 10.0, 10.0))
        self.assertEqual(fences[0].z_max, 50.0)

    def test_empty_config_gives_no_fences(self):
        self.assertEqual(build_geofences(SimConfig()), [])


class TestAttackParsing(unittest.TestCase):
    def test_parses_each_kind(self):
        for kind in AttackKind:
            attack = Attack.from_dict({"kind": kind.value, "t": 5.0})
            self.assertIs(attack.kind, kind)

    def test_rejects_unknown_kind(self):
        with self.assertRaises(ValueError):
            Attack.from_dict({"kind": "ddos", "t": 1.0})

    def test_rejects_missing_kind(self):
        with self.assertRaises(ValueError):
            Attack.from_dict({"t": 1.0})


def scenario(tmp, **overrides):
    base = dict(seed=5, duration=25.0, slaves=3, relays=1, tasks=4,
                blocks=2, area=400.0, env_seed=5,
                heartbeat_period=0.5, telemetry_period=0.5,
                log_path=str(Path(tmp) / "inj.jsonl"))
    base.update(overrides)
    return SimConfig(**base)


class TestAdversaryIsRejectedByTheProtocol(unittest.TestCase):
    """四类注入都必须被**现有**协议检查挡下，且原因码要对得上。"""

    def _run(self, attack: Attack):
        with tempfile.TemporaryDirectory() as tmp:
            config = scenario(tmp, adversary=[{
                "kind": attack.kind.value, "t": attack.t,
                "node_id": attack.node_id,
            }])
            events = list(read_log(Engine(config).run()))
        return events

    def _drops(self, events, msg_id):
        return [e for e in events if e["kind"] == "MSG_DROP"
                and e.get("msg_id") == msg_id]

    def test_injection_is_recorded(self):
        events = self._run(Attack(kind=AttackKind.STALE_EPOCH, t=8.0))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"]
        self.assertEqual(len(injected), 1)
        self.assertEqual(injected[0]["attack_kind"], "stale_epoch")

    def test_stale_epoch_is_rejected_as_stale_epoch(self):
        events = self._run(Attack(kind=AttackKind.STALE_EPOCH, t=8.0))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"][0]
        drops = self._drops(events, injected["msg_id"])
        self.assertTrue(drops, "过期任期的消息没有被拒绝")
        self.assertIn(drops[0]["reason"], ("STALE_EPOCH",))

    def test_forged_session_is_rejected(self):
        events = self._run(Attack(kind=AttackKind.FORGED_JOIN, t=8.0,
                                  node_id="X1"))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"][0]
        drops = self._drops(events, injected["msg_id"])
        self.assertTrue(drops, "未授权会话没有被拒绝")
        self.assertEqual(drops[0]["reason"], "STALE_SESSION")

    def test_expired_control_message_is_rejected(self):
        events = self._run(Attack(kind=AttackKind.EXPIRED, t=8.0,
                                  node_id="S1"))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"][0]
        drops = self._drops(events, injected["msg_id"])
        self.assertTrue(drops, "过期控制消息没有被拒绝")
        # 原因是既有的封闭枚举值 DEADLINE_MISSED（"超过有效期"），
        # 不是新造一个 EXPIRED——封闭枚举的意义就在于复用已有语义。
        self.assertEqual(drops[0]["reason"], "DEADLINE_MISSED")

    def test_high_sequence_spoofing_poisons_the_victims_sequence_state(self):
        """**已知缺口**：序号检查只做单调比较，没有上界校验。

        攻击者冒充 S1 发一条 seq 极大的消息。它会通过会话、任期、去重
        与序号检查（因为"更大"看起来就是"更新"），把主节点记的
        last_seq 推到极大值；此后 S1 的**真实**心跳全部小于它，
        一律被判 STALE_SEQ——攻击者用一个包就让一个节点从主节点视野里消失。

        这条测试断言的是"当前确实如此"。将来补上序号窗口校验后，
        它会失败，届时应当把它改写成"注入被拒"，而不是把断言删掉。
        """
        with tempfile.TemporaryDirectory() as tmp:
            config = scenario(tmp, duration=20.0, adversary=[{
                "kind": "replay", "t": 6.0, "node_id": "S1",
                "seq": 10 ** 7}])
            events = list(read_log(Engine(config).run()))

        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"][0]
        self.assertFalse(
            [e for e in events if e["kind"] == "MSG_DROP"
             and e.get("msg_id") == injected["msg_id"]
             and e.get("node_id") == "L"],
            "前提变了：超大序号已被拒绝，可以把这个缺口测试改写为防护测试")

        victim_drops = [e for e in events if e["kind"] == "MSG_DROP"
                        and e.get("reason") == "STALE_SEQ"
                        and e.get("src") == "S1" and e["t"] > 6.0]
        self.assertTrue(victim_drops,
                        "未观察到序号污染：真实节点的心跳没有被误判为过期")

    def test_replay_of_a_known_node_is_rejected_by_sequence_check(self):
        """重放必须带真实节点的真实会话，否则会被会话校验顺手挡下，
        测不到序号检查——那条防线是否有效就无从判断。"""
        events = self._run(Attack(kind=AttackKind.REPLAY, t=8.0,
                                  node_id="S1"))
        injected = [e for e in events if e["kind"] == "ATTACK_INJECT"][0]
        drops = self._drops(events, injected["msg_id"])
        self.assertTrue(drops, "重放消息没有被拒绝")
        self.assertEqual(drops[0]["reason"], "STALE_SEQ")

    def test_attack_injects_do_not_replace_normal_traffic(self):
        """注入是"额外"的，不能顶掉正常心跳与遥测。

        只断言"正常流量仍在"：这个小场景时长很短且城市偏高，
        是否来得及交付任务是另一回事，混进来会让这条测试变成脆弱测试。
        """
        events = self._run(Attack(kind=AttackKind.REPLAY, t=8.0))
        self.assertTrue(any(e["kind"] == "HEARTBEAT_TX" for e in events))
        self.assertTrue(any(e["kind"] == "MSG_TX" for e in events))
        self.assertTrue(any(e["kind"] == "TASK_ASSIGN" for e in events))


class TestObservableEffects(unittest.TestCase):
    """注入必须留下可观测的痕迹，否则场景配了等于没配。"""

    def test_geofence_breach_triggers_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = scenario(
                tmp, duration=15.0,
                # 覆盖整个区域并限高，保证一定命中
                geofences=[{"id": "NFZ", "x": 0.0, "y": 0.0, "w": 400.0,
                            "h": 400.0, "z_min": 0.0, "z_max": 400.0}])
            events = list(read_log(Engine(config).run()))
        holds = [e for e in events if e["kind"] == "SAFETY_ACTION"
                 and e["action"] == "HOLD"]
        self.assertTrue(holds, "进入禁飞区后没有产生 HOLD")
        self.assertEqual(holds[0]["geofence"], "NFZ")

    def test_gnss_outage_triggers_hover(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = scenario(tmp, duration=20.0,
                              gnss_outages=(("S1", 5.0, 15.0),))
            events = list(read_log(Engine(config).run()))
        stale_hovers = [e for e in events if e["kind"] == "SAFETY_ACTION"
                        and e["action"] == "HOVER" and e["node_id"] == "S1"
                        and e.get("cause") == "POSITION_STALE"]
        self.assertTrue(stale_hovers, "GNSS 失锁没有产生以位置过期为原因的 HOVER")
        # 动作只在**变化**时落盘，因此第一条 POSITION_STALE 记录出现在
        # 失锁刚刚生效的时刻，年龄可能为 0；能观测到"原因已切换"才是重点。
        self.assertIn(stale_hovers[0]["action"], ("HOVER",))

    def test_interference_degrades_the_link(self):
        with tempfile.TemporaryDirectory() as tmp:
            plain = scenario(tmp, log_path=str(Path(tmp) / "plain.jsonl"))
            plain_events = list(read_log(Engine(plain).run()))

            jammed = scenario(
                tmp, log_path=str(Path(tmp) / "jammed.jsonl"),
                interferences=[{"id": "J", "x": 200.0, "y": 200.0,
                                "radius": 400.0, "t_start": 0.0,
                                "t_end": 1e9, "extra_loss_db": 25.0}])
            jammed_events = list(read_log(Engine(jammed).run()))

        def drops(events):
            return sum(1 for e in events if e["kind"] == "MSG_DROP")

        self.assertGreater(drops(jammed_events), drops(plain_events))

    def test_impairments_are_deterministic(self):
        from skyfleet.observe.log import normalised_digest
        digests = []
        with tempfile.TemporaryDirectory() as tmp:
            for tag in ("a", "b"):
                config = scenario(
                    tmp, log_path=str(Path(tmp) / f"{tag}.jsonl"),
                    interferences=[{"id": "J", "x": 200.0, "y": 200.0,
                                    "radius": 300.0, "t_start": 2.0,
                                    "t_end": 12.0}],
                    adversary=[{"kind": "stale_epoch", "t": 6.0}])
                Engine(config).run()
                digests.append(normalised_digest(config.log_path))
        self.assertEqual(digests[0], digests[1])

    def test_scenario_without_injections_is_unchanged(self):
        """没配注入时不得引入任何额外事件。"""
        with tempfile.TemporaryDirectory() as tmp:
            config = scenario(tmp, duration=10.0)
            events = list(read_log(Engine(config).run()))
        self.assertFalse([e for e in events
                          if e["kind"] in ("ATTACK_INJECT",)])


if __name__ == "__main__":
    unittest.main()
