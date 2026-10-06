"""日志不变量：封闭枚举必须在**实际写出的日志**上成立。

这些断言看起来多余（枚举本来就是枚举），但它们抓到过一个真实缺口：
`accept()` 会返回 `STALE_SEQ` 并被当作丢弃原因写进日志，而
`DropReason` 里根本没有这个值——"封闭枚举"在代码层面是假的，
只有把日志与枚举对一遍才会发现。

新增任何原因码、动作名、注入类型时，这里会自动把它们纳入检查。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.events import DropReason, EventKind
from skyfleet.observe.log import read_log
from skyfleet.sim.adversary import AttackKind
from skyfleet.sim.engine import Engine

#: 安全动作与触发原因的封闭集合。刻意写死而非从源码推导：
#: 推导出来的集合永远与源码一致，也就永远测不出问题。
SAFETY_ACTIONS = frozenset({"NONE", "HOVER", "RETURN_HOME", "LAND", "HOLD"})
SAFETY_CAUSES = frozenset({"NONE", "GEOFENCE", "LOW_BATTERY",
                           "POSITION_STALE", "LINK_LOST"})

#: `cause` 字段并非 SAFETY_ACTION 专用：节点离网/重新入网也用它说明缘由。
#: 因此必须**按事件类型**分别约束，否则一条"未知原因"的断言会把
#: 完全正常的 SILENT/REVIVE 判成错误，或者反过来放过真正的越界值。
CAUSE_SETS = {
    "SAFETY_ACTION": SAFETY_CAUSES,
    "NODE_LEAVE": frozenset({"SILENT", "BATTERY"}),
    "NODE_JOIN": frozenset({"REVIVE"}),
}


def busy_scenario(tmp, name, **overrides):
    """一个把各类注入全部打开的短场景。"""
    base = dict(
        seed=7, duration=40.0, slaves=4, relays=1, tasks=6, blocks=3,
        area=400.0, env_seed=7, heartbeat_period=0.4, telemetry_period=0.4,
        log_path=str(Path(tmp) / f"{name}.jsonl"),
        # 禁飞区必须真的罩住站位：节点布在半径 area*0.35 的环上，
        # 只在角落放一小块区域是罩不住的，那样"禁飞区没生效"的结论
        # 其实来自场景配错，而不是功能有问题。
        geofences=[{"id": "NFZ", "x": 40.0, "y": 40.0, "w": 320.0,
                    "h": 320.0, "z_min": 0.0, "z_max": 400.0}],
        gnss_outages=(("S1", 5.0, 12.0),),
        interferences=[{"id": "J", "x": 300.0, "y": 300.0, "radius": 150.0,
                        "t_start": 10.0, "t_end": 25.0,
                        "extra_loss_db": 12.0}],
        obstacles=[{"id": "T", "x": 190.0, "y": 100.0, "w": 20.0, "h": 40.0,
                    "height": 150.0, "t_start": 8.0, "t_end": 20.0}],
        kills=(("S2", 15.0),),
        revives=(("S2", 30.0),),
        adversary=[
            {"kind": "replay", "t": 6.0, "node_id": "S1"},
            {"kind": "stale_epoch", "t": 9.0, "node_id": "S1"},
            {"kind": "expired", "t": 12.0, "node_id": "S1"},
            {"kind": "forged_join", "t": 15.0, "node_id": "X1"},
        ],
    )
    base.update(overrides)
    return SimConfig(**base)


class TestLogInvariants(unittest.TestCase):
    """默认配置下的日志不变量。"""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        config = busy_scenario(cls.tmp.name, "invariants")
        cls.events = list(read_log(Engine(config).run()))
        assert cls.events, "场景没有产生任何事件，后面的断言会全部空转"

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_event_kind_is_known(self):
        known = {k.value for k in EventKind}
        unknown = sorted({e["kind"] for e in self.events} - known)
        self.assertEqual(unknown, [], f"日志里出现了未登记的事件类型：{unknown}")

    def test_every_drop_reason_is_in_the_closed_enum(self):
        known = {r.value for r in DropReason}
        seen = {e["reason"] for e in self.events if e.get("reason")}
        unknown = sorted(seen - known)
        self.assertEqual(unknown, [], f"丢弃原因不在封闭枚举内：{unknown}")

    def test_every_safety_action_is_known(self):
        actions = {e["action"] for e in self.events if "action" in e}
        self.assertTrue(actions <= SAFETY_ACTIONS,
                        f"未知安全动作：{sorted(actions - SAFETY_ACTIONS)}")

    def test_every_cause_is_known_for_its_event_kind(self):
        for kind, allowed in CAUSE_SETS.items():
            seen = {e["cause"] for e in self.events
                    if e["kind"] == kind and "cause" in e}
            self.assertTrue(seen <= allowed,
                            f"{kind} 出现未知 cause：{sorted(seen - allowed)}")

    def test_cause_is_never_written_without_a_known_set(self):
        """新增一类带 cause 的事件时必须同步登记，否则这条会亮红。"""
        carriers = {e["kind"] for e in self.events if "cause" in e}
        self.assertTrue(carriers <= set(CAUSE_SETS),
                        f"这些事件写了 cause 但没有登记封闭集合："
                        f"{sorted(carriers - set(CAUSE_SETS))}")

    def test_every_attack_kind_is_known(self):
        known = {k.value for k in AttackKind}
        seen = {e["attack_kind"] for e in self.events if "attack_kind" in e}
        self.assertEqual(sorted(seen - known), [])

    def test_all_four_attacks_were_injected(self):
        kinds = {e["attack_kind"] for e in self.events
                 if e["kind"] == "ATTACK_INJECT"}
        self.assertEqual(kinds, {k.value for k in AttackKind})

    def test_faults_and_revival_both_happened(self):
        # 击落与复活分别记为 NODE_LEAVE(cause=SILENT) 与 NODE_JOIN(cause=REVIVE)，
        # 没有独立的 NODE_KILL/NODE_REVIVE 事件类型。
        self.assertTrue(any(e["kind"] == "NODE_LEAVE" and e.get("cause") == "SILENT"
                            for e in self.events), "注入的击落没有产生 NODE_LEAVE")
        self.assertTrue(any(e["kind"] == "NODE_JOIN" and e.get("cause") == "REVIVE"
                            for e in self.events), "复活没有产生 NODE_JOIN")

    def test_log_is_chronological(self):
        times = [e["t"] for e in self.events]
        self.assertEqual(times, sorted(times), "事件日志不是按时间递增的")

    def test_geofence_was_observed(self):
        causes = {e["cause"] for e in self.events if "cause" in e}
        self.assertIn("GEOFENCE", causes, "禁飞区注入没有产生任何安全动作")

    def test_stale_position_was_observed(self):
        """必须**去掉禁飞区**再测：禁飞区优先级高于定位过期，
        两者同时命中时只会记 GEOFENCE，POSITION_STALE 被合理地掩盖。
        在同一个场景里断言两件事，会把"优先级正确"误读成"失锁没生效"。
        """
        with tempfile.TemporaryDirectory() as tmp:
            config = busy_scenario(tmp, "no_fence", geofences=[])
            events = list(read_log(Engine(config).run()))
        causes = {e["cause"] for e in events if "cause" in e}
        self.assertIn("POSITION_STALE", causes, "GNSS 失锁没有产生任何安全动作")

    def test_interference_and_obstacle_actually_reduced_quality(self):
        """注入器的作用是"让链路变差"；没有丢弃事件就说明它没接上。"""
        reasons = {e["reason"] for e in self.events if e.get("reason")}
        self.assertIn("OCCLUDED", reasons)

    def test_determinism_with_every_injection_enabled(self):
        from skyfleet.observe.log import normalised_digest
        digests = []
        for tag in ("r1", "r2"):
            config = busy_scenario(self.tmp.name, tag)
            Engine(config).run()
            digests.append(normalised_digest(config.log_path))
        self.assertEqual(digests[0], digests[1])


if __name__ == "__main__":
    unittest.main()


class TestLogInvariantsUnderBatteryCrash(TestLogInvariants):
    """把电量耗尽处置打开，让 crash 路径也被同一组不变量覆盖。

    否则 `BATTERY` 这个 cause 值只在非默认配置下出现，而默认场景里
    永远看不到它——CAUSE_SETS 的子集检查会**空转放行**。这类"守衛只
    覆盖默认路徑"的漏洞最难发现：测试全绿，覆盖为零。
    """

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        config = busy_scenario(
            cls.tmp.name, "invariants_crash",
            on_battery_empty="crash",
            # 让电量真的耗尽：场景 40 s、线性模型 0.05/s 共耗 2.0 单位，
            # 故初始电量取 1.0 才能在中途归零。同时不能因低电而停止接任务。
            initial_energy=1.0, min_energy_for_task=0.0)
        cls.events = list(read_log(Engine(config).run()))
        assert cls.events, "场景没有产生任何事件"

    def test_determinism_with_battery_crash_enabled(self):
        """父类那条确定性测试调用 busy_scenario 时**没有**带 crash 设定，
        因此它在这里复测的是默认策略的确定性，而不是崩溃路径的。
        重写一遍，让本类的确定性断言真的覆盖 crash。
        """
        from skyfleet.observe.log import normalised_digest
        digests = []
        for tag in ("cr1", "cr2"):
            config = busy_scenario(self.tmp.name, tag,
                                   on_battery_empty="crash",
                                   initial_energy=1.0,
                                   min_energy_for_task=0.0)
            Engine(config).run()
            digests.append(normalised_digest(config.log_path))
        self.assertEqual(digests[0], digests[1])

    def test_the_crash_path_actually_fired(self):
        """先确认这条路径真的被走到了，否则本类只是重复父类。"""
        kinds = {e["kind"] for e in self.events}
        self.assertIn("BATTERY_EMPTY", kinds,
                      "崩溃路径没有被走到，本类的断言全部空转")
        self.assertIn("BATTERY",
                      {e.get("cause") for e in self.events
                       if e["kind"] == "NODE_LEAVE"})
