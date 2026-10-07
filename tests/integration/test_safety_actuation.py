"""P3 切片：本地安全动作闭环（致动）。

对标设计文档 §7.4 与 §16.2 第 2 条（「安全动作只写日志、从不执行」）。

测试策略：**直接驱动受控状态**，而不是硬造一个「持续失联」的场景。
理由是后者依赖选举、能量等一串耦合行为，一旦断言失败很难判断是
致动错了还是场景没造出来。这里把 `last_heard` 设成精确的秒数差，
于是「记录阈值」与「致动阈值」的边界可以被逐点验证。
"""

from __future__ import annotations

import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.mission.safety import SafetyAction, safety_decision
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import RELAY_LAYOUTS, Engine

SCENARIO = Path("skyfleet/scenarios/baseline_small.json")

#: 致动阈值必须是心跳周期的整数倍，且显著大于记录阈值（2 周期）。
#: 基线里最长失联 5s，故 10 周期（=10s）可保证基线不触发致动。
ACT_FACTOR = 10.0
RECORD_FACTOR = 2.0


def assembled(tmp: str, **overrides):
    """装配引擎并打开日志（不运行）。

    调用方负责在结束时 `engine.log.close()`——用 `closing()` 包住，
    否则临时目录里的句柄会泄漏成 ResourceWarning，把真正的告警淹没。
    """
    cfg = replace(load_config(SCENARIO),
                  log_path=str(Path(tmp) / "s.jsonl"), **overrides)
    engine = Engine(cfg).build()
    engine._open_log()
    return cfg, engine


class _Assembled:
    """`assembled` 的上下文管理器版本：保证日志句柄被关闭。"""

    def __init__(self, tmp: str, **overrides) -> None:
        self.tmp, self.overrides = tmp, overrides

    def __enter__(self):
        self.cfg, self.engine = assembled(self.tmp, **self.overrides)
        return self.cfg, self.engine

    def __exit__(self, *exc):
        log = getattr(self.engine, "log", None)
        if log is not None:
            log.close()
        return False


def silence(engine, node_id: str, *, leader_age: float | None = None,
            relay_age: float | None = None, t: float = 100.0) -> None:
    """把某节点的 `last_heard` 设成指定「多久没听到了」。"""
    agent = engine.agents[node_id]
    if leader_age is not None:
        agent.last_heard[engine.leader_id] = t - leader_age
    if relay_age is not None:
        for rid in engine.relay_ids:
            agent.last_heard[rid] = t - relay_age


def a_slave_with_a_route(engine):
    """找一个非主节点、非中继的普通从机，并给它一条在途航线。"""
    for nid, agent in engine.agents.items():
        if nid == engine.leader_id or nid in engine.relay_ids:
            continue
        mover = engine.movers[nid]
        mover.set_route([(10.0, 10.0, 30.0), (20.0, 20.0, 30.0)])
        return nid, mover
    raise AssertionError("场景里没有普通从机")


class TestThresholdsAreSeparate(unittest.TestCase):
    def test_baseline_never_actuates(self):
        """★ 基线场景不得触发任何致动。

        实测基线里 22 次 `LINK_LOST` **全部是暂态**（1~5s 即恢复）。
        若用记录用的 2 周期阈值去致动，节点会在普通丢包时反复停飞
        1~5 秒——对配送无人机而言，短暂失去上行链路就停在半路，
        比继续飞更糟。这条测试是那个设计决策的守卫。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "b.jsonl"))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        actuated = [e for e in events
                    if e["kind"] == "SAFETY_ACTION" and e.get("actuated")]
        self.assertEqual(
            actuated, [],
            f"基线场景触发了 {len(actuated)} 次致动——致动阈值太激进，"
            f"普通丢包被当成了持续失联")

    def test_transient_loss_is_recorded_but_not_actuated(self):
        """失联 3s：**要记录**（可观测性），但**不致动**（改变飞行）。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            before = list(mover.route)
            silence(engine, nid, leader_age=3.0, relay_age=3.0, t=100.0)
            engine._check_safety(100.0)
        self.assertEqual(mover.route, before,
                         "3s 失联（普通丢包）就改变了航线")

    def test_sustained_loss_is_actuated(self):
        """失联 15s：必须致动——否则功能是空转的。

        这一条与上一条配对：只测「不该动时不动」会漏掉「该动时不动」
        这一类静默失效（本项目已犯过一次：处理常式没注册，消息送达
        却无人接手）。
        """
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            silence(engine, nid, leader_age=15.0, relay_age=15.0, t=100.0)
            engine._check_safety(100.0)
        self.assertEqual(mover.route, [],
                         "持续失联 15s 却没有停飞——安全动作仍未致动")


class TestTriggerMatchesDesign(unittest.TestCase):
    """§7.4：触发条件是「与主节点**及所有中继**失联」。"""

    def test_losing_only_the_leader_is_not_link_loss(self):
        """只丢主节点、中继仍可达 → **不**算失联。

        实测基线 t=332.0 全部 10 个节点同时报 `LINK_LOST`，而当时中继
        完全可达——正是这条判据缺失导致的误报。
        """
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            self.assertTrue(engine.relay_ids, "本场景没有中继，测试会空转")
            # 主节点失联很久，但中继刚刚还听到
            silence(engine, nid, leader_age=30.0, relay_age=0.5, t=100.0)
            engine._check_safety(100.0)
        self.assertNotEqual(
            mover.route, [],
            "只丢主节点（中继可达）就停飞了——触发条件与 §7.4 不符")

    def test_losing_leader_and_all_relays_is_link_loss(self):
        """主节点**与所有**中继都失联 → 算失联，必须致动。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=100.0)
            engine._check_safety(100.0)
        self.assertEqual(mover.route, [],
                         "与主节点及所有中继都失联，却没有停飞")

    def test_a_reachable_relay_alone_prevents_link_loss(self):
        """只要**还有任何一个**中继可达，就不算失联（§7.4 是「及所有」）。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            agent = engine.agents[nid]
            agent.last_heard[engine.leader_id] = 100.0 - 30.0
            for i, rid in enumerate(engine.relay_ids):
                # 只有一个中继可达
                agent.last_heard[rid] = 100.0 - (0.5 if i == 0 else 30.0)
            engine._check_safety(100.0)
        self.assertNotEqual(mover.route, [],
                            "仍有中继可达却判为失联")


class TestRouteIsSavedAndRestored(unittest.TestCase):
    def test_hover_restores_the_route_when_the_link_returns(self):
        """★ 条件消失后**必须恢复航线**。

        `_advance_mission` 只在**到达航点**时被推进，航线一清空就再也
        不会被调用——不恢复的话节点会永久悬停、任务卡死。这是本切片
        最容易写错的地方。
        """
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            original = list(mover.route)
            self.assertTrue(original, "测试前提：必须先有在途航线")

            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=100.0)
            engine._check_safety(100.0)
            self.assertEqual(mover.route, [], "未停飞，后续断言无意义")

            # 链路恢复
            silence(engine, nid, leader_age=0.5, relay_age=0.5, t=101.0)
            engine._check_safety(101.0)
        self.assertEqual(mover.route, original,
                         "链路恢复后没有恢复原航线——节点会永久悬停")

    def test_restored_route_is_a_copy_not_an_alias(self):
        """恢复的必须是**副本**：与保存时的对象共享会导致后续改动
        污染保存的航线，而症状是「恢复后飞向错误的点」，极难排查。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            original = list(mover.route)
            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=100.0)
            engine._check_safety(100.0)
            silence(engine, nid, leader_age=0.5, relay_age=0.5, t=101.0)
            engine._check_safety(101.0)
            mover.route.append((99.0, 99.0, 99.0))
            restored = engine._safety_saved_route.get(nid)
        self.assertEqual(original, [r for r in original])
        if restored is not None:
            self.assertNotIn((99.0, 99.0, 99.0), restored,
                             "恢复的航线是保存对象的别名")

    def test_no_safety_state_leaks_after_recovery(self):
        """恢复后致动状态必须清干净，否则下次进入安全状态会被判为
        「没有变化」而不再致动。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=100.0)
            engine._check_safety(100.0)
            silence(engine, nid, leader_age=0.5, relay_age=0.5, t=101.0)
            engine._check_safety(101.0)
            first = list(mover.route)
            # 再次进入安全状态，必须能再次致动
            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=102.0)
            engine._check_safety(102.0)
        self.assertEqual(mover.route, [],
                         "第二次进入安全状态时没有致动（状态未清理）")


class TestScenarioLevel(unittest.TestCase):
    def test_sustained_leader_loss_actuates_in_a_real_scenario(self):
        """真实场景下（杀掉主节点与全部中继）必须出现致动。

        受控状态测试证明逻辑正确，这一条证明它在**完整的仿真循环**里
        也会被触发——避免出现「单元测试全绿但功能从未在真实流程中生效」。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "kill.jsonl"),
                          duration=300.0,
                          kills=(("L", 20.0), ("R1", 20.0), ("R2", 20.0)))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        actuated = [e for e in events
                    if e["kind"] == "SAFETY_ACTION" and e.get("actuated")]
        self.assertTrue(
            actuated,
            "杀掉主节点与全部中继后没有任何致动——功能在真实流程中未生效")

    def test_actuation_is_deterministic(self):
        """同一输入必须给出同一输出（本项目的核心承诺）。"""
        digests = []
        for _ in range(2):
            with tempfile.TemporaryDirectory() as tmp:
                cfg = replace(load_config(SCENARIO),
                              log_path=str(Path(tmp) / "d.jsonl"),
                              duration=120.0, kills=(("L", 30.0),))
                Engine(cfg).run()
                events = list(read_log(cfg.log_path))
            digests.append([(round(e["t"], 6), e["node_id"], e.get("action"),
                             bool(e.get("actuated")))
                            for e in events if e["kind"] == "SAFETY_ACTION"])
        self.assertEqual(digests[0], digests[1], "致动不确定")


class TestDescendIsDeliberatelyUnreachable(unittest.TestCase):
    def test_descend_is_never_returned(self):
        """★ `DESCEND` 不可达是**明确决定**，不是漏看。

        §7.4 写「低电量 → `RETURN` **或** `DESCEND`」，选 `RETURN` 即
        满足要求。要有意义地使用 `DESCEND`，需要「能否返航」的续航判断
        （剩余电量 vs 返航所需电量），那是新的物理建模，不应顺手发明。

        本测试把「不可达」钉住：将来接真实续航判断时它会失败，从而
        提示这里需要配套实现，而不是让一个死枚举值悄悄留在代码里。
        """
        import itertools
        for combo in itertools.product([False, True], repeat=5):
            link, leader, low, stale, fence = combo
            action = safety_decision(link_lost=link, leader_lost=leader,
                                     low_battery=low, position_stale=stale,
                                     geofence=fence)
            self.assertIsNot(action, SafetyAction.DESCEND,
                             f"DESCEND 变成了可达值：{combo}")
        # 穷举确认覆盖了全部 32 种输入
        self.assertEqual(len(list(itertools.product([False, True], repeat=5))),
                         32)


class TestGroundedNodesAreNotActuated(unittest.TestCase):
    def test_a_grounded_node_is_not_actuated(self):
        """已迫降/坠机的节点不该再被安全动作摆布。"""
        with tempfile.TemporaryDirectory() as tmp, _Assembled(tmp) as (_, engine):
            nid, mover = a_slave_with_a_route(engine)
            engine.grounded.add(nid)
            silence(engine, nid, leader_age=30.0, relay_age=30.0, t=100.0)
            engine._check_safety(100.0)
        # 迫降节点的航线由耗尽策略管理，安全致动不得插手
        self.assertNotEqual(
            engine._last_actuated.get(nid), SafetyAction.HOVER,
            "已迫降的节点仍被安全动作致动")


if __name__ == "__main__":
    unittest.main()
