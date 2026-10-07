"""能量模型可替换性验收。

分析文档 `docs/analysis/2026-10-05-thrust-margin-analysis.md` §5 查出：
引擎原先对每个存活节点**无条件**扣 `energy_hover_per_s * dt`，与它在
巡航、悬停、爬升还是下降无关——那是一个时钟，不是能量模型。

这里钉住三件事：

1. `linear`（默认）行为**逐位不变**——否则所有已发表的扫描结果与
   golden digest 都会被无声作废。
2. `physical` 真的改变结果，且改变的方向与分析一致：被迫悬停的代价
   被放大。若两者结果相同，说明配置没生效——这种"静默无效"比报错难发现。
3. 协议层与指标层不引用具体能量模型（与 §9 的保险栓同一要求）。
"""
from __future__ import annotations

import ast
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.observe.log import read_log
from skyfleet.observe.metrics import compute_metrics
from skyfleet.sim.engine import Engine
from skyfleet.world.power import (
    AIRFRAMES,
    LinearEnergyModel,
    PhysicalEnergyModel,
    build_energy_model,
)

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "skyfleet" / "scenarios" / "baseline_small.json"


def _run(tmp: str, tag: str, duration: float = 120.0, **overrides):
    config = replace(load_config(SCENARIO),
                     log_path=os.path.join(tmp, f"{tag}.jsonl"),
                     duration=duration, env_seed=1, **overrides)
    Engine(config).run()
    # read_log 是惰性生成器：必须在这里就物化，否则调用方在
    # `with TemporaryDirectory` 之外迭代时会撞上"文件已随目录删除"。
    return list(read_log(config.log_path)), compute_metrics(config.log_path)


def _relative_spread(values: dict[str, float]) -> float:
    """(max-min)/mean，单位 %。用来衡量"该指标能否区分节点"。"""
    vals = list(values.values())
    mean = sum(vals) / len(vals)
    return (max(vals) - min(vals)) / mean * 100.0


def _final_energies(events) -> dict[str, float]:
    last: dict[str, float] = {}
    for e in events:
        if e["kind"] == "ENERGY_SAMPLE":
            last[e["node_id"]] = e["energy"]
    return last


class TestArchitectureGuard(unittest.TestCase):
    """协议层与指标层不得引用具体能量模型。"""

    def _imports(self, folder: str) -> set[str]:
        found: set[str] = set()
        for path in (ROOT / "skyfleet" / folder).rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module:
                    found.add(node.module)
                elif isinstance(node, ast.Import):
                    for alias in node.names:
                        found.add(alias.name)
        return found

    #: 分层禁止的依赖。协议层与任务层既不能碰能量模型，也不能碰仿真引擎
    #: ——早前只挡了 world.power，一条 `mission -> sim.engine` 会溜过去。
    FORBIDDEN = ("world.power", "sim.engine")

    def test_protocol_does_not_import_the_power_model(self):
        offenders = [m for m in self._imports("protocol")
                     if any(f in m for f in self.FORBIDDEN)]
        self.assertEqual(offenders, [])

    def test_mission_does_not_import_the_power_model(self):
        offenders = [m for m in self._imports("mission")
                     if any(f in m for f in self.FORBIDDEN)]
        self.assertEqual(offenders, [])

    def test_metrics_layer_does_not_import_the_power_model(self):
        """指标从日志复算——若它引用能量模型，就变成第二个事实来源。"""
        offenders = [m for m in self._imports("observe")
                     if "world.power" in m or "sim.engine" in m]
        self.assertEqual(offenders, [])


class TestLinearIsUnchanged(unittest.TestCase):
    """默认模型必须与 P0/P1 完全一致。"""

    def test_default_scenario_uses_linear(self):
        model = build_energy_model(load_config(SCENARIO))
        self.assertIsInstance(model, LinearEnergyModel)

    def test_linear_spend_is_the_clock_plus_tx(self):
        """linear 下耗电 ≈ 0.05*时长：恒定时钟占绝对主导。

        实测 120 s 场景均值 6.077 单位，其中恒定项恰为 6.0，
        余额是发送耗电。**不是完全相等**——转发较多的 R1 会略高，
        但差值不到 0.5%，所以该指标区分不了"谁在飞、谁在等"。
        """
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = _run(tmp, "linear")
        config = load_config(SCENARIO)
        spent = {k: config.initial_energy - v
                 for k, v in _final_energies(events).items()}
        self.assertTrue(spent)
        clock = config.energy_hover_per_s * 120.0
        mean = sum(spent.values()) / len(spent)
        self.assertAlmostEqual(mean, clock, delta=clock * 0.02)
        self.assertLess(_relative_spread(spent), 1.0,
                        f"linear 模型下各节点耗电过于离散：{spent}")


class TestPhysicalChangesTheResult(unittest.TestCase):
    """physical 模型必须真的生效，且方向正确。"""

    def test_physical_is_selected_and_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            events, metrics = _run(tmp, "physical", energy_model="physical")
        self.assertTrue(any(e["kind"] == "TASK_DELIVER" for e in events))
        self.assertGreater(metrics["tasks"]["delivered"], 0)

    def test_physical_makes_the_metric_able_to_tell_nodes_apart(self):
        """★ 核心价值：physical 让电量指标真正区分节点。

        实测同一场景：linear 离散度 0.49%（等于没有信息），
        physical 31.9%。这条把"原先的能耗指标区分不了谁在飞"这一
        结论钉死——它正是分析文档 §5.2 指出的失真。
        """
        with tempfile.TemporaryDirectory() as tmp:
            lin, _ = _run(tmp, "lin", energy_model="linear")
            phy, _ = _run(tmp, "phy", energy_model="physical")
        lin_spent = {k: 100.0 - v for k, v in _final_energies(lin).items()}
        phy_spent = {k: 100.0 - v for k, v in _final_energies(phy).items()}
        lin_spread = _relative_spread(lin_spent)
        phy_spread = _relative_spread(phy_spent)
        self.assertGreater(phy_spread, 10.0 * lin_spread,
                           f"physical 离散度 {phy_spread:.2f}% 未显著超过 "
                           f"linear 的 {lin_spread:.2f}%")
        self.assertGreater(phy_spread, 20.0)

    def test_stationary_infrastructure_pays_the_hover_penalty(self):
        """物理结论在日志里可见：**站桩的中继比巡航的从机更费电**。

        中继是基础设施（busy_nodes），它们不接任务、基本停在原地；
        从机在巡航。而巡航比悬停省电——所以中继耗电应当高于从机。
        这条把"悬停最贵"从公式变成可观测结果。
        """
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = _run(tmp, "phy", energy_model="physical")
        spent = {k: 100.0 - v for k, v in _final_energies(events).items()}
        relays = [v for k, v in spent.items() if k.startswith("R")]
        slaves = [v for k, v in spent.items() if k.startswith("S")]
        self.assertTrue(relays and slaves)
        self.assertGreater(min(relays), max(slaves),
                           f"中继 {relays} 未高于从机 {slaves}")

    def test_physical_drains_less_than_linear_here(self):
        """本场景无人机大部分时间在巡航，而巡航比悬停省电。

        所以 physical 的总耗电应当**低于** linear 的恒定时钟。
        若这条红了，先怀疑功率公式而不是场景。
        """
        with tempfile.TemporaryDirectory() as tmp:
            lin, _ = _run(tmp, "lin", energy_model="linear")
            phy, _ = _run(tmp, "phy", energy_model="physical")
        lin_spent = sum(100.0 - v for v in _final_energies(lin).values())
        phy_spent = sum(100.0 - v for v in _final_energies(phy).values())
        self.assertLess(phy_spent, lin_spent)

    def test_physical_is_deterministic(self):
        """换成物理模型不得引入非确定性。"""
        with tempfile.TemporaryDirectory() as tmp:
            a = _digest(tmp, "a")
            b = _digest(tmp, "b")
        self.assertEqual(a, b)

    def test_physical_respects_payload(self):
        """载荷越大耗电越多——载荷是分析文档 §3 指出的饱和来源之一。"""
        with tempfile.TemporaryDirectory() as tmp:
            light, _ = _run(tmp, "light", energy_model="physical",
                            payload_kg=0.0)
            heavy, _ = _run(tmp, "heavy", energy_model="physical",
                            payload_kg=1.0)
        light_spent = sum(100.0 - v for v in _final_energies(light).values())
        heavy_spent = sum(100.0 - v for v in _final_energies(heavy).values())
        self.assertGreater(heavy_spent, light_spent)

    def test_airframe_choice_changes_the_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            px4, _ = _run(tmp, "px4", energy_model="physical",
                          airframe="px4_sihsim_quadx")
            cf, _ = _run(tmp, "cf", energy_model="physical",
                         airframe="crazyflie")
        px4_spent = sum(100.0 - v for v in _final_energies(px4).values())
        cf_spent = sum(100.0 - v for v in _final_energies(cf).values())
        self.assertNotAlmostEqual(px4_spent, cf_spent, places=3)

    def test_protocol_outcome_is_unaffected_by_energy_model(self):
        """能量模型只影响电量，不该改变交付结果——否则就是把两件事耦合了。

        这正是 §9 可替换性要求的意义：换模型不动协议。
        """
        with tempfile.TemporaryDirectory() as tmp:
            _, lin = _run(tmp, "lin", energy_model="linear")
            _, phy = _run(tmp, "phy", energy_model="physical")
        self.assertEqual(lin["tasks"]["delivered"], phy["tasks"]["delivered"])
        self.assertEqual(lin["drop_reasons"], phy["drop_reasons"])
        self.assertEqual(lin["loss"], phy["loss"])


class TestKnownGapsExposedByThePhysicalModel(unittest.TestCase):
    """物理能量模型暴露出的两处缺口——**刻意不修**，用测试钉住。

    它们直接关系到"会不会掉机"：目前的答案是"不会，因为没有任何机制
    把电量或安全决策接到飞行上"。这属于协议语义选择，应当由团队决定，
    而不是由实现在这里悄悄补上。
    """

    def test_zero_energy_does_not_stop_a_node(self):
        """电量归零后节点仍照常飞行与发送。

        `Node.consume` 把下界压在 0，但没有"电量耗尽"这个事件，
        也没有迫降/坠机。所以物理上不可能的任务（30 g 机型挂 0.5 kg）
        仍然报告 100% 完成率——扫参里 min_final 已经到 0.00 而完成率
        仍是 1.00。
        """
        with tempfile.TemporaryDirectory() as tmp:
            events, metrics = _run(tmp, "empty", duration=600.0,
                                   energy_model="physical",
                                   airframe="crazyflie", payload_kg=0.5,
                                   min_energy_for_task=0.0)
        finals = _final_energies(events)
        exhausted = [k for k, v in finals.items() if v <= 0.0]
        self.assertTrue(exhausted, f"没有节点耗尽电量，无法验证：{finals}")
        # 耗尽之后仍在发送
        last_t = {}
        for e in events:
            if e["kind"] == "MSG_TX":
                last_t[e["node_id"]] = e["t"]
        for node_id in exhausted:
            self.assertIn(node_id, last_t,
                          f"{node_id} 电量耗尽后完全不再发送（缺口已被修？）")
        self.assertEqual(metrics["tasks"]["completion_rate"], 1.0)

    def test_safety_actions_are_logged_but_never_actuated(self):
        """安全动作只写日志，不改变飞行。

        `safety_decision` 的输出只在 engine 里做变化检测与记录，
        没有任何代码用它来改变航线或悬停。所以 `RETURN` 不会返航、
        `HOVER` 不会真的悬停、`HOLD` 不会真的停住。

        这条测试证明"返回"决策出现后节点仍在正常收发——即它是**建议性**的。
        修好之后这条会红，那时应当连同报告一起更新。
        """
        with tempfile.TemporaryDirectory() as tmp:
            # 电量低于阈值 -> 一直判定 LOW_BATTERY -> RETURN
            events, _ = _run(tmp, "return", energy_model="linear",
                             initial_energy=5.0, min_energy_for_task=20.0)
        actions = [e for e in events if e["kind"] == "SAFETY_ACTION"]
        self.assertTrue(actions, "没有产生任何安全动作，无法验证")
        self.assertIn("RETURN", {e["action"] for e in actions})
        # RETURN 期间仍在发送：说明决策没有被执行
        tx = [e for e in events if e["kind"] == "MSG_TX"]
        self.assertGreater(len(tx), 0,
                           "RETURN 后完全静默——决策似乎已经生效（缺口已被修？）")


class TestEngineChargesTxThroughTheModel(unittest.TestCase):
    """工作项 (A) 的引擎接线必须真的被观察到。

    早前没有任何测试能看见这次改动：物理模型的发送单价（~9e-11）
    比旧常量（5e-7）小约 5 个数量级，任何端到端数值都不动，把引擎
    改回 `config.energy_tx_per_bit` 依然是 622 测试全绿。

    做法：把模型自报的发送单价换成一个**夸张**的值。若引擎确实调用
    模型的接口，总耗电会随之暴涨；若引擎仍读 config，则毫无变化。
    """

    def _total_spent(self, tmp, tag, tx_rate=None):
        cfg = replace(load_config(SCENARIO), log_path=str(Path(tmp) / f"{tag}.jsonl"),
                      duration=60.0, env_seed=1, energy_model="physical")
        engine = Engine(cfg).build()      # self.energy 在 build() 里才建立
        model_type = type(engine.energy)
        original = model_type.tx_energy_per_bit
        if tx_rate is not None:
            model_type.tx_energy_per_bit = lambda self: tx_rate
        try:
            engine.run()
        finally:
            model_type.tx_energy_per_bit = original
        events = list(read_log(cfg.log_path))
        bits = sum(e["bits"] for e in events if e["kind"] == "MSG_TX")
        return bits, engine.agents, events

    def test_a_huge_model_tx_rate_moves_the_energy_ledger(self):
        with tempfile.TemporaryDirectory() as tmp:
            bits, agents, _ = self._total_spent(tmp, "huge", tx_rate=1.0)
        self.assertGreater(bits, 0, "场景没有发送任何消息，本测试会空转")
        # 每 bit 1.0 个电量单位、log 里有百万级 bit——若有接线，耗电必然
        # 远超初始电量（100）。
        spent = sum(a.node.energy_spent for a in agents.values())
        self.assertGreater(spent, 1000.0,
                           "把模型发送单价设成 1.0 后总耗电仍很小——"
                           "引擎没有读模型的 tx_energy_per_bit()")

    def test_the_engine_rate_is_the_models_not_the_legacy_constant(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, agents, _ = self._total_spent(tmp, "normal")
        spent = sum(a.node.energy_spent for a in agents.values())
        # 60 s、11 节点：飞行耗电约 11 * 0.05 * 60 量级；若引擎用旧常量
        # (5e-7/bit)，发送会额外贡献约 1 个单位，用模型则是 1e-5 量级。
        self.assertLess(spent, 100.0)


class TestBatteryExhaustionPolicy(unittest.TestCase):
    """电量耗尽之后怎么办——这是"会不会掉机"的答案所在。

    原先电量归零**没有任何后果**：没有耗尽事件、没有迫降、没有坠机，
    因此物理上不可能的任务（30 g 机型挂 0.5 kg）仍报告 100% 完成率。
    这里把三种策略的行为钉住；默认 `none` 保留既有行为。
    """

    #: 一个必定耗尽电量的组合：Crazyflie（30 g）挂 0.5 kg，推重比 0.11，
    #: 物理上根本飞不起来。
    #:
    #: initial_energy=30 是关键：默认 100 时无人机能撑到 t≈485 s，而全部
    #: 20 个任务在 t≈197 s 就投递完了——**掉机发生在任务结束之后**，
    #: 于是没有任何任务会失败。要让闭环可观测，必须让耗尽可能发生在
    #: 任务仍在进行时。30 时首个坠机在 t≈129 s，完成率降到 0.80。
    DOOMED = dict(energy_model="physical", airframe="crazyflie",
                  payload_kg=0.5, min_energy_for_task=0.0,
                  initial_energy=30.0)

    def _run_doomed(self, tmp, tag, **overrides):
        kw = dict(self.DOOMED)
        kw.update(overrides)
        return _run(tmp, tag, duration=600.0, **kw)

    def test_default_policy_is_none_and_preserves_legacy_behaviour(self):
        """默认必须是 none，且行为上真的没有任何后果。

        只读 JSON 是空断言（把整个特性删掉它照样通过）；这里同时验证
        行为：电量归零不产生事件、不产生 grounded、不改变完成率。
        """
        self.assertEqual(load_config(SCENARIO).on_battery_empty, "none")
        with tempfile.TemporaryDirectory() as tmp:
            events, metrics = self._run_doomed(tmp, "legacy_none")
            engine_grounded = [e for e in events
                               if e["kind"] in ("NODE_LEAVE", "BATTERY_EMPTY")]
            self.assertEqual([e for e in engine_grounded
                              if e.get("cause") == "BATTERY"], [],
                             "none 策略下不应有任何电池相关事件")
            self.assertEqual(metrics["tasks"]["completion_rate"], 1.0,
                             "none 策略下电量归零不应影响任务完成率")

    def test_none_policy_emits_no_exhaustion_event(self):
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = self._run_doomed(tmp, "none", on_battery_empty="none")
        self.assertFalse([e for e in events
                          if e["kind"] == "BATTERY_EMPTY"])

    def test_land_policy_emits_one_event_per_node_and_stops_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = self._run_doomed(tmp, "land", on_battery_empty="land")
        empty = [e for e in events if e["kind"] == "BATTERY_EMPTY"]
        self.assertTrue(empty, "land 策略下没有产生 BATTERY_EMPTY 事件")
        nodes = [e["node_id"] for e in empty]
        self.assertEqual(len(nodes), len(set(nodes)), "同一节点报了多次耗尽")
        # 迫降后不再移动
        landed = {e["node_id"]: e["t"] for e in empty}
        last_pos: dict[str, tuple] = {}
        for e in events:
            if e["kind"] == "ENERGY_SAMPLE" and e["node_id"] in landed                     and e["t"] >= landed[e["node_id"]]:
                pos = tuple(e["pos"])
                if e["node_id"] in last_pos:
                    self.assertEqual(pos, last_pos[e["node_id"]],
                                     f"{e['node_id']} 迫降后仍在移动")
                last_pos[e["node_id"]] = pos

    def test_crash_policy_removes_the_node(self):
        """坠机复用既有失效机制：NODE_LEAVE(cause=BATTERY)。"""
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = self._run_doomed(tmp, "crash",
                                         on_battery_empty="crash")
        leaves = [e for e in events if e["kind"] == "NODE_LEAVE"
                  and e.get("cause") == "BATTERY"]
        self.assertTrue(leaves, "crash 策略下没有产生 NODE_LEAVE(BATTERY)")
        nodes = [e["node_id"] for e in leaves]
        self.assertEqual(len(nodes), len(set(nodes)), "同一节点坠机多次")
        # 坠机后不再发送
        crashed = {e["node_id"]: e["t"] for e in leaves}
        for e in events:
            if e["kind"] == "MSG_TX" and e["node_id"] in crashed:
                self.assertLess(e["t"], crashed[e["node_id"]] + 1e-9,
                                f"{e['node_id']} 坠机后仍在发送")

    def test_crash_makes_an_impossible_mission_actually_fail(self):
        """★ 闭环：物理上不可能的任务必须真的失败。

        这正是"集群通讯会不会掉机"的答案——加上耗尽处置之后，
        完成率**不再**是 1.00，掉机终于有后果。
        """
        with tempfile.TemporaryDirectory() as tmp:
            _, none_m = self._run_doomed(tmp, "none", on_battery_empty="none")
            _, crash_m = self._run_doomed(tmp, "crash",
                                          on_battery_empty="crash")
        # 默认策略下电量归零毫无后果：20/20 照常完成
        self.assertEqual(none_m["tasks"]["completion_rate"], 1.0)
        # 坠机策略下任务真的失败了
        self.assertLess(crash_m["tasks"]["completion_rate"], 1.0,
                        "坠机策略下完成率仍是 1.00，闭环没有生效")
        self.assertGreater(crash_m["tasks"]["delivered"], 0,
                           "一个任务都没交付，说明不是掉机导致而是整体崩了")

    def test_a_live_leader_reassigns_a_failed_member(self):
        """活着的主节点必须能察觉成员失效并重派任务。

        用 `kills=` 注入制造"从机先死、主节点还活着"的时序。**不能**靠
        自动耗尽制造这个时序：均匀初始电量下主节点与中继 100% 时间悬停，
        而悬停最费电，它们总是最先耗尽。

        早前这条测试断言的是"掉机后主节点有反应"，而实测那些反应**全部**
        来自一个已经坠机的主节点——它靠冻结的成员视图把活着的成员判成
        失效。那条断言当时认证的恰恰是自己的反面。
        """
        with tempfile.TemporaryDirectory() as tmp:
            events, _ = self._run_doomed(tmp, "live_leader",
                                         kills=(("S2", 100.0),))
        deaths = [e["t"] for e in events
                  if e["kind"] == "NODE_LEAVE" and e["node_id"] == "L"
                  and e.get("cause") == "BATTERY"]
        leader_death = min(deaths) if deaths else float("inf")

        rs = [e for e in events if e["kind"] == "TASK_REASSIGN"]
        self.assertTrue(rs, "活着的主节点没有重派任何任务——失效被察觉了吗？")
        for e in rs:
            with self.subTest(t=e["t"]):
                self.assertLess(e["t"], leader_death,
                                "重派发生在主节点死亡之后，是幽灵事件")
        # 主节点在做出这些判断时必须真的发得出消息
        self.assertTrue(
            [e for e in events if e["kind"] == "MSG_TX"
             and e["node_id"] == "L" and e["t"] < leader_death],
            "主节点在重派之前没有发出任何消息")

    def test_no_leader_failover_is_a_documented_limitation(self):
        """均匀初始电量下，主节点与中继先耗尽，集群失去任务编排。

        这是**已知限制**，不是已解决的行为：本模拟器没有主节点故障切换。
        这条测试把它钉住，免得日后被误读成"掉机后集群照常工作"。
        """
        with tempfile.TemporaryDirectory() as tmp:
            events, metrics = self._run_doomed(tmp, "no_failover",
                                               on_battery_empty="crash")
        deaths = [e["t"] for e in events
                  if e["kind"] == "NODE_LEAVE" and e.get("cause") == "BATTERY"]
        self.assertTrue(deaths)
        infra = [t for t, n in ((e["t"], e["node_id"]) for e in events
                                if e["kind"] == "NODE_LEAVE"
                                and e.get("cause") == "BATTERY")
                 if n in ("L", "R1", "R2")]
        slaves = [t for t, n in ((e["t"], e["node_id"]) for e in events
                                 if e["kind"] == "NODE_LEAVE"
                                 and e.get("cause") == "BATTERY")
                  if n.startswith("S")]
        self.assertTrue(infra and slaves)
        self.assertLess(min(infra), min(slaves),
                        "基础设施不再最先耗尽，这条限制说明需要重写")
        self.assertLess(metrics["tasks"]["completion_rate"], 1.0)

    def test_a_crashed_leader_stops_and_a_new_one_takes_over(self):
        """★ 坠机的主节点必须停止，且**有节点接管**。

        原缺陷（对抗式审查发现）：`_on_leader_check` 是排程回调，绑定在
        引擎上而不看 `agent.alive`；而 `_on_rx` 对已死节点提前返回，于是
        死主节点的成员视图**冻结**，接着把仍然活着的成员一个个判成
        FAILED 并重派它们的任务。实测（battery_limit + crash）：L 于
        t=485.1 坠机，之后仍写了 104 条任务/成员管理事件而 MSG_TX = 0。

        当时只是"让它停"，本模拟器**没有故障切换**——集群会永久失去
        任务编排能力。P2 切片 B 补上了选举。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(Path("skyfleet/scenarios/battery_limit.json")),
                          log_path=str(Path(tmp) / "leader.jsonl"),
                          on_battery_empty="crash")
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))

        deaths = [e["t"] for e in events
                  if e["kind"] == "NODE_LEAVE" and e["node_id"] == "L"
                  and e.get("cause") == "BATTERY"]
        self.assertTrue(deaths, "主节点没有坠机，本测试会空转")
        death = min(deaths)

        # 1) 死者闭嘴：主节点死后**它自己**不能再发消息，也不能再署名下令。
        self.assertEqual(
            [e for e in events if e["t"] > death and e["kind"] == "MSG_TX"
             and e["node_id"] == "L"], [],
            "主节点死后仍在发送")
        from_l = [e for e in events if e["t"] > death
                  and e["kind"] in ("TASK_ASSIGN", "TASK_REASSIGN")
                  and e.get("node_id") == "L"]
        self.assertEqual(from_l, [],
                         f"死掉的主节点仍以自己的名义下达了 {len(from_l)} 条指令")

        # 2) 生者接任。注意本场景**会**出现多次宣告——见
        #    test_cascading_failover_when_leaders_themselves_die。
        announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
        self.assertTrue(announces, "主节点死亡后没有任何节点接管")
        first = announces[0]
        self.assertNotEqual(first["node_id"], "L", "宣告者仍是已死的主节点")
        self.assertEqual(first["old_leader"], "L")
        self.assertGreaterEqual(first["t"], death, "宣告早于主节点死亡")

        # 3) 接管者必须**真的在行使主节点职责**。
        #
        # 原先这里断言"接任之后必须有 TASK_ASSIGN"，那是**脆弱断言**：
        # 本场景 60 个任务在 t≈0 就全部指派完毕，接任发生在 485s，
        # 此时已交付 59 个——"死后再有指派"本就不可达。它此前能通过，
        # 只是因为当时有较多任务失败被重派；行为改善后反而变红。
        #
        # 正确的证据是"新主节点在持续广播心跳"（主节点职责的载体），
        # 而"接任后任务继续被编排"由下面
        # test_a_single_failure_causes_exactly_one_election 覆盖，
        # 那里任务确实会在接任后继续被指派。
        beats = [e for e in events
                 if e["kind"] == "HEARTBEAT_TX"
                 and e["node_id"] == first["node_id"]
                 and e["t"] > first["t"]]
        self.assertTrue(
            beats,
            f"{first['node_id']} 宣布接任后没有再广播心跳——它没有真正"
            f"承担主节点职责")

    def test_cascading_failover_when_leaders_themselves_die(self):
        """★ 续航受限场景下的**串联故障切换**，是一处真实行为而非缺陷。

        在 `battery_limit`（初始电量 45、耗尽即坠机）里，所有节点最终都会
        耗尽——包括刚上任的主节点。实测时间线：

        | 时刻 | 事件 |
        |---|---|
        | 485.1 | L、R1、R2 同时耗尽（它们不飞任务航线，悬停最费电）|
        | 485.2 | S1 当选（epoch 1）|
        | 560.7 | **S1 自己也耗尽** |
        | 560.9 | S2 当选（epoch 2）|
        | 562.9 | S2 也耗尽 |
        | 563.0 | S3 当选（epoch 3）|

        这说明两件事，都要写进报告：
        1. 任期号**单调递增**，选举链是可追溯的；
        2. 选举次数不是"1"，所以任何断言"恰好一次宣告"的测试在续航受限
           场景下都是错的——本测试把这个区分固定下来。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(Path("skyfleet/scenarios/battery_limit.json")),
                          log_path=str(Path(tmp) / "cascade.jsonl"),
                          on_battery_empty="crash")
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))

        announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
        self.assertGreaterEqual(len(announces), 2,
                                "本场景应出现串联重选，若只有一次说明前提已变")
        # 任期严格单调递增
        epochs = [e["leader_epoch"] for e in announces]
        self.assertEqual(epochs, sorted(set(epochs)),
                         f"任期号没有严格递增：{epochs}")
        # 每次当选者与被替代者必须不同，且链条连续
        for prev, cur in zip(announces, announces[1:]):
            self.assertEqual(cur["old_leader"], prev["node_id"],
                             "选举链断了：新宣告的 old_leader 不是上一任")
        # 当选者不得是已死节点
        dead_at = {}
        for e in events:
            if e["kind"] == "NODE_LEAVE":
                dead_at.setdefault(e["node_id"], e["t"])
        for ann in announces:
            d = dead_at.get(ann["node_id"])
            self.assertTrue(d is None or ann["t"] < d,
                            f"{ann['node_id']} 在自己死亡之后当选")
        # **不是选举风暴**。判据是"有没有换掉一个还活着的主节点"，
        # 而**不是**"两次宣告间隔多久"——后者是错的：实测 S2 于 560.9
        # 当选、562.9 就自己耗尽，间隔只有 2.1s，但那次选举完全正当。
        # 用间隔当判据会把正确的串联切换误报成风暴。
        for ann in announces[1:]:
            old_leader = ann["old_leader"]
            died = dead_at.get(old_leader)
            self.assertIsNotNone(
                died,
                f"{ann['node_id']} 在 {ann['t']:.1f}s 取代了 {old_leader}，"
                f"但 {old_leader} 从未死亡——这是换掉活主节点的选举风暴")
            self.assertLessEqual(
                died, ann["t"] + 1e-9,
                f"在 {old_leader} 死亡（{died:.1f}s）之前就把它换掉了")

    def test_a_single_failure_causes_exactly_one_election(self):
        """单次故障必须**恰好**产生一次选举（防选举风暴）。

        与上一个测试互补：续航受限场景里多次宣告是**正确**的（每次都
        有新的主节点耗尽），而在单次击杀的干净场景里，多于一次就说明
        有节点在反复重选。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "single.jsonl"),
                          duration=200.0, kills=(("L", 20.0),))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
        self.assertEqual(len(announces), 1,
                         f"单次故障产生了 {len(announces)} 次选举（选举风暴）")
        self.assertEqual(announces[0]["leader_epoch"], 1)

        # ★ 接任之后任务**必须继续被编排**——这是"有接管者"的实证，
        #   也是规格 §4 对切片 B 的验收条款。本场景（20 任务、主节点在
        #   20s 被杀）在接任后仍有任务需要指派/重派，实测 6 笔。
        after = [e for e in events if e["t"] > announces[0]["t"]
                 and e["kind"] in ("TASK_ASSIGN", "TASK_REASSIGN")]
        self.assertTrue(
            after,
            "接任之后没有任何任务编排——接管者没有真正接手编排")

    def test_no_fault_means_no_election(self):
        """无故障场景**不得**产生任何选举事件。

        golden digest 的完整性依赖这一点：无故障路径必须逐位不变。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "nofault.jsonl"))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        self.assertEqual([e for e in events if e["kind"] == "LEADER_ANNOUNCE"],
                         [], "无故障场景发生了选举")

    def test_the_new_leader_is_not_a_relay(self):
        """中继不参选：一旦中继成了主节点，它就会离开站位去送货，
        把自己作为通信基础设施的价值抹掉。"""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "elect.jsonl"),
                          duration=200.0, kills=(("L", 20.0),))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
        self.assertTrue(announces, "主节点被杀却没有选举")
        elected = announces[0]["node_id"]
        self.assertFalse(elected.startswith("R"),
                         f"中继 {elected} 当选了主节点")

    def test_a_revived_old_leader_cannot_take_back_over(self):
        """旧主节点复活后**不得复辟**。

        `leader_epoch` 在选举时递增，这正是 `STALE_EPOCH` 分支第一次
        真正生效——在此之前它恒为 0、从不递增，是一条死代码。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "revive.jsonl"),
                          duration=200.0,
                          kills=(("L", 20.0),), revives=(("L", 60.0),))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
        self.assertTrue(announces, "主节点被杀却没有选举")
        after_revive = [e for e in events if e["t"] > 60.0
                        and e["kind"] in ("TASK_ASSIGN", "TASK_REASSIGN")
                        and e.get("node_id") == "L"]
        self.assertEqual(after_revive, [],
                         f"复活的旧主节点复辟了：{len(after_revive)} 条指令署名 L")
        self.assertEqual(len(announces), 1,
                         f"复活引发了额外选举，共 {len(announces)} 次")

    def test_revived_node_does_not_fly_on_a_flat_battery(self):
        """★ 复活一个电量耗尽的节点，不能让它带着 0 电量复飞。

        否则 `_empty_batteries` 已去重、不会再触发耗尽，节点会永远以 0 电量
        飞行并发送——"电量没有后果"这个刚修掉的缺陷会从另一扇门溜回来。
        语义：要重新入网就得先换电池，故复活时恢复初始电量。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(
                load_config(SCENARIO),
                log_path=str(Path(tmp) / "revive.jsonl"),
                duration=90.0, env_seed=1, slaves=3, relays=1, tasks=6,
                blocks=2, area=400.0, heartbeat_period=0.4,
                telemetry_period=0.4,
                energy_model="linear", on_battery_empty="crash",
                initial_energy=1.0, min_energy_for_task=0.0,
                kills=(("S2", 60.0),), revives=(("S2", 70.0),))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))

        joins = [e["t"] for e in events
                 if e["kind"] == "NODE_JOIN" and e["node_id"] == "S2"]
        self.assertIn(70.0, joins, "复活注入没有生效，本测试会空转")

        # 复活那一刻电量必须已被换满。注意不能用 min()：场景只跑到 90 s，
        # 而换上的电池在 0.05/s 下 20 s 就会再次耗尽，min() 会把"再次耗尽"
        # 误判成"没有换电池"。
        # 取**严格晚于**入网时刻的第一个采样。用 >= 会在采样点恰好落在
        # 70.0 时取到复活前的那一条（0.0），造成与实现无关的偶发失败。
        after = [e for e in events
                 if e["kind"] == "ENERGY_SAMPLE" and e["node_id"] == "S2"
                 and e["t"] > 70.0]
        self.assertTrue(after, "复活后没有 S2 的采样")
        self.assertGreater(after[0]["energy"], 0.5,
                           "S2 带着耗尽前的电量复活（没有换电池）")

        # 而且不能因此获得永久免疫：换上电池后应当能再次耗尽。
        empty = [e["t"] for e in events
                 if e["kind"] == "BATTERY_EMPTY" and e["node_id"] == "S2"]
        self.assertGreaterEqual(len(empty), 2,
                                "复活后 S2 再未耗尽——去重集合没有被清掉，"
                                "节点获得了永久免疫")

    def test_revive_under_none_policy_keeps_legacy_behaviour(self):
        """默认策略下复活**不得**改电量——那会动已发表的数字。"""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(
                load_config(SCENARIO),
                log_path=str(Path(tmp) / "revive_none.jsonl"),
                duration=90.0, env_seed=1, slaves=3, relays=1, tasks=6,
                blocks=2, area=400.0, heartbeat_period=0.4,
                telemetry_period=0.4,
                energy_model="linear", on_battery_empty="none",
                initial_energy=1.0, min_energy_for_task=0.0,
                kills=(("S2", 60.0),), revives=(("S2", 70.0),))
            Engine(cfg).run()
            events = list(read_log(cfg.log_path))
        after = [e for e in events
                 if e["kind"] == "ENERGY_SAMPLE" and e["node_id"] == "S2"
                 and e["t"] >= 70.0]
        self.assertTrue(after)
        self.assertEqual(max(e["energy"] for e in after), 0.0,
                         "none 策略下复活改了电量，破坏了既有行为")

    def test_unknown_policy_is_rejected_loudly(self):
        with self.assertRaises(ValueError):
            Engine(replace(load_config(SCENARIO),
                           on_battery_empty="explode")).build()


def _digest(tmp: str, tag: str) -> str:
    from skyfleet.observe.log import normalised_digest
    config = replace(load_config(SCENARIO),
                     log_path=os.path.join(tmp, f"{tag}.jsonl"),
                     duration=120.0, env_seed=1, energy_model="physical")
    Engine(config).run()
    return normalised_digest(config.log_path)


class TestAirframeRegistry(unittest.TestCase):
    def test_all_registered_airframes_are_usable(self):
        for name, af in AIRFRAMES.items():
            with self.subTest(airframe=name):
                m = PhysicalEnergyModel(af)
                self.assertGreater(m.hover_power_w(), 0.0)
                self.assertGreater(m.endurance_s(), 0.0)

    def test_registry_matches_the_analysis_document_count(self):
        """分析文档 §3 列了四种机型；增删应当是有意识的。"""
        self.assertEqual(len(AIRFRAMES), 4)


if __name__ == "__main__":
    unittest.main()
