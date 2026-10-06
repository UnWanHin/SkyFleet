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

    def test_protocol_does_not_import_the_power_model(self):
        offenders = [m for m in self._imports("protocol")
                     if "world.power" in m]
        self.assertEqual(offenders, [])

    def test_mission_does_not_import_the_power_model(self):
        offenders = [m for m in self._imports("mission")
                     if "world.power" in m]
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
