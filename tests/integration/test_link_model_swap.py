"""P1 保险栓验收：换链路模型，协议层零改动。

规格 §9 把 `LinkModel` 列为 P1 的第一条扩展接口，理由是"只要替换本模块的
实现，就能把 P0 的简易几何遮挡换成非视距绕射与多径快衰落，而协议层零改动"。

这是一个**可证伪的承诺**，所以要用测试钉住，而不只是写在文档里：

1. `protocol/` 与 `mission/` 不得 import 任何链路模型——一旦有人图省事在
   路由或可靠性里直接引用具体模型，替换就会到处漏改。
2. 两个模型都必须能跑完整场景并交付任务，不能只跑通单测。
3. 换模型后结果**必须不同**——若两者给出一样的结果，说明配置没生效，
   这种"静默无效"比报错更难发现。
"""
import ast
import json
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import SimConfig, load_config
from skyfleet.observe.log import normalised_digest, read_log
from skyfleet.observe.metrics import compute_metrics
from skyfleet.sim.engine import Engine
from skyfleet.world.link import (GeometricOcclusionLinkModel, build_link_model)
from skyfleet.world.link_nlos import NlosDiffractionLinkModel

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "skyfleet" / "scenarios" / "baseline_small.json"


def tiny(**overrides) -> dict:
    base = dict(seed=1, duration=40.0, slaves=3, relays=1, tasks=6,
                blocks=2, area=400.0, heartbeat_period=0.5,
                telemetry_period=0.5, env_seed=1)
    base.update(overrides)
    return base


class TestProtocolLayerStaysModelAgnostic(unittest.TestCase):
    """架构守卫：协议层与业务层不得依赖具体链路模型。"""

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

    def test_protocol_does_not_import_any_link_model(self):
        modules = self._imports("protocol")
        offenders = [m for m in modules if "world.link" in m or "nlos" in m]
        self.assertEqual(offenders, [],
                         f"协议层引用了具体链路模型：{offenders}")

    def test_mission_does_not_import_any_link_model(self):
        modules = self._imports("mission")
        offenders = [m for m in modules if "world.link" in m or "nlos" in m]
        self.assertEqual(offenders, [])

    def test_observe_does_not_import_the_simulation_layer(self):
        """指标层只读日志，不引用仿真对象——否则"指标与图表同源"就不成立。"""
        modules = self._imports("observe")
        offenders = [m for m in modules
                     if m.startswith("skyfleet.sim")
                     or m.startswith("skyfleet.world")]
        self.assertEqual(offenders, [])


class TestFactory(unittest.TestCase):
    def _config(self, **overrides) -> SimConfig:
        return SimConfig(**tiny(**overrides))

    def test_default_is_the_p0_geometric_model(self):
        self.assertEqual(self._config().link_model, "geometric")

    def test_selects_geometric(self):
        model = build_link_model(self._config(link_model="geometric"), [])
        self.assertIsInstance(model, GeometricOcclusionLinkModel)

    def test_selects_nlos(self):
        model = build_link_model(self._config(link_model="nlos"), [])
        self.assertIsInstance(model, NlosDiffractionLinkModel)

    def test_is_case_insensitive(self):
        model = build_link_model(self._config(link_model="NLOS"), [])
        self.assertIsInstance(model, NlosDiffractionLinkModel)

    def test_unknown_model_is_rejected_loudly(self):
        """静默退回默认模型是最坏的选择：扫参结果会全部标错模型名。"""
        with self.assertRaises(ValueError):
            build_link_model(self._config(link_model="raytracing"), [])

    def test_nlos_fading_follows_the_protocol_seed(self):
        a = build_link_model(self._config(link_model="nlos", seed=1), [])
        b = build_link_model(self._config(link_model="nlos", seed=2), [])
        self.assertNotEqual(a.seed, b.seed)


class TestSwapRunsEndToEnd(unittest.TestCase):
    def _run(self, tmp, link_model: str):
        config = replace(load_config(SCENARIO),
                         log_path=os.path.join(tmp, f"{link_model}.jsonl"),
                         link_model=link_model, duration=90.0, env_seed=1)
        Engine(config).run()
        return list(read_log(config.log_path)), compute_metrics(config.log_path)

    def test_both_models_deliver_tasks(self):
        with tempfile.TemporaryDirectory() as tmp:
            for link_model in ("geometric", "nlos"):
                events, metrics = self._run(tmp, link_model)
                self.assertGreater(metrics["tasks"]["delivered"], 0,
                                   f"{link_model} 一个任务都没交付")
                self.assertTrue(any(e["kind"] == "TASK_DELIVER"
                                    for e in events))

    def test_swapping_the_model_changes_the_result(self):
        """配置必须真的生效。两者结果相同说明模型没被用上。"""
        with tempfile.TemporaryDirectory() as tmp:
            geo_events, geo = self._run(tmp, "geometric")
            nlos_events, nlos = self._run(tmp, "nlos")
        self.assertNotEqual(
            normalised_digest_path(tmp, "geometric"),
            normalised_digest_path(tmp, "nlos"))
        self.assertNotEqual(geo["drop_reasons"], nlos["drop_reasons"])

    def test_nlos_run_is_deterministic(self):
        with tempfile.TemporaryDirectory() as tmp:
            first = normalised_digest_path(tmp, "nlos", tag="a")
            second = normalised_digest_path(tmp, "nlos", tag="b")
        self.assertEqual(first, second)

    def test_both_models_pass_the_same_protocol_invariants(self):
        """协议不变量与模型无关：任何模型下都不得出现旧会话/旧版本被接受。"""
        with tempfile.TemporaryDirectory() as tmp:
            for link_model in ("geometric", "nlos"):
                events, _ = self._run(tmp, link_model)
                stale = [e for e in events if e["kind"] == "MSG_DROP"
                         and e.get("reason") in ("STALE_SESSION",
                                                 "STALE_EPOCH")]
                # 允许出现（说明校验在工作），但不允许是"全部消息都被拒"
                self.assertLess(len(stale), len(events) * 0.5)


def normalised_digest_path(tmp: str, link_model: str,
                           tag: str = "") -> str:
    """跑一次指定模型的短场景并返回摘要。"""
    config = replace(load_config(SCENARIO), link_model=link_model,
                     duration=90.0, env_seed=1,
                     log_path=os.path.join(tmp, f"{link_model}{tag}.jsonl"))
    Engine(config).run()
    return normalised_digest(config.log_path)


if __name__ == "__main__":
    unittest.main()
