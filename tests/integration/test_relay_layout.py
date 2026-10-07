"""切片 C：中继布点可比较化。

背景（P1 的失败教训）：扫参发现「中继门限在两轮扫描中均无可测影响」。
根因不是门限，而是**布点**——原布点让所有中继要么都够得着、要么都够
不着，门限因此调不动。在没有可比较的布点之前，任何「优化」都不可验证。

本文件只验证「布点确实可比较」这一件事，**不**验证哪种布点更优——
后者需要跨多个 `env_seed` 的独立扫参（见分析文档 §11 的教训：单布局
方差可解释约 94% 的完成率方差，不跨布局比较必然得出错误结论）。
"""

from __future__ import annotations

import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine

SCENARIO = Path("skyfleet/scenarios/baseline_small.json")

#: 规格 §3.2 规定的三种布点。
LAYOUTS = ("ring", "grid", "farthest")


def build(layout: str, tmp: str, **overrides):
    """按指定布点装配引擎（不运行）。"""
    cfg = replace(load_config(SCENARIO),
                  log_path=str(Path(tmp) / f"{layout}.jsonl"),
                  relay_layout=layout, **overrides)
    return cfg, Engine(cfg).build()


def relay_positions(engine) -> dict[str, tuple[float, float, float]]:
    return {nid: engine.nodes[nid].pos for nid in sorted(engine.relay_ids)}


class TestRelayLayoutsAreDistinct(unittest.TestCase):
    def test_ring_is_the_default(self):
        """默认必须是 `ring`：向后兼容，且 golden digest 依赖它。"""
        self.assertEqual(load_config(SCENARIO).relay_layout, "ring")

    def test_ring_changes_nothing(self):
        """显式写 `relay_layout="ring"` 与不写必须完全等价。

        这是「默认布点逐位不变」的直接证据——digest 测试是端到端的，
        这一条是它的定位版本：一旦不相等，能立刻指出是布点而非协议漂移。
        """
        with tempfile.TemporaryDirectory() as tmp:
            _, implicit = build("ring", tmp)
            explicit_cfg = replace(load_config(SCENARIO),
                                   log_path=str(Path(tmp) / "rings.jsonl"),
                                   relay_layout="ring")
            explicit = Engine(explicit_cfg).build()
        self.assertEqual(relay_positions(implicit), relay_positions(explicit))

    def test_the_three_layouts_place_relays_differently(self):
        """三种布点必须产生**不同**的中继坐标。

        否则「可比较」是假的：看起来在做对照实验，实际三组跑的是同一套
        布点，结论自然是「无差异」——这正是 P1 那一轮扫参的表象。
        """
        with tempfile.TemporaryDirectory() as tmp:
            places = {}
            for layout in LAYOUTS:
                _, engine = build(layout, tmp)
                places[layout] = relay_positions(engine)
        for a, b in (("ring", "grid"), ("ring", "farthest"),
                     ("grid", "farthest")):
            self.assertNotEqual(
                places[a], places[b],
                f"{a} 与 {b} 的中继坐标完全相同——布点不可比较")

    def test_relays_stay_inside_the_area(self):
        """所有布点都必须把中继放在任务域内。

        跑到域外的中继在几何上「更远」，却对任何一条链路都没有帮助。
        """
        for layout in LAYOUTS:
            with self.subTest(layout=layout), tempfile.TemporaryDirectory() as tmp:
                cfg, engine = build(layout, tmp)
                area = cfg.area
                for nid, (x, y, z) in relay_positions(engine).items():
                    self.assertGreaterEqual(x, 0.0, f"{nid} x 越界")
                    self.assertLessEqual(x, area, f"{nid} x 越界")
                    self.assertGreaterEqual(y, 0.0, f"{nid} y 越界")
                    self.assertLessEqual(y, area, f"{nid} y 越界")

    def test_relays_are_all_at_high_altitude_in_every_layout(self):
        """中继在任何布点下都必须在高空。

        中继是通信基础设施：低空意味着更多楼宇遮挡，恰好抹掉它存在的
        理由。若某个布点顺手把中继放低，比较的就不只是「布点」了。
        """
        for layout in LAYOUTS:
            with self.subTest(layout=layout), tempfile.TemporaryDirectory() as tmp:
                cfg, engine = build(layout, tmp)
                _, high = cfg.flight_altitude
                for nid, (_, _, z) in relay_positions(engine).items():
                    self.assertAlmostEqual(z, high, places=9,
                                           msg=f"{nid} 不在高空")

    def test_relays_stay_distinct_in_every_layout(self):
        """同一布点内，两个中继不得落在同一点。

        重合的中继等价于一个中继，会静默地把「2 个中继」的实验变成
        「1 个中继」，而配置上看起来毫无异常。
        """
        for layout in LAYOUTS:
            with self.subTest(layout=layout), tempfile.TemporaryDirectory() as tmp:
                _, engine = build(layout, tmp)
                places = list(relay_positions(engine).values())
                xy = [(round(x, 6), round(y, 6)) for x, y, _ in places]
                self.assertEqual(len(set(xy)), len(xy),
                                 f"{layout} 布点下有中继重合：{xy}")


class TestGridBreaksTheRingSymmetry(unittest.TestCase):
    def test_grid_is_not_a_ring(self):
        """`grid` 必须真的消除「环」带来的对称偏置。

        `ring` 把所有中继放在同一半径上，于是它们到主节点的距离几乎
        相同——这会让「中继够不够得着」变成一个近乎全有或全无的性质，
        门限因此调不动（P1 的结论）。`grid` 的半径必须明显分散。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg, ring_engine = build("ring", tmp)
            _, grid_engine = build("grid", tmp)
        center = (cfg.area / 2, cfg.area / 2)

        def radii(engine):
            return [math.dist((x, y), center)
                    for x, y, _ in relay_positions(engine).values()]

        ring_radii, grid_radii = radii(ring_engine), radii(grid_engine)
        ring_spread = max(ring_radii) - min(ring_radii)
        grid_spread = max(grid_radii) - min(grid_radii)
        self.assertGreater(
            grid_spread, ring_spread,
            f"grid 的半径分散度（{grid_spread:.1f} m）没有超过 ring"
            f"（{ring_spread:.1f} m）——对称性没有被打破，grid 切片"
            f"失去了它唯一的存在理由")


class TestFarthestIsAnUpperBoundReference(unittest.TestCase):
    def test_farthest_reaches_at_least_as_far_as_ring(self):
        """★ `farthest` 作为**可达性上界参考**：其到主节点的平均距离
        必须不低于 `ring`。

        若这条断言失败，说明「最远布点 = 覆盖上界」的直觉是错的——
        那本身就是一个要写进报告的结论，而不是把断言改掉了事。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg, ring_engine = build("ring", tmp)
            _, far_engine = build("farthest", tmp)
        center = (cfg.area / 2, cfg.area / 2)

        def mean_r(engine):
            rs = [math.dist((x, y), center)
                  for x, y, _ in relay_positions(engine).values()]
            return sum(rs) / len(rs)

        self.assertGreaterEqual(
            mean_r(far_engine) + 1e-9, mean_r(ring_engine),
            f"farthest 的平均半径 {mean_r(far_engine):.1f} m 小于 ring 的 "
            f"{mean_r(ring_engine):.1f} m——「最远布点」并没有更远，"
            f"该结论需如实写入报告")


class TestLayoutValidation(unittest.TestCase):
    def test_unknown_layout_is_rejected(self):
        """未知布点必须报错，而不是静默退回默认。

        拼错 `relay_layout` 却静默跑默认布点，会让整轮扫参的结论都是
        假的——配置层已经因为同样的理由对未知字段一律报错
        （见 `coerce_overrides` 的说明）。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "bad.jsonl"),
                          relay_layout="rihng")
            with self.assertRaises(ValueError) as ctx:
                Engine(cfg).build()
        self.assertIn("rihng", str(ctx.exception))
        self.assertIn("relay_layout", str(ctx.exception))

    def test_every_declared_layout_is_accepted(self):
        """`LAYOUTS` 里的每一种都必须能被装配——防止文档列了、代码没实现。"""
        with tempfile.TemporaryDirectory() as tmp:
            for layout in LAYOUTS:
                with self.subTest(layout=layout):
                    build(layout, tmp)


class TestLayoutsRunEndToEnd(unittest.TestCase):
    def test_every_layout_produces_a_valid_log(self):
        """三种布点都必须能跑完并写出可解析的日志。

        布点改动如果让中继跑出界或让路由成环，症状会是日志为空或事件
        种类缺失——而不是抛异常。这里做最低限度的端到端体检。
        """
        for layout in LAYOUTS:
            with self.subTest(layout=layout), tempfile.TemporaryDirectory() as tmp:
                cfg = replace(load_config(SCENARIO),
                              log_path=str(Path(tmp) / f"{layout}.jsonl"),
                              relay_layout=layout, duration=60.0)
                engine = Engine(cfg).build()
                relay_ids = set(engine.relay_ids)
                engine.run()
                events = list(read_log(cfg.log_path))
                kinds = {e["kind"] for e in events}
                self.assertIn("SCENARIO_INIT", kinds)
                self.assertIn("MSG_TX", kinds, f"{layout} 下没有任何发送")
                # 中继必须真的被用到，否则「布点」对结果没有影响通道。
                relayed = [e for e in events
                           if e["kind"] == "MSG_TX"
                           and e.get("node_id") in relay_ids]
                self.assertTrue(
                    relayed, f"{layout} 布点下中继完全没有参与转发")


if __name__ == "__main__":
    unittest.main()
