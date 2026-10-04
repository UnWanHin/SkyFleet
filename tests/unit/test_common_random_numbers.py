"""共同随机数（CRN）：环境随机性与协议随机性必须能分开。

标定实验的信噪比几乎完全取决于这一条。若每次重复都重抽楼群，那么
"参数 A 与参数 B 的差异"里混进了"城市布局的差异"，而布局对遮挡的影响
远大于多数协议参数——实测中间隔会大到区间比均值还宽，扫参结论无法成立。

正确做法是固定环境（同一座城市、同一组初始站位），只让协议侧的随机性
（冲突判定、重传抖动、时钟漂移）随重复变化。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine


def config(tmpdir, **overrides):
    base = dict(seed=1, duration=1.0, slaves=3, relays=1, tasks=0,
                blocks=2, area=400.0, motion_period=0.1,
                log_path=str(Path(tmpdir) / "crn.jsonl"))
    base.update(overrides)
    return SimConfig(**base)


class TestCommonRandomNumbers(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def _buildings(self, **overrides):
        engine = Engine(config(self.root, **overrides)).build()
        return [(b.id, round(b.height, 6))
                for b in sorted(engine.buildings, key=lambda b: b.id)]

    def _positions(self, **overrides):
        engine = Engine(config(self.root, **overrides)).build()
        return {nid: tuple(round(v, 6) for v in node.pos)
                for nid, node in sorted(engine.nodes.items())}

    def test_same_env_seed_gives_the_same_city_across_protocol_seeds(self):
        a = self._buildings(seed=1, env_seed=99)
        b = self._buildings(seed=2, env_seed=99)
        c = self._buildings(seed=12345, env_seed=99)
        self.assertEqual(a, b)
        self.assertEqual(a, c)

    def test_different_env_seed_gives_a_different_city(self):
        a = self._buildings(seed=1, env_seed=1)
        b = self._buildings(seed=1, env_seed=2)
        self.assertNotEqual(a, b)

    def test_same_env_seed_gives_the_same_initial_positions(self):
        a = self._positions(seed=1, env_seed=7)
        b = self._positions(seed=2, env_seed=7)
        self.assertEqual(a, b)

    def test_env_seed_defaults_to_the_main_seed(self):
        """不显式指定时必须退回原行为，不能悄悄改变已有场景的结果。"""
        explicit = self._buildings(seed=42, env_seed=42)
        implicit = self._buildings(seed=42, env_seed=None)
        self.assertEqual(explicit, implicit)

    def test_fixing_env_seed_shrinks_metric_variance(self):
        """CRN 的作用必须体现在**指标方差**上，而不只是"楼一样"。

        这是本测试的重点：若方差没有下降，说明还有别的环境随机性
        没被 env_seed 管住（例如节点初始站位或任务点抽样）。
        """
        def occlusion_spread(**overrides):
            values = []
            for seed in (1, 2, 3, 4):
                cfg = config(self.root, duration=30.0, tasks=6,
                             heartbeat_period=0.5, telemetry_period=0.5,
                             **{**overrides, "seed": seed})
                events = list(read_log(Engine(cfg).run()))
                values.append(sum(1 for e in events
                                  if e["kind"] == "MSG_DROP"
                                  and e.get("reason") == "OCCLUDED"))
            return max(values) - min(values)

        varied = occlusion_spread(env_seed=None)
        fixed = occlusion_spread(env_seed=7)
        self.assertLess(fixed, varied,
                        f"固定环境后方差未下降：变={varied} 固定={fixed}")


if __name__ == "__main__":
    unittest.main()
