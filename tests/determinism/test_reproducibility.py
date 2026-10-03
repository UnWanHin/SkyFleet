"""确定性验收。

这是全案的第一验收项：同一种子跑两次，规范化日志摘要必须逐字节相同。
它之所以能成立，靠的是四件具体的事，任何一件坏掉都会失败：

1. 随机数只来自 RngRegistry 的具名独立流（不用内置 hash()）；
2. 事件排序是 (t, tie)，同一时刻的先后有确定顺序；
3. 信道竞争在同一时刻统一结算，而不是逐个判定；
4. 消息 ID 由节点本地确定性生成，不依赖跨运行的模块级计数器。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.kernel.rng import RngRegistry, derive_seed
from skyfleet.observe.log import normalised_digest, read_log
from skyfleet.sim.engine import Engine


def config(tmpdir, name, **overrides):
    base = dict(seed=42, duration=40.0, slaves=4, relays=1, tasks=8,
                blocks=2, area=400.0, heartbeat_period=0.5,
                telemetry_period=0.5,
                log_path=str(Path(tmpdir) / name))
    base.update(overrides)
    return SimConfig(**base)


class TestSameSeedReproduces(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, name, **overrides):
        return Engine(config(self._tmp.name, name, **overrides)).run()

    def test_log_digest_is_identical(self):
        a = normalised_digest(self._run("a.jsonl"))
        b = normalised_digest(self._run("b.jsonl"))
        self.assertEqual(a, b)

    def test_log_line_count_is_identical(self):
        first = list(read_log(self._run("a.jsonl")))
        second = list(read_log(self._run("b.jsonl")))
        self.assertEqual(len(first), len(second))

    def test_log_is_identical_line_by_line(self):
        first = list(read_log(self._run("a.jsonl")))
        second = list(read_log(self._run("b.jsonl")))
        for index, (x, y) in enumerate(zip(first, second)):
            self.assertEqual(x, y, f"第 {index} 行不一致")

    def test_different_seed_diverges(self):
        a = normalised_digest(self._run("a.jsonl", seed=1))
        b = normalised_digest(self._run("b.jsonl", seed=2))
        self.assertNotEqual(a, b)

    def test_determinism_survives_longer_run(self):
        a = normalised_digest(self._run("a.jsonl", duration=90.0))
        b = normalised_digest(self._run("b.jsonl", duration=90.0))
        self.assertEqual(a, b)


class TestSeedDerivation(unittest.TestCase):
    def test_derived_seed_is_stable(self):
        self.assertEqual(derive_seed(42, "channel:collision"),
                         derive_seed(42, "channel:collision"))

    def test_named_streams_are_independent(self):
        self.assertNotEqual(derive_seed(42, "channel:collision"),
                            derive_seed(42, "env:placement"))

    def test_registry_returns_same_instance_for_same_name(self):
        registry = RngRegistry(1)
        self.assertIs(registry.get("x"), registry.get("x"))

    def test_registry_streams_do_not_interfere(self):
        a, b = RngRegistry(7), RngRegistry(7)
        a.get("left").random()
        self.assertEqual(a.get("right").random(), b.get("right").random())


if __name__ == "__main__":
    unittest.main()
