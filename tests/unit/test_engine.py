"""仿真引擎装配测试。"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.log import normalised_digest, read_log
from skyfleet.sim.engine import Engine


def small_config(tmpdir, **overrides):
    base = dict(seed=1, duration=20.0, slaves=4, relays=1, tasks=6,
                blocks=2, area=400.0,
                log_path=str(Path(tmpdir) / "run.jsonl"))
    base.update(overrides)
    return SimConfig(**base)


class TestBuild(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = small_config(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_builds_configured_node_count(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertEqual(len(engine.nodes), 6)

    def test_has_one_leader(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertIn(self.cfg.leader_id, engine.nodes)
        self.assertEqual(engine.nodes[self.cfg.leader_id].role.value, "LEADER")

    def test_builds_buildings(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertEqual(len(engine.buildings), 4)

    def test_creates_relays(self):
        engine = Engine(self.cfg)
        engine.build()
        relays = [n for n in engine.nodes.values() if n.role.value == "RELAY"]
        self.assertEqual(len(relays), 1)


class TestRun(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = small_config(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_run_writes_log(self):
        engine = Engine(self.cfg)
        path = engine.run()
        self.assertTrue(Path(path).exists())
        self.assertGreater(len(list(read_log(path))), 10)

    def test_log_records_all_nodes_joining(self):
        path = Engine(self.cfg).run()
        joined = {r.get("node_id") for r in read_log(path)
                  if r["kind"] == "NODE_JOIN"}
        self.assertEqual(len(joined), 6)

    def test_log_records_task_creation(self):
        path = Engine(self.cfg).run()
        created = [r for r in read_log(path) if r["kind"] == "TASK_CREATE"]
        self.assertEqual(len(created), 6)

    def test_log_records_message_traffic(self):
        path = Engine(self.cfg).run()
        kinds = {r["kind"] for r in read_log(path)}
        self.assertIn("MSG_TX", kinds)

    def test_log_records_drops_with_reason(self):
        path = Engine(self.cfg).run()
        drops = [r for r in read_log(path) if r["kind"] == "MSG_DROP"]
        self.assertTrue(all(r.get("reason") for r in drops))

    def test_run_is_deterministic(self):
        first = Engine(self.cfg).run()
        digest_a = normalised_digest(first)
        second = Engine(self.cfg).run()
        self.assertEqual(digest_a, normalised_digest(second))

    def test_different_seed_changes_outcome(self):
        a = Engine(small_config(self._tmp.name, seed=1)).run()
        digest_a = normalised_digest(a)
        b = Engine(small_config(self._tmp.name, seed=2)).run()
        self.assertNotEqual(digest_a, normalised_digest(b))

    def test_rerun_truncates_previous_log(self):
        Engine(self.cfg).run()
        path = Engine(self.cfg).run()
        joined = [r for r in read_log(path) if r["kind"] == "NODE_JOIN"]
        self.assertEqual(len(joined), 6)


if __name__ == "__main__":
    unittest.main()
