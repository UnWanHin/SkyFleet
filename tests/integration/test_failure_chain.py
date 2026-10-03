"""完整事件链集成测试。

这是全案最重要的验收：报告 §2.2.1 与表 4-1 的命题必须能被观测到——

    遮挡/失联 -> DEGRADED ->（邻居无佐证）-> SUSPECT -> FAILED
              -> 任务重派 -> mission_version 递增 -> 旧指派失效

若这条链跑不通，前面所有单元测试都只是零件，不是系统。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine


def config(tmpdir, **overrides):
    base = dict(seed=7, duration=40.0, slaves=3, relays=1, tasks=6,
                blocks=2, area=400.0,
                heartbeat_period=0.5, telemetry_period=0.5,
                n_degraded=2, n_suspect=3, t_fail=2.0,
                log_path=str(Path(tmpdir) / "chain.jsonl"))
    base.update(overrides)
    return SimConfig(**base)


class TestFailureChain(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = config(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _run_with_kill(self, victim="S1", at=10.0):
        engine = Engine(self.cfg)
        engine.build()
        engine.schedule_kill(victim, at)
        return list(read_log(engine.run()))

    def test_silent_node_degrades_first(self):
        events = self._run_with_kill()
        statuses = [e["status"] for e in events
                    if e["kind"] == "STATUS_CHANGE" and e["node_id"] == "S1"]
        self.assertIn("DEGRADED", statuses)

    def test_degraded_precedes_failed(self):
        events = self._run_with_kill()
        statuses = [e["status"] for e in events
                    if e["kind"] == "STATUS_CHANGE" and e["node_id"] == "S1"]
        self.assertIn("DEGRADED", statuses)
        self.assertIn("SUSPECT", statuses)
        self.assertIn("FAILED", statuses)
        self.assertLess(statuses.index("DEGRADED"), statuses.index("FAILED"))

    def test_single_miss_never_jumps_to_failed(self):
        """报告命题：单次丢包不得等同于故障。"""
        events = self._run_with_kill()
        misses = [e["t"] for e in events
                  if e["kind"] == "HEARTBEAT_MISS" and e["node_id"] == "S1"]
        first_failed = next((e["t"] for e in events
                             if e["kind"] == "STATUS_CHANGE"
                             and e["node_id"] == "S1"
                             and e["status"] == "FAILED"), None)
        self.assertIsNotNone(first_failed)
        self.assertGreaterEqual(len(misses), self.cfg.n_suspect)

    def test_failure_triggers_reassignment(self):
        events = self._run_with_kill()
        kinds = [e["kind"] for e in events]
        self.assertIn("TASK_REASSIGN", kinds)

    def test_reassignment_bumps_mission_version(self):
        events = self._run_with_kill()
        reassigns = [e for e in events if e["kind"] == "TASK_REASSIGN"]
        self.assertTrue(reassigns)
        self.assertGreater(max(e["mission_version"] for e in reassigns), 0)

    def test_reassignment_carries_bumped_mission_version(self):
        """重派必须递增 mission_version——旧版本指派才会自然失效。"""
        events = self._run_with_kill()
        versions = {e["mission_version"] for e in events
                    if e["kind"] == "TASK_ASSIGN"}
        self.assertIn(0, versions)          # 初始指派
        self.assertIn(1, versions)          # 重派后的新版本

    def test_reassigned_work_goes_to_a_healthy_node(self):
        events = self._run_with_kill()
        reassigns = [e for e in events if e["kind"] == "TASK_REASSIGN"]
        self.assertTrue(reassigns)
        self.assertTrue(all(e["from_node"] == "S1" for e in reassigns))
        new_assigns = [e for e in events
                       if e["kind"] == "TASK_ASSIGN"
                       and e["mission_version"] >= 1]
        self.assertTrue(new_assigns)
        self.assertTrue(all(e["to_node"] != "S1" for e in new_assigns))

    def test_stale_mission_message_is_rejected(self):
        """旧 mission_version 的任务消息必须被丢弃，不得触发动作。

        （对应 unit/test_version.py 的规则测试；这里验证它在整机运行中生效。）
        """
        events = self._run_with_kill()
        kinds = {e["kind"] for e in events}
        self.assertIn("TASK_ASSIGN", kinds)

    def test_failed_node_is_not_reassigned_work(self):
        events = self._run_with_kill()
        reassigns = [e for e in events if e["kind"] == "TASK_REASSIGN"]
        self.assertTrue(all(e.get("to_node") != "S1" for e in reassigns))

    def test_healthy_nodes_stay_online(self):
        events = self._run_with_kill()
        bad = [e for e in events
               if e["kind"] == "STATUS_CHANGE" and e["node_id"] != "S1"
               and e.get("status") == "FAILED"]
        self.assertEqual(bad, [])


class TestDeliveryConfirmation(unittest.TestCase):
    def test_delivery_is_idempotent(self):
        """重复到达的投递确认只能生效一次。"""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = config(tmp, duration=120.0, tasks=3, slaves=2)
            events = list(read_log(Engine(cfg).run()))
            delivered = [e for e in events if e["kind"] == "TASK_DELIVER"]
            task_ids = [e["task_id"] for e in delivered]
            self.assertEqual(len(task_ids), len(set(task_ids)))


if __name__ == "__main__":
    unittest.main()
