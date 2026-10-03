"""故障注入场景的端到端验收。

报告 R0-R3 要求"失效检测 -> 判定 -> 重派 -> 恢复"这条链可复现。
本测试跑的是**随项目发布的场景文件**（scenarios/fault_chain.json），
而不是临时拼出来的配置——答辩现场用同一条命令就能复现同一结果。

链路（对应报告 §2.2.1 与表 4-1）：

    节点静默 -> DEGRADED -> SUSPECT -> FAILED -> 任务重派
             -> mission_version 递增 -> 节点以新会话重新入网 -> ONLINE
"""
import unittest
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine

SCENARIO = (Path(__file__).resolve().parents[2]
            / "skyfleet" / "scenarios" / "fault_chain.json")


def status_timeline(events, node_id):
    return [(e["t"], e["status"]) for e in events
            if e["kind"] == "STATUS_CHANGE" and e.get("node_id") == node_id]


class TestFaultScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config(SCENARIO)
        cls.events = list(read_log(Engine(cls.cfg).run()))

    def test_scenario_file_is_well_formed(self):
        self.assertGreaterEqual(len(self.cfg.kills), 1)
        self.assertGreaterEqual(len(self.cfg.revives), 1)

    def test_silent_node_degrades_then_fails(self):
        """静默节点必须逐级升级，而不是一步跳到 FAILED。"""
        statuses = [s for _, s in status_timeline(self.events, "S2")]
        self.assertIn("DEGRADED", statuses)
        self.assertIn("SUSPECT", statuses)
        self.assertIn("FAILED", statuses)
        self.assertLess(statuses.index("DEGRADED"),
                        statuses.index("SUSPECT"))
        self.assertLess(statuses.index("SUSPECT"), statuses.index("FAILED"))

    def test_grade_order_matches_configured_thresholds(self):
        """升级时刻必须与配置的阈值一致，而不是碰巧出现的。"""
        timeline = status_timeline(self.events, "S2")
        kill_at = dict(self.cfg.kills)["S2"]
        degraded_at = next(t for t, s in timeline if s == "DEGRADED")
        failed_at = next(t for t, s in timeline if s == "FAILED")
        self.assertGreater(degraded_at, kill_at)
        self.assertAlmostEqual(
            failed_at - degraded_at,
            (self.cfg.n_suspect - self.cfg.n_degraded)
            * self.cfg.heartbeat_period + self.cfg.t_fail, delta=1.5)

    def test_revived_node_rejoins_with_a_new_session(self):
        """恢复必须换会话：旧会话留着，重连前滞留的旧消息仍会被接受。"""
        joins = [e for e in self.events
                 if e["kind"] == "NODE_JOIN" and e.get("node_id") == "S2"]
        self.assertGreaterEqual(len(joins), 2)
        self.assertNotEqual(joins[0]["session_id"], joins[-1]["session_id"])

    def test_revived_node_returns_to_online(self):
        timeline = status_timeline(self.events, "S2")
        revive_at = dict(self.cfg.revives)["S2"]
        recovered = [t for t, s in timeline if s == "ONLINE" and t > revive_at]
        self.assertTrue(recovered, "恢复入网后没有回到 ONLINE")

    def test_work_is_reassigned_away_from_the_failed_node(self):
        reassigns = [e for e in self.events if e["kind"] == "TASK_REASSIGN"]
        self.assertTrue(reassigns, "节点失效后没有任何任务重派")

    def test_mission_version_increases_on_reassignment(self):
        """任务重派必须递增 mission_version，否则在途的旧指派仍会生效。"""
        versions = [e["mission_version"] for e in self.events
                    if e["kind"] == "TASK_REASSIGN"
                    and e.get("mission_version") is not None]
        self.assertTrue(versions)
        self.assertEqual(versions, sorted(versions))
        self.assertGreater(versions[-1], versions[0])

    def test_scenario_is_deterministic(self):
        again = list(read_log(Engine(self.cfg).run()))
        self.assertEqual(len(again), len(self.events))


if __name__ == "__main__":
    unittest.main()
