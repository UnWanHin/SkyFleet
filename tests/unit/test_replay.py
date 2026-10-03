"""回放数据重建测试。

回放与指标必须读同一份日志，且**只读日志**：如果回放还需要仿真内存
或配置文件，那么归档一份日志就不再能复现当时的画面。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.replay import build_timeline
from skyfleet.sim.engine import Engine

CFG = None


def setUpModule():
    global CFG
    tmp = tempfile.TemporaryDirectory()
    CFG = tmp
    cfg = SimConfig(seed=11, duration=30.0, slaves=3, relays=1, tasks=5,
                    blocks=2, area=400.0,
                    log_path=str(Path(tmp.name) / "replay.jsonl"))
    Engine(cfg).run()
    CFG.path = cfg.log_path


def tearDownModule():
    CFG.cleanup()


def timeline():
    return build_timeline(CFG.path)


class TestTimeline(unittest.TestCase):
    def test_has_world_description(self):
        data = timeline()
        self.assertEqual(data["area"], 400.0)
        self.assertEqual(data["duration"], 30.0)
        self.assertEqual(data["leader_id"], "L")

    def test_buildings_are_embedded(self):
        self.assertEqual(len(timeline()["buildings"]), 4)

    def test_building_shape(self):
        building = timeline()["buildings"][0]
        self.assertEqual(len(building), 5)      # x, y, w, h, height

    def test_all_nodes_present(self):
        self.assertEqual(len(timeline()["nodes"]), 5)

    def test_nodes_have_role(self):
        nodes = timeline()["nodes"]
        self.assertEqual(nodes["L"]["role"], "LEADER")

    def test_trajectories_are_time_ordered(self):
        for node in timeline()["nodes"].values():
            times = [s[0] for s in node["samples"]]
            self.assertEqual(times, sorted(times))

    def test_trajectories_are_non_empty(self):
        for node in timeline()["nodes"].values():
            self.assertTrue(node["samples"])

    def test_tasks_are_recorded(self):
        self.assertEqual(len(timeline()["tasks"]), 5)

    def test_task_has_origin_and_destination(self):
        task = next(iter(timeline()["tasks"].values()))
        self.assertEqual(len(task["origin"]), 3)
        self.assertEqual(len(task["destination"]), 3)

    def test_status_history_present(self):
        statuses = timeline()["nodes"]["S1"]["status"]
        self.assertTrue(statuses)

    def test_rebuild_is_deterministic(self):
        self.assertEqual(timeline(), timeline())


if __name__ == "__main__":
    unittest.main()
