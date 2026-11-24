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


class TestThreeDimensionalReplay(unittest.TestCase):
    def test_visual_only_terrain_is_embedded_in_3d_scene(self):
        from skyfleet.viz.replay_3d import render_replay_3d
        from skyfleet.observe.log import EventLog
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "urban.jsonl"
            with EventLog(log, append=False) as writer:
                writer.write("SCENARIO_INIT", 0, area=150, duration=3, seed=1,
                             leader_id="L", buildings=[[40, -10, 20, 20, 60]],
                             nodes=[["L", "LEADER", [0, 0, 30]]],
                             terrain={"type": "visual-only", "bounds": [-100, -100, 180, 150],
                                      "hill": [130, 105, 10, 48],
                                      "roads": [[0, 22, 100, 12]],
                                      "park": [-82, 64, 55, 52]})
            out = Path(tmp) / "urban.html"
            render_replay_3d(log, out)
            html = out.read_text()
            self.assertIn('"terrain":{', html)
            self.assertIn('"type":"visual-only"', html)
            self.assertIn('function terrainHeight(', html)
            self.assertIn('for(const road of terrain.roads', html)

    def test_three_dimensional_replay_focus_does_not_hardcode_d5_identifiers(self):
        from skyfleet.viz.replay_3d import render_replay_3d
        from skyfleet.observe.log import EventLog
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "city.jsonl"
            with EventLog(log, append=False) as writer:
                writer.write("SCENARIO_INIT", 0, area=100, duration=3, seed=1,
                             leader_id="HQ", buildings=[[40, -10, 20, 20, 60]],
                             nodes=[["HQ", "LEADER", [0, 0, 30]],
                                    ["D8", "SLAVE", [100, 0, 30]],
                                    ["Relay7", "RELAY", [50, 60, 80]],
                                    ["Remote", "SLAVE", [4000, 4000, 80]]])
            out = Path(tmp) / "city.html"
            render_replay_3d(log, out)
            html = out.read_text()
            self.assertNotIn('id==="S5"', html)
            self.assertNotIn('id==="R3"', html)
            self.assertIn('visibleIds', html)

    def test_retried_packet_does_not_retroactively_mark_prior_tx_received(self):
        from skyfleet.viz.replay_3d import build_3d_timeline
        from skyfleet.observe.log import EventLog
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "retry.jsonl"
            with EventLog(log, append=False) as writer:
                writer.write("SCENARIO_INIT", 0, area=100, duration=10, seed=1,
                             leader_id="L", buildings=[],
                             nodes=[["L", "LEADER", [0, 0, 30]],
                                    ["S5", "SLAVE", [10, 0, 30]]])
                writer.write("MSG_TX", 1, node_id="S5", next_hop="L", msg_id="m1")
                writer.write("MSG_TX", 2, node_id="S5", next_hop="L", msg_id="m1")
                writer.write("MSG_RX", 2.1, node_id="L", src="S5", msg_id="m1")
            radio = build_3d_timeline(log)["radio"]
            self.assertEqual([r["received"] for r in radio], [False, True])

    def test_3d_cli_exports_replay_from_existing_log(self):
        import subprocess
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "city3d.html"
            result = subprocess.run([
                ".venv/bin/python", "-m", "skyfleet.cli", "replay3d",
                "--log", CFG.path, "--out", str(out)
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(out.is_file())

    def test_3d_export_contains_webgl_city_and_radio_events(self):
        import json
        from skyfleet.viz.replay_3d import render_replay_3d
        from skyfleet.observe.log import EventLog
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "events.jsonl"
            with EventLog(log, append=False) as writer:
                writer.write("SCENARIO_INIT", 0, area=120, duration=3, seed=1,
                             leader_id="L", buildings=[[40,-10,20,20,60]],
                             nodes=[["L","LEADER",[0,0,30]],
                                    ["S5","SLAVE",[100,0,30]]])
                writer.write("ENERGY_SAMPLE", 1, node_id="S5", pos=[80,0,30], energy=90)
                writer.write("MSG_TX", 1.1, node_id="S5", next_hop="L",
                             msg_id="m1", msg_type="TELEMETRY")
                writer.write("MSG_RX", 1.2, node_id="L", src="S5",
                             msg_id="m1", msg_type="TELEMETRY")
                writer.write("OCCLUSION_ENTER", 1.3, node_id="S5")
            output = Path(tmp) / "view.html"
            render_replay_3d(log, output)
            html = output.read_text()
            self.assertIn('canvas id="stage"', html)
            self.assertIn('getContext("webgl2",', html)
            self.assertIn('"buildings":[[40,-10,20,20,60]]', html)
            self.assertIn('"next_hop":"L"', html)
            self.assertIn('"received":true', html)
            self.assertIn('"samples":[[0.0,100,0,30],[1.0,80,0,30]]', html)
            self.assertNotIn('https://', html)


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
