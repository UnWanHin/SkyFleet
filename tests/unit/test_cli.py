"""命令行接口测试。"""
import json
import tempfile
import unittest
from pathlib import Path

from skyfleet.cli import main


def write_config(tmpdir, **overrides):
    cfg = {
        "seed": 3, "duration": 12.0, "slaves": 3, "relays": 1, "tasks": 4,
        "blocks": 2, "area": 400.0,
        "log_path": str(Path(tmpdir) / "cli.jsonl"),
    }
    cfg.update(overrides)
    path = Path(tmpdir) / "scenario.json"
    path.write_text(json.dumps(cfg), encoding="utf-8")
    return str(path)


class TestRunCommand(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self._tmp.cleanup()

    def test_run_returns_zero(self):
        cfg = write_config(self._tmp.name)
        self.assertEqual(main(["run", "--config", cfg]), 0)

    def test_run_creates_log_file(self):
        cfg = write_config(self._tmp.name)
        main(["run", "--config", cfg])
        self.assertTrue((Path(self._tmp.name) / "cli.jsonl").exists())

    def test_duration_override_shortens_run(self):
        cfg = write_config(self._tmp.name)
        out = Path(self._tmp.name) / "short.jsonl"
        main(["run", "--config", cfg, "--duration", "3.0",
              "--log", str(out)])
        events = [json.loads(l) for l in out.read_text().splitlines() if l]
        self.assertLess(max(e["t"] for e in events), 4.0)

    def test_seed_override_changes_output(self):
        cfg = write_config(self._tmp.name)
        a = Path(self._tmp.name) / "a.jsonl"
        b = Path(self._tmp.name) / "b.jsonl"
        main(["run", "--config", cfg, "--seed", "1", "--log", str(a)])
        main(["run", "--config", cfg, "--seed", "2", "--log", str(b)])
        self.assertNotEqual(a.read_text(), b.read_text())

    def test_missing_config_returns_nonzero(self):
        self.assertNotEqual(main(["run", "--config", "/nope.json"]), 0)


class TestDispatchCommand(unittest.TestCase):
    def test_dispatch_runs_selected_nodes_and_exports_replay(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = write_config(folder, duration=8.0, tasks=0, relays=0,
                               slaves=2, area=120.0)
            out = Path(folder) / "dispatch"
            self.assertEqual(main([
                "dispatch", "--config", cfg, "--nodes", "S1,S2",
                "--destination", "60,0,30", "--at", "2.5",
                "--group", "east", "--out", str(out),
            ]), 0)
            log = out / "dispatch.jsonl"
            self.assertTrue(log.is_file())
            records = [json.loads(line) for line in log.read_text().splitlines()]
            issued = [r for r in records if r["kind"] == "CONTROL_ISSUED"]
            self.assertEqual({r["node_id"] for r in issued}, {"S1", "S2"})
            accepted = [r for r in records if r["kind"] == "CONTROL_ACCEPTED"]
            self.assertEqual({r["node_id"] for r in accepted}, {"S1", "S2"})
            self.assertTrue((out / "dispatch_3d.html").is_file())
            from skyfleet.observe.replay import build_timeline
            timeline = build_timeline(log)
            self.assertEqual(timeline["groups"]["S1"], "east")
            self.assertTrue(any(event[1] == "TOPOLOGY_CHANGE"
                                for event in timeline["events"]))
            self.assertTrue(any(event[1] == "CONTROL_ACCEPTED"
                                for event in timeline["events"]))


class TestMetricsCommand(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        cfg = write_config(self._tmp.name, duration=60.0)
        main(["run", "--config", cfg])
        self.log = str(Path(self._tmp.name) / "cli.jsonl")

    def tearDown(self):
        self._tmp.cleanup()

    def test_metrics_returns_zero(self):
        self.assertEqual(main(["metrics", "--log", self.log]), 0)

    def test_metrics_on_missing_log_returns_nonzero(self):
        self.assertNotEqual(main(["metrics", "--log", "/nope.jsonl"]), 0)


class TestReplay3DCommand(unittest.TestCase):
    def test_all_exports_3d_alongside_2d(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = write_config(folder, duration=2.0)
            out = Path(folder) / "out"
            self.assertEqual(main(["all", "--config", cfg, "--out", str(out)]), 0)
            self.assertTrue((out / "replay" / "cli_3d.html").is_file())


class TestArgumentParsing(unittest.TestCase):
    def test_no_command_returns_nonzero(self):
        self.assertNotEqual(main([]), 0)

    def test_unknown_command_returns_nonzero(self):
        self.assertNotEqual(main(["frobnicate"]), 0)


if __name__ == "__main__":
    unittest.main()
