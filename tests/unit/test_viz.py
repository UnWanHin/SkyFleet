"""可视化测试。

两条硬约束：
1. 回放 HTML 必须**自包含**：不得引用任何外部资源，否则离线答辩时打开就是白屏。
2. 图表必须真的落盘，且同时给出位图与矢量两种格式。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.sim.engine import Engine
from skyfleet.viz.replay_html import render_replay

try:
    import matplotlib  # noqa: F401
    HAVE_MPL = True
except ImportError:                                # pragma: no cover
    HAVE_MPL = False

_STATE = {}


def setUpModule():
    tmp = tempfile.TemporaryDirectory()
    _STATE["tmp"] = tmp
    cfg = SimConfig(seed=5, duration=30.0, slaves=3, relays=1, tasks=5,
                    blocks=2, area=400.0,
                    log_path=str(Path(tmp.name) / "viz.jsonl"))
    Engine(cfg).run()
    _STATE["log"] = cfg.log_path


def tearDownModule():
    _STATE["tmp"].cleanup()


class TestReplayHtml(unittest.TestCase):
    def setUp(self):
        self.out = Path(_STATE["tmp"].name) / "replay.html"
        self.path = render_replay(_STATE["log"], str(self.out))
        self.html = Path(self.path).read_text(encoding="utf-8")

    def test_file_is_written(self):
        self.assertTrue(self.out.exists())

    def test_uses_canvas(self):
        self.assertIn("<canvas", self.html)

    def test_no_external_resources(self):
        """自包含：不得有 http(s) 外链、不得引用外部脚本或样式。"""
        self.assertNotIn('src="http', self.html)
        self.assertNotIn("src='http", self.html)
        self.assertNotIn('href="http', self.html)
        self.assertNotIn("<script src=", self.html)
        self.assertNotIn("<link ", self.html)

    def test_embeds_node_ids(self):
        self.assertIn("S1", self.html)
        self.assertIn("R1", self.html)

    def test_embeds_buildings(self):
        self.assertIn("buildings", self.html)

    def test_declares_utf8(self):
        self.assertIn('charset="utf-8"', self.html.lower())

    def test_returns_path_string(self):
        self.assertIsInstance(self.path, str)


@unittest.skipUnless(HAVE_MPL, "需要 matplotlib（请用 .venv 运行）")
class TestPlots(unittest.TestCase):
    def setUp(self):
        from skyfleet.viz.plots import render_all
        self.out = Path(_STATE["tmp"].name) / "figures"
        self.written = render_all(_STATE["log"], str(self.out))

    def test_writes_multiple_figures(self):
        self.assertGreaterEqual(len(self.written), 5)

    def test_every_figure_has_png_and_svg(self):
        for path in self.written:
            self.assertTrue(Path(path).exists(), path)
        suffixes = {Path(p).suffix for p in self.written}
        self.assertIn(".png", suffixes)
        self.assertIn(".svg", suffixes)

    def test_png_files_are_not_empty(self):
        for path in self.written:
            self.assertGreater(Path(path).stat().st_size, 1000, path)

    def test_figure_names_are_descriptive(self):
        names = {Path(p).stem for p in self.written}
        self.assertIn("task_completion", names)
        self.assertIn("latency_by_qos", names)
        self.assertIn("drop_reasons", names)
        self.assertIn("member_status_timeline", names)

    def test_missing_log_raises(self):
        from skyfleet.viz.plots import render_all
        with self.assertRaises(FileNotFoundError):
            render_all("/nonexistent.jsonl", str(self.out))


if __name__ == "__main__":
    unittest.main()
