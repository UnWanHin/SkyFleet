"""配置加载测试。

关键约束：所有门限参数必须来自配置文件，不得硬编码在代码里，
否则第 6.2 节的状态机门限与指标阈值都无法标定。
"""
import json
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import (SimConfig, apply_overrides, baseline_path,
                                    load_config)

REQUIRED_THRESHOLD_KEYS = (
    "n_degraded", "n_suspect", "t_fail", "q_degraded",
    "h_up", "h_down", "t_hold",
)


class TestBaselineScenario(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(baseline_path())

    def test_baseline_matches_spec(self):
        self.assertEqual(self.cfg.slaves, 8)
        self.assertEqual(self.cfg.relays, 2)
        self.assertEqual(self.cfg.tasks, 20)
        self.assertEqual(self.cfg.area, 500.0)
        self.assertEqual(self.cfg.duration, 600.0)

    def test_all_thresholds_come_from_config(self):
        raw = json.loads(Path(baseline_path()).read_text(encoding="utf-8"))
        for key in REQUIRED_THRESHOLD_KEYS:
            self.assertIn(key, raw)

    def test_height_range_is_tuple(self):
        self.assertIsInstance(self.cfg.height_range, tuple)
        self.assertEqual(len(self.cfg.height_range), 2)

    def test_total_nodes(self):
        self.assertEqual(self.cfg.total_nodes, 11)


class TestLoading(unittest.TestCase):
    def test_unknown_key_raises(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.json"
            p.write_text(json.dumps({"slaves": 1, "not_a_field": 3}),
                         encoding="utf-8")
            with self.assertRaises(ValueError):
                load_config(p)

    def test_missing_file_raises(self):
        with self.assertRaises(FileNotFoundError):
            load_config("/nonexistent/scenario.json")

    def test_partial_config_uses_defaults(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "min.json"
            p.write_text(json.dumps({"seed": 99}), encoding="utf-8")
            cfg = load_config(p)
            self.assertEqual(cfg.seed, 99)
            self.assertEqual(cfg.slaves, SimConfig().slaves)

    def test_invalid_json_raises(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "broken.json"
            p.write_text("{not json", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_config(p)


class TestOverrides(unittest.TestCase):
    def test_seed_override(self):
        cfg = apply_overrides(SimConfig(), seed=123)
        self.assertEqual(cfg.seed, 123)

    def test_duration_override(self):
        cfg = apply_overrides(SimConfig(), duration=5.0)
        self.assertEqual(cfg.duration, 5.0)

    def test_none_leaves_value_unchanged(self):
        base = SimConfig(seed=7, duration=42.0)
        cfg = apply_overrides(base, seed=None, duration=None)
        self.assertEqual(cfg.seed, 7)
        self.assertEqual(cfg.duration, 42.0)

    def test_override_returns_new_object(self):
        base = SimConfig()
        self.assertIsNot(apply_overrides(base, seed=1), base)


if __name__ == "__main__":
    unittest.main()
