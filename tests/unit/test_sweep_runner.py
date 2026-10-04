"""扫参执行层的测试。

用很小的场景（2 从机、6 s、2 任务）保证测试足够快，
但断言的是**执行层最容易坏的性质**：可复现性与结果拼装顺序。
"""
import csv
import json
import tempfile
import unittest
from pathlib import Path

from skyfleet.study.runner import run_sweep
from skyfleet.study.spec import SweepSpec

TINY_SCENARIO = {
    "seed": 1,
    "duration": 8.0,
    "log_path": "tiny.jsonl",
    "area": 300.0,
    "blocks": 2,
    "gap": 20.0,
    "height_range": [50.0, 100.0],
    "flight_altitude": [110.0, 150.0],
    "cruise_altitude": 140.0,
    "delivery_altitude": [80.0, 110.0],
    "slaves": 2,
    "relays": 1,
    "tasks": 2,
    "heartbeat_period": 1.0,
    "telemetry_period": 0.5,
    "motion_period": 0.1,
}


class SweepTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.scenario = self.root / "tiny.json"
        self.scenario.write_text(json.dumps(TINY_SCENARIO), encoding="utf-8")
        self.out = self.root / "study"

    def tearDown(self):
        self._tmp.cleanup()

    def spec(self, **overrides):
        payload = {
            "name": "tiny",
            "base": str(self.scenario),
            "seed": 11,
            "repeats": 2,
            "grid": {"telemetry_period": [0.5, 1.0]},
            "metrics": ["tasks.completion_rate", "loss.control_rate",
                        "heartbeat.delivery_rate"],
        }
        payload.update(overrides)
        return SweepSpec.from_dict(payload)


class TestRunSweep(SweepTestCase):
    def test_runs_every_combination_and_repeat(self):
        result = run_sweep(self.spec(), self.out)
        self.assertEqual(len(result.records), 4)
        keys = {(r.combo_index, r.repeat) for r in result.records}
        self.assertEqual(keys, {(0, 0), (0, 1), (1, 0), (1, 1)})

    def test_each_run_gets_a_distinct_seed(self):
        result = run_sweep(self.spec(), self.out)
        seeds = {r.seed for r in result.records}
        self.assertEqual(len(seeds), 4)

    def test_records_carry_their_parameters(self):
        result = run_sweep(self.spec(), self.out)
        by_combo = {r.combo_index: r.params for r in result.records}
        self.assertEqual(by_combo[0]["telemetry_period"], 0.5)
        self.assertEqual(by_combo[1]["telemetry_period"], 1.0)

    def test_metrics_are_extracted_for_every_requested_path(self):
        result = run_sweep(self.spec(), self.out)
        for record in result.records:
            self.assertEqual(
                sorted(record.metrics),
                ["heartbeat.delivery_rate", "loss.control_rate",
                 "tasks.completion_rate"])

    def test_is_deterministic_across_calls(self):
        a = run_sweep(self.spec(), self.out / "a")
        b = run_sweep(self.spec(), self.out / "b")
        self.assertEqual([r.metrics for r in a.records],
                         [r.metrics for r in b.records])

    def test_is_deterministic_across_worker_counts(self):
        """并发跑不能改变结果：这是并行扫参唯一真正危险的地方。"""
        serial = run_sweep(self.spec(), self.out / "s", jobs=1)
        parallel = run_sweep(self.spec(), self.out / "p", jobs=2)
        self.assertEqual([(r.combo_index, r.repeat, r.metrics)
                          for r in serial.records],
                         [(r.combo_index, r.repeat, r.metrics)
                          for r in parallel.records])

    def test_record_order_is_stable(self):
        result = run_sweep(self.spec(), self.out)
        order = [(r.combo_index, r.repeat) for r in result.records]
        self.assertEqual(order, sorted(order))

    def test_logs_are_written_and_parseable(self):
        result = run_sweep(self.spec(), self.out)
        for record in result.records:
            path = Path(record.log_path)
            self.assertTrue(path.exists(), path)
            first = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
            self.assertIn("kind", first)

    def test_keeping_logs_can_be_turned_off(self):
        result = run_sweep(self.spec(), self.out, keep_logs=False)
        for record in result.records:
            self.assertFalse(Path(record.log_path).exists())


class TestSweepOutputs(SweepTestCase):
    def test_csv_has_one_row_per_run(self):
        result = run_sweep(self.spec(), self.out)
        path = result.write_csv(self.out / "runs.csv")
        rows = list(csv.DictReader(path.read_text(encoding="utf-8").splitlines()))
        self.assertEqual(len(rows), 4)

    def test_csv_header_is_stable_and_ordered(self):
        result = run_sweep(self.spec(), self.out)
        path = result.write_csv(self.out / "runs.csv")
        with path.open(encoding="utf-8") as fh:
            header = next(csv.reader(fh))
        self.assertEqual(header[:4],
                         ["combo", "repeat", "seed", "telemetry_period"])
        self.assertTrue(header[-1].startswith("m:"))

    def test_csv_round_trips_parameters(self):
        result = run_sweep(self.spec(), self.out)
        path = result.write_csv(self.out / "runs.csv")
        rows = list(csv.DictReader(path.read_text(encoding="utf-8").splitlines()))
        self.assertEqual(float(rows[0]["telemetry_period"]), 0.5)
        self.assertEqual(float(rows[2]["telemetry_period"]), 1.0)

    def test_summary_has_one_row_per_combination(self):
        result = run_sweep(self.spec(), self.out)
        summary = result.summary()
        self.assertEqual(len(summary), 2)
        self.assertEqual({row["params"]["telemetry_period"]
                          for row in summary}, {0.5, 1.0})

    def test_summary_aggregates_over_repeats(self):
        result = run_sweep(self.spec(), self.out)
        row = result.summary()[0]
        stats = row["metrics"]["tasks.completion_rate"]
        self.assertEqual(stats["n"], 2)
        self.assertIsNotNone(stats["mean"])

    def test_markdown_report_is_written(self):
        result = run_sweep(self.spec(), self.out)
        path = result.write_markdown(self.out / "report.md")
        text = path.read_text(encoding="utf-8")
        self.assertIn("telemetry_period", text)
        self.assertIn("tasks.completion_rate", text)

    def test_markdown_marks_missing_metrics_instead_of_printing_nan(self):
        """没有样本的指标要显式写作"—"，不能打印 nan 让人以为是数值。"""
        result = run_sweep(self.spec(), self.out)
        text = result.write_markdown(self.out / "report.md").read_text("utf-8")
        self.assertNotIn("nan", text.lower())

    def test_spec_echo_is_saved(self):
        spec = self.spec()
        result = run_sweep(spec, self.out)
        path = result.write_spec(self.out / "spec.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(payload["name"], spec.name)
        self.assertEqual(payload["grid"], {"telemetry_period": [0.5, 1.0]})


if __name__ == "__main__":
    unittest.main()
