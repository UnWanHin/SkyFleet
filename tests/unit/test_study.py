"""P1 参数标定框架的单元测试。

标定这件事最容易出的错不是算错，而是**不可复现**：
换了 worker 数、换了参数顺序，结果就变。所以这里的测试重点在
"同一份扫描规格必须永远给出同一批数字"。
"""
import json
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig, baseline_path, load_config
from skyfleet.study.spec import (
    SweepSpec, coerce_params, expand_grid, get_path, run_seed,
)
from skyfleet.study.stats import aggregate, t_critical


class TestExpandGrid(unittest.TestCase):
    def test_single_parameter(self):
        combos = expand_grid({"slot_duration": [0.001, 0.002]})
        self.assertEqual(combos, [{"slot_duration": 0.001},
                                  {"slot_duration": 0.002}])

    def test_cartesian_product(self):
        combos = expand_grid({"a": [1, 2], "b": [10, 20]})
        self.assertEqual(len(combos), 4)
        self.assertEqual([c["a"] for c in combos], [1, 1, 2, 2])
        self.assertEqual([c["b"] for c in combos], [10, 20, 10, 20])

    def test_order_does_not_depend_on_dict_insertion_order(self):
        """参数顺序换了，组合顺序不能变——否则并行跑出来的行次序会漂。"""
        a = expand_grid({"x": [1, 2], "y": [3]})
        b = expand_grid({"y": [3], "x": [1, 2]})
        self.assertEqual(a, b)

    def test_empty_grid_yields_one_empty_combo(self):
        self.assertEqual(expand_grid({}), [{}])

    def test_empty_value_list_yields_no_combo(self):
        self.assertEqual(expand_grid({"a": []}), [])

    def test_raises_on_non_sequence(self):
        with self.assertRaises(ValueError):
            expand_grid({"a": 1})


class TestRunSeed(unittest.TestCase):
    def test_deterministic(self):
        self.assertEqual(run_seed(1, 0, 0), run_seed(1, 0, 0))

    def test_combination_and_repeat_both_matter(self):
        seeds = {run_seed(1, i, r) for i in range(3) for r in range(3)}
        self.assertEqual(len(seeds), 9)

    def test_master_seed_matters(self):
        self.assertNotEqual(run_seed(1, 0, 0), run_seed(2, 0, 0))

    def test_fits_in_63_bits(self):
        for i in range(5):
            self.assertLess(run_seed(7, i, i), 2 ** 63)


class TestGetPath(unittest.TestCase):
    def test_nested(self):
        self.assertEqual(get_path({"a": {"b": {"c": 3}}}, "a.b.c"), 3)

    def test_missing_returns_none(self):
        self.assertIsNone(get_path({"a": {}}, "a.b.c"))
        self.assertIsNone(get_path({}, "x"))

    def test_none_is_preserved_not_confused_with_missing(self):
        self.assertIsNone(get_path({"a": None}, "a"))


class TestCoerceParams(unittest.TestCase):
    def test_tuple_fields_become_tuples(self):
        out = coerce_params({"height_range": [10, 20]})
        self.assertEqual(out["height_range"], (10.0, 20.0))

    def test_pair_list_fields_become_tuples_of_tuples(self):
        out = coerce_params({"kills": [["S1", 5]]})
        self.assertEqual(out["kills"], (("S1", 5.0),))

    def test_unknown_parameter_is_rejected(self):
        """拼错的参数名必须报错，不能静默忽略——否则扫参结果会骗人。"""
        with self.assertRaises(ValueError):
            coerce_params({"slot_duraton": 0.002})


class TestSweepSpec(unittest.TestCase):
    def _write(self, payload) -> Path:
        tmp = Path(self._tmp.name) / "spec.json"
        tmp.write_text(json.dumps(payload), encoding="utf-8")
        return tmp

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self._tmp.cleanup()

    def test_loads_and_normalises(self):
        path = self._write({"name": "slot", "base": "x.json", "seed": 1,
                            "grid": {"slot_duration": [0.001, 0.002]}})
        spec = SweepSpec.from_json(path)
        self.assertEqual(spec.name, "slot")
        self.assertEqual(spec.repeats, 1)
        self.assertEqual(len(spec.combos()), 2)

    def test_rejects_unknown_top_level_key(self):
        path = self._write({"name": "s", "base": "x.json", "seed": 1,
                            "grid": {}, "repats": 2})
        with self.assertRaises(ValueError):
            SweepSpec.from_json(path)

    def test_rejects_zero_repeats(self):
        path = self._write({"name": "s", "base": "x.json", "seed": 1,
                            "grid": {}, "repeats": 0})
        with self.assertRaises(ValueError):
            SweepSpec.from_json(path)

    def test_default_metrics_are_non_empty(self):
        path = self._write({"name": "s", "base": "x.json", "seed": 1,
                            "grid": {}})
        self.assertTrue(SweepSpec.from_json(path).metrics)

    def test_run_count_is_combos_times_repeats(self):
        path = self._write({"name": "s", "base": "x.json", "seed": 1,
                            "repeats": 3,
                            "grid": {"a": [1, 2], "b": [3, 4]}})
        self.assertEqual(SweepSpec.from_json(path).run_count(), 12)


class TestTcritical(unittest.TestCase):
    def test_known_values(self):
        self.assertAlmostEqual(t_critical(1), 12.706, places=3)
        self.assertAlmostEqual(t_critical(10), 2.228, places=3)

    def test_large_df_approaches_normal(self):
        self.assertAlmostEqual(t_critical(1000), 1.96, places=2)

    def test_degrades_gracefully_below_one(self):
        self.assertEqual(t_critical(0), t_critical(1))


class TestAggregate(unittest.TestCase):
    def test_single_value_has_no_spread(self):
        stats = aggregate([5.0])
        self.assertEqual(stats["mean"], 5.0)
        self.assertEqual(stats["n"], 1)
        self.assertIsNone(stats["stdev"])

    def test_mean_and_sample_stdev(self):
        stats = aggregate([1.0, 2.0, 3.0])
        self.assertAlmostEqual(stats["mean"], 2.0)
        self.assertAlmostEqual(stats["stdev"], 1.0)

    def test_ci_brackets_the_mean(self):
        stats = aggregate([1.0, 2.0, 3.0])
        self.assertLess(stats["ci_low"], stats["mean"])
        self.assertGreater(stats["ci_high"], stats["mean"])

    def test_ci_narrows_as_n_grows(self):
        wide = aggregate([1.0, 3.0])
        narrow = aggregate([1.0, 2.0, 2.0, 2.0, 2.0, 3.0])
        self.assertLess(narrow["ci_high"] - narrow["ci_low"],
                        wide["ci_high"] - wide["ci_low"])

    def test_none_values_are_dropped_not_treated_as_zero(self):
        """None 表示"该指标没有样本"（如无样本的恢复时间），不是 0。"""
        stats = aggregate([None, 4.0, 6.0])
        self.assertEqual(stats["n"], 2)
        self.assertAlmostEqual(stats["mean"], 5.0)

    def test_all_none_gives_empty_stats(self):
        stats = aggregate([None, None])
        self.assertEqual(stats["n"], 0)
        self.assertIsNone(stats["mean"])

    def test_empty_input(self):
        self.assertEqual(aggregate([])["n"], 0)


class TestConfigApplication(unittest.TestCase):
    """扫参必须能在不落盘的情况下得到一个新配置。"""

    def test_applies_to_a_loaded_config(self):
        base = load_config(baseline_path())
        from skyfleet.study.spec import apply_params
        out = apply_params(base, {"slot_duration": 0.004,
                                  "slaves": 4})
        self.assertEqual(out.slot_duration, 0.004)
        self.assertEqual(out.slaves, 4)
        self.assertEqual(base.slot_duration, base.slot_duration)

    def test_returns_a_new_object(self):
        base = load_config(baseline_path())
        from skyfleet.study.spec import apply_params
        self.assertIsNot(apply_params(base, {}), base)

    def test_rejects_unknown_key(self):
        base = load_config(baseline_path())
        from skyfleet.study.spec import apply_params
        with self.assertRaises(ValueError):
            apply_params(base, {"nope": 1})


if __name__ == "__main__":
    unittest.main()
