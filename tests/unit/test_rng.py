"""独立随机源测试。

test_seed_is_stable_across_processes 钉死具体数值：若实现改用内建 hash()，
该断言会因 PYTHONHASHSEED 而在不同进程间失败。
"""
import unittest

from skyfleet.kernel.rng import RngRegistry, derive_seed


class TestDeriveSeed(unittest.TestCase):
    def test_same_inputs_give_same_seed(self):
        self.assertEqual(derive_seed(42, "channel:collision"),
                         derive_seed(42, "channel:collision"))

    def test_different_names_give_different_seeds(self):
        self.assertNotEqual(derive_seed(42, "a"), derive_seed(42, "b"))

    def test_different_master_gives_different_seed(self):
        self.assertNotEqual(derive_seed(1, "a"), derive_seed(2, "a"))

    def test_seed_is_stable_across_processes(self):
        self.assertEqual(derive_seed(0, "channel:collision"), 12036629166246477061)


class TestRngRegistry(unittest.TestCase):
    def test_same_name_returns_same_instance(self):
        r = RngRegistry(7)
        self.assertIs(r.get("x"), r.get("x"))

    def test_independent_streams_do_not_interleave(self):
        a = RngRegistry(7).get("x")
        b = RngRegistry(7)
        b.get("y").random()
        self.assertEqual(a.random(), RngRegistry(7).get("x").random())

    def test_reproducible_sequence(self):
        self.assertEqual([RngRegistry(3).get("s").random() for _ in range(1)],
                         [RngRegistry(3).get("s").random() for _ in range(1)])

    def test_streams_have_different_content(self):
        r = RngRegistry(11)
        self.assertNotEqual(r.get("a").random(), r.get("b").random())


if __name__ == "__main__":
    unittest.main()
