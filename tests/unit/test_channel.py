"""共享信道测试。

核心命题（报告 §5.5 / §3.1）：信道是共享资源，而非每链路一个固定丢包率。
同一时刻互相干扰的并发发送必须冲突；信道被占用时后来者必须等待（接入时延）。
"""
import unittest

from skyfleet.kernel.rng import RngRegistry
from skyfleet.observe.events import DropReason
from skyfleet.world.channel import Channel

P1 = (0.0, 0.0, 50.0)
P2 = (100.0, 0.0, 50.0)
P_FAR = (5000.0, 5000.0, 50.0)


def mk(interference=300.0, rng_seed=1):
    return Channel(rate_bps=2_000_000, interference_range_m=interference,
                   rng=RngRegistry(rng_seed).get("channel:collision"))


class TestBasics(unittest.TestCase):
    def test_free_channel_succeeds_without_waiting(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        c.flush()
        self.assertTrue(a.ok)
        self.assertEqual(a.t_access, 0.0)

    def test_tx_time_is_size_over_rate(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertAlmostEqual(a.t_tx, 1.0)

    def test_propagation_delay_is_computed(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        self.assertAlmostEqual(a.t_prop, 100.0 / 299792458.0, places=12)

    def test_busy_until_advances_after_success(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        c.flush()
        self.assertAlmostEqual(c.busy_until(), a.t_end)

    def test_zero_quality_cannot_transmit(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.0, 512)
        self.assertFalse(a.ok)
        self.assertIsNotNone(a.reason)

    def test_no_reason_when_successful(self):
        a = mk().attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        self.assertIsNone(a.reason)


class TestContention(unittest.TestCase):
    def test_simultaneous_senders_collide(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        b = c.attempt("m2", "S2", P1, "L", P2, 0.0, 0.9, 512)
        c.flush()
        self.assertFalse(a.ok)
        self.assertFalse(b.ok)
        self.assertIs(a.reason, DropReason.COLLISION)
        self.assertIs(b.reason, DropReason.COLLISION)

    def test_second_sender_waits_when_channel_busy(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        c.flush()
        a = c.attempt("m2", "S2", P1, "L", P2, 0.0, 0.9, 2_000_000)
        self.assertGreater(a.t_access, 0.0)

    def test_distant_sender_does_not_interfere(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 512)
        b = c.attempt("m2", "S2", P_FAR, "L", (5100.0, 5000.0, 50.0),
                      0.0, 0.9, 512)
        c.flush()
        self.assertTrue(a.ok)
        self.assertTrue(b.ok)

    def test_non_overlapping_in_time_do_not_collide(self):
        c = mk()
        a = c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)  # 0.0-1.0
        c.flush()
        b = c.attempt("m2", "S2", P1, "L", P2, 5.0, 0.9, 512)
        c.flush()
        self.assertTrue(a.ok)
        self.assertTrue(b.ok)

    def test_channel_frees_after_transmission_ends(self):
        c = mk()
        c.attempt("m1", "S1", P1, "L", P2, 0.0, 0.9, 2_000_000)
        c.flush()
        a = c.attempt("m2", "S2", P1, "L", P2, 1.5, 0.9, 512)
        c.flush()
        self.assertTrue(a.ok)
        self.assertEqual(a.t_access, 0.0)


class TestQuality(unittest.TestCase):
    def test_low_quality_may_drop_even_without_contention(self):
        c = mk()
        drops = 0
        for i in range(100):
            a = c.attempt(f"m{i}", "S1", P1, "L", P2, float(i) * 10.0, 0.2, 512)
            c.flush()
            if not a.ok:
                drops += 1
                self.assertIs(a.reason, DropReason.OCCLUDED)
        self.assertGreater(drops, 0)

    def test_high_quality_never_drops_without_contention(self):
        c = mk()
        for i in range(50):
            a = c.attempt(f"m{i}", "S1", P1, "L", P2, float(i) * 10.0, 0.9, 512)
            c.flush()
            self.assertTrue(a.ok)


class TestDeterminism(unittest.TestCase):
    def test_same_inputs_same_outcome(self):
        def run():
            c = mk()
            out = []
            for i in range(5):
                a = c.attempt(f"m{i}", f"S{i}", P1, "L", P2, 0.0, 0.4, 512)
                c.flush()
                out.append((a.ok, a.reason))
            return out

        self.assertEqual(run(), run())


if __name__ == "__main__":
    unittest.main()
