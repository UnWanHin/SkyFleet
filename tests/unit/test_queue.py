"""优先队列与限流测试。

注意 helper 的 msg_type 映射：R0 会触发"最新值覆盖"，
所以测试 FIFO 语义时必须用非 R0 的消息类型。
"""
import unittest

from skyfleet.observe.events import DropReason
from skyfleet.protocol.message import Msg, Qos
from skyfleet.protocol.queue import RateLimiter, SendQueue, TokenBucket

TYPE_FOR_QOS = {
    Qos.Q0: "EMERGENCY",       # R3
    Qos.Q1: "TASK_ASSIGN",     # R2
    Qos.Q2: "MEMBER_UPDATE",   # R1
    Qos.Q3: "TELEMETRY",       # R0
    Qos.Q4: "BULK",            # R0
}


def m(qos, src="S1", t=0.0, seq=1, msg_type=None):
    return Msg.new(msg_type or TYPE_FOR_QOS[qos], src=src, dst="L",
                   t=t, seq=seq)


class TestTokenBucket(unittest.TestCase):
    def test_allows_up_to_burst(self):
        b = TokenBucket(rate_per_s=10.0, burst=3.0)
        self.assertEqual([b.allow(0.0) for _ in range(4)],
                         [True, True, True, False])

    def test_refills_over_time(self):
        b = TokenBucket(rate_per_s=10.0, burst=3.0)
        for _ in range(3):
            b.allow(0.0)
        self.assertFalse(b.allow(0.0))
        self.assertTrue(b.allow(0.2))

    def test_never_exceeds_burst_after_long_idle(self):
        b = TokenBucket(rate_per_s=10.0, burst=2.0)
        self.assertEqual([b.allow(1000.0) for _ in range(3)],
                         [True, True, False])


class TestSendQueue(unittest.TestCase):
    def test_strict_priority_dequeue(self):
        q = SendQueue(depth_limit=100, rng=None)
        q.enqueue(m(Qos.Q3, seq=1), 0.0)
        q.enqueue(m(Qos.Q0, seq=2), 0.0)
        q.enqueue(m(Qos.Q1, seq=3), 0.0)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q0)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q1)
        self.assertIs(q.dequeue(0.0).qos, Qos.Q3)

    def test_fifo_within_same_qos(self):
        q = SendQueue(depth_limit=100, rng=None)
        q.enqueue(m(Qos.Q2, seq=1), 0.0)
        q.enqueue(m(Qos.Q2, seq=2), 0.0)
        self.assertEqual(q.dequeue(0.0).seq, 1)
        self.assertEqual(q.dequeue(0.0).seq, 2)

    def test_queue_full_returns_reason(self):
        q = SendQueue(depth_limit=1, rng=None)
        self.assertIsNone(q.enqueue(m(Qos.Q2, src="S1", seq=1), 0.0))
        self.assertIs(q.enqueue(m(Qos.Q2, src="S2", seq=2), 0.0),
                      DropReason.QUEUE_FULL)

    def test_r0_latest_value_overwrites_same_source(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q3, seq=1), 0.0)
        q.enqueue(m(Qos.Q3, seq=2), 0.0)
        self.assertEqual(q.size(), 1)
        self.assertEqual(q.dequeue(0.0).seq, 2)

    def test_r0_overwrite_does_not_overflow_full_queue(self):
        q = SendQueue(depth_limit=1, rng=None)
        q.enqueue(m(Qos.Q3, src="S1", seq=1), 0.0)
        self.assertIsNone(q.enqueue(m(Qos.Q3, src="S1", seq=2), 0.0))
        self.assertEqual(q.size(), 1)

    def test_different_sources_do_not_overwrite(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q3, src="S1", seq=1), 0.0)
        q.enqueue(m(Qos.Q3, src="S2", seq=1), 0.0)
        self.assertEqual(q.size(), 2)

    def test_empty_dequeue_is_none(self):
        self.assertIsNone(SendQueue(depth_limit=1, rng=None).dequeue(0.0))

    def test_size_by_qos(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q0, seq=1), 0.0)
        q.enqueue(m(Qos.Q3, seq=2), 0.0)
        self.assertEqual(q.size(Qos.Q0), 1)
        self.assertEqual(q.size(), 2)

    def test_sample_reports_all_levels(self):
        q = SendQueue(depth_limit=10, rng=None)
        q.enqueue(m(Qos.Q0, seq=1), 0.0)
        s = q.sample()
        self.assertEqual(s["Q0"], 1)
        self.assertEqual(s["Q3"], 0)


class TestRateLimiter(unittest.TestCase):
    def test_telemetry_is_never_rate_limited(self):
        rl = RateLimiter(q0_rate=1.0, q0_burst=1.0, q1_rate=1.0, q1_burst=1.0)
        self.assertTrue(all(rl.allow(m(Qos.Q3, seq=i), 0.0)
                            for i in range(1, 20)))

    def test_emergency_burst_then_blocked(self):
        rl = RateLimiter(q0_rate=1.0, q0_burst=2.0, q1_rate=1.0, q1_burst=2.0)
        self.assertTrue(rl.allow(m(Qos.Q0, seq=1), 0.0))
        self.assertTrue(rl.allow(m(Qos.Q0, seq=2), 0.0))
        self.assertFalse(rl.allow(m(Qos.Q0, seq=3), 0.0))

    def test_q0_and_q1_have_separate_buckets(self):
        rl = RateLimiter(q0_rate=1.0, q0_burst=1.0, q1_rate=1.0, q1_burst=1.0)
        self.assertTrue(rl.allow(m(Qos.Q0, seq=1), 0.0))
        self.assertTrue(rl.allow(m(Qos.Q1, seq=2), 0.0))
        self.assertFalse(rl.allow(m(Qos.Q0, seq=3), 0.0))


if __name__ == "__main__":
    unittest.main()
