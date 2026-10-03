"""可靠性策略测试。

核心可验证命题（报告表 4-1）：重传不是无条件的——超过 deadline 必须停手。
"""
import unittest
from dataclasses import replace

from skyfleet.protocol.message import Msg, Rel
from skyfleet.protocol.reliability import ReliabilityManager, max_retries


def mk(rel, t=0.0, deadline=10.0):
    m = Msg.new("DELIVERY_CONFIRM", src="S1", dst="L", t=t)
    return replace(m, reliability=rel, deadline=deadline)


class TestPolicy(unittest.TestCase):
    def test_max_retries_by_level(self):
        self.assertEqual(max_retries(Rel.R0), 0)
        self.assertEqual(max_retries(Rel.R1), 2)
        self.assertEqual(max_retries(Rel.R2), 3)
        self.assertEqual(max_retries(Rel.R3), 2)


class TestManager(unittest.TestCase):
    def test_ack_marks_done(self):
        rm = ReliabilityManager()
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        self.assertTrue(rm.ack(msg.msg_id))

    def test_ack_unknown_message_is_false(self):
        self.assertFalse(ReliabilityManager().ack("nope"))

    def test_ack_twice_is_false(self):
        rm = ReliabilityManager()
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        rm.ack(msg.msg_id)
        self.assertFalse(rm.ack(msg.msg_id))

    def test_retry_due_after_timeout(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        self.assertEqual(rm.due_retries(0.5), [])
        self.assertEqual([x.msg_id for x in rm.due_retries(1.5)], [msg.msg_id])

    def test_due_retries_not_repeated_immediately(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        self.assertEqual(len(rm.due_retries(1.5)), 1)
        self.assertEqual(rm.due_retries(1.6), [])

    def test_acked_message_never_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        rm.ack(msg.msg_id)
        self.assertEqual(rm.due_retries(5.0), [])

    def test_no_retry_past_deadline(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2, deadline=1.2)
        rm.register(msg, 0.0)
        self.assertEqual(rm.due_retries(2.0), [])
        self.assertTrue(rm.expired(2.0))

    def test_not_expired_before_deadline(self):
        rm = ReliabilityManager(retry_interval=1.0)
        rm.register(mk(Rel.R2, deadline=1.2), 0.0)
        self.assertEqual(rm.expired(1.0), [])

    def test_r0_never_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        rm.register(mk(Rel.R0), 0.0)
        self.assertEqual(rm.due_retries(5.0), [])

    def test_exhausted_after_max_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        for i in range(3):
            rm.on_retry_sent(msg.msg_id, float(i) + 1.0)
        self.assertTrue(rm.exhausted(msg.msg_id))

    def test_not_exhausted_before_limit(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R2)
        rm.register(msg, 0.0)
        rm.on_retry_sent(msg.msg_id, 1.0)
        self.assertFalse(rm.exhausted(msg.msg_id))

    def test_exhausted_stops_retries(self):
        rm = ReliabilityManager(retry_interval=1.0)
        msg = mk(Rel.R1)
        rm.register(msg, 0.0)
        for i in range(2):
            rm.on_retry_sent(msg.msg_id, float(i) + 1.0)
        self.assertEqual(rm.due_retries(9.0), [])

    def test_pending_count(self):
        rm = ReliabilityManager()
        a, b = mk(Rel.R1), mk(Rel.R1)
        rm.register(a, 0.0)
        rm.register(b, 0.0)
        rm.ack(a.msg_id)
        self.assertEqual(rm.pending_count(), 1)


if __name__ == "__main__":
    unittest.main()
