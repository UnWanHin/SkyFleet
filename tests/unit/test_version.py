"""版本、任期与去重接受规则测试（纯函数）。"""
import unittest

from skyfleet.protocol.message import Msg
from skyfleet.protocol.version import (AcceptDecision, DedupTable, LocalView,
                                       accept)


def make(**kw):
    base = dict(msg_type="TASK_ASSIGN", src="L", dst="S1", t=0.0)
    base.update(kw)
    return Msg.new(**base)


class TestAcceptRules(unittest.TestCase):
    def setUp(self):
        self.view = LocalView(leader_epoch=2, mission_version=5,
                              route_version=1, sessions={"L#1"})
        self.dedup = DedupTable(ttl=10.0, capacity=100)

    def test_accepts_fresh_message(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_old_epoch(self):
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_accepts_newer_epoch(self):
        m = make(leader_epoch=3, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_unknown_session(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SESSION)

    def test_rejects_old_mission_version(self):
        m = make(leader_epoch=2, mission_version=4, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_VERSION)

    def test_epoch_beats_session_check(self):
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_rejects_duplicate_operation(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#1",
                 operation_id="op-7")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)
        self.assertIs(accept(m, self.view, self.dedup, 0.1), AcceptDecision.DEDUP)

    def test_rejects_stale_seq(self):
        self.view.last_seq[("L", 2, 5)] = 9
        m = make(leader_epoch=2, mission_version=5, seq=9, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SEQ)

    def test_accepts_higher_seq(self):
        self.view.last_seq[("L", 2, 5)] = 9
        m = make(leader_epoch=2, mission_version=5, seq=10, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_accept_updates_view(self):
        m = make(leader_epoch=4, mission_version=7, seq=3, session_id="L#1")
        accept(m, self.view, self.dedup, 0.0)
        self.assertEqual(self.view.leader_epoch, 4)
        self.assertEqual(self.view.mission_version, 7)
        self.assertEqual(self.view.last_seq[("L", 4, 7)], 3)

    def test_reject_does_not_mutate_view(self):
        before = (self.view.leader_epoch, self.view.mission_version)
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#1")
        accept(m, self.view, self.dedup, 0.0)
        self.assertEqual((self.view.leader_epoch, self.view.mission_version), before)


class TestDedupTable(unittest.TestCase):
    def test_first_sight_is_not_duplicate(self):
        d = DedupTable(ttl=5.0, capacity=10)
        self.assertFalse(d.seen("op", 0.0))

    def test_repeat_within_ttl_is_duplicate(self):
        d = DedupTable(ttl=5.0, capacity=10)
        d.seen("op", 0.0)
        self.assertTrue(d.seen("op", 4.9))

    def test_expires_after_ttl(self):
        d = DedupTable(ttl=5.0, capacity=10)
        d.seen("op", 0.0)
        self.assertFalse(d.seen("op", 5.1))

    def test_capacity_is_bounded(self):
        d = DedupTable(ttl=1000.0, capacity=3)
        for i in range(50):
            d.seen(f"op{i}", float(i))
        self.assertLessEqual(len(d), 3)

    def test_prune_drops_expired(self):
        d = DedupTable(ttl=1.0, capacity=100)
        for i in range(10):
            d.seen(f"op{i}", 0.0)
        d.prune(5.0)
        self.assertEqual(len(d), 0)


if __name__ == "__main__":
    unittest.main()
