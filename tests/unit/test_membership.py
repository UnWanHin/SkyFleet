"""成员状态机测试。

核心命题（报告 §2.2.1）：单次丢包不得等同于故障；邻居间接探测必须能
抑制"疑似失联"的误判。
"""
import unittest

from skyfleet.protocol.membership import (MemberStatus, MembershipParams,
                                          MembershipView)

P = MembershipParams(n_degraded=2, n_suspect=4, t_fail=3.0, q_degraded=0.4)


class TestHysteresis(unittest.TestCase):
    def setUp(self):
        self.v = MembershipView(P)
        self.v.on_join("S1", "S1#1", 0.0)

    def test_join_starts_online(self):
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_single_miss_does_not_change_status(self):
        self.v.on_miss("S1", 1.0)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_two_misses_go_degraded_not_failed(self):
        self.v.on_miss("S1", 1.0)
        self.v.on_miss("S1", 2.0)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_degraded_never_becomes_failed_without_suspect(self):
        for i in range(1, 4):
            self.v.on_miss("S1", float(i))
        self.assertNotEqual(self.v.status("S1"), MemberStatus.FAILED)

    def test_low_quality_triggers_degraded_without_misses(self):
        self.v.on_heartbeat("S1", "S1#1", 0.1, 0.1, 1)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_sustained_misses_go_suspect(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.assertIs(self.v.status("S1"), MemberStatus.SUSPECT)

    def test_indirect_evidence_prevents_suspect(self):
        for i in range(1, 4):
            self.v.on_miss("S1", float(i))
        self.v.on_indirect("S1", 3.5)
        self.v.on_miss("S1", 4.0)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_indirect_evidence_demotes_suspect(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.on_indirect("S1", 4.5)
        self.assertIs(self.v.status("S1"), MemberStatus.DEGRADED)

    def test_suspect_fails_after_t_fail(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))       # t=4.0 进入 SUSPECT
        self.v.tick(6.9)                          # 距进入 SUSPECT 仅 2.9s
        self.assertIs(self.v.status("S1"), MemberStatus.SUSPECT)
        self.v.tick(7.1)
        self.assertIs(self.v.status("S1"), MemberStatus.FAILED)

    def test_heartbeat_recovers_to_online(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.on_heartbeat("S1", "S1#1", 5.0, 0.9, 2)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_heartbeat_resets_miss_count(self):
        self.v.on_miss("S1", 1.0)
        self.v.on_miss("S1", 2.0)
        self.v.on_heartbeat("S1", "S1#1", 3.0, 0.9, 2)
        self.v.on_miss("S1", 4.0)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)

    def test_leave_sets_left(self):
        self.v.on_leave("S1", 1.0)
        self.assertIs(self.v.status("S1"), MemberStatus.LEFT)

    def test_rejoin_from_failed_goes_online(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        self.v.tick(9.0)
        self.assertIs(self.v.status("S1"), MemberStatus.FAILED)
        self.v.on_join("S1", "S1#2", 10.0)
        self.assertIs(self.v.status("S1"), MemberStatus.ONLINE)


class TestView(unittest.TestCase):
    def setUp(self):
        self.v = MembershipView(P)
        self.v.on_join("S1", "S1#1", 0.0)
        self.v.on_join("S2", "S2#1", 0.0)

    def test_members_filter_by_status(self):
        self.v.on_leave("S2", 1.0)
        self.assertEqual(self.v.members(statuses={MemberStatus.ONLINE}), ["S1"])

    def test_members_all_is_sorted(self):
        self.assertEqual(self.v.members(), ["S1", "S2"])

    def test_unknown_node_status_is_none(self):
        self.assertIsNone(self.v.status("NOPE"))

    def test_tick_reports_only_changes(self):
        for i in range(1, 5):
            self.v.on_miss("S1", float(i))
        changes = self.v.tick(9.0)
        self.assertEqual(changes, [("S1", MemberStatus.FAILED)])
        self.assertEqual(self.v.tick(10.0), [])

    def test_available_excludes_low_energy(self):
        self.v.on_heartbeat("S1", "S1#1", 0.5, 0.9, 1)
        self.v.set_energy("S1", 5.0)
        self.assertEqual(self.v.available(min_energy=10.0), ["S2"])


if __name__ == "__main__":
    unittest.main()
