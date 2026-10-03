"""身份、会话与租约测试。"""
import unittest

from skyfleet.protocol.identity import IdentityRegistry, Lease, Role


class TestJoin(unittest.TestCase):
    def test_first_join_creates_session(self):
        self.assertEqual(IdentityRegistry().join("S1", 0.0), "S1#1")

    def test_rejoin_creates_new_session(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        self.assertEqual(r.join("S1", 5.0), "S1#2")

    def test_old_session_becomes_invalid(self):
        r = IdentityRegistry()
        old = r.join("S1", 0.0)
        r.join("S1", 5.0)
        self.assertFalse(r.is_valid_session("S1", old))

    def test_current_session_is_valid(self):
        r = IdentityRegistry()
        s = r.join("S1", 0.0)
        self.assertTrue(r.is_valid_session("S1", s))

    def test_current_session_is_latest(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        new = r.join("S1", 5.0)
        self.assertEqual(r.current_session("S1"), new)

    def test_unknown_node_has_no_session(self):
        self.assertIsNone(IdentityRegistry().current_session("X"))

    def test_sessions_are_independent_between_nodes(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        r.join("S2", 0.0)
        self.assertEqual(r.current_session("S1"), "S1#1")
        self.assertEqual(r.current_session("S2"), "S2#1")

    def test_node_id_does_not_change_address_semantics(self):
        r = IdentityRegistry()
        r.join("S1", 0.0)
        r.join("S1", 1.0)
        r.join("S1", 2.0)
        self.assertEqual(r.current_session("S1"), "S1#3")


class TestLease(unittest.TestCase):
    def test_valid_before_expiry(self):
        self.assertTrue(Lease("L", Role.LEADER, 0, "all", 10.0).valid(9.9))

    def test_invalid_at_expiry(self):
        self.assertFalse(Lease("L", Role.LEADER, 0, "all", 10.0).valid(10.0))

    def test_invalid_after_expiry(self):
        self.assertFalse(Lease("L", Role.LEADER, 0, "all", 10.0).valid(99.0))

    def test_grant_and_query_active(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        self.assertEqual(len(r.active_leases(4.0)), 1)

    def test_expired_lease_not_active(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        self.assertEqual(r.active_leases(5.0), [])

    def test_regrant_replaces_previous(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        r.grant_lease("L", Role.STANDBY, 1, "limited", t=6.0, ttl=5.0)
        active = r.active_leases(7.0)
        self.assertEqual(len(active), 1)
        self.assertIs(active[0].role, Role.STANDBY)

    def test_lease_for_node(self):
        r = IdentityRegistry()
        r.grant_lease("L", Role.LEADER, 0, "all", t=0.0, ttl=5.0)
        self.assertIsNotNone(r.lease_for("L", 1.0))
        self.assertIsNone(r.lease_for("L", 9.0))


if __name__ == "__main__":
    unittest.main()
