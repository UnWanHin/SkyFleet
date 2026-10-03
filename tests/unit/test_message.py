"""消息格式与封闭枚举测试。"""
import unittest

from skyfleet.observe.events import DropReason, EventKind
from skyfleet.protocol.message import MSG_TYPES, Msg, Qos, Rel


class TestEnums(unittest.TestCase):
    def test_qos_ordering(self):
        self.assertLess(Qos.Q0, Qos.Q3)
        self.assertLess(Qos.Q1, Qos.Q4)

    def test_rel_ordering(self):
        self.assertLess(Rel.R0, Rel.R3)

    def test_drop_reason_is_closed_set(self):
        self.assertEqual(len(DropReason), 12)
        self.assertEqual(DropReason.OCCLUDED.value, "OCCLUDED")

    def test_event_kinds_are_json_friendly(self):
        for kind in EventKind:
            self.assertIsInstance(kind.value, str)


class TestMsgFactory(unittest.TestCase):
    def test_new_fills_qos_and_rel_from_type(self):
        m = Msg.new("TELEMETRY", src="S1", dst="LEADER", t=0.0)
        self.assertIs(m.qos, Qos.Q3)
        self.assertIs(m.reliability, Rel.R0)

    def test_emergency_is_q0_r3(self):
        m = Msg.new("EMERGENCY", src="S1", dst="LEADER", t=1.0)
        self.assertIs(m.qos, Qos.Q0)
        self.assertIs(m.reliability, Rel.R3)

    def test_ack_is_r2(self):
        self.assertIs(MSG_TYPES["ACK"][1], Rel.R2)

    def test_unknown_type_raises(self):
        with self.assertRaises(KeyError):
            Msg.new("NOPE", src="S1", dst="L", t=0.0)

    def test_msg_id_is_unique(self):
        a = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        b = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertNotEqual(a.msg_id, b.msg_id)

    def test_is_frozen(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        with self.assertRaises(Exception):
            m.seq = 99

    def test_defaults(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertEqual(m.hop, 0)
        self.assertEqual(m.leader_epoch, 0)
        self.assertEqual(m.mission_version, 0)
        self.assertEqual(m.route_version, 0)
        self.assertIsNone(m.operation_id)
        self.assertEqual(m.payload, {})
        self.assertIsInstance(m.session_id, str)

    def test_deadline_defaults_to_never_for_r0(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0)
        self.assertEqual(m.deadline, float("inf"))

    def test_deadline_is_tight_for_r3(self):
        m = Msg.new("EMERGENCY", src="S1", dst="L", t=10.0)
        self.assertLessEqual(m.deadline - 10.0, 0.1)

    def test_explicit_fields_win_over_defaults(self):
        m = Msg.new("TELEMETRY", src="S1", dst="L", t=0.0,
                    seq=42, leader_epoch=3, operation_id="op-1")
        self.assertEqual(m.seq, 42)
        self.assertEqual(m.leader_epoch, 3)
        self.assertEqual(m.operation_id, "op-1")


if __name__ == "__main__":
    unittest.main()
