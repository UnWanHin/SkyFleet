"""单节点协议代理测试。"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.kernel.rng import RngRegistry
from skyfleet.observe.events import EventKind
from skyfleet.observe.log import EventLog, read_log
from skyfleet.protocol.identity import IdentityRegistry, Role
from skyfleet.protocol.message import Msg, Rel
from skyfleet.protocol.version import AcceptDecision
from skyfleet.sim.agent import NodeAgent
from skyfleet.world.node import Node
from tests.support.fakes import FakeTransport


def build_agent(tmpdir, node_id="S1", role=Role.SLAVE, cfg=None,
                transport=None):
    cfg = cfg or SimConfig()
    log = EventLog(Path(tmpdir) / "log.jsonl")
    identity = IdentityRegistry()
    transport = transport or FakeTransport()
    node = Node(node_id, role, (0.0, 0.0, 50.0), speed=12.0, energy=100.0)
    agent = NodeAgent(node=node, config=cfg, identity=identity,
                      rngs=RngRegistry(cfg.seed), log=log,
                      transport=transport)
    agent.session_id = identity.join(node_id, 0.0)
    return agent, transport, log


class TestPeriodicTraffic(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.agent, self.transport, self.log = build_agent(self._tmp.name)

    def tearDown(self):
        self.log.close()
        self._tmp.cleanup()

    def test_heartbeat_tick_transmits_heartbeat(self):
        self.agent.on_heartbeat_tick(0.0)
        self.assertEqual(len(self.transport.messages_of_type("HEARTBEAT")), 1)

    def test_heartbeat_carries_session_id(self):
        self.agent.on_heartbeat_tick(0.0)
        msg = self.transport.messages_of_type("HEARTBEAT")[0]
        self.assertEqual(msg.session_id, self.agent.session_id)

    def test_heartbeat_seq_increments(self):
        self.agent.on_heartbeat_tick(0.0)
        self.agent.on_heartbeat_tick(1.0)
        seqs = [m.seq for m in self.transport.messages_of_type("HEARTBEAT")]
        self.assertEqual(seqs, [1, 2])

    def test_telemetry_tick_transmits_telemetry(self):
        self.agent.on_telemetry_tick(0.0)
        self.assertEqual(len(self.transport.messages_of_type("TELEMETRY")), 1)

    def test_telemetry_payload_has_position_and_energy(self):
        self.agent.on_telemetry_tick(0.0)
        msg = self.transport.messages_of_type("TELEMETRY")[0]
        self.assertIn("pos", msg.payload)
        self.assertIn("energy", msg.payload)

    def test_heartbeat_is_r1(self):
        self.agent.on_heartbeat_tick(0.0)
        msg = self.transport.messages_of_type("HEARTBEAT")[0]
        self.assertIs(msg.reliability, Rel.R1)

    def test_periodic_status_is_not_blindly_retransmitted(self):
        """周期状态由周期本身提供重复，无条件重传只会灌爆信道。"""
        self.agent.on_heartbeat_tick(0.0)
        self.assertEqual(self.agent.reliability.pending_count(), 0)

    def test_telemetry_is_r0_and_not_tracked(self):
        self.agent.on_telemetry_tick(0.0)
        self.assertEqual(self.agent.reliability.pending_count(), 0)

    def test_r2_message_is_tracked_for_retransmission(self):
        self.agent.submit(self._task_assign(1), 0.0)
        self.assertEqual(self.agent.reliability.pending_count(), 1)

    def _task_assign(self, seq):
        return Msg.new("TASK_ASSIGN", src="L", dst="S1", t=0.0, seq=seq,
                       session_id="L#1")


class TestRateLimiting(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        cfg = SimConfig(q0_rate=1.0, q0_burst=1.0)
        self.agent, self.transport, self.log = build_agent(
            self._tmp.name, cfg=cfg)

    def tearDown(self):
        self.log.close()
        self._tmp.cleanup()

    def test_emergency_within_burst_passes(self):
        self.assertTrue(self.agent.submit(self._emergency(1), 0.0))

    def test_emergency_beyond_burst_is_rate_limited(self):
        self.agent.submit(self._emergency(1), 0.0)
        self.assertFalse(self.agent.submit(self._emergency(2), 0.0))

    def test_rate_limit_is_logged_with_reason(self):
        self.agent.submit(self._emergency(1), 0.0)
        self.agent.submit(self._emergency(2), 0.0)
        self.log.close()
        reasons = [r["reason"] for r in read_log(self.log.path)
                   if r["kind"] == EventKind.MSG_DROP.value]
        self.assertIn("RATE_LIMITED", reasons)

    def _emergency(self, seq):
        return Msg.new("EMERGENCY", src="S1", dst="L", t=0.0, seq=seq,
                       session_id=self.agent.session_id)


class TestReceive(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.agent, self.transport, self.log = build_agent(self._tmp.name)

    def tearDown(self):
        self.log.close()
        self._tmp.cleanup()

    def _from_leader(self, **kw):
        base = dict(msg_type="TASK_ASSIGN", src="L", dst="S1", t=0.0,
                    session_id="L#1")
        base.update(kw)
        return Msg.new(**base)

    def test_fresh_message_accepted(self):
        self.agent.view.sessions.add("L#1")
        self.assertIs(self.agent.receive(self._from_leader(), 0.0, 0.9),
                      AcceptDecision.ACCEPT)

    def test_duplicate_operation_rejected(self):
        self.agent.view.sessions.add("L#1")
        first = self._from_leader(operation_id="op-1", seq=1)
        second = self._from_leader(operation_id="op-1", seq=1)
        self.agent.receive(first, 0.0, 0.9)
        self.assertIs(self.agent.receive(second, 0.1, 0.9),
                      AcceptDecision.DEDUP)

    def test_stale_epoch_rejected(self):
        self.agent.view.sessions.add("L#1")
        self.agent.view.leader_epoch = 5
        msg = self._from_leader(leader_epoch=1)
        self.assertIs(self.agent.receive(msg, 0.0, 0.9),
                      AcceptDecision.STALE_EPOCH)

    def test_heartbeat_updates_membership(self):
        self.agent.view.sessions.add("L#1")
        msg = Msg.new("HEARTBEAT", src="L", dst="S1", t=0.0,
                      session_id="L#1", payload={"nearby": ["S2"]})
        self.agent.receive(msg, 0.0, 0.9)
        self.assertEqual(self.agent.membership.status("L"), "ONLINE")

    def test_r2_message_triggers_ack(self):
        self.agent.view.sessions.add("L#1")
        self.agent.receive(self._from_leader(seq=1), 0.0, 0.9)
        self.assertEqual(len(self.transport.messages_of_type("ACK")), 1)

    def test_r0_message_does_not_trigger_ack(self):
        self.agent.view.sessions.add("L#1")
        msg = Msg.new("TELEMETRY", src="L", dst="S1", t=0.0, session_id="L#1")
        self.agent.receive(msg, 0.0, 0.9)
        self.assertEqual(len(self.transport.messages_of_type("ACK")), 0)

    def test_ack_clears_pending(self):
        self.agent.view.sessions.add("L#1")
        sent = self._from_leader(seq=1)
        self.agent.submit(sent, 0.0)
        self.assertEqual(self.agent.reliability.pending_count(), 1)
        ack = Msg.new("ACK", src="L", dst="S1", t=0.1, session_id="L#1",
                      payload={"ack_for": sent.msg_id})
        self.agent.receive(ack, 0.1, 0.9)
        self.assertEqual(self.agent.reliability.pending_count(), 0)


if __name__ == "__main__":
    unittest.main()
