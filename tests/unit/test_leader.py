"""主节点任务编排测试。"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.kernel.rng import RngRegistry
from skyfleet.mission.task import TaskBoard, TaskState
from skyfleet.observe.log import EventLog
from skyfleet.protocol.identity import IdentityRegistry, Role
from skyfleet.protocol.membership import MembershipParams, MembershipView
from skyfleet.sim.agent import NodeAgent
from skyfleet.sim.leader import LeaderController
from skyfleet.world.node import Node
from tests.support.fakes import FakeTransport


class TestLeader(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = SimConfig()
        self.log = EventLog(Path(self._tmp.name) / "log.jsonl")
        self.identity = IdentityRegistry()
        self.transport = FakeTransport()
        self.board = TaskBoard()
        self.members = MembershipView(MembershipParams())

        leader_node = Node("L", Role.LEADER, (250.0, 250.0, 100.0),
                           speed=0.0, energy=100.0)
        self.agent = NodeAgent(leader_node, self.cfg, self.identity,
                               RngRegistry(1), self.log, self.transport)
        self.agent.session_id = self.identity.join("L", 0.0)
        self.leader = LeaderController(self.cfg, self.board, self.members,
                                       self.agent, self.identity, self.log)

    def tearDown(self):
        self.log.close()
        self._tmp.cleanup()

    def _add_member(self, node_id, energy=100.0, t=0.0):
        self.members.on_join(node_id, f"{node_id}#1", t)
        self.members.set_energy(node_id, energy)

    def test_create_tasks_creates_configured_count(self):
        self.leader.create_tasks(0.0)
        self.assertEqual(len(self.board.all()), 20)

    def test_created_tasks_start_in_created_state(self):
        self.leader.create_tasks(0.0)
        self.assertTrue(all(t.state is TaskState.CREATED
                            for t in self.board.all()))

    def test_task_ids_are_unique(self):
        self.leader.create_tasks(0.0)
        ids = [t.task_id for t in self.board.all()]
        self.assertEqual(len(set(ids)), len(ids))

    def test_assign_pending_assigns_all_to_online_members(self):
        self.leader.create_tasks(0.0)
        for i in range(3):
            self._add_member(f"S{i}")
        assigned = self.leader.assign_pending(1.0)
        self.assertEqual(assigned, 20)
        self.assertTrue(all(t.state is TaskState.ASSIGNED
                            for t in self.board.all()))

    def test_assign_skips_low_energy_members(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1", energy=5.0)
        assigned = self.leader.assign_pending(1.0)
        self.assertEqual(assigned, 0)

    def test_assign_skips_failed_members(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.members.on_leave("S1", 0.5)
        self.assertEqual(self.leader.assign_pending(1.0), 0)

    def test_assignment_is_round_robin(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self._add_member("S2")
        self.leader.assign_pending(1.0)
        assignees = {t.assignee for t in self.board.all()}
        self.assertEqual(assignees, {"S1", "S2"})

    def test_member_failed_reassigns_its_tasks(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.leader.assign_pending(1.0)
        version_before = self.leader.mission_version
        count = self.leader.on_member_failed("S1", 5.0)
        self.assertGreater(count, 0)
        self.assertGreater(self.leader.mission_version, version_before)

    def test_reassign_clears_assignee(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.leader.assign_pending(1.0)
        self.leader.on_member_failed("S1", 5.0)
        self.assertTrue(all(t.assignee is None for t in self.board.all()))

    def test_reassigned_tasks_can_be_assigned_again(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.leader.assign_pending(1.0)
        self.leader.on_member_failed("S1", 5.0)
        self._add_member("S2")
        self.assertGreater(self.leader.assign_pending(6.0), 0)

    def test_member_failed_with_no_tasks_returns_zero(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.leader.assign_pending(1.0)
        self.assertEqual(self.leader.on_member_failed("S9", 5.0), 0)

    def test_assignment_message_carries_mission_version(self):
        self.leader.create_tasks(0.0)
        self._add_member("S1")
        self.leader.assign_pending(1.0)
        assigns = self.transport.messages_of_type("TASK_ASSIGN")
        self.assertTrue(assigns)
        self.assertTrue(all(m.mission_version == self.leader.mission_version
                            for m in assigns))


if __name__ == "__main__":
    unittest.main()
