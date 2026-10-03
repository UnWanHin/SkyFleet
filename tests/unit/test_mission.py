"""任务生命周期、投递幂等与安全动作测试。"""
import unittest

from skyfleet.mission.delivery import DeliveryLedger
from skyfleet.mission.safety import SafetyAction, safety_decision
from skyfleet.mission.task import TaskBoard, TaskState


class TestTaskLifecycle(unittest.TestCase):
    def setUp(self):
        self.b = TaskBoard()
        self.b.create("T1", (0.0, 0.0, 50.0), (100.0, 100.0, 50.0),
                      deadline=100.0, t=0.0)

    def test_create_starts_created(self):
        self.assertIs(self.b.get("T1").state, TaskState.CREATED)

    def test_assign_sets_assignee(self):
        self.assertTrue(self.b.assign("T1", "S1", 0.0))
        task = self.b.get("T1")
        self.assertIs(task.state, TaskState.ASSIGNED)
        self.assertEqual(task.assignee, "S1")

    def test_happy_path(self):
        self.assertTrue(self.b.assign("T1", "S1", 0.0))
        for state in (TaskState.ENROUTE, TaskState.PICKED, TaskState.DELIVERED):
            self.assertTrue(self.b.transition("T1", state, 1.0))
        self.assertIs(self.b.get("T1").state, TaskState.DELIVERED)

    def test_cannot_go_enroute_before_assignment(self):
        self.assertFalse(self.b.transition("T1", TaskState.ENROUTE, 1.0))

    def test_cannot_skip_to_delivered(self):
        self.assertFalse(self.b.transition("T1", TaskState.DELIVERED, 1.0))

    def test_delivered_is_terminal(self):
        self.b.assign("T1", "S1", 0.0)
        self.b.transition("T1", TaskState.ENROUTE, 1.0)
        self.b.transition("T1", TaskState.PICKED, 2.0)
        self.b.transition("T1", TaskState.DELIVERED, 3.0)
        self.assertFalse(self.b.transition("T1", TaskState.ENROUTE, 4.0))

    def test_unknown_task_transition_is_false(self):
        self.assertFalse(self.b.transition("NOPE", TaskState.ENROUTE, 1.0))

    def test_failure_then_reassign(self):
        self.b.assign("T1", "S1", 0.0)
        self.assertTrue(self.b.transition("T1", TaskState.FAILED, 1.0))
        self.assertTrue(self.b.transition("T1", TaskState.REASSIGNED, 2.0))
        self.assertTrue(self.b.assign("T1", "S2", 3.0))

    def test_counts(self):
        self.b.assign("T1", "S1", 0.0)
        counts = self.b.counts()
        self.assertEqual(counts.get("ASSIGNED"), 1)
        self.assertEqual(counts.get("CREATED", 0), 0)

    def test_completion_rate_helper(self):
        self.b.assign("T1", "S1", 0.0)
        for s in (TaskState.ENROUTE, TaskState.PICKED, TaskState.DELIVERED):
            self.b.transition("T1", s, 1.0)
        self.assertAlmostEqual(self.b.completion_rate(), 1.0)

    def test_completion_rate_zero_without_tasks(self):
        self.assertEqual(TaskBoard().completion_rate(), 0.0)


class TestDeliveryLedger(unittest.TestCase):
    def test_first_confirmation_executes(self):
        self.assertTrue(DeliveryLedger().confirm("op-1", 0.0))

    def test_repeat_confirmation_is_rejected(self):
        ledger = DeliveryLedger()
        ledger.confirm("op-1", 0.0)
        self.assertFalse(ledger.confirm("op-1", 1.0))

    def test_duplicate_is_counted(self):
        ledger = DeliveryLedger()
        ledger.confirm("op-1", 0.0)
        ledger.confirm("op-1", 1.0)
        ledger.confirm("op-1", 2.0)
        self.assertEqual(ledger.confirm_count, 1)
        self.assertEqual(ledger.duplicate_count, 2)

    def test_distinct_operations_both_execute(self):
        ledger = DeliveryLedger()
        self.assertTrue(ledger.confirm("op-1", 0.0))
        self.assertTrue(ledger.confirm("op-2", 0.0))

    def test_is_confirmed(self):
        ledger = DeliveryLedger()
        ledger.confirm("op-1", 0.0)
        self.assertTrue(ledger.is_confirmed("op-1"))
        self.assertFalse(ledger.is_confirmed("op-9"))


class TestSafetyDecision(unittest.TestCase):
    def test_all_clear_is_none(self):
        self.assertIs(safety_decision(), SafetyAction.NONE)

    def test_link_lost_hovers(self):
        self.assertIs(safety_decision(link_lost=True), SafetyAction.HOVER)

    def test_leader_lost_hovers(self):
        self.assertIs(safety_decision(leader_lost=True), SafetyAction.HOVER)

    def test_low_battery_returns(self):
        self.assertIs(safety_decision(low_battery=True), SafetyAction.RETURN)

    def test_position_stale_hovers(self):
        self.assertIs(safety_decision(position_stale=True), SafetyAction.HOVER)

    def test_geofence_holds(self):
        self.assertIs(safety_decision(geofence=True), SafetyAction.HOLD)

    def test_low_battery_beats_link_loss(self):
        self.assertIs(safety_decision(link_lost=True, low_battery=True),
                      SafetyAction.RETURN)

    def test_geofence_beats_low_battery(self):
        self.assertIs(safety_decision(geofence=True, low_battery=True),
                      SafetyAction.HOLD)


if __name__ == "__main__":
    unittest.main()
