"""离散事件调度器测试：核心命题是确定性。"""
import unittest

from skyfleet.kernel.scheduler import Scheduler


class TestOrdering(unittest.TestCase):
    def test_runs_events_in_time_order(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.payload))
        s.schedule_at(2.0, "e", "b")
        s.schedule_at(1.0, "e", "a")
        s.schedule_at(3.0, "e", "c")
        s.run()
        self.assertEqual(seen, ["a", "b", "c"])

    def test_same_timestamp_uses_insertion_order(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.payload))
        for tag in ("first", "second", "third"):
            s.schedule_at(1.0, "e", tag)
        s.run()
        self.assertEqual(seen, ["first", "second", "third"])


class TestClock(unittest.TestCase):
    def test_clock_advances_to_current_event(self):
        s, stamps = Scheduler(), []
        s.subscribe("e", lambda ev: stamps.append(s.now))
        s.schedule_at(5.0, "e")
        s.run()
        self.assertEqual(stamps, [5.0])

    def test_now_starts_at_zero(self):
        self.assertEqual(Scheduler().now, 0.0)


class TestScheduling(unittest.TestCase):
    def test_schedule_is_relative_to_current_time(self):
        # 注意：schedule_at 只是入堆，不推进时钟；相对调度须在时钟已推进后才有意义
        s, seen = Scheduler(), []
        s.subscribe("a", lambda ev: s.schedule(2.5, "b"))
        s.subscribe("b", lambda ev: seen.append(ev.t))
        s.schedule_at(10.0, "a")
        s.run()
        self.assertEqual(seen, [12.5])

    def test_negative_delay_is_clamped(self):
        s = Scheduler()
        s.schedule_at(5.0, "noop")
        s.run()
        self.assertEqual(s.schedule(-3.0, "e").t, 5.0)

    def test_run_until_stops_early(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(ev.t))
        s.schedule_at(1.0, "e")
        s.schedule_at(9.0, "e")
        s.run(until=5.0)
        self.assertEqual(seen, [1.0])

    def test_cancel_prevents_handler(self):
        s, seen = Scheduler(), []
        s.subscribe("e", lambda ev: seen.append(1))
        ev = s.schedule_at(1.0, "e")
        self.assertTrue(s.cancel(ev))
        s.run()
        self.assertEqual(seen, [])

    def test_cancel_twice_is_false(self):
        s = Scheduler()
        ev = s.schedule_at(1.0, "e")
        s.cancel(ev)
        self.assertFalse(s.cancel(ev))

    def test_events_can_schedule_more_events(self):
        s, seen = Scheduler(), []

        def handler(ev):
            seen.append(ev.payload)
            if ev.payload < 3:
                s.schedule(1.0, "e", ev.payload + 1)

        s.subscribe("e", handler)
        s.schedule_at(0.0, "e", 1)
        s.run()
        self.assertEqual(seen, [1, 2, 3])


class TestTimestampHook(unittest.TestCase):
    """信道竞争需要在"同一时刻的事件处理完之后"统一结算。"""

    def test_on_timestamp_end_fires_once_per_timestamp(self):
        s, marks = Scheduler(), []
        s.on_timestamp_end = marks.append
        s.subscribe("e", lambda ev: None)
        s.schedule_at(1.0, "e")
        s.schedule_at(1.0, "e")
        s.schedule_at(2.0, "e")
        s.run()
        self.assertEqual(marks, [1.0, 2.0])

    def test_hook_fires_for_final_timestamp(self):
        s, marks = Scheduler(), []
        s.on_timestamp_end = marks.append
        s.subscribe("e", lambda ev: None)
        s.schedule_at(7.0, "e")
        s.run()
        self.assertEqual(marks, [7.0])

    def test_no_hook_is_fine(self):
        s = Scheduler()
        s.subscribe("e", lambda ev: None)
        s.schedule_at(1.0, "e")
        s.run()


class TestDeterminism(unittest.TestCase):
    def test_two_runs_of_same_schedule_are_identical(self):
        def build():
            s, seen = Scheduler(), []
            s.subscribe("e", lambda ev: seen.append(ev.payload))
            for i in (3, 1, 2, 1):
                s.schedule_at(float(i), "e", i)
            s.run()
            return seen

        self.assertEqual(build(), build())


if __name__ == "__main__":
    unittest.main()
