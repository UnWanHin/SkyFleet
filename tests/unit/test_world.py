"""楼群、节点与航点运动测试。"""
import unittest

from skyfleet.kernel.rng import RngRegistry
from skyfleet.protocol.identity import Role
from skyfleet.world.building import Building, grid_buildings
from skyfleet.world.mobility import WaypointMover
from skyfleet.world.node import Node


class TestGridBuildings(unittest.TestCase):
    def test_generates_expected_count(self):
        bs = grid_buildings(area=500.0, blocks=3, gap=20.0,
                            height_range=(30.0, 120.0),
                            rng=RngRegistry(1).get("b"))
        self.assertEqual(len(bs), 9)

    def test_buildings_stay_inside_area(self):
        for b in grid_buildings(500.0, 3, 20.0, (30.0, 120.0),
                                RngRegistry(1).get("b")):
            x, y, w, h = b.rect
            self.assertGreaterEqual(x, 0.0)
            self.assertGreaterEqual(y, 0.0)
            self.assertLessEqual(x + w, 500.0)
            self.assertLessEqual(y + h, 500.0)

    def test_heights_within_range(self):
        for b in grid_buildings(500.0, 3, 20.0, (30.0, 120.0),
                                RngRegistry(1).get("b")):
            self.assertGreaterEqual(b.height, 30.0)
            self.assertLessEqual(b.height, 120.0)

    def test_reproducible(self):
        a = grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(5).get("b"))
        b = grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(5).get("b"))
        self.assertEqual([x.height for x in a], [x.height for x in b])

    def test_ids_are_unique(self):
        bs = grid_buildings(500.0, 3, 20.0, (30.0, 120.0), RngRegistry(1).get("b"))
        self.assertEqual(len({b.id for b in bs}), len(bs))


class TestNode(unittest.TestCase):
    def test_consume_reduces_energy(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        n.consume(30.0)
        self.assertAlmostEqual(n.energy, 70.0)

    def test_energy_floors_at_zero(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=5.0)
        n.consume(50.0)
        self.assertEqual(n.energy, 0.0)

    def test_energy_never_negative(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=0.0)
        n.consume(1.0)
        self.assertEqual(n.energy, 0.0)

    def test_low_battery_is_still_online_but_not_available(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=10.0)
        self.assertTrue(n.is_useful(min_energy=20.0) is False)


class TestWaypointMover(unittest.TestCase):
    def test_moves_toward_waypoint(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(100.0, 0.0, 50.0)])
        self.assertFalse(mv.step(1.0))
        self.assertAlmostEqual(n.pos[0], 10.0)

    def test_arrives_and_advances(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(10.0, 0.0, 50.0), (20.0, 0.0, 50.0)])
        self.assertTrue(mv.step(1.0))
        self.assertAlmostEqual(n.pos[0], 10.0)
        self.assertFalse(mv.arrived)

    def test_reports_arrived_when_route_exhausted(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=100.0, energy=100.0)
        mv = WaypointMover(n, speed=100.0)
        mv.set_route([(10.0, 0.0, 50.0)])
        mv.step(1.0)
        self.assertTrue(mv.arrived)

    def test_leftover_distance_carries_to_next_leg(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(4.0, 0.0, 50.0), (14.0, 0.0, 50.0)])
        mv.step(1.0)
        self.assertAlmostEqual(n.pos[0], 10.0)

    def test_step_after_arrival_is_noop(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([(1.0, 0.0, 50.0)])
        mv.step(1.0)
        self.assertFalse(mv.step(1.0))
        self.assertAlmostEqual(n.pos[0], 1.0)

    def test_empty_route_is_arrived(self):
        n = Node("S1", Role.SLAVE, (0.0, 0.0, 50.0), speed=10.0, energy=100.0)
        mv = WaypointMover(n, speed=10.0)
        mv.set_route([])
        self.assertTrue(mv.arrived)


if __name__ == "__main__":
    unittest.main()
