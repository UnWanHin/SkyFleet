"""仿真引擎装配测试。"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.log import normalised_digest, read_log
from skyfleet.sim.engine import Engine


def small_config(tmpdir, **overrides):
    base = dict(seed=1, duration=20.0, slaves=4, relays=1, tasks=6,
                blocks=2, area=400.0,
                log_path=str(Path(tmpdir) / "run.jsonl"))
    base.update(overrides)
    return SimConfig(**base)


class TestBuild(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = small_config(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_builds_configured_node_count(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertEqual(len(engine.nodes), 6)

    def test_has_one_leader(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertIn(self.cfg.leader_id, engine.nodes)
        self.assertEqual(engine.nodes[self.cfg.leader_id].role.value, "LEADER")

    def test_builds_buildings(self):
        engine = Engine(self.cfg)
        engine.build()
        self.assertEqual(len(engine.buildings), 4)

    def test_creates_relays(self):
        engine = Engine(self.cfg)
        engine.build()
        relays = [n for n in engine.nodes.values() if n.role.value == "RELAY"]
        self.assertEqual(len(relays), 1)


class TestInteractiveDispatch(unittest.TestCase):
    def test_adaptive_route_switches_matrix_column_and_recovers(self):
        from skyfleet.world.geometry import adaptive_route
        from skyfleet.world.building import Building
        walls = [Building("left", (10, 0, 5, 20), 40),
                 Building("right", (17, 0, 5, 20), 40)]
        clear, mode = adaptive_route((16, -10, 10), (16, 30, 10),
                                     [], {}, node_id="S1", members=["S1", "S2"])
        self.assertEqual(mode, "MATRIX")
        route, mode = adaptive_route((16, -10, 10), (16, 30, 10),
                                     walls, {}, node_id="S1", members=["S1", "S2"])
        self.assertEqual(mode, "COLUMN")
        self.assertTrue(all(p[0] == 16 for p in route[:-1]))
        inside, inside_mode = adaptive_route((16, 10, 10), (16, 30, 10),
                                              walls, {}, node_id="S1",
                                              members=["S1", "S2"])
        self.assertEqual(inside_mode, "COLUMN")
        self.assertGreaterEqual(inside[0][1], 10)
        recovered, mode = adaptive_route((16, 24, 10), (16, 30, 10),
                                          walls, {}, node_id="S1", members=["S1", "S2"])
        self.assertEqual(mode, "MATRIX")
        self.assertEqual(recovered[-1][2], 10)

    def test_adaptive_command_logs_formation_after_radio_acceptance(self):
        from skyfleet.observe.log import read_log
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=2, relays=0, tasks=0,
                               blocks=1, area=120, duration=5)
            engine = Engine(cfg).build()
            engine.nodes["L"].pos = (0, -5, 10)
            engine.nodes["S1"].pos = (0, 0, 10)
            engine.nodes["S2"].pos = (3, 0, 10)
            engine.schedule_dispatch(1.5, ["S1", "S2"], (0, 30, 10),
                                     "east", adaptive=True)
            engine.run()
            events = list(read_log(cfg.log_path))
            accepted = [e for e in events if e["kind"] == "CONTROL_ACCEPTED"]
            formations = [e for e in events if e["kind"] == "FORMATION_CHANGE"]
            self.assertTrue(accepted)
            self.assertTrue(formations)
            self.assertGreaterEqual(formations[0]["t"], accepted[0]["t"])

    def test_adaptive_hold_does_not_claim_arrival(self):
        from skyfleet.world.building import Building
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=1, relays=0, tasks=0,
                               blocks=1, area=120, duration=3)
            engine = Engine(cfg).build()
            engine.buildings = [Building("sealed", (4, -10, 12, 80), 60)]
            engine.nodes["L"].pos = (0, -5, 10)
            engine.nodes["S1"].pos = (0, 0, 10)
            engine.schedule_dispatch(1.5, ["S1"], (20, 30, 10),
                                     "east", adaptive=True)
            engine.run()
            events = list(read_log(cfg.log_path))
            self.assertTrue(any(e["kind"] == "CONTROL_ACCEPTED" for e in events))
            self.assertTrue(any(e["kind"] == "FORMATION_CHANGE" and
                                e["mode"] == "HOLD" for e in events))
            self.assertFalse(any(e["kind"] == "CONTROL_ARRIVED" for e in events))

    def test_adaptive_reacts_to_building_added_during_flight(self):
        from skyfleet.world.building import Building
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=1, relays=0, tasks=0,
                               blocks=1, area=120, duration=5)
            engine = Engine(cfg).build()
            engine.buildings = []
            engine.nodes["L"].pos = (0, -5, 10)
            engine.nodes["S1"].pos = (0, 0, 10)
            engine.schedule_dispatch(1.5, ["S1"], (0, 30, 10),
                                     "east", adaptive=True)
            inserted = []
            def introduce_wall(sim, t):
                if not inserted and t >= 2.5:
                    sim.buildings.append(Building("new-wall", (-5, 8, 10, 15), 40))
                    inserted.append(t)
            engine.live_hook = introduce_wall
            engine.run()
            events = list(read_log(cfg.log_path))
            changes = [(e["t"], e["mode"]) for e in events
                       if e["kind"] == "FORMATION_CHANGE" and
                          e["node_id"] == "S1"]
            self.assertTrue(inserted)
            self.assertIn("MATRIX", [mode for _, mode in changes])
            self.assertIn("HOLD", [mode for _, mode in changes])
            self.assertFalse(any(e["kind"] == "CONTROL_ARRIVED" for e in events))

    def test_adaptive_peer_separation_holds_later_drone(self):
        from skyfleet.world.geometry import adaptive_route
        route, mode = adaptive_route((0, 0, 10), (0, 30, 10), [],
                                     {"S1": (0, 1, 10)}, node_id="S2",
                                     members=["S1", "S2"])
        self.assertEqual((route, mode), ([], "HOLD"))

    def test_narrow_passage_staggers_column_and_restores_matrix(self):
        from skyfleet.world.geometry import plan_passage
        from skyfleet.world.building import Building
        pillars = [Building("left", (10, 0, 5, 20), 40),
                   Building("right", (17, 0, 5, 20), 40)]
        routes = plan_passage((16, 30, 10), ["S1", "S2", "S3", "S4"],
                              pillars, spacing=3, clearance=0.4,
                              drone_radius=0.3)
        self.assertEqual(set(routes), {"S1", "S2", "S3", "S4"})
        # Each flight crosses between x=15 and 17, not through a pillar.
        for index, node_id in enumerate(sorted(routes)):
            route = routes[node_id]
            self.assertEqual(route[-1],
                             (16 + ((index % 2) - .5) * 3,
                              30 + ((index // 2) - .5) * 3, 10))
            inside = [point for point in route if 0 <= point[1] <= 20]
            self.assertTrue(inside)
            self.assertTrue(all(point[0] == 16 for point in inside))
        self.assertEqual(len({routes[n][0][1] for n in routes}), 4)
        # Later aircraft queue farther from the corridor entrance.
        for index, node_id in enumerate(sorted(routes)):
            self.assertAlmostEqual(routes[node_id][0][1], -3.7 - index * 3)

    def test_synthetic_archway_regroups_before_northern_tower(self):
        from skyfleet.world.city_import import load_buildings_geojson
        from skyfleet.world.geometry import plan_passage
        buildings = load_buildings_geojson(
            "skyfleet/scenarios/d5_urban_city.geojson")
        routes = plan_passage((50, 92, 30), ["S1", "S2"], buildings,
                              through=(50, 75, 30))
        self.assertEqual({node: route[-1] for node, route in routes.items()},
                         {"S1": (48.5, 90.5, 30),
                          "S2": (51.5, 90.5, 30)})
        with self.assertRaisesRegex(ValueError, "route intersects building N2"):
            plan_passage((50, 100, 30), ["S1", "S2"], buildings,
                         through=(50, 75, 30))

    def test_dispatch_with_deadline_expired_leaves_route_unchanged(self):
        from skyfleet.kernel.config import SimConfig
        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=8, slaves=1, relays=0, tasks=0,
                            blocks=1, area=120,
                            log_path=str(Path(tmp) / "undelivered.jsonl"),
                            network_backend="classic", physics_movers=False)
            engine = Engine(cfg).build()
            engine.nodes["S1"].pos = (5000, 5000, 80)
            original = engine.movers["S1"].route[:]
            engine.schedule_dispatch(1.5, ["S1"], (60, 0, 30), "east")
            engine.run()
            self.assertEqual(engine.movers["S1"].route, original)

    def test_dispatch_changes_route_only_after_wireless_delivery(self):
        from skyfleet.kernel.config import SimConfig
        from skyfleet.observe.log import read_log
        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=12, slaves=4, relays=0, tasks=0,
                            blocks=1, area=120,
                            log_path=str(Path(tmp) / "dispatch.jsonl"),
                            network_backend="classic", physics_movers=False)
            engine = Engine(cfg).build()
            for name, pos in {"L":(0,0,30),"S1":(10,0,30),"S2":(12,0,30),
                              "S3":(14,0,30),"S4":(16,0,30)}.items():
                engine.nodes[name].pos = pos
            engine.schedule_dispatch(2, ["S1", "S2", "S3"], (60, 0, 30),
                                     group_id="east")
            engine.run()
            events = list(read_log(cfg.log_path))
            for node_id in ("S1", "S2", "S3"):
                received = next(e for e in events if e["kind"] == "MSG_RX" and
                                e.get("msg_type") == "FLIGHT_CORRECTION" and
                                e.get("node_id") == node_id)
                accepted = next(e for e in events if e["kind"] == "CONTROL_ACCEPTED" and
                                e.get("node_id") == node_id)
                self.assertGreaterEqual(accepted["t"], received["t"])
            self.assertEqual(engine.nodes["S1"].pos, (60.0, 0.0, 30.0))
            self.assertTrue(any(e["kind"] == "GROUP_CHANGE" for e in events))

    def test_opt_in_requires_four_distinct_delivered_votes(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=4, relays=0, tasks=0,
                               election_votes_required=4)
            engine = Engine(cfg).build()
            engine._open_log()
            try:
                engine.agents["L"].alive = False
                engine._maybe_elect(5.0)
                self.assertEqual(engine.leader_id, "L")
                self.assertTrue(engine._election_degraded)
                for voter in ("S1", "S2", "S3"):
                    engine._record_election_vote(voter, "S1", 0, 5.1)
                engine._maybe_elect(5.2)
                self.assertEqual(engine.leader_id, "L")
                engine._record_election_vote("S3", "S1", 0, 5.3)
                engine._record_election_vote("S4", "S1", 0, 5.4)
                engine._maybe_elect(5.5)
                self.assertEqual(engine.leader_id, "S1")
            finally:
                engine.log.close()

    def test_four_vote_takeover_and_three_vote_degradation_on_transport(self):
        for alive in (4, 3):
            with self.subTest(alive=alive), tempfile.TemporaryDirectory() as tmp:
                cfg = small_config(tmp, slaves=4, relays=0, tasks=0,
                                   duration=16.0, blocks=1, area=100.0,
                                   election_votes_required=4,
                                   kills=(("L", 2.0),) +
                                         (("S4", 2.0),) * (alive == 3))
                events = list(read_log(Engine(cfg).run()))
                announces = [e for e in events if e["kind"] == "LEADER_ANNOUNCE"]
                votes = [e for e in events if e["kind"] == "ELECTION_VOTE"]
                self.assertEqual(bool(announces), alive == 4)
                self.assertEqual(len({e["node_id"] for e in votes}), alive)
                if alive == 3:
                    self.assertTrue(any(e["kind"] == "ELECTION_DEGRADED"
                                        for e in events))

    def test_three_votes_actuate_hover_after_grace_period(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=4, relays=0, tasks=0,
                               duration=8.0, blocks=1, area=100.0,
                               election_votes_required=4,
                               kills=(("L", 2.0), ("S4", 2.0)))
            engine = Engine(cfg).build()
            engine.movers["S2"].set_route([(80.0, 80.0, 140.0)])
            events = list(read_log(engine.run()))
            actuated = [e for e in events if e["kind"] == "SAFETY_ACTION"
                        and e.get("cause") == "ELECTION_QUORUM"
                        and e.get("actuated") and e.get("node_id") == "S2"]
            self.assertTrue(actuated)
            self.assertEqual(engine.movers["S2"].route, [])

    def test_quorum_votes_from_old_epoch_or_dead_node_are_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = small_config(tmp, slaves=4, relays=0, tasks=0,
                               election_votes_required=4)
            engine = Engine(cfg).build()
            engine._open_log()
            try:
                engine.agents["L"].alive = False
                engine.agents["S4"].alive = False
                for voter in ("S1", "S2", "S3", "S4"):
                    engine._record_election_vote(voter, "S1", 0, 5.0)
                engine._record_election_vote("S4", "S1", 1, 5.0)
                engine._maybe_elect(5.1)
                self.assertEqual(engine.leader_id, "L")
            finally:
                engine.log.close()


class TestRun(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cfg = small_config(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_run_writes_log(self):
        engine = Engine(self.cfg)
        path = engine.run()
        self.assertTrue(Path(path).exists())
        self.assertGreater(len(list(read_log(path))), 10)

    def test_log_records_all_nodes_joining(self):
        path = Engine(self.cfg).run()
        joined = {r.get("node_id") for r in read_log(path)
                  if r["kind"] == "NODE_JOIN"}
        self.assertEqual(len(joined), 6)

    def test_log_records_task_creation(self):
        path = Engine(self.cfg).run()
        created = [r for r in read_log(path) if r["kind"] == "TASK_CREATE"]
        self.assertEqual(len(created), 6)

    def test_log_records_message_traffic(self):
        path = Engine(self.cfg).run()
        kinds = {r["kind"] for r in read_log(path)}
        self.assertIn("MSG_TX", kinds)

    def test_log_records_drops_with_reason(self):
        path = Engine(self.cfg).run()
        drops = [r for r in read_log(path) if r["kind"] == "MSG_DROP"]
        self.assertTrue(all(r.get("reason") for r in drops))

    def test_run_is_deterministic(self):
        first = Engine(self.cfg).run()
        digest_a = normalised_digest(first)
        second = Engine(self.cfg).run()
        self.assertEqual(digest_a, normalised_digest(second))

    def test_different_seed_changes_outcome(self):
        a = Engine(small_config(self._tmp.name, seed=1)).run()
        digest_a = normalised_digest(a)
        b = Engine(small_config(self._tmp.name, seed=2)).run()
        self.assertNotEqual(digest_a, normalised_digest(b))

    def test_rerun_truncates_previous_log(self):
        Engine(self.cfg).run()
        path = Engine(self.cfg).run()
        joined = [r for r in read_log(path) if r["kind"] == "NODE_JOIN"]
        self.assertEqual(len(joined), 6)


if __name__ == "__main__":
    unittest.main()
