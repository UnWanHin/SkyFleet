"""ns-3 transport contract and closed-loop smoke tests."""
from __future__ import annotations

import os
import unittest
from hashlib import sha256
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory

from skyfleet.kernel.config import load_config
from skyfleet.kernel.scheduler import Scheduler
from skyfleet.protocol.message import Msg
from skyfleet.sim.engine import EV_RX
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine
from skyfleet.sim.ns3_bridge import Ns3Bridge

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(os.environ.get("SKYFLEET_NS3_BRIDGE"), "ns-3 bridge binary not built")
class Ns3Integration(unittest.TestCase):
    def test_shadow_reduces_modeled_receive_power_for_same_delivery(self):
        from skyfleet.world.building import Building
        def power(blocks):
            with Ns3Bridge({"L": (0, 0, 30), "S1": (10, 0, 30)}, seed=4,
                           buildings=blocks, building_loss_db=5.) as radio:
                radio.send("shadow", "L", "S1", 256, .1)
                events = []
                while radio.now < 1.3:
                    events.extend(radio.advance(1.3))
                return next(e["modeled_rx_dbm"] for e in events
                            if e.get("dst") == "S1")
        wall = Building("wall", (4., -10., 2., 20.), 60.)
        self.assertAlmostEqual(power([]) - power([wall]), 5., places=3)

    def test_received_power_is_modeled_and_obstruction_attenuates(self):
        from skyfleet.world.building import Building
        block = Building("wall", (4., -10., 2., 20.), 60.)
        with Ns3Bridge({"L": (0, 0, 30), "S1": (10, 0, 30)}, seed=4,
                       buildings=[block], building_loss_db=5.) as radio:
            radio.send("shadow", "L", "S1", 256, 0.1)
            events = []
            while radio.now < 1.3:
                events.extend(radio.advance(1.3))
            self.assertTrue(any(x.get("done") for x in events))
            rx = [x for x in events if x.get("dst") == "S1"]
            self.assertTrue(rx, "moderate shadow should still deliver")
            self.assertIsInstance(rx[0]["modeled_rx_dbm"], float)
            self.assertTrue(-200 < rx[0]["modeled_rx_dbm"] < 25)

    def test_wifi_delivers_nearby_and_loses_distant_packet(self):
        with Ns3Bridge({"L": (0, 0, 10), "S1": (4, 0, 10),
                        "S2": (6000, 0, 10)}, seed=4) as radio:
            radio.send("near", "L", "S1", 256, 0.1)
            radio.send("far", "L", "S2", 256, 0.2)
            deliveries = []
            while radio.now < 0.6:
                deliveries.extend(radio.advance(0.6))
            self.assertIn(("near", "S1"), {(x["id"], x["dst"]) for x in deliveries})
            self.assertNotIn(("far", "S2"), {(x["id"], x["dst"]) for x in deliveries})
            self.assertTrue(all(x["t"] >= 0.1 for x in deliveries))

    def test_packet_retention_reports_completion_after_reception(self):
        with Ns3Bridge({"L": (0, 0, 10), "S1": (4, 0, 10),
                        "S2": (6000, 0, 10)}, seed=4) as radio:
            radio.send("near", "L", "S1", 256, 0.1)
            radio.send("far", "L", "S2", 256, 0.2)
            radio.send("broadcast", "L", "*", 256, 0.3)
            events = []
            while radio.now < 1.6:
                events.extend(radio.advance(1.6))
            for identifier in ("near", "far", "broadcast"):
                done = [x for x in events if x["id"] == identifier and x.get("done")]
                self.assertEqual(len(done), 1)
            self.assertNotIn("far", {x["id"] for x in events if not x.get("done")})
            near = [x for x in events if x["id"] == "near" and not x.get("done")]
            self.assertEqual(len(near), 1)
            self.assertLess(near[0]["t"], next(x["t"] for x in events
                                                if x["id"] == "near" and x.get("done")))

    def test_building_blocks_wifi_and_moving_out_restores_delivery(self):
        from skyfleet.world.building import Building

        buildings = [Building("B0", (40.0, -10.0, 20.0, 20.0), 60.0)]
        with Ns3Bridge({"L": (0, 0, 30), "S1": (100, 0, 30)}, seed=4,
                       buildings=buildings, building_loss_db=120.0) as radio:
            radio.send("blocked", "L", "S1", 256, 0.1)
            blocked = []
            while radio.now < 0.4:
                blocked.extend(radio.advance(0.4))
            self.assertNotIn("blocked", [x["id"] for x in blocked if not x.get("done")])
            radio.position("S1", (0, 100, 30))
            radio.send("restored", "L", "S1", 256, 0.5)
            restored = []
            while radio.now < 0.8:
                restored.extend(radio.advance(0.8))
            self.assertIn("restored", [x["id"] for x in restored if not x.get("done")])

    def test_building_relay_bypasses_shadow_and_overflight_clears_it(self):
        from skyfleet.world.building import Building

        building = Building("B0", (40.0, -10.0, 20.0, 20.0), 60.0)
        positions = {"L": (0, 0, 30), "D5": (100, 0, 30),
                     "D3": (50, 60, 80)}
        with Ns3Bridge(positions, seed=4, buildings=[building],
                       building_loss_db=120.0) as radio:
            radio.send("direct", "D5", "L", 256, 0.1)
            radio.send("relayone", "D5", "D3", 256, 0.2)
            radio.send("relaytwo", "D3", "L", 256, 0.3)
            seen = []
            while radio.now < 0.8:
                seen.extend(radio.advance(0.8))
            delivered = {x["id"] for x in seen if not x.get("done")}
            self.assertNotIn("direct", delivered)
            self.assertIn("relayone", delivered)
            self.assertIn("relaytwo", delivered)
            radio.position("D5", (100, 0, 100))
            radio.position("L", (0, 0, 100))
            radio.send("overflight", "D5", "L", 256, 0.9)
            seen = []
            while radio.now < 1.2:
                seen.extend(radio.advance(1.2))
            self.assertIn("overflight", {x["id"] for x in seen if not x.get("done")})

    def test_d5_replay_samples_flight_more_often_than_one_second(self):
        from skyfleet.study.ns3_d5 import run_comparison
        from skyfleet.observe.replay import build_timeline
        with TemporaryDirectory() as folder:
            report = run_comparison(Path(folder))
            samples = build_timeline(report["relay_log"])["nodes"]["S5"]["samples"]
            self.assertGreater(len(samples), 25)
            self.assertLess(max(b[0] - a[0] for a, b in zip(samples, samples[1:])), 1.0)

    def test_ns3_vote_quorum_degrades_with_three_voters(self):
        from skyfleet.kernel.config import SimConfig

        with TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=4, relays=0, tasks=0, duration=10.0,
                            blocks=1, area=100.0, network_backend="ns3",
                            election_votes_required=4,
                            kills=(("L", 2.0), ("S4", 2.0)),
                            log_path=str(Path(folder) / "election.jsonl"))
            events = list(read_log(Engine(cfg).run()))
            self.assertTrue(any(e["kind"] == "ELECTION_DEGRADED"
                                for e in events))
            self.assertEqual([e for e in events if e["kind"] == "LEADER_ANNOUNCE"], [])

    def test_ns3_vote_quorum_takeover_with_received_ballots(self):
        from skyfleet.kernel.config import SimConfig

        with TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=4, relays=0, tasks=0, duration=10.0,
                            blocks=1, area=100.0, network_backend="ns3",
                            election_votes_required=4,
                            kills=(("L", 2.0),),
                            log_path=str(Path(folder) / "election.jsonl"))
            events = list(read_log(Engine(cfg).run()))
            accepted = {e["node_id"] for e in events
                        if e["kind"] == "ELECTION_VOTE"}
            self.assertEqual(accepted, {"S1", "S2", "S3", "S4"})
            self.assertEqual(len([e for e in events
                                  if e["kind"] == "LEADER_ANNOUNCE"]), 1)

    def test_city_geojson_projected_rectangles_reach_ns3_and_geometry(self):
        from skyfleet.world.city_import import load_buildings_geojson
        import json

        with TemporaryDirectory() as folder:
            city = Path(folder) / "city.geojson"
            city.write_text(json.dumps({"type": "FeatureCollection", "skyfleet_units": "metres", "features": [
                {"type": "Feature", "id": "tower", "properties": {"height": 60},
                 "geometry": {"type": "Polygon", "coordinates": [
                     [[40, -10], [60, -10], [60, 10], [40, 10], [40, -10]]]}}
            ]}))
            buildings = load_buildings_geojson(city)
            self.assertEqual(len(buildings), 1)
            self.assertEqual(buildings[0].rect, (40.0, -10.0, 20.0, 20.0))
            with Ns3Bridge({"L": (0, 0, 30), "S5": (100, 0, 30)}, seed=4,
                           buildings=buildings, building_loss_db=120.0) as radio:
                radio.send("blocked", "S5", "L", 256, 0.1)
                seen = []
                while radio.now < 0.6:
                    seen.extend(radio.advance(0.6))
                self.assertNotIn("blocked", {e["id"] for e in seen if not e.get("done")})

    def test_city_import_rejects_degree_coordinates_and_nonrectangles(self):
        from skyfleet.world.city_import import load_buildings_geojson
        import json

        footprint = {"type": "Feature", "properties": {"height": 40},
                     "geometry": {"type": "Polygon", "coordinates": [[
                         [0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]]}}
        with TemporaryDirectory() as folder:
            city = Path(folder) / "city.geojson"
            city.write_text(json.dumps({"type": "FeatureCollection",
                                        "features": [footprint]}))
            with self.assertRaisesRegex(ValueError, "metres"):
                load_buildings_geojson(city)
            footprint["geometry"]["coordinates"][0][2] = [1, 2]
            city.write_text(json.dumps({"type": "FeatureCollection",
                            "skyfleet_units": "metres", "features": [footprint]}))
            with self.assertRaisesRegex(ValueError, "non-rectangular"):
                load_buildings_geojson(city)

    def test_city_config_loads_buildings_for_engine(self):
        import json
        from skyfleet.kernel.config import SimConfig

        with TemporaryDirectory() as folder:
            city = Path(folder) / "city.geojson"
            city.write_text(json.dumps({"type": "FeatureCollection",
                "skyfleet_units": "metres", "features": [{"type": "Feature",
                "properties": {"height": 60},
                "geometry": {"type": "Polygon", "coordinates": [[
                    [40, -10], [60, -10], [60, 10], [40, 10], [40, -10]]]}}]}))
            cfg = SimConfig(slaves=1, relays=0, tasks=0,
                            city_geojson=str(city), ns3_building_loss_db=120.0)
            engine = Engine(cfg).build()
            self.assertEqual(len(engine.buildings), 1)
            self.assertEqual(engine.buildings[0].height, 60)

    def test_urban_drone_actually_flies_through_clear_archway(self):
        from skyfleet.study.ns3_d5 import run_comparison, URBAN_CITY
        from skyfleet.observe.replay import build_timeline
        from skyfleet.world.city_import import load_buildings_geojson
        with TemporaryDirectory() as folder:
            report = run_comparison(Path(folder), city=URBAN_CITY)
            samples = build_timeline(report["relay_log"])["nodes"]["S5"]["samples"]
            beam = next(b for b in load_buildings_geojson(URBAN_CITY) if b.id == "ARCH_BEAM")
            x, y, w, d = beam.rect
            inside = [p for p in samples if x < p[1] < x+w and y < p[2] < y+d]
            self.assertGreaterEqual(len(inside), 5)
            self.assertLess(inside[0][2], inside[-1][2])
            self.assertTrue(all(0 < p[3] < beam.base_height for p in inside))
            self.assertEqual(report["control"]["occluded_direct_rx"], 0)
            self.assertEqual(report["relay"]["occluded_forwarded_rx"], 13)

    def test_elevated_archway_does_not_block_low_radio_but_blocks_upper_ray(self):
        from skyfleet.world.building import Building
        from skyfleet.sim.ns3_bridge import Ns3Bridge
        beam = Building("arch", (40, -10, 20, 20), 65, base_height=48)
        with Ns3Bridge({"A": (0, 0, 30), "B": (100, 0, 30)}, seed=3,
                       buildings=[beam], building_loss_db=120) as radio:
            radio.send("low", "A", "B", 256, 0.1)
            received = radio.advance(0.7)
            self.assertIn("low", {e["id"] for e in received if e.get("dst") == "B"})
        with Ns3Bridge({"A": (0, 0, 55), "B": (100, 0, 55)}, seed=3,
                       buildings=[beam], building_loss_db=120) as radio:
            radio.send("high", "A", "B", 256, 0.1)
            received = radio.advance(0.7)
            self.assertNotIn("high", {e["id"] for e in received if e.get("dst") == "B"})

    def test_urban_city_has_many_distinct_buildings_and_replays(self):
        from skyfleet.study.ns3_d5 import run_comparison, URBAN_CITY
        from skyfleet.world.city_import import load_buildings_geojson
        from skyfleet.viz.replay_3d import build_3d_timeline
        buildings = load_buildings_geojson(URBAN_CITY)
        self.assertGreaterEqual(len(buildings), 12)
        self.assertGreaterEqual(len({b.height for b in buildings}), 5)
        with TemporaryDirectory() as folder:
            report = run_comparison(Path(folder), city=URBAN_CITY)
            self.assertEqual(report["building_count"], len(buildings))
            self.assertEqual(report["control"]["occluded_direct_rx"], 0)
            self.assertGreater(report["relay"]["occluded_forwarded_rx"], 0)
            timeline = build_3d_timeline(report["relay_log"])
            self.assertEqual(len(timeline["buildings"]), len(buildings))
            self.assertTrue(timeline["terrain"])

    def test_d5_fixed_flight_relay_vs_direct_control(self):
        from skyfleet.study.ns3_d5 import run_comparison

        with TemporaryDirectory() as folder:
            report = run_comparison(Path(folder))
            self.assertLess(report["occlusion_enter"], report["occlusion_exit"])
            self.assertEqual(report["control"]["occluded_direct_rx"], 0)
            self.assertGreater(report["control"]["occluded_direct_tx"], 0)
            self.assertGreater(report["relay"]["occluded_relay_rx"], 0)
            self.assertGreater(report["relay"]["occluded_forwarded_rx"], 0)
            self.assertGreater(report["relay"]["recovered_direct_rx"], 0)
            self.assertGreater(report["relay"]["switches_to_d3"], 0)
            self.assertGreater(report["relay"]["switches_back_direct"], 0)
            self.assertEqual(report["control"]["occluded_relay_rx"], 0)
            self.assertEqual(report["relay"]["occluded_relay_rx"],
                             report["relay"]["occluded_forwarded_rx"])
            self.assertEqual(report["relay"]["recovered_direct_rx"],
                             report["control"]["recovered_direct_rx"])
            switches = report["relay"]["route_switches_observed_at_tx"]
            self.assertEqual([(s["next_hop"], s["version"]) for s in switches],
                             [("L", 1), ("R3", 2), ("L", 3)])

    def test_updated_positions_change_radio_delivery(self):
        with Ns3Bridge({"L": (0, 0, 10), "S1": (4, 0, 10)}, seed=4) as radio:
            radio.send("first", "L", "S1", 256, 0.1)
            self.assertIn("first", [x["id"] for x in radio.advance(0.3)])
            radio.advance(0.4)
            radio.position("S1", (6000, 0, 10))
            radio.send("second", "L", "S1", 256, 0.5)
            deliveries = []
            while radio.now < 0.8:
                deliveries.extend(radio.advance(0.8))
            self.assertNotIn("second", [x["id"] for x in deliveries])

    def test_expired_broadcast_and_lost_unicast_release_packet_state(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "expiry.jsonl"
            config = replace(load_config(str(ROOT / "skyfleet/scenarios/ns3_mujoco.json")),
                             duration=15.0, log_path=str(path))
            engine = Engine(config)
            engine.run()
            events = list(read_log(path))
            self.assertTrue(any(x["kind"] == "MSG_TX" and
                                x.get("msg_type") == "HEARTBEAT" for x in events))
            self.assertTrue(any(x["kind"] == "MSG_RX" and
                                x.get("msg_type") == "TELEMETRY" for x in events))
            self.assertLess(len(engine._ns3_pending), 10,
                            "expired packets must not accumulate for the entire run")
            self.assertEqual(set(engine._ns3_pending), set(engine._ns3_hops))

    def test_physics_telemetry_station_correction_through_ns3(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "run.jsonl"
            config = replace(load_config(str(ROOT / "skyfleet/scenarios/baseline_nlos.json")),
                             network_backend="ns3", physics_movers=True,
                             duration=15.0, log_path=str(path))
            engine = Engine(config)
            engine.run()
            events = list(read_log(path))
            self.assertTrue(any(x["kind"] == "MSG_RX" and x.get("flight")
                                for x in events), "physical motor data must arrive")
            self.assertTrue(any(x["kind"] == "MSG_RX" and
                                x.get("msg_type") == "FLIGHT_CORRECTION"
                                for x in events), "remote correction must reach aircraft")
            self.assertTrue(any(x["kind"] == "MSG_TX" and x.get("network_backend") == "ns3"
                                for x in events), "ns-3 must own packet delivery")

    def test_relay_forwarding_uses_ns3_hop(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "relay.jsonl"
            config = replace(load_config(str(ROOT / "skyfleet/scenarios/ns3_mujoco.json")),
                             duration=12.0, max_link_range=280.0,
                             relay_direct_threshold=0.95, log_path=str(path))
            Engine(config).run()
            events = list(read_log(path))
            forwards = [x for x in events if x["kind"] == "MSG_FORWARD"]
            self.assertTrue(forwards, "relay drone must forward actual traffic")
            forwarded_ids = {x["msg_id"] for x in forwards}
            self.assertTrue(any(x["kind"] == "MSG_TX" and
                                x.get("network_backend") == "ns3" and
                                x.get("msg_id") in forwarded_ids and
                                x.get("node_id", "").startswith("R")
                                for x in events), "relay hop must enter ns-3 Wi-Fi")

    def test_done_before_receive_does_not_advance_scheduler_past_receive(self):
        from types import SimpleNamespace
        engine = Engine(load_config(str(ROOT / "skyfleet/scenarios/ns3_mujoco.json")))
        scheduler = Scheduler()
        seen = []
        scheduler.subscribe(EV_RX, lambda ev: seen.append(ev.t))
        scheduler.schedule_at(3.0, "end")
        receiver = SimpleNamespace(node=SimpleNamespace(node_id="S1"), alive=True)
        sender = SimpleNamespace(node=SimpleNamespace(node_id="L"), alive=True)
        engine.agents = {"S1": receiver}
        msg = Msg.new("TELEMETRY", "L", "S1", 0.0)
        engine._ns3_pending = {"p1": (msg, sender, 0.0)}
        engine._ns3_hops = {"p1": "S1"}

        class Radio:
            now = 0.0

            def advance(self, target):
                if self.now == 0.0:
                    self.now = 1.0
                    return [{"id": "stale", "done": True, "t": 1.0}]
                if self.now == 1.0:
                    self.now = 2.0
                    return [{"id": "p1", "dst": "S1", "t": 2.0}]
                self.now = target
                return []

        engine._ns3 = Radio()
        engine.scheduler = scheduler
        scheduler.before_time = engine._ns3_advance
        scheduler.run(until=3.0)
        self.assertEqual(seen, [2.0])

    def test_same_seed_produces_identical_coupled_event_log(self):
        with TemporaryDirectory() as folder:
            config = load_config(str(ROOT / "skyfleet/scenarios/ns3_mujoco.json"))
            digests = []
            for index in range(2):
                path = Path(folder) / f"run{index}.jsonl"
                Engine(replace(config, duration=8.0, log_path=str(path))).run()
                digests.append(sha256(path.read_bytes()).hexdigest())
            self.assertEqual(digests[0], digests[1])

    def test_delivered_correction_changes_actual_mujoco_controller(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "run.jsonl"
            config = replace(load_config(str(ROOT / "skyfleet/scenarios/ns3_mujoco.json")),
                             motor_guard_ratio=0.1, duration=8.0, log_path=str(path))
            engine = Engine(config)
            engine.run()
            events = list(read_log(path))
            self.assertTrue(any(x["kind"] == "MSG_RX" and
                                x.get("msg_type") == "FLIGHT_CORRECTION"
                                for x in events))
            self.assertTrue(any(m._ready and m._sim.gains.v_max == 6.0
                                for m in engine.movers.values() if m is not None),
                            "the network command must change the physical flight controller")
