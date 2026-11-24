"""Tests for commands injected into the same running simulation."""
import tempfile
import time
import unittest
from pathlib import Path

from skyfleet.kernel.config import SimConfig
from skyfleet.observe.log import EventLog, read_log
from skyfleet.sim.engine import Engine
from skyfleet.sim.live_control import LiveController


class TestLiveControl(unittest.TestCase):
    def test_ns3_expired_unicast_is_counted_from_done_event(self):
        from skyfleet.protocol.message import Msg
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "expiry.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            class Radio:
                now = 0.
                def advance(self, t):
                    self.now = t
                    return [{"id": "packet", "done": True, "t": t}]
            engine._ns3 = Radio()
            msg = Msg.new("TELEMETRY", "L", "S1", .1)
            engine._ns3_pending["packet"] = (msg, engine.agents["L"], .1)
            engine._ns3_hops["packet"] = "S1"
            engine._ns3_advance(1.2)
            control.tick(engine, 1.2)
            self.assertEqual(control.snapshot["network"]["lost"], 1)
            self.assertEqual(control.snapshot["radio_edges"][0]["status"], "lost")

    def test_building_obstruction_marks_received_link_marginal(self):
        from skyfleet.world.building import Building
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "wall.jsonl"))
            engine = Engine(cfg).build()
            leader, slave = engine.nodes["L"], engine.nodes["S1"]
            center_x = (leader.pos[0] + slave.pos[0]) / 2
            center_y = (leader.pos[1] + slave.pos[1]) / 2
            wall = Building("barrier", (center_x - 100, center_y - 100, 200, 200), 500.)
            engine.buildings.append(wall)
            engine.link_model.buildings.append(wall)
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.telemetry_observer("L", "S1", .1, .106, -50., False)
            control.tick(engine, .2)
            edge = control.snapshot["radio_edges"][0]
            self.assertTrue(edge["modeled_obstruction"])
            self.assertEqual(edge["status"], "edge")

    def test_recent_loss_does_not_turn_into_false_successful_route(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "stale.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.telemetry_observer("L", "S1", 0., .005, -50., False)
            engine.telemetry_observer("L", "S1", .1, None, None, False)
            control.tick(engine, .2)
            self.assertEqual(control.snapshot["nodes"]["S1"]["path"], ["L", "S1"])
            self.assertEqual(control.snapshot["radio_edges"][0]["status"], "lost")

    def test_network_connectivity_uses_observed_receptions_not_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "c.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            control.tick(engine, .1)
            self.assertEqual(control.snapshot["network"]["connectivity"], 0.)
            self.assertIsNone(control.snapshot["nodes"]["S1"]["path"])
            engine.telemetry_observer("L", "S1", .1, .107, -40., False)
            control.tick(engine, .2)
            self.assertEqual(control.snapshot["network"]["connectivity"], 1.)
            control.tick(engine, 2.)
            self.assertEqual(control.snapshot["network"]["connectivity"], 0.)

    def test_accepted_e2e_delay_does_not_count_forwarding(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=1, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "e2e.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.e2e_observer(.012)
            engine.e2e_observer(.020)
            control.tick(engine, .1)
            self.assertEqual(control.snapshot["network"]["avg_e2e_delay_ms"], 16.)

    def test_native_viewer_draws_formation_deviation(self):
        import mujoco
        from flightlab.viewer import LiveSwarmDisplay
        display = LiveSwarmDisplay(["L", "S1"])
        nodes = {"L": {"qpos": [0., 0., 20., 1., 0., 0., 0.]},
                 "S1": {"qpos": [5., 0., 20., 1., 0., 0., 0.],
                        "formation_reference": [10., 0., 20.],
                        "formation_error_m": 5.}}
        display.update({"nodes": nodes, "radio_edges": []})
        scene = mujoco.MjvScene(display.model, maxgeom=128)
        display.overlay(scene)
        self.assertTrue(any(g.type == mujoco.mjtGeom.mjGEOM_CAPSULE and
                            .2 < g.rgba[0] < .9 and .2 < g.rgba[1] < .9
                            for g in scene.geoms[:scene.ngeom]))

    def test_web_telemetry_panel_binds_to_snapshot(self):
        from skyfleet.viz.live_page import NATIVE_CONTROL_HTML
        self.assertIn('id="node-telemetry"', NATIVE_CONTROL_HTML)
        self.assertIn('avg_e2e_delay_ms', NATIVE_CONTROL_HTML)
        self.assertIn('formation_skeleton', NATIVE_CONTROL_HTML)
        self.assertIn('radio_edges', NATIVE_CONTROL_HTML)
        self.assertIn('telemetry(s)', NATIVE_CONTROL_HTML)

    def test_native_overlay_draws_graded_link_and_formation(self):
        import mujoco
        from flightlab.viewer import LiveSwarmDisplay
        display = LiveSwarmDisplay(["L", "R1", "S1"])
        nodes = {name: {"qpos": [i * 5., 0., 20., 1., 0., 0., 0.],
                        "pos": [i * 5., 0., 20.], "target": None}
                 for i, name in enumerate(display.ids)}
        display.update({"nodes": nodes, "radio_edges": [
            {"a": "L", "b": "R1", "status": "good", "role": "direct"},
            {"a": "R1", "b": "S1", "status": "lost", "role": "relay"}],
            "formation_skeleton": [["R1", "S1"]]})
        scene = mujoco.MjvScene(display.model, maxgeom=128)
        display.overlay(scene)
        link_colors = [tuple(g.rgba[:3]) for g in scene.geoms[:scene.ngeom]
                       if g.type == mujoco.mjtGeom.mjGEOM_CAPSULE]
        self.assertTrue(any(c[1] > c[0] * 1.4 for c in link_colors))
        self.assertTrue(any(c[0] > c[1] * 1.4 for c in link_colors))
        self.assertGreaterEqual(len(link_colors), 3)

    def test_protocol_upstream_is_separate_from_observed_candidate_path(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=1, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "upstream.jsonl"))
            engine = Engine(cfg).build()
            engine.agents["S1"].routing.parent = "R1"
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            control.tick(engine, .1)
            node = control.snapshot["nodes"]["S1"]
            self.assertEqual(node["protocol_parent"], "R1")
            self.assertIsNone(node["path"])

    def test_formation_reference_and_relay_path_are_separate(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=2, relays=1, tasks=0, blocks=1,
                            network_backend="ns3", log_path=str(Path(folder) / "f.jsonl"))
            engine = Engine(cfg).build()
            engine._group_ids.update({"S1": "east", "S2": "east"})
            engine._adaptive_targets.update({"S1": (20, 30, 40), "S2": (20, 30, 40)})
            engine._adaptive_modes.update({"S1": "MATRIX", "S2": "MATRIX"})
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.telemetry_observer("L", "R1", 0., .006, -65., False)
            engine.telemetry_observer("R1", "S1", 0., .013, -71., False)
            control.tick(engine, .1)
            state = control.snapshot
            self.assertEqual(state["nodes"]["S1"]["path"], ["L", "R1", "S1"])
            self.assertEqual(state["nodes"]["S1"]["parent"], "R1")
            self.assertGreater(state["nodes"]["S1"]["distance_to_leader_m"], 0)
            self.assertEqual(state["nodes"]["S1"]["modeled_rx_dbm"], -71.)
            self.assertEqual(len(state["formation_skeleton"]), 1)
            self.assertNotEqual(state["nodes"]["S1"]["formation_reference"],
                                state["nodes"]["S2"]["formation_reference"])
            self.assertGreaterEqual(state["nodes"]["S1"]["formation_error_m"], 0)

    def test_ns3_telemetry_counts_delivered_and_expired_unicast(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, tasks=0, blocks=1, network_backend="ns3",
                            log_path=str(Path(folder) / "radio.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.telemetry_observer("L", "S1", 1.0, 1.006, -70., False)
            engine.telemetry_observer("L", "S1", 1.2, None, None, False)
            control.tick(engine, 1.5)
            state = control.snapshot
            edge = next(e for e in state["radio_edges"] if e["a"] == "L" and e["b"] == "S1")
            self.assertEqual(edge["received"], 1)
            self.assertEqual(edge["lost"], 1)
            self.assertEqual(edge["status"], "lost")
            self.assertAlmostEqual(edge["delay_ms"], 6.)
            self.assertEqual(edge["modeled_rx_dbm"], -70.)
            self.assertEqual(state["network"]["lost"], 1)
            self.assertEqual(state["network"]["avg_hop_delay_ms"], 6.)
            self.assertEqual(state["network"]["connectivity"], .333)

    def test_stop_signal_terminates_engine_before_bridge_close(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            duration=100, physics_movers=True,
                            log_path=str(Path(folder) / "stop.jsonl"))
            control = LiveController(Engine(cfg).build(), tick_seconds=.001)
            thread = control.start()
            try:
                deadline = time.monotonic() + 5
                while control.snapshot["t"] < .2 and thread.is_alive():
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.005)
                control.stop()
                thread.join(5)
                self.assertFalse(thread.is_alive())
                self.assertFalse(control.snapshot["running"])
                self.assertLess(control.snapshot["t"], cfg.duration)
            finally:
                control.stop()
                thread.join(5)

    def test_web_preview_is_shared_without_dispatching(self):
        from skyfleet.viz.live_page import NATIVE_CONTROL_HTML
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "preview.jsonl"))
            control = LiveController(Engine(cfg).build())
            with self.assertRaises(ValueError):
                control.set_preview([1, float('nan'), 3])
            self.assertEqual(control.set_preview([12, 24, 36]), [12., 24., 36.])
            self.assertEqual(control.snapshot['preview'], [12., 24., 36.])
            self.assertTrue(control.commands.empty())
            control.submit({'nodes': ['S1'], 'destination': [12, 24, 36],
                            'group': 'main'})
            self.assertIsNone(control.snapshot['preview'])
            control.set_preview([12, 24, 36])
            control.set_preview(None)
            self.assertIsNone(control.snapshot['preview'])
            self.assertIn('id="minimap"', NATIVE_CONTROL_HTML)
            self.assertIn('/api/preview', NATIVE_CONTROL_HTML)

    def test_native_viewer_close_waits_for_render_thread(self):
        from flightlab.viewer import native_viewer_session
        from unittest.mock import patch
        class FakeViewer:
            def close(self):
                timeline.append("viewer.close")
        class FakeThread:
            name = "Thread-1 (_launch_internal)"
            def join(self):
                timeline.append("render.join")
            def is_alive(self):
                return False
        timeline = []
        with patch("flightlab.viewer.mujoco.viewer.launch_passive",
                   return_value=FakeViewer()), patch(
                   "flightlab.viewer.threading.enumerate",
                   side_effect=[[object()], [object(), FakeThread()]]):
            with native_viewer_session(object(), object()) as viewer:
                self.assertIsInstance(viewer, FakeViewer)
            self.assertEqual(timeline, ["viewer.close", "render.join"])

    def test_physics_snapshot_uses_real_mujoco_qpos(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            physics_movers=True,
                            log_path=str(Path(folder) / "log.jsonl"))
            engine = Engine(cfg).build()
            mover = engine.movers["S1"]
            sim = mover._ensure_sim()
            sim.data.qpos[0:3] = [2., 3., 11.]
            sim.data.qpos[3:7] = [0.5, 0.5, 0.5, 0.5]
            sim.data.qpos[7] = 123.
            control = LiveController(engine, tick_seconds=0)
            control.tick(engine, 0.1)
            state = control.snapshot["nodes"]["S1"]
            self.assertEqual(state["qpos"], list(sim.data.qpos[:7]))
            self.assertEqual(state["rotors"][0], 123.)

    def test_radio_observation_uses_actual_ns3_hop_including_forwarding(self):
        from skyfleet.protocol.message import Msg
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=1, tasks=0, blocks=1,
                            network_backend="ns3",
                            log_path=str(Path(folder) / "log.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine._ns3 = SimpleNamespace(now=0.1, advance=lambda t: [
                {"id": "packet", "dst": "L", "t": 0.2}])
            engine.log = EventLog(cfg.log_path)
            try:
                msg = Msg.new("HEARTBEAT", src="S1", dst="L", t=0,
                              msg_id="hop-check", seq=1, session_id="s",
                              leader_epoch=0, mission_version=0)
                engine._ns3_pending["packet"] = (msg, engine.agents["R1"], 0.1)
                engine._ns3_hops["packet"] = "L"
                seen = []
                engine.scheduler.subscribe("rx", lambda ev: seen.append(ev.payload))
                engine._ns3_advance(0.2)
                self.assertEqual(len(seen), 0)
                self.assertEqual(engine.scheduler.pending(), 1)
                rx = engine.scheduler._heap[0][2].payload
                self.assertEqual(rx.radio_from, "R1")
            finally:
                engine.log.close()

    def test_actual_radio_links_expire_without_receipts(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            network_backend="ns3",
                            log_path=str(Path(folder) / "log.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            control.start_radio_observation()
            engine.radio_observer("L", "S1", 1.0)
            control.tick(engine, 1.1)
            self.assertEqual(control.snapshot["radio_links"], [["L", "S1"]])
            control.tick(engine, 3.0)
            self.assertEqual(control.snapshot["radio_links"], [])

    def test_viewer_display_scene_copies_pose_without_stepping(self):
        from flightlab.viewer import LiveSwarmDisplay
        from skyfleet.world.building import Building
        display = LiveSwarmDisplay(["L", "S1"],
                                   [Building("block", (10, 20, 4, 8), 30)])
        state = {"nodes": {"L": {"pos": [0, 0, 12],
                                 "qpos": [0, 0, 12, 1, 0, 0, 0],
                                 "rotors": [1, 2, 3, 4]},
                           "S1": {"pos": [30, 40, 50],
                                  "qpos": [30, 40, 50, .5, .5, .5, .5],
                                  "rotors": [4, 3, 2, 1]}}}
        start = display.data.time
        display.update(state)
        self.assertEqual(display.data.time, start)
        self.assertEqual(list(display.data.qpos[display._qpos[1]:
                                                display._qpos[1] + 7]),
                         state["nodes"]["S1"]["qpos"])
        self.assertAlmostEqual(display.data.body("d1").xpos[2], 50.)
        self.assertGreater(display.model.ngeom, 2 * 11)

    def test_viewer_overlay_draws_waypoint_and_observed_radio(self):
        import mujoco
        from flightlab.viewer import LiveSwarmDisplay
        display = LiveSwarmDisplay(["L", "S1"])
        display.update({"nodes": {"L": {"qpos": [0, 0, 10, 1, 0, 0, 0]},
                                  "S1": {"qpos": [2, 4, 10, 1, 0, 0, 0],
                                         "target": [3, 6, 8]}},
                        "radio_links": [["L", "S1"]]})
        scene = mujoco.MjvScene(display.model, maxgeom=64)
        display.overlay(scene)
        self.assertEqual(scene.ngeom, 6)  # markers, labels, target, ns-3 hop
        self.assertEqual(scene.geoms[4].type, mujoco.mjtGeom.mjGEOM_SPHERE)
        self.assertEqual(scene.geoms[5].type, mujoco.mjtGeom.mjGEOM_CAPSULE)
        display.overlay(scene, preview=(12., 24., 36.))
        self.assertEqual(scene.ngeom, 8)
        self.assertEqual(scene.geoms[5].type, mujoco.mjtGeom.mjGEOM_SPHERE)
        self.assertEqual(list(scene.geoms[5].pos), [12., 24., 36.])
        self.assertAlmostEqual(scene.geoms[5].rgba[0], 1.)
        display.update({"nodes": {"L": {"qpos": [0, 0, 10, 1, 0, 0, 0]},
                                  "S1": {"qpos": [2, 4, 10, 1, 0, 0, 0]}},
                        "radio_links": []})
        display.overlay(scene)
        self.assertEqual(scene.ngeom, 4)

    def test_selection_is_shared_with_web_and_snapshots(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=2, relays=1, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build(), tick_seconds=0)
            self.assertEqual(control.set_selection(["R1", "S2"]), ["R1", "S2"])
            control.tick(control.engine, .1)
            self.assertEqual(control.snapshot["selected"], ["R1", "S2"])
            self.assertEqual(control.snapshot["selection_version"], 1)
            with self.assertRaises(ValueError):
                control.set_selection(["R1", "R1"])

    def test_selection_matches_web_native_node_ids_and_click_ignores_city(self):
        from flightlab.viewer import LiveSwarmDisplay
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=2, relays=2, tasks=0, blocks=1,
                            physics_movers=True,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            display = LiveSwarmDisplay(sorted(control.engine.nodes))
            self.assertEqual(set(display.ids), set(control.engine.nodes))
            self.assertIsNone(display.body_to_node(0))
            self.assertEqual(display.body_to_node(display.model.body("d4").id),
                             display.ids[4])

    def test_selection_rejects_malformed_nonlist_ids(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            for invalid in (None, "S1", ["S1", {}], ["S1", "S1"]):
                with self.assertRaises(ValueError):
                    control.set_selection(invalid)

    def test_native_key_preview_and_submit_uses_shared_control(self):
        from flightlab.viewer import LiveKeys
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=2, relays=2, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            control.set_selection(["S1"])
            keys = LiveKeys(control, ["L", "R1", "R2", "S1", "S2"])
            keys.process([50], (10, 20, 30), {"S1": [0, 0, 22]})
            self.assertEqual(control.selected(), ["R1", "R2"])
            keys.process([70, 93], (10, 20, 30),
                         {"R1": [0, 0, 22]})
            self.assertEqual(keys.preview, (10., 20., 27.))
            keys.process([257], (10, 20, 30), {})
            self.assertEqual(control.commands.get_nowait().nodes, ["R1", "R2"])
            self.assertIsNone(keys.preview)
            keys.process([256], (0, 0, 0), {})
            self.assertIsNone(keys.preview)
            keys.process([65], (0, 0, 0), {})
            self.assertEqual(control.selected(), ["L", "R1", "R2", "S1", "S2"])
            keys.process([258, (258, True)], (0, 0, 0), {})
            self.assertEqual(control.selected(), ["S2"])

    def test_shift_tab_modifier_is_captured_at_key_event(self):
        from flightlab.viewer import LiveKeys
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=2, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            keys = LiveKeys(control, ["L", "S1", "S2"])
            control.set_selection(["S1"])
            keys.process([(258, True)], (0, 0, 0), {})
            self.assertEqual(control.selected(), ["L"])
            keys.process([(258, False)], (0, 0, 0), {})
            self.assertEqual(control.selected(), ["S1"])

    def test_native_display_scales_visual_geometry_without_changing_physics_pose(self):
        import numpy as np
        from flightlab.viewer import LiveSwarmDisplay
        standard = LiveSwarmDisplay(["L", "S1"], visual_scale=1)
        enhanced = LiveSwarmDisplay(["L", "S1"], visual_scale=4)
        geom = standard.model.body("d0").geomadr
        self.assertAlmostEqual(enhanced.model.geom_size[geom, 0],
                               4 * standard.model.geom_size[geom, 0])
        rotor = enhanced.model.body("d0_rotor0").id
        self.assertAlmostEqual(enhanced.model.body_pos[rotor, 0],
                               4 * standard.model.body_pos[rotor, 0])
        state = {"nodes": {"L": {"qpos": [250, 250, 160, 1, 0, 0, 0]},
                           "S1": {"qpos": [251, 250, 160, 1, 0, 0, 0]}}}
        enhanced.update(state)
        np.testing.assert_allclose(enhanced.data.body("d0").xpos,
                                   [250, 250, 160])
        self.assertEqual(enhanced.data.time, 0.)

    def test_camera_starts_at_leader_and_follows_selected_on_c(self):
        from flightlab.viewer import LiveCamera, LiveKeys
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            control.set_selection(["S1"])
            keys = LiveKeys(control, ["L", "S1"])
            camera = SimpleNamespace(lookat=[0., 0., 0.], distance=550.,
                                     elevation=-45.)
            follow = LiveCamera(camera)
            follow.start([250., 250., 160.])
            self.assertEqual(camera.lookat, [250., 250., 160.])
            self.assertLessEqual(camera.distance, 30.)
            keys.process([67], (0, 0, 0), {})
            follow.update(keys, control.selected(), {"S1": [12., 13., 44.]})
            self.assertEqual(camera.lookat, [12., 13., 44.])
            follow.update(keys, control.selected(), {"S1": [14., 15., 45.]})
            self.assertEqual(camera.lookat, [14., 15., 45.])

    def test_native_labels_highlights_and_preview_differ_from_targets(self):
        import mujoco
        from flightlab.viewer import LiveSwarmDisplay
        display = LiveSwarmDisplay(["L", "R1", "S1"])
        display.update({"nodes": {name: {"qpos": [i*3, 0, 10, 1, 0, 0, 0],
                                        "target": [8, 8, 15] if name == "S1" else None}
                                  for i, name in enumerate(display.ids)},
                        "radio_links": []})
        scene = mujoco.MjvScene(display.model, maxgeom=128)
        display.overlay(scene, selected=["R1"], preview=(5, 6, 20))
        geoms = scene.geoms[:scene.ngeom]
        self.assertEqual({g.label for g in geoms if g.type == mujoco.mjtGeom.mjGEOM_LABEL},
                         {"L", "R1", "S1", "(5.0, 6.0, 20.0)"})
        self.assertTrue(any(g.type == mujoco.mjtGeom.mjGEOM_CYLINDER
                            and g.pos[0] == 3 for g in geoms))
        self.assertTrue(any(g.type == mujoco.mjtGeom.mjGEOM_SPHERE
                            and g.pos[0] == 8 and g.rgba[1] > g.rgba[0]
                            for g in geoms))
        self.assertTrue(any(g.type == mujoco.mjtGeom.mjGEOM_SPHERE
                            and g.pos[0] == 5 and g.rgba[0] > g.rgba[1]
                            for g in geoms))
        self.assertEqual(display.body_to_node(display.model.body("d1").id), "R1")
        self.assertEqual(display.body_to_node(display.model.body("d1_rotor0").id), "R1")

    def test_received_live_command_changes_formation(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(seed=7, duration=6, slaves=2, relays=0, tasks=0,
                            blocks=1, area=120,
                            log_path=str(Path(folder) / "live.jsonl"))
            engine = Engine(cfg).build()
            for name, pos in {"L": (0, 0, 30), "S1": (2, 0, 30),
                              "S2": (5, 0, 30)}.items():
                engine.nodes[name].pos = pos
            control = LiveController(engine, tick_seconds=.002)
            thread = control.start()
            try:
                deadline = time.monotonic() + 2
                while not control.snapshot["running"] and not control.finished:
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.001)
                queued = control.submit({"nodes": ["S1", "S2"],
                                         "destination": [10, 20, 30],
                                         "group": "east"})
                self.assertEqual(queued, {"queued": ["S1", "S2"]})
                thread.join(2)
                self.assertFalse(thread.is_alive())
                self.assertIsNone(control.failure)
                events = list(read_log(cfg.log_path))
                accepted = [e for e in events if e["kind"] == "CONTROL_ACCEPTED"]
                self.assertEqual({e["node_id"] for e in accepted}, {"S1", "S2"})
                formations = [e for e in events if e["kind"] == "FORMATION_CHANGE"]
                self.assertTrue(formations)
                self.assertGreaterEqual(formations[0]["t"], accepted[0]["t"])
                self.assertEqual(control.snapshot["nodes"]["S1"]["group"], "east")
                self.assertTrue(control.snapshot["components"])
            finally:
                thread.join(2)

    def test_native_control_accepts_leader_and_relay_selection(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=1, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            engine = Engine(cfg).build()
            control = LiveController(engine, tick_seconds=0)
            engine.enable_live_control()
            engine._subscribe()
            self.assertEqual(control.submit({"nodes": ["L", "R1", "S1"],
                                             "destination": [20, 30, 40],
                                             "group": "east"})["queued"],
                             ["L", "R1", "S1"])
            control.tick(engine, 1.0)
            from skyfleet.sim.engine import EV_LOCAL_DISPATCH
            event = next(e for _, _, e in engine.scheduler._heap
                         if e.kind == EV_LOCAL_DISPATCH)
            self.assertEqual(event.t, 1.1)
            engine.log = EventLog(cfg.log_path)
            try:
                engine._on_local_dispatch(event)
            finally:
                engine.log.close()
            self.assertIn("L", engine._adaptive_targets)
            self.assertEqual(engine.movers["L"].route[-1], (20., 30., 40.))

    def test_rejects_invalid_commands(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = SimConfig(slaves=1, relays=0, tasks=0, blocks=1,
                            log_path=str(Path(folder) / "log.jsonl"))
            control = LiveController(Engine(cfg).build())
            for command in ({"nodes": ["missing"], "destination": [1, 2, 3]},
                            {"nodes": ["S1"], "destination": [1, 2, float("nan")]},
                            {"nodes": ["S1", "S1"], "destination": [1, 2, 3]},
                            {"nodes": [{}], "destination": [1, 2, 3]}):
                with self.assertRaises(ValueError):
                    control.submit(command)
