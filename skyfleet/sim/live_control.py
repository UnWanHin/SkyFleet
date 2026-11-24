"""Loopback interactive controller for one running SkyFleet simulation."""
from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from skyfleet.sim.engine import Engine


@dataclass
class LiveRequest:
    nodes: list[str]
    destination: tuple[float, float, float]
    group: str


class LiveController:
    def __init__(self, engine: Engine, tick_seconds: float = .1):
        self.engine = engine
        self.tick_seconds = tick_seconds
        self.commands: queue.Queue[LiveRequest] = queue.Queue()
        self.lock = threading.Lock()
        self.snapshot = {"t": 0.0, "duration": engine.config.duration,
                         "nodes": {}, "edges": [], "components": [],
                         "buildings": [], "events": [], "running": False}
        self.finished = False
        self.shutdown = threading.Event()
        self.worker: threading.Thread | None = None
        self.failure: str | None = None
        self._last_radio_rx: dict[tuple[str, str], float] = {}
        self._selected: list[str] = []
        self._selection_version = 0

    @property
    def selection_version(self) -> int:
        with self.lock:
            return self._selection_version

    def selected(self) -> list[str]:
        with self.lock:
            return self._selected.copy()

    def set_selection(self, ids: list[str]) -> list[str]:
        if (not isinstance(ids, list) or
                any(not isinstance(n, str) or n not in self.engine.agents for n in ids) or
                len(ids) != len(set(ids))):
            raise ValueError("select distinct known drones")
        with self.lock:
            if ids != self._selected:
                self._selected = ids.copy()
                self._selection_version += 1
                self.snapshot = {**self.snapshot, "selected": ids.copy(),
                                 "selection_version": self._selection_version}
            return self._selected.copy()

    def start_radio_observation(self) -> None:
        def observe(src: str, dst: str, t: float) -> None:
            self._last_radio_rx[tuple(sorted((src, dst)))] = t
        self.engine.radio_observer = observe

    def submit(self, data: dict) -> dict:
        if self.finished or self.shutdown.is_set():
            raise ValueError("simulation finished")
        ids = data.get("nodes")
        dest = data.get("destination")
        group = data.get("group", "main")
        if (not isinstance(ids, list) or not ids or
                any(not isinstance(n, str) or n not in self.engine.agents
                    for n in ids) or
                len(set(ids)) != len(ids)):
            raise ValueError("select distinct known drones")
        if not isinstance(dest, list) or len(dest) != 3 or any(
                not isinstance(v, (int, float)) or isinstance(v, bool) or
                not math.isfinite(v) for v in dest):
            raise ValueError("destination must have three finite coordinates")
        if not isinstance(group, str) or not group.strip():
            raise ValueError("group must be nonempty")
        self.commands.put(LiveRequest(ids, tuple(map(float, dest)), group.strip()))
        return {"queued": ids}

    def tick(self, engine: Engine, t: float) -> None:
        while True:
            try:
                request = self.commands.get_nowait()
            except queue.Empty:
                break
            for index, node_id in enumerate(request.nodes):
                when = t + engine.config.motion_period + index * .1
                if node_id == engine.leader_id:
                    engine.dispatch_leader_local(when, request.destination,
                                                 request.group, adaptive=True)
                else:
                    engine.dispatch_live(when, [node_id], request.destination,
                                         request.group, adaptive=True)
        ids = sorted(n for n, a in engine.agents.items() if a.alive)
        edges = []
        peers = {n: set() for n in ids}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                link = engine.link_model.evaluate(engine.nodes[a], engine.nodes[b], t)
                if not link.obstructed and link.quality >= engine.config.loss_threshold:
                    edges.append([a, b]); peers[a].add(b); peers[b].add(a)
        components = []
        unseen = set(ids)
        while unseen:
            stack = [min(unseen)]; component = []
            while stack:
                n = stack.pop()
                if n not in unseen:
                    continue
                unseen.remove(n); component.append(n); stack.extend(sorted(peers[n] & unseen))
            components.append(sorted(component))
        nodes = {}
        for n in ids:
            node = engine.nodes[n]
            nodes[n] = {"pos": list(node.pos), "role": node.role.value,
                        "group": engine._group_ids.get(n, "main"),
                        "formation": engine._adaptive_modes.get(n, "-"),
                        "target": list(engine._adaptive_targets[n])
                        if n in engine._adaptive_targets else None}
            if engine.config.physics_movers and engine.movers[n] is not None:
                nodes[n].update(engine.movers[n].visual_pose())
        radio_links = [list(pair) for pair, last in sorted(self._last_radio_rx.items())
                       if t - last <= 1.5 and set(pair) <= set(ids)]
        snapshot = {"t": round(t, 2), "duration": engine.config.duration,
                    "running": True, "nodes": nodes, "edges": edges,
                    "radio_links": radio_links,
                    "components": components,
                    "buildings": [[*b.rect, b.height, b.base_height]
                                  for b in engine.buildings]}
        with self.lock:
            snapshot["selected"] = self._selected.copy()
            snapshot["selection_version"] = self._selection_version
            self.snapshot = snapshot
        self.shutdown.wait(self.tick_seconds)

    def stop(self) -> None:
        self.shutdown.set()

    def join(self) -> None:
        if self.worker is not None:
            self.worker.join()

    def start(self) -> threading.Thread:
        self.engine.enable_live_control()
        self.start_radio_observation()
        self.engine.live_hook = self.tick
        self.engine.scheduler.stop_requested = self.shutdown.is_set
        thread = threading.Thread(target=self._run, daemon=False)
        self.worker = thread
        thread.start()
        return thread

    def _run(self) -> None:
        try:
            self.engine.run()
        except Exception as exc:
            self.failure = str(exc)
        finally:
            self.finished = True
            with self.lock:
                self.snapshot = {**self.snapshot, "running": False,
                                 "error": self.failure}


def serve_live(config, *, host="127.0.0.1", port=8765, tick_seconds=.1,
               viewer="web"):
    from skyfleet.viz.live_page import LIVE_HTML, NATIVE_CONTROL_HTML
    engine = Engine(replace(config, tasks=0,
                            replay_sample_period=min(config.motion_period, .2))).build()
    if viewer not in ("web", "mujoco"):
        raise ValueError("viewer must be web or mujoco")
    if viewer == "mujoco" and not config.physics_movers:
        raise ValueError("native MuJoCo viewer needs physics_movers=true")
    controller = LiveController(engine, tick_seconds)
    controller.start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = (NATIVE_CONTROL_HTML if viewer == "mujoco" else LIVE_HTML).encode("utf-8")
                content_type = "text/html; charset=utf-8"
            elif self.path == "/api/state":
                with controller.lock:
                    body = json.dumps(controller.snapshot).encode()
                content_type = "application/json"
            else:
                self.send_error(404); return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(body)

        def do_POST(self):
            if self.path not in ("/api/dispatch", "/api/selection"):
                self.send_error(404); return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("request size out of bounds")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("expected JSON object")
                if self.path == "/api/selection":
                    result = {"selected": controller.set_selection(body.get("nodes"))}
                    code = 200
                else:
                    result = controller.submit(body)
                    code = 202
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                result = {"error": str(exc)}; code = 400
            payload = json.dumps(result).encode()
            self.send_response(code); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers(); self.wfile.write(payload)

    with ThreadingHTTPServer((host, port), Handler) as server:
        print(f"SkyFleet control: http://{host}:{server.server_port}/", flush=True)
        if viewer == "web":
            try:
                server.serve_forever()
            finally:
                controller.stop()
                controller.join()
        else:
            from flightlab.viewer import view_live
            http_thread = threading.Thread(target=server.serve_forever,
                                           daemon=False)
            http_thread.start()
            try:
                view_live(controller)
            finally:
                controller.stop()
                controller.join()  # Engine closes ns-3 before server teardown.
                server.shutdown()
                http_thread.join()
