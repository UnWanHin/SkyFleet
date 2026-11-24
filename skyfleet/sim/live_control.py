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
        self._hop_stats: dict[tuple[str, str], dict] = {}
        self._received_hops = 0
        self._lost_hops = 0
        self._total_hop_delay_ms = 0.0
        self._received_e2e = 0
        self._total_e2e_delay_ms = 0.0
        self._selected: list[str] = []
        self._selection_version = 0
        self._preview: list[float] | None = None

    def set_preview(self, destination: list[float] | None) -> list[float] | None:
        """Publish a display-only target; delivery begins only at submit()."""
        if self.finished or self.shutdown.is_set():
            raise ValueError("simulation finished")
        if destination is not None:
            if (not isinstance(destination, list) or len(destination) != 3 or
                    any(not isinstance(v, (int, float)) or isinstance(v, bool)
                        or not math.isfinite(v) for v in destination)):
                raise ValueError("preview must have three finite coordinates")
            destination = list(map(float, destination))
        with self.lock:
            self._preview = destination
            self.snapshot = {**self.snapshot, "preview": destination}
            return destination

    def preview(self) -> tuple[float, float, float] | None:
        with self.lock:
            return tuple(self._preview) if self._preview is not None else None

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

        def hop(src: str, dst: str, sent: float, received: float | None,
                dbm: float | None, broadcast: bool) -> None:
            pair = tuple(sorted((src, dst)))
            record = self._hop_stats.setdefault(pair, {
                "a": pair[0], "b": pair[1], "received": 0, "lost": 0,
                "delay_ms": None, "modeled_rx_dbm": None,
                "last_rx": None, "last_loss": None})
            if received is None:
                record["lost"] += 1
                record["last_loss"] = sent + 1.1  # MAC retention expiry
                self._lost_hops += 1
            else:
                delay = max(0., (received - sent) * 1000.)
                record["received"] += 1
                record["last_rx"] = received
                record["delay_ms"] = round(delay, 3)
                record["modeled_rx_dbm"] = dbm
                self._received_hops += 1
                self._total_hop_delay_ms += delay
        self.engine.telemetry_observer = hop

        def end_to_end(seconds: float) -> None:
            self._received_e2e += 1
            self._total_e2e_delay_ms += seconds * 1000.
        self.engine.e2e_observer = end_to_end

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
        with self.lock:
            self._preview = None
            self.snapshot = {**self.snapshot, "preview": None}
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
                        "protocol_parent": engine.agents[n].upstream()
                        if n != engine.leader_id else None,
                        "target": list(engine._adaptive_targets[n])
                        if n in engine._adaptive_targets else None}
            if engine.config.physics_movers and engine.movers[n] is not None:
                nodes[n].update(engine.movers[n].visual_pose())
        # Candidate path from recent observed reception adjacency (undirected);
        # it does not certify the protocol's selected next hop or packet route.
        adjacency: dict[str, dict[str, dict]] = {name: {} for name in ids}
        for pair, stat in self._hop_stats.items():
            if (stat["last_rx"] is not None and t - stat["last_rx"] <= 1.5 and
                    pair[0] in adjacency and pair[1] in adjacency):
                adjacency[pair[0]][pair[1]] = stat
                adjacency[pair[1]][pair[0]] = stat
        paths = {engine.leader_id: [engine.leader_id]}
        queue_paths = [engine.leader_id]
        for parent in queue_paths:
            for child in sorted(adjacency.get(parent, {})):
                if child not in paths:
                    paths[child] = [*paths[parent], child]
                    queue_paths.append(child)
        leader_pos = engine.nodes[engine.leader_id].pos
        for name, node in nodes.items():
            path = paths.get(name)
            parent = path[-2] if path is not None and len(path) > 1 else None
            node["path"] = path
            node["parent"] = parent
            node["distance_to_leader_m"] = round(math.dist(node["pos"], leader_pos), 2)
            node["modeled_rx_dbm"] = adjacency[parent][name]["modeled_rx_dbm"] if parent else None
        formation_skeleton = []
        grouped: dict[tuple[str, tuple], list[str]] = {}
        for name in ids:
            target = engine._adaptive_targets.get(name)
            if target is not None:
                grouped.setdefault((engine._group_ids.get(name, "main"), tuple(target)), []).append(name)
        for (_, target), members in grouped.items():
            members.sort()
            cols = math.ceil(math.sqrt(len(members)))
            for rank, name in enumerate(members):
                row, col = divmod(rank, cols)
                spacing = 3.0
                ref = [target[0] + (col - (cols - 1) / 2) * spacing,
                       target[1] + (row - (cols - 1) / 2) * spacing, target[2]]
                nodes[name]["formation_reference"] = ref
                nodes[name]["formation_error_m"] = round(math.dist(nodes[name]["pos"], ref), 2)
                if rank:
                    formation_skeleton.append([members[rank - 1], name])
        radio_edges = []
        for pair, stat in sorted(self._hop_stats.items()):
            last_rx, last_loss = stat["last_rx"], stat["last_loss"]
            if pair[0] not in ids or pair[1] not in ids:
                continue
            if last_loss is not None and t - last_loss <= 3 and (last_rx is None or last_loss > last_rx):
                status = "lost"
            elif last_rx is not None and t - last_rx <= 1.5:
                status = "edge" if (stat["modeled_rx_dbm"] is not None and stat["modeled_rx_dbm"] < -75) or (stat["delay_ms"] is not None and stat["delay_ms"] > 30) else "good"
            else:
                continue
            kind = ("direct" if engine.leader_id in pair else
                    "relay" if any(name.startswith("R") for name in pair) else "peer")
            # Pure geometry: evaluating LinkModel here would mutate its trend
            # history and thereby change the protocol's future routing input.
            from skyfleet.world.geometry import segment_intersects_rect
            pa, pb = engine.nodes[pair[0]].pos, engine.nodes[pair[1]].pos
            obstructed = any(
                b.height >= min(pa[2], pb[2]) and
                b.base_height <= max(pa[2], pb[2]) and
                segment_intersects_rect(pa[:2], pb[:2], b.rect)
                for b in engine.buildings)
            if status == "good" and obstructed:
                status = "edge"
            radio_edges.append({**stat, "role": kind, "status": status,
                                "modeled_obstruction": obstructed})
        radio_links = [list(pair) for pair, last in sorted(self._last_radio_rx.items())
                       if t - last <= 1.5 and set(pair) <= set(ids)]
        network = {"connectivity": round((len(paths) - 1) / max(1, len(ids) - 1), 3) if engine.config.network_backend == "ns3" else None,
                   "avg_hop_delay_ms": round(self._total_hop_delay_ms / self._received_hops, 3) if self._received_hops else None,
                   "avg_e2e_delay_ms": round(self._total_e2e_delay_ms / self._received_e2e, 3) if self._received_e2e else None,
                   "lost": self._lost_hops, "received": self._received_hops}
        snapshot = {"t": round(t, 2), "duration": engine.config.duration,
                    "running": True, "nodes": nodes, "edges": edges,
                    "radio_links": radio_links, "radio_edges": radio_edges,
                    "network": network, "formation_skeleton": formation_skeleton,
                    "components": components,
                    "buildings": [[*b.rect, b.height, b.base_height]
                                  for b in engine.buildings]}
        with self.lock:
            snapshot["selected"] = self._selected.copy()
            snapshot["preview"] = self._preview.copy() if self._preview is not None else None
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
            if self.path not in ("/api/dispatch", "/api/selection", "/api/preview"):
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
                elif self.path == "/api/preview":
                    result = {"preview": controller.set_preview(body.get("destination"))}
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
