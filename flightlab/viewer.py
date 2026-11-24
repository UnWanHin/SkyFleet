"""交互式实时查看器：真的把仿真"开起来"看。

**与 render.py 的分工**：render 出的是固定镜头的视频，适合放进报告；
viewer 是实时窗口，鼠标能自由转视角、能暂停、能改目标高度，适合调试和演示。

**为什么用 launch_passive 而不是 launch**：passive 模式下我们自己的循环拥有
物理步进的节奏，能精确控制实时倍率、能在每步插入控制器。非 passive 模式
由 viewer 内部驱动，插入自定义控制的接口受限。

**已实测的已知问题**：Wayland 下会打印
`Failed to load plugin 'libdecor-gtk.so'`——这是窗口装饰插件缺失，不影响
渲染与交互。退出时可能有一次 core dump（GLFW/MuJoCo 的 teardown 竞态），
仿真本身已完成，不影响结果。
"""
from __future__ import annotations

import math
import os
import time
import threading
from contextlib import contextmanager

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco                                     # noqa: E402
import mujoco.viewer                              # noqa: E402
import numpy as np                                # noqa: E402

from flightlab.params import DEFAULT as PARAMS     # noqa: E402
from flightlab.sim import QuadrotorSim             # noqa: E402
from flightlab.swarm import SwarmLayout, SwarmSim  # noqa: E402


class LiveSwarmDisplay:
    """A display-only MJCF mirror; SkyFleet's per-aircraft simulators own physics.

    Call update on the viewer thread with a locked snapshot from LiveController.
    Never call SwarmSim.step here: that would simulate a second, divergent fleet.
    """

    def __init__(self, ids, buildings=(), *, visual_scale=4.):
        if not math.isfinite(visual_scale) or visual_scale <= 0:
            raise ValueError("visual_scale must be finite and positive")
        self.ids = tuple(ids)
        self.visual_scale = visual_scale
        self.swarm = SwarmSim(SwarmLayout(count=len(self.ids)),
                              buildings=buildings)
        self.model, self.data = self.swarm.model, self.swarm.data
        # This model is a display-only mirror. Scale body-local offsets and
        # geometry, never free-joint qpos or the authoritative flight models.
        if visual_scale != 1:
            for body_id in range(1, self.model.nbody):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                         body_id) or ""
                if "_rotor" in name:
                    self.model.body_pos[body_id] *= visual_scale
            for geom_id in range(self.model.ngeom):
                body_id = int(self.model.geom_bodyid[geom_id])
                if self._drone_body(body_id):
                    self.model.geom_size[geom_id] *= visual_scale
                    self.model.geom_pos[geom_id] *= visual_scale
            mujoco.mj_setConst(self.model, self.data)
        self._qpos = self.swarm._qpos
        self._rotors = [[self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                              f"d{i}_motor{j}")] for j in range(4)]
            for i in range(len(self.ids))]
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.latest = None

    def update(self, snapshot):
        for i, name in enumerate(self.ids):
            node = snapshot.get("nodes", {}).get(name)
            if node is None:
                continue
            pose = node.get("qpos")
            if pose is None:
                raise ValueError("native viewer requires physical qpos per aircraft")
            if len(pose) != 7:
                raise ValueError("invalid free-joint qpos")
            self.data.qpos[self._qpos[i]:self._qpos[i] + 7] = pose
            for index, angle in zip(self._rotors[i], node.get("rotors", ())):
                self.data.qpos[index] = angle
        mujoco.mj_forward(self.model, self.data)
        self.latest = snapshot

    @staticmethod
    def _geom(scene, kind, size, pos, color):
        if scene.ngeom >= scene.maxgeom:
            return None
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, kind, np.asarray(size, dtype=float),
                            np.asarray(pos, dtype=float), np.eye(3).ravel(),
                            np.asarray(color, dtype=np.float32))
        scene.ngeom += 1
        return geom

    def _drone_body(self, body_id):
        while body_id > 0:
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                     body_id) or ""
            if name.startswith("d") and name[1:].isdigit():
                return True
            body_id = int(self.model.body_parentid[body_id])
        return False

    def body_to_node(self, body_id):
        while body_id > 0:
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                     body_id)
            if name and name.startswith("d") and name[1:].isdigit():
                index = int(name[1:])
                return self.ids[index] if index < len(self.ids) else None
            body_id = int(self.model.body_parentid[body_id])
        return None

    def overlay(self, scene, *, selected=(), preview=None, show_coverage=False,
                render_t=None):
        """Draw identifiers, selected rings, preview and accepted ns-3 hops."""
        scene.ngeom = 0
        if not self.latest:
            return
        nodes = self.latest.get("nodes", {})
        for i, name in enumerate(self.ids):
            node = nodes.get(name)
            if node is None:
                continue
            pos = node["qpos"][:3]
            # A colored marker distinguishes L / relays / selected slaves.
            color = (.95, .3, .2, .9) if name == "L" else (
                (.35, .9, .5, .85) if name.startswith("R") else (.3, .65, 1., .85))
            self._geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                       (.45, .45, .45), (pos[0], pos[1], pos[2] + 1.5), color)
            label = self._geom(scene, mujoco.mjtGeom.mjGEOM_LABEL,
                               (.1, .1, .1), (pos[0], pos[1], pos[2] + 3),
                               (1., 1., 1., 1.))
            if label is not None:
                label.label = name
            if name in selected:
                self._geom(scene, mujoco.mjtGeom.mjGEOM_CYLINDER,
                           (2., .08, .08), pos, (1., .9, .08, .85))
            target = node.get("target")
            if target is not None:
                self._geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                           (.8, .8, .8), target, (.1, 1., .35, .95))
        if preview is not None:
            self._geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                       (1.3, 1.3, 1.3), preview, (1., .78, .1, .4))
            label = self._geom(scene, mujoco.mjtGeom.mjGEOM_LABEL,
                               (.1, .1, .1), (preview[0], preview[1],
                                              preview[2] + 3), (1., 1., .4, 1.))
            if label is not None:
                label.label = str(tuple(round(float(x), 1) for x in preview))
        edges = self.latest.get("radio_edges")
        if edges is None:
            edges = [{"a": a, "b": b, "status": "good"}
                     for a, b in self.latest.get("radio_links", [])]
        for edge in edges:
            a, b = edge["a"], edge["b"]
            if a not in nodes or b not in nodes:
                continue
            start, end = np.asarray(nodes[a]["qpos"][:3]), np.asarray(nodes[b]["qpos"][:3])
            status = edge.get("status", "good")
            color = {"good": (.15, 1., .45, .9), "edge": (1., .67, .12, .85),
                     "lost": (1., .2, .18, .9)}.get(status, (.5, .5, .5, .7))
            segments = ((0., .2), (.4, .6), (.8, 1.)) if status == "lost" else ((0., 1.),)
            for lo, hi in segments:
                geom = self._geom(scene, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                  (.1, .1, .1), start, color)
                if geom is not None:
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                         .11, start + (end - start) * lo,
                                         start + (end - start) * hi)
        for node in nodes.values():
            ref = node.get("formation_reference")
            if ref is None or "qpos" not in node:
                continue
            start, end = np.asarray(node["qpos"][:3]), np.asarray(ref)
            if np.linalg.norm(end - start) < .1:
                continue
            geom = self._geom(scene, mujoco.mjtGeom.mjGEOM_CAPSULE,
                              (.05, .05, .05), start, (.5, .55, .68, .55))
            if geom is not None:
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                     .045, start, end)
        # Packet highlights take priority over the optional coverage raster.
        # Only observed ns-3 reception/expiry events reach this stream.
        now = self.latest.get("t", 0.) if render_t is None else render_t
        for event in self.latest.get("packet_flow", [])[-32:]:
            age = now - event["at"]
            if event["type"] == "DROP":
                if not 0 <= age <= .65:
                    continue
                position = event.get("position")
                color = (1., .16, .12, .8 if int(age * 12) % 2 == 0 else .2)
            else:
                if not 0 <= age <= .8:
                    continue
                start, end = event.get("start"), event.get("end")
                if start is None or end is None:
                    continue
                fraction = max(0., min(1., age / .8))
                position = [start[i] + (end[i] - start[i]) * fraction for i in range(3)]
                color = ((.05, .94, 1., .95) if event["type"] == "DOWNLINK"
                         else (.95, .42, 1., .95))
            if position is not None and scene.ngeom < min(scene.maxgeom, 256):
                self._geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                           (.55, .55, .55), position, color)
        for a, b in self.latest.get("formation_skeleton", []):
            if a not in nodes or b not in nodes:
                continue
            start = np.asarray(nodes[a].get("formation_reference", nodes[a]["qpos"][:3]))
            end = np.asarray(nodes[b].get("formation_reference", nodes[b]["qpos"][:3]))
            for lo, hi in ((0., .2), (.4, .6), (.8, 1.)):
                geom = self._geom(scene, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                  (.1, .1, .1), start, (.65, .7, .76, .35))
                if geom is not None:
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                         .07, start + (end - start) * lo,
                                         start + (end - start) * hi)
        coverage = self.latest.get("coverage") if show_coverage else None
        if coverage:
            xmin, ymin, _, _ = coverage["bounds"]
            for col, row, grade in coverage["cells"]:
                if scene.ngeom >= min(scene.maxgeom, 192):
                    break
                x = xmin + (col + .5) * coverage["dx"]
                y = ymin + (row + .5) * coverage["dy"]
                color = {"strong": (.15, .85, .45, .16),
                         "shadow": (.6, .11, .12, .24),
                         "weak": (.28, .28, .32, .12)}[grade]
                self._geom(scene, mujoco.mjtGeom.mjGEOM_BOX,
                           (coverage["dx"]*.48, coverage["dy"]*.48, .07),
                           (x, y, coverage["origin"][2] - 1.), color)


class LiveCamera:
    """Close-up camera framing and optional follow of the selected drone."""

    def __init__(self, camera):
        self.camera = camera
        self.following = False

    def start(self, position):
        self.camera.lookat[:] = position
        self.camera.distance = 25.
        self.camera.elevation = -25.

    def update(self, keys, selected, positions):
        if keys.recenter:
            keys.recenter = False
            self.following = True
            self.camera.distance = 25.
            self.camera.elevation = -25.
        if self.following and len(selected) == 1 and selected[0] in positions:
            self.camera.lookat[:] = positions[selected[0]]
        elif self.following and len(selected) != 1:
            self.following = False


class LiveKeys:
    """Passive-viewer keyboard events, processed on the owning Python thread."""

    def __init__(self, controller, ids):
        import queue
        self.controller = controller
        self.ids = tuple(ids)
        self.events = queue.SimpleQueue()
        self.preview = None
        self.recenter = False
        self.show_coverage = False

    def __call__(self, keycode):
        # MuJoCo only forwards keycode. Capture modifier *at callback time*;
        # querying it later on the render loop may miss a released Shift key.
        import glfw
        window = glfw.get_current_context()
        shift = bool(window and (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                                 glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS))
        self.events.put((keycode, shift))

    def drain(self, focus, positions):
        events = []
        while not self.events.empty():
            events.append(self.events.get_nowait())
        if events:
            self.process(events, focus, positions)

    def process(self, events, focus, positions):
        for event in events:
            key, shift = event if isinstance(event, tuple) else (event, False)
            if key in (48, 65):  # 0 / A
                self.controller.set_selection(list(self.ids))
            elif key == 49:  # 1
                self.controller.set_selection([n for n in self.ids if n == 'L'])
            elif key == 50:  # 2
                self.controller.set_selection([n for n in self.ids if n.startswith('R')])
            elif key == 51:  # 3
                self.controller.set_selection([n for n in self.ids if n.startswith('S')])
            elif key == 258:  # Tab; viewer forwards keycodes, not modifiers
                current = self.controller.selected()
                index = self.ids.index(current[0]) if len(current) == 1 else -1
                direction = -1 if shift else 1
                self.controller.set_selection([self.ids[(index + direction) % len(self.ids)]])
            elif key == 86:  # V: toggle the read-only modeled LoS coverage layer
                self.show_coverage = not self.show_coverage
            elif key == 67:  # C: recenter and follow a single selected drone
                self.recenter = True
            elif key == 70:  # F: camera focus, altitude of selected drone
                selected = self.controller.selected()
                if selected:
                    point = positions.get(selected[0])
                    if point is not None:
                        self.preview = (float(focus[0]), float(focus[1]),
                                        float(point[2]))
            elif key in (91, 93) and self.preview is not None:
                delta = -5. if key == 91 else 5.
                self.preview = (self.preview[0], self.preview[1],
                                max(0., self.preview[2] + delta))
            elif key in (257, 335) and self.preview is not None:
                selected = self.controller.selected()
                if selected:
                    self.controller.submit({'nodes': selected,
                                            'destination': list(self.preview),
                                            'group': 'main'})
                    self.preview = None
            elif key == 256:
                self.preview = None


@contextmanager
def native_viewer_session(model, data, **kwargs):
    """Close passive Viewer and await MuJoCo's owned render thread teardown.

    MuJoCo 3.17.0 Handle.close only signals exit; the GLFW/EGL worker may
    still swap buffers when Python's atexit terminates GLFW. Capture the
    launch's native worker, rather than waiting on unrelated threads.
    """
    before = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data, **kwargs)
    workers = [thread for thread in set(threading.enumerate()) - before
               if getattr(thread, 'name', '').endswith('(_launch_internal)')]
    try:
        yield viewer
    finally:
        viewer.close()
        for worker in workers:
            worker.join()


def view_live(controller, *, refresh_s=1/30):
    """Run the native GLFW window on the main thread, consuming live poses."""
    display = LiveSwarmDisplay(sorted(controller.engine.nodes),
                               controller.engine.buildings)
    keys = LiveKeys(controller, display.ids)
    with native_viewer_session(display.model, display.data,
                               key_callback=keys) as viewer:
        camera = LiveCamera(viewer.cam)
        leader = controller.engine.leader_id
        with viewer.lock():
            camera.start(controller.engine.nodes[leader].pos)
            print(f"MuJoCo camera lookat={tuple(viewer.cam.lookat)} "
                  f"distance={viewer.cam.distance:.1f}m "
                  f"elevation={viewer.cam.elevation:.1f}deg", flush=True)
        last = None
        last_selected_body = 0
        last_selection_version = -1
        last_preview = None
        last_coverage = False
        last_wall = time.monotonic()
        logged_pose_range = False
        while viewer.is_running() and not controller.shutdown.is_set():
            with controller.lock:
                state = controller.snapshot
            with viewer.lock():
                selected_body = viewer.perturb.select
                focus = tuple(viewer.cam.lookat)
            if state.get("nodes"):
                if selected_body and selected_body != last_selected_body:
                    node = display.body_to_node(selected_body)
                    if node is not None:
                        controller.set_selection([node])
                last_selected_body = selected_body
                positions = {name: node['qpos'][:3] for name, node
                             in state['nodes'].items() if 'qpos' in node}
                if not logged_pose_range and len(positions) == len(display.ids):
                    bounds = np.asarray(list(positions.values()))
                    print(f"MuJoCo drone qpos bounds min={bounds.min(axis=0)} "
                          f"max={bounds.max(axis=0)}", flush=True)
                    logged_pose_range = True
                keys.drain(focus, positions)
                selected = controller.selected()
                if keys.recenter or camera.following:
                    with viewer.lock():
                        camera.update(keys, selected, positions)
                preview = keys.preview if keys.preview is not None else controller.preview()
                if (state is not last or preview != last_preview or
                        state.get('packet_flow') or
                        keys.show_coverage != last_coverage or
                        controller.selection_version != last_selection_version):
                    if state is not last:
                        last_wall = time.monotonic()
                    with viewer.lock():
                        if state is not last:
                            display.update(state)
                        display.overlay(viewer.user_scn, selected=selected,
                                        preview=preview,
                                        show_coverage=keys.show_coverage,
                                        render_t=min(state.get('t', 0.) + .8,
                                            state.get('t', 0.) + max(0., time.monotonic()-last_wall)))
                    last_coverage = keys.show_coverage
                    last = state
                    last_selection_version = controller.selection_version
                    last_preview = preview
            viewer.sync()
            if controller.finished:
                break
            controller.shutdown.wait(refresh_s)
        controller.stop()
        controller.join()  # No engine/HTTP worker can touch the viewer at close.


class _Keys:
    """键盘状态。

    MuJoCo 的回调只给按键码，不给"当前是否按下"，所以自己维护一个集合。
    """

    def __init__(self) -> None:
        self.down: set[int] = set()
        self.events: list[int] = []

    def __call__(self, keycode: int) -> None:
        self.events.append(keycode)

    #: GLFW 键码（MuJoCo viewer 直接透传）
    SPACE = 32
    R = 82
    UP = 265
    DOWN = 264
    ESC = 256


def view_single(*, duration: float = 1e9, altitude: float = 1.2,
                hold: bool = False) -> None:
    """单机实时查看。方向键上下改目标高度，空格暂停，R 复位。"""
    sim = QuadrotorSim()
    sim.reset(pos=(0.0, 0.0, 0.05), rotor_speed=0.0)
    keys = _Keys()
    target = np.array([0.0, 0.0, altitude])
    paused = False
    sim_t = 0.0

    with mujoco.viewer.launch_passive(sim.model, sim.data,
                                      key_callback=keys) as viewer:
        while viewer.is_running() and sim_t < duration:
            for k in keys.events:
                if k == _Keys.SPACE:
                    paused = not paused
                elif k == _Keys.R:
                    sim.reset(pos=(0.0, 0.0, 0.05), rotor_speed=0.0)
                    sim_t = 0.0
                elif k == _Keys.UP:
                    target[2] += 0.1
                elif k == _Keys.DOWN:
                    target[2] = max(0.15, target[2] - 0.1)
            keys.events.clear()

            if not paused:
                # 按实时节奏跑：每帧推进 dt，然后睡到墙钟对齐
                step_start = time.perf_counter()
                if hold:
                    target[:2] = sim.pos[:2]
                sim.step(target, sim_t)
                sim_t += sim.dt
                viewer.sync()
                slack = sim.dt - (time.perf_counter() - step_start)
                if slack > 0:
                    time.sleep(slack)
            else:
                viewer.sync()
                time.sleep(1.0 / 60.0)
        # 在 close() 之前打印：MuJoCo 的 teardown 可能 core dump，
        # 放在 with 之外会永远执行不到
        print(f"单机：仿真 {sim_t:.2f} s，最终高度 {sim.pos[2]:.3f} m，"
              f"目标 {target[2]:.2f} m，末转速 {np.round(sim.rotor_speeds(), 0)}",
              flush=True)




def view_swarm(*, count: int = 11, duration: float = 1e9) -> None:
    """多机实时查看。编队做呼吸式径向进出。"""
    layout = SwarmLayout(count=count, radius=1.0 if count > 4 else 0.8,
                         altitude=1.2)
    sim = SwarmSim(layout)
    sim.reset()
    home = [sim.pos(i).copy() for i in range(count)]
    keys = _Keys()
    paused = False
    sim_t = 0.0

    def targets(t: float) -> list[np.ndarray]:
        amp = 0.0 if paused else 0.5 * math.sin(2.0 * math.pi * t / 6.0)
        out = []
        for h in home:
            r = math.hypot(h[0], h[1]) or 1.0
            out.append(np.array([h[0] * (1.0 + amp / r),
                                 h[1] * (1.0 + amp / r), 1.2]))
        return out

    with mujoco.viewer.launch_passive(sim.model, sim.data,
                                      key_callback=keys) as viewer:
        viewer.cam.distance = max(3.5, 1.6 * count ** 0.5)
        viewer.cam.elevation = -30.0
        viewer.cam.lookat[:] = np.array([0.0, 0.0, 1.15])
        while viewer.is_running() and sim_t < duration:
            for k in keys.events:
                if k == _Keys.SPACE:
                    paused = not paused
                elif k == _Keys.R:
                    sim.reset()
                    sim_t = 0.0
            keys.events.clear()

            step_start = time.perf_counter()
            sim.step(targets(sim_t), sim_t)
            sim_t += sim.dt
            viewer.sync()
            slack = sim.dt - (time.perf_counter() - step_start)
            if slack > 0:
                time.sleep(slack)
        zs = [sim.pos(i)[2] for i in range(count)]
        print(f"编队：仿真 {sim_t:.2f} s，{count} 机高度 "
              f"{min(zs):.3f}-{max(zs):.3f} m，末转速 "
              f"{np.round(sim.rotor_speeds(0), 0)}", flush=True)


