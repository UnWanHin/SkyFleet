"""把四旋翼仿真渲染成 3D 视频。

**为什么要真的渲染**：控制曲线能证明"数值收敛"，但证明不了"看起来像在飞"。
姿态、倾斜、旋翼转速这些量必须眼见为实，否则"能飞"只是一句话。

实现要点：
* 用 MuJoCo 的离屏渲染（`MUJOCO_GL=glfw`），不依赖窗口系统；
* 相机跟随飞行器但**不跟随姿态**——跟随姿态会让画面里的飞行器永远水平，
  看不出倾斜，等于把最该展示的东西藏起来了；
* 逐帧通过管道喂给系统 ffmpeg，避免把整段视频攒在内存里。
"""
from __future__ import annotations

import math
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from flightlab.params import DEFAULT
from flightlab.sim import QuadrotorSim

#: 离屏渲染后端必须在 import mujoco 之前选定。GLFW 在无窗口环境下也能工作，
#: 但不适合多进程并发；单进程渲染足够。
os.environ.setdefault("MUJOCO_GL", "glfw")


@dataclass(frozen=True)
class RenderConfig:
    width: int = 1280
    height: int = 720
    fps: int = 30
    #: 相机与飞行器的距离 m
    distance: float = 1.4
    elevation_deg: float = -22.0
    azimuth_deg: float = 130.0


def _ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


class FlightRenderer:
    """逐帧渲染四旋翼飞行并写成 MP4。"""

    def __init__(self, sim: QuadrotorSim, cfg: RenderConfig | None = None) -> None:
        self.sim = sim
        self.cfg = cfg or RenderConfig()
        self.renderer = mujoco.Renderer(sim.model, self.cfg.height,
                                        self.cfg.width)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.distance = self.cfg.distance
        self.camera.elevation = self.cfg.elevation_deg
        self.camera.azimuth = self.cfg.azimuth_deg

    def _aim_camera(self) -> None:
        """相机看向飞行器当前位置，保持固定朝向角。"""
        self.camera.lookat[:] = self.sim.pos

    def render_frame(self) -> np.ndarray:
        self._aim_camera()
        mujoco.mj_forward(self.sim.model, self.sim.data)
        self.renderer.update_scene(self.sim.data, camera=self.camera)
        return self.renderer.render()

    def close(self) -> None:
        self.renderer.close()


def render_mission(out_path: str | Path, *, duration: float = 18.0,
                   waypoints: list[tuple[float, float, float]] | None = None,
                   cfg: RenderConfig | None = None,
                   sim: QuadrotorSim | None = None,
                   progress: bool = True) -> Path:
    """跑一段任务并渲染成 MP4。返回输出路径。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not _ffmpeg_available():
        raise RuntimeError("需要系统 ffmpeg 才能输出视频")

    sim = sim or QuadrotorSim()
    sim.reset(pos=(0.0, 0.0, 0.05), rotor_speed=0.0)
    cfg = cfg or RenderConfig()

    wps = waypoints or [(0.0, 0.0, 1.5), (2.0, 0.0, 1.5), (2.0, 2.0, 2.0),
                        (0.0, 2.0, 2.0), (0.0, 0.0, 1.5)]
    seg = duration / len(wps)

    def target(t: float) -> np.ndarray:
        return np.array(wps[min(int(t / seg), len(wps) - 1)])

    renderer = FlightRenderer(sim, cfg)
    # MuJoCo 物理步长 2 ms，视频 30 fps -> 每帧推进约 16.7 步
    steps_per_frame = max(1, int(round(1.0 / (cfg.fps * sim.dt))))
    total_frames = int(duration * cfg.fps)

    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{cfg.width}x{cfg.height}", "-r", str(cfg.fps),
        "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "medium",
        "-crf", "20", "-pix_fmt", "yuv420p",
        str(out_path),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    t = 0.0
    try:
        for i in range(total_frames):
            for _ in range(steps_per_frame):
                sim.step(target(t), t)
                t += sim.dt
            frame = renderer.render_frame()
            proc.stdin.write(frame.tobytes())
            if progress and i % cfg.fps == 0:
                print(f"  渲染 {i}/{total_frames} 帧  t={t:5.1f}s  "
                      f"高度 {sim.pos[2]:5.2f} m", flush=True)
    finally:
        if proc.stdin:
            proc.stdin.close()
        proc.wait()
        renderer.close()

    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 退出码 {proc.returncode}")
    return out_path


def render_stills(out_dir: str | Path, *, times: tuple[float, ...] = (0.0, 2.0, 6.0, 12.0),
                  waypoints: list[tuple[float, float, float]] | None = None,
                  cfg: RenderConfig | None = None) -> list[Path]:
    """在若干时刻各截一张图，用于报告插图（比视频更好引用）。"""
    from PIL import Image

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = cfg or RenderConfig()
    sim = QuadrotorSim()
    sim.reset(pos=(0.0, 0.0, 0.05), rotor_speed=0.0)
    wps = waypoints or [(0.0, 0.0, 1.5), (2.0, 0.0, 1.5), (2.0, 2.0, 2.0),
                        (0.0, 2.0, 2.0), (0.0, 0.0, 1.5)]
    seg = 18.0 / len(wps)

    def target(t: float) -> np.ndarray:
        return np.array(wps[min(int(t / seg), len(wps) - 1)])

    renderer = FlightRenderer(sim, cfg)
    written: list[Path] = []
    t = 0.0
    try:
        for wanted in sorted(times):
            while t < wanted - 1e-9:
                sim.step(target(t), t)
                t += sim.dt
            frame = renderer.render_frame()
            path = out_dir / f"flight_t{wanted:05.1f}s.png"
            Image.fromarray(frame).save(path)
            written.append(path)
    finally:
        renderer.close()
    return written
