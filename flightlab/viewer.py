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

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco                                     # noqa: E402
import mujoco.viewer                              # noqa: E402
import numpy as np                                # noqa: E402

from flightlab.params import DEFAULT as PARAMS     # noqa: E402
from flightlab.sim import QuadrotorSim             # noqa: E402
from flightlab.swarm import SwarmLayout, SwarmSim  # noqa: E402


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


