"""flightlab 命令行入口。

    python -m flightlab view            开实时窗口，看它飞（鼠标可转视角）
    python -m flightlab demo            跑五航点任务并打印指标
    python -m flightlab render -o x.mp4 渲染 3D 视频
    python -m flightlab stills -o DIR   渲染报告插图（静态帧）
    python -m flightlab tune            跑增益扫描，输出对比表

**为什么要有 CLI**：库函数好用与否，取决于别人能不能在不知道内部结构的情况下
跑起来。一条命令能跑完，才算交付；否则每次演示都要现场写脚本。
"""
from __future__ import annotations

import argparse
import os
import sys

# 离屏渲染后端必须在 import mujoco 之前确定
os.environ.setdefault("MUJOCO_GL", "glfw")

import numpy as np                                    # noqa: E402

from flightlab.params import DEFAULT as PARAMS        # noqa: E402
from flightlab.sim import QuadrotorSim                # noqa: E402

#: 默认演示航线：起飞、绕正方形一圈、回到起点。闭合航线能同时暴露
#: 起飞瞬态、连续转弯、以及"回到起点是否还有累积误差"。
DEFAULT_WAYPOINTS = [(0.0, 0.0, 1.5), (2.0, 0.0, 1.5), (2.0, 2.0, 2.0),
                     (0.0, 2.0, 2.0), (0.0, 0.0, 1.5)]


def _cmd_demo(args: argparse.Namespace) -> int:
    seg = args.duration / len(DEFAULT_WAYPOINTS)

    def target(t: float) -> np.ndarray:
        return np.array(DEFAULT_WAYPOINTS[min(int(t / seg), len(DEFAULT_WAYPOINTS) - 1)])

    sim = QuadrotorSim()
    sim.reset(pos=(0.0, 0.0, 0.05), rotor_speed=0.0)
    log = sim.run(args.duration, target)
    pos = log.positions()
    times = log.times()

    print(f"仿真 {args.duration:.0f} s（dt={sim.dt*1000:.0f} ms，"
          f"{len(log.steps)} 个采样点）")
    print(f"机型：质量 {PARAMS.mass*1000:.1f} g，臂长 {PARAMS.arm_length*1000:.0f} mm，"
          f"推重比 {PARAMS.thrust_to_weight:.2f}")
    print()
    print(f"{'时刻':>7} {'目标':>20} {'实际':>26} {'误差m':>8} {'倾角°':>7}")
    worst = 0.0
    for i, wp in enumerate(DEFAULT_WAYPOINTS):
        k = int(np.argmin(np.abs(times - (i * seg + seg * 0.95))))
        p = pos[k]
        st = log.steps[k]
        err = float(np.linalg.norm(p - np.array(wp)))
        worst = max(worst, err)
        tilt = float(np.degrees(np.arccos(np.clip(st.R[2, 2], -1, 1))))
        print(f"{times[k]:7.2f} ({wp[0]:4.1f},{wp[1]:4.1f},{wp[2]:4.1f})  "
              f"({p[0]:7.3f},{p[1]:7.3f},{p[2]:7.3f})  {err:8.4f} {tilt:7.2f}")

    print()
    print(f"最大跟踪误差      : {worst:.4f} m")
    print(f"轨迹长度          : "
          f"{float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))):.2f} m")
    print(f"最大角速度        : "
          f"{max(float(np.linalg.norm(s.omega)) for s in log.steps):.2f} rad/s")
    print(f"最终转子转速      : {np.round(sim.rotor_speeds(), 0)} rad/s"
          f"（悬停理论值 {PARAMS.omega_hover:.0f}）")
    print(f"最终偏航角速度    : {sim.omega[2]:+.6f} rad/s")
    return 0


def _cmd_render(args: argparse.Namespace) -> int:
    if args.swarm:
        from flightlab.render import SwarmRenderConfig, render_swarm

        cfg = SwarmRenderConfig(width=args.width, height=args.height,
                                fps=args.fps)
        path = render_swarm(args.output, count=args.swarm,
                            duration=args.duration, cfg=cfg,
                            progress=not args.quiet)
        print(f"多机视频已写出：{path}")
        return 0

    from flightlab.render import RenderConfig, render_mission

    cfg = RenderConfig(width=args.width, height=args.height, fps=args.fps)
    path = render_mission(args.output, duration=args.duration,
                          waypoints=DEFAULT_WAYPOINTS, cfg=cfg,
                          progress=not args.quiet)
    print(f"视频已写出：{path}")
    return 0


def _cmd_stills(args: argparse.Namespace) -> int:
    from flightlab.render import RenderConfig, render_stills

    cfg = RenderConfig(width=args.width, height=args.height)
    paths = render_stills(args.output, times=tuple(args.times),
                          waypoints=DEFAULT_WAYPOINTS, cfg=cfg)
    for p in paths:
        print(f"已写出：{p}")
    return 0


def _cmd_swarm(args: argparse.Namespace) -> int:
    import math

    from flightlab.swarm import SwarmLayout, SwarmSim

    layout = SwarmLayout(count=args.count,
                         radius=1.0 if args.count > 4 else 0.8, altitude=1.2)
    sim = SwarmSim(layout)
    sim.reset()
    home = [sim.pos(i).copy() for i in range(args.count)]

    def targets(t: float):
        k = 0.5 * math.sin(2 * math.pi * t / 6.0)
        out = []
        for h in home:
            r = math.hypot(h[0], h[1]) or 1.0
            out.append(np.array([h[0] * (1 + k / r),
                                 h[1] * (1 + k / r), 1.2]))
        return out

    log = sim.run(args.duration, targets)
    pos = [log.positions(i) for i in range(args.count)]
    print(f"{args.count} 机编队，仿真 {args.duration:.0f} s")
    print(f"场景规模：{sim.model.nbody} bodies / {sim.model.njnt} joints / "
          f"{sim.model.nu} actuators")
    print()
    print(f"{'机号':>4} {'末位置':>28} {'高度误差mm':>12} {'末转速':>8}")
    worst_z = 0.0
    for i in range(args.count):
        p = pos[i][-1]
        dz = abs(p[2] - 1.2) * 1000
        worst_z = max(worst_z, dz)
        print(f"{i:>4} ({p[0]:7.3f},{p[1]:7.3f},{p[2]:7.3f}) "
              f"{dz:12.1f} {sim.rotor_speeds(i)[0]:8.0f}")
    sep = min(float(np.linalg.norm(pos[i][-1] - pos[j][-1]))
              for i in range(args.count) for j in range(i + 1, args.count))
    print()
    print(f"最大高度误差      : {worst_z:.1f} mm")
    print(f"最小机间距        : {sep:.3f} m")
    print(f"悬停转速理论值    : {PARAMS.omega_hover:.0f} rad/s")
    return 0


def _cmd_view(args: argparse.Namespace) -> int:
    from flightlab.viewer import view_single, view_swarm

    if args.swarm:
        print(f"打开 {args.swarm} 机实时窗口：鼠标拖动转视角，空格暂停，R 复位，"
              f"关窗口退出")
        view_swarm(count=args.swarm, duration=args.duration)
    else:
        print("打开单机实时窗口：↑↓ 改目标高度，空格暂停，R 复位，关窗口退出")
        view_single(duration=args.duration, altitude=args.altitude)
    return 0


def _cmd_tune(args: argparse.Namespace) -> int:
    from flightlab.tuning import run_default_study

    print(run_default_study(axis=args.axis))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m flightlab",
        description="四旋翼 3D 仿真（MuJoCo）：演示、渲染、调参")
    sub = p.add_subparsers(dest="command", required=True)

    d = sub.add_parser("demo", help="跑五航点任务并打印跟踪指标")
    d.add_argument("--duration", type=float, default=30.0, help="总时长 s")
    d.set_defaults(func=_cmd_demo)

    v = sub.add_parser("view", help="交互式实时窗口（需要图形界面）")
    v.add_argument("--swarm", type=int, default=0, metavar="N",
                   help="查看 N 机编队而不是单机")
    v.add_argument("--duration", type=float, default=1e9,
                   help="最长运行秒数（默认一直跑）")
    v.add_argument("--altitude", type=float, default=1.2)
    v.set_defaults(func=_cmd_view)

    w = sub.add_parser("swarm", help="多机编队飞行并打印指标")
    w.add_argument("-n", "--count", type=int, default=11, help="机数")
    w.add_argument("--duration", type=float, default=10.0)
    w.set_defaults(func=_cmd_swarm)

    r = sub.add_parser("render", help="渲染 3D 视频")
    r.add_argument("-o", "--output", default="flight.mp4")
    r.add_argument("--duration", type=float, default=18.0)
    r.add_argument("--width", type=int, default=1280)
    r.add_argument("--height", type=int, default=720)
    r.add_argument("--fps", type=int, default=30)
    r.add_argument("--quiet", action="store_true")
    r.add_argument("--swarm", type=int, default=0, metavar="N",
                   help="渲染 N 机编队而不是单机")
    r.set_defaults(func=_cmd_render)

    s = sub.add_parser("stills", help="渲染静态帧（报告插图）")
    s.add_argument("-o", "--output", default="figures")
    s.add_argument("--times", type=float, nargs="+",
                   default=[0.0, 2.0, 6.0, 10.0, 14.0])
    s.add_argument("--width", type=int, default=1280)
    s.add_argument("--height", type=int, default=720)
    s.set_defaults(func=_cmd_stills)

    t = sub.add_parser("tune", help="增益扫描，输出对比表")
    t.add_argument("--axis", choices=("x", "z"), default="x",
                   help="阶跃方向（默认 x：水平机动才暴露姿态环）")
    t.set_defaults(func=_cmd_tune)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
