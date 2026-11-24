"""命令行入口。

    python3 -m skyfleet.cli run     [--config C] [--seed N] [--duration S] [--log P]
    python3 -m skyfleet.cli metrics --log P
    python3 -m skyfleet.cli plot    --log P [--out DIR]
    python3 -m skyfleet.cli replay  --log P [--out FILE]
    python3 -m skyfleet.cli replay3d --log P [--out FILE]
    python3 -m skyfleet.cli all     [--config C] [--out DIR]

所有子命令都返回退出码而不抛异常，便于脚本与 CI 使用。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from skyfleet.kernel.config import SimConfig, apply_overrides, baseline_path

#: 允许被"翻译成人话"的可选依赖。仿真核心零依赖，只有可视化需要它们。
OPTIONAL_DEPENDENCIES = frozenset({"matplotlib"})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="skyfleet", description="空中主从无人机集群配送通信仿真")
    sub = parser.add_subparsers(dest="command")

    run = sub.add_parser("run", help="运行仿真并写事件日志")
    run.add_argument("--config", default=baseline_path())
    run.add_argument("--seed", type=int, default=None)
    run.add_argument("--duration", type=float, default=None)
    run.add_argument("--log", default=None)

    metrics = sub.add_parser("metrics", help="从日志重算并打印评价指标")
    metrics.add_argument("--log", required=True)

    plot = sub.add_parser("plot", help="从日志生成图表 PNG")
    plot.add_argument("--log", required=True)
    plot.add_argument("--out", default="out/figures")

    replay = sub.add_parser("replay", help="从日志生成自包含 HTML 回放")
    replay.add_argument("--log", required=True)
    replay.add_argument("--out", default="out/replay/replay.html")

    replay3d = sub.add_parser("replay3d", help="从日志生成离线 WebGL2 三维城市回放")
    replay3d.add_argument("--log", required=True)
    replay3d.add_argument("--out", default="out/replay/replay3d.html")

    dispatch = sub.add_parser("dispatch", help="向多架无人机发送仿真航点并导出回放")
    dispatch.add_argument("--config", default=baseline_path())
    dispatch.add_argument("--nodes", required=True,
                          help="逗号分隔的无人机 ID，例如 S1,S2,S3")
    dispatch.add_argument("--destination", required=True,
                          help="目标 X,Y,Z（米）")
    dispatch.add_argument("--at", type=float, default=2.0,
                          help="仿真下令时刻，秒")
    dispatch.add_argument("--group", default="east")
    dispatch.add_argument("--through", default=None,
                          help="可選窄通道中心 X,Y,Z；規劃排隊通道航線並恢復矩陣")
    dispatch.add_argument("--adaptive", action="store_true",
                          help="運行中每次運動週期重估障礙與同組隊形")
    dispatch.add_argument("--city", default=None,
                          help="覆蓋設定中的 GeoJSON 城市樓群")
    dispatch.add_argument("--out", default="out/dispatch")

    live = sub.add_parser("live", help="運行中選機派遣並觀察通信拓撲")
    live.add_argument("--config", default=baseline_path())
    live.add_argument("--city", default=None)
    live.add_argument("--duration", type=float, default=None)
    live.add_argument("--port", type=int, default=8765)
    live.add_argument("--tick-seconds", type=float, default=0.1)
    live.add_argument("--viewer", choices=("web", "mujoco"), default="web",
                      help="mujoco: native live 3D window; web: 2D status page")
    live.add_argument("--log", default="out/live/dispatch.jsonl")

    everything = sub.add_parser("all", help="运行 + 指标 + 图表 + 回放")
    everything.add_argument("--config", default=baseline_path())
    everything.add_argument("--seed", type=int, default=None)
    everything.add_argument("--duration", type=float, default=None)
    everything.add_argument("--out", default="out")

    study = sub.add_parser("study", help="P1 参数标定：按规格扫参并汇总")
    study.add_argument("--spec", required=True, help="扫参规格 JSON")
    study.add_argument("--out", default=None,
                       help="输出目录，默认 out/study/<规格名>")
    study.add_argument("--jobs", type=int, default=1,
                       help="并发进程数；结果与并发度无关")
    study.add_argument("--no-logs", action="store_true",
                       help="不保留每轮的原始日志（只留汇总）")
    study.add_argument("--quiet", action="store_true")

    return parser


def _load(args) -> SimConfig:
    config = apply_overrides(load_config(args.config), seed=args.seed,
                             duration=args.duration)
    if getattr(args, "log", None):
        config.log_path = args.log
    return config


def load_config(path: str) -> SimConfig:
    from skyfleet.kernel.config import load_config as _load_config
    return _load_config(path)


def _cmd_run(args) -> int:
    from skyfleet.sim.engine import Engine

    config = _load(args)
    path = Engine(config).run()
    print(f"仿真完成：{config.duration:g} s（种子 {config.seed}）")
    print(f"事件日志：{path}")
    return 0


def _cmd_dispatch(args) -> int:
    from dataclasses import replace
    from skyfleet.sim.engine import Engine
    from skyfleet.viz.replay_3d import render_replay_3d

    selected = [node.strip() for node in args.nodes.split(",") if node.strip()]
    if not selected or len(selected) != len(set(selected)):
        raise ValueError("--nodes 需要非空且不重複的節點 ID")
    try:
        destination = tuple(float(value.strip())
                            for value in args.destination.split(","))
    except ValueError as exc:
        raise ValueError("--destination 格式需為 X,Y,Z") from exc
    if len(destination) != 3 or not all(map(math.isfinite, destination)):
        raise ValueError("--destination 格式需為有限數值 X,Y,Z")
    if args.at < 0:
        raise ValueError("--at 必須大於等於零")
    if not args.group.strip():
        raise ValueError("--group 不可為空")

    base = _load(argparse.Namespace(config=args.config, seed=None,
                                    duration=None, log=None))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    log_path = out / "dispatch.jsonl"
    config = replace(base, tasks=0, log_path=str(log_path),
                     city_geojson=args.city or base.city_geojson,
                     replay_sample_period=min(base.motion_period, 0.2))
    engine = Engine(config).build()
    unknown = set(selected) - (set(engine.agents) - {engine.leader_id})
    if unknown:
        raise ValueError(f"未知或非從機節點：{', '.join(sorted(unknown))}")
    route = None
    if args.through:
        try:
            passage = tuple(float(value.strip())
                            for value in args.through.split(","))
        except ValueError as exc:
            raise ValueError("--through 格式需為 X,Y,Z") from exc
        if len(passage) != 3 or not all(map(math.isfinite, passage)):
            raise ValueError("--through 格式需為有限數值 X,Y,Z")
        from skyfleet.world.geometry import plan_passage
        routes = plan_passage(destination, selected, engine.buildings,
                              spacing=3.0, clearance=0.4, drone_radius=0.3,
                              through=passage)
        # The corridor waypoints are precomputed separately; individual delivery
        # preserves sequential entry rather than creating a radio burst.
        for index, node_id in enumerate(sorted(selected)):
            route = routes[node_id]
            engine.schedule_dispatch(args.at + index * 1.0, [node_id],
                                     route[-1], args.group.strip(), route=route,
                                     adaptive=args.adaptive)
    else:
        engine.schedule_dispatch(args.at, selected, destination,
                                 args.group.strip(), adaptive=args.adaptive)
    engine.run()
    replay = render_replay_3d(log_path, out / "dispatch_3d.html")
    print(f"仿真日誌：{log_path}")
    print(f"三維回放：{replay}")
    return 0


def _cmd_live(args) -> int:
    from dataclasses import replace
    from skyfleet.sim.live_control import serve_live

    if not 0 <= args.port <= 65535:
        raise ValueError("--port must be 0..65535")
    if not math.isfinite(args.tick_seconds) or args.tick_seconds < 0:
        raise ValueError("--tick-seconds must be nonnegative and finite")
    base = load_config(args.config)
    if args.duration is not None and (not math.isfinite(args.duration) or
                                       args.duration <= 0):
        raise ValueError("--duration must be positive and finite")
    config = replace(base, log_path=args.log,
                     city_geojson=args.city or base.city_geojson,
                     duration=args.duration or base.duration)
    serve_live(config, port=args.port, tick_seconds=args.tick_seconds,
               viewer=args.viewer)
    return 0


def _cmd_metrics(args) -> int:
    from skyfleet.observe.metrics import compute_metrics, format_report

    report = format_report(compute_metrics(args.log))
    print(report)
    out = Path(args.log).with_suffix(".metrics.txt")
    out.write_text(report + "\n", encoding="utf-8")
    print(f"\n指标已写入：{out}")
    return 0


def _cmd_plot(args) -> int:
    from skyfleet.viz.plots import render_all

    written = render_all(args.log, args.out)
    for path in written:
        print(f"图表：{path}")
    return 0


def _cmd_replay(args) -> int:
    from skyfleet.viz.replay_html import render_replay

    path = render_replay(args.log, args.out)
    print(f"回放页面：{path}")
    return 0


def _cmd_replay3d(args) -> int:
    from skyfleet.viz.replay_3d import render_replay_3d

    path = render_replay_3d(args.log, args.out)
    print(f"三维回放页面：{path}")
    return 0


def _cmd_all(args) -> int:
    from skyfleet.sim.engine import Engine

    config = _load(argparse.Namespace(config=args.config, seed=args.seed,
                                      duration=args.duration, log=None))
    out = Path(args.out)
    # 产物按**场景名**分目录。原先硬编码成 baseline，三个场景跑下来会互相
    # 覆盖：读 out/logs/baseline.jsonl 拿到的其实是最后一个场景的结果。
    name = Path(config.log_path).stem
    config.log_path = str(out / "logs" / f"{name}.jsonl")
    log_path = Engine(config).run()
    print(f"仿真完成：{config.duration:g} s（种子 {config.seed}）")

    from skyfleet.observe.metrics import compute_metrics, format_report
    report = format_report(compute_metrics(log_path))
    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "logs" / f"{name}.metrics.txt").write_text(report + "\n",
                                                     encoding="utf-8")
    print(report)

    from skyfleet.viz.plots import render_all
    for path in render_all(log_path, str(out / "figures" / name)):
        print(f"图表：{path}")

    from skyfleet.viz.replay_html import render_replay
    print(f"回放页面：{render_replay(log_path, str(out / 'replay' / f'{name}.html'))}")
    from skyfleet.viz.replay_3d import render_replay_3d
    print(f"三维回放页面：{render_replay_3d(log_path, str(out / 'replay' / f'{name}_3d.html'))}")
    return 0


def _cmd_study(args) -> int:
    from skyfleet.study.runner import run_sweep
    from skyfleet.study.spec import SweepSpec

    spec = SweepSpec.from_json(args.spec)
    out = Path(args.out) if args.out else Path("out") / "study" / spec.name

    print(f"扫参：{spec.name}（{spec.run_count()} 次运行，"
          f"{spec.repeats} 次重复/组合，{args.jobs} 进程）")
    result = run_sweep(spec, out, jobs=args.jobs,
                       keep_logs=not args.no_logs, verbose=not args.quiet)

    # 规格先存档：表格必须能对应到一份确切的规格，否则数字只是截图
    spec_path = result.write_spec(out / "spec.json")
    csv_path = result.write_csv(out / "runs.csv")
    md_path = result.write_markdown(out / "summary.md")

    print(f"规格：{spec_path}")
    print(f"逐次结果：{csv_path}")
    print(f"汇总：{md_path}")
    return 0


COMMANDS = {
    "run": _cmd_run,
    "metrics": _cmd_metrics,
    "dispatch": _cmd_dispatch,
    "live": _cmd_live,
    "plot": _cmd_plot,
    "replay": _cmd_replay,
    "replay3d": _cmd_replay3d,
    "all": _cmd_all,
    "study": _cmd_study,
}


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:                 # argparse 的用法错误
        return int(exc.code or 2)

    if args.command is None:
        parser.print_help()
        return 2

    try:
        return COMMANDS[args.command](args)
    except FileNotFoundError as exc:
        print(f"错误：找不到文件 {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    except ImportError as exc:
        # 只把**可选依赖**缺失翻译成人话。曾经这里无条件捕获 ImportError，
        # 结果代码里真正的 import 错误被报成"缺少 skyfleet.world.geometry"，
        # 排查方向被彻底带偏。真实错误必须原样抛出。
        root = (exc.name or "").split(".")[0]
        if root not in OPTIONAL_DEPENDENCIES:
            raise
        # 措辞必须与事实一致：只有**绘图**需要 matplotlib。回放 HTML 是
        # 纯 json + pathlib 生成的（`viz/replay_html.py` 不 import 它），
        # 而旧消息说「画图与回放需要 matplotlib」会让缺依赖的用户以为
        # 回放也用不了，从而放弃本可正常使用的功能。
        # 实测：强制封锁 matplotlib 后回放仍能正常产出 HTML。
        print(f"错误：缺少可选依赖 {exc.name}（当前命令：{args.command}）。\n"
              f"      只有**绘图**需要 matplotlib；仿真、指标与回放 HTML "
              f"不依赖它。\n"
              f"      如需绘图请安装：python3 -m pip install matplotlib",
              file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
