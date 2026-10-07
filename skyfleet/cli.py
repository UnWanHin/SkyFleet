"""命令行入口。

    python3 -m skyfleet.cli run     [--config C] [--seed N] [--duration S] [--log P]
    python3 -m skyfleet.cli metrics --log P
    python3 -m skyfleet.cli plot    --log P [--out DIR]
    python3 -m skyfleet.cli replay  --log P [--out FILE]
    python3 -m skyfleet.cli all     [--config C] [--out DIR]

所有子命令都返回退出码而不抛异常，便于脚本与 CI 使用。
"""
from __future__ import annotations

import argparse
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
    "plot": _cmd_plot,
    "replay": _cmd_replay,
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
