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
    config.log_path = str(out / "logs" / "baseline.jsonl")
    log_path = Engine(config).run()
    print(f"仿真完成：{config.duration:g} s（种子 {config.seed}）")

    from skyfleet.observe.metrics import compute_metrics, format_report
    report = format_report(compute_metrics(log_path))
    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "logs" / "baseline.metrics.txt").write_text(report + "\n",
                                                       encoding="utf-8")
    print(report)

    from skyfleet.viz.plots import render_all
    for path in render_all(log_path, str(out / "figures")):
        print(f"图表：{path}")

    from skyfleet.viz.replay_html import render_replay
    print(f"回放页面：{render_replay(log_path, str(out / 'replay' / 'replay.html'))}")
    return 0


COMMANDS = {
    "run": _cmd_run,
    "metrics": _cmd_metrics,
    "plot": _cmd_plot,
    "replay": _cmd_replay,
    "all": _cmd_all,
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


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
