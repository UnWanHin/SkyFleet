"""可选依赖的边界：只有**绘图**需要 matplotlib。

为什么值得单独一个文件：README 与 CLI 都曾声称「画图与回放需要
matplotlib」，而回放 HTML 是纯 `json` + `pathlib` 生成的
（`viz/replay_html.py` 不 import 它）。这个说法会让缺依赖的用户以为
回放也用不了，从而放弃本可正常使用的功能。

更重要的是架构约束：「仿真核心不依赖绘图库」是本项目的一条设计承诺
——导出图要能在一台没装 matplotlib 的机器上跑完整轮实验。口头承诺
会漂，所以在这里用**强制封锁 import** 的方式把它钉死。
"""

from __future__ import annotations

import builtins
import json
import sys
import tempfile
import unittest
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path

from skyfleet.cli import OPTIONAL_DEPENDENCIES
from skyfleet.kernel.config import load_config
from skyfleet.observe.metrics import compute_metrics
from skyfleet.sim.engine import Engine

SCENARIO = Path("skyfleet/scenarios/baseline_small.json")


@contextmanager
def matplotlib_forbidden():
    """让 `import matplotlib` 抛 ImportError，并断言**确实被调用过**。"""
    real_import = builtins.__import__
    attempts: list[str] = []

    def guarded(name, *args, **kwargs):
        if name.split(".")[0] == "matplotlib":
            attempts.append(name)
            raise ImportError(f"blocked: {name}")
        return real_import(name, *args, **kwargs)

    saved = {k: v for k, v in sys.modules.items()
             if k.split(".")[0] == "matplotlib"}
    for key in saved:
        del sys.modules[key]
    builtins.__import__ = guarded
    try:
        yield attempts
    finally:
        builtins.__import__ = real_import


class TestReplayDoesNotNeedMatplotlib(unittest.TestCase):
    def test_replay_html_renders_with_matplotlib_blocked(self):
        """★ 回放 HTML 必须在**没有** matplotlib 的机器上也能产出。

        这是「仿真核心不依赖绘图库」承诺的一部分：导出的回放要能在
        一台纯仿真机上生成。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "r.jsonl"), duration=30.0)
            Engine(cfg).run()
            out = Path(tmp) / "replay.html"
            with matplotlib_forbidden() as attempts:
                from skyfleet.viz.replay_html import render_replay
                render_replay(cfg.log_path, str(out))
            self.assertTrue(out.exists(), "回放 HTML 没有生成")
            self.assertGreater(out.stat().st_size, 0, "回放 HTML 是空的")
            self.assertEqual(attempts, [],
                             f"回放过程尝试导入了 matplotlib：{attempts}")

    def test_replay_html_is_self_contained(self):
        """回放必须**自包含**：不依赖本地资源、不需要联网。

        否则「零依赖回放」只是把依赖从 Python 包换成了外部文件。
        """
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "r.jsonl"), duration=30.0)
            Engine(cfg).run()
            out = Path(tmp) / "replay.html"
            from skyfleet.viz.replay_html import render_replay
            render_replay(cfg.log_path, str(out))
            html = out.read_text(encoding="utf-8")
        lowered = html.lower()
        for pattern in ("http://", "https://", "//cdn"):
            self.assertNotIn(pattern, lowered,
                             f"回放 HTML 含外部引用 {pattern}，不再自包含")


class TestMetricsDoNotNeedMatplotlib(unittest.TestCase):
    def test_metrics_compute_with_matplotlib_blocked(self):
        """指标计算同样不得依赖绘图库。"""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = replace(load_config(SCENARIO),
                          log_path=str(Path(tmp) / "m.jsonl"), duration=30.0)
            Engine(cfg).run()
            with matplotlib_forbidden() as attempts:
                metrics = compute_metrics(cfg.log_path)
            self.assertEqual(attempts, [],
                             f"指标计算尝试导入了 matplotlib：{attempts}")
        # 指标必须真的算出来了，而不是静默返回空壳
        self.assertIn("tasks", metrics)
        self.assertEqual(metrics["tasks"]["created"],
                         load_config(SCENARIO).tasks)


class TestDeclaredOptionalDependencies(unittest.TestCase):
    def test_matplotlib_is_declared_optional(self):
        self.assertIn("matplotlib", OPTIONAL_DEPENDENCIES)

    def test_only_matplotlib_is_declared_optional(self):
        """声明表必须**恰好**是实际的可选依赖集。

        多一个会让真实错误被误译成「缺少可选依赖」；少一个会让用户
        看到一段无法据以行动的 ImportError。本条防止两者漂开。
        """
        self.assertEqual(set(OPTIONAL_DEPENDENCIES), {"matplotlib"})


class TestCliMessageMatchesReality(unittest.TestCase):
    @staticmethod
    def _message_region() -> str:
        """取缺依赖分支的**代码**，剥掉注释。

        必须剥注释：解释「旧消息曾经怎么写」的注释里必然含有那句错误
        说法，直接搜源码会把注释当成消息本身——这正是本测试第一次
        写错的原因（断言红在一个只存在于注释里的字符串上）。
        """
        source = Path("skyfleet/cli.py").read_text(encoding="utf-8")
        start = source.index("root not in OPTIONAL_DEPENDENCIES")
        window = source[start:start + 900]
        return "\n".join(line for line in window.splitlines()
                         if not line.lstrip().startswith("#"))

    def test_the_missing_dependency_message_does_not_claim_replay_needs_it(self):
        """CLI 的缺依赖提示不得声称回放需要 matplotlib。

        旧消息写「画图与回放需要 matplotlib」，与事实不符（见本文件
        第一个测试类）。消息文本会漂，所以这里断言它。
        """
        code = self._message_region()
        self.assertNotIn(
            "画图与回放", code,
            "CLI 又声称回放需要 matplotlib 了——回放实际不需要")
        self.assertIn("matplotlib", code)

    def test_the_message_says_replay_does_not_depend_on_it(self):
        """光说「不声称」不够，还应当**主动澄清**回放不受影响。"""
        code = self._message_region()
        self.assertIn("回放", code,
                      "缺依赖提示没有澄清回放不受影响，用户仍会以为"
                      "回放也用不了")

    def test_the_message_names_the_failing_command(self):
        """提示必须带上出错的命令名，否则用户不知道是哪一步缺依赖。"""
        code = self._message_region()
        self.assertIn("args.command", code,
                      "缺依赖提示没有指明是哪个命令失败")


if __name__ == "__main__":
    unittest.main()
