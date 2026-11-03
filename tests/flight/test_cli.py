"""flightlab 命令行的测试。

**为什么 CLI 要测**：库函数再对，命令行参数拼错、默认值不合理、子命令
没注册，用户第一次就会撞墙。而这些错误单元测试完全照不到。
"""
from __future__ import annotations

import contextlib
import io
import unittest

from flightlab.cli import build_parser, main


class TestParser(unittest.TestCase):
    def test_all_documented_commands_are_registered(self):
        """文档里写的四条命令必须真的存在。"""
        parser = build_parser()
        for name in ("demo", "render", "stills", "tune", "view", "swarm"):
            with self.subTest(command=name):
                args = parser.parse_args([name])
                self.assertTrue(callable(getattr(args, "func", None)),
                                f"子命令 {name} 没有绑定处理函数")

    def test_missing_command_is_an_error(self):
        with self.assertRaises(SystemExit):
            build_parser().parse_args([])

    def test_tune_defaults_to_horizontal_axis(self):
        """默认轴必须是 x。

        纯升降不倾斜机身，姿态环完全不参与，k_R / k_omega 扫出来三行一模
        一样——那样扫了等于没扫。默认值必须是能暴露姿态环的那个。
        """
        args = build_parser().parse_args(["tune"])
        self.assertEqual(args.axis, "x")

    def test_view_defaults_to_single_drone(self):
        """view 默认单机；--swarm N 才切多机。"""
        self.assertEqual(build_parser().parse_args(["view"]).swarm, 0)
        self.assertEqual(
            build_parser().parse_args(["view", "--swarm", "11"]).swarm, 11)

    def test_tune_rejects_a_non_physical_axis(self):
        with self.assertRaises(SystemExit):
            build_parser().parse_args(["tune", "--axis", "w"])


class TestDemoCommand(unittest.TestCase):
    def test_demo_reports_tracking_error_and_hover_speed(self):
        """demo 的输出必须同时给出跟踪误差与悬停转速。

        前者证明跟得住，后者证明物理对（收敛到理论值 1873 rad/s）。
        """
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = main(["demo", "--duration", "12"])
        out = buf.getvalue()
        self.assertEqual(code, 0)
        self.assertIn("最大跟踪误差", out)
        self.assertIn("最终转子转速", out)
        self.assertIn("1873", out, "末转速没有收敛到悬停理论值")


if __name__ == "__main__":
    unittest.main()
