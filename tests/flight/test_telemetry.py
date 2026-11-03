"""遥测与指令台的测试。

**接缝**（测试只打在这些地方，不动内部实现）：
- `sample_telemetry(sim, t)`：从仿真抽一帧遥测
- `parse_command(text)`：把操作员输入解析成指令
- `apply_command(sim, cmd)`：把指令作用到仿真，返回一句确认
- `run_scripted(stream, out, ...)`：端到端脚本会话

**期望值来源必须独立于实现**：解析解（`sqrt(mg/4k_f)`）、物理守恒
（推力总和 = 重力）、相对比较（比值而非绝对差）。凡是把代码重算一遍的
断言都算同义反复，这里一律不用。
"""
from __future__ import annotations

import io
import math
import unittest

import numpy as np

from flightlab.params import DEFAULT as P
from flightlab.sim import QuadrotorSim
from flightlab.telemetry import sample_telemetry


class TestSampleTelemetry(unittest.TestCase):
    def test_hover_frame_reports_analytic_values(self):
        """★ 悬停一帧的转速与推力必须等于解析解。

        这是整条遥测链的证伪测试：如果采样取错了字段（比如读到 qvel 的
        别的位置、或者把符号搞反），数字立刻对不上解析解。
        """
        sim = QuadrotorSim()
        sim.reset(pos=(0.0, 0.0, 1.2), rotor_speed=P.omega_hover)
        target = np.array([0.0, 0.0, 1.2])
        for k in range(200):
            sim.step(target, k * sim.dt)

        f = sample_telemetry(sim, 1.0)

        # 悬停：四桨转速都应是 omega_hover（解析解 sqrt(mg/4k_f)）
        np.testing.assert_allclose(f.rotor_speeds, P.omega_hover, rtol=0.02,
                                   err_msg="悬停转速没收敛到解析解")
        # 总推力必须支撑重力——这是**守恒关系**，不是把 k_f*ω² 重算一遍。
        # （曾经这里断言 f.thrust == k_f*speeds²，那只是把实现抄了一遍，
        # 采样返回垃圾转速时照样通过。）
        self.assertAlmostEqual(f.thrust_total, P.mass * P.g, delta=0.01)
        # 每桨推力必须为正且合理：悬停时每桨承担约 1/4 机重
        np.testing.assert_allclose(f.thrust, P.mass * P.g / 4.0, rtol=0.05)

    def test_frame_carries_attitude_and_position(self):
        """位置与姿态必须各有四个/三个分量，且姿态是单位四元数。"""
        sim = QuadrotorSim()
        sim.reset(pos=(0.3, -0.2, 1.0), rotor_speed=P.omega_hover)
        f = sample_telemetry(sim, 0.0)
        self.assertEqual(f.pos.shape, (3,))
        self.assertEqual(f.quat.shape, (4,))
        self.assertEqual(f.omega.shape, (3,))
        self.assertEqual(f.rotor_speeds.shape, (4,))
        self.assertEqual(f.thrust.shape, (4,))
        self.assertAlmostEqual(float(np.linalg.norm(f.quat)), 1.0, places=6)
        np.testing.assert_allclose(f.pos, [0.3, -0.2, 1.0], atol=1e-6)

    def test_forward_flight_tilts_the_body(self):
        """★ 前飞时机身必须倾斜，且倾角可从公开派生量读出。

        原来这里断言 `quat[3] < 0.9999`——索引 3 是 **z/yaw** 分量，前飞
        （绕 y 轴俯仰）时它恰好是 0，断言通过是因为「没有偏航」而不是
        「有倾角」。纯偏航的姿态也能通过。改为断言 `tilt_deg` 本身。
        """
        sim = QuadrotorSim()
        sim.reset(pos=(0.0, 0.0, 1.2), rotor_speed=P.omega_hover)
        target = np.array([2.0, 0.0, 1.2])
        for k in range(300):
            sim.step(target, k * sim.dt)
        f = sample_telemetry(sim, 0.6)
        self.assertGreater(f.tilt_deg, 1.0, "前飞时机身未倾斜")

    def test_hover_has_near_zero_tilt(self):
        """反向对照：悬停时倾角应接近 0——否则上面的断言可能恒真。"""
        sim = QuadrotorSim()
        sim.reset(pos=(0.0, 0.0, 1.2), rotor_speed=P.omega_hover)
        target = np.array([0.0, 0.0, 1.2])
        for k in range(300):
            sim.step(target, k * sim.dt)
        f = sample_telemetry(sim, 0.6)
        self.assertLess(f.tilt_deg, 1.0, "悬停时竟有倾角")


class TestParseCommand(unittest.TestCase):
    """指令解析。操作员输入的东西必须能变成确定的指令，垃圾必须被挡住。"""

    def test_set_target_parses_three_coordinates(self):
        from flightlab.gcs import parse_command

        cmd = parse_command("SET_TARGET 1.5 -2 3")
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.kind, "SET_TARGET")
        np.testing.assert_allclose(cmd.value, [1.5, -2.0, 3.0])

    def test_gain_command_parses_name_and_value(self):
        from flightlab.gcs import parse_command

        cmd = parse_command("SET_GAIN kp_pos 2.5")
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.kind, "SET_GAIN")
        self.assertEqual(cmd.key, "kp_pos")
        self.assertAlmostEqual(cmd.value, 2.5)

    def test_every_advertised_gain_is_actually_accepted(self):
        """★ HELP 里列出的每个增益都必须真能被 SET_GAIN 接受。

        原来 `TUNABLE_GAINS` 有个 `k_R` 键，但输入被 lower() 成 `k_r` 后
        匹配不上，于是 `SET_GAIN k_R` 在所有拼写下都被拒——而 HELP 和文档
        都写着它可调。这条测试锁死这个不变量。
        """
        from flightlab.gcs import TUNABLE_GAINS, parse_command

        for name in TUNABLE_GAINS:
            with self.subTest(name=name):
                cmd = parse_command(f"SET_GAIN {name} 1.0")
                self.assertIsNotNone(cmd, f"{name} 在 TUNABLE_GAINS 里却无法解析")

    def test_mode_commands_need_no_arguments(self):
        from flightlab.gcs import parse_command

        for text, kind in [("HOLD", "HOLD"), ("RESUME", "RESUME"),
                           ("RETURN", "RETURN"), ("QUIT", "QUIT"),
                           ("HELP", "HELP")]:
            with self.subTest(text=text):
                cmd = parse_command(text)
                self.assertIsNotNone(cmd)
                self.assertEqual(cmd.kind, kind)

    def test_garbage_is_rejected_not_crashed(self):
        """垃圾输入必须返回 None，不能抛异常——这是终端程序不崩的前提。"""
        from flightlab.gcs import parse_command

        for bad in ["", "   ", "FLY", "SET_TARGET 1 2", "SET_TARGET a b c",
                    "SET_GAIN kp_pos", "SET_GAIN kp_pos abc", "HELP me"]:
            with self.subTest(bad=bad):
                self.assertIsNone(parse_command(bad),
                                  f"{bad!r} 应被拒绝")

    def test_non_finite_numbers_are_rejected(self):
        """★ nan / inf 必须被拒。

        `float("nan")` 能解析，且 `nan <= 0.0` 是 False，所以原来那条
        `val <= 0` 的护栏完全拦不住 nan。nan 一旦进控制器，MuJoCo 每步都
        报 "simulation is unstable"，而控制台还在报"已生效"。
        """
        from flightlab.gcs import parse_command

        for bad in ["SET_GAIN kp_pos nan", "SET_GAIN kp_pos inf",
                    "SET_GAIN kp_pos -inf", "SET_GAIN kp_pos 1e400",
                    "SET_TARGET nan 0 1", "SET_TARGET inf 0 1",
                    "SET_TARGET 1e400 0 1"]:
            with self.subTest(bad=bad):
                self.assertIsNone(parse_command(bad), f"{bad!r} 应被拒绝")

    def test_case_and_whitespace_insensitive(self):
        from flightlab.gcs import parse_command

        self.assertEqual(parse_command("  hold  ").kind, "HOLD")
        self.assertEqual(parse_command("set_target 0 0 1").kind, "SET_TARGET")


class TestApplyCommand(unittest.TestCase):
    """指令上行：指令必须真的改变飞行，不只是改个字段。"""

    def _sim(self):
        sim = QuadrotorSim()
        sim.reset(pos=(0.0, 0.0, 1.2), rotor_speed=P.omega_hover)
        return sim

    def _travel(self, sim, target, seconds, t0=0.0):
        sim.target = np.asarray(target, dtype=float)
        n = int(seconds / sim.dt)
        for k in range(n):
            sim.step(sim.target, (t0 + k) * sim.dt)

    def test_hold_freezes_target_at_current_position(self):
        """HOLD：目标锁到当前位置，飞机停住不再前进。"""
        from flightlab.gcs import apply_command, parse_command

        sim = self._sim()
        self._travel(sim, (1.0, 0.0, 1.2), 0.4)
        frozen = float(sim.pos[0])
        apply_command(sim, parse_command("HOLD"))
        np.testing.assert_allclose(sim.target, sim.pos, atol=1e-9)
        # 无人机有惯性：目标冻结后它会冲过头再被拉回。要验的是「最终停住」，
        # 不是「立刻停」——后者在物理上不可能。
        for k in range(750):
            sim.step(sim.target, (200 + k) * sim.dt)
        self.assertLess(abs(float(sim.pos[0]) - frozen), 0.05,
                        f"HOLD 后仍明显漂移 x={float(sim.pos[0]):.3f}")

    def test_resume_restores_the_pre_hold_target(self):
        """★ RESUME 必须把 HOLD 之前的目标还回来。

        原来 RESUME 只返回一句确认，什么都不改：HOLD 已经把 target 冻在
        当前位置，RESUME 之后飞机仍停在那里，而遥测却显示 TRACK。
        """
        from flightlab.gcs import apply_command, parse_command

        sim = self._sim()
        original = (1.0, 0.0, 1.2)
        self._travel(sim, original, 0.4)
        apply_command(sim, parse_command("HOLD"))
        apply_command(sim, parse_command("RESUME"))
        np.testing.assert_allclose(sim.target, original, atol=1e-9,
                                   err_msg="RESUME 没有恢复 HOLD 前的目标")

    def test_set_target_moves_the_drone_there(self):
        """★ 改目标 → 飞机真的飞过去。这是整条上行的证伪测试。"""
        from flightlab.gcs import apply_command, parse_command

        sim = self._sim()
        sim.target = np.array([0.0, 0.0, 1.2])
        apply_command(sim, parse_command("SET_TARGET 0.5 0 1.2"))
        self._travel(sim, sim.target, 3.0)
        self.assertGreater(sim.pos[0], 0.35,
                           f"没飞到目标，实际 x={sim.pos[0]:.3f}")
        self.assertLess(abs(sim.pos[0] - 0.5), 0.10,
                        f"超调过大或没到位 x={sim.pos[0]:.3f}")

    def test_every_advertised_gain_actually_changes_flight(self):
        """★ 每个可调增益都必须真的影响飞行，且改完三处引用保持一致。

        原来 `apply_command` 只替换 `sim.controller.gains`：
        - `kp_motor` / `torque_rate_max` 属于 `sim.esc.gains`，回一句
          "已生效"但轨迹逐位不变；
        - `sim.gains` 与 `sim.esc.gains` 仍是旧对象，公开属性与实际生效
          的增益不一致。
        """
        from flightlab.gcs import TUNABLE_GAINS, apply_command, parse_command

        for name in TUNABLE_GAINS:
            with self.subTest(gain=name):
                sim = self._sim()
                self._travel(sim, (1.0, 0.0, 1.2), 0.3)   # 先生成运动
                before = (tuple(sim.pos), tuple(sim.rotor_speeds()))
                nominal = getattr(sim.gains, TUNABLE_GAINS[name])
                apply_command(sim, parse_command(f"SET_GAIN {name} "
                                                 f"{nominal * 0.25:.6g}"))
                # 三处引用必须仍指向同一对象（初始不变量）
                self.assertIs(sim.gains, sim.controller.gains,
                              f"{name}: sim.gains 与 controller.gains 失联")
                self.assertIs(sim.gains, sim.esc.gains,
                              f"{name}: sim.gains 与 esc.gains 失联")
                self.assertAlmostEqual(
                    getattr(sim.gains, TUNABLE_GAINS[name]),
                    nominal * 0.25, places=6)
                self._travel(sim, (1.0, 0.0, 1.2), 0.5, t0=0.3)
                after = (tuple(sim.pos), tuple(sim.rotor_speeds()))
                self.assertNotEqual(before, after,
                                    f"{name} 改了但轨迹逐位不变（假指令）")

    def test_set_gain_reduces_response_when_lowered(self):
        """把 kp_pos 调小 → 同样时间内应飞得更慢。比值断言，不看绝对值。"""
        from flightlab.gcs import apply_command, parse_command

        def travel_after(kp):
            sim = self._sim()
            self._travel(sim, (0.0, 0.0, 1.2), 0.1)
            apply_command(sim, parse_command(f"SET_GAIN kp_pos {kp}"))
            self._travel(sim, (1.0, 0.0, 1.2), 0.8, t0=0.1)
            return float(sim.pos[0])

        fast = travel_after(3.0)
        slow = travel_after(0.3)
        self.assertGreater(fast, slow * 1.5,
                           f"调小 kp_pos 后位移应明显变小：{fast:.4f} vs {slow:.4f}")


class TestRunScripted(unittest.TestCase):
    """指令台主循环。用脚本化输入测，不需要真人按键。"""

    def test_scripted_session_executes_and_rejects(self):
        """★ 端到端：有效指令被执行、垃圾指令被拒，且飞机真的飞过去了。"""
        from flightlab.gcs import run_scripted

        script = io.StringIO("SET_TARGET 0.4 0 1.2\n"
                             "FLY\n"                  # 垃圾指令，不应中断
                             "SET_GAIN kp_pos 2.0\n")
        out = io.StringIO()
        result = run_scripted(script, out, duration=2.5, telemetry_hz=10.0,
                              command_interval=0.2)

        text = out.getvalue()
        self.assertIn("目标已设为", text)
        self.assertIn("kp_pos", text)
        self.assertEqual(result["steps"], 2, "两条有效指令应被计数")
        self.assertEqual(result["rejected"], 1, "垃圾指令应被计入 rejected")
        self.assertGreater(result["final_pos"][0], 0.25,
                           f"没飞到目标 x={result['final_pos'][0]:.3f}")

    def test_help_is_recognised_in_scripted_mode(self):
        """★ HELP 是文档里的合法指令，脚本模式也不能当成"未知指令"。"""
        from flightlab.gcs import run_scripted

        out = io.StringIO()
        result = run_scripted(io.StringIO("HELP\nSET_TARGET 0.4 0 1.2\n"),
                              out, duration=0.6, command_interval=0.2)
        text = out.getvalue()
        self.assertIn("SET_TARGET", text, "HELP 没有输出说明")
        self.assertNotIn("未知指令", text)
        self.assertEqual(result["rejected"], 0, "HELP 不该被计入 rejected")

    def test_quit_stops_the_session_early(self):
        """QUIT 必须真的停下，不能跑满时长。"""
        from flightlab.gcs import run_scripted

        out = io.StringIO()
        result = run_scripted(io.StringIO("QUIT\n"), out, duration=5.0,
                              command_interval=0.5)
        self.assertLess(result["t"], 1.0, "QUIT 后仍继续跑")

    def test_scripted_session_without_quit_runs_to_duration(self):
        """没有 QUIT 时必须跑满时长，不能挂死。"""
        from flightlab.gcs import run_scripted

        result = run_scripted(io.StringIO("HOLD\n"), io.StringIO(), duration=0.5)
        self.assertAlmostEqual(result["t"], 0.5, delta=0.05)

    def test_zero_telemetry_hz_does_not_crash(self):
        """--hz 0 不能让程序崩：原来 run_interactive 会 ZeroDivisionError。"""
        from flightlab.gcs import run_scripted

        result = run_scripted(io.StringIO("SET_TARGET 1 0 1.2\n"),
                              io.StringIO(), duration=0.3, telemetry_hz=0.0)
        self.assertAlmostEqual(result["t"], 0.3, delta=0.05)


if __name__ == "__main__":
    unittest.main()
