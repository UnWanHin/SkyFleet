"""指令台：遥测下行 + 指令上行。

**为什么要有这一层**：仿真里内环（读转速 → ESC 修正 → 再积分）每 2 ms 跑一次，
但那是**飞控内部**的事，操作员看不见也管不着。真实无人机的链路是：

    飞控 ──遥测(位置/姿态/电机转速)──> 指令台
    飞控 <──指令(改目标/改增益/切换模式)── 指令台

本模块实现后半段的**指令侧**与链路数据结构；遥测帧在 `telemetry.py`。
分开的理由：指令格式是**协议**，会被外部脚本复用；遥测帧是**采样结果**。

设计取舍：
- 不用 MAVLink——它需要 pymavlink 依赖，而本项目零第三方依赖是刻意维持的
  （见 `docs/flight/simulation-stack-selection.md`）。
- 指令是纯文本，一行一条。好处是可以直接 `echo "HOLD" |` 喂进去做脚本化
  演示，也让指令台在没有 TUI 的机器上仍可用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

from flightlab.telemetry import sample_telemetry


#: 合法飞行模式
MODES = ("TRACK", "HOLD", "RETURN")

#: 可以在线调整的增益名 → `Gains` 上的属性名。
#:
#: **键必须与 `Gains` 的属性名逐字一致，且全小写**：`parse_command` 会把
#: 输入键 `.lower()` 以做到大小写不敏感，若这里留了 `k_R` 这种带大写的键，
#: `SET_GAIN k_R` 在任何拼写下都匹配不上。
#:
#: 只开放经过调参验证过的几项，不开放任意字段，避免操作员把仿真弄崩。
TUNABLE_GAINS: dict[str, str] = {
    "kp_pos": "kp_pos",
    "kd_pos": "kd_pos",
    "k_r": "k_R",
    "k_omega": "k_omega",
    "kp_motor": "kp_motor",
    "v_max": "v_max",
    "a_max_horizontal": "a_max_horizontal",
}


@dataclass(frozen=True)
class Command:
    """一条已解析的指令。

    `kind` 决定用哪些字段：SET_TARGET 用 `value`（三元），SET_GAIN 用
    `key` + `value`（标量），模式类指令两者都不用。
    """

    kind: str
    value: Any = None
    key: str = ""

    def __str__(self) -> str:
        if self.kind == "SET_TARGET":
            x, y, z = self.value
            return f"SET_TARGET({x:g}, {y:g}, {z:g})"
        if self.kind == "SET_GAIN":
            return f"SET_GAIN({self.key}={self.value:g})"
        return self.kind


def _finite(value: float) -> bool:
    """只有有限实数才允许进入仿真。

    `float("nan")` 与 `float("inf")` 都能被 `float()` 解析，而且
    `nan <= 0.0` 是 **False**——所以单纯用一个下界护栏拦不住 nan。nan 一旦
    进控制器，MuJoCo 会在每一步打印 "simulation is unstable" 而控制台还在
    报"已生效"，是典型的静默失败。
    """
    return math.isfinite(value)


def parse_command(text: str) -> Command | None:
    """把操作员输入解析成指令；无法解析返回 None。

    **为什么不抛异常**：这是终端交互循环的入口。垃圾输入是常态（手滑、
    粘贴了别的东西、多打了个空格），抛异常会让整个指令台崩掉，而操作员
    本该只看到一句「无法识别」。
    """
    if not text:
        return None
    parts = text.strip().split()
    if not parts:
        return None
    head = parts[0].upper()
    args = parts[1:]

    if head == "SET_TARGET":
        if len(args) != 3:
            return None
        try:
            xyz = [float(a) for a in args]
        except ValueError:
            return None
        if not all(_finite(v) for v in xyz):
            return None
        return Command(kind="SET_TARGET", value=xyz)

    if head == "SET_GAIN":
        if len(args) != 2:
            return None
        key, raw = args[0].lower(), args[1]
        if key not in TUNABLE_GAINS:
            return None
        try:
            val = float(raw)
        except ValueError:
            return None
        if not _finite(val) or val <= 0.0:
            # 增益为 0、负或非有限会让控制器失稳，直接拒绝比让仿真发散好。
            return None
        return Command(kind="SET_GAIN", key=key, value=val)

    # 无参数指令
    for kind in ("HOLD", "RESUME", "RETURN", "QUIT", "HELP"):
        if head == kind and not args:
            return Command(kind=kind)
    return None


def apply_command(sim, cmd: Command) -> str:
    """把指令作用到仿真，返回一句给操作员看的确认。

    **为什么改的是 `sim.target` 而不是直接改位置**：指令台发的是**目标**，
    不是瞬移。无人机必须靠自己的控制器飞过去——这正是要演示的东西。
    直接写 `sim.pos` 会让内环失去意义。

    HOLD 之前的目标记在 `sim` 自己身上（`sim.gcs_saved_target`），而不是由
    调用方传一个字典进来：会话状态本来就该跟着被控制的那个对象走，否则任何
    一个新的前端忘了传字典，RESUME 就成了空承诺。
    """
    import numpy as np

    if cmd.kind == "SET_TARGET":
        sim.target = np.asarray(cmd.value, dtype=float)
        return f"目标已设为 ({cmd.value[0]:g}, {cmd.value[1]:g}, {cmd.value[2]:g})"

    if cmd.kind == "SET_GAIN":
        attr = TUNABLE_GAINS[cmd.key]
        gains = getattr(sim, "gains", None)
        if gains is None or not hasattr(gains, attr):
            return f"该增益不可调：{cmd.key}"
        old = getattr(gains, attr)
        # `Gains` 是 frozen dataclass，必须整体替换而非 setattr。替换后要把
        # **三处引用**（sim.gains / controller.gains / esc.gains）全部指向新
        # 对象：它们初始是同一个实例，只换其中一个会让公开属性与实际生效的
        # 增益不一致；`kp_motor` 更是只被 `ESCController` 读，只换 controller
        # 会导致"确认已生效但轨迹逐位不变"。
        new_gains = replace(gains, **{attr: float(cmd.value)})
        sim.gains = new_gains
        for holder in ("controller", "esc"):
            obj = getattr(sim, holder, None)
            if obj is not None and hasattr(obj, "gains"):
                obj.gains = new_gains
        return f"{cmd.key}: {old:g} -> {cmd.value:g}"

    if cmd.kind == "HOLD":
        # 只在第一次 HOLD 时保存，重复 HOLD 不能把"已冻结的当前位置"当成原目标
        if getattr(sim, "gcs_saved_target", None) is None:
            sim.gcs_saved_target = np.asarray(sim.target, dtype=float).copy()
        sim.target = np.asarray(sim.pos, dtype=float).copy()
        return "已悬停（目标锁在当前位置，原目标已记住）"

    if cmd.kind == "RESUME":
        restored = getattr(sim, "gcs_saved_target", None)
        if restored is None:
            return "已恢复航线跟踪（此前没有 HOLD，无目标可恢复）"
        sim.target = np.asarray(restored, dtype=float).copy()
        sim.gcs_saved_target = None
        return (f"已恢复航线跟踪，目标回到 "
                f"({restored[0]:g}, {restored[1]:g}, {restored[2]:g})")

    if cmd.kind == "RETURN":
        sim.target = np.zeros(3)
        return "正在返航（目标设为原点）"

    if cmd.kind == "HELP":
        return ""          # 由调用方打印完整说明

    return f"未知指令：{cmd.kind}"


HELP_TEXT = """\
指令台指令：
  SET_TARGET X Y Z     设定飞行目标（米）
  SET_GAIN NAME VALUE  在线调参，可调：""" + ", ".join(TUNABLE_GAINS) + """
  HOLD                 悬停（目标锁在当前位置，原目标会被记住）
  RESUME               恢复 HOLD 之前的目标
  RETURN               返航（目标设为原点）
  HELP                 显示本说明
  QUIT                 退出
遥测字段：pos=位置  tilt=倾角  ω=四电机转速  F=四桨总推力
说明：SET_GAIN 的键大小写不敏感（k_r 与 k_R 等价）
"""


def _dispatch(sim, raw: str, out) -> tuple[int, int]:
    """处理一行输入，返回 (执行数, 拒绝数)。

    交互与脚本两个前端共用这一处，避免"演示能跑通、真人一按就崩"。
    """
    cmd = parse_command(raw)
    if cmd is None:
        if raw.strip():
            out.write(f"[拒绝] 无法识别：{raw.strip()}\n")
            return 0, 1
        return 0, 0
    if cmd.kind == "HELP":
        out.write(HELP_TEXT + "\n")
        return 1, 0
    msg = apply_command(sim, cmd)
    out.write(f"[执行] {msg}\n")
    return 1, 0


def run_scripted(stream, out, *, duration: float = 10.0,
                 telemetry_hz: float = 10.0, command_interval: float = 1.0,
                 start=(0.0, 0.0, 1.2),
                 target=(1.0, 0.0, 1.2)) -> dict:
    """无交互地跑一段会话：从 `stream` 逐行读指令，遥测写到 `out`。

    与 `run_interactive` 共用同一套指令语义，区别只在输入源。

    返回会话摘要（步数、被拒指令数、最终位置），供测试与日志使用。
    """
    import numpy as np

    from flightlab.sim import QuadrotorSim

    sim = QuadrotorSim()
    sim.reset(pos=np.asarray(start, dtype=float),
              rotor_speed=sim.params.omega_hover)
    sim.target = np.asarray(target, dtype=float).copy()

    pending = [ln for ln in stream.read().splitlines() if ln.strip()]
    steps = rejected = 0
    t = 0.0
    next_report = 0.0
    next_command = 0.0
    report_every = 1.0 / telemetry_hz if telemetry_hz > 0 else 0.0

    while t < duration:
        # 指令節奏與遙測頻率**解耦**：若綁在一起，一個 4 行腳本會在任何時長
        # 下都於 0.4 s 內跑完，演示時看不到飛機執行過程。
        quit_requested = False
        if pending and t >= next_command:
            next_command = t + command_interval
            raw = pending.pop(0)
            if parse_command(raw) is not None and \
                    parse_command(raw).kind == "QUIT":
                out.write("[退出]\n")
                quit_requested = True
            else:
                s, r = _dispatch(sim, raw, out)
                steps += s
                rejected += r
        if quit_requested:
            break

        if report_every and t >= next_report:
            next_report += report_every
            frame = sample_telemetry(sim, t, target=sim.target)
            out.write(frame.as_row(sim.params) + "\n")

        sim.step(sim.target, t)
        t += sim.dt

    return {
        "steps": steps,
        "rejected": rejected,
        "t": t,
        "final_pos": np.asarray(sim.pos, dtype=float).copy(),
        "sim": sim,
    }


def run_interactive(*, duration: float = 600.0, telemetry_hz: float = 20.0,
                    command_interval: float = 0.0, realtime: bool = True,
                    start=(0.0, 0.0, 1.2), target=(1.0, 0.0, 1.2),
                    view: bool = False) -> None:
    """交互式指令台：终端实时显示遥测，键盘输入指令。

    **为什么用后台线程读 stdin**：主循环必须按物理步长（2 ms）推进，不能
    阻塞在 `input()` 上。线程只负责把整行丢进队列，主循环每圈非阻塞取一次。

    **为什么不用 curses**：curses 会接管整个终端，退出后画面残留、管道里
    也没法重定向。这里用「原地覆盖一行」实现刷新，重定向到文件时自动退化成
    逐行追加——演示录屏和事后查日志都能用。

    `realtime=True` 时按墙钟节流，使 1 s 仿真 ≈ 1 s 真实时间；否则物理会
    跑到约 27× 实时，`--duration` 就不代表操作员以为的秒数了。
    """
    import queue
    import select
    import sys
    import threading
    import time

    import numpy as np

    from flightlab.sim import QuadrotorSim

    sim = QuadrotorSim()
    sim.reset(pos=np.asarray(start, dtype=float),
              rotor_speed=sim.params.omega_hover)
    sim.target = np.asarray(target, dtype=float).copy()

    report_every = 1.0 / telemetry_hz if telemetry_hz > 0 else 0.0

    cmds: queue.Queue[str] = queue.Queue()
    stop = threading.Event()

    def reader() -> None:
        """逐行读 stdin。用 select 带超时轮询，否则线程会永远阻塞在
        `for line in sys.stdin` 上，`stop` 永远看不到，退出后留下僵尸线程
        继续抢管道输入。"""
        while not stop.is_set():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            line = sys.stdin.readline()
            if not line:
                return
            cmds.put(line)

    tty = sys.stdout.isatty()
    threading.Thread(target=reader, daemon=True).start()

    sys.stdout.write(HELP_TEXT + "\n")
    sys.stdout.flush()

    t = 0.0
    next_report = 0.0
    next_command = 0.0
    mode = "TRACK"
    viewer = None
    t_wall0 = time.monotonic()

    try:
        if view:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)

        while t < duration:
            # ---- 指令 ----
            # 交互打字時 command_interval=0：來一條執行一條。
            # 但從管道餵腳本時全部指令會在同一時刻到達，若不限速，最後一條
            # （例如 HOLD）會立刻覆蓋先前所有目標，飛機一步都不飛。
            if t >= next_command:
                try:
                    raw = cmds.get_nowait()
                except queue.Empty:
                    raw = None
                if raw is not None:
                    next_command = t + command_interval
                    cmd = parse_command(raw)
                    if cmd is not None and cmd.kind == "QUIT":
                        raise KeyboardInterrupt
                    s, _ = _dispatch(sim, raw, sys.stdout)
                    if s:
                        if cmd.kind in ("HOLD", "RETURN"):
                            mode = cmd.kind
                        elif cmd.kind in ("RESUME", "SET_TARGET"):
                            mode = "TRACK"
                    sys.stdout.flush()

            # ---- 遥测 ----
            if report_every and t >= next_report:
                next_report += report_every
                frame = sample_telemetry(sim, t, target=sim.target, mode=mode)
                line = frame.as_row(sim.params)
                if tty:
                    sys.stdout.write("\r" + line.ljust(110))
                else:
                    sys.stdout.write(line + "\n")
                sys.stdout.flush()

            sim.step(sim.target, t)
            t += sim.dt
            if viewer is not None and viewer.is_running():
                viewer.sync()
            if realtime:
                # 按墙钟节流，否则物理跑到 ~27× 实时，--duration 名不副实。
                lag = (t_wall0 + t) - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if viewer is not None:
            viewer.close()
        if tty:
            sys.stdout.write("\n")
        sys.stdout.write(
            f"\n会话结束：t={t:.2f}s  pos=({sim.pos[0]:.3f}, {sim.pos[1]:.3f}, "
            f"{sim.pos[2]:.3f})\n")
        sys.stdout.flush()
