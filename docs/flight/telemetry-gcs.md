# 遥测下行与指令台

本文说明 `flightlab` 的遥测/指令链路：为什么要有它、它和飞行控制是什么关系、
以及怎么用。

## 1. 三个环，别混在一起

仿真里同时存在三个闭环，它们的时间尺度差三个数量级：

| 环 | 内容 | 频率 | 在哪 |
|---|---|---|---|
| **内环** | 读实际转速 → ESC 算力矩 → 力矩驱动转子 → 再积分 | 500 Hz（2 ms） | `sim.py` / `control.py` |
| **遥测/指令环** | 抽帧送终端、终端发指令回仿真 | 20 Hz | `telemetry.py` / `gcs.py` |
| **任务环** | 航点、投递、返航 | 0.1–1 Hz | `cli.py` |

**内环是飞控本身**，不是外部接口。即使没人看遥测，内环照跑。这一点很容易搞错，
所以先说清：`ESCController` 在每一步都执行

```
tau = k_m * w_des^2  +  kp_motor * (w_des - w_actual)
```

前半是前馈（转速平方即所需力矩），后半是反馈（实际与期望之差）。`w_actual`
就是从 MuJoCo 关节读回来的真实转速——这就是「回传电机数据再修正」的物理实现。

**遥测不是控制回路的一部分**，它只是把某一时刻的结果抽出来给人看。

## 2. 遥测帧字段

`TelemetryFrame`（`flightlab/telemetry.py`）字段全是物理量，不暴露 MuJoCo 的
`qpos`/`qvel` 布局，因此仿真内部怎么改都不影响消费方：

| 字段 | 含义 | 单位 |
|---|---|---|
| `t` | 时刻 | s |
| `pos` | 世界坐标位置 | m |
| `quat` | 姿态，wxyz 四元数 | — |
| `omega` | 机体角速度 | rad/s |
| `rotor_speeds` | 四电机**实际**转速 | rad/s |
| `thrust` | 四电机实际推力 `k_f·ω²` | N |
| `target` | 当前飞行目标 | m |
| `mode` | `TRACK` / `HOLD` / `RETURN` | — |

派生量：`thrust_total`（四桨总推力）、`tilt_deg`（机体 z 轴与竖直夹角）。

## 3. 指令格式

一行一条，纯文本。不用 MAVLink 是为了守住零第三方依赖。

| 指令 | 作用 |
|---|---|
| `SET_TARGET X Y Z` | 设飞行目标（米） |
| `SET_GAIN NAME VALUE` | 在线调参 |
| `HOLD` | 悬停：目标锁到当前位置，原目标记在 `sim.gcs_saved_target` |
| `RESUME` | 恢复 HOLD 之前的目标（没 HOLD 过则只回一句提示）|
| `RETURN` | 返航：目标设原点 |
| `HELP` | 显示说明 |
| `QUIT` | 退出 |

可调增益（`TUNABLE_GAINS`，只开放经过调参验证的几项，避免操作员把仿真弄崩）：
`kp_pos`、`kd_pos`、`k_r`、`k_omega`、`kp_motor`、`v_max`、`a_max_horizontal`。

**键名一律小写**，输入键会被 `.lower()` 以做到大小写不敏感，所以 `k_r` 和 `k_R`
等价。`TUNABLE_GAINS` 的值才是 `Gains` 上的真实属性名（`k_r` → `k_R`）。

`SET_GAIN` 改增益时会把 `sim.gains`、`sim.controller.gains`、`sim.esc.gains`
**三处引用一起换成新对象**。它们初始是同一个实例；只换 `controller.gains` 会让
`kp_motor`（只被 `ESCController` 读）确认"已生效"却轨迹逐位不变，也会让公开属性
`sim.gains` 与实际生效的增益不一致。

**垃圾输入返回 `None` 而不抛异常**——这是终端交互入口，手滑是常态，抛异常会让
整个指令台崩掉。

**`nan` / `inf` 必须经 `math.isfinite` 拦掉**。`float("nan")` 能解析，且
`nan <= 0.0` 是 `False`，所以单靠下界护栏完全拦不住。nan 一旦进控制器，MuJoCo
每步都报 `simulation is unstable`，而控制台还在报"已生效"——典型的静默失败。

**`SET_GAIN` 用 `dataclasses.replace` 整体替换 `Gains`**，因为它是 frozen
dataclass。好处是改增益是原子操作，不会出现一半新一半旧的中间态。

## 4. 用法

```bash
# 交互：终端实时遥测，键盘发指令
.venv/bin/python -m flightlab gcs

# 同时开 3D 窗口
.venv/bin/python -m flightlab gcs --view

# 管道喂脚本（演示/自动化）
printf 'SET_TARGET 0.8 0.5 1.6\nSET_GAIN kp_pos 2.5\nHOLD\n' \
  | .venv/bin/python -m flightlab gcs --duration 2.5 --hz 2 --command-interval 0.8
```

**`--command-interval` 为什么必要**：交互打字时指令天然间隔，但管道喂脚本时三条
指令会在同一时刻全部到达。若不限速，最后一条 `HOLD` 会立刻覆盖前面的目标，飞机
一步都不飞——看着像"指令无效"，实际是指令太快。交互模式默认 0（来一条执行一条），
喂脚本时设成 0.3–1.0 s。

## 5. 实测

`printf 'SET_TARGET 0.8 0.5 1.6\nSET_GAIN kp_pos 2.5\nHOLD\n' | flightlab gcs
--duration 2.5 --hz 2 --command-interval 0.8`：

```
[执行] 目标已设为 (0.8, 0.5, 1.6)
t=  0.00s  pos=( 0.000, 0.000, 1.200)  tilt=  0.0°  ω=(1873,1873,1873,1873)  F= 280.6mN  TRACK
t=  0.50s  pos=( 0.156, 0.099, 1.344)  tilt= 12.6°  ω=(1879,1844,1871,1864)  F= 278.1mN  TRACK
[执行] kp_pos: 1.562 -> 2.5
t=  1.00s  pos=( 0.613, 0.384, 1.519)  tilt=  5.8°  ω=(1856,1840,1849,1845)  F= 273.1mN  TRACK
t=  1.50s  pos=( 0.844, 0.528, 1.611)  tilt=  8.9°  ω=(1839,1845,1843,1844)  F= 271.6mN  TRACK
[执行] 已悬停（目标锁在当前位置）
t=  2.00s  pos=( 0.826, 0.516, 1.616)  tilt=  2.5°  ω=(1875,1869,1873,1871)  F= 280.3mN  HOLD
```

三处可核对：

1. **悬停推力** 280.6 mN ≈ 机重 28.6 g × 9.81。差 0.6 mN 是腿的浮力项，已在
   `swarm.py` 的 `_assert_mass_matches_params` 中约束。
2. **前飞时转速分裂** 1879/1844/1871/1864——X 型布局的差动，不是对称的。
3. **改增益后位移加速** 同样 0.5 s，1.562 时走 0.156 m，2.5 时走 0.457 m。

## 6. 测试

`tests/flight/test_telemetry.py`，21 条，分四组：

| 组 | 验什么 | 关键断言 |
|---|---|---|
| `TestSampleTelemetry` | 抽帧取对了字段 | 悬停转速等于解析解 `sqrt(mg/4k_f)`；总推力等于 `mg`（守恒）；前飞时 `tilt_deg > 1°`，悬停时 `< 1°` |
| `TestParseCommand` | 指令解析与拒绝 | 垃圾输入返回 `None` 不抛异常；大小写/空白无关 |
| `TestApplyCommand` | 指令真的改变飞行 | `SET_TARGET` 后 3 s 内飞到 0.35 m 以上；**每个**可调增益改完轨迹都必须变，且三处引用仍同一对象；`RESUME` 恢复 HOLD 前目标 |
| `TestRunScripted` | 端到端循环 | 有效指令计数、垃圾指令计数、`HELP` 不算未知指令、`QUIT` 提前停、`--hz 0` 不崩 |

**这些断言为什么不是同义反复**：期望值来自独立来源——解析解（`sqrt(mg/4k_f)`）、
物理守恒（推力总和 = 重力）、相对比较（比值而非绝对差）、因果关系（改增益则轨迹
必变）。没有一条是把代码重算一遍。

初版有一条正是同义反复：`assert_allclose(f.thrust, k_f * f.rotor_speeds**2)` 等于把
`telemetry.py` 那一行抄进测试，采样返回垃圾转速时照样通过。已改为断言总推力等于
`mg`。同样，倾角测试原来断言 `quat[3] < 0.9999`——索引 3 是 z/yaw 分量，前飞（绕 y
俯仰）时恰为 0，通过的原因是没有偏航而非有倾角；已改为断言 `tilt_deg` 并加了悬停
反向对照。

## 7. 已知限制

- **遥测只到 20 Hz**。内环是 500 Hz，遥测是采样；想看内环细节要直接读 `sim`
  每个 `dt` 的中间量，走遥测会漏掉。这是刻意的：500 Hz 打进终端没人看得见。
- **`RETURN` 的"家"是原点**。单机仿真没有起降点概念，`target` 设 `(0,0,0)`
  会让飞机往地面撞。多机场景里 home 由 `SwarmLayout` 决定。
- **没有断链**。指令台和仿真是同一进程，`apply_command` 立即生效。真实链路有
  延迟和丢包——那是 `skyfleet` 那侧负责建模的东西，不在这里。
- **`--hz 0` 表示"不打遥测"**，循环仍按物理步长推进。
- **`realtime=True` 默认按墙钟节流**，使 1 s 仿真 ≈ 1 s 真实时间（实测 0.86×）。
  关掉它会跑到约 27× 实时，`--duration` 就不再是操作员以为的秒数。
