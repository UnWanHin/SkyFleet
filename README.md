# SkyFleet —— 空中近距离主从无人机集群通信仿真

面向《计算机网络》课程报告的离散事件仿真：**1 主节点 + N 从节点 + 中继**的
无人机配送集群，在城市楼群上空执行取餐—投递任务，并在建筑遮挡、信道竞争、
节点失效的条件下维持通信与任务闭环。

**零第三方依赖**：仿真核心只用 Python 标准库（`heapq` / `dataclasses` /
`hashlib` / `json`）。不需要 ROS 2、ns-3、OMNeT++、MATLAB——仿真与通信
协议是同一份纯 Python 代码。只有画图需要 matplotlib（可选，见
[可视化](#可视化)）。

---

## 快速开始

```bash
# 1) 跑基线场景（600 s 虚拟时间，约 10 s 出结果）
python3 -m skyfleet.cli all --out out

# 2) 压力对照场景
python3 -m skyfleet.cli all --config skyfleet/scenarios/stress_canyon.json --out out

# 3) 故障注入场景（失效检测 → 判定 → 重派 → 恢复）
python3 -m skyfleet.cli all --config skyfleet/scenarios/fault_chain.json --out out
```

产物：

产物按**场景名**分目录，三个场景互不覆盖：

```
out/logs/<name>.jsonl           事件溯源日志（唯一事实来源）
out/logs/<name>.metrics.txt     指标报告
out/figures/<name>/*.png|svg    报告插图（7 张，中英文标签）
out/replay/<name>.html          单文件自包含动态回放（浏览器直接打开）
```

其中 `<name>` 为 `baseline` / `stress` / `fault`。

子命令也可以单独调用：

```bash
python3 -m skyfleet run     --out out          # 只跑仿真
python3 -m skyfleet metrics --log out/logs/baseline.jsonl
python3 -m skyfleet plot    --log out/logs/baseline.jsonl --out out/figures/baseline
python3 -m skyfleet replay  --log out/logs/baseline.jsonl --out out/replay/baseline.html
```

`run` 支持 `--seed` / `--duration` 覆盖，用于扫参与敏感性分析。

---

## 场景与实测结果

三个场景构成受控对照：**同一套协议**，只改环境与负载。

| 场景 | 文件 | 规模 / 关键设置 | 任务完成 | 控制丢包 | 心跳送达 | 冲突 | 遮挡 |
|---|---|---|---|---|---|---|---|
| 基线 | `baseline_small.json` | 500 m、8 从、2 中继、20 任务 | **20/20** | 5.2 % | 99.5 % | 1078 | 135 |
| 压力 | `stress_canyon.json` | 城区 700 m、遥测 4 Hz、投递高度 50–90 m | 2/20 | 35.4 % | 48.4 % | 522 | 24159 |
| 故障 | `fault_chain.json` | 基线 + S2 于 60 s 静默、120 s 恢复；R1 于 150 s 静默 | 9/12 | 14.5 % | 84.4 % | — | 2372 |

**故障场景实测时间线**（种子 3）：

```
t= 60.00  S2 静默
t= 62.00  S2 → DEGRADED   （连续 2 次心跳未到）
t= 64.00  S2 → SUSPECT    （连续 4 次，且邻居间接探测无佐证）
t= 67.00  S2 → FAILED     （持续 3 s 未恢复）
t= 67.xx  任务重派，mission_version 递增，在途旧指派自动失效
t=120.00  S2 以新会话重新入网
t=120.85  S2 → ONLINE
```

已恢复节点 3 个，平均恢复时间 18.2 s，最长 53.9 s。

---

## 项目结构

```
skyfleet/
  kernel/       L0 仿真核心   事件调度器、独立随机流、配置
  world/        L1 世界层     几何、楼块、节点运动、链路模型、共享信道
  protocol/     L2 协议层     身份会话、成员状态机、版本/任期/去重、队列、可靠性、路由
  mission/      L3 业务层     任务生命周期、投递确认与幂等、安全动作
  observe/      L4 观测层     事件日志、指标重算、回放数据
  sim/          装配层        代理、主节点编排、引擎
  viz/          可视化        matplotlib 图表、单文件 HTML 回放
  scenarios/    场景配置
tests/
  unit/         单元测试
  integration/  事件链集成测试（含故障注入场景验收）
  determinism/  同种子可复现性
docs/superpowers/
  specs/        设计方案（含 §15 实现阶段的 11 处设计修正）
  plans/        实施计划（含验收记录）
```

依赖方向是单向的：`viz/` 只读日志文件，仿真核心**不 import** `viz/`。
因此"指标与图表同源"——两者都从日志重算，不存在两套口径。

---

## 测试

```bash
python3 -m unittest discover -s tests -t .            # 系统 Python，跳过 5 个画图测试
.venv/bin/python -m unittest discover -s tests -t .   # 含可视化，343 passed
```

**确定性是第一验收项**：同一 `master_seed` 连续两次运行，规范化日志的
SHA-256 必须一致。基线种子 1 的值为

```
6f40ceea3386dcea521e700cb4f0da0452a080a0a50ff6d09500ab1e195ce76f
```

实现方式：所有随机性来自按名字派生的独立随机流（`blake2b(master_seed + 流名)`），
不使用内建 `hash()`；所有字典遍历排序后再消费；所有周期性事件带确定性抖动。

---

## 设计要点

**信道是共享资源，不是每链路一个固定丢包率。** 这是报告 §3.1 的硬性要求：
若每条链路只掷一个固定概率的骰子，就不会有任何东西在竞争，"拥塞、排队、
竞争加剧"这些现象根本不会产生。因此信道按时隙划分竞争单位，落在同一时隙
且互相处于干扰范围内的并发发送判为冲突；广播是一次占用、多接收端分别判定。

**成员状态机带迟滞**，依据三类证据：直接心跳、邻居间接探测、链路质量趋势。
升级路径 `ONLINE → DEGRADED → SUSPECT → FAILED` 的每一级都有可配置阈值，
集成测试会断言升级**时刻**与配置阈值一致，而不是碰巧出现。

**版本/任期/顺序是载荷相关的**：`leader_epoch → session → mission_version →
去重(operation_id) → seq`。版本检查只作用于任务域消息；序号按
`(src, msg_type, epoch, version)` 分别记账。

**任务闭环止于确认被 ACK**：投递确认发出后停在原地等确认，收到 ACK 才结束
本地任务。一次确认丢失不会让任务停在"已取件"而无人知晓。

**航线 = 高空巡航 + 末端下降 + 爬升后上报**：低空链路上确认发不出去、
ACK 也回不来，所以投递完成后先爬升脱离遮挡再上报。遮挡因此是每次投递都会
短暂发生、并可自动恢复的现象。

---

## 可视化

```bash
.venv/bin/python -m pip install matplotlib     # 唯一可选依赖
.venv/bin/python -m skyfleet plot --log out/logs/baseline.jsonl --out out/figures/baseline
```

7 张图：任务完成情况、按时延分位数、六段时延分解、丢弃原因分布、
成员状态时间线、电量曲线、城区地图与轨迹。每张同时输出 PNG（220 dpi）
与 SVG（保留可搜索、可编辑的文字，便于插入报告后再改标签）。
检测到 CJK 字体时用中文标签，否则自动退回英文，不会出现方块字。

回放页面是**单个自包含 HTML**，数据内嵌，不依赖网络与外部资源，
双击即可用浏览器打开。

---

## 已知限制

- 不含飞行动力学与避障，运动为按速度沿航点直线前进（报告明确排除）。
- 能量模型是线性的，发送/悬停系数目前无实测依据，属待标定参数。
- 链路模型是 P0 的几何遮挡；非视距绕射与多径快衰落留作 P1 的
  `LinkModel` 替换实现（接口已就位，协议层零改动）。
- 时隙长度、时钟漂移、中继启用门限等参数已给出初值，尚需 P1 扫参确定。

详见 `docs/superpowers/specs/2026-10-03-uav-swarm-comms-sim-design.md`
§14（待定参数与开放问题）与 §15（实现阶段的设计修正）。
