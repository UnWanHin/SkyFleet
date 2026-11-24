# SkyFleet —— 空中近距离主从无人机集群通信仿真

面向《计算机网络》课程报告的离散事件仿真：**1 主节点 + N 从节点 + 中继**的
无人机配送集群，在城市楼群上空执行取餐—投递任务，并在建筑遮挡、信道竞争、
节点失效的条件下维持通信与任务闭环。

**零第三方依赖**：仿真核心只用 Python 标准库（`heapq` / `dataclasses` /
`hashlib` / `json`）。不需要 ROS 2、ns-3、OMNeT++、MATLAB——仿真与通信
协议是同一份纯 Python 代码。只有画图需要 matplotlib（可选，见
[可视化](#可视化)）。

---

## Docker 一键城市演示

```bash
podman build -t skyfleet:urban .
podman run --rm --name skyfleet -p 127.0.0.1:8765:8765 \
  -v skyfleet-out:/app/out skyfleet:urban
# 浏览 http://127.0.0.1:8765/，选择 S4 → (100,20,30) 派遣
```

Docker 用户把 `podman` 换成 `docker`。鏡像內建 ns-3.46 bridge 與 MuJoCo 物理仿真，瀏覽器版不開啟原生桌面視窗。預建鏡像發布後，可直接 `podman pull IMAGE` 並把上述運行命令的鏡像名替換為 `IMAGE`；本倉庫未預設外部鏡像地址。詳見 [Docker 使用演示、技術棧與 L/R/S 通訊說明](docs/docker-demo.md)。

---

### 報告章節的構建（交付物）

`docs/report/2026-10-04-results-chapter.html` 的**数字是生成的，不是手抄的**：

```bash
python3 docs/report/build_energy_section.py          # 从 out/study/*/runs.csv 生成正文
python3 docs/report/build_docx.py                    # HTML -> docx（零依赖 OOXML）
```

两层转换都不会在数据变化时自己报警，因此 `tests/integration/test_report_artifact.py`
把「交付物与数据一致」变成断言：HTML 必须与生成器逐字节一致、docx 必须能构建
并通过 OOXML 分部件与表格结构校验、必需的表号与小节必须真的在正文里。
改动扫描数据后忘记重跑生成器，会直接变红。

## 快速开始

> ns-3 通信 + MuJoCo 四旋翼的联合场景见 [ns3_bridge/README.md](ns3_bridge/README.md)：
> `network_backend=ns3` 时封包走 ns-3 Wi-Fi，飞行状态仍由 MuJoCo 推进。
> 以下默认 `all` 命令仍使用原来的纯 Python 通信与运动学模型。

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
out/replay/<name>.html          单文件二维动态回放（浏览器直接打开）
out/replay/<name>_3d.html       单文件 WebGL2 三维城市/飞行回放
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

七个场景：三个受控对照 + 四个扩展（NLOS 链路、大规模、对抗注入、物理能量）。

| 场景 | 文件 | 规模 / 关键设置 | 任务完成 | 控制丢包 | 心跳送达 | 冲突 | 遮挡 |
|---|---|---|---|---|---|---|---|
| 基线 | `baseline_small.json` | 500 m、8 从、2 中继、20 任务 | **20/20** | 5.2 % | 99.5 % | 1078 | 135 |
| 压力 | `stress_canyon.json` | 城区 700 m、遥测 4 Hz、投递高度 50–90 m | 2/20 | 35.4 % | 48.4 % | 522 | 24159 |
| 故障 | `fault_chain.json` | 基线 + S2 于 60 s 静默、120 s 恢复；R1 于 150 s 静默 | 9/12 | 14.5 % | 84.4 % | — | 2372 |
| NLOS | `baseline_nlos.json` | 基线 + ITU-R P.526 绕射链路模型 | 20/20 | 5.4 % | 99.3 % | 950 | 314 |
| 大规模 | `swarm_large.json` | 500 m、20 从、4 中继、25 节点 | 20/20 | 29.1 % | 68.8 % | — | — |
| 对抗 | `contested.json` | 峡谷 + 干扰 + 吊塔 + 禁飞区 + GNSS 失锁 + 击落 + 四类协议注入 | 2/20 | 38.2 % | 36.0 % | — | — |
| 物理能量 | `baseline_physical.json` | 基线 + 四旋翼动量理论能量模型 | 20/20 | 5.2 % | 99.5 % | 1078 | 135 |
| 续航受限 | `battery_limit.json` | 900 s、60 任务、PX4 挂 0.5 kg、初始电量 45、耗尽即坠机 | 见下 | — | — | — | — |

> **物理能量场景的协议结果与基线逐项相同**：换能量模型不得影响交付率、
> 丢失率与时延，这是 §9 可替换性要求的可证伪承诺，有架构守卫测试。
> 变的只有电量：末值最小从 69.32 升到 84.18（巡航比悬停省电），
> 且节点间离散度从 0.5 % 升到 8.8 %——原先的能耗指标区分不了谁在飞。

> **基线数字的适用范围（重要）**：上表基线列（20/20、5.2 %、99.5 %）是
> `env_seed=1` 这一次**布局抽样**的结果。该布局楼高 57.4–96.6 m，交付高度
> 80–110 m，因而交付阶段几乎全程视距。换五个布局种子做同一组协议参数，
> 控制丢包率变成 **25.6 %–33.1 %**，任务完成率降到 52.5 %–90 %，
> 遮挡丢弃数相差两个数量级。**不要把基线的单点数字当作协议性能**，
> 跨布局分布见 [P1 结果文档](docs/superpowers/specs/2026-10-04-p1-calibration-and-findings.md) §1.1。

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
  specs/        设计方案（§15 为 P0 的 12 处设计修正）；
                2026-10-04-p1-calibration-and-findings.md 为 P1 实测结果
  plans/        实施计划（含验收记录）
```

依赖方向是单向的：`viz/` 只读日志文件，仿真核心**不 import** `viz/`。
因此"指标与图表同源"——两者都从日志重算，不存在两套口径。

---

## 测试

```bash
python3 -m unittest discover -s tests -t .            # 系统 Python；缺少可选依赖时跳过相应测试
.venv/bin/python -m unittest discover -s tests -t .   # 有 MuJoCo/NumPy；未配置桥接时跳过 ns-3 测试
SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge" \
  .venv/bin/python -m unittest discover -s tests -t .  # 同时执行 ns-3 联动测试
```

**确定性是第一验收项**：同一 `master_seed` 连续两次运行，规范化日志的
SHA-256 必须一致。基线种子 1 的值为

```
aff4a602142b20c55fec107a7ebb1e7b8f115e4505490576c70254f4b259a854
```

实现方式：所有随机性来自按名字派生的独立随机流（`blake2b(master_seed + 流名)`），
不使用内建 `hash()`；所有字典遍历排序后再消费；所有周期性事件带确定性抖动。

**图也一并可重现**：固定 `svg.hashsalt` 使元素 id 不再来自内存地址，
并把 SVG 元数据里的导出时刻归一化，因此同一份日志两次导出的 SVG 文件
**逐字节相同**。不这么做的话，图能重现而文件不能，diff 里全是噪声。

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
双击即可用浏览器打开。`all` 同时输出二维和 WebGL2 三维回放；也可
单独运行 `python3 -m skyfleet.cli replay3d --log out/logs/baseline.jsonl
--out out/replay/baseline_3d.html`。三维回放从日志重建建筑、飞行轨迹和
通信跳段，不是实时 MuJoCo 窗口。多机派遣可用 `dispatch` 命令重算一段可追溯仿真：

```bash
.venv/bin/python -m skyfleet.cli dispatch \
  --config skyfleet/scenarios/ns3_mujoco.json \
  --nodes S1,S2,S3 --destination 60,0,30 --at 2 --group east \
  --out out/dispatch
```

D5 合成拱廊示例：加入 `--city skyfleet/scenarios/d5_urban_city.geojson --through 50,75,30`，
並把 `--destination` 設為 `50,92,30`；通道中心與出口集結點各自指定。
`--through X,Y,Z` 可依場景樓塊幾何規劃排隊穿越窄通道、離開後回復矩陣；
若使用 ns-3，控制指令必須實際收到才改航線。`TOPOLOGY_CHANGE` 記錄的是幾何估算的
無線連通分量，編組標籤則是任務分組，兩者都不代表建立了獨立組長／共識協議。
輸出包含 `dispatch.jsonl` 和 `dispatch_3d.html`。運行中互動可在專案目錄直接執行 `./start.sh`（預設開啟 MuJoCo 3D 與 `http://127.0.0.1:8765/` 控制台；可用 `./start.sh --duration 300 --port 9000` 覆蓋參數）。若從其他目錄啟動，也可指定腳本完整路徑。首次使用需先依 `ns3_bridge/README.md` 建置 ns-3 bridge。啟動腳本預設使用獨立的 `ns3_mujoco_urban_multihop.json`：20 dBm、LogDistance 指數 2.5、樓塊遮擋一次扣 35 dB，並啟用本地一跳鄰居心跳（從機原有的主節點上行心跳仍保留）。此場景將 R1 固定於 `(140,150,100)` 形成遮擋時仍通往 L 的兩跳路徑；直連候選門限與上游採納門限皆為 0.35、保持時間 0.2 秒，其他場景沿用原預設。派遣 S4 至 `(100,20,30)` 的原生實測中，S4 遮擋直連實際失收、切換至 R1 後首條遙測在遮擋後約 0.48 秒抵達 L（固定種子與該軌跡；不是任意城市的保證）。實測空曠單跳 160 m 收到、170 m 失收；此距離只是固定種子及封包長度的測試結果，不是所有信道條件的硬截斷。原版 `ns3_mujoco.json` 及 `classic` 的配置預設維持不變。控制台可向 S1–S4 派遣 `[100,20,30]`，保持 R1/R2 靜止以檢驗直連遇樓遮擋後切換中繼；R1/R2 同時派遣到樓後會破壞中繼回傳鏈路。原本的完整指令仍可使用：

```bash
SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge" \
  .venv/bin/python -m skyfleet.cli live \
  --config skyfleet/scenarios/ns3_mujoco_urban_multihop.json \
  --city skyfleet/scenarios/d5_urban_city.geojson \
  --duration 120 --tick-seconds 0.1 --port 8765 --viewer mujoco
```

MuJoCo 會開啟原生 3D 即時視窗，提供旋轉、縮放與自由視角；同時開啟 `http://127.0.0.1:8765/` 作為輕量派遣控制台。左側 2D 俯視迷你圖實時顯示建築與飛機位置；點擊地圖填入 X/Y，或手動輸入座標並用右側 Z 滑桿微調，原生 3D 窗口會同步顯示黃色預覽球。選 L、R1/R2、S1~S8 後按「發送適應性編隊指令」，預覽球即消失；只有被接收並執行的目標才會顯示綠球（無線封包可能丟失）。L 使用本地導航命令，R/S 仍需經 ns-3 Wi-Fi 實際接收才變更航線。3D 姿態／旋翼來自每架物理模擬器的 qpos；樓塊是展示幾何，不作為 MuJoCo 物理碰撞體；綠線代表最近 1.5 秒被協議接受的 ns-3 封包，而非射頻強度／永久連通。3D 中機號標籤與選中光環同步控制台勾選；原生雙擊機身選單機，`Tab` 循環，`1` 選 L、`2` 選中繼、`3` 選全部從機、`0`/`A` 全選。預設相機對準 L，距離 25 m；`C` 居中並跟隨目前單選機。鼠標左鍵拖動旋轉、右鍵拖動平移、滾輪縮放。展示模型預設放大 4 倍（`LiveSwarmDisplay(..., visual_scale=4)`），不改物理模型或根關節世界座標；`F` 用相機焦點 XY 與選中機高度建立黃色半透明預覽，`[`/`]` 每次調整高度 5 m，`Enter` 派遣、`Esc` 取消；收到並執行的目標顯示為綠色球。
原生視窗以快照刷新物理 3D 機身；關閉視窗會停止仿真並等待 ns-3 與 MuJoCo 渲染執行緒退出。控制台約每 300 ms 更新編隊狀態與最近接收的 ns-3 跳段；命令進入同一段正在運行的模擬，R/S 透過無線封包送達後才套用。可設 `--log out/live/dispatch.jsonl` 留存事件。
`--adaptive`（CLI 派遣）或控制頁命令啟用運動週期幾何再評估：開闊採矩陣、
窄通道採單列、碰撞風險時 HOLD；現階段採用已知矩形樓塊及節點位置，
物理慣性、感測器未知障礙與連續時間避撞不在此模型的保證內。
D5/ns-3/MuJoCo 的高頻採樣對照場景
和独立三维回放命令见 `ns3_bridge/README.md`；合成城区版本在
`skyfleet/scenarios/d5_urban_city.geojson`，包含 14 栋楼、道路、公园和丘陵。
道路与丘陵是三维回放的视觉层，尚未参与 MuJoCo 碰撞或无线传播。

---

## 已知限制

**设计上的限制**

- 不含飞行动力学与避障，运动为按速度沿航点直线前进（报告明确排除）。
- **能量模型默认仍是线性的**，发送/悬停系数无实测依据。已提供可选的
  物理模型（`energy_model: "physical"`，四旋翼动量理论，按悬停/巡航/
  爬升/下降分别计费），但**默认不启用**——启用会改变所有已发表数字与
  golden digest，那应当是一次有意识的重新基线。分析见
  [推力余裕分析](docs/analysis/2026-10-05-thrust-margin-analysis.md)。
- 邻居相对定位未实现（`PositionSource` 只做了 GNSS 漂移与失锁）。

**实测发现的协议缺口**（P1 对抗注入测出，详见
[P1 结果文档](docs/superpowers/specs/2026-10-04-p1-calibration-and-findings.md) §6）

- **序号检查无上界（未修复）**：冒充者发一个 `seq = 10^7` 的包即可把主节点记录的
  `last_seq` 推到极大值，此后该节点的**真实**心跳全部被判 `STALE_SEQ`——
  单包拒绝服务。已用测试固定该行为；修复方向是滑动窗口校验。
- `leader_epoch` **从不递增**（无重新选主流程），因此"过期任期"这一防御分支
  是死代码，端到端无法触发。
- `deadline` 原本只有发送端在用，接收端不检查有效期（**已修复**：
  `TASK_ASSIGN` / `TASK_REASSIGN` 过期即判 `DEADLINE_MISSED`）。

**电量耗尽处置**（`on_battery_empty`，默认 `none` 保留原行为；详见
[推力余裕分析](docs/analysis/2026-10-05-thrust-margin-analysis.md) §10）

- `none`：**电量归零没有任何后果**——没有耗尽事件、没有迫降、没有坠机。
  这是 P0/P1 的行为，逐位兼容。
- `land`：就地迫降，`BATTERY_EMPTY` 事件 + 停止移动 + 不再接受任务。
- `crash`：复用既有失效机制（`NODE_LEAVE(cause=BATTERY)`），主节点据此
  判定失效并重派任务。**这才是"会不会掉机"的完整闭环。**
- ⚠️ **早前的定量结论已被推翻**（见
  [推力余裕分析](docs/analysis/2026-10-05-thrust-margin-analysis.md) §11）：
  当时的 36 次运行**全部固定在城市布局 `env_seed=1`** 上，而那是一个
  离群布局，因此置信区间只覆盖协议随机性、**不含布局方差**。
- **重做后（3 策略 × 6 布局 × 3 重复 = 54 次）**：以布局配对，
  `none − crash` 完成率差为 **+0.043，t = 1.96（df=5，临界值 2.571），
  不显著**；6 个布局中有 1 个方向相反。
- **修正后的结论**：续航受限场景里**城市布局解释了完成率变异的约 94 %**
  （布局间 sd 0.304，布局内 sd 0.059）。**决定成败的是城市布局，
  不是耗尽处置策略。**
- 转发量**双峰**（走直连或经中继，总字节只差 4 %），而且
  **三种策略都是双峰**——早前称 `crash` 单峰是门限选取错误所致。
- 方法学注意：框架的 t 区间对有界离散指标不适用（会输出 <0 或 >1 的
  区间），且**从未真正做过显著性检验**（`welch_significant` 是死代码）。
  详见分析文档 §11.4。

**尚未执行的**：

- **安全动作仍只写日志、从不执行**：`safety_decision` 的输出仅用于变化检测
  与记录，`RETURN` 不返航、`HOVER` 不悬停、`HOLD` 不停住。这与
  `on_battery_empty` 不同——后者已可配置，前者仍无任何执行路径。
- 实测 **81.5 % 的节点时间在悬停**（主节点与中继按设计不接任务、全程静止；
  从机有 67 %~84 % 的时间在等任务与等 ACK）。这对仿真保真度是一个应当
  记录的观察。

**实测推翻的设计预期**

- 接入抖动**不降低冲突数**，而是把冲突从"相位对齐的彩票"变成可复现的统计量
  （降低方差，不降低均值）。
- 中继门限在两轮扫参中**均无有效影响**：转发量翻倍而丢包与送达率不变。
  根因是中继布点与从机被同一批建筑遮挡，多出的转发只是额外空口占用。
  真正需要优化的是布点，已列为 P2 第一优先项。
- 恢复时间的参数敏感性**样本量不足**，置信区间与均值同量级，报告中不得给出结论。

**尚需标定**

- 时隙长度、时钟漂移、遥测/心跳周期、中继门限已完成扫参（见 P1 结果文档）；
  交付高度相对楼高的取值被证明比任何协议参数都更敏感，列为 P2。

详见 `docs/superpowers/specs/2026-10-03-uav-swarm-comms-sim-design.md`
§9（P1 接口实现与三个协议发现）、§14（待标定参数与口径）、§15（P0 设计修正）、
§16（P0/P1 状态与 P2 路线）。
