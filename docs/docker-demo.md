# SkyFleet Docker 城市集群演示

## 1. 一条命令启动

本镜像在构建时编译 ns-3.46 和 SkyFleet bridge，运行时已包含 Python、NumPy、MuJoCo。镜像仅包含网页控制台（二维地图、路由与网络快照）；**没有原生 3D MuJoCo 窗口**。MuJoCo 仍在镜像内负责飞行动力学与姿态。Linux x86-64 的 Docker 或 Podman 均可运行。

仓库维护者首次构建：

```bash
podman build -t skyfleet:urban .               # Docker 用户：将 podman 换成 docker
podman run --rm --name skyfleet -p 127.0.0.1:8765:8765 \
  -v skyfleet-out:/app/out skyfleet:urban
```

发布：仓库已有 `.github/workflows/container.yml`，维护者推送 `v*` 标签或手动运行 GitHub Actions 后，GHCR 自动构建并发布 `:urban` 与版本标签（仅 linux/amd64）。首次发布后请在 GitHub 的 Packages 页面把该镜像可见性设为 **Public**；确认仓库 Actions 的 workflow 有 `packages: write` 权限。发布前仓库里只有 Dockerfile 与本地构建的镜像，没有可供其他人拉取的远端镜像。发布完成后：

```bash
IMAGE=ghcr.io/unwanhin/skyfleet:urban
podman pull "$IMAGE"
podman run --rm --name skyfleet -p 127.0.0.1:8765:8765 \
  -v skyfleet-out:/app/out "$IMAGE"
```

访问 <http://127.0.0.1:8765/>。Docker 的命令完全相同，只需把 `podman` 改为 `docker`。`127.0.0.1:8765:8765` 把 HTTP 控制台限定在宿主机本机；如需让同一局域网访问，再明确改成 `-p 8765:8765`。HTTP 接口没有登录认证，不应直接暴露在公网。`-v skyfleet-out:/app/out` 保留 JSONL 事件日志；`podman cp skyfleet:/app/out/live/dispatch.jsonl ./dispatch.jsonl` 可复制日志（容器运行期間）。停止使用 Ctrl+C。

镜像入口默认运行 `python3 -m skyfleet.cli live`，参数是城市 GeoJSON、120 秒仿真、`--viewer web`、容器内 `--host 0.0.0.0`、端口 8765。仿真完成后网页仍能读取最终快照；再次演示请重新运行容器。想延长演示时间，覆盖入口参数：

```bash
podman run --rm -p 127.0.0.1:8765:8765 -v skyfleet-out:/app/out \
  skyfleet:urban --config skyfleet/scenarios/ns3_mujoco_urban_multihop.json \
  --city skyfleet/scenarios/d5_urban_city.geojson \
  --host 0.0.0.0 --port 8765 --viewer web \
  --duration 300 --tick-seconds 0.1 --log out/live/dispatch.jsonl
```

`classic` 仍是 Python 配置的默认网络后端；本 Docker 的默认**演示场景**专门选择 ns-3，并未改变 `classic`。

## 2. 现场演示：建筑遮挡触发中继

1. 打开网页，等 `L`（主控）、`R1/R2`（中继）、`S1–S8`（执行机）出现在城市平面图。演示开始后的前几秒派遣，留足飞行时间。仅选择 `S4`，输入 `X=100, Y=20, Z=30`、分组 `mesh`，点击「发送适应性编队指令」。**不要**派遣 R1/R2：中继驻留才能保持回传链路。
2. 看时间、地图里的 S4 轨迹和 `api/state` 的 `routing_tree.S4`。进入 N4 楼影后，S4→L 的直连可能失收，然后父节点选为 `R1`；继续观察 S4 的遥测经 R1 转发到 L。图上几何阴影是模型估算；接收要看实际 ns-3 事件。
3. 可用终端派遣并检查快照（若时间已过，重启容器）：

```bash
curl -sS -X POST http://127.0.0.1:8765/api/dispatch \
  -H 'Content-Type: application/json' \
  -d '{"nodes":["S4"],"destination":[100,20,30],"group":"mesh"}'
curl -sS http://127.0.0.1:8765/api/state | python3 -c \
  'import json,sys;s=json.load(sys.stdin);print("t",s["t"],"S4 parent",s.get("routing_tree",{}).get("S4"),"PDR",s.get("network",{}).get("pdr_percent"))'
```

4. 查看日志中的 `OCCLUSION_ENTER`、S4 的 `MSG_TX`（先到 `L`、再到 `R1`）、R1 的 `MSG_FORWARD` 和 L 对相同 `msg_id` 的 `MSG_RX`。日志是仿真事件，不是抓到的物理无线帧；示例值因启动时间、派遣时刻与运行环境而异。项目实桥回归以相同消息 ID 和 3 秒截止时间做自动断言。

## 3. 技术栈和边界

| 层 | 实际组件 | 职责 |
|---|---|---|
| 调度与协议 | Python 3、SkyFleet `kernel` / `protocol` / `sim` | 离散事件、成员状态、选主、QoS 队列、逐跳选择、重试/ACK、任务与安全动作 |
| 物理运动 | MuJoCo 3.x + NumPy、`flightlab` | 每机四旋翼刚体积分、串级控制、旋翼动力；位置和姿态反馈给调度 |
| 无线网络 | C++23、ns-3.46 Wi-Fi 802.11a ad-hoc、6 Mb/s | 发送/接收时间、MAC/PHY、信道干扰和链路交付 |
| 城市传播 | ns-3 LogDistance + 项目 `BuildingShadowLoss`、GeoJSON 楼块 | 城市中视线遮挡时附加 35 dB（每条遮挡链路一次）；场景设 20 dBm、距离指数 2.5 |
| 服务与呈现 | Python `ThreadingHTTPServer`、HTML Canvas、HTTP JSON | 派遣接口、地图/快照；网页不是 MuJoCo 原生 3D 窗口 |
| 打包 | Fedora 多阶段构建 + Podman/Docker | 自带 ns-3 动态库和 bridge；运行日志可挂卷持久化 |

**边界：**这里的 11 个 L/R/S 是一场仿真里的逻辑无人机，不是 11 个独立容器、Kubernetes 节点或真实飞控设备；bridge 是同容器的子进程，通过 stdin/stdout 文本命令同步虚拟时间。`--viewer web` 不会启动需要 GPU/桌面服务器的原生窗口。`/api/state` 的 PDR 统计已完成的**单播跳段**，不是 PHY 误码率；网页里的 `modeled_rx_dbm` 是传播模型重新计算值，不是实际硬件 RSSI。

## 4. 集群之间具体传什么

- **L → S/R：** `TASK_ASSIGN` 携带任务 ID、起终点、截止时间、优先级；网页派遣发 `FLIGHT_CORRECTION`，携带 `destination`、`group_id`、`adaptive`（可选 `route`）。必要时由 R 中继下行。L 也会发送心跳、任务版本/成员/路由更新与 ACK。
- **S → L：** `TELEMETRY` 携带 `[x,y,z]` 位置和能量；实际飞行后另带 `flight`（速度、转速等状态）。`DELIVERY_CONFIRM` 携带任务 ID、完成时刻；依可靠性级别等待 ACK。直达不足时从 S→R→L，R 保留原消息 ID 转发。
- **R ↔ S、S ↔ S：** 城市 ns-3 场景额外发送一跳广播 `HEARTBEAT`，含 `pos`、`energy`、`nearby`（近期实际听到的邻居）。这是邻居发现与成员判断，不代表 S–S 间每条消息都经路由转发；原有到 L 的上行心跳仍保留。
- **故障/选主：** `ELECTION_VOTE` 附候选者、旧主控 ID；`ACK` 附 `ack_for` 原消息 ID；`EMERGENCY`、`MEMBER_UPDATE`、`ROUTE_UPDATE` 也属于协议类型，具体发生取决于场景事件。
- **统一消息头：** `msg_id`、类型、源/目的、会话 ID、主控任期、任务/路由版本、序号、操作 ID、QoS、可靠性、创建时刻、期限、跳数/TTL、业务 `payload`、完整性字段。业务层做版本判断、去重与可靠性控制。

**实现细节：**ns-3 bridge 的模拟 MAC 帧填的是消息 ID 和用于占用时间的填充字节，SkyFleet 在同一进程保留完整 `Msg` 对象并按交付 ID 匹配；因此上表的业务负载是**仿真协议语义**，不是逐字节序列化进入 ns-3 Wi-Fi 帧的真实载荷。对外 HTTP 只负责操作/展示，不是无人机之间的通信链路。
