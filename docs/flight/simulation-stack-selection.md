# 仿真平台选型：Fedora 44 上的四旋翼 3D 仿真

> 结论先行：**用 MuJoCo 做物理与渲染，RotorPy 做交叉验证参考，不装 Gazebo。**
> 除非课程明确要求 PX4 固件在环（HITL/SITL），那时才用 podman 跑 Ubuntu 容器。

## 1. 为什么要专门写一份选型记录

"用 Gazebo"在 Linux 上听起来是默认答案，但它默认的是 **Ubuntu + ROS 2** 的
生态。本项目开发机是 Fedora 44，这个前提不成立。凭空假设会浪费几天时间在
编译依赖上，所以这里把每条路径的实际代价摊开写清。

所有结论都在本机实测过，不是凭印象。

## 2. 本机实测事实

| 项目 | 实测结果 |
|---|---|
| 发行版 | Fedora release 44 (Forty Four) |
| Python | 3.14.8（venv） |
| Gazebo / gz-sim | **dnf 无包**。列出的 `ignition-*` 是 Fedora CoreOS 的配置校验工具，与 Gazebo 无关 |
| ROS 2 | **dnf 无包**（`rosegarden4` 是音频软件） |
| 容器 | podman 可用，docker 未装 |
| GPU | NVIDIA RTX 5070 Ti Laptop，12227 MiB，驱动正常 |
| 资源 | 32 核 / 76 GB RAM / 851 GB 可用 |
| 已验证可用 | MuJoCo 3.17.0、RotorPy 3.0.0、ffmpeg 8.1.3 |

## 3. 候选方案对比

| 方案 | Fedora 可行性 | 四电机建模 | 3D 可视化 | 上手成本 | 适合本项目 |
|---|---|---|---|---|---|
| **MuJoCo** | ✅ pip 直装，无系统依赖 | 自己做（hinge 关节 + `F=k_f·ω²`） | ✅ 离屏渲染 + 交互查看器 | 低 | **★ 最佳** |
| Gazebo + PX4 SITL | ⚠️ 必须 podman 跑 Ubuntu 容器 | ✅ PX4 内置 | ✅ 最好 | **高**（镜像 5–10 GB，GUI 转发在 Wayland 上易出问题） | 仅当需要固件在环 |
| Isaac Sim | ⚠️ 官方支持 Ubuntu，Fedora 需自行解决 | ✅ | ✅ 最强 | **很高**（Omniverse 约 20 GB） | 不必要 |
| PyBullet | ⚠️ wheel 只到 cp311，Python 3.14 无轮子 | 自己做 | ✅ | 中（需另建 3.11 环境） | 弱于 MuJoCo |
| RotorPy 单独 | ✅ pip 直装 | ✅ 内置 | ❌ 只有 matplotlib | 低 | 做参考，不做主引擎 |
| AirSim / Flightmare | ❌ 已归档 / 依赖 ROS+Unity | — | — | — | 排除 |

## 4. 为什么选 MuJoCo

1. **零系统依赖**。`pip install mujoco` 就完事，不需要 ROS、不需要容器、
   不需要编译。这直接消除了 Fedora 上最大的风险源。
2. **真刚体动力学**。6 自由度、接触、关节约束都由成熟引擎处理。本项目只写
   真正需要自己写的两块：旋翼气动与控制律——这正是 Gazebo + PX4 的分工方式
   （引擎管物理，控制器管飞行）。
3. **3D 可视化两条路都有**。离屏渲染（`MUJOCO_GL=glfw`）能出 1280×720 视频
   和报告插图；交互查看器可以边调增益边看姿态。
4. **分工边界清晰**。引擎**没有**旋翼气动（这是引擎边界，不是本项目简化），
   所以推力按 `F=k_f·ω²` 由转子实际角速度算出，用 `mj_applyFT` 作用在转子
   位置。这条边界写进了代码注释，不会让人误以为是完整气动模型。

## 5. 与 RotorPy 的交叉验证结果

自己写动力学最大的风险不是算错，而是"和公认实现用的不是同一个模型"，然后
所有结论都建在孤立假设上。所以做了可证伪的对比
（`tests/flight/test_rotorpy_crossvalidation.py`，4 条测试常驻）。

把 MJCF 的质量、转动惯量、`k_f`、`k_m` 全部对齐 RotorPy 的 `crazyflie_params`
后，给两边相同的马达转速指令：

| 配置 | 1 秒后高度 |
|---|---|
| RotorPy（`aero=False`，转子从 `w_cmd` 起转） | **2.0301 m** |
| MuJoCo（转子从 `w_cmd` 起转） | **2.0293 m** |
| 差异 | **0.8 mm** |

**刚体动力学 + 气动模型一致到毫米级。** 而两边都从 0 起转时差 0.4 m——
这个差异全部来自**电机起动动态**：本项目用转动惯量 + 力矩驱动 + 二次阻力，
RotorPy 用一阶转速滞后。这是电机模型不同，不是物理不同。

这条差异被单独写成一条测试固定下来，防止将来有人看到 0.4 m 就去"修"刚体
动力学——修错地方比不修更糟。

### 顺带发现的公开参数缺陷

RotorPy 的 crazyflie 惯量 `Ixx=Iyy=1.43e-5, Izz=2.89e-5` 满足不了刚体三角
不等式 `Ixx+Iyy >= Izz`（2.86e-5 < 2.89e-5，约差 1%）。MuJoCo 直接拒绝加载，
要靠 `balanceinertia="true"` 把惯量微调进可行域。公开参数里存在这种物理
不一致，引用时需要知道。

## 6. 如果确实需要 Gazebo

只有当课程/验收明确要求 **PX4 固件在环**（跑真实飞控代码、验证 MAVLink、
验证 EKF）时才值得。那种情况下 MuJoCo 替不了，因为要的是固件而不是物理。

路径与代价：

```bash
# 1) 拉 Ubuntu 镜像（约 1.5 GB，加上 PX4 依赖后 5-10 GB）
podman pull ubuntu:24.04

# 2) 容器内装 Gazebo Harmonic + PX4
podman run -it --rm \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  ubuntu:24.04 bash

# 3) 容器内
apt update && apt install -y curl git make
# 按 PX4 官方 dev_setup.sh 安装，然后：
#   make px4_sitl gz_x500
```

已知代价：Wayland 下 GUI 转发需要额外配置（XWayland 或 `QT_QPA_PLATFORM`），
磁盘占用大，PX4 首次编译约 20–40 分钟，且需要接受 Ubuntu 侧的版本锁定。

**判断标准**：如果验收只要求"看到四旋翼飞起来、姿态正确、控制可调"，
MuJoCo 完全够，没必要付这个代价。

## 7. 本项目的最终技术栈

| 层 | 选用 | 理由 |
|---|---|---|
| 刚体物理 + 积分 | MuJoCo 3.17（RK4，dt=2 ms） | 成熟引擎，零系统依赖 |
| 旋翼气动 | 自写 `F=k_f·ω²`、`τ=k_m·ω²` | 引擎无此功能；与 RotorPy 同构，已验证 |
| 电机 | 自写带转动惯量的 hinge 关节 + 前馈转速环 | 能看到真实起动滞后 |
| 控制 | 自写串级：位置 PD → 几何 SE3 → 混控 → 转速环 | 每级可单独测、单独调 |
| 参考/验证 | RotorPy 3.0.0 | 交叉验证到 0.8 mm |
| 3D 渲染 | MuJoCo 离屏渲染 + ffmpeg | 出视频与报告插图 |
| 通信层 | 已有的纯 Python 事件仿真（`skyfleet/`） | 与本层解耦 |

## 8. 引用与出处

- MuJoCo：<https://mujoco.readthedocs.io/>
- RotorPy：<https://github.com/spencerfolk/rotorpy>
- 几何 SE3 控制律：T. Lee, M. Leok, N. H. McClamroch,
  *Geometric Tracking Control of a Quadrotor UAV on SE(3)*, CDC 2010.
- PX4 SITL：<https://docs.px4.io/main/en/simulation/>
