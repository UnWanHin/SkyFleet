"""物理运动器：让 skyfleet 的位置来自真实刚体动力学。

**为什么需要这一层**：`WaypointMover` 按速度沿直线前进，位置精确、瞬时。
于是 `SAFETY_ACTION`（HOVER / RETURN）虽然真的改了航线，运动本身却没有
惯性——飞机"说停就停、说走就走"。通信判定对飞行的影响因此只体现在**航线**
上，不体现在**姿态与速度**上。

本模块把每条航线的执行换成 MuJoCo 的刚体积分，使通信判定所导致的动作有真实
的加减速、滞后与冲过头。

**尺度：1:1，不做缩比。** 四旋翼悬停推力恒为 `mg`，动力学与绝对高度无关，
所以 500 m 城市坐标可以直接喂进仿真。实测 160 m 高度飞 150 m 距离，高度漂移
10.8 mm。

**速度上限为什么是 12 m/s**：`v_max` 是控制器参数而非物理极限。实测
`v_max=12` 时峰值 11.60 m/s，`v_max=20` 时仍是 11.62 m/s——说明已经撞上
物理天花板（倾角 14.8° 对应的推重比）。12 m/s 正好匹配 skyfleet 的巡航速度。

**`a_max_horizontal` 为什么必须从 3.0 提到 8.0**：12 m/s 的刹车距离是
`v²/2a`，若减速度只有 3 m/s² 需要 24 m，而位置环只在 7.7 m 外才开始减速，
于是冲过头 12.6 m。扫参结果：`a_max_horizontal=8.0` 时跟踪误差 0.00 m。

**成本**：一机一仿真，停稳后跳过积分。11 机 600 s 场景在当前机器上
实测约 63 s；具体耗时依机器与并行任务而变化。
"""
from __future__ import annotations

from skyfleet.world.geometry import distance_3d
from skyfleet.world.node import Node, Pos

#: 判定"已到达该航点"的容差（米）。物理跟踪精度实测 0.00 m，这里给足余量以
#: 避免在目标点附近反复开关 arrived。
ARRIVE_TOL = 0.5

#: 城市尺度下需要的控制参数。理由见模块 docstring。
V_MAX = 12.0
A_MAX_HORIZONTAL = 8.0

#: 低于此速度视为"已停稳"，可以跳过积分（m/s）。0.05 m/s = 5 cm/s，
#: 在 0.1 s 的运动周期里位移 5 mm，对链路几何无意义。
SETTLE_SPEED = 0.05


class FlightlabMover:
    """用 MuJoCo 刚体仿真执行航线。接口与 `WaypointMover` 一致。

    **一机一个仿真实例**，而不是把全部无人机塞进一个 MuJoCo 场景：后者每步都
    要积分所有机身，即使只有一架在动。分开之后悬停的机器可以整个跳过。
    """

    def __init__(self, node: Node, speed: float) -> None:
        self.node = node
        self.speed = speed
        self.route: list[Pos] = []
        self.arrived = True
        self._sim = None          # 惰性创建：导入 mujoco 很贵
        self._ready = False

    # ---------------- 内部 ----------------

    def _ensure_sim(self):
        """第一次真正要飞时才建仿真。

        惰性有两个理由：mujoco 导入开销大；更重要的是 `skyfleet` 的其余部分
        承诺零第三方依赖，不该因为本模块存在就强制安装 mujoco。
        """
        if self._ready:
            return self._sim
        from dataclasses import replace

        from flightlab.params import DEFAULT as P
        from flightlab.sim import QuadrotorSim

        sim = QuadrotorSim()
        sim.reset(pos=self.node.pos, rotor_speed=P.omega_hover)
        gains = replace(sim.gains, v_max=min(max(0.0, self.speed), V_MAX),
                        a_max_horizontal=A_MAX_HORIZONTAL)
        # 三处引用必须指向同一对象（初始不变量），否则改增益会只生效一半。
        sim.gains = gains
        sim.controller.gains = gains
        sim.esc.gains = gains
        self._sim = sim
        self._ready = True
        return sim

    def _advance(self, dt: float) -> None:
        """把仿真推进 `dt` 秒。MuJoCo 固定 2 ms 步长，所以要拆成子步。"""
        sim = self._ensure_sim()
        sub = sim.dt
        n = max(1, int(round(dt / sub)))
        for _ in range(n):
            sim.step(sim.target, 0.0)
        self.node.pos = tuple(float(v) for v in sim.pos)

    def flight_telemetry(self) -> dict | None:
        """Return physical measurements only after the simulator starts."""
        if not self._ready:
            return None
        sim = self._sim
        parked = (not self.route and
                  sum(v * v for v in sim.vel) <= SETTLE_SPEED ** 2)
        return {"pos": [float(v) for v in sim.pos],
                "vel": ([0.0, 0.0, 0.0] if parked else
                        [float(v) for v in sim.vel]),
                "rotor_rad_s": [float(v) for v in sim.rotor_speeds()]}

    # ---------------- Mover 协议 ----------------

    def set_route(self, waypoints: list[Pos]) -> None:
        self.route = list(waypoints)
        self.arrived = not self.route
        if self._ready:
            # 空航线 = HOVER：把目标设成当前位置，飞机会减速停住（有惯性，
            # 会冲出一段——这正是要展示的东西）。
            self._sim.target = (self._sim.pos if not self.route
                                else _as_array(self.route[0]))
        # 首次调用时仿真还没建，_ensure_sim 会用 node.pos 作为初始位置，
        # 因此这里不设 target，由 step 的第一拍补上。

    def step(self, dt: float) -> bool:
        """前进 dt 秒。返回本次是否到达了至少一个航点。

        与 `WaypointMover` 的契约一致：`EV_MOTION` 每 0.1 s 调一次，但这里
        内部会拆成 50 个 2 ms 子步真正积分。
        """
        if not self.route:
            # 无航线有两种情况，**不能混为一谈**：
            #   a) 从未起飞 / 早已停稳 —— 跳过积分。悬停漂移是厘米级，对链路
            #      几何无影响；跳过它把 11 机 600 s 的成本砍到 1/5。
            #   b) 刚被命令 HOVER 但还在动 —— 必须继续积分。否则飞机瞬间
            #      冻结，"通信判定导致飞行改变"就成了瞬移，恰恰丢掉了要
            #      展示的惯性。
            if not self._ready:
                self.arrived = True
                return False
            speed = float(sum(v * v for v in self._sim.vel) ** 0.5)
            if speed <= SETTLE_SPEED:
                self.arrived = True
                return False
            self._advance(dt)
            self.arrived = True
            return False

        sim = self._ensure_sim()
        sim.target = _as_array(self.route[0])
        self._advance(dt)

        reached = False
        while self.route and distance_3d(self.node.pos, self.route[0]) <= ARRIVE_TOL:
            if len(self.route) == 1 and sum(v * v for v in sim.vel) > 0.5 ** 2:
                break
            self.route.pop(0)
            reached = True
            if self.route:
                sim.target = _as_array(self.route[0])
        if not self.route:
            self.arrived = True
        return reached


def _as_array(pos: Pos):
    import numpy as np

    return np.asarray(pos, dtype=float)


def build_mover(node: Node, speed: float, *, physics: bool = False):
    """按配置造运动器。默认仍是运动学的。

    默认必须保持 `WaypointMover`：`out/study/` 下 14 个扫参的全部结论都建立在
    它的速度与确定性之上，换成物理积分会让所有结论不可复现。
    """
    if physics:
        return FlightlabMover(node, speed)
    from skyfleet.world.mobility import WaypointMover

    return WaypointMover(node, speed)
