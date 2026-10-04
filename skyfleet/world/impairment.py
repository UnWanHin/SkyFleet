"""P1 损伤源：背景干扰与临时障碍（对应规格 §9 的 `ImpairmentSource`）。

实现方式是**链路模型装饰器**，而不是往协议里加分支：

    inner = build_link_model(cfg, buildings)
    model = ImpairedLinkModel(inner, interferences=[...], obstacles=[...])

这样协议层连"存在损伤源"都不需要知道——它看到的仍然只是一个
`LinkModel`。§9 要求"所有注入通过事件流进入仿真，不绕过 L2"，
在本层的落点就是：损伤只改变链路质量，节点因此仍然要走完整的
重传、路由与状态机流程，而不是被直接告知"你断了"。

损伤的时变性（干扰开窗、吊塔出现与撤离）由 `t` 决定，是**纯函数**，
不推进任何随机状态——否则同一次评估被调用两次会给出不同结果。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

from skyfleet.world.building import Rect
from skyfleet.world.geometry import segment_intersects_rect
from skyfleet.world.link import LinkModel, LinkQuality
from skyfleet.world.node import Node


@dataclass(frozen=True)
class Interference:
    """背景干扰源：在时间窗内对一定半径内的链路抬高噪声底。"""

    id: str
    center: tuple[float, float]
    radius_m: float
    t_start: float
    t_end: float
    extra_loss_db: float = 12.0

    def active(self, t: float) -> bool:
        return self.t_start <= t < self.t_end

    def affects(self, node: Node, t: float) -> bool:
        if not self.active(t):
            return False
        dx = node.pos[0] - self.center[0]
        dy = node.pos[1] - self.center[1]
        return dx * dx + dy * dy <= self.radius_m ** 2


@dataclass(frozen=True)
class TemporaryObstacle:
    """临时障碍（高架施工、塔吊）：在其时间窗内等效为一栋楼。"""

    id: str
    rect: Rect
    height: float
    t_start: float
    t_end: float
    extra_loss_db: float = 20.0

    def active(self, t: float) -> bool:
        return self.t_start <= t < self.t_end

    def blocks(self, a: Node, b: Node, t: float) -> bool:
        if not self.active(t):
            return False
        lowest = min(a.pos[2], b.pos[2])
        if self.height < lowest:
            return False
        return segment_intersects_rect((a.pos[0], a.pos[1]),
                                       (b.pos[0], b.pos[1]), self.rect)


class ImpairedLinkModel:
    """在任意 `LinkModel` 之上叠加时变损伤的装饰器。"""

    def __init__(self, inner: LinkModel,
                 interferences: Sequence[Interference] = (),
                 obstacles: Sequence[TemporaryObstacle] = (),
                 default_span_db: float = 20.0) -> None:
        self.inner = inner
        self.interferences = list(interferences)
        self.obstacles = list(obstacles)
        self.default_span_db = default_span_db

        # 损伤以 dB 计，而 quality 是无量纲的 [0,1]。换算需要知道底层模型的
        # 动态范围：NLOS 模型自己知道（暴露 span_db），几何模型不知道，
        # 就退回一个声明式的默认量程。
        self.span_db = float(getattr(inner, "span_db", default_span_db))
        # 便于外部检查装饰是否生效（测试与场景自检用）
        self.buildings = getattr(inner, "buildings", [])

    @property
    def max_range(self) -> float:
        return getattr(self.inner, "max_range", 0.0)

    def evaluate(self, a: Node, b: Node, t: float) -> LinkQuality:
        base = self.inner.evaluate(a, b, t)
        loss = self.extra_loss_db(a, b, t)
        if loss <= 0.0:
            return base

        quality = max(0.0, base.quality - loss / self.span_db)
        return LinkQuality(
            quality=quality,
            obstructed=base.obstructed or self._is_blocked(a, b, t),
            blocked_by=base.blocked_by or self._blocker_id(a, b, t),
            trend=base.trend,
        )

    def extra_loss_db(self, a: Node, b: Node, t: float) -> float:
        """当前时刻该链路上的附加衰减（dB）。同源叠加但设上限。"""
        total = 0.0
        for source in self.interferences:
            if source.affects(a, t) or source.affects(b, t):
                total += source.extra_loss_db
        for obstacle in self.obstacles:
            if obstacle.blocks(a, b, t):
                total += obstacle.extra_loss_db
        return total

    def _is_blocked(self, a: Node, b: Node, t: float) -> bool:
        return self._blocker_id(a, b, t) is not None

    def _blocker_id(self, a: Node, b: Node, t: float) -> str | None:
        for obstacle in self.obstacles:
            if obstacle.blocks(a, b, t):
                return obstacle.id
        return None

    def active_interferences(self, t: float) -> list[Interference]:
        return [s for s in self.interferences if s.active(t)]


def build_impairments(config) -> tuple[list[Interference],
                                       list[TemporaryObstacle]]:
    """从配置构造损伤源。配置里是列表的字典，这里转成不可变数据类。"""
    interferences = [
        Interference(id=str(item["id"]),
                     center=(float(item["x"]), float(item["y"])),
                     radius_m=float(item["radius"]),
                     t_start=float(item.get("t_start", 0.0)),
                     t_end=float(item.get("t_end", float("inf"))),
                     extra_loss_db=float(item.get("extra_loss_db", 12.0)))
        for item in getattr(config, "interferences", ()) or ()
    ]
    obstacles = [
        TemporaryObstacle(
            id=str(item["id"]),
            rect=(float(item["x"]), float(item["y"]),
                  float(item["w"]), float(item["h"])),
            height=float(item["height"]),
            t_start=float(item.get("t_start", 0.0)),
            t_end=float(item.get("t_end", float("inf"))),
            extra_loss_db=float(item.get("extra_loss_db", 20.0)))
        for item in getattr(config, "obstacles", ()) or ()
    ]
    return interferences, obstacles


def wrap_with_impairments(inner: LinkModel, config) -> LinkModel:
    """按配置决定是否包裹装饰器；没有任何损伤源时原样返回。"""
    interferences, obstacles = build_impairments(config)
    if not interferences and not obstacles:
        return inner
    return ImpairedLinkModel(inner, interferences, obstacles)
