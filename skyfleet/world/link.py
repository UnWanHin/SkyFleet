"""可插拔链路模型。

这是 P1 的保险栓：只要替换本模块的实现，就能把 P0 的简易几何遮挡换成
非视距绕射与多径快衰落，而协议层零改动。

P0 的判定规则（对应报告 §3.1）：
  线段与楼块矩形相交 **且** 楼高 >= 两端较低者的飞行高度 -> 遮挡。
"飞行高度高于楼高"因此自然地避开了遮挡，这正是报告所要的
"链路质量随位置、建筑遮挡和飞行高度变化"。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from skyfleet.world.building import Building
from skyfleet.world.geometry import distance_3d, segment_intersects_rect
from skyfleet.world.node import Node


@dataclass(frozen=True)
class LinkQuality:
    """一次链路评估的结果。"""

    quality: float                 # [0, 1]
    obstructed: bool
    blocked_by: str | None
    trend: float                   # 滑动窗口斜率，供状态机区分临时遮挡与持续失联


@runtime_checkable
class LinkModel(Protocol):
    """链路模型接口。P1 的非视距/多径模型只需实现同一签名。"""

    def evaluate(self, a: Node, b: Node, t: float) -> LinkQuality: ...


class GeometricOcclusionLinkModel:
    """P0 实现：几何遮挡 + 距离衰减 + 滑动窗口趋势。"""

    def __init__(self, buildings: list[Building], max_range: float,
                 min_quality: float = 0.05, trend_window: int = 8,
                 obstruction_quality: float = 0.05) -> None:
        self.buildings = list(buildings)
        self.max_range = max_range
        self.min_quality = min_quality
        self.trend_window = trend_window
        self.obstruction_quality = obstruction_quality
        self._history: dict[tuple[str, str], deque[float]] = {}

    def evaluate(self, a: Node, b: Node, t: float) -> LinkQuality:
        blocked = self._first_blocker(a, b)
        distance = distance_3d(a.pos, b.pos)

        if blocked is not None:
            quality = self.obstruction_quality
        elif distance > self.max_range:
            quality = 0.0
        else:
            quality = max(self.min_quality, 1.0 - distance / self.max_range)

        history = self._history.setdefault(
            (a.node_id, b.node_id), deque(maxlen=self.trend_window))
        history.append(quality)
        trend = (history[-1] - history[0]) if len(history) > 1 else 0.0

        return LinkQuality(quality=quality, obstructed=blocked is not None,
                           blocked_by=blocked, trend=trend)

    def _first_blocker(self, a: Node, b: Node) -> str | None:
        lowest_altitude = min(a.pos[2], b.pos[2])
        start = (a.pos[0], a.pos[1])
        end = (b.pos[0], b.pos[1])
        for building in self.buildings:
            if building.height < lowest_altitude:
                continue
            if segment_intersects_rect(start, end, building.rect):
                return building.id
        return None


def build_link_model(config, buildings):
    """按配置构造链路模型。

    工厂放在这里而不是引擎里：引擎只声明"我要一个链路模型"，
    换成绕射模型或将来换成实测拟合模型都不需要改装配代码。
    """
    kind = (config.link_model or "geometric").lower()
    if kind == "geometric":
        return GeometricOcclusionLinkModel(buildings,
                                           max_range=config.max_link_range)
    if kind == "nlos":
        from skyfleet.world.link_nlos import NlosDiffractionLinkModel
        # 衰落用协议种子而非环境种子：城市布局要固定（共同随机数），
        # 但每次重复应当采样不同的衰落实现，否则方差被系统性低估。
        return NlosDiffractionLinkModel(buildings,
                                        max_range=config.max_link_range,
                                        seed=config.seed)
    raise ValueError(f"未知链路模型 {config.link_model!r}，"
                     f"可选 geometric / nlos")
