"""航点式移动。

报告 §1 明确排除完整飞控与真实飞行动力学，因此这里只有
"按速度沿航点直线前进"这一个规则。
"""
from __future__ import annotations

from skyfleet.world.geometry import distance_3d
from skyfleet.world.node import Node, Pos


class WaypointMover:
    """沿航点列表推进。不含转向动力学、不含避障（P1 再加）。"""

    def __init__(self, node: Node, speed: float) -> None:
        self.node = node
        self.speed = speed
        self.route: list[Pos] = []
        self.arrived = False

    def set_route(self, waypoints: list[Pos]) -> None:
        self.route = list(waypoints)
        self.arrived = not self.route

    def step(self, dt: float) -> bool:
        """前进 dt 秒。返回本次是否到达了至少一个航点。"""
        if self.arrived or not self.route:
            self.arrived = True
            return False

        remaining = self.speed * dt
        reached = False
        while remaining > 0.0 and self.route:
            target = self.route[0]
            dist = distance_3d(self.node.pos, target)
            if dist <= remaining:
                self.node.pos = target
                remaining -= dist
                self.route.pop(0)
                reached = True
                if not self.route:
                    self.arrived = True
            else:
                ratio = remaining / dist
                self.node.pos = tuple(
                    p + (q - p) * ratio
                    for p, q in zip(self.node.pos, target))
                remaining = 0.0
        return reached
