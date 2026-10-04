"""P1 禁飞区约束（规格 §9 的 `GeofenceProvider`）。

报告 §2.2.6 把禁飞区列为优先级最高的本地安全动作，并强调它**不能依赖
主节点**：与主节点失联时仍必须遵守。因此判定放在本地、按位置直接求值，
不经过任何消息往返。

约束是"矩形底面 + 高度区间"：同一片空域可能低空禁飞而高空放行
（这正是城市无人机走廊的常见形态），只判平面会把可飞高度一起禁掉。
"""
from __future__ import annotations

from dataclasses import dataclass

from skyfleet.world.building import Rect


@dataclass(frozen=True)
class Geofence:
    """禁飞区：平面矩形 + 高度区间。"""

    id: str
    rect: Rect
    z_min: float = float("-inf")
    z_max: float = float("inf")

    def contains(self, pos: Pos) -> bool:
        x, y, z = pos
        x0, y0, w, h = self.rect
        if not (x0 <= x <= x0 + w and y0 <= y <= y0 + h):
            return False
        return self.z_min <= z <= self.z_max


def build_geofences(config) -> list[Geofence]:
    return [
        Geofence(
            id=str(item["id"]),
            rect=(float(item["x"]), float(item["y"]),
                  float(item["w"]), float(item["h"])),
            z_min=float(item.get("z_min", float("-inf"))),
            z_max=float(item.get("z_max", float("inf"))),
        )
        for item in getattr(config, "geofences", ()) or ()
    ]


def breach(pos: Pos, fences: list[Geofence]) -> str | None:
    """返回最先命中的禁飞区 id；未命中返回 None。"""
    for fence in fences:
        if fence.contains(pos):
            return fence.id
    return None
