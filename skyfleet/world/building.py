"""楼块与规则楼群生成。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

Rect = tuple[float, float, float, float]   # (x, y, w, h)


class _Rng(Protocol):
    def uniform(self, a: float, b: float) -> float: ...


@dataclass(frozen=True)
class Building:
    """轴对齐楼块：底面矩形 + 高度。"""

    id: str
    rect: Rect
    height: float


def grid_buildings(area: float, blocks: int, gap: float,
                   height_range: tuple[float, float],
                   rng: _Rng) -> list[Building]:
    """把 area x area 区域切成 blocks x blocks 的楼群网格。

    楼高由传入的 rng 抽样——必须用独立随机流，才能保证可复现。
    """
    cell = area / blocks
    size = cell - gap
    if size <= 0:
        raise ValueError("gap 过大，楼块尺寸非正")

    buildings: list[Building] = []
    for i in range(blocks):
        for j in range(blocks):
            x = i * cell + gap / 2.0
            y = j * cell + gap / 2.0
            height = rng.uniform(*height_range)
            buildings.append(Building(f"B{i}_{j}", (x, y, size, size), height))
    return buildings
