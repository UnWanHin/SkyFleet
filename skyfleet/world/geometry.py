"""几何纯函数。

遮挡判定的正确性完全依赖本模块，是单元测试的重点。
"""
from __future__ import annotations

import math

Point = tuple[float, float]
Rect = tuple[float, float, float, float]   # (x, y, w, h)


def _inside(p: Point, r: Rect) -> bool:
    x, y, w, h = r
    return x <= p[0] <= x + w and y <= p[1] <= y + h


def _orient(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))


def _segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    d1 = _orient(p3, p4, p1)
    d2 = _orient(p3, p4, p2)
    d3 = _orient(p1, p2, p3)
    d4 = _orient(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


def segment_intersects_rect(p1: Point, p2: Point, rect: Rect) -> bool:
    """线段与轴对齐矩形是否相交（含边界接触、含线段完全在矩形内）。"""
    if _inside(p1, rect) or _inside(p2, rect):
        return True
    x, y, w, h = rect
    corners = ((x, y), (x + w, y), (x + w, y + h), (x, y + h))
    for i in range(4):
        if _segments_intersect(p1, p2, corners[i], corners[(i + 1) % 4]):
            return True
    return False


def distance_2d(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance_3d(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)
