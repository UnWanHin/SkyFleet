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


def plan_passage(destination, node_ids, buildings, *, spacing=3.0,
                 clearance=0.4, drone_radius=0.3, through=None):
    """Queue south of a northbound corridor, then regroup past its exit."""
    if not node_ids or spacing <= 0 or clearance < 0 or drone_radius <= 0:
        raise ValueError("invalid formation parameters")
    x, y, z = destination
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("destination must be finite")
    cx, cy, cz = through if through is not None else (x, y, z)
    if not all(map(math.isfinite, (cx, cy, cz))) or cz != z:
        raise ValueError("passage center must have the destination altitude")
    margin = drone_radius + clearance
    blockers = [b for b in buildings
                if b.base_height - drone_radius <= z <= b.height + drone_radius]
    pairs = []
    for left in blockers:
        lx, ly, lw, lh = left.rect
        if lx + lw > cx - margin or (through is not None and not ly <= cy <= ly + lh):
            continue
        for right in blockers:
            rx, ry, _, rh = right.rect
            if rx < cx + margin or (through is not None and not ry <= cy <= ry + rh):
                continue
            start, end = max(ly, ry), min(ly + lh, ry + rh)
            if end > start and rx - lx - lw >= 2 * margin:
                pairs.append((rx - lx - lw, start, end))
    if not pairs:
        raise ValueError("passage center needs two flanking buildings")
    _, start, end = min(pairs)
    if y <= end + margin:
        raise ValueError("destination must be beyond passage exit")
    ids = sorted(set(node_ids))
    cols = math.ceil(math.sqrt(len(ids)))
    routes = {}
    for index, node_id in enumerate(ids):
        row, col = divmod(index, cols)
        final = (x + (col - (cols - 1) / 2) * spacing,
                 y + (row - (cols - 1) / 2) * spacing, z)
        points = [(cx, start - margin - (index + 1) * spacing, z),
                  (cx, start - margin, z), (cx, (start + end) / 2, z),
                  (cx, end + margin, z), final]
        _validate_passage_route(points, buildings, z, drone_radius)
        routes[node_id] = points
    return routes


def adaptive_route(position, destination, buildings, peers, *, node_id,
                   members, spacing=3.0, clearance=0.4, drone_radius=0.3):
    """Re-evaluate a corridor at each tick; an unsafe path yields HOLD.

    Waypoints are geometric guidance, not a hard real-time collision guarantee
    for an inertial mover. No new route is published across an occupied point.
    """
    x, y, z = destination
    members = sorted(set(members))
    rank = members.index(node_id)
    cols = math.ceil(math.sqrt(len(members)))
    row, col = divmod(rank, cols)
    matrix_target = (x + (col - (cols - 1) / 2) * spacing,
                     y + (row - (cols - 1) / 2) * spacing, z)
    radius = 2 * drone_radius + clearance
    for name, peer in peers.items():
        if math.dist(position, peer) < max(spacing, radius) and name < node_id:
            return [], "HOLD"
    try:
        _validate_passage_route([position, matrix_target], buildings, z,
                                drone_radius + clearance)
        return [matrix_target], "MATRIX"
    except ValueError:
        pass
    if position[2] > z + 2 and all(math.dist(matrix_target, peer) >= radius
                                    for peer in peers.values()):
        # At cruising altitude, route above roofs before descending in a
        # verified clear destination column; never descend through a roof.
        cruise_z = max(position[2], z,
                       *(b.height + drone_radius + clearance + 1
                         for b in buildings))
        cruise = [(position[0], position[1], cruise_z),
                  (matrix_target[0], matrix_target[1], cruise_z),
                  matrix_target]
        try:
            _validate_passage_route([position, *cruise], buildings, z,
                                    drone_radius)
        except ValueError:
            pass
        else:
            return cruise, "MATRIX"
    for left in buildings:
        lx, ly, lw, lh = left.rect
        if lx + lw >= x:
            continue
        for right in buildings:
            rx, ry, _, rh = right.rect
            start, end = max(ly, ry), min(ly + lh, ry + rh)
            if (rx <= x or end <= start or
                    rx - lx - lw < 2 * (drone_radius + clearance) or
                    not (ly <= (start + end) / 2 <= ly + lh)):
                continue
            # Once beyond the corridor, matrix geometry is checked anew above.
            if position[1] > end + drone_radius + clearance:
                continue
            enter = (x, start - drone_radius - clearance, z)
            exit_point = (x, end + drone_radius + clearance, z)
            route = [point for point in
                     (enter, (x, (start + end) / 2, z), exit_point)
                     if point[1] > position[1] + 1e-6]
            route.append(matrix_target)
            try:
                _validate_passage_route([position, *route], buildings, z,
                                        drone_radius)
            except ValueError:
                continue
            if any(math.dist(point, peer) < radius for point in route[:3]
                   for peer in peers.values()):
                return [], "HOLD"
            return route, "COLUMN"
    return [], "HOLD"


def _validate_passage_route(points, buildings, altitude, radius):
    """Check sampled axis-aligned route segments against expanded footprints."""
    step = max(radius, 0.25)
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        samples = max(1, math.ceil(distance / step))
        for index in range(samples + 1):
            ratio = index / samples
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            z = start[2] + (end[2] - start[2]) * ratio
            for building in buildings:
                if not building.base_height - radius <= z <= building.height + radius:
                    continue
                bx, by, bw, bd = building.rect
                if bx - radius <= x <= bx + bw + radius and \
                        by - radius <= y <= by + bd + radius:
                    raise ValueError(f"route intersects building {building.id}")


def distance_2d(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance_3d(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)
