"""Import a projected-metre GeoJSON building footprint subset.

Only axis-aligned, hole-free rectangles are accepted. Geographic lon/lat,
arbitrary polygons and multipolygons require explicit preprocessing; silently
using a bounding box would invent radio obstructions outside real buildings.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from skyfleet.world.building import Building


def load_city_terrain(path: str | Path) -> dict | None:
    """Optional visual-only landscape metadata carried with city geometry."""
    root = json.loads(Path(path).read_text(encoding="utf-8"))
    terrain = root.get("skyfleet_terrain")
    if terrain is None:
        return None
    if (not isinstance(terrain, dict) or terrain.get("type") != "visual-only" or
            not isinstance(terrain.get("bounds"), list) or len(terrain["bounds"]) != 4 or
            not all(isinstance(v, (int, float)) and math.isfinite(v)
                    for v in terrain["bounds"])):
        raise ValueError("skyfleet_terrain requires visual-only metric bounds")
    return terrain


def load_buildings_geojson(path: str | Path) -> list[Building]:
    root = json.loads(Path(path).read_text(encoding="utf-8"))
    if root.get("type") != "FeatureCollection" or not isinstance(root.get("features"), list):
        raise ValueError("expected GeoJSON FeatureCollection")
    # RFC 7946 coordinates are WGS84 degrees unless explicitly supplied in
    # a separate local projected-metre export; require a caller-side declaration.
    if root.get("skyfleet_units") != "metres":
        raise ValueError("GeoJSON requires skyfleet_units='metres' (project coordinates first)")
    result = []
    for index, feature in enumerate(root["features"]):
        geom = feature.get("geometry") or {}
        rings = geom.get("coordinates")
        if geom.get("type") != "Polygon" or not isinstance(rings, list) or len(rings) != 1:
            raise ValueError(f"feature {index}: only hole-free Polygon rectangles supported")
        ring = rings[0]
        if not isinstance(ring, list) or len(ring) != 5 or ring[0] != ring[-1]:
            raise ValueError(f"feature {index}: expected closed rectangle ring")
        if any(not isinstance(p, list) or len(p) != 2 or
               not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p)
               for p in ring):
            raise ValueError(f"feature {index}: invalid projected coordinates")
        xs, ys = {p[0] for p in ring}, {p[1] for p in ring}
        if len(xs) != 2 or len(ys) != 2 or set(map(tuple, ring[:-1])) != {
                (x, y) for x in xs for y in ys}:
            raise ValueError(f"feature {index}: non-rectangular polygon")
        properties = feature.get("properties") or {}
        height = properties.get("height")
        base_height = properties.get("base_height", 0)
        if not isinstance(height, (int, float)) or not math.isfinite(height) or height <= 0:
            raise ValueError(f"feature {index}: positive height in metres required")
        if (not isinstance(base_height, (int, float)) or not math.isfinite(base_height)
                or not 0 <= base_height < height):
            raise ValueError(f"feature {index}: base_height must be below height")
        x, y = min(xs), min(ys)
        result.append(Building(str(feature.get("id", index)),
                               (float(x), float(y), float(max(xs) - x),
                                float(max(ys) - y)), float(height), float(base_height)))
    return result
