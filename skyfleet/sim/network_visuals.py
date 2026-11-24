"""Read-only, bounded visualization of the ns-3 log-distance and building shadow model.

The grid estimates modeled received power (not PHY RSSI or delivery certainty).
No radio model state is touched by these functions.
"""
from __future__ import annotations

import math


def _blocked(a, b, building):
    x, y, w, d = building.rect
    enter, leave = 0., 1.
    for origin, delta, low, high in ((a[0], b[0]-a[0], x, x+w),
                                     (a[1], b[1]-a[1], y, y+d)):
        if delta == 0:
            if not low <= origin <= high:
                return False
            continue
        lo, hi = (low-origin)/delta, (high-origin)/delta
        enter, leave = max(enter, min(lo, hi)), min(leave, max(lo, hi))
        if enter > leave:
            return False
    z0 = a[2] + (b[2]-a[2])*enter
    z1 = a[2] + (b[2]-a[2])*leave
    return min(z0, z1) <= building.height and max(z0, z1) >= building.base_height


def coverage_grid(origin, buildings, bounds, *, building_loss_db=0.,
                  columns=24, rows=20, threshold_dbm=-75.,
                  tx_power_dbm=25., path_loss_exponent=2.):
    """Estimate strong/weak/obstructed points in the source altitude plane.

    ns-3 defaults: 25 dBm Tx, LogDistance exponent 2, 46.6777 dB
    reference loss at 1 m; add one BuildingShadowLoss attenuation.
    """
    if not 0 < columns * rows <= 480:
        raise ValueError("coverage grid exceeds visual budget")
    xmin, ymin, xmax, ymax = bounds
    dx, dy = (xmax-xmin)/columns, (ymax-ymin)/rows
    if dx <= 0 or dy <= 0:
        raise ValueError("invalid coverage bounds")
    cells = []
    for row in range(rows):
        for col in range(columns):
            target = (xmin + (col+.5)*dx, ymin + (row+.5)*dy, origin[2])
            obstructed = any(_blocked(origin, target, block) for block in buildings)
            dbm = tx_power_dbm - 46.6777 - 10.*path_loss_exponent*math.log10(
                max(1., math.dist(origin, target)))
            if obstructed:
                dbm -= building_loss_db
            status = "shadow" if obstructed else "strong" if dbm >= threshold_dbm else "weak"
            cells.append([col, row, status])
    return {"origin": list(origin), "bounds": list(bounds), "dx": dx,
            "dy": dy, "cells": cells, "threshold_dbm": threshold_dbm,
            "provenance": "modeled_ns3_log_distance_and_building_shadow"}
