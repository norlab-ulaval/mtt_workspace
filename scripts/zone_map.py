#!/usr/bin/env python3
"""Runtime loader/query for a zone map built by build_zone_map.py.

No ROS dependency (importable standalone for testing); the conductor/monitor
import ZoneMap and call .distance_m(x, y) in the map frame. Pure numpy
bilinear lookup on a precomputed grid -- O(1), no live costmap, no planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class ZoneMap:
    distance_m: np.ndarray  # [ny, nx], meters to nearest occupied cell
    occupied: np.ndarray    # [ny, nx] bool
    origin_x: float
    origin_y: float
    resolution_m: float

    @classmethod
    def load(cls, path: Path) -> "ZoneMap":
        data = np.load(path)
        return cls(
            distance_m=data["distance_m"],
            occupied=data["occupied"],
            origin_x=float(data["origin_x"]),
            origin_y=float(data["origin_y"]),
            resolution_m=float(data["resolution_m"]),
        )

    def distance_to_boundary_m(self, x: float, y: float) -> float:
        """Bilinear-interpolated distance (m) to the nearest boundary/wall
        cell at map-frame (x, y). Returns 0.0 for any query OUTSIDE the grid
        extent -- fail-safe by construction: unmapped space is never treated
        as free, exactly like an out-of-bounds costmap query should behave."""
        ny, nx = self.distance_m.shape
        fx = (x - self.origin_x) / self.resolution_m - 0.5
        fy = (y - self.origin_y) / self.resolution_m - 0.5
        if fx < 0 or fy < 0 or fx > nx - 1 or fy > ny - 1:
            return 0.0
        x0, y0 = int(np.floor(fx)), int(np.floor(fy))
        x1, y1 = min(x0 + 1, nx - 1), min(y0 + 1, ny - 1)
        tx, ty = fx - x0, fy - y0
        d00 = self.distance_m[y0, x0]
        d10 = self.distance_m[y0, x1]
        d01 = self.distance_m[y1, x0]
        d11 = self.distance_m[y1, x1]
        d0 = d00 * (1 - tx) + d10 * tx
        d1 = d01 * (1 - tx) + d11 * tx
        return float(d0 * (1 - ty) + d1 * ty)

    def safe_direction_deg(self, x: float, y: float, step_m: float = 0.5) -> float:
        """Heading (degrees, atan2 convention, CCW from +x) that MOST INCREASES
        distance-to-wall from (x, y) -- i.e. the direction to move to get away
        from the nearest wall fastest. Central-difference gradient of the
        distance field, using step_m directly instead of grid cells so it's
        resolution-independent. Added 2026-07-30 after a real field incident:
        reversing blind (without this) can curve an articulated vehicle BACK
        toward the wall it just stopped near -- this gives the operator an
        actual heading to aim for instead of guessing."""
        d_px = self.distance_to_boundary_m(x + step_m, y)
        d_mx = self.distance_to_boundary_m(x - step_m, y)
        d_py = self.distance_to_boundary_m(x, y + step_m)
        d_my = self.distance_to_boundary_m(x, y - step_m)
        grad_x = (d_px - d_mx) / (2.0 * step_m)
        grad_y = (d_py - d_my) / (2.0 * step_m)
        if grad_x == 0.0 and grad_y == 0.0:
            return float("nan")
        return float(np.degrees(np.arctan2(grad_y, grad_x)))

    def path_clear(self, xy_points, margin_m: float) -> bool:
        """True iff every (x, y) in xy_points has distance_to_boundary_m >=
        margin_m. Intended use: forward-simulate a candidate segment's
        trajectory (e.g. via the same M0 kinematics already in
        analyze_ice_session.py) and check the whole predicted path before
        arming the segment -- proactive, not just reactive-at-the-wall."""
        return all(self.distance_to_boundary_m(x, y) >= margin_m for x, y in xy_points)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Quick CLI query against a saved zone map.")
    parser.add_argument("zone_map_npz", type=Path)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    args = parser.parse_args()
    zm = ZoneMap.load(args.zone_map_npz)
    d = zm.distance_to_boundary_m(args.x, args.y)
    print(f"distance to boundary at ({args.x}, {args.y}): {d:.2f} m")
