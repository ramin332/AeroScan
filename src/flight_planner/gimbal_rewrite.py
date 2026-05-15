"""Rewrite imported DJI waypoints for NEN-2767 perpendicular inspection.

Takes AutoExplore-generated waypoints (photogrammetry rosette, pitch ~-19°)
and rewrites gimbal pitch/yaw so the camera faces each waypoint's nearest
outward facade head-on. This is the MVP deliverable — we keep DJI's
flight-tested trajectory and waypoint spacing, and only change where the
camera points.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Sequence

import numpy as np

from .models import (
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    Facade,
    Waypoint,
)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def _pick_facade_for_waypoint(
    wp_xyz: np.ndarray,
    facades: Sequence[Facade],
    max_distance_m: float,
    wall_distance_bonus: float = 0.5,
) -> tuple[int, float] | None:
    """Find the best outward-facing facade for this waypoint.

    Returns (facade_index, 3d_distance_to_center) or None if no facade is
    within ``max_distance_m`` in 3D on its outward side.

    Gate: the WP must be on the facet's outward side (``dot(normal,
    WP - center) > 0``). Sort metric: 3D distance from the WP to the
    facet's centroid (lower wins).

    Walls (label starts with ``wall_``) get their distance multiplied by
    ``wall_distance_bonus`` (default 0.5). This biases the picker toward
    walls when both wall and non-wall (roof/tilted) are candidates at
    similar 3D range — e.g., a wall at 8 m beats a roof at 4.5 m, but
    a roof at 3 m beats a wall at 8 m.

    History: this used to gate AND sort on signed perpendicular standoff
    (the projection of WP-to-center onto the normal). That had a hidden
    failure mode — a small facet 30 m away laterally could have a tiny
    perp standoff (because the WP happens to be 'in front of' its
    infinite plane) and beat a much closer facet whose plane is slightly
    further. Verified on Mijande/flight0016: 46% of WPs were picking
    facets > 20 m away in 3D, even with ``max_distance_m=60``, because
    the picker only bounded perp standoff. Switching the sort metric to
    3D distance makes the picker prefer geometrically near facets, which
    is what 'aim at the building's nearby surface' actually means.
    """
    best: tuple[int, float] | None = None
    best_weighted = float("inf")

    for i, f in enumerate(facades):
        n = _unit(np.asarray(f.normal, dtype=np.float64))
        c = np.asarray(f.center, dtype=np.float64)
        signed = float(np.dot(n, wp_xyz - c))
        if signed <= 0:
            continue  # WP is on the inward side or coplanar — facet not reachable
        dist3d = float(np.linalg.norm(c - wp_xyz))
        if dist3d > max_distance_m:
            continue
        is_wall = (f.label or "").startswith("wall_")
        weighted = dist3d * (wall_distance_bonus if is_wall else 1.0)
        if weighted < best_weighted:
            best_weighted = weighted
            best = (i, dist3d)

    return best


def rewrite_gimbals_perpendicular(
    waypoints: list[Waypoint],
    facades: list[Facade],
    max_distance_m: float = 60.0,
    pitch_margin_deg: float = 2.0,
    preserve_heading: bool = True,
    wall_distance_bonus: float = 0.5,
) -> list[Waypoint]:
    """Rewrite ``gimbal_pitch_deg`` / ``gimbal_yaw_deg`` so each waypoint
    photographs the nearest facade head-on.

    Parameters
    ----------
    waypoints
        Imported DJI waypoints in local ENU (x=E, y=N, z=Up).
    facades
        Building facades; ``facade.normal`` must be outward-facing.
    max_distance_m
        Skip facades further than this from the waypoint. Waypoints with no
        qualifying facade keep their original gimbal pose.
    pitch_margin_deg
        Margin from hardware limits (Matrice 4E: -90°..+35°).
    preserve_heading
        If True (default), only update the gimbal and leave aircraft heading
        alone — safer, since DJI's waypoint heading drives turn smoothing.
        Gimbal yaw is then stored absolutely (relative to north, not aircraft).

    Returns a new list of Waypoints; input list is not mutated.
    """
    pitch_min = GIMBAL_TILT_MIN_DEG + pitch_margin_deg
    pitch_max = GIMBAL_TILT_MAX_DEG - pitch_margin_deg

    out: list[Waypoint] = []
    for wp in waypoints:
        pos = np.array([wp.x, wp.y, wp.z], dtype=np.float64)
        pick = _pick_facade_for_waypoint(pos, facades, max_distance_m, wall_distance_bonus=wall_distance_bonus)
        if pick is None:
            out.append(replace(wp, facade_index=wp.facade_index))
            continue

        idx, _ = pick
        facade = facades[idx]
        center = np.asarray(facade.center, dtype=np.float64)

        # Camera look direction = WP → facade center.
        look = center - pos
        norm = float(np.linalg.norm(look))
        if norm < 1e-6:
            out.append(replace(wp, facade_index=wp.facade_index))
            continue
        look = look / norm
        # Yaw: bearing from north, clockwise. ENU x=East, y=North.
        yaw_rad = math.atan2(look[0], look[1])
        yaw_deg = (math.degrees(yaw_rad) + 360.0) % 360.0
        if yaw_deg > 180.0:
            yaw_deg -= 360.0

        # Pitch: angle from horizontal. Positive = up, negative = down.
        horiz = math.hypot(look[0], look[1])
        pitch_deg = math.degrees(math.atan2(look[2], horiz))
        pitch_deg = max(pitch_min, min(pitch_max, pitch_deg))

        new_wp = replace(
            wp,
            gimbal_pitch_deg=float(pitch_deg),
            gimbal_yaw_deg=float(yaw_deg),
            heading_deg=wp.heading_deg if preserve_heading else float(yaw_deg),
            facade_index=idx,
            # Drop the photogrammetry rosette action group — we want a single
            # head-on photo per waypoint, not the 5-pose smart oblique.
            actions=[a for a in wp.actions if getattr(a, "action_type", None)],
        )
        out.append(new_wp)

    return out
