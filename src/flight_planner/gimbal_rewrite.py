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
    prefer_walls: bool = True,
) -> tuple[int, float] | None:
    """Find the closest outward-facing facade to this waypoint.

    Returns (facade_index, signed_perp_distance) or None if no facade is
    within ``max_distance_m`` on its outward side.

    With ``prefer_walls=True`` (default), walls are searched first: if any
    wall is reachable, the closest wall wins, even when a roof/tilted facet
    is geometrically closer. This matches NEN-2767 inspection intent —
    inspectors care about walls; roofs/soffits are secondary. Without this
    bias, on a Mijande-style building (~5× more roof facets than wall
    facets), a large fraction of WPs end up aimed straight down at roofs
    or up at soffits instead of perpendicular to the wall they're flying
    past. Falls back to all facets if no wall is in range — preserves
    coverage of non-wall features.
    """
    if prefer_walls:
        wall_facades = [(i, f) for i, f in enumerate(facades)
                        if (f.label or "").startswith("wall_")]
        if wall_facades:
            pick = _closest_outward(wp_xyz, wall_facades, max_distance_m)
            if pick is not None:
                return pick
    return _closest_outward(
        wp_xyz, list(enumerate(facades)), max_distance_m,
    )


def _closest_outward(
    wp_xyz: np.ndarray,
    indexed_facades: Sequence[tuple[int, Facade]],
    max_distance_m: float,
) -> tuple[int, float] | None:
    """Pick the facade with smallest positive (outward) perpendicular distance."""
    best: tuple[int, float] | None = None
    best_abs = float("inf")

    for i, f in indexed_facades:
        n = _unit(np.asarray(f.normal, dtype=np.float64))
        c = np.asarray(f.center, dtype=np.float64)
        signed = float(np.dot(n, wp_xyz - c))
        if signed <= 0 or signed > max_distance_m:
            continue
        if signed < best_abs:
            best_abs = signed
            best = (i, signed)

    return best


def rewrite_gimbals_perpendicular(
    waypoints: list[Waypoint],
    facades: list[Facade],
    max_distance_m: float = 60.0,
    pitch_margin_deg: float = 2.0,
    preserve_heading: bool = True,
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
        pick = _pick_facade_for_waypoint(pos, facades, max_distance_m)
        if pick is None:
            out.append(replace(wp, facade_index=wp.facade_index))
            continue

        idx, _dist = pick
        facade = facades[idx]
        n = _unit(np.asarray(facade.normal, dtype=np.float64))

        # Camera look direction = -outward_normal (into the facade).
        look = -n
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
