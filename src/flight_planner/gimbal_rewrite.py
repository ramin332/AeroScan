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

    Returns (facade_index, signed_perp_distance) or None if no facade is
    within ``max_distance_m`` on its outward side.

    Walls (label starts with ``wall_``) get their selection distance
    multiplied by ``wall_distance_bonus`` (default 0.5). This biases the
    picker toward walls when both wall and non-wall (roof/tilted) are
    candidates at similar range — e.g., a wall at 8 m beats a roof at
    4.5 m, but a roof at 3 m beats a wall at 8 m.

    Why: on field clouds (especially /blackbox-derived ones with ~5:1
    roof:wall facet ratios) the closest facet to a WP flying alongside
    the building is frequently a roof corner at WP altitude rather than
    a wall below. The aim-at-center pose for that roof points the camera
    horizontally at building height, instead of down at the actual wall.
    The bonus restores walls' priority for inspection use.

    For wide ratio gaps the bonus stops mattering — a roof at 1 m always
    beats a wall at 10 m (1 < 5), so genuine "fly over the roof" cases
    still produce roof matches.
    """
    best: tuple[int, float] | None = None
    best_weighted = float("inf")

    for i, f in enumerate(facades):
        n = _unit(np.asarray(f.normal, dtype=np.float64))
        c = np.asarray(f.center, dtype=np.float64)
        signed = float(np.dot(n, wp_xyz - c))
        if signed <= 0 or signed > max_distance_m:
            continue
        is_wall = (f.label or "").startswith("wall_")
        weighted = signed * (wall_distance_bonus if is_wall else 1.0)
        if weighted < best_weighted:
            best_weighted = weighted
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
        center = np.asarray(facade.center, dtype=np.float64)

        # Camera look direction = WP → facade center (aim AT the facet).
        #
        # Earlier this was `look = -facade.normal` for "perpendicular"
        # framing, but that fails on noisy facade extractions: when the
        # picker matches a downward-tilted soffit/overhang, perpendicular-
        # to-its-normal sends the camera up at the sky instead of at the
        # building. Aim-at-center always points the camera AT the chosen
        # facet — for a well-placed WP directly in front of a wall, the
        # WP→center vector reduces to -normal anyway, so good cases stay
        # good. Bad picks (which CGAL produces from noisy /blackbox-style
        # clouds) at least still aim at the building rather than empty
        # sky/ground.
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
