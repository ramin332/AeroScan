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
    """Find the best outward-facing facade for this waypoint by weighted distance.

    Returns (facade_index, signed_perp_distance) or None if no facade is
    within ``max_distance_m`` on its outward side.

    Walls (label starts with ``wall_``) get their selection distance
    multiplied by ``wall_distance_bonus``. With the default 0.5, a wall at
    10 m beats a roof at 4.9 m, but a roof at 4 m beats a wall at 10 m —
    the picker leans toward walls when reasonable, but still chooses
    non-wall facets when the WP is genuinely closer to one (e.g., flying
    *over* a roof during a transit segment). Set bonus to 1.0 to disable
    the bias entirely, or below 0.5 for a stronger wall preference.
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


def _smooth_gimbal_ema(
    waypoints: list[Waypoint],
    alpha: float,
) -> list[Waypoint]:
    """Exponentially-smooth gimbal pitch/yaw along the trajectory.

    Drone flight paths are smooth, so the gimbal should glide between
    facades rather than snap. EMA filters out occasional bad facade picks
    and produces a more cinematic / calmer gimbal track.

    * ``alpha=1.0`` → no smoothing (identity).
    * ``alpha=0.6`` → mild smoothing (default).
    * ``alpha→0`` → almost frozen (the first matched WP dominates).

    Only re-aimed WPs (``facade_index >= 0``) participate. Unmatched WPs
    keep their original DJI pose untouched, AND reset the EMA state — so a
    matched WP after a gap starts fresh instead of carrying state from a
    different building region.

    Yaw is smoothed on the unit circle (cos/sin EMA → atan2) to handle the
    ±180° wraparound correctly; otherwise crossing 180° would lurch the
    EMA the long way around the compass.
    """
    if not waypoints or alpha >= 1.0:
        return waypoints

    out = list(waypoints)
    ema_pitch: float | None = None
    ema_yx: float | None = None
    ema_yy: float | None = None

    for i, wp in enumerate(out):
        if wp.facade_index < 0:
            ema_pitch = ema_yx = ema_yy = None
            continue

        p = float(wp.gimbal_pitch_deg)
        yaw_deg = (
            float(wp.gimbal_yaw_deg)
            if wp.gimbal_yaw_deg is not None
            else float(wp.heading_deg)
        )
        yx = math.cos(math.radians(yaw_deg))
        yy = math.sin(math.radians(yaw_deg))

        if ema_pitch is None:
            ema_pitch, ema_yx, ema_yy = p, yx, yy
            continue

        ema_pitch = alpha * p + (1.0 - alpha) * ema_pitch
        ema_yx = alpha * yx + (1.0 - alpha) * ema_yx
        ema_yy = alpha * yy + (1.0 - alpha) * ema_yy

        smoothed_yaw = math.degrees(math.atan2(ema_yy, ema_yx))
        out[i] = replace(
            wp,
            gimbal_pitch_deg=float(ema_pitch),
            gimbal_yaw_deg=float(smoothed_yaw),
        )

    return out


def rewrite_gimbals_perpendicular(
    waypoints: list[Waypoint],
    facades: list[Facade],
    max_distance_m: float = 60.0,
    pitch_margin_deg: float = 2.0,
    preserve_heading: bool = True,
    wall_distance_bonus: float = 0.5,
    smooth_alpha: float = 0.6,
) -> list[Waypoint]:
    """Rewrite ``gimbal_pitch_deg`` / ``gimbal_yaw_deg`` so each waypoint
    photographs the best nearby facade head-on.

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
    wall_distance_bonus
        Selection-distance multiplier applied to wall facets (0–1, default
        0.5). Lower values bias the picker more toward walls; 1.0 disables
        the bias. Keeps roof-aiming valid when the WP is genuinely above one.
    smooth_alpha
        EMA coefficient (0–1, default 0.6) for post-pick gimbal smoothing
        across consecutive matched WPs. 1.0 disables smoothing; lower
        values produce a calmer gimbal track at the cost of lag through
        facade transitions.

    Returns a new list of Waypoints; input list is not mutated.
    """
    pitch_min = GIMBAL_TILT_MIN_DEG + pitch_margin_deg
    pitch_max = GIMBAL_TILT_MAX_DEG - pitch_margin_deg

    out: list[Waypoint] = []
    for wp in waypoints:
        pos = np.array([wp.x, wp.y, wp.z], dtype=np.float64)
        pick = _pick_facade_for_waypoint(
            pos, facades, max_distance_m,
            wall_distance_bonus=wall_distance_bonus,
        )
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

    return _smooth_gimbal_ema(out, alpha=smooth_alpha)
