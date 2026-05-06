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

    Returns (facade_index, 3d_distance_to_center) or None if no facade is
    within ``max_distance_m`` on its outward side.

    Distance is the **3D Euclidean distance from the WP to the facade
    center**, not the perpendicular distance to the facade plane. This
    properly penalizes large facades whose plane is close but whose extent
    is elsewhere — e.g., a 30 m-long wall whose plane is 5 m from the WP
    but whose center is 25 m away should NOT win over a closer compact
    facade. (The earlier perpendicular-distance picker had this bug — it
    treated facades as infinite planes and locked onto big distant ones.)

    The signed perpendicular component is still used as a hard
    outward-side filter (signed > 0) so the camera never aims through the
    building at a wall behind it.

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
        delta = wp_xyz - c
        signed = float(np.dot(n, delta))
        if signed <= 0:
            continue
        center_dist = float(np.linalg.norm(delta))
        if center_dist > max_distance_m:
            continue
        is_wall = (f.label or "").startswith("wall_")
        weighted = center_dist * (wall_distance_bonus if is_wall else 1.0)
        if weighted < best_weighted:
            best_weighted = weighted
            best = (i, center_dist)

    return best


def _smooth_gimbal_window(
    waypoints: list[Waypoint],
    window: int,
) -> list[Waypoint]:
    """Centered moving-window average of gimbal pitch/yaw along the trajectory.

    Single-pass EMA proved too narrow for our use case — drones travel in
    long flowy arcs around buildings (corners, orbits, sweeps), and a
    causal exponential filter both lags through transitions and produces
    asymmetric smoothing (head-of-arc smoother than tail-of-arc). A
    centered window doesn't lag and gives uniform smoothing along the
    whole arc.

    For each WP at index i, average over WPs in [i - window/2, i + window/2]
    that have a matched facade. Unmatched WPs keep their original DJI
    pose and are excluded from any window's average — they don't pollute
    neighboring smoothed values.

    * ``window=1`` → no smoothing (identity).
    * ``window=5`` → average over current + 2 before + 2 after (default).
    * ``window=11`` → much heavier smoothing for very long-arc flights.

    Yaw is averaged on the unit circle (cos/sin → atan2) to handle the
    ±180° wraparound; otherwise crossing 180° would yield the wrong
    average direction.
    """
    if not waypoints or window <= 1:
        return waypoints

    n = len(waypoints)
    half = window // 2

    pitches = [float(w.gimbal_pitch_deg) for w in waypoints]
    yaws = [
        float(w.gimbal_yaw_deg) if w.gimbal_yaw_deg is not None else float(w.heading_deg)
        for w in waypoints
    ]
    matched = [w.facade_index >= 0 for w in waypoints]

    out = list(waypoints)
    for i, wp in enumerate(waypoints):
        if not matched[i]:
            continue
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        ps: list[float] = []
        yx_acc = 0.0
        yy_acc = 0.0
        count = 0
        for k in range(lo, hi):
            if not matched[k]:
                continue
            ps.append(pitches[k])
            yx_acc += math.cos(math.radians(yaws[k]))
            yy_acc += math.sin(math.radians(yaws[k]))
            count += 1
        if count == 0:
            continue
        avg_pitch = sum(ps) / count
        avg_yaw = math.degrees(math.atan2(yy_acc, yx_acc))
        out[i] = replace(
            wp,
            gimbal_pitch_deg=float(avg_pitch),
            gimbal_yaw_deg=float(avg_yaw),
        )

    return out


def rewrite_gimbals_perpendicular(
    waypoints: list[Waypoint],
    facades: list[Facade],
    max_distance_m: float = 60.0,
    pitch_margin_deg: float = 2.0,
    preserve_heading: bool = True,
    wall_distance_bonus: float = 0.5,
    smooth_window: int = 5,
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
    smooth_window
        Centered moving-average window size in WPs (default 5). 1 disables
        smoothing; larger values produce a smoother track that better
        matches long flowy flight arcs around buildings. Unmatched WPs
        are excluded from each window's average.

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

    return _smooth_gimbal_window(out, window=smooth_window)
