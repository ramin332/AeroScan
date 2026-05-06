"""Tests for gimbal rewrite module."""

import math

import numpy as np
import pytest

from flight_planner.gimbal_rewrite import rewrite_gimbals_perpendicular
from flight_planner.models import Facade, Waypoint


def _make_facade(
    normal: tuple[float, float, float],
    center: tuple[float, float, float],
    size: float = 4.0,
    label: str = "test",
) -> Facade:
    """Build a square facade of given size, centered at `center`, facing `normal`."""
    n = np.array(normal, dtype=np.float64)
    n /= np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, up); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    c = np.array(center, dtype=np.float64)
    h = size / 2.0
    verts = np.array([c - h * u - h * v, c + h * u - h * v, c + h * u + h * v, c - h * u + h * v])
    return Facade(vertices=verts, normal=n, component_tag="21.1", label=label, index=0)


def test_east_facing_wall_gets_yaw_west():
    # +X (east) facade at x=0, waypoint 5 m east of it
    facade = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=5.0, y=0.0, z=2.0)

    out = rewrite_gimbals_perpendicular([wp], [facade])[0]

    # Camera should look into facade (west → yaw = -90°)
    assert abs(out.gimbal_yaw_deg - (-90.0)) < 0.5
    # Horizontal → pitch 0
    assert abs(out.gimbal_pitch_deg) < 0.5
    assert out.facade_index == 0


def test_north_facade_gets_yaw_south():
    # +Y facade at y=0, waypoint 5 m north
    facade = _make_facade(normal=(0.0, 1.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=0.0, y=5.0, z=2.0)

    out = rewrite_gimbals_perpendicular([wp], [facade])[0]

    # Camera looks south → yaw = 180° (or -180°)
    assert abs(abs(out.gimbal_yaw_deg) - 180.0) < 0.5
    assert abs(out.gimbal_pitch_deg) < 0.5


def test_waypoint_behind_facade_is_skipped():
    # +X facade, waypoint on the -X (interior) side
    facade = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=-5.0, y=0.0, z=2.0, gimbal_pitch_deg=-19.0, gimbal_yaw_deg=42.0)

    out = rewrite_gimbals_perpendicular([wp], [facade])[0]

    # No outward-facing facade nearby → keep original pose
    assert out.gimbal_pitch_deg == -19.0
    assert out.gimbal_yaw_deg == 42.0
    assert out.facade_index == -1


def test_pitch_clamps_to_hardware_limits():
    # Upward-facing facade directly overhead → look direction is straight down,
    # but with pitch margin we clamp to -88° not -90°.
    facade = _make_facade(normal=(0.0, 0.0, 1.0), center=(0.0, 0.0, 0.0))
    wp = Waypoint(x=0.1, y=0.0, z=10.0)

    out = rewrite_gimbals_perpendicular([wp], [facade], pitch_margin_deg=2.0)[0]

    # Pitch near -90° but clamped
    assert out.gimbal_pitch_deg >= -88.0 - 0.1
    assert out.gimbal_pitch_deg <= 35.0


def test_picks_nearest_facade_when_multiple_available():
    # Two parallel +X facades; waypoint is between them but closer to the
    # inner one. The outer facade is behind the waypoint so it's skipped.
    near = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0))
    far = _make_facade(normal=(1.0, 0.0, 0.0), center=(-10.0, 0.0, 2.0))
    wp = Waypoint(x=3.0, y=0.0, z=2.0)

    out = rewrite_gimbals_perpendicular([wp], [near, far])[0]

    assert out.facade_index == 0


def test_wall_bias_wins_at_modest_distance():
    # Wall is moderately farther than the roof; with default bonus=0.5, a
    # wall up to ~2× the roof distance should still win — biases toward
    # walls during inspection passes alongside the building.
    roof = _make_facade(
        normal=(0.0, 0.0, 1.0), center=(0.0, 0.0, 4.0), label="roof_3",
    )
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(5.0, 0.0, 5.0), label="wall_7",
    )
    # WP is 3 m above the roof, 5 m east of the wall → weighted: roof=3.0,
    # wall=5.0×0.5=2.5 → wall wins.
    wp = Waypoint(x=10.0, y=0.0, z=7.0)

    out = rewrite_gimbals_perpendicular([wp], [roof, wall])[0]

    assert out.facade_index == 1
    assert abs(out.gimbal_yaw_deg - (-90.0)) < 5.0


def test_roof_wins_when_wp_genuinely_above():
    # WP is over the roof and the wall is far enough that the bonus can't
    # save it. Captures the "flying over the building" case the bias must
    # not break.
    roof = _make_facade(
        normal=(0.0, 0.0, 1.0), center=(15.0, 0.0, 4.0), label="roof_3",
    )
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(10.0, 0.0, 3.0), label="wall_7",
    )
    # roof signed = 6-4 = 2; wall signed = 15-10 = 5 → weighted roof=2,
    # wall=2.5 → roof wins.
    wp = Waypoint(x=15.0, y=0.0, z=6.0)

    out = rewrite_gimbals_perpendicular([wp], [roof, wall])[0]

    assert out.facade_index == 0
    # Pointed down at the roof (clamped within hardware pitch margin)
    assert out.gimbal_pitch_deg < -60.0


def test_window_smooths_yaw_across_facade_transition():
    # Two adjacent walls 90° apart. Trajectory walks along facade A then
    # transitions to facade B. With a centered window the WP at the
    # transition should land partway between the two yaws — not snap.
    east_wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    north_wall = _make_facade(
        normal=(0.0, 1.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_1",
    )
    wps = [
        Waypoint(x=5.0, y=-5.0, z=2.0, index=0),
        Waypoint(x=5.0, y=-3.0, z=2.0, index=1),
        Waypoint(x=-3.0, y=5.0, z=2.0, index=2),
        Waypoint(x=-5.0, y=5.0, z=2.0, index=3),
    ]

    smoothed = rewrite_gimbals_perpendicular(
        wps, [east_wall, north_wall], smooth_window=5,
    )
    raw = rewrite_gimbals_perpendicular(
        wps, [east_wall, north_wall], smooth_window=1,
    )

    # Raw: WP[2] aimed straight at north wall. Smoothed: averaged with
    # neighboring east-wall WPs, so yaw differs from raw.
    assert raw[2].facade_index == 1
    assert smoothed[2].facade_index == 1
    assert smoothed[2].gimbal_yaw_deg != raw[2].gimbal_yaw_deg


def test_smoothing_disabled_when_window_one():
    # window=1 should be identity — verify with a 3-WP smoke trajectory.
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    wps = [Waypoint(x=5.0, y=float(j), z=2.0, index=j) for j in range(3)]
    out = rewrite_gimbals_perpendicular(wps, [wall], smooth_window=1)
    for w in out:
        assert abs(w.gimbal_yaw_deg - (-90.0)) < 0.5


def test_window_smoothing_is_centered_not_lagging():
    # Centered window means the smoothed value at index i is symmetric
    # over [i-half, i+half]. With a step change in yaw at WP[5], the
    # smoothed values should ramp symmetrically *around* the step — not
    # only after it (which is what a causal EMA would produce).
    east_wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_a",
    )
    north_wall = _make_facade(
        normal=(0.0, 1.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_b",
    )
    # 11 WPs, transition at index 5: WPs 0..4 see east_wall, WPs 5..10
    # see north_wall.
    wps = []
    for j in range(11):
        if j < 5:
            wps.append(Waypoint(x=5.0, y=-3.0 + 0.1 * j, z=2.0, index=j))
        else:
            wps.append(Waypoint(x=-3.0 + 0.1 * j, y=5.0, z=2.0, index=j))

    smoothed = rewrite_gimbals_perpendicular(
        wps, [east_wall, north_wall], smooth_window=5,
    )
    # WP[3] (2 before transition) should already start drifting from pure
    # east_wall yaw (-90°) toward the north_wall yaw (180°), i.e., centered
    # window peeks ahead. A causal EMA wouldn't do this.
    assert smoothed[3].gimbal_yaw_deg != smoothed[0].gimbal_yaw_deg
