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


def test_prefers_walls_over_geometrically_closer_roof():
    # Roof facade (upward-facing) is geometrically closer to the waypoint,
    # but the wall (in range) should win because of the wall-preference bias.
    # Waypoint is high up + slightly east of building center.
    roof = _make_facade(
        normal=(0.0, 0.0, 1.0), center=(0.0, 0.0, 4.0), label="roof_3",
    )
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(2.0, 0.0, 3.0), label="wall_7",
    )
    wp = Waypoint(x=4.0, y=0.0, z=4.5)

    out = rewrite_gimbals_perpendicular([wp], [roof, wall])[0]

    # Wall is at index 1 in the input; assert the picker chose the wall.
    assert out.facade_index == 1
    # Looking west into the wall → yaw ≈ -90°; pitch should be near horizontal,
    # NOT straight-down (which is what aiming at the roof would have produced).
    assert abs(out.gimbal_yaw_deg - (-90.0)) < 5.0
    assert out.gimbal_pitch_deg > -45.0


def test_falls_back_to_roof_when_no_wall_in_range():
    # Roof is the only facet within max_distance; wall-preference should
    # gracefully fall through to the all-facets pick.
    roof = _make_facade(
        normal=(0.0, 0.0, 1.0), center=(0.0, 0.0, 4.0), label="roof_3",
    )
    far_wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(-200.0, 0.0, 3.0), label="wall_7",
    )
    wp = Waypoint(x=0.5, y=0.0, z=10.0)

    out = rewrite_gimbals_perpendicular(
        [wp], [roof, far_wall], max_distance_m=20.0,
    )[0]

    assert out.facade_index == 0
