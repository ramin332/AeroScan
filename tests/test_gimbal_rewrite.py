"""Tests for gimbal rewrite module."""

import math
from dataclasses import replace

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


def test_smoothing_keeps_raw_through_sharp_transition():
    # Two walls 90° apart. With outlier rejection the WPs straddling the
    # transition should fall back to their raw value — *not* be averaged
    # across the transition. This is the fix for the "yaw=0 / pitch=0
    # randomly pointing nowhere" bug from the field augment.
    east_wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    north_wall = _make_facade(
        normal=(0.0, 1.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_1",
    )
    # 4 WPs: 2 firmly east-wall, 2 firmly north-wall. The transition is
    # at WP[1]→WP[2] (a 90° yaw flip).
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

    # Each WP's smoothed yaw should equal its raw yaw — outlier rejection
    # discards the cross-wall WPs from each window.
    for i in range(4):
        assert smoothed[i].gimbal_yaw_deg == raw[i].gimbal_yaw_deg


def test_smoothing_averages_within_coherent_run():
    # 5 WPs all looking at the same wall, but with small per-WP yaw
    # jitter from imperfect facade picks (±2°). Window smoothing should
    # average these toward the central -90° yaw.
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    wps = [Waypoint(x=5.0, y=float(j), z=2.0, index=j) for j in range(5)]
    out = rewrite_gimbals_perpendicular(wps, [wall], smooth_window=5)
    # All 5 WPs see the same wall — yaw should be exactly -90° everywhere.
    for w in out:
        assert abs(w.gimbal_yaw_deg - (-90.0)) < 0.5


def test_smoothing_rejects_one_outlier_in_window():
    # 5 WPs near a wall, but inject a fake "bad pick" outlier into one
    # by replacing its raw gimbal yaw post-pick. Smoothing should reject
    # the outlier when computing neighbors' values.
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    wps = [Waypoint(x=5.0, y=float(j), z=2.0, index=j) for j in range(5)]
    raw = rewrite_gimbals_perpendicular(wps, [wall], smooth_window=1)
    # Vandalize WP[2] with a bogus yaw (simulating a bad facade pick).
    bogus = [
        replace(w, gimbal_yaw_deg=160.0) if w.index == 2 else w
        for w in raw
    ]
    smoothed = rewrite_gimbals_perpendicular(
        bogus, [wall], smooth_window=5,
    )
    # WP[1] and WP[3] should not be polluted by WP[2]'s bogus value —
    # outlier rejection drops it from their windows.
    assert abs(smoothed[1].gimbal_yaw_deg - (-90.0)) < 0.5
    assert abs(smoothed[3].gimbal_yaw_deg - (-90.0)) < 0.5


def test_smoothing_disabled_when_window_one():
    # window=1 should be identity — verify with a 3-WP smoke trajectory.
    wall = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), label="wall_0",
    )
    wps = [Waypoint(x=5.0, y=float(j), z=2.0, index=j) for j in range(3)]
    out = rewrite_gimbals_perpendicular(wps, [wall], smooth_window=1)
    for w in out:
        assert abs(w.gimbal_yaw_deg - (-90.0)) < 0.5


def test_smoothing_does_not_collapse_to_zero_on_pitch_signflip():
    # Regression for the field bug: 5 WPs where pitches alternate in sign
    # (camera straddling a pitch-flip across two facades) used to mean to
    # ~0° → camera pointing nowhere. Outlier rejection now keeps each WP
    # at its raw pitch when neighbors disagree by > pitch_outlier_deg.
    wall_high = _make_facade(
        normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 5.0), label="wall_a",
    )
    # Position WPs so half look up (pitch>0) at wall_high above, half look
    # down (pitch<0) at the same plane below. Use just one facade so all
    # WPs match, but at very different pitches.
    wps = [
        # Two looking down sharply at wall_high (WP above wall center)
        Waypoint(x=5.0, y=0.0, z=20.0, index=0),
        Waypoint(x=5.0, y=0.0, z=18.0, index=1),
        # Two looking up at wall_high (WP below wall center)
        Waypoint(x=5.0, y=0.0, z=-5.0, index=3),
        Waypoint(x=5.0, y=0.0, z=-7.0, index=4),
    ]
    raw = rewrite_gimbals_perpendicular(wps, [wall_high], smooth_window=1)
    smoothed = rewrite_gimbals_perpendicular(wps, [wall_high], smooth_window=5)
    # No smoothed pitch should land at 0° (unless its raw pitch was
    # already there) — that was the cancellation bug signature.
    for i, (r, s) in enumerate(zip(raw, smoothed)):
        if abs(r.gimbal_pitch_deg) > 5.0:
            assert abs(s.gimbal_pitch_deg) > 1.0, (
                f"WP[{i}] collapsed from raw pitch {r.gimbal_pitch_deg:.1f}° "
                f"to ~0° via smoothing — outlier rejection failed"
            )
