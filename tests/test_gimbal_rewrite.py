"""Tests for gimbal rewrite module."""

import math

import numpy as np
import pytest

from flight_planner.gimbal_rewrite import rewrite_gimbals_perpendicular
from flight_planner.models import Facade, Waypoint


def _make_facade(normal: tuple[float, float, float], center: tuple[float, float, float], size: float = 4.0) -> Facade:
    """Build a square facade of given size, centered at `center`, facing `normal`."""
    n = np.array(normal, dtype=np.float64)
    n /= np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, up); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    c = np.array(center, dtype=np.float64)
    h = size / 2.0
    verts = np.array([c - h * u - h * v, c + h * u - h * v, c + h * u + h * v, c - h * u + h * v])
    return Facade(vertices=verts, normal=n, component_tag="21.1", label="test", index=0)


def test_east_facing_wall_gets_yaw_west():
    # +X (east) facade at x=0, waypoint 5 m east of it
    facade = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=5.0, y=0.0, z=2.0)

    # command_gimbal_yaw=True: this test is about the yaw MATH. The shipping
    # default no longer emits a yaw command (see test_default_leaves_yaw_uncommanded).
    out = rewrite_gimbals_perpendicular([wp], [facade], command_gimbal_yaw=True)[0]

    # Camera should look into facade (west → yaw = -90°)
    assert abs(out.gimbal_yaw_deg - (-90.0)) < 0.5
    # Horizontal → pitch 0
    assert abs(out.gimbal_pitch_deg) < 0.5
    assert out.facade_index == 0


def test_north_facade_gets_yaw_south():
    # +Y facade at y=0, waypoint 5 m north
    facade = _make_facade(normal=(0.0, 1.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=0.0, y=5.0, z=2.0)

    out = rewrite_gimbals_perpendicular([wp], [facade], command_gimbal_yaw=True)[0]

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


def test_default_leaves_yaw_uncommanded_so_the_gimbal_cannot_saturate():
    """The 2026-07-10 fix: no gimbal yaw command, so required pan is identically 0.

    Ground truth from that flight's JPEG XMP: commanded pan was 0 deg everywhere,
    yet actual |GimbalYaw - FlightYaw| sat at a median of 51.5 deg with a p90 of
    exactly 61.0 deg -- pinned against the +-60 deg mechanical stop on 14% of
    photos. The yaw axis was not honouring the absolute-north command. Pitch is
    still commanded; it has no travel limit to saturate against.
    """
    facade = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0))
    wp = Waypoint(x=5.0, y=0.0, z=2.0)

    out = rewrite_gimbals_perpendicular([wp], [facade], preserve_heading=False)[0]

    assert out.gimbal_yaw_deg is None, "yaw must be uncommanded; the gimbal follows the nose"
    assert out.gimbal_pitch_deg is not None
    # The nose carries the azimuth instead.
    assert abs(out.heading_deg - (-90.0)) < 0.5
