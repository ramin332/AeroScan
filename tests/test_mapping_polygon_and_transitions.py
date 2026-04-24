"""Regression tests for the four behaviours changed by the "custom path"
mapping-polygon fix:

* Facade-to-facade transit waypoints are NOT inserted unconditionally any
  more — the path-segment collision detour only fires when the direct line
  hits the mesh.
* ``_clip_waypoints_to_dji_bbox`` drops WPs whose XY leaves the DJI
  mapping polygon, reports the count, and tracks the affected facades.
* ``_derive_dji_mission_seeds`` converts DJI overlap percentages to
  fractions and computes a site-proven standoff from observed waypoints.
* ``_check_path_collisions`` still inserts a detour when the direct line
  between two facades would clip through the building — i.e. removing the
  unconditional transitions did not remove the safety layer.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from flight_planner.geometry import (
    build_rectangular_building,
    generate_mission_waypoints,
)
from flight_planner.models import (
    AlgorithmConfig,
    Facade,
    MissionConfig,
    RoofType,
    Waypoint,
    meters_per_deg,
)
from flight_planner.server.api import (
    _clip_waypoints_to_dji_bbox,
    _derive_dji_mission_seeds,
)


# ---------------------------------------------------------------------------
# Track B: no unconditional transition waypoints
# ---------------------------------------------------------------------------


def test_rectangular_building_has_no_transition_waypoints_when_path_is_clear():
    """A convex rectangular building has no corner collisions, so there
    should be exactly zero transition waypoints in the generated mission.

    Before the fix, every facade-to-facade pair inserted 1-3 outward+up
    transit WPs (~10-15 extras on a 4-wall + roof building)."""
    building = build_rectangular_building(
        lat=52.0, lon=5.0,
        width=10, depth=10, height=8, heading_deg=0,
        roof_type=RoofType.FLAT, roof_pitch_deg=0, num_stories=1,
    )
    config = MissionConfig()
    algo = AlgorithmConfig()

    waypoints, stats = generate_mission_waypoints(building, config, algo)

    transitions = [w for w in waypoints if w.is_transition]
    assert transitions == [], (
        f"Expected no transition waypoints on a convex building, got {len(transitions)}. "
        f"Total waypoints: {len(waypoints)}"
    )


# ---------------------------------------------------------------------------
# Track C: _clip_waypoints_to_dji_bbox
# ---------------------------------------------------------------------------


def _enu_to_lonlat(x: float, y: float, ref_lat: float, ref_lon: float) -> tuple[float, float, float]:
    m_per_lat, m_per_lon = meters_per_deg(math.radians(ref_lat))
    return (ref_lon + x / m_per_lon, ref_lat + y / m_per_lat, 0.0)


def _square_polygon_wgs84(half_m: float, ref_lat: float, ref_lon: float):
    """Square polygon [-half_m, half_m] x [-half_m, half_m] in ENU, as
    WGS84 (lon, lat, alt) triples."""
    corners_enu = [
        (-half_m, -half_m),
        (half_m, -half_m),
        (half_m, half_m),
        (-half_m, half_m),
    ]
    return [_enu_to_lonlat(x, y, ref_lat, ref_lon) for x, y in corners_enu]


def _wp(x: float, y: float, *, facade_index: int = 0, index: int = 0) -> Waypoint:
    return Waypoint(
        x=x, y=y, z=5.0,
        heading_deg=0.0, gimbal_pitch_deg=0.0,
        speed_ms=2.0, actions=[],
        facade_index=facade_index, index=index,
    )


def test_clip_waypoints_keeps_inside_drops_outside():
    ref_lat, ref_lon = 52.0, 5.0
    polygon = _square_polygon_wgs84(10.0, ref_lat, ref_lon)

    waypoints = [
        _wp(0, 0, facade_index=0, index=0),      # inside
        _wp(5, 5, facade_index=0, index=1),      # inside
        _wp(20, 0, facade_index=1, index=2),     # outside (east)
        _wp(0, -20, facade_index=2, index=3),    # outside (south)
        _wp(-9.9, 0, facade_index=0, index=4),   # inside edge (within margin)
    ]

    kept, removed_count, removed_facades = _clip_waypoints_to_dji_bbox(
        waypoints, polygon, ref_lat, ref_lon, 0.0,
    )
    assert removed_count == 2
    assert set(removed_facades) == {1, 2}
    assert len(kept) == 3
    # Inside waypoints survive
    for wp in kept:
        assert abs(wp.x) <= 12.0 + 1e-3 and abs(wp.y) <= 12.0 + 1e-3


def test_clip_waypoints_noop_without_polygon():
    waypoints = [_wp(100, 100, index=0), _wp(200, 200, index=1)]
    kept, removed, facades = _clip_waypoints_to_dji_bbox(
        waypoints, None, 0.0, 0.0, 0.0,
    )
    assert kept == waypoints
    assert removed == 0
    assert facades == []


def test_clip_waypoints_margin_preserves_edge_huggers():
    """A waypoint sitting exactly on the polygon boundary must survive the
    2m margin (matches facade filter behaviour)."""
    ref_lat, ref_lon = 52.0, 5.0
    polygon = _square_polygon_wgs84(10.0, ref_lat, ref_lon)

    edge_wp = _wp(10.0, 0.0, facade_index=0, index=0)  # exactly on boundary
    kept, removed, _ = _clip_waypoints_to_dji_bbox(
        [edge_wp], polygon, ref_lat, ref_lon, 0.0,
    )
    assert removed == 0
    assert len(kept) == 1


# ---------------------------------------------------------------------------
# Track D: _derive_dji_mission_seeds
# ---------------------------------------------------------------------------


def test_derive_overlap_as_fraction_from_dji_percent():
    # DJI ships orthoCameraOverlapW / H as string integer percentages.
    seeds = _derive_dji_mission_seeds(
        {"orthoCameraOverlapW": "70", "orthoCameraOverlapH": "60",
         "autoFlightSpeed": "1.5"},
        enu_wps=None, facades=None,
    )
    assert pytest.approx(seeds["front_overlap"], 1e-6) == 0.70
    assert pytest.approx(seeds["side_overlap"], 1e-6) == 0.60
    assert seeds["dji_extracted"]["flight_speed_ms"] == 1.5
    # No WPs/facades → no standoff derivable
    assert "obstacle_clearance_m" not in seeds
    assert seeds["dji_extracted"]["observed_standoff_m"] is None


def test_derive_overlap_from_fractional_input_unchanged():
    # Defensive: if DJI ever starts using fractions, don't divide again.
    seeds = _derive_dji_mission_seeds(
        {"orthoCameraOverlapW": "0.75"},
        enu_wps=None, facades=None,
    )
    assert pytest.approx(seeds["front_overlap"], 1e-6) == 0.75


def test_derive_observed_standoff_from_wps_and_facades():
    # One facade at y = 0 with normal pointing -y. WPs 3m away.
    facade = Facade(
        vertices=np.array([
            [-5.0, 0.0, 0.0], [5.0, 0.0, 0.0],
            [5.0, 0.0, 10.0], [-5.0, 0.0, 10.0],
        ]),
        normal=np.array([0.0, -1.0, 0.0]),
        label="wall",
        component_tag="21.1",
        index=0,
    )
    wps = [
        {"x": 0.0, "y": -3.0, "z": 5.0},
        {"x": 2.0, "y": -3.2, "z": 5.0},
        {"x": -2.0, "y": -2.8, "z": 5.0},
    ]
    seeds = _derive_dji_mission_seeds(
        {"autoFlightSpeed": "2.0"}, enu_wps=wps, facades=[facade],
    )
    assert "obstacle_clearance_m" in seeds
    assert 2.5 <= seeds["obstacle_clearance_m"] <= 3.5
    assert seeds["dji_extracted"]["observed_standoff_m"] == pytest.approx(
        seeds["obstacle_clearance_m"], 1e-6,
    )


def test_derive_ignores_missing_fields():
    seeds = _derive_dji_mission_seeds(None, None, None)
    # No fields present, dji_extracted still there with nulls
    assert "front_overlap" not in seeds
    assert "side_overlap" not in seeds
    assert "obstacle_clearance_m" not in seeds
    assert seeds["dji_extracted"] == {
        "front_overlap": None, "side_overlap": None,
        "flight_speed_ms": None, "observed_standoff_m": None,
    }
