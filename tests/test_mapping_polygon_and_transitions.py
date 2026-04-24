"""Regression tests for the "custom path" behaviour changes:

* Facade-to-facade transit waypoints are NOT inserted unconditionally any
  more — the mesh path-segment collision detour only fires when the
  direct line hits the mesh.
* ``_filter_waypoints_by_pointcloud`` drops inspection WPs whose clearance
  ball contains non-target cloud points (obstacles: trees, wires,
  adjacent structures), while keeping WPs whose only nearby cloud points
  are on the facade being photographed.
* ``_derive_dji_mission_seeds`` converts DJI overlap percentages to
  fractions and computes a site-proven standoff from observed waypoints.
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
    _derive_dji_mission_seeds,
    _filter_waypoints_by_pointcloud,
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
# Track C (revised): _filter_waypoints_by_pointcloud
# ---------------------------------------------------------------------------


def _wp(x: float, y: float, z: float = 5.0, *, facade_index: int = 0, index: int = 0, is_transition: bool = False) -> Waypoint:
    return Waypoint(
        x=x, y=y, z=z,
        heading_deg=0.0, gimbal_pitch_deg=0.0,
        speed_ms=2.0, actions=[],
        facade_index=facade_index, index=index,
        is_transition=is_transition,
    )


def _south_facade(index: int = 0) -> Facade:
    """A wall at y=0 facing -y (south). Inspection WPs sit at y<0 looking north."""
    return Facade(
        vertices=np.array([
            [-5.0, 0.0, 0.0], [5.0, 0.0, 0.0],
            [5.0, 0.0, 10.0], [-5.0, 0.0, 10.0],
        ]),
        normal=np.array([0.0, -1.0, 0.0]),
        label=f"wall{index}",
        component_tag="21.1",
        index=index,
    )


def test_pointcloud_filter_keeps_wp_when_only_target_facade_is_near():
    """WP at y=-3 photographing a wall at y=0. The wall's own cloud points
    are within clearance, but they're all in the viewing cone, so the WP
    must survive."""
    facade = _south_facade(0)
    # Cloud points sampled along the wall plane (target surface).
    cloud = np.array([[x, 0.0, z] for x in np.linspace(-3, 3, 7) for z in np.linspace(1, 9, 5)])
    wp = _wp(0.0, -3.0, 5.0, facade_index=0, index=0)  # 3m standoff

    kept, removed, facades = _filter_waypoints_by_pointcloud(
        [wp], cloud, [facade], clearance_m=4.0,
    )
    assert removed == 0
    assert kept == [wp]
    assert facades == []


def test_pointcloud_filter_rejects_wp_with_off_axis_obstacle():
    """Same WP, but now there's a cloud point OFF to the side (tree trunk
    beside the drone) — that must reject the WP."""
    facade = _south_facade(0)
    cloud = np.array([
        [-4.0, -3.0, 5.0],  # tree 4m west of the WP, dist ~4m — outside clearance
        [-1.5, -3.0, 5.0],  # close obstacle 1.5m west of WP — INSIDE clearance
    ])
    wp = _wp(0.0, -3.0, 5.0, facade_index=0, index=0)

    kept, removed, facades = _filter_waypoints_by_pointcloud(
        [wp], cloud, [facade], clearance_m=2.0,
    )
    assert removed == 1
    assert kept == []
    assert facades == [0]


def test_pointcloud_filter_noop_without_cloud():
    wp = _wp(0.0, -3.0, index=0)
    kept, removed, facades = _filter_waypoints_by_pointcloud(
        [wp], None, [_south_facade(0)], clearance_m=2.0,
    )
    assert kept == [wp]
    assert removed == 0
    assert facades == []


def test_pointcloud_filter_transit_wp_has_no_view_cone_relief():
    """Transit waypoints (no facade target) have no viewing cone, so any
    cloud point within clearance rejects them."""
    cloud = np.array([[0.0, 1.0, 0.0]])  # point 1m north of WP
    wp = _wp(0.0, 0.0, 0.0, facade_index=-1, index=0, is_transition=True)

    kept, removed, _ = _filter_waypoints_by_pointcloud(
        [wp], cloud, [], clearance_m=2.0,
    )
    assert removed == 1
    assert kept == []


def test_pointcloud_filter_wp_at_exact_cloud_point_is_rejected():
    cloud = np.array([[0.0, -3.0, 5.0]])  # exactly on the WP
    wp = _wp(0.0, -3.0, 5.0, facade_index=0, index=0)
    kept, removed, _ = _filter_waypoints_by_pointcloud(
        [wp], cloud, [_south_facade(0)], clearance_m=2.0,
    )
    assert removed == 1
    assert kept == []


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
