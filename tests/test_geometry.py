"""Tests for geometry module."""

import math

import numpy as np
import pytest

from flight_planner.geometry import (
    build_rectangular_building,
    convert_enu_to_wgs84,
    generate_mission_waypoints,
    generate_waypoints_for_facade,
)
from flight_planner.models import MissionConfig, RoofType


class TestBuildRectangularBuilding:
    def test_flat_roof_has_5_facades(self):
        """4 walls + 1 flat roof = 5 facades."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        assert len(b.facades) == 5

    def test_pitched_roof_has_6_facades(self):
        """4 walls + 2 roof planes = 6 facades."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
            roof_type=RoofType.PITCHED, roof_pitch_deg=30,
        )
        assert len(b.facades) == 6

    def test_wall_normals_are_horizontal(self):
        """Wall normals should have z=0 (horizontal)."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        for facade in b.facades[:4]:  # first 4 are walls
            assert abs(facade.normal[2]) < 0.01, f"{facade.label} normal z={facade.normal[2]}"

    def test_wall_normals_are_unit_vectors(self):
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        for facade in b.facades:
            norm = np.linalg.norm(facade.normal)
            assert abs(norm - 1.0) < 1e-6

    def test_wall_normals_point_outward(self):
        """All wall normals should point away from building center (origin)."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        for facade in b.facades[:4]:
            center_to_wall = facade.center[:2]  # ignore Z
            dot = np.dot(facade.normal[:2], center_to_wall)
            assert dot > 0, f"{facade.label} normal points inward"

    def test_flat_roof_normal_points_up(self):
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        roof = b.facades[4]
        assert roof.normal[2] > 0.99

    def test_pitched_roof_normals_tilt_outward(self):
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
            roof_type=RoofType.PITCHED, roof_pitch_deg=30,
        )
        for facade in b.facades[4:]:
            assert facade.normal[2] > 0, f"{facade.label} roof normal points down"
            assert abs(facade.normal[2]) < 1.0, f"{facade.label} roof normal is vertical"

    def test_heading_rotates_building(self):
        """A 90° heading should swap the wall orientations."""
        b0 = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8, heading_deg=0,
        )
        b90 = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8, heading_deg=90,
        )
        # The first wall's normal should rotate ~90°
        azimuth_diff = abs(b0.facades[0].azimuth_deg - b90.facades[0].azimuth_deg)
        # Account for wrapping
        azimuth_diff = min(azimuth_diff, 360 - azimuth_diff)
        assert abs(azimuth_diff - 90) < 1.0

    def test_wall_height_matches(self):
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        for facade in b.facades[:4]:
            assert abs(facade.height - 8.0) < 0.1, f"{facade.label} height={facade.height}"


class TestWaypointGeneration:
    def test_generates_waypoints_for_wall(self):
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        wall = b.facades[0]  # back wall
        wps = generate_waypoints_for_facade(wall, config)
        assert len(wps) > 0

    def test_waypoints_offset_from_facade(self):
        """Waypoints should be offset from the facade along the outward normal."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        wall = b.facades[2]  # front wall (normal points toward +y)
        wps = generate_waypoints_for_facade(wall, config)

        # All waypoints should be on the outward side of the facade
        for wp in wps:
            wp_pos = np.array([wp.x, wp.y, wp.z])
            to_wp = wp_pos - wall.center
            dot = np.dot(to_wp[:2], wall.normal[:2])
            assert dot > 0, "Waypoint is on wrong side of facade"

    def test_gimbal_horizontal_for_walls(self):
        """Gimbal pitch should be 0° for vertical walls."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        config = MissionConfig()
        for wall in b.facades[:4]:
            wps = generate_waypoints_for_facade(wall, config)
            for wp in wps:
                assert wp.gimbal_pitch_deg == 0.0

    def test_gimbal_down_for_flat_roof(self):
        """Gimbal pitch should be near -90° for flat roof (clamped with safety margin)."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=8,
        )
        config = MissionConfig()
        roof = b.facades[4]
        wps = generate_waypoints_for_facade(roof, config)
        for wp in wps:
            assert wp.gimbal_pitch_deg <= -80.0, "Flat roof gimbal should be near nadir"
            assert wp.gimbal_pitch_deg >= -90.0, "Gimbal should not exceed hardware limit"

    def test_boustrophedon_order(self):
        """Consecutive waypoints in the same row should be close together."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=40, depth=10, height=8,
        )
        config = MissionConfig(target_gsd_mm_per_px=3.0)  # larger GSD = fewer waypoints
        wall = b.facades[0]
        wps = generate_waypoints_for_facade(wall, config)
        if len(wps) > 2:
            # Check that consecutive waypoints don't jump across the facade
            for i in range(len(wps) - 1):
                dist = math.sqrt(
                    (wps[i+1].x - wps[i].x)**2 +
                    (wps[i+1].y - wps[i].y)**2 +
                    (wps[i+1].z - wps[i].z)**2
                )
                # Should be within one grid step + vertical step
                assert dist < 25, f"Large jump between WP{i} and WP{i+1}: {dist:.1f}m"

    def test_minimum_altitude(self):
        """No waypoint should be below 2m altitude."""
        b = build_rectangular_building(
            lat=53.2, lon=5.8, width=20, depth=10, height=3,
        )
        config = MissionConfig()
        wps = generate_mission_waypoints(b, config)
        for wp in wps:
            assert wp.z >= 2.0, f"WP{wp.index} altitude {wp.z}m below minimum"


class TestCoordinateConversion:
    def test_origin_maps_to_ref_point(self):
        """A waypoint at ENU origin should map to the reference GPS point."""
        from flight_planner.models import Waypoint
        wp = Waypoint(x=0, y=0, z=10)
        convert_enu_to_wgs84([wp], ref_lat=53.2, ref_lon=5.8, ref_alt=0)
        assert abs(wp.lat - 53.2) < 1e-6
        assert abs(wp.lon - 5.8) < 1e-6
        assert abs(wp.alt - 10) < 0.1

    def test_north_offset(self):
        """Moving north in ENU should increase latitude."""
        from flight_planner.models import Waypoint
        wp = Waypoint(x=0, y=100, z=0)  # 100m north
        convert_enu_to_wgs84([wp], ref_lat=53.2, ref_lon=5.8)
        assert wp.lat > 53.2

    def test_east_offset(self):
        """Moving east in ENU should increase longitude."""
        from flight_planner.models import Waypoint
        wp = Waypoint(x=100, y=0, z=0)  # 100m east
        convert_enu_to_wgs84([wp], ref_lat=53.2, ref_lon=5.8)
        assert wp.lon > 5.8
