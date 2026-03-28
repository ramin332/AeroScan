"""Integration tests: end-to-end building -> KMZ -> validate."""

import io
import zipfile

import numpy as np

from flight_planner.building_presets import (
    l_shaped_block,
    large_apartment_block,
    pitched_roof_house,
    simple_box,
)
from flight_planner.camera import compute_distance_for_gsd, get_camera
from flight_planner.geometry import generate_mission_waypoints
from flight_planner.kmz_builder import build_kmz_bytes
from flight_planner.models import MissionConfig


class TestSimpleBoxEndToEnd:
    def test_full_pipeline(self):
        """Simple box -> waypoints -> KMZ -> validate."""
        building = simple_box()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        waypoints = generate_mission_waypoints(building, config)

        assert len(waypoints) > 0
        assert all(wp.lat != 0 for wp in waypoints)
        assert all(wp.lon != 0 for wp in waypoints)

        # Generate KMZ
        kmz_bytes = build_kmz_bytes(waypoints, config)
        assert len(kmz_bytes) > 0

        # Validate KMZ structure
        with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as zf:
            assert "wpmz/template.kml" in zf.namelist()

    def test_waypoint_count_reasonable(self):
        """A 20x10x8m building at 2mm GSD should produce a manageable number of waypoints."""
        building = simple_box()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        waypoints = generate_mission_waypoints(building, config)
        assert 10 < len(waypoints) < 1000

    def test_gsd_consistency(self):
        """Verify that waypoint distance from facade matches target GSD."""
        building = simple_box()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        camera = get_camera(config.camera)
        expected_distance = compute_distance_for_gsd(camera, config.target_gsd_mm_per_px)

        waypoints = generate_mission_waypoints(building, config)
        for wp in waypoints:
            if wp.facade_index < 4 and not wp.is_transition:
                facade = building.facades[wp.facade_index]
                wp_pos = np.array([wp.x, wp.y, wp.z])
                to_wp = wp_pos - facade.center
                dist = abs(np.dot(to_wp, facade.normal))
                assert abs(dist - expected_distance) < 2.0, (
                    f"WP{wp.index} distance {dist:.1f}m != expected {expected_distance:.1f}m"
                )


class TestPitchedRoofEndToEnd:
    def test_full_pipeline(self):
        building = pitched_roof_house()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        waypoints = generate_mission_waypoints(building, config)

        assert len(waypoints) > 0

        roof_wps = [wp for wp in waypoints if wp.facade_index >= 4]
        assert len(roof_wps) > 0
        for wp in roof_wps:
            assert wp.gimbal_pitch_deg < 0, "Roof gimbal should tilt downward"
            assert wp.gimbal_pitch_deg > -90, "Pitched roof gimbal shouldn't be straight down"

        kmz_bytes = build_kmz_bytes(waypoints, config)
        assert len(kmz_bytes) > 0


class TestLShapedEndToEnd:
    def test_full_pipeline(self):
        building = l_shaped_block()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        waypoints = generate_mission_waypoints(building, config)

        assert len(waypoints) > 0
        facade_indices = set(wp.facade_index for wp in waypoints)
        assert len(facade_indices) >= 4  # at least some walls + roofs (interior walls removed)

        kmz_bytes = build_kmz_bytes(waypoints, config)
        assert len(kmz_bytes) > 0


class TestMaxWaypoints:
    def test_under_mission_limit(self):
        """Even the largest preset should be under the 65535 waypoint limit."""
        building = large_apartment_block()
        config = MissionConfig(target_gsd_mm_per_px=2.0)
        waypoints = generate_mission_waypoints(building, config)
        assert len(waypoints) <= 65535


class TestCoordinateFormat:
    def test_kmz_is_valid(self):
        """KMZ should be a valid ZIP with template.kml."""
        building = simple_box()
        config = MissionConfig()
        waypoints = generate_mission_waypoints(building, config)
        kmz_bytes = build_kmz_bytes(waypoints, config)

        assert zipfile.is_zipfile(io.BytesIO(kmz_bytes))
        with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as zf:
            assert "wpmz/template.kml" in zf.namelist()
            # Template should be valid XML
            import xml.etree.ElementTree as ET
            kml = ET.fromstring(zf.read("wpmz/template.kml"))
            assert kml.tag.endswith("kml")
