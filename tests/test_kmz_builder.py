"""Tests for KMZ builder."""

import io
import os
import tempfile
import zipfile

import pytest
from lxml import etree

from flight_planner.kmz_builder import (
    KML_NS,
    WPML_NS,
    build_kmz,
    build_kmz_bytes,
)
from flight_planner.models import (
    ActionType,
    CameraAction,
    CameraName,
    MissionConfig,
    Waypoint,
)


def _make_test_waypoints(n: int = 5) -> list[Waypoint]:
    """Create a simple list of test waypoints."""
    waypoints = []
    for i in range(n):
        wp = Waypoint(
            x=float(i * 3),
            y=0.0,
            z=10.0,
            lat=53.2 + i * 0.0001,
            lon=5.8 + i * 0.0001,
            alt=10.0,
            heading_deg=180.0,
            gimbal_pitch_deg=0.0,
            speed_ms=3.0,
            actions=[
                CameraAction(action_type=ActionType.TAKE_PHOTO, camera=CameraName.WIDE),
            ],
            facade_index=0,
            component_tag="21.1",
            index=i,
        )
        waypoints.append(wp)
    return waypoints


class TestKMZStructure:
    def test_kmz_is_valid_zip(self):
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        assert zipfile.is_zipfile(io.BytesIO(data))

    def test_kmz_contains_required_files(self):
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            names = zf.namelist()
            assert "wpmz/template.kml" in names
            assert "wpmz/waylines.wpml" in names

    def test_kmz_file_write(self):
        with tempfile.NamedTemporaryFile(suffix=".kmz", delete=False) as f:
            path = f.name
        try:
            result = build_kmz(_make_test_waypoints(), MissionConfig(), path)
            assert os.path.exists(result)
            assert os.path.getsize(result) > 0
        finally:
            os.unlink(path)


class TestTemplateKML:
    def _parse_template(self, waypoints=None, config=None):
        wps = waypoints or _make_test_waypoints()
        cfg = config or MissionConfig()
        data = build_kmz_bytes(wps, cfg)
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            kml_data = zf.read("wpmz/template.kml")
        return etree.fromstring(kml_data)

    def test_root_is_kml(self):
        root = self._parse_template()
        assert root.tag == f"{{{KML_NS}}}kml"

    def test_has_document(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        assert doc is not None

    def test_has_mission_config(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        mc = doc.find(f"{{{WPML_NS}}}missionConfig")
        assert mc is not None

    def test_has_folder_with_placemarks(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        assert folder is not None
        placemarks = folder.findall(f"{{{KML_NS}}}Placemark")
        assert len(placemarks) == 5

    def test_placemark_has_coordinates(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        point = pm.find(f"{{{KML_NS}}}Point")
        assert point is not None
        coords = point.find(f"{{{KML_NS}}}coordinates")
        assert coords is not None
        assert "," in coords.text

    def test_placemark_has_waypoint_index(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        idx = pm.find(f"{{{WPML_NS}}}index")
        assert idx is not None
        assert idx.text == "0"

    def test_placemark_has_heading(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        heading_param = pm.find(f"{{{WPML_NS}}}waypointHeadingParam")
        assert heading_param is not None

    def test_placemark_has_actions(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        action_group = pm.find(f"{{{WPML_NS}}}actionGroup")
        assert action_group is not None
        actions = action_group.findall(f"{{{WPML_NS}}}action")
        # Should have gimbalRotate + takePhoto = 2 actions
        assert len(actions) == 2

    def test_drone_info_present(self):
        root = self._parse_template()
        doc = root.find(f"{{{KML_NS}}}Document")
        mc = doc.find(f"{{{WPML_NS}}}missionConfig")
        drone_info = mc.find(f"{{{WPML_NS}}}droneInfo")
        assert drone_info is not None
        enum_val = drone_info.find(f"{{{WPML_NS}}}droneEnumValue")
        assert enum_val is not None


class TestWaylinesWPML:
    def _parse_waylines(self, waypoints=None, config=None):
        wps = waypoints or _make_test_waypoints()
        cfg = config or MissionConfig()
        data = build_kmz_bytes(wps, cfg)
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            wpml_data = zf.read("wpmz/waylines.wpml")
        return etree.fromstring(wpml_data)

    def test_structure_mirrors_template(self):
        root = self._parse_waylines()
        doc = root.find(f"{{{KML_NS}}}Document")
        assert doc is not None
        folder = doc.find(f"{{{KML_NS}}}Folder")
        assert folder is not None
        placemarks = folder.findall(f"{{{KML_NS}}}Placemark")
        assert len(placemarks) == 5

    def test_has_execute_height(self):
        root = self._parse_waylines()
        doc = root.find(f"{{{KML_NS}}}Document")
        folder = doc.find(f"{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        eh = pm.find(f"{{{WPML_NS}}}executeHeight")
        assert eh is not None
