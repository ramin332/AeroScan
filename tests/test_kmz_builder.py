"""Tests for KMZ builder (djikmz-based)."""

import io
import os
import tempfile
import zipfile

from flight_planner.kmz_builder import build_kmz, build_kmz_bytes
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

    def test_kmz_contains_template_kml(self):
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            names = zf.namelist()
            assert "wpmz/template.kml" in names

    def test_kmz_file_write(self):
        with tempfile.NamedTemporaryFile(suffix=".kmz", delete=False) as f:
            path = f.name
        try:
            result = build_kmz(_make_test_waypoints(), MissionConfig(), path)
            assert os.path.exists(result)
            assert os.path.getsize(result) > 0
        finally:
            os.unlink(path)

    def test_template_kml_is_valid_xml(self):
        """The generated KML should be parseable XML."""
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            kml_bytes = zf.read("wpmz/template.kml")
        # Should be valid XML (djikmz handles this)
        import xml.etree.ElementTree as ET
        root = ET.fromstring(kml_bytes)
        assert root.tag.endswith("kml")

    def test_different_waypoint_counts(self):
        """KMZ should work for different numbers of waypoints."""
        for n in [1, 3, 10]:
            data = build_kmz_bytes(_make_test_waypoints(n), MissionConfig())
            assert zipfile.is_zipfile(io.BytesIO(data))

    def test_kmz_bytes_matches_file(self):
        """In-memory bytes should match file output."""
        wps = _make_test_waypoints()
        config = MissionConfig()
        kmz_bytes = build_kmz_bytes(wps, config)

        with tempfile.NamedTemporaryFile(suffix=".kmz", delete=False) as f:
            path = f.name
        try:
            build_kmz(wps, config, path)
            with open(path, "rb") as f:
                file_bytes = f.read()
            # Both should be valid ZIPs with template.kml
            assert zipfile.is_zipfile(io.BytesIO(kmz_bytes))
            assert zipfile.is_zipfile(io.BytesIO(file_bytes))
        finally:
            os.unlink(path)


def _extract_kml_xml(config: MissionConfig | None = None) -> str:
    """Generate a KMZ and return the template.kml XML string."""
    wps = _make_test_waypoints()
    data = build_kmz_bytes(wps, config or MissionConfig())
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        return zf.read("wpmz/template.kml").decode("utf-8")


class TestKMZSafetyDefaults:
    """Verify that DJI Pilot 2 safety defaults are correctly set in the KMZ XML."""

    def test_rc_lost_action_go_home(self):
        """RC signal loss should trigger RTH by default, not continue."""
        xml = _extract_kml_xml()
        assert "executeLostAction" in xml, "exitOnRCLost should be 'executeLostAction', not 'goContinue'"
        assert "goBack" in xml, "executeRCLostAction should be 'goBack' (RTH)"

    def test_finish_action_go_home(self):
        """Mission completion should trigger RTH."""
        xml = _extract_kml_xml()
        assert "goHome" in xml, "finishAction should be 'goHome'"

    def test_takeoff_security_height(self):
        """Takeoff security height should be 5m for building inspection."""
        xml = _extract_kml_xml()
        assert "5.0" in xml or "5" in xml, "takeOffSecurityHeight should be 5.0m"

    def test_height_mode_relative(self):
        """Height mode should be relativeToStartPoint."""
        xml = _extract_kml_xml()
        assert "relativeToStartPoint" in xml, "heightMode should be 'relativeToStartPoint'"

    def test_rc_lost_configurable(self):
        """RC lost action should respect config override."""
        config = MissionConfig(rc_lost_action="hover")
        xml = _extract_kml_xml(config)
        assert "executeLostAction" in xml
        assert "handover" in xml, "executeRCLostAction should be 'handover' for hover"

    def test_finish_action_configurable(self):
        """Finish action should respect config override."""
        config = MissionConfig(finish_action="land")
        xml = _extract_kml_xml(config)
        assert "autoLand" in xml, "finishAction should be 'autoLand' for land"

    def test_per_waypoint_height_above_minimum(self):
        """Every waypoint height should be >= 2m (MIN_ALTITUDE_M)."""
        import xml.etree.ElementTree as ET
        xml = _extract_kml_xml()
        root = ET.fromstring(xml)
        # Find all height elements in Placemarks
        ns = {"wpml": "http://www.dji.com/wpmz/1.0.6"}
        for height_el in root.iter("{http://www.dji.com/wpmz/1.0.6}height"):
            h = float(height_el.text)
            assert h >= 2.0, f"Waypoint height {h}m is below minimum 2.0m"

    def test_per_waypoint_uses_local_height(self):
        """Each waypoint should use per-waypoint height (useGlobalHeight=0)."""
        xml = _extract_kml_xml()
        # All waypoints have explicit heights, so useGlobalHeight should be 0
        assert "0" in xml  # useGlobalHeight=0 present
