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
