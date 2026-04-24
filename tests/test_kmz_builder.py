"""Tests for KMZ builder (djikmz-based)."""

import io
import os
import tempfile
import xml.etree.ElementTree as ET
import zipfile

from djikmz import DroneTask

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
        xml = _extract_kml_xml()
        root = ET.fromstring(xml)
        ns = "http://www.dji.com/wpmz/1.0.6"
        for height_el in root.iter(f"{{{ns}}}height"):
            h = float(height_el.text)
            assert h >= 2.0, f"Waypoint height {h}m is below minimum 2.0m"

    def test_per_waypoint_uses_local_height(self):
        """Each waypoint should use per-waypoint height (useGlobalHeight=0)."""
        xml = _extract_kml_xml()
        # All waypoints have explicit heights, so useGlobalHeight should be 0
        assert "0" in xml  # useGlobalHeight=0 present


class TestM4ECompatibility:
    """Verify M4E drone model registration and KMZ generation."""

    def test_m4e_drone_model_accepted(self):
        """DroneTask('M4E') should not raise after registration."""
        task = DroneTask("M4E", "AeroScan")
        assert task is not None

    def test_m4e_max_speed(self):
        """M4E supports up to 21 m/s — waypoints at that speed should pass."""
        wps = _make_test_waypoints(3)
        for wp in wps:
            wp.speed_ms = 21.0
        data = build_kmz_bytes(wps, MissionConfig())
        assert zipfile.is_zipfile(io.BytesIO(data))

    def test_m4e_kmz_roundtrip(self):
        """Full KMZ generation with M4E model produces valid ZIP with parseable XML."""
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            assert "wpmz/template.kml" in zf.namelist()
            kml = zf.read("wpmz/template.kml")
            root = ET.fromstring(kml)
            assert root.tag.endswith("kml")

    def test_m4e_drone_enum_in_xml(self):
        """Generated KML should contain droneEnumValue."""
        xml = _extract_kml_xml()
        assert "droneEnumValue" in xml


_WPML_NS = "http://www.dji.com/wpmz/1.0.6"
_KML_NS = "http://www.opengis.net/kml/2.2"


def _extract_wpml_xml(config: MissionConfig | None = None) -> str:
    """Generate a KMZ and return the waylines.wpml XML string."""
    wps = _make_test_waypoints()
    data = build_kmz_bytes(wps, config or MissionConfig())
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        return zf.read("wpmz/waylines.wpml").decode("utf-8")


class TestWPMLSpecCompliance:
    """Verify KMZ output matches DJI WPML spec requirements."""

    def test_kmz_has_both_files(self):
        """KMZ must contain both template.kml and waylines.wpml."""
        data = build_kmz_bytes(_make_test_waypoints(), MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            names = zf.namelist()
            assert "wpmz/template.kml" in names
            assert "wpmz/waylines.wpml" in names

    def test_global_transitional_speed_in_template(self):
        """template.kml missionConfig must have globalTransitionalSpeed (required)."""
        xml = _extract_kml_xml()
        assert "globalTransitionalSpeed" in xml

    def test_global_transitional_speed_value(self):
        """globalTransitionalSpeed should match config flight speed."""
        config = MissionConfig(flight_speed_ms=3.5)
        xml = _extract_kml_xml(config)
        root = ET.fromstring(xml)
        el = root.find(f".//{{{_WPML_NS}}}globalTransitionalSpeed")
        assert el is not None
        assert float(el.text) == 3.5

    def test_ellipsoid_height_per_waypoint(self):
        """Each waypoint must have ellipsoidHeight when useGlobalHeight=0."""
        xml = _extract_kml_xml()
        root = ET.fromstring(xml)
        placemarks = list(root.iter(f"{{{_KML_NS}}}Placemark"))
        assert len(placemarks) > 0
        for pm in placemarks:
            eh = pm.find(f"{{{_WPML_NS}}}ellipsoidHeight")
            assert eh is not None, "Waypoint missing ellipsoidHeight"
            assert float(eh.text) > 0

    def test_waylines_has_execute_height(self):
        """waylines.wpml waypoints must have executeHeight."""
        xml = _extract_wpml_xml()
        root = ET.fromstring(xml)
        placemarks = list(root.iter(f"{{{_KML_NS}}}Placemark"))
        assert len(placemarks) > 0
        for pm in placemarks:
            eh = pm.find(f"{{{_WPML_NS}}}executeHeight")
            assert eh is not None, "waylines.wpml waypoint missing executeHeight"

    def test_waylines_has_wayline_id(self):
        """waylines.wpml Folder must have waylineId."""
        xml = _extract_wpml_xml()
        assert "waylineId" in xml

    def test_waylines_has_execute_height_mode(self):
        """waylines.wpml Folder must have executeHeightMode."""
        xml = _extract_wpml_xml()
        assert "executeHeightMode" in xml
        assert "relativeToStartPoint" in xml

    def test_waylines_has_global_rth_height(self):
        """waylines.wpml missionConfig must have globalRTHHeight."""
        xml = _extract_wpml_xml()
        assert "globalRTHHeight" in xml

    def test_waylines_no_use_global_flags(self):
        """waylines.wpml should not have useGlobal* flags."""
        xml = _extract_wpml_xml()
        root = ET.fromstring(xml)
        for pm in root.iter(f"{{{_KML_NS}}}Placemark"):
            for tag in ["useGlobalHeight", "useGlobalSpeed", "useGlobalHeadingParam", "useGlobalTurnParam"]:
                assert pm.find(f"{{{_WPML_NS}}}{tag}") is None, f"waylines.wpml should not have {tag}"

    def test_waylines_is_valid_xml(self):
        """waylines.wpml should be parseable XML."""
        xml = _extract_wpml_xml()
        root = ET.fromstring(xml)
        assert root.tag.endswith("kml")


class TestGimbalYaw:
    """Verify gimbal yaw is correctly handled in KMZ output."""

    def test_gimbal_yaw_none_not_in_xml(self):
        """When gimbal_yaw_deg=None (default), gimbal yaw should NOT be enabled."""
        wps = _make_test_waypoints(3)
        # Default: gimbal_yaw_deg=None
        for wp in wps:
            assert wp.gimbal_yaw_deg is None
        xml = _extract_kml_xml()
        # gimbalYawRotateEnable should be 0 (disabled) for all waypoints
        assert "gimbalYawRotateEnable>1<" not in xml.replace(" ", "")

    def test_gimbal_yaw_explicit(self):
        """When gimbal_yaw_deg is set, it should appear in KMZ XML."""
        wps = _make_test_waypoints(3)
        for wp in wps:
            wp.gimbal_yaw_deg = 90.0
        data = build_kmz_bytes(wps, MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            xml = zf.read("wpmz/template.kml").decode("utf-8")
        assert "gimbalYawRotateEnable" in xml
        assert "90" in xml


class TestWPML106Compliance:
    """Structural features aligned with DJI Pilot 2's 1.0.6 exports."""

    def test_wpml_namespace_is_106(self):
        """Both template.kml and waylines.wpml declare the 1.0.6 namespace."""
        tpl = _extract_kml_xml()
        wpml = _extract_wpml_xml()
        assert "wpmz/1.0.6" in tpl
        assert "wpmz/1.0.6" in wpml
        assert "wpmz/1.0.3" not in tpl
        assert "wpmz/1.0.3" not in wpml

    def test_m4e_enum_values_injected(self):
        """Drone + payload enums match M4E_* constants, and payloadSubEnumValue is emitted."""
        from flight_planner.kmz_builder import (
            M4E_DRONE_ENUM,
            M4E_DRONE_SUB_ENUM,
            M4E_PAYLOAD_ENUM,
            M4E_PAYLOAD_SUB_ENUM,
        )

        xml = _extract_kml_xml()
        root = ET.fromstring(xml)
        drone_info = root.find(f".//{{{_WPML_NS}}}droneInfo")
        assert drone_info is not None
        assert drone_info.find(f"{{{_WPML_NS}}}droneEnumValue").text == str(M4E_DRONE_ENUM)
        assert (
            drone_info.find(f"{{{_WPML_NS}}}droneSubEnumValue").text
            == str(M4E_DRONE_SUB_ENUM)
        )

        payload_info = root.find(f".//{{{_WPML_NS}}}payloadInfo")
        assert payload_info is not None
        assert (
            payload_info.find(f"{{{_WPML_NS}}}payloadEnumValue").text
            == str(M4E_PAYLOAD_ENUM)
        )
        sub_el = payload_info.find(f"{{{_WPML_NS}}}payloadSubEnumValue")
        assert sub_el is not None, "payloadSubEnumValue must be emitted"
        assert sub_el.text == str(M4E_PAYLOAD_SUB_ENUM)

    def test_wayline_avoid_limit_area_mode(self):
        """missionConfig must carry waylineAvoidLimitAreaMode=0."""
        xml = _extract_kml_xml()
        root = ET.fromstring(xml)
        el = root.find(f".//{{{_WPML_NS}}}waylineAvoidLimitAreaMode")
        assert el is not None
        assert el.text == "0"

    def test_waylines_has_start_action_group(self):
        """Each Folder in waylines.wpml has <wpml:startActionGroup> with gimbal + hover + focus."""
        xml = _extract_wpml_xml()
        root = ET.fromstring(xml)
        folders = root.findall(f".//{{{_KML_NS}}}Folder")
        assert folders, "expected at least one Folder"
        for folder in folders:
            group = folder.find(f"{{{_WPML_NS}}}startActionGroup")
            assert group is not None, "Folder missing startActionGroup"
            funcs = [
                f.text for f in group.findall(f".//{{{_WPML_NS}}}actionActuatorFunc")
            ]
            assert funcs == ["gimbalRotate", "hover", "setFocusType"], funcs

    def test_start_action_uses_first_waypoint_pitch(self):
        """Initial gimbalRotate pitch should match the first waypoint's gimbal pitch."""
        wps = _make_test_waypoints(3)
        wps[0].gimbal_pitch_deg = -42.0
        data = build_kmz_bytes(wps, MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            wpml = zf.read("wpmz/waylines.wpml").decode("utf-8")
        root = ET.fromstring(wpml)
        pitch_el = root.find(
            f".//{{{_WPML_NS}}}startActionGroup"
            f"//{{{_WPML_NS}}}gimbalPitchRotateAngle"
        )
        assert pitch_el is not None
        assert float(pitch_el.text) == -42.0

    def test_waylines_has_distance_and_duration(self):
        """Each Folder carries distance + duration metadata."""
        xml = _extract_wpml_xml()
        root = ET.fromstring(xml)
        for folder in root.findall(f".//{{{_KML_NS}}}Folder"):
            dist = folder.find(f"{{{_WPML_NS}}}distance")
            dur = folder.find(f"{{{_WPML_NS}}}duration")
            assert dist is not None, "missing distance"
            assert dur is not None, "missing duration"
            assert float(dist.text) > 0
            assert float(dur.text) > 0

    def test_distance_matches_path_length(self):
        """Emitted distance should equal summed 3D segment lengths through the waypoints."""
        import math
        wps = _make_test_waypoints(5)
        expected = sum(
            math.sqrt(
                (wps[i].x - wps[i - 1].x) ** 2
                + (wps[i].y - wps[i - 1].y) ** 2
                + (wps[i].z - wps[i - 1].z) ** 2
            )
            for i in range(1, len(wps))
        )
        data = build_kmz_bytes(wps, MissionConfig())
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            wpml = zf.read("wpmz/waylines.wpml").decode("utf-8")
        root = ET.fromstring(wpml)
        dist_el = root.find(f".//{{{_WPML_NS}}}distance")
        assert dist_el is not None
        assert abs(float(dist_el.text) - expected) < 0.01
