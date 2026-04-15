"""KMZ builder: generates DJI WPML-compliant KMZ mission files.

Generates Waypoint 3.0 (WPML/KMZ) format for the DJI Matrice 4E + Manifold 3.
Waypoint V3 uploads KMZ directly via DjiWaypointV3_UploadKmzFile() — no
programmatic mission structs needed on the PSDK side.

M4E camera has no PSDK runtime API (all '-' in compatibility table) — all
camera/gimbal control is encoded in KMZ waypoint actions. There is no runtime
cruise speed API in V3; speeds are baked per-waypoint in the WPML.

Uses the `djikmz` library for validated KMZ generation.
Our code computes waypoint positions and gimbal angles;
djikmz handles the WPML XML serialization and validation.

KMZ structure (per DJI WPML spec):
  wpmz/template.kml   — planning template (user-editable)
  wpmz/waylines.wpml   — execution instructions (what the FC flies)
"""

from __future__ import annotations

import copy
import io
import math
import os
import xml.etree.ElementTree as ET
import zipfile

from djikmz import DroneTask
from djikmz.model.mission_config import (
    DroneModel,
    FinishAction,
    MODEL_TO_VAL,
    PayloadModel,
    RCLostAction,
)
from djikmz.task_builder import DRONE_CONFIGS

from .models import (
    ActionType,
    AlgorithmConfig,
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    MAX_WAYPOINTS_PER_MISSION,
    MissionConfig,
    Waypoint,
)

# ---------------------------------------------------------------------------
# Register Matrice 4E in djikmz (library only knows M3E/M3D/M30/M300/M350).
# PROVISIONAL: droneEnumValue unknown — using M3E values [77, 0] as stand-in.
# DJI Pilot 2 validates against the connected drone at runtime, not at KMZ
# import, so the mission will still load. To discover real values: create a
# mission in DJI Pilot 2 on an M4E, export the KMZ, and inspect template.kml
# for droneEnumValue / payloadEnumValue.
# ---------------------------------------------------------------------------
_M4E_DRONE_ENUM = 77  # PROVISIONAL (same as M3E)
_M4E_DRONE_SUB_ENUM = 0  # PROVISIONAL

# Extend DroneModel str enum at runtime
_m4e_member = str.__new__(DroneModel, "M4E")
_m4e_member._name_ = "M4E"
_m4e_member._value_ = "M4E"
DroneModel._member_map_["M4E"] = _m4e_member
DroneModel._value2member_map_["M4E"] = _m4e_member

MODEL_TO_VAL[_m4e_member] = [_M4E_DRONE_ENUM, _M4E_DRONE_SUB_ENUM]

DRONE_CONFIGS["M4E"] = {
    "model": _m4e_member,
    "default_height": 80.0,
    "default_speed": 2.0,  # inspection speed
    "max_speed": 21.0,  # M4E hardware max (M3E was 15)
    "supports_rtk": True,  # M4E has RTK
    "takeoff_security_height": 5.0,
    "default_payload": PayloadModel.M3E,  # PROVISIONAL (66)
}

_DEFAULT_DRONE_MODEL = "M4E"

# WPML XML namespace
_WPML_NS = "http://www.dji.com/wpmz/1.0.3"
_KML_NS = "http://www.opengis.net/kml/2.2"


def _clamp_pitch(pitch: float) -> float:
    """Clamp gimbal pitch to hardware limits."""
    return max(GIMBAL_TILT_MIN_DEG, min(GIMBAL_TILT_MAX_DEG, pitch))


def _heading_to_djikmz(heading_deg: float) -> float:
    """Convert 0-360 heading to djikmz range (-180 to 180)."""
    h = heading_deg % 360
    if h > 180:
        h -= 360
    return h


def _build_mission(
    waypoints: list[Waypoint],
    config: MissionConfig,
    algo: AlgorithmConfig | None = None,
) -> DroneTask:
    """Build a djikmz DroneTask from our waypoints and config."""
    if algo is None:
        algo = AlgorithmConfig()

    if len(waypoints) > MAX_WAYPOINTS_PER_MISSION:
        raise ValueError(
            f"Mission has {len(waypoints)} waypoints, "
            f"exceeding max {MAX_WAYPOINTS_PER_MISSION}"
        )

    _RC_LOST_MAP = {
        "go_home": RCLostAction.GO_HOME,
        "hover": RCLostAction.HOVER,
        "land": RCLostAction.LAND,
    }
    _FINISH_MAP = {
        "return_home": FinishAction.GO_HOME,
        "hover": FinishAction.NO_ACTION,
        "land": FinishAction.AUTOLAND,
    }

    mission = DroneTask(_DEFAULT_DRONE_MODEL, "AeroScan")
    mission.name(config.mission_name)
    mission.speed(config.flight_speed_ms)

    # Safety defaults for DJI Pilot 2 (operator can adjust before flying)
    mission._mission_config.rclost_action = _RC_LOST_MAP.get(
        config.rc_lost_action, RCLostAction.GO_HOME
    )
    mission._mission_config.finish_action = _FINISH_MAP.get(
        config.finish_action, FinishAction.GO_HOME
    )
    mission._mission_config.take_off_height = config.takeoff_security_height_m

    for wp in waypoints:
        height = max(wp.z, algo.min_waypoint_height_m)
        wb = mission.fly_to(wp.lat, wp.lon, height=height)
        wb.speed(wp.speed_ms)

        # Set ellipsoidHeight (WGS84 absolute altitude) — required by WPML spec
        # when useGlobalHeight=0. wp.alt is ref_alt + wp.z from ENU→WGS84 conversion.
        wb._waypoint.ellipsoid_height = wp.alt if wp.alt != 0.0 else height

        heading = _heading_to_djikmz(wp.heading_deg)
        wb.heading(heading)

        if wp.is_transition:
            # Transit: fly through without stopping, no gimbal/photo
            wb.turn_mode("curve_and_pass")
            continue

        # Inspection: set gimbal, take photo
        # M4E mechanical shutter allows fly-through capture at slow speeds
        wb.turn_mode("curve_and_stop" if config.stop_at_waypoint else "curve_and_pass")
        # gimbal_yaw_deg=None means "follow aircraft heading" (don't command yaw).
        # When set, it's absolute degrees from geographic north (WPML headingBase=north).
        wb.gimbal_rotate(
            pitch=_clamp_pitch(wp.gimbal_pitch_deg),
            yaw=wp.gimbal_yaw_deg,
        )

        for action in wp.actions:
            if action.action_type == ActionType.TAKE_PHOTO:
                wb.take_photo(f"wp{wp.index}")
            elif action.action_type == ActionType.HOVER:
                wb.hover(action.hover_time_s)

    return mission


# ---------------------------------------------------------------------------
# XML post-processing: inject WPML elements that djikmz doesn't generate
# ---------------------------------------------------------------------------

def _inject_template_elements(root: ET.Element, config: MissionConfig) -> None:
    """Inject missing required WPML elements into template.kml DOM."""
    ns = {"wpml": _WPML_NS, "kml": _KML_NS}

    doc = root.find("kml:Document", ns)
    if doc is None:
        doc = root.find("Document")
    if doc is None:
        return

    mission_config = doc.find("wpml:missionConfig", ns)
    if mission_config is None:
        return

    # globalTransitionalSpeed — REQUIRED by WPML spec. Speed (m/s) to fly from
    # current position to first waypoint. djikmz doesn't generate this.
    if mission_config.find(f"{{{_WPML_NS}}}globalTransitionalSpeed") is None:
        el = ET.SubElement(mission_config, f"{{{_WPML_NS}}}globalTransitionalSpeed")
        el.text = str(config.flight_speed_ms)


def _generate_waylines_wpml(template_root: ET.Element, config: MissionConfig) -> ET.Element:
    """Generate waylines.wpml DOM from template.kml DOM.

    waylines.wpml is the execution file — what the flight controller actually
    flies. It resolves all global/local toggles and uses executeHeight instead
    of height.
    """
    # Deep copy the template DOM
    root = copy.deepcopy(template_root)
    ns = {"wpml": _WPML_NS, "kml": _KML_NS}

    doc = root.find("kml:Document", ns)
    if doc is None:
        doc = root.find("Document")
    if doc is None:
        return root

    # Add globalRTHHeight to missionConfig (required in waylines.wpml)
    mission_config = doc.find("wpml:missionConfig", ns)
    if mission_config is not None:
        if mission_config.find(f"{{{_WPML_NS}}}globalRTHHeight") is None:
            el = ET.SubElement(mission_config, f"{{{_WPML_NS}}}globalRTHHeight")
            el.text = str(config.takeoff_security_height_m)

    # Process each Folder (wayline) — Folder is in default KML namespace
    for folder in doc.findall(f"{{{_KML_NS}}}Folder"):
        # Add waylineId (required in waylines.wpml)
        if folder.find(f"{{{_WPML_NS}}}waylineId") is None:
            el = ET.SubElement(folder, f"{{{_WPML_NS}}}waylineId")
            el.text = "0"

        # Add executeHeightMode (required in waylines.wpml)
        if folder.find(f"{{{_WPML_NS}}}executeHeightMode") is None:
            el = ET.SubElement(folder, f"{{{_WPML_NS}}}executeHeightMode")
            el.text = "relativeToStartPoint"

        # Process each waypoint Placemark (also in KML namespace)
        for placemark in folder.findall(f"{{{_KML_NS}}}Placemark"):
            # Add executeHeight from height value
            height_el = placemark.find(f"{{{_WPML_NS}}}height")
            if height_el is not None:
                execute_el = ET.SubElement(placemark, f"{{{_WPML_NS}}}executeHeight")
                execute_el.text = height_el.text

            # Remove useGlobal* flags (waylines.wpml has everything explicit)
            for tag_suffix in [
                "useGlobalHeight", "useGlobalSpeed",
                "useGlobalHeadingParam", "useGlobalTurnParam",
            ]:
                el = placemark.find(f"{{{_WPML_NS}}}{tag_suffix}")
                if el is not None:
                    placemark.remove(el)

    return root


def _build_kmz_zip(
    waypoints: list[Waypoint],
    config: MissionConfig,
    algo: AlgorithmConfig | None = None,
) -> bytes:
    """Build a complete KMZ (ZIP) with both template.kml and waylines.wpml."""
    mission = _build_mission(waypoints, config, algo)
    kml = mission.build()
    template_xml_str = kml.to_xml()

    # Register namespaces so ET doesn't mangle prefixes
    ET.register_namespace("", _KML_NS)
    ET.register_namespace("wpml", _WPML_NS)

    # Parse and inject missing required elements into template.kml
    template_root = ET.fromstring(template_xml_str)
    _inject_template_elements(template_root, config)
    template_xml = ET.tostring(template_root, encoding="unicode", xml_declaration=True)

    # Generate waylines.wpml from the (already-patched) template
    waylines_root = _generate_waylines_wpml(template_root, config)
    waylines_xml = ET.tostring(waylines_root, encoding="unicode", xml_declaration=True)

    # Write both files to ZIP
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("wpmz/template.kml", template_xml)
        zf.writestr("wpmz/waylines.wpml", waylines_xml)
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def build_kmz(
    waypoints: list[Waypoint],
    config: MissionConfig,
    output_path: str,
    algo: AlgorithmConfig | None = None,
) -> str:
    """Generate a DJI WPML-compliant KMZ file.

    Args:
        waypoints: Ordered list of mission waypoints.
        config: Mission configuration.
        output_path: Path to write the .kmz file.
        algo: Algorithm configuration overrides.

    Returns:
        The absolute path to the generated KMZ file.
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    data = _build_kmz_zip(waypoints, config, algo)
    with open(output_path, "wb") as f:
        f.write(data)
    return os.path.abspath(output_path)


def build_kmz_bytes(
    waypoints: list[Waypoint],
    config: MissionConfig,
    algo: AlgorithmConfig | None = None,
) -> bytes:
    """Generate a DJI WPML-compliant KMZ file as bytes (in-memory)."""
    return _build_kmz_zip(waypoints, config, algo)
