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
# M4E drone / payload enum values — PROVISIONAL until confirmed against a
# DJI-exported M4E mission. To discover the real values: create a mission in
# DJI Pilot 2 with an M4E connected, export the KMZ, then run:
#   python -m flight_planner.tools.inspect_kmz <exported.kmz> --patch
# That tool prints the discovered values and rewrites these constants.
# ---------------------------------------------------------------------------
M4E_DRONE_ENUM = 77       # PROVISIONAL — same as M3E
M4E_DRONE_SUB_ENUM = 0    # PROVISIONAL
M4E_PAYLOAD_ENUM = 66     # PROVISIONAL — same as M3E payload
M4E_PAYLOAD_SUB_ENUM = 0  # PROVISIONAL

# Register Matrice 4E in djikmz (library only ships M3E/M3D/M30/M300/M350).
# djikmz's emitted enum values get overwritten in post-processing by
# _inject_m4e_enums so these seed values are only a placeholder for the
# library's internal validation.
_m4e_member = str.__new__(DroneModel, "M4E")
_m4e_member._name_ = "M4E"
_m4e_member._value_ = "M4E"
DroneModel._member_map_["M4E"] = _m4e_member
DroneModel._value2member_map_["M4E"] = _m4e_member

MODEL_TO_VAL[_m4e_member] = [M4E_DRONE_ENUM, M4E_DRONE_SUB_ENUM]

DRONE_CONFIGS["M4E"] = {
    "model": _m4e_member,
    "default_height": 80.0,
    "default_speed": 2.0,
    "max_speed": 21.0,
    "supports_rtk": True,
    "takeoff_security_height": 5.0,
    "default_payload": PayloadModel.M3E,
}

_DEFAULT_DRONE_MODEL = "M4E"

# WPML XML namespace — 1.0.6 matches what DJI Pilot 2 emits on current firmware.
# djikmz internally writes 1.0.3; _rewrite_wpml_version bumps the output.
_WPML_NS_LEGACY = "http://www.dji.com/wpmz/1.0.3"
_WPML_NS = "http://www.dji.com/wpmz/1.0.6"
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


def _rewrite_wpml_version(xml_str: str) -> str:
    """Bump djikmz's emitted WPML namespace to the current spec version."""
    return xml_str.replace(_WPML_NS_LEGACY, _WPML_NS)


def _inject_m4e_enums(root: ET.Element) -> None:
    """Force authoritative M4E drone/payload enum values into droneInfo + payloadInfo.

    djikmz emits values based on its own enum table (derived from M3E). We
    overwrite them here so the output matches what DJI Pilot 2 expects for an
    M4E. Also emits payloadSubEnumValue, which djikmz omits.
    """
    for drone_info in root.iter(f"{{{_WPML_NS}}}droneInfo"):
        for tag_name, value in (
            ("droneEnumValue", M4E_DRONE_ENUM),
            ("droneSubEnumValue", M4E_DRONE_SUB_ENUM),
        ):
            el = drone_info.find(f"{{{_WPML_NS}}}{tag_name}")
            if el is None:
                el = ET.SubElement(drone_info, f"{{{_WPML_NS}}}{tag_name}")
            el.text = str(value)

    for payload_info in root.iter(f"{{{_WPML_NS}}}payloadInfo"):
        for tag_name, value in (
            ("payloadEnumValue", M4E_PAYLOAD_ENUM),
            ("payloadSubEnumValue", M4E_PAYLOAD_SUB_ENUM),
        ):
            el = payload_info.find(f"{{{_WPML_NS}}}{tag_name}")
            if el is None:
                el = ET.SubElement(payload_info, f"{{{_WPML_NS}}}{tag_name}")
            el.text = str(value)


def _signed_angle_delta(a: float, b: float) -> float:
    """Smallest signed difference between two headings in degrees (handles 360° wraparound)."""
    d = (a - b) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def _dedupe_pose_actions(root: ET.Element, config: MissionConfig) -> None:
    """Strip redundant gimbalRotate + rotateYaw actions whose pose matches the last emitted one within threshold.

    Facade sweeps hold gimbal pitch/yaw constant along every row — re-emitting
    a fresh gimbalRotate at every waypoint balloons the XML without changing
    behaviour. Same for aircraft heading on long parallel passes. Removing the
    redundant actions cuts action-element count by ~40-60% on typical missions
    and sidesteps the Pilot 2 renderer stall seen at high WP counts.
    """
    gimbal_thr = config.gimbal_dedup_threshold_deg
    heading_thr = config.heading_dedup_threshold_deg

    last_pitch: float | None = None
    last_yaw: float | None = None
    last_heading: float | None = None

    for placemark in root.iter(f"{{{_KML_NS}}}Placemark"):
        for action_group in placemark.findall(f"{{{_WPML_NS}}}actionGroup"):
            for action in list(action_group.findall(f"{{{_WPML_NS}}}action")):
                func_el = action.find(f"{{{_WPML_NS}}}actionActuatorFunc")
                params = action.find(f"{{{_WPML_NS}}}actionActuatorFuncParam")
                if func_el is None or params is None:
                    continue
                func = func_el.text

                if func == "gimbalRotate":
                    pitch_el = params.find(f"{{{_WPML_NS}}}gimbalPitchRotateAngle")
                    yaw_en_el = params.find(f"{{{_WPML_NS}}}gimbalYawRotateEnable")
                    yaw_el = params.find(f"{{{_WPML_NS}}}gimbalYawRotateAngle")
                    pitch = (
                        float(pitch_el.text) if pitch_el is not None and pitch_el.text else None
                    )
                    yaw = float(yaw_el.text) if yaw_el is not None and yaw_el.text else None
                    yaw_enabled = (
                        yaw_en_el is not None and (yaw_en_el.text or "").strip() == "1"
                    )

                    pitch_same = (
                        last_pitch is not None
                        and pitch is not None
                        and abs(pitch - last_pitch) <= gimbal_thr
                    )
                    yaw_same = (not yaw_enabled) or (
                        last_yaw is not None
                        and yaw is not None
                        and abs(_signed_angle_delta(yaw, last_yaw)) <= gimbal_thr
                    )
                    if pitch_same and yaw_same:
                        action_group.remove(action)
                        continue

                    if pitch is not None:
                        last_pitch = pitch
                    if yaw_enabled and yaw is not None:
                        last_yaw = yaw

                elif func == "rotateYaw":
                    heading_el = params.find(f"{{{_WPML_NS}}}aircraftHeading")
                    heading = (
                        float(heading_el.text)
                        if heading_el is not None and heading_el.text
                        else None
                    )
                    if (
                        last_heading is not None
                        and heading is not None
                        and abs(_signed_angle_delta(heading, last_heading)) <= heading_thr
                    ):
                        action_group.remove(action)
                        continue
                    if heading is not None:
                        last_heading = heading


def _renumber_action_ids(root: ET.Element) -> None:
    """Renumber actionIds sequentially within each actionGroup after dedup leaves gaps."""
    for action_group in root.iter(f"{{{_WPML_NS}}}actionGroup"):
        for idx, action in enumerate(action_group.findall(f"{{{_WPML_NS}}}action")):
            id_el = action.find(f"{{{_WPML_NS}}}actionId")
            if id_el is None:
                id_el = ET.SubElement(action, f"{{{_WPML_NS}}}actionId")
            id_el.text = str(idx)


def _path_distance_m(waypoints: list[Waypoint]) -> float:
    """Total 3D path length through the waypoints, in metres (ENU frame)."""
    total = 0.0
    for i in range(1, len(waypoints)):
        a, b = waypoints[i - 1], waypoints[i]
        total += math.sqrt(
            (b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2
        )
    return total


def _estimate_duration_s(waypoints: list[Waypoint], config: MissionConfig) -> float:
    """Rough flight-time estimate: cruise time + per-waypoint action overhead.

    DJI decelerates at turns and pauses for photo actions, so this is an
    approximation — good enough for the wpml:duration display field.
    """
    dist = _path_distance_m(waypoints)
    speed = max(config.flight_speed_ms, 0.1)
    per_wp_overhead = 0.5
    return dist / speed + len(waypoints) * per_wp_overhead


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

    # waylineAvoidLimitAreaMode — controls geofence avoidance behaviour.
    # 0 = do not avoid (default in DJI-exported KMZs).
    if mission_config.find(f"{{{_WPML_NS}}}waylineAvoidLimitAreaMode") is None:
        el = ET.SubElement(
            mission_config, f"{{{_WPML_NS}}}waylineAvoidLimitAreaMode"
        )
        el.text = "0"


def _build_start_action_group(initial_pitch_deg: float) -> ET.Element:
    """Build the <wpml:startActionGroup> that primes the camera before flight.

    Mirrors DJI-exported waylines: rotate gimbal to the first waypoint's pitch,
    short hover to let the gimbal settle, then switch camera to manual focus.
    """
    group = ET.Element(f"{{{_WPML_NS}}}startActionGroup")

    a0 = ET.SubElement(group, f"{{{_WPML_NS}}}action")
    ET.SubElement(a0, f"{{{_WPML_NS}}}actionId").text = "0"
    ET.SubElement(a0, f"{{{_WPML_NS}}}actionActuatorFunc").text = "gimbalRotate"
    p0 = ET.SubElement(a0, f"{{{_WPML_NS}}}actionActuatorFuncParam")
    for tag, val in (
        ("gimbalHeadingYawBase", "aircraft"),
        ("gimbalRotateMode", "absoluteAngle"),
        ("gimbalPitchRotateEnable", "1"),
        ("gimbalPitchRotateAngle", f"{_clamp_pitch(initial_pitch_deg):g}"),
        ("gimbalRollRotateEnable", "0"),
        ("gimbalRollRotateAngle", "0"),
        ("gimbalYawRotateEnable", "1"),
        ("gimbalYawRotateAngle", "0"),
        ("gimbalRotateTimeEnable", "0"),
        ("gimbalRotateTime", "10"),
        ("payloadPositionIndex", "0"),
    ):
        ET.SubElement(p0, f"{{{_WPML_NS}}}{tag}").text = val

    a1 = ET.SubElement(group, f"{{{_WPML_NS}}}action")
    ET.SubElement(a1, f"{{{_WPML_NS}}}actionId").text = "1"
    ET.SubElement(a1, f"{{{_WPML_NS}}}actionActuatorFunc").text = "hover"
    p1 = ET.SubElement(a1, f"{{{_WPML_NS}}}actionActuatorFuncParam")
    ET.SubElement(p1, f"{{{_WPML_NS}}}hoverTime").text = "0.5"

    a2 = ET.SubElement(group, f"{{{_WPML_NS}}}action")
    ET.SubElement(a2, f"{{{_WPML_NS}}}actionId").text = "2"
    ET.SubElement(a2, f"{{{_WPML_NS}}}actionActuatorFunc").text = "setFocusType"
    p2 = ET.SubElement(a2, f"{{{_WPML_NS}}}actionActuatorFuncParam")
    ET.SubElement(p2, f"{{{_WPML_NS}}}cameraFocusType").text = "manual"
    ET.SubElement(p2, f"{{{_WPML_NS}}}payloadPositionIndex").text = "0"

    return group


def _generate_waylines_wpml(
    template_root: ET.Element,
    config: MissionConfig,
    waypoints: list[Waypoint],
) -> ET.Element:
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

    initial_pitch = waypoints[0].gimbal_pitch_deg if waypoints else 0.0
    total_distance = _path_distance_m(waypoints)
    total_duration = _estimate_duration_s(waypoints, config)

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

        # Distance + duration metadata (shown in DJI Pilot 2 mission summary)
        if folder.find(f"{{{_WPML_NS}}}distance") is None:
            el = ET.SubElement(folder, f"{{{_WPML_NS}}}distance")
            el.text = f"{total_distance:.3f}"
        if folder.find(f"{{{_WPML_NS}}}duration") is None:
            el = ET.SubElement(folder, f"{{{_WPML_NS}}}duration")
            el.text = f"{total_duration:.3f}"

        # startActionGroup — prime gimbal + camera focus before first waypoint
        if folder.find(f"{{{_WPML_NS}}}startActionGroup") is None:
            folder.append(_build_start_action_group(initial_pitch))

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
    template_xml_str = _rewrite_wpml_version(kml.to_xml())

    # Register namespaces so ET doesn't mangle prefixes
    ET.register_namespace("", _KML_NS)
    ET.register_namespace("wpml", _WPML_NS)

    # Parse and inject missing required elements into template.kml
    template_root = ET.fromstring(template_xml_str)
    _inject_template_elements(template_root, config)
    _inject_m4e_enums(template_root)
    _dedupe_pose_actions(template_root, config)
    _renumber_action_ids(template_root)
    template_xml = ET.tostring(template_root, encoding="unicode", xml_declaration=True)

    # Generate waylines.wpml from the (already-patched) template
    waylines_root = _generate_waylines_wpml(template_root, config, waypoints)
    _inject_m4e_enums(waylines_root)
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
