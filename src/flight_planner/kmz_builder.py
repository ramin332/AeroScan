"""KMZ builder: generates DJI WPML-compliant KMZ mission files.

Output is a ZIP archive (.kmz) containing:
  wpmz/template.kml   — mission template (for DJI Pilot 2)
  wpmz/waylines.wpml  — executable waylines (for PSDK/MSDK)
"""

from __future__ import annotations

import io
import os
import zipfile

from lxml import etree

from .models import ActionType, CameraAction, CameraName, MissionConfig, Waypoint

# XML namespaces
KML_NS = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.2"

NSMAP = {
    None: KML_NS,
    "wpml": WPML_NS,
}

# Map CameraName to DJI payloadLensIndex values
_LENS_INDEX = {
    CameraName.WIDE: "wide",
    CameraName.MEDIUM_TELE: "zoom",
    CameraName.TELEPHOTO: "zoom",
}


def _wpml(tag: str) -> str:
    """Create a namespaced WPML element tag."""
    return f"{{{WPML_NS}}}{tag}"


def _add_sub(parent: etree._Element, tag: str, text: str | None = None) -> etree._Element:
    """Add a sub-element with optional text content."""
    el = etree.SubElement(parent, tag)
    if text is not None:
        el.text = str(text)
    return el


def _build_action_group(
    parent: etree._Element,
    action_group_id: int,
    waypoint_index: int,
    actions: list[CameraAction],
) -> None:
    """Build an actionGroup element for a waypoint's actions.

    Args:
        action_group_id: Globally unique ID across the entire mission.
        waypoint_index: Index of the waypoint this group applies to.
        actions: List of actions to perform.
    """
    if not actions:
        return

    action_group = _add_sub(parent, _wpml("actionGroup"))
    _add_sub(action_group, _wpml("actionGroupId"), str(action_group_id))
    _add_sub(action_group, _wpml("actionGroupStartIndex"), str(waypoint_index))
    _add_sub(action_group, _wpml("actionGroupEndIndex"), str(waypoint_index))
    _add_sub(action_group, _wpml("actionGroupMode"), "sequence")

    trigger = _add_sub(action_group, _wpml("actionTrigger"))
    _add_sub(trigger, _wpml("actionTriggerType"), "reachPoint")

    for action_idx, action in enumerate(actions):
        action_el = _add_sub(action_group, _wpml("action"))
        _add_sub(action_el, _wpml("actionId"), str(action_idx))
        _add_sub(action_el, _wpml("actionActuatorFunc"), action.action_type.value)

        param = _add_sub(action_el, _wpml("actionActuatorFuncParam"))

        if action.action_type == ActionType.TAKE_PHOTO:
            _add_sub(param, _wpml("payloadPositionIndex"), "0")
            lens = _LENS_INDEX.get(action.camera, "wide")
            _add_sub(param, _wpml("payloadLensIndex"), lens)
            _add_sub(param, _wpml("useGlobalPayloadLensIndex"), "0")

        elif action.action_type == ActionType.GIMBAL_ROTATE:
            _add_sub(param, _wpml("gimbalHeadingYawBase"), "aircraft")
            _add_sub(param, _wpml("gimbalRotateMode"), "absoluteAngle")
            _add_sub(param, _wpml("gimbalPitchRotateEnable"), "1")
            _add_sub(param, _wpml("gimbalPitchRotateAngle"), f"{action.gimbal_pitch_deg:.1f}")
            _add_sub(param, _wpml("gimbalRollRotateEnable"), "0")
            _add_sub(param, _wpml("gimbalRollRotateAngle"), "0")
            _add_sub(param, _wpml("gimbalYawRotateEnable"), "1")
            _add_sub(param, _wpml("gimbalYawRotateAngle"), f"{action.gimbal_yaw_deg:.1f}")
            _add_sub(param, _wpml("payloadPositionIndex"), "0")

        elif action.action_type == ActionType.ROTATE_YAW:
            _add_sub(param, _wpml("aircraftHeading"), f"{action.aircraft_heading_deg:.1f}")
            _add_sub(param, _wpml("aircraftPathMode"), "counterClockwise")

        elif action.action_type == ActionType.HOVER:
            _add_sub(param, _wpml("hoverTime"), str(int(action.hover_time_s)))


def _build_mission_config(parent: etree._Element, config: MissionConfig) -> None:
    """Build the wpml:missionConfig element (shared by template.kml and waylines.wpml)."""
    mc = _add_sub(parent, _wpml("missionConfig"))
    _add_sub(mc, _wpml("flyToWaylineMode"), "safely")
    _add_sub(mc, _wpml("finishAction"), "goHome")
    _add_sub(mc, _wpml("exitOnRCLost"), "executeLostAction")
    _add_sub(mc, _wpml("executeRCLostAction"), "goBack")
    _add_sub(mc, _wpml("takeOffSecurityHeight"), "20")
    _add_sub(mc, _wpml("globalTransitionalSpeed"), f"{config.flight_speed_ms:.1f}")

    # Drone info
    drone_info = _add_sub(mc, _wpml("droneInfo"))
    # TODO: confirm droneEnumValue for Matrice 4E
    _add_sub(drone_info, _wpml("droneEnumValue"), str(config.drone_enum_value))
    _add_sub(drone_info, _wpml("droneSubEnumValue"), "0")

    # Payload info
    payload_info = _add_sub(mc, _wpml("payloadInfo"))
    _add_sub(payload_info, _wpml("payloadEnumValue"), str(config.payload_enum_value))
    _add_sub(payload_info, _wpml("payloadSubEnumValue"), "0")
    _add_sub(payload_info, _wpml("payloadPositionIndex"), "0")


def _build_folder(
    parent: etree._Element,
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> None:
    """Build a Folder element containing all waypoint Placemarks."""
    folder = _add_sub(parent, "Folder")
    _add_sub(folder, _wpml("templateId"), "0")
    _add_sub(folder, _wpml("executeHeightMode"), "relativeToStartPoint")
    _add_sub(folder, _wpml("waylineId"), "0")
    _add_sub(folder, _wpml("autoFlightSpeed"), f"{config.flight_speed_ms:.1f}")
    _add_sub(folder, _wpml("templateType"), "waypoint")
    _add_sub(folder, _wpml("distance"), "0")
    _add_sub(folder, _wpml("duration"), "0")

    for wp in waypoints:
        placemark = _add_sub(folder, "Placemark")

        point = _add_sub(placemark, "Point")
        # DJI coordinate format: lon,lat,alt
        _add_sub(point, "coordinates", f"{wp.lon:.8f},{wp.lat:.8f},{wp.alt:.2f}")

        _add_sub(placemark, _wpml("index"), str(wp.index))
        _add_sub(placemark, _wpml("executeHeight"), f"{wp.z:.2f}")
        _add_sub(placemark, _wpml("waypointSpeed"), f"{wp.speed_ms:.1f}")

        # Waypoint heading — fixed to face the facade
        heading_param = _add_sub(placemark, _wpml("waypointHeadingParam"))
        _add_sub(heading_param, _wpml("waypointHeadingMode"), "fixed")
        _add_sub(heading_param, _wpml("waypointHeadingAngle"), f"{wp.heading_deg:.1f}")
        _add_sub(heading_param, _wpml("waypointHeadingPathMode"), "counterClockwise")

        # Waypoint turn — stop at each point for inspection photos
        turn_param = _add_sub(placemark, _wpml("waypointTurnParam"))
        _add_sub(turn_param, _wpml("waypointTurnMode"), "toPointAndStopWithDiscontinuityCurvature")
        _add_sub(turn_param, _wpml("waypointTurnDampingDist"), "0.2")

        _add_sub(placemark, _wpml("useStraightLine"), "1")

        # Actions: gimbal rotate first, then waypoint-specific actions
        gimbal_action = CameraAction(
            action_type=ActionType.GIMBAL_ROTATE,
            gimbal_pitch_deg=wp.gimbal_pitch_deg,
            gimbal_yaw_deg=wp.gimbal_yaw_deg,
        )
        all_actions = [gimbal_action] + wp.actions
        # actionGroupId must be globally unique — use waypoint index
        _build_action_group(placemark, wp.index, wp.index, all_actions)


def _build_template_kml(
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> bytes:
    """Build the template.kml XML document."""
    kml = etree.Element("kml", nsmap=NSMAP)
    document = _add_sub(kml, "Document")
    _build_mission_config(document, config)
    _build_folder(document, waypoints, config)
    return etree.tostring(kml, xml_declaration=True, encoding="UTF-8", pretty_print=True)


def _build_waylines_wpml(
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> bytes:
    """Build the waylines.wpml XML document.

    Mirrors template.kml with all parameters explicitly specified per waypoint.
    The drone firmware reads this file directly for PSDK missions.
    """
    kml = etree.Element("kml", nsmap=NSMAP)
    document = _add_sub(kml, "Document")
    _build_mission_config(document, config)
    _build_folder(document, waypoints, config)
    return etree.tostring(kml, xml_declaration=True, encoding="UTF-8", pretty_print=True)


def build_kmz(
    waypoints: list[Waypoint],
    config: MissionConfig,
    output_path: str,
) -> str:
    """Generate a DJI WPML-compliant KMZ file.

    Args:
        waypoints: Ordered list of mission waypoints.
        config: Mission configuration.
        output_path: Path to write the .kmz file.

    Returns:
        The absolute path to the generated KMZ file.
    """
    template_kml = _build_template_kml(waypoints, config)
    waylines_wpml = _build_waylines_wpml(waypoints, config)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("wpmz/template.kml", template_kml)
        zf.writestr("wpmz/waylines.wpml", waylines_wpml)

    return os.path.abspath(output_path)


def build_kmz_bytes(
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> bytes:
    """Generate a DJI WPML-compliant KMZ file as bytes (in-memory)."""
    template_kml = _build_template_kml(waypoints, config)
    waylines_wpml = _build_waylines_wpml(waypoints, config)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("wpmz/template.kml", template_kml)
        zf.writestr("wpmz/waylines.wpml", waylines_wpml)

    return buf.getvalue()
