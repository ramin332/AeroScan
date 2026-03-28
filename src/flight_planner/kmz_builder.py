"""KMZ builder: generates DJI WPML-compliant KMZ mission files.

Uses the `djikmz` library for validated KMZ generation.
Our code computes waypoint positions and gimbal angles;
djikmz handles the WPML XML serialization and validation.
"""

from __future__ import annotations

import os
import tempfile

from djikmz import DroneTask
from djikmz.model.mission_config import FinishAction, RCLostAction

from .models import (
    ActionType,
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    MAX_WAYPOINTS_PER_MISSION,
    MissionConfig,
    Waypoint,
)

# TODO: confirm droneEnumValue for Matrice 4E — using M3E as closest match
_DEFAULT_DRONE_MODEL = "M3E"


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
) -> DroneTask:
    """Build a djikmz DroneTask from our waypoints and config."""
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
        wb = mission.fly_to(wp.lat, wp.lon, height=max(wp.z, 2.0))
        wb.speed(wp.speed_ms)

        heading = _heading_to_djikmz(wp.heading_deg)
        wb.heading(heading)

        if wp.is_transition:
            # Transit: fly through without stopping, no gimbal/photo
            wb.turn_mode("curve_and_pass")
            continue

        # Inspection: stop at waypoint, set gimbal, take photo
        wb.turn_mode("curve_and_stop")
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
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    mission = _build_mission(waypoints, config)
    mission.to_kmz(output_path)
    return os.path.abspath(output_path)


def build_kmz_bytes(
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> bytes:
    """Generate a DJI WPML-compliant KMZ file as bytes (in-memory)."""
    with tempfile.NamedTemporaryFile(suffix=".kmz", delete=False) as f:
        tmp_path = f.name

    try:
        mission = _build_mission(waypoints, config)
        mission.to_kmz(tmp_path)
        with open(tmp_path, "rb") as f:
            return f.read()
    finally:
        os.unlink(tmp_path)
