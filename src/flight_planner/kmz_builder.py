"""KMZ builder: generates DJI WPML-compliant KMZ mission files.

Uses the `djikmz` library for validated KMZ generation.
Our code computes waypoint positions and gimbal angles;
djikmz handles the WPML XML serialization and validation.
"""

from __future__ import annotations

import os
import tempfile

from djikmz import DroneTask

from .models import ActionType, MissionConfig, Waypoint

# Map our CameraName to djikmz lens identifiers
_LENS_MAP = {
    "wide": "wide",
    "medium_tele": "zoom",
    "telephoto": "zoom",
}

# TODO: confirm droneEnumValue for Matrice 4E — using M3E as closest match
_DEFAULT_DRONE_MODEL = "M3E"


def _build_mission(
    waypoints: list[Waypoint],
    config: MissionConfig,
) -> DroneTask:
    """Build a djikmz DroneTask from our waypoints and config."""
    mission = DroneTask(_DEFAULT_DRONE_MODEL, "AeroScan")
    mission.name(config.mission_name)
    mission.speed(config.flight_speed_ms)

    for wp in waypoints:
        # Add waypoint
        wb = mission.fly_to(wp.lat, wp.lon, height=wp.z)
        wb.speed(wp.speed_ms)

        # Set aircraft heading to face the facade
        heading = wp.heading_deg
        # djikmz expects -180 to 180
        if heading > 180:
            heading -= 360
        wb.heading(heading)

        # Set gimbal orientation
        wb.gimbal_rotate(pitch=wp.gimbal_pitch_deg, yaw=wp.gimbal_yaw_deg)

        # Execute actions
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
