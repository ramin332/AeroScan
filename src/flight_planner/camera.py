"""Camera calculations: GSD, footprint, overlap-to-spacing conversion."""

from __future__ import annotations

from dataclasses import dataclass

from .models import CAMERAS, CameraName, CameraSpec


@dataclass
class PhotoFootprint:
    """Dimensions of the area captured by a single photo."""

    width_m: float  # horizontal extent on the target surface
    height_m: float  # vertical extent on the target surface


def get_camera(name: CameraName) -> CameraSpec:
    """Get camera specification by name."""
    return CAMERAS[name]


def compute_gsd(camera: CameraSpec, distance_m: float) -> float:
    """Compute ground sample distance (mm/pixel) at a given distance.

    GSD = (distance * sensor_width) / (focal_length * image_width) * 1000
    """
    return (distance_m * camera.sensor_width_mm) / (
        camera.focal_length_mm * camera.image_width_px
    ) * 1000


def compute_distance_for_gsd(camera: CameraSpec, target_gsd_mm_per_px: float) -> float:
    """Compute the camera-to-surface distance needed to achieve a target GSD.

    distance = (focal_length * target_gsd * image_width) / (sensor_width * 1000)
    """
    return (
        camera.focal_length_mm * target_gsd_mm_per_px * camera.image_width_px
    ) / (camera.sensor_width_mm * 1000)


def compute_footprint(camera: CameraSpec, distance_m: float) -> PhotoFootprint:
    """Compute the photo footprint at a given distance.

    footprint_width = distance * sensor_width / focal_length
    footprint_height = distance * sensor_height / focal_length
    """
    width = distance_m * camera.sensor_width_mm / camera.focal_length_mm
    height = distance_m * camera.sensor_height_mm / camera.focal_length_mm
    return PhotoFootprint(width_m=width, height_m=height)


def compute_grid_spacing(
    footprint: PhotoFootprint,
    front_overlap: float = 0.80,
    side_overlap: float = 0.70,
) -> tuple[float, float]:
    """Compute waypoint grid spacing from footprint and overlap requirements.

    Returns (horizontal_step_m, vertical_step_m).

    horizontal_step = footprint_width * (1 - side_overlap)
    vertical_step = footprint_height * (1 - front_overlap)
    """
    horizontal_step = footprint.width_m * (1 - side_overlap)
    vertical_step = footprint.height_m * (1 - front_overlap)
    return horizontal_step, vertical_step


def compute_photo_interval_distance(
    camera: CameraSpec,
    flight_speed_ms: float,
) -> float:
    """Compute the minimum distance between photos based on camera interval and speed.

    This ensures the camera has time to capture between waypoints.
    """
    return camera.min_interval_s * flight_speed_ms
