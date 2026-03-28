"""Data models for the AeroScan flight planner."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class RoofType(Enum):
    FLAT = "flat"
    PITCHED = "pitched"


class CameraName(Enum):
    WIDE = "wide"
    MEDIUM_TELE = "medium_tele"
    TELEPHOTO = "telephoto"


class ActionType(Enum):
    TAKE_PHOTO = "takePhoto"
    GIMBAL_ROTATE = "gimbalRotate"
    ROTATE_YAW = "rotateYaw"
    HOVER = "hover"


@dataclass
class CameraSpec:
    """Matrice 4E camera specifications."""

    name: CameraName
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float
    image_width_px: int
    image_height_px: int
    fov_deg: float
    min_interval_s: float


# Matrice 4E camera definitions
CAMERAS: dict[CameraName, CameraSpec] = {
    CameraName.WIDE: CameraSpec(
        name=CameraName.WIDE,
        focal_length_mm=24,
        sensor_width_mm=17.3,
        sensor_height_mm=13.0,
        image_width_px=5280,
        image_height_px=3956,
        fov_deg=84,
        min_interval_s=0.5,
    ),
    CameraName.MEDIUM_TELE: CameraSpec(
        name=CameraName.MEDIUM_TELE,
        focal_length_mm=70,
        sensor_width_mm=9.6,
        sensor_height_mm=7.2,
        image_width_px=8064,
        image_height_px=6048,
        fov_deg=35,
        min_interval_s=0.7,
    ),
    CameraName.TELEPHOTO: CameraSpec(
        name=CameraName.TELEPHOTO,
        focal_length_mm=168,
        sensor_width_mm=8.8,
        sensor_height_mm=6.6,
        image_width_px=8192,
        image_height_px=6144,
        fov_deg=15,
        min_interval_s=0.7,
    ),
}

# Gimbal constraints (Matrice 4E)
GIMBAL_TILT_MIN_DEG = -90  # straight down
GIMBAL_TILT_MAX_DEG = 35  # looking up
GIMBAL_PAN_MIN_DEG = -60  # left (software-controlled via PSDK only)
GIMBAL_PAN_MAX_DEG = 60  # right

# Flight constraints
MAX_SPEED_MS = 21
INSPECTION_SPEED_MS = 3
MIN_ALTITUDE_M = 2
MAX_ALTITUDE_M = 6000
MAX_WAYPOINTS_PER_MISSION = 65535
MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN = 32
OBSTACLE_CLEARANCE_M = 2

# WGS84 geodetic coefficients for local ENU ↔ WGS84 conversion.
# Suitable for small areas (<1 km). Used by geometry, building_import, and visualize.
WGS84_LAT_A = 111132.92  # base meters per degree latitude
WGS84_LAT_B = -559.82  # 2nd-harmonic latitude correction
WGS84_LAT_C = 1.175  # 4th-harmonic latitude correction
WGS84_LON_A = 111412.84  # base meters per degree longitude
WGS84_LON_B = -93.5  # 3rd-harmonic longitude correction


def meters_per_deg(ref_lat_rad: float) -> tuple[float, float]:
    """Return (meters_per_deg_lat, meters_per_deg_lon) at a reference latitude (radians)."""
    m_lat = WGS84_LAT_A + WGS84_LAT_B * math.cos(2 * ref_lat_rad) + WGS84_LAT_C * math.cos(4 * ref_lat_rad)
    m_lon = WGS84_LON_A * math.cos(ref_lat_rad) + WGS84_LON_B * math.cos(3 * ref_lat_rad)
    return m_lat, m_lon


@dataclass
class CameraAction:
    """An action to perform at a waypoint."""

    action_type: ActionType
    camera: CameraName = CameraName.WIDE
    # Gimbal angles (for gimbalRotate)
    gimbal_pitch_deg: float = 0.0
    gimbal_yaw_deg: float = 0.0
    # Hover duration (for hover)
    hover_time_s: float = 2.0
    # Aircraft heading (for rotateYaw)
    aircraft_heading_deg: float = 0.0


@dataclass
class Waypoint:
    """A single waypoint in the mission."""

    # Position in local ENU coordinates (meters, relative to building center)
    x: float  # East
    y: float  # North
    z: float  # Up (altitude above ground)

    # Position in WGS84 (set during coordinate conversion)
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0  # altitude above WGS84 ellipsoid

    # Aircraft heading (degrees from north, clockwise)
    heading_deg: float = 0.0

    # Gimbal angles
    gimbal_pitch_deg: float = 0.0
    gimbal_yaw_deg: float = 0.0

    # Flight speed to this waypoint (m/s)
    speed_ms: float = INSPECTION_SPEED_MS

    # Actions to execute at this waypoint
    actions: list[CameraAction] = field(default_factory=list)

    # Metadata
    facade_index: int = -1
    component_tag: str = ""  # NL-SfB code, e.g. "21.1"
    is_detail_point: bool = False
    is_transition: bool = False  # transit waypoint (no photo, higher speed)
    index: int = 0  # global waypoint index, set during mission assembly


@dataclass
class Facade:
    """A planar surface of a building (wall, roof plane, soffit)."""

    # Vertices of the facade polygon in local ENU (meters)
    # Ordered counter-clockwise when viewed from outside
    vertices: np.ndarray  # shape (N, 3)

    # Outward-facing unit normal vector
    normal: np.ndarray  # shape (3,)

    # NL-SfB component type
    component_tag: str = ""  # e.g. "21.1" = brick facade

    # Facade label
    label: str = ""  # e.g. "north_wall", "roof_south"

    # Facade index in the building
    index: int = 0

    @property
    def center(self) -> np.ndarray:
        """Centroid of the facade polygon."""
        return self.vertices.mean(axis=0)

    @property
    def width(self) -> float:
        """Width of the facade (horizontal extent)."""
        # Project vertices onto the horizontal plane and compute extent
        # along the direction perpendicular to the normal in the horizontal plane
        horiz_normal = np.array([self.normal[0], self.normal[1], 0.0])
        norm = np.linalg.norm(horiz_normal)
        if norm < 1e-9:
            # Horizontal surface (roof) — use bounding box
            mins = self.vertices.min(axis=0)
            maxs = self.vertices.max(axis=0)
            dx = maxs[0] - mins[0]
            dy = maxs[1] - mins[1]
            return max(dx, dy)
        horiz_normal /= norm
        # Tangent direction along facade (horizontal)
        tangent = np.array([-horiz_normal[1], horiz_normal[0], 0.0])
        projections = self.vertices @ tangent
        return float(projections.max() - projections.min())

    @property
    def height(self) -> float:
        """Height of the facade (vertical extent for walls, slope length for roofs)."""
        if abs(self.normal[2]) > 0.99:
            # Nearly horizontal surface — use the shorter horizontal dimension
            mins = self.vertices.min(axis=0)
            maxs = self.vertices.max(axis=0)
            dx = maxs[0] - mins[0]
            dy = maxs[1] - mins[1]
            return min(dx, dy)
        # For walls and pitched roofs, use the extent along the surface's "up" direction
        z_values = self.vertices[:, 2]
        z_range = z_values.max() - z_values.min()
        if abs(self.normal[2]) < 0.01:
            # Vertical wall
            return float(z_range)
        # Pitched surface — slope length
        return float(z_range / abs(math.sqrt(1 - self.normal[2] ** 2)))

    @property
    def azimuth_deg(self) -> float:
        """Azimuth of the outward normal (degrees from north, clockwise)."""
        return float(math.degrees(math.atan2(self.normal[0], self.normal[1])) % 360)

    @property
    def tilt_from_vertical_deg(self) -> float:
        """Angle between the normal and the horizontal plane.

        0° = vertical wall (normal is horizontal)
        90° = flat roof (normal points straight up)
        """
        return float(math.degrees(math.asin(abs(self.normal[2]))))


@dataclass
class Building:
    """A building defined by its planar surfaces.

    Can be constructed from manual dimensions or imported from a 3D model.
    """

    # Reference GPS position (center of building footprint)
    lat: float = 0.0
    lon: float = 0.0
    ground_altitude: float = 0.0  # meters above WGS84 ellipsoid

    # Building dimensions (for rectangular buildings)
    width: float = 0.0  # meters (east-west extent at heading=0)
    depth: float = 0.0  # meters (north-south extent at heading=0)
    height: float = 0.0  # meters (eave height)
    heading_deg: float = 0.0  # orientation of long axis from north (clockwise)

    # Roof
    roof_type: RoofType = RoofType.FLAT
    roof_pitch_deg: float = 0.0  # angle from horizontal (for pitched roofs)

    # Number of stories (for component segmentation)
    num_stories: int = 1

    # Facades (computed from dimensions or imported)
    facades: list[Facade] = field(default_factory=list)

    # Label
    label: str = ""


@dataclass
class AlgorithmConfig:
    """Tunable algorithm parameters that are not backed by hardware specs.

    These control internal thresholds, safety margins, and heuristics
    throughout the pipeline. Defaults match the original hardcoded values.
    """

    # -- Flight time estimation (validate.py) --
    hover_time_per_wp_s: float = 1.0  # assumed hover per inspection waypoint
    takeoff_landing_overhead_s: float = 60.0  # fixed overhead for takeoff/landing
    battery_warning_threshold: float = 0.80  # fraction of max flight time → RTH warning
    battery_info_threshold: float = 0.65  # fraction of max flight time → info message
    gimbal_near_limit_deg: float = -80.0  # pitch threshold for "near nadir" info

    # -- Geometry / grid generation (geometry.py) --
    facade_edge_inset_m: float = 0.1  # margin from facade edges for waypoints
    transition_altitude_margin_m: float = 2.0  # extra height during facade transitions
    roof_normal_threshold: float = 0.5  # abs(nz) > this → surface is roof (transitions)
    min_altitude_m: float = 2.0  # safety floor for all waypoints

    # -- Mesh import (building_import.py) --
    default_building_height_m: float = 8.0  # fallback height when unknown
    min_mesh_faces: int = 4  # reject meshes with fewer faces
    downward_face_threshold: float = -0.3  # nz < this → floor/ceiling (filtered)
    ground_level_threshold_m: float = 0.3  # z < this → ground-level surface (filtered)
    occlusion_ray_offset_m: float = 0.05  # offset along normal for ray origin
    occlusion_hit_fraction: float = 0.5  # hit dist < fraction of bbox → interior wall
    flat_roof_normal_threshold: float = 0.95  # nz > this → flat roof
    wall_normal_threshold: float = 0.3  # nz < this → wall
    auto_scale_height_threshold_m: float = 50.0  # mesh height > this → assume wrong units
    auto_scale_target_height_m: float = 8.0  # rescale target when auto-scaling
    region_growing_angle_deg: float = 15.0  # coplanarity threshold for region growing

    # -- Waypoint LOS occlusion (geometry.py) --
    enable_waypoint_los: bool = True  # ray-cast LOS check per waypoint against mesh
    los_tolerance_m: float = 0.5  # hit closer than (target_dist - tolerance) = occluded
    los_min_visible_ratio: float = 0.4  # min fraction of sample rays that must reach facade

    # -- Path optimization (optimize.py) --
    enable_path_dedup: bool = True  # merge near-coincident cross-facade waypoints
    enable_path_tsp: bool = True  # 2-opt TSP facade ordering (vs greedy nearest-neighbor)
    enable_sweep_reversal: bool = True  # flip sweep direction per facade for shorter transitions
    dedup_max_gimbal_diff_deg: float = 20.0  # max gimbal angle diff for merge eligibility

    # -- KMZ export (kmz_builder.py) --
    min_waypoint_height_m: float = 2.0  # clamp waypoint Z in KMZ output


@dataclass
class MissionConfig:
    """Configuration for a flight mission."""

    # Target GSD (ground sample distance)
    target_gsd_mm_per_px: float = 2.0

    # Camera to use
    camera: CameraName = CameraName.WIDE

    # Photo overlap
    front_overlap: float = 0.80  # 80%
    side_overlap: float = 0.70  # 70%

    # Flight parameters
    flight_speed_ms: float = INSPECTION_SPEED_MS
    obstacle_clearance_m: float = OBSTACLE_CLEARANCE_M

    # Mission metadata
    mission_name: str = "AeroScan Inspection"

    # Drone enum value
    # TODO: confirm droneEnumValue for Matrice 4E
    drone_enum_value: int = 77  # placeholder, M3E=77, M4E TBD
    payload_enum_value: int = 52  # placeholder

    # Detail capture: take telephoto photos at flagged points
    enable_detail_capture: bool = False
    detail_camera: CameraName = CameraName.MEDIUM_TELE

    # Tunable constraints (exposed as frontend levers)
    gimbal_pitch_margin_deg: float = 5.0  # safety margin from hardware pitch limits
    min_photo_distance_m: float = 1.5  # min distance between photo waypoints (dedup)
    yaw_rate_deg_per_s: float = 60.0  # assumed drone yaw rate for time estimates

    # DJI Pilot 2 safety defaults (pre-populate the operator's UI)
    rc_lost_action: str = "go_home"  # "go_home" | "hover" | "land" — what drone does on signal loss
    finish_action: str = "return_home"  # "return_home" | "hover" | "land" — what drone does after last waypoint
    takeoff_security_height_m: float = 5.0  # safe climb height before starting route (building proximity)
