"""Geometry module: facade segmentation, normal computation, waypoint grid generation.

Coordinate system:
- Local ENU (East-North-Up) relative to building center GPS point.
- x = East, y = North, z = Up
- All units in meters.
"""

from __future__ import annotations

import math

import numpy as np
from pyproj import Transformer

from .camera import compute_distance_for_gsd, compute_footprint, compute_grid_spacing, get_camera
from .models import (
    ActionType,
    Building,
    CameraAction,
    CameraName,
    Facade,
    MAX_SPEED_MS,
    MIN_ALTITUDE_M,
    MissionConfig,
    RoofType,
    Waypoint,
)


def build_rectangular_building(
    lat: float,
    lon: float,
    width: float,
    depth: float,
    height: float,
    heading_deg: float = 0.0,
    roof_type: RoofType = RoofType.FLAT,
    roof_pitch_deg: float = 0.0,
    num_stories: int = 1,
    ground_altitude: float = 0.0,
    label: str = "",
) -> Building:
    """Create a Building from rectangular dimensions.

    The building footprint is centered at the origin in local ENU coordinates.
    Width is along the building's local X axis, depth along the local Y axis.
    The heading rotates the building clockwise from north.

    Args:
        lat, lon: GPS center of building footprint.
        width: Building width in meters (along the heading direction).
        depth: Building depth in meters (perpendicular to heading).
        height: Eave height in meters.
        heading_deg: Orientation of the width axis, degrees clockwise from north.
        roof_type: FLAT or PITCHED.
        roof_pitch_deg: Roof pitch angle from horizontal (for PITCHED roofs).
        num_stories: Number of stories.
        ground_altitude: Ground altitude above WGS84 ellipsoid.
        label: Building label.
    """
    building = Building(
        lat=lat,
        lon=lon,
        ground_altitude=ground_altitude,
        width=width,
        depth=depth,
        height=height,
        heading_deg=heading_deg,
        roof_type=roof_type,
        roof_pitch_deg=roof_pitch_deg,
        num_stories=num_stories,
        label=label,
    )

    # Rotation matrix for heading (clockwise from north = counterclockwise in ENU)
    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    # Heading from north (Y axis), clockwise: rotate ENU coordinates
    # A heading of 0° means width along North, depth along East
    # heading rotates: local_x -> sin(h)*E + cos(h)*N, local_y -> cos(h)*E - sin(h)*N

    def rotate(local_x: float, local_y: float) -> tuple[float, float]:
        """Rotate local building coords to ENU."""
        e = local_x * cos_t + local_y * sin_t
        n = -local_x * sin_t + local_y * cos_t
        return e, n

    hw, hd = width / 2, depth / 2

    # Footprint corners in local building coords (x=along width, y=along depth)
    # Order: SW, SE, NE, NW (looking down, with local Y pointing "forward")
    local_corners = [
        (-hw, -hd),  # 0: left-back
        (hw, -hd),   # 1: right-back
        (hw, hd),    # 2: right-front
        (-hw, hd),   # 3: left-front
    ]

    # Convert to ENU
    corners_2d = [rotate(lx, ly) for lx, ly in local_corners]

    # 3D corners at ground and eave level
    ground = np.array([[e, n, 0.0] for e, n in corners_2d])
    eave = np.array([[e, n, height] for e, n in corners_2d])

    facades = []

    # Four walls: each wall is a quad from ground to eave
    wall_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    wall_labels = ["back", "right", "front", "left"]
    # Default NL-SfB: 21.1 = brick facade
    wall_component = "21.1"

    for idx, ((i, j), wlabel) in enumerate(zip(wall_edges, wall_labels)):
        # Quad vertices: ground[i], ground[j], eave[j], eave[i]
        verts = np.array([ground[i], ground[j], eave[j], eave[i]])

        # Outward normal: cross product of edge vectors
        edge1 = ground[j] - ground[i]  # along the base of the wall
        edge2 = eave[i] - ground[i]  # up the wall
        normal = np.cross(edge1, edge2)
        normal = normal / np.linalg.norm(normal)

        # Ensure normal points outward (away from building center)
        center_to_wall = verts.mean(axis=0)
        if np.dot(normal, center_to_wall) < 0:
            normal = -normal

        facades.append(Facade(
            vertices=verts,
            normal=normal,
            component_tag=wall_component,
            label=f"{wlabel}_wall",
            index=idx,
        ))

    facade_idx = len(facades)

    if roof_type == RoofType.FLAT:
        # Flat roof: single horizontal surface
        roof_verts = eave.copy()
        roof_normal = np.array([0.0, 0.0, 1.0])
        facades.append(Facade(
            vertices=roof_verts,
            normal=roof_normal,
            component_tag="47.1",  # roof tiles / covering
            label="roof_flat",
            index=facade_idx,
        ))

    elif roof_type == RoofType.PITCHED:
        # Pitched roof: two sloped planes along the width axis
        # Ridge runs along the width (local X) direction at the center
        pitch_rad = math.radians(roof_pitch_deg)
        ridge_rise = (depth / 2) * math.tan(pitch_rad)
        ridge_height = height + ridge_rise

        # Ridge line endpoints (midpoints of front and back edges, at ridge height)
        ridge_mid_back_local = (0.0, -hd + hd)  # actually mid of back edge...
        # Ridge runs along local X at local Y=0
        ridge_left = rotate(-hw, 0.0)
        ridge_right = rotate(hw, 0.0)
        ridge_left_3d = np.array([ridge_left[0], ridge_left[1], ridge_height])
        ridge_right_3d = np.array([ridge_right[0], ridge_right[1], ridge_height])

        # South slope: eave[0]-eave[1] to ridge
        south_verts = np.array([eave[0], eave[1], ridge_right_3d, ridge_left_3d])
        # North slope: eave[2]-eave[3] to ridge (note: eave[2]=right-front, eave[3]=left-front)
        north_verts = np.array([eave[2], eave[3], ridge_left_3d, ridge_right_3d])

        for i, (verts, rlabel) in enumerate([
            (south_verts, "roof_back"),
            (north_verts, "roof_front"),
        ]):
            edge1 = verts[1] - verts[0]
            edge2 = verts[3] - verts[0]
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal)
            # Ensure normal points outward (has a positive Z component for roofs)
            if normal[2] < 0:
                normal = -normal

            facades.append(Facade(
                vertices=verts,
                normal=normal,
                component_tag="47.1",
                label=rlabel,
                index=facade_idx + i,
            ))

    building.facades = facades
    return building


def generate_waypoints_for_facade(
    facade: Facade,
    config: MissionConfig,
) -> list[Waypoint]:
    """Generate a grid of waypoints for a single facade.

    The grid is parallel to the facade plane, offset along the outward normal
    by the distance needed to achieve the target GSD. Waypoints are ordered
    in a boustrophedon (lawnmower) pattern.
    """
    camera = get_camera(config.camera)
    distance = compute_distance_for_gsd(camera, config.target_gsd_mm_per_px)

    # Ensure minimum obstacle clearance
    distance = max(distance, config.obstacle_clearance_m)

    footprint = compute_footprint(camera, distance)
    h_step, v_step = compute_grid_spacing(footprint, config.front_overlap, config.side_overlap)

    # Build a local coordinate frame on the facade surface
    # u_axis: horizontal direction along the facade
    # v_axis: vertical direction along the facade (or slope direction for roofs)
    # n_axis: outward normal

    normal = facade.normal
    is_horizontal = abs(normal[2]) > 0.99  # flat roof
    is_vertical = abs(normal[2]) < 0.01  # vertical wall

    if is_horizontal:
        # Flat roof: u_axis = East, v_axis = North
        u_axis = np.array([1.0, 0.0, 0.0])
        v_axis = np.array([0.0, 1.0, 0.0])
    elif is_vertical:
        # Vertical wall: u_axis = horizontal tangent, v_axis = up
        u_axis = np.array([-normal[1], normal[0], 0.0])
        u_axis /= np.linalg.norm(u_axis)
        v_axis = np.array([0.0, 0.0, 1.0])
    else:
        # Pitched surface: u_axis = horizontal component perpendicular to slope direction
        horiz_normal = np.array([normal[0], normal[1], 0.0])
        horiz_norm = np.linalg.norm(horiz_normal)
        horiz_normal /= horiz_norm
        u_axis = np.array([-horiz_normal[1], horiz_normal[0], 0.0])
        u_axis /= np.linalg.norm(u_axis)
        # v_axis: along the slope (perpendicular to u_axis and normal)
        v_axis = np.cross(normal, u_axis)
        v_axis /= np.linalg.norm(v_axis)
        # Ensure v_axis has positive Z component (points uphill)
        if v_axis[2] < 0:
            v_axis = -v_axis

    # Project facade vertices onto the u-v plane to find extents
    center = facade.center
    verts_centered = facade.vertices - center
    u_coords = verts_centered @ u_axis
    v_coords = verts_centered @ v_axis

    u_min, u_max = float(u_coords.min()), float(u_coords.max())
    v_min, v_max = float(v_coords.min()), float(v_coords.max())

    # Add small inset to avoid edge photos
    inset = 0.1  # 10cm inset from edges
    u_min += inset
    u_max -= inset
    v_min += inset
    v_max -= inset

    if u_max <= u_min or v_max <= v_min:
        return []

    # Generate grid positions
    n_cols = max(1, int(math.ceil((u_max - u_min) / h_step)) + 1)
    n_rows = max(1, int(math.ceil((v_max - v_min) / v_step)) + 1)

    # Center the grid within the facade extents
    u_total = (n_cols - 1) * h_step if n_cols > 1 else 0
    v_total = (n_rows - 1) * v_step if n_rows > 1 else 0
    u_start = (u_min + u_max) / 2 - u_total / 2
    v_start = (v_min + v_max) / 2 - v_total / 2

    # Compute camera orientation
    # Aircraft heading: face the surface normal (nose toward facade)
    heading_deg = float(math.degrees(math.atan2(normal[0], normal[1])) % 360)
    # We want to face the facade, so heading is opposite to normal
    heading_deg = (heading_deg + 180) % 360

    # Gimbal pitch: compute from the look-at direction (-normal)
    # The camera looks opposite to the outward normal (toward the surface).
    # Pitch = atan2(vertical_component, horizontal_magnitude) of the look direction.
    look_dir = -normal  # look toward the surface
    horiz_mag = math.sqrt(look_dir[0] ** 2 + look_dir[1] ** 2)
    gimbal_pitch = math.degrees(math.atan2(look_dir[2], horiz_mag))
    # Vertical wall: pitch=0 (horizontal), flat roof: pitch=-90 (nadir),
    # 30° pitched roof: pitch=-60° (tilted well down)

    # Clamp gimbal pitch to hardware limits
    gimbal_pitch = max(-90.0, min(35.0, gimbal_pitch))

    # Offset vector from facade surface to camera position
    offset = normal * distance

    waypoints = []
    for row in range(n_rows):
        v_pos = v_start + row * v_step

        # Boustrophedon: reverse column order on odd rows
        col_range = range(n_cols) if row % 2 == 0 else range(n_cols - 1, -1, -1)

        for col in col_range:
            u_pos = u_start + col * h_step

            # Position on the facade surface
            surface_pos = center + u_pos * u_axis + v_pos * v_axis
            # Camera position (offset from surface)
            cam_pos = surface_pos + offset

            # Ensure minimum altitude
            if cam_pos[2] < 2.0:
                cam_pos[2] = 2.0

            # Create photo action
            actions = [
                CameraAction(
                    action_type=ActionType.TAKE_PHOTO,
                    camera=config.camera,
                ),
            ]

            wp = Waypoint(
                x=float(cam_pos[0]),
                y=float(cam_pos[1]),
                z=float(cam_pos[2]),
                heading_deg=heading_deg,
                gimbal_pitch_deg=gimbal_pitch,
                speed_ms=config.flight_speed_ms,
                actions=actions,
                facade_index=facade.index,
                component_tag=facade.component_tag,
            )
            waypoints.append(wp)

    return waypoints


def _make_transition_waypoint(
    x: float, y: float, z: float,
    heading_deg: float,
    speed_ms: float,
) -> Waypoint:
    """Create a transit waypoint (no photo, higher speed)."""
    return Waypoint(
        x=x, y=y, z=max(z, MIN_ALTITUDE_M),
        heading_deg=heading_deg,
        gimbal_pitch_deg=0.0,
        speed_ms=speed_ms,
        actions=[],
        is_transition=True,
    )


def _generate_transition_waypoints(
    prev_wp: Waypoint,
    next_wp: Waypoint,
    prev_facade: Facade,
    next_facade: Facade,
    clearance: float,
    speed: float,
) -> list[Waypoint]:
    """Generate clearance waypoints for transitioning between facades.

    Flies out from the last waypoint along the facade normal,
    then approaches the next facade from outside. This prevents
    the drone from flying through the building at corners.
    """
    import numpy as np

    # For transitions to/from roof, just go straight (already above building)
    if abs(prev_facade.normal[2]) > 0.5 or abs(next_facade.normal[2]) > 0.5:
        return []

    # Exit point: fly further out along prev facade normal
    exit_pos = np.array([prev_wp.x, prev_wp.y, prev_wp.z])
    exit_pos[:2] += prev_facade.normal[:2] * clearance

    # Entry point: approach next facade from outside
    entry_pos = np.array([next_wp.x, next_wp.y, next_wp.z])
    entry_pos[:2] += next_facade.normal[:2] * clearance

    # Use a safe altitude for transition (highest of exit/entry + margin)
    transit_z = max(exit_pos[2], entry_pos[2], prev_wp.z, next_wp.z) + 2.0

    transition_wps = []

    # Exit waypoint: pull out from facade
    transition_wps.append(_make_transition_waypoint(
        float(exit_pos[0]), float(exit_pos[1]), transit_z,
        prev_wp.heading_deg, speed,
    ))

    # Corner waypoint (if exit and entry are far apart)
    dist = np.linalg.norm(exit_pos[:2] - entry_pos[:2])
    if dist > clearance:
        # Add a midpoint at the corner to avoid clipping
        mid = (exit_pos + entry_pos) / 2
        # Push the midpoint outward from building center
        mid_dir = mid[:2].copy()
        mid_norm = np.linalg.norm(mid_dir)
        if mid_norm > 0.1:
            mid_dir /= mid_norm
            mid[:2] += mid_dir * clearance * 0.5
        transition_wps.append(_make_transition_waypoint(
            float(mid[0]), float(mid[1]), transit_z,
            next_wp.heading_deg, speed,
        ))

    # Entry waypoint: approach next facade
    transition_wps.append(_make_transition_waypoint(
        float(entry_pos[0]), float(entry_pos[1]), transit_z,
        next_wp.heading_deg, speed,
    ))

    return transition_wps


def generate_mission_waypoints(
    building: Building,
    config: MissionConfig,
) -> list[Waypoint]:
    """Generate all waypoints for a building inspection mission.

    Iterates over all facades, generates waypoint grids, and inserts
    transition waypoints between facades to avoid flying through the building.
    """
    # Generate inspection waypoints per facade
    facade_groups: list[list[Waypoint]] = []
    for facade in building.facades:
        facade_wps = generate_waypoints_for_facade(facade, config)
        if facade_wps:
            facade_groups.append(facade_wps)

    # Assemble with transition waypoints between facades
    all_waypoints: list[Waypoint] = []
    transit_speed = min(config.flight_speed_ms * 3, MAX_SPEED_MS)

    for i, group in enumerate(facade_groups):
        if i > 0 and all_waypoints:
            # Insert transition waypoints between facades
            prev_wp = all_waypoints[-1]
            next_wp = group[0]
            prev_facade = building.facades[prev_wp.facade_index]
            next_facade = building.facades[next_wp.facade_index]

            transitions = _generate_transition_waypoints(
                prev_wp, next_wp, prev_facade, next_facade,
                clearance=config.obstacle_clearance_m,
                speed=transit_speed,
            )
            all_waypoints.extend(transitions)

        all_waypoints.extend(group)

    # Assign global indices
    for i, wp in enumerate(all_waypoints):
        wp.index = i

    # Convert local ENU to WGS84
    convert_enu_to_wgs84(all_waypoints, building.lat, building.lon, building.ground_altitude)

    return all_waypoints


def convert_enu_to_wgs84(
    waypoints: list[Waypoint],
    ref_lat: float,
    ref_lon: float,
    ref_alt: float = 0.0,
) -> None:
    """Convert local ENU coordinates to WGS84 (lat, lon, alt).

    Uses a simple offset calculation suitable for small areas (<1km).
    For more precision, pyproj can be used.
    """
    # Approximate meters per degree at the reference latitude
    lat_rad = math.radians(ref_lat)
    meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
    meters_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)

    for wp in waypoints:
        wp.lat = ref_lat + wp.y / meters_per_deg_lat
        wp.lon = ref_lon + wp.x / meters_per_deg_lon
        wp.alt = ref_alt + wp.z


def build_l_shaped_building(
    lat: float,
    lon: float,
    wing1_width: float,
    wing1_depth: float,
    wing2_width: float,
    wing2_depth: float,
    height: float,
    heading_deg: float = 0.0,
    ground_altitude: float = 0.0,
    label: str = "",
) -> Building:
    """Create an L-shaped building from two rectangular wings.

    Wing 1 runs along the heading direction (the main wing).
    Wing 2 extends perpendicular from the end of wing 1.

    The L-shape is formed by placing wing1 and wing2 so they share a corner.
    """
    b1 = build_rectangular_building(
        lat=lat, lon=lon,
        width=wing1_width, depth=wing1_depth, height=height,
        heading_deg=heading_deg,
        label=f"{label}_wing1" if label else "wing1",
    )
    # Compute offset for wing 2 center
    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # Wing 2 center offset in local building coords
    # Place wing 2 at the end of wing 1, extending perpendicular
    local_x = wing1_width / 2 + wing2_depth / 2  # shift along width of wing1
    local_y = wing1_depth / 2 - wing2_width / 2  # align with front edge of wing1

    # Rotate to ENU
    offset_e = local_x * cos_t + local_y * sin_t
    offset_n = -local_x * sin_t + local_y * cos_t

    # Approximate GPS offset
    lat_rad = math.radians(lat)
    meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad)
    meters_per_deg_lon = 111412.84 * math.cos(lat_rad)

    lat2 = lat + offset_n / meters_per_deg_lat
    lon2 = lon + offset_e / meters_per_deg_lon

    b2 = build_rectangular_building(
        lat=lat2, lon=lon2,
        width=wing2_width, depth=wing2_depth, height=height,
        heading_deg=heading_deg + 90,  # perpendicular
        label=f"{label}_wing2" if label else "wing2",
    )

    # Combine facades, re-index
    combined = Building(
        lat=lat, lon=lon,
        ground_altitude=ground_altitude,
        width=wing1_width, depth=wing1_depth,
        height=height,
        heading_deg=heading_deg,
        label=label or "l_shaped",
    )

    all_facades = []
    for i, f in enumerate(b1.facades + b2.facades):
        f.index = i
        all_facades.append(f)
    combined.facades = all_facades

    return combined
