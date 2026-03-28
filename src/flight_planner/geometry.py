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
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    MAX_SPEED_MS,
    MIN_ALTITUDE_M,
    MissionConfig,
    RoofType,
    Waypoint,
)


def _rotation_matrix_z(angle_rad: float) -> np.ndarray:
    """3x3 rotation matrix around the Z (up) axis.

    Positive angle = counterclockwise when viewed from above.
    Used for building heading rotation in ENU coordinates.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


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

    # Rotation matrix for heading around Z (up) axis.
    # Heading is CW from north in ENU: local Y (forward) maps to north at heading=0.
    # R_z(theta) rotates local coords to ENU.
    R = _rotation_matrix_z(math.radians(heading_deg))

    hw, hd = width / 2, depth / 2

    # Footprint corners in local building coords (x=along width, y=along depth)
    local_corners_3d = np.array([
        [-hw, -hd, 0.0],  # 0: left-back
        [hw, -hd, 0.0],   # 1: right-back
        [hw, hd, 0.0],    # 2: right-front
        [-hw, hd, 0.0],   # 3: left-front
    ])

    # Rotate to ENU and set heights
    ground = (R @ local_corners_3d.T).T
    eave = ground.copy()
    eave[:, 2] = height

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

        # Ridge runs along local X at local Y=0, rotated to ENU
        ridge_local = np.array([
            [-hw, 0.0, ridge_height],
            [hw, 0.0, ridge_height],
        ])
        ridge_enu = (R @ ridge_local.T).T
        # Restore Z since R only rotates in XY
        ridge_enu[:, 2] = ridge_height
        ridge_left_3d = ridge_enu[0]
        ridge_right_3d = ridge_enu[1]

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

    # Gimbal pitch limits with configurable safety margin
    pitch_min = GIMBAL_TILT_MIN_DEG + config.gimbal_pitch_margin_deg
    pitch_max = GIMBAL_TILT_MAX_DEG - config.gimbal_pitch_margin_deg

    # Offset vector from facade surface to camera position
    offset = normal * distance

    waypoints = []
    for row in range(n_rows):
        v_pos = v_start + row * v_step

        # Boustrophedon: reverse column order on odd rows
        col_range = range(n_cols) if row % 2 == 0 else range(n_cols - 1, -1, -1)

        for col in col_range:
            u_pos = u_start + col * h_step

            # Target point on the facade surface
            target_pos = center + u_pos * u_axis + v_pos * v_axis
            # Camera position (offset from surface along normal)
            cam_pos = target_pos + offset

            # Ensure minimum altitude
            if cam_pos[2] < MIN_ALTITUDE_M:
                cam_pos[2] = float(MIN_ALTITUDE_M)

            # Per-waypoint look-at vector: from camera to target surface point
            look = target_pos - cam_pos
            horiz_mag = math.sqrt(look[0] ** 2 + look[1] ** 2)

            # Heading: face toward the target point
            wp_heading = float(math.degrees(math.atan2(look[0], look[1])) % 360)

            # Gimbal pitch: angle from horizontal to look direction
            wp_pitch = math.degrees(math.atan2(look[2], horiz_mag)) if horiz_mag > 1e-6 else -90.0
            wp_pitch = max(pitch_min, min(pitch_max, wp_pitch))

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
                heading_deg=wp_heading,
                gimbal_pitch_deg=wp_pitch,
                speed_ms=config.flight_speed_ms,
                actions=actions,
                facade_index=facade.index,
                component_tag=facade.component_tag,
            )
            waypoints.append(wp)

    # Deduplicate waypoints closer than min_photo_distance
    if len(waypoints) > 1 and config.min_photo_distance_m > 0:
        deduped = [waypoints[0]]
        for wp in waypoints[1:]:
            prev = deduped[-1]
            dist = math.sqrt(
                (wp.x - prev.x) ** 2 + (wp.y - prev.y) ** 2 + (wp.z - prev.z) ** 2
            )
            if dist >= config.min_photo_distance_m:
                deduped.append(wp)
        waypoints = deduped

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


def _nearest_neighbor_order(groups: list[list[Waypoint]]) -> list[list[Waypoint]]:
    """Reorder facade waypoint groups by nearest-neighbor to minimize transit.

    Uses the last waypoint of the current group → first waypoint of the next
    group as the distance metric. Greedy nearest-neighbor starting from
    the group closest to the origin.
    """
    n = len(groups)
    if n <= 2:
        return groups

    # Compute centroid (midpoint of first and last waypoint) for each group
    centroids = []
    for g in groups:
        cx = (g[0].x + g[-1].x) / 2
        cy = (g[0].y + g[-1].y) / 2
        cz = (g[0].z + g[-1].z) / 2
        centroids.append((cx, cy, cz))

    # Start from the group closest to the origin (building center)
    remaining = set(range(n))
    start = min(remaining, key=lambda i: centroids[i][0] ** 2 + centroids[i][1] ** 2 + centroids[i][2] ** 2)

    order = [start]
    remaining.discard(start)

    while remaining:
        last = order[-1]
        # Distance from the last waypoint of current group to first of candidates
        lx, ly, lz = groups[last][-1].x, groups[last][-1].y, groups[last][-1].z
        nearest = min(remaining, key=lambda i: (
            (groups[i][0].x - lx) ** 2 +
            (groups[i][0].y - ly) ** 2 +
            (groups[i][0].z - lz) ** 2
        ))
        order.append(nearest)
        remaining.discard(nearest)

    return [groups[i] for i in order]


def generate_mission_waypoints(
    building: Building,
    config: MissionConfig,
) -> list[Waypoint]:
    """Generate all waypoints for a building inspection mission.

    Generates inspection waypoint grids per facade, reorders by nearest-neighbor
    to minimize transit distance, and lets the DJI flight controller handle
    obstacle avoidance between waypoints (APAS).
    """
    facade_groups: list[list[Waypoint]] = []
    for facade in building.facades:
        facade_wps = generate_waypoints_for_facade(facade, config)
        if facade_wps:
            facade_groups.append(facade_wps)

    # Reorder facades by nearest-neighbor to minimize transit distance
    if len(facade_groups) > 2:
        facade_groups = _nearest_neighbor_order(facade_groups)

    # Concatenate all inspection waypoints — no artificial transition waypoints.
    # The drone flies straight between facades; its obstacle avoidance handles clearance.
    all_waypoints: list[Waypoint] = []
    for group in facade_groups:
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
    # Compute offset for wing 2 center using rotation matrix
    R = _rotation_matrix_z(math.radians(heading_deg))

    local_offset = np.array([
        wing1_width / 2 + wing2_depth / 2,   # shift along width of wing1
        wing1_depth / 2 - wing2_width / 2,    # align with front edge of wing1
        0.0,
    ])
    enu_offset = R @ local_offset

    # Approximate GPS offset
    lat_rad = math.radians(lat)
    meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
    meters_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)

    lat2 = lat + enu_offset[1] / meters_per_deg_lat
    lon2 = lon + enu_offset[0] / meters_per_deg_lon

    b2 = build_rectangular_building(
        lat=lat2, lon=lon2,
        width=wing2_width, depth=wing2_depth, height=height,
        heading_deg=heading_deg + 90,  # perpendicular
        label=f"{label}_wing2" if label else "wing2",
    )

    # Remove interior facades: walls whose center is inside the other wing.
    # Use a simple 2D point-in-footprint check (ground vertices of each wing).
    def _footprint_2d(building: Building) -> np.ndarray:
        """Get ground-level XY coords of wall vertices as a polygon."""
        pts = []
        for f in building.facades:
            if abs(f.normal[2]) < 0.01:  # walls only
                for v in f.vertices:
                    if abs(v[2]) < 0.1:
                        pts.append(v[:2])
        if not pts:
            return np.array([])
        pts = np.array(pts)
        # Sort by angle around centroid for convex polygon
        cx, cy = pts.mean(axis=0)
        angles = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
        return pts[np.argsort(angles)]

    def _point_in_polygon_2d(point: np.ndarray, polygon: np.ndarray) -> bool:
        """Ray casting point-in-polygon test (2D)."""
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    fp1 = _footprint_2d(b1)
    fp2 = _footprint_2d(b2)

    exterior_facades = []
    for f in b1.facades + b2.facades:
        if abs(f.normal[2]) > 0.5:
            # Roof — always keep
            exterior_facades.append(f)
            continue
        # Check if wall center is inside the other wing
        center_2d = f.center[:2]
        is_b1_facade = any(np.array_equal(f.vertices, bf.vertices) for bf in b1.facades)
        other_fp = fp2 if is_b1_facade else fp1
        if len(other_fp) >= 3 and _point_in_polygon_2d(center_2d, other_fp):
            continue  # interior wall — skip
        exterior_facades.append(f)

    # Re-index
    combined = Building(
        lat=lat, lon=lon,
        ground_altitude=ground_altitude,
        width=wing1_width, depth=wing1_depth,
        height=height,
        heading_deg=heading_deg,
        label=label or "l_shaped",
    )

    for i, f in enumerate(exterior_facades):
        f.index = i
    combined.facades = exterior_facades

    return combined
