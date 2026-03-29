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
from .optimize import optimize_flight_path
from .models import (
    ActionType,
    AlgorithmConfig,
    Building,
    CameraAction,
    CameraName,
    Facade,
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    MAX_SPEED_MS,
    meters_per_deg,
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
    algo: AlgorithmConfig | None = None,
    mesh: object | None = None,
) -> list[Waypoint]:
    """Generate a grid of waypoints for a single facade.

    The grid is parallel to the facade plane, offset along the outward normal
    by the distance needed to achieve the target GSD. Waypoints are ordered
    in a boustrophedon (lawnmower) pattern.
    """
    if algo is None:
        algo = AlgorithmConfig()

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
    inset = algo.facade_edge_inset_m
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

    # LOS sample offsets for multi-ray visibility check
    _los_samples = None
    if mesh is not None and algo.enable_waypoint_los:
        _los_samples = [
            np.zeros(3),
            u_axis * h_step * 0.3,
            -u_axis * h_step * 0.3,
            v_axis * v_step * 0.3,
            -v_axis * v_step * 0.3,
        ]

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
            if cam_pos[2] < algo.min_altitude_m:
                cam_pos[2] = algo.min_altitude_m

            # Line-of-sight check: cast rays from camera to facade sample
            # points and skip waypoints where the mesh blocks the view.
            if _los_samples is not None:
                origins = []
                directions = []
                ray_lengths = []
                for s_off in _los_samples:
                    sample_target = target_pos + s_off
                    ray_vec = sample_target - cam_pos
                    ray_len = float(np.linalg.norm(ray_vec))
                    if ray_len < 1e-6:
                        continue
                    origins.append(cam_pos.copy())
                    directions.append(ray_vec / ray_len)
                    ray_lengths.append(ray_len)

                if origins:
                    hits, idx_ray, _ = mesh.ray.intersects_location(
                        ray_origins=np.array(origins),
                        ray_directions=np.array(directions),
                    )
                    n_visible = len(origins)
                    tol = algo.los_tolerance_m
                    for ri in range(len(origins)):
                        mask = idx_ray == ri
                        if mask.any():
                            hit_dists = np.linalg.norm(
                                hits[mask] - origins[ri], axis=1
                            )
                            if float(hit_dists.min()) < ray_lengths[ri] - tol:
                                n_visible -= 1

                    if n_visible < len(origins) * algo.los_min_visible_ratio:
                        continue  # too much of the view is blocked

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
    min_altitude_m: float = MIN_ALTITUDE_M,
) -> Waypoint:
    """Create a transit waypoint (no photo, higher speed)."""
    return Waypoint(
        x=x, y=y, z=max(z, min_altitude_m),
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
    algo: AlgorithmConfig | None = None,
) -> list[Waypoint]:
    """Generate clearance waypoints for transitioning between facades.

    Flies out from the last waypoint along the facade normal,
    then approaches the next facade from outside. This prevents
    the drone from flying through the building at corners.
    """
    import numpy as np

    if algo is None:
        algo = AlgorithmConfig()

    # For transitions to/from roof, just go straight (already above building)
    if abs(prev_facade.normal[2]) > algo.roof_normal_threshold or abs(next_facade.normal[2]) > algo.roof_normal_threshold:
        return []

    # Exit point: fly further out along prev facade normal
    exit_pos = np.array([prev_wp.x, prev_wp.y, prev_wp.z])
    exit_pos[:2] += prev_facade.normal[:2] * clearance

    # Entry point: approach next facade from outside
    entry_pos = np.array([next_wp.x, next_wp.y, next_wp.z])
    entry_pos[:2] += next_facade.normal[:2] * clearance

    # Use a safe altitude for transition (highest of exit/entry + margin)
    transit_z = max(exit_pos[2], entry_pos[2], prev_wp.z, next_wp.z) + algo.transition_altitude_margin_m

    transition_wps = []

    # Exit waypoint: pull out from facade
    transition_wps.append(_make_transition_waypoint(
        float(exit_pos[0]), float(exit_pos[1]), transit_z,
        prev_wp.heading_deg, speed, algo.min_altitude_m,
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
            next_wp.heading_deg, speed, algo.min_altitude_m,
        ))

    # Entry waypoint: approach next facade
    transition_wps.append(_make_transition_waypoint(
        float(entry_pos[0]), float(entry_pos[1]), transit_z,
        next_wp.heading_deg, speed, algo.min_altitude_m,
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


def _generate_surface_sample_waypoints(
    mesh: object,
    building: Building,
    config: MissionConfig,
    algo: AlgorithmConfig,
) -> tuple[list[list[Waypoint]], dict]:
    """Generate waypoints by uniformly sampling the mesh surface.

    Instead of per-facade boustrophedon grids, this places cameras at every
    sample point offset along the face normal. Covers the entire building
    surface like a blanket — dormers, curves, corners, overhangs — not just
    the planes that facade extraction identifies.

    Uses trimesh.sample (Poisson-disk) for uniform coverage,
    scipy.spatial.KDTree for deduplication, and nearest-neighbor
    traversal for an efficient flight path.

    Returns (facade_groups, stats) ready for optimize_flight_path.
    """
    from scipy.spatial import KDTree

    camera = get_camera(config.camera)
    distance = compute_distance_for_gsd(camera, config.target_gsd_mm_per_px)
    distance = max(distance, config.obstacle_clearance_m)

    pitch_min = GIMBAL_TILT_MIN_DEG + config.gimbal_pitch_margin_deg
    pitch_max = GIMBAL_TILT_MAX_DEG - config.gimbal_pitch_margin_deg

    # 1. Sample the mesh surface
    points, face_indices = mesh.sample(algo.surface_sample_count, return_index=True)
    normals = mesh.face_normals[face_indices].copy()

    # 2. Orient normals outward (away from mesh centroid)
    centroid = mesh.centroid
    to_point = points - centroid
    flip_mask = np.einsum('ij,ij->i', normals, to_point) < 0
    normals[flip_mask] *= -1

    # 3. Filter surfaces that can't produce useful photos
    keep = (
        (normals[:, 2] >= algo.downward_face_threshold)
        & (points[:, 2] >= algo.ground_level_threshold_m)
    )
    points = points[keep]
    normals = normals[keep]
    n_filtered = int((~keep).sum())

    # 4. Filter interior faces: cast ray from surface along outward normal.
    #    Exterior faces escape to open air; interior faces hit the outer
    #    shell nearby. Same logic as facade extraction's occlusion check.
    if len(points) > 0:
        offset = algo.occlusion_ray_offset_m
        ray_origins = points + normals * offset
        max_extent = float(max(mesh.bounding_box.extents))
        hit_threshold = max_extent * algo.occlusion_hit_fraction

        hits, idx_ray, _ = mesh.ray.intersects_location(
            ray_origins=ray_origins,
            ray_directions=normals,
        )
        exterior_mask = np.ones(len(points), dtype=bool)
        if len(hits) > 0:
            for i in range(len(points)):
                ray_hits = hits[idx_ray == i]
                if len(ray_hits) > 0:
                    min_dist = float(np.linalg.norm(
                        ray_hits - ray_origins[i], axis=1
                    ).min())
                    if min_dist < hit_threshold:
                        exterior_mask[i] = False
        n_interior = int((~exterior_mask).sum())
        n_filtered += n_interior
        points = points[exterior_mask]
        normals = normals[exterior_mask]

    # 5. Compute camera positions
    cam_positions = points + normals * distance

    # Filter out cameras that would be underground
    altitude_ok = cam_positions[:, 2] >= algo.min_altitude_m
    cam_positions = cam_positions[altitude_ok]
    points = points[altitude_ok]
    normals = normals[altitude_ok]
    n_filtered += int((~altitude_ok).sum())

    # Compute look vectors and orientations
    look = points - cam_positions
    horiz_mag = np.sqrt(look[:, 0] ** 2 + look[:, 1] ** 2)

    headings = np.degrees(np.arctan2(look[:, 0], look[:, 1])) % 360
    pitches = np.where(
        horiz_mag > 1e-6,
        np.degrees(np.arctan2(look[:, 2], horiz_mag)),
        -90.0,
    )
    pitches = np.clip(pitches, pitch_min, pitch_max)

    # 5. Spatial deduplication via KDTree
    n_before_dedup = len(cam_positions)
    if n_before_dedup > 1:
        tree = KDTree(cam_positions)
        pairs = tree.query_pairs(r=algo.surface_dedup_radius_m)
        removed: set[int] = set()
        for i, j in sorted(pairs):
            if i in removed or j in removed:
                continue
            h_diff = abs(headings[i] - headings[j])
            if h_diff > 180:
                h_diff = 360 - h_diff
            if h_diff < algo.surface_dedup_max_angle_deg:
                if abs(pitches[i]) <= abs(pitches[j]):
                    removed.add(j)
                else:
                    removed.add(i)
        keep_idx = sorted(set(range(n_before_dedup)) - removed)
        cam_positions = cam_positions[keep_idx]
        points = points[keep_idx]
        headings = headings[keep_idx]
        pitches = pitches[keep_idx]

    n_deduped = n_before_dedup - len(cam_positions)

    # 6. LOS check against mesh
    n_los_removed = 0
    if algo.enable_waypoint_los and len(cam_positions) > 0:
        visible_mask = np.ones(len(cam_positions), dtype=bool)
        for i in range(len(cam_positions)):
            ray_vec = points[i] - cam_positions[i]
            ray_len = float(np.linalg.norm(ray_vec))
            if ray_len < 1e-6:
                visible_mask[i] = False
                continue
            ray_dir = ray_vec / ray_len
            hits, _, _ = mesh.ray.intersects_location(
                ray_origins=[cam_positions[i]],
                ray_directions=[ray_dir],
            )
            if len(hits) > 0:
                hit_dists = np.linalg.norm(hits - cam_positions[i], axis=1)
                if float(hit_dists.min()) < ray_len - algo.los_tolerance_m:
                    visible_mask[i] = False
        n_los_removed = int((~visible_mask).sum())
        cam_positions = cam_positions[visible_mask]
        points = points[visible_mask]
        headings = headings[visible_mask]
        pitches = pitches[visible_mask]

    # 7. Assign facade_index by nearest facade center (for viewer coloring)
    facade_centers = np.array([f.center for f in building.facades])
    facade_tags = {f.index: f.component_tag for f in building.facades}

    facade_indices = np.zeros(len(points), dtype=int)
    if len(facade_centers) > 0 and len(points) > 0:
        facade_tree = KDTree(facade_centers)
        _, nearest = facade_tree.query(points)
        for i, ni in enumerate(nearest):
            facade_indices[i] = building.facades[ni].index

    # 8. Nearest-neighbor ordering for efficient flight path
    #    Instead of facade-based groups (which scatter spatially),
    #    order all waypoints as a single spatial chain.
    n_wps = len(cam_positions)
    if n_wps > 1:
        order = [0]
        used = np.zeros(n_wps, dtype=bool)
        used[0] = True
        nn_tree = KDTree(cam_positions)
        for _ in range(n_wps - 1):
            # Query enough neighbors to likely find an unused one
            k = min(32, n_wps)
            _, ii = nn_tree.query(cam_positions[order[-1]], k=k)
            found = False
            for idx in ii:
                if not used[idx]:
                    order.append(idx)
                    used[idx] = True
                    found = True
                    break
            if not found:
                # Fallback: brute-force nearest unused
                unused = np.where(~used)[0]
                dists = np.linalg.norm(
                    cam_positions[unused] - cam_positions[order[-1]], axis=1
                )
                best = unused[np.argmin(dists)]
                order.append(best)
                used[best] = True
        cam_positions = cam_positions[order]
        points = points[order]
        headings = headings[order]
        pitches = pitches[order]
        facade_indices = facade_indices[order]

    # 9. Build Waypoint objects in NN order.
    #    Return as a single group — the NN ordering IS the flight path.
    #    facade_index on each waypoint is for viewer coloring only.
    all_wps: list[Waypoint] = []
    for i in range(len(cam_positions)):
        fi = int(facade_indices[i])
        all_wps.append(Waypoint(
            x=float(cam_positions[i][0]),
            y=float(cam_positions[i][1]),
            z=float(cam_positions[i][2]),
            heading_deg=float(headings[i]),
            gimbal_pitch_deg=float(pitches[i]),
            speed_ms=config.flight_speed_ms,
            actions=[CameraAction(action_type=ActionType.TAKE_PHOTO, camera=config.camera)],
            facade_index=fi,
            component_tag=facade_tags.get(fi, "21.1"),
        ))

    facade_groups = [all_wps] if all_wps else []

    stats = {
        "strategy": "surface_sampling",
        "samples_requested": algo.surface_sample_count,
        "samples_after_filter": n_before_dedup,
        "filtered_total": n_filtered,
        "deduped": n_deduped,
        "los_removed": n_los_removed,
        "waypoints_generated": len(all_wps),
    }
    return facade_groups, stats


def generate_mission_waypoints(
    building: Building,
    config: MissionConfig,
    algo: AlgorithmConfig | None = None,
    mesh: object | None = None,
    waypoint_strategy: str = "facade_grid",
) -> tuple[list[Waypoint], dict]:
    """Generate all waypoints for a building inspection mission.

    Supports two strategies:
    - "facade_grid": per-facade boustrophedon grid (default, works for all buildings)
    - "surface_sampling": uniform mesh surface sampling (mesh buildings only)

    Returns (waypoints, stats) where stats contains generation metrics.
    """
    if algo is None:
        algo = AlgorithmConfig()

    surface_stats = None
    total_before_dedup = 0
    per_facade_stats: list[dict] = []

    # --- Surface sampling strategy ---
    if waypoint_strategy == "surface_sampling" and mesh is not None:
        facade_groups, surface_stats = _generate_surface_sample_waypoints(
            mesh, building, config, algo,
        )
    else:
        # --- Facade grid strategy (default) ---
        facade_groups = []

        for facade in building.facades:
            facade_wps = generate_waypoints_for_facade(facade, config, algo, mesh=mesh)
            before = len(facade_wps)
            total_before_dedup += before
            if facade_wps:
                facade_groups.append(facade_wps)
                per_facade_stats.append({
                    "facade_index": facade.index,
                    "label": facade.label,
                    "waypoints": len(facade_wps),
                    "before_dedup": before,
                })

    facades_with_wps = len(facade_groups)

    # Optimize flight path: dedup + TSP ordering + sweep direction reversal
    facade_groups, opt_result = optimize_flight_path(
        facade_groups,
        merge_radius_m=config.min_photo_distance_m,
        enable_dedup=algo.enable_path_dedup,
        enable_tsp=algo.enable_path_tsp,
        enable_sweep_reversal=algo.enable_sweep_reversal,
        max_gimbal_angle_diff_deg=algo.dedup_max_gimbal_diff_deg,
        tsp_method=algo.tsp_method,
    )

    # Concatenate all inspection waypoints
    all_waypoints: list[Waypoint] = []
    for group in facade_groups:
        all_waypoints.extend(group)

    total_after = len(all_waypoints)

    # Assign global indices
    for i, wp in enumerate(all_waypoints):
        wp.index = i

    # Convert local ENU to WGS84
    convert_enu_to_wgs84(all_waypoints, building.lat, building.lon, building.ground_altitude)

    stats: dict = {
        "strategy": waypoint_strategy,
        "facades_total": len(building.facades),
        "facades_with_waypoints": facades_with_wps,
        "waypoints_after_dedup": total_after,
        "optimization": {
            "waypoints_merged": opt_result.waypoints_merged,
            "facade_order": opt_result.facade_order_after,
            "facades_reversed": opt_result.facades_reversed,
            "transit_distance_before_m": round(opt_result.transit_distance_before, 2),
            "transit_distance_after_m": round(opt_result.transit_distance_after, 2),
            "transit_saved_m": round(
                opt_result.transit_distance_before - opt_result.transit_distance_after, 2
            ),
            "two_opt_improvements": opt_result.two_opt_improvements,
        },
    }
    if surface_stats:
        stats["surface_sampling"] = surface_stats
    else:
        stats["waypoints_before_dedup"] = total_before_dedup
        stats["per_facade"] = per_facade_stats

    return all_waypoints, stats


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
    meters_per_deg_lat, meters_per_deg_lon = meters_per_deg(lat_rad)

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
    meters_per_deg_lat, meters_per_deg_lon = meters_per_deg(lat_rad)

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
