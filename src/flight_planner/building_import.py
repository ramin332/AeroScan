"""Build Building objects from uploaded geometry.

Supports:
- GeoJSON polygon footprints (BAG data, manual outlines)
- OBJ/PLY/STL 3D meshes (Smart 3D Explore output, prior scans)

Each polygon edge or mesh footprint edge becomes a wall facade,
and the polygon at height becomes a roof.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .models import Building, Facade, RoofType


def _convex_hull_2d(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Compute 2D convex hull using Andrew's monotone chain algorithm."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def _footprint_to_building(
    enu_coords: list[tuple[float, float]],
    center_lat: float,
    center_lon: float,
    height: float,
    num_stories: int,
    roof_type_str: str,
    roof_pitch_deg: float,
    name: str,
) -> Building:
    """Create a Building from a footprint polygon in local ENU coordinates.

    Shared logic for GeoJSON and mesh import pipelines.
    """
    # Ensure counterclockwise winding for consistent outward normals
    # Shoelace formula: positive = CCW, negative = CW
    n = len(enu_coords)
    signed_area = 0.0
    for i in range(n):
        j = (i + 1) % n
        signed_area += enu_coords[i][0] * enu_coords[j][1]
        signed_area -= enu_coords[j][0] * enu_coords[i][1]
    if signed_area < 0:  # clockwise — reverse to counterclockwise
        enu_coords = list(reversed(enu_coords))

    # Create wall facades from polygon edges
    facades: list[Facade] = []
    for i in range(n):
        j = (i + 1) % n
        x0, y0 = enu_coords[i]
        x1, y1 = enu_coords[j]

        # Wall quad: bottom-left, bottom-right, top-right, top-left
        vertices = np.array([
            [x0, y0, 0.0],
            [x1, y1, 0.0],
            [x1, y1, height],
            [x0, y0, height],
        ])

        # Outward normal for CCW polygon: right-hand perpendicular
        edge_x = x1 - x0
        edge_y = y1 - y0
        normal = np.array([edge_y, -edge_x, 0.0])
        norm_len = np.linalg.norm(normal)
        if norm_len > 1e-9:
            normal = normal / norm_len

        # Label based on dominant direction of the normal
        azimuth = math.degrees(math.atan2(normal[0], normal[1])) % 360
        if azimuth < 45 or azimuth >= 315:
            direction = "north"
        elif azimuth < 135:
            direction = "east"
        elif azimuth < 225:
            direction = "south"
        else:
            direction = "west"

        facades.append(Facade(
            vertices=vertices,
            normal=normal,
            component_tag="21.1",
            label=f"{direction}_wall_{i}",
            index=i,
        ))

    # Add flat roof (pitched roofs on arbitrary polygons default to flat for now)
    roof_vertices = np.array([[x, y, height] for x, y in enu_coords])
    facades.append(Facade(
        vertices=roof_vertices,
        normal=np.array([0.0, 0.0, 1.0]),
        component_tag="47.1",
        label="roof",
        index=n,
    ))

    # Compute bounding box dimensions
    xs = [c[0] for c in enu_coords]
    ys = [c[1] for c in enu_coords]
    width = max(xs) - min(xs)
    depth = max(ys) - min(ys)

    return Building(
        lat=center_lat,
        lon=center_lon,
        width=round(width, 1),
        depth=round(depth, 1),
        height=height,
        heading_deg=0.0,
        roof_type=RoofType(roof_type_str),
        roof_pitch_deg=roof_pitch_deg,
        num_stories=num_stories,
        facades=facades,
        label=name,
    )


def build_building_from_geojson(
    geojson: dict,
    height: float = 8.0,
    num_stories: int = 1,
    roof_type: str = "flat",
    roof_pitch_deg: float = 0.0,
    name: str = "",
) -> Building:
    """Create a Building from a GeoJSON Feature with Polygon geometry.

    Supports Feature, FeatureCollection (first feature), or bare Polygon/MultiPolygon.
    Properties in the GeoJSON (height, num_stories, roof_type, etc.) override defaults.
    """
    # Extract geometry and properties from various GeoJSON formats
    if geojson.get("type") == "Feature":
        geometry = geojson["geometry"]
        properties = geojson.get("properties", {})
    elif geojson.get("type") == "FeatureCollection":
        features = geojson.get("features", [])
        if not features:
            raise ValueError("FeatureCollection has no features")
        geometry = features[0]["geometry"]
        properties = features[0].get("properties", {})
    elif geojson.get("type") in ("Polygon", "MultiPolygon"):
        geometry = geojson
        properties = {}
    else:
        raise ValueError(f"Unsupported GeoJSON type: {geojson.get('type')}")

    # Override defaults with GeoJSON properties where available
    height = float(properties.get("height", height) or height)
    num_stories = int(properties.get("num_stories", num_stories) or num_stories)
    roof_type_str = str(properties.get("roof_type", roof_type) or roof_type)
    roof_pitch_deg = float(properties.get("roof_pitch_deg", roof_pitch_deg) or roof_pitch_deg)
    name = str(properties.get("name", name) or name or "Uploaded building")

    # Get exterior ring coordinates
    if geometry["type"] == "MultiPolygon":
        coords = geometry["coordinates"][0][0]
    else:
        coords = geometry["coordinates"][0]

    # Remove closing point if present (GeoJSON rings are closed)
    if len(coords) > 1 and coords[0][0] == coords[-1][0] and coords[0][1] == coords[-1][1]:
        coords = coords[:-1]

    if len(coords) < 3:
        raise ValueError("Polygon must have at least 3 vertices")

    # Compute centroid
    lons = [c[0] for c in coords]
    lats = [c[1] for c in coords]
    center_lon = sum(lons) / len(lons)
    center_lat = sum(lats) / len(lats)

    # Convert WGS84 to local ENU (meters from centroid)
    m_per_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(center_lat))
    m_per_lon = 111412.84 * math.cos(math.radians(center_lat))

    enu_coords: list[tuple[float, float]] = []
    for c in coords:
        x = (c[0] - center_lon) * m_per_lon
        y = (c[1] - center_lat) * m_per_lat
        enu_coords.append((x, y))

    return _footprint_to_building(
        enu_coords=enu_coords,
        center_lat=center_lat,
        center_lon=center_lon,
        height=height,
        num_stories=num_stories,
        roof_type_str=roof_type_str,
        roof_pitch_deg=roof_pitch_deg,
        name=name,
    )


def _extract_region_growing(
    mesh: "trimesh.Trimesh",
    min_area_m2: float = 1.0,
    angle_threshold_deg: float = 15.0,
) -> Optional[list[Facade]]:
    """Extract building surfaces via region growing on the mesh face adjacency graph.

    Walks the face adjacency graph, only crossing edges where the angle between
    adjacent face normals is below the threshold. Connected components of
    similarly-oriented adjacent faces become facades.

    This is superior to RANSAC for mesh input because:
    - Preserves mesh topology (parallel walls stay separate naturally)
    - No sampling error (uses exact face normals and areas)
    - Scale-invariant (angle threshold works for any building size)
    - O(F) single pass vs iterative RANSAC
    - Only 2 parameters (angle_threshold, min_area) vs 5 for RANSAC
    """
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import connected_components

    n_faces = len(mesh.faces)
    if n_faces < 4:
        return None

    adj = mesh.face_adjacency                          # (M, 2) pairs
    angles_deg = np.degrees(mesh.face_adjacency_angles)  # degrees per pair

    # Build sparse adjacency graph: only connect faces with similar normals
    mask = angles_deg < angle_threshold_deg
    rows = np.concatenate([adj[mask, 0], adj[mask, 1]])
    cols = np.concatenate([adj[mask, 1], adj[mask, 0]])
    data = np.ones(len(rows), dtype=bool)
    graph = csr_matrix((data, (rows, cols)), shape=(n_faces, n_faces))

    # Connected components = groups of coplanar adjacent faces
    n_comp, labels = connected_components(graph, directed=False)

    building_center = mesh.centroid.copy()
    max_extent = float(max(mesh.bounding_box.extents))
    facades: list[Facade] = []
    idx = 0

    for comp_id in range(n_comp):
        face_mask = labels == comp_id
        comp_area = float(mesh.area_faces[face_mask].sum())

        if comp_area < min_area_m2:
            continue

        # Area-weighted average normal
        comp_normals = mesh.face_normals[face_mask]
        comp_areas = mesh.area_faces[face_mask]
        avg_normal = (comp_normals * comp_areas[:, np.newaxis]).sum(axis=0)
        norm_len = np.linalg.norm(avg_normal)
        if norm_len < 1e-9:
            continue
        avg_normal /= norm_len

        # Get vertices of all faces in this component
        face_indices = np.where(face_mask)[0]
        vert_indices = np.unique(mesh.faces[face_indices].flatten())
        verts = mesh.vertices[vert_indices]
        center = verts.mean(axis=0)

        # Orient normal outward (away from building center)
        if np.dot(avg_normal, center - building_center) < 0:
            avg_normal = -avg_normal

        nz = abs(avg_normal[2])

        # Skip downward-facing (floors/ceilings) and ground-level surfaces
        if avg_normal[2] < -0.3:
            continue
        if center[2] < 0.3:
            continue

        # Occlusion check (trimesh ray casting): cast ray from facade center
        # along outward normal. If it hits the mesh, a drone can't photograph
        # this surface from that direction (wall in the way).
        ray_origin = center + avg_normal * 0.05
        hits = mesh.ray.intersects_location(
            ray_origins=[ray_origin],
            ray_directions=[avg_normal],
        )
        if len(hits[0]) > 0:
            hit_dist = float(np.linalg.norm(hits[0][0] - ray_origin))
            if hit_dist < max_extent * 0.5:
                continue

        # Classify surface type
        if nz < 0.3:
            azimuth = math.degrees(math.atan2(avg_normal[0], avg_normal[1])) % 360
            direction = ("north_wall" if azimuth < 45 or azimuth >= 315 else
                         "east_wall" if azimuth < 135 else
                         "south_wall" if azimuth < 225 else "west_wall")
            component = "21.1"
        elif nz > 0.95:
            direction = "roof_flat"
            component = "47.1"
        else:
            tilt_deg = round(math.degrees(math.acos(min(nz, 1.0))))
            azimuth = math.degrees(math.atan2(avg_normal[0], avg_normal[1])) % 360
            compass = ("N" if azimuth < 45 or azimuth >= 315 else
                       "E" if azimuth < 135 else
                       "S" if azimuth < 225 else "W")
            direction = f"roof_{compass}_{tilt_deg}deg"
            component = "47.1"

        # Convex hull in the plane's local 2D coordinate system
        if nz < 0.9:
            ref = np.array([0.0, 0.0, 1.0])
        else:
            ref = np.array([1.0, 0.0, 0.0])

        u_axis = np.cross(avg_normal, ref)
        u_axis /= np.linalg.norm(u_axis)
        v_axis = np.cross(avg_normal, u_axis)
        v_axis /= np.linalg.norm(v_axis)

        relative = verts - center
        pts_2d = list(zip(
            (relative @ u_axis).tolist(),
            (relative @ v_axis).tolist(),
        ))
        hull_2d = _convex_hull_2d(pts_2d)

        if len(hull_2d) < 3:
            continue

        hull_3d = np.array([
            center + u * u_axis + v * v_axis
            for u, v in hull_2d
        ])

        facades.append(Facade(
            vertices=hull_3d,
            normal=avg_normal,
            component_tag=component,
            label=f"{direction}_{idx}",
            index=idx,
        ))
        idx += 1

    return facades if facades else None


def _extract_convex_hull(
    mesh: "trimesh.Trimesh",
    min_area_m2: float = 1.0,
    **_kwargs: object,
) -> Optional[list[Facade]]:
    """Extract facades from the mesh's convex hull footprint.

    Simplest method: projects mesh to 2D, computes convex hull, extrudes
    walls from ground to mesh height, adds flat roof. Always produces a
    clean result regardless of mesh complexity.
    """
    height = float(mesh.vertices[:, 2].max())
    if height < 0.1:
        return None

    points_2d = [(float(v[0]), float(v[1])) for v in mesh.vertices]
    hull = _convex_hull_2d(points_2d)
    if len(hull) < 3:
        return None

    # Reuse the footprint-to-building logic (walls from edges + flat roof)
    building = _footprint_to_building(
        enu_coords=hull,
        center_lat=0, center_lon=0,  # placeholder, overwritten by caller
        height=height,
        num_stories=1,
        roof_type_str="flat",
        roof_pitch_deg=0,
        name="",
    )
    return building.facades if building.facades else None


def _extract_meshlab(
    mesh: "trimesh.Trimesh",
    min_area_m2: float = 1.0,
    **_kwargs: object,
) -> Optional[list[Facade]]:
    """Extract facades using PyMeshLab's face-normal selection + connected components.

    Industry-standard MeshLab pipeline:
    1. For each cardinal direction, select faces by normal condition
    2. Split selected faces into connected components
    3. Filter by area
    4. Compute convex hull per component → Facade

    This is MeshLab's native approach — battle-tested by thousands of companies.
    """
    import pymeshlab
    import tempfile
    import os

    # Export trimesh to temp file for PyMeshLab (it needs a file path)
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as f:
        mesh.export(f.name)
        tmp_path = f.name

    try:
        building_center = mesh.centroid.copy()

        # Direction conditions: muParser syntax for face normals (fnx, fny, fnz)
        direction_configs = [
            ("north_wall", "fny > 0.85",                        "21.1"),
            ("south_wall", "fny < -0.85",                       "21.1"),
            ("east_wall",  "fnx > 0.85",                        "21.1"),
            ("west_wall",  "fnx < -0.85",                       "21.1"),
            ("roof_flat",  "fnz > 0.95",                        "47.1"),
            ("roof_slope", "fnz > 0.3 && fnz <= 0.95 && fny > 0.3",  "47.1"),
            ("roof_slope", "fnz > 0.3 && fnz <= 0.95 && fny < -0.3", "47.1"),
            ("roof_slope", "fnz > 0.3 && fnz <= 0.95 && fnx > 0.3",  "47.1"),
            ("roof_slope", "fnz > 0.3 && fnz <= 0.95 && fnx < -0.3", "47.1"),
        ]

        facades: list[Facade] = []
        idx = 0

        for base_label, condition, component_tag in direction_configs:
            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(tmp_path)

            ms.compute_selection_by_condition_per_face(condselect=condition)
            n_selected = ms.current_mesh().selected_face_number()
            if n_selected < 3:
                continue

            # Extract selected faces as new mesh
            ms.generate_from_selected_faces()
            ms.set_current_mesh(1)

            # Split into connected components
            ms.generate_splitting_by_connected_components()

            # Process each component (meshes 2+ are the components)
            for i in range(2, ms.mesh_number()):
                ms.set_current_mesh(i)
                cm = ms.current_mesh()
                if cm.face_number() < 3:
                    continue

                verts = cm.vertex_matrix()
                face_normals = cm.face_normal_matrix()

                # Compute area-weighted average normal
                # PyMeshLab doesn't expose per-face area easily, use simple average
                avg_normal = face_normals.mean(axis=0)
                norm_len = np.linalg.norm(avg_normal)
                if norm_len < 1e-9:
                    continue
                avg_normal /= norm_len

                center = verts.mean(axis=0)

                # Estimate area from vertex bounding box projection onto plane
                # More accurate: use trimesh for this component
                import trimesh as _tm
                comp_mesh = _tm.Trimesh(vertices=verts, faces=cm.face_matrix())
                comp_area = float(comp_mesh.area)

                if comp_area < min_area_m2:
                    continue

                # Orient normal outward
                if np.dot(avg_normal, center - building_center) < 0:
                    avg_normal = -avg_normal

                # Skip downward / ground level
                if avg_normal[2] < -0.3:
                    continue
                if center[2] < 0.3:
                    continue

                # Classify
                nz = abs(avg_normal[2])
                if nz < 0.3:
                    azimuth = math.degrees(math.atan2(avg_normal[0], avg_normal[1])) % 360
                    direction = ("north_wall" if azimuth < 45 or azimuth >= 315 else
                                 "east_wall" if azimuth < 135 else
                                 "south_wall" if azimuth < 225 else "west_wall")
                elif nz > 0.95:
                    direction = "roof_flat"
                else:
                    tilt = round(math.degrees(math.acos(min(nz, 1.0))))
                    azimuth = math.degrees(math.atan2(avg_normal[0], avg_normal[1])) % 360
                    compass = ("N" if azimuth < 45 or azimuth >= 315 else
                               "E" if azimuth < 135 else
                               "S" if azimuth < 225 else "W")
                    direction = f"roof_{compass}_{tilt}deg"

                # Convex hull
                if nz < 0.9:
                    ref = np.array([0.0, 0.0, 1.0])
                else:
                    ref = np.array([1.0, 0.0, 0.0])
                u_axis = np.cross(avg_normal, ref)
                u_axis /= np.linalg.norm(u_axis)
                v_axis = np.cross(avg_normal, u_axis)
                v_axis /= np.linalg.norm(v_axis)

                relative = verts - center
                pts_2d = list(zip(
                    (relative @ u_axis).tolist(),
                    (relative @ v_axis).tolist(),
                ))
                hull_2d = _convex_hull_2d(pts_2d)
                if len(hull_2d) < 3:
                    continue

                hull_3d = np.array([center + u * u_axis + v * v_axis for u, v in hull_2d])

                facades.append(Facade(
                    vertices=hull_3d,
                    normal=avg_normal,
                    component_tag=component_tag,
                    label=f"{direction}_{idx}",
                    index=idx,
                ))
                idx += 1

        return facades if facades else None
    finally:
        os.unlink(tmp_path)


# --- Modular extraction dispatcher ---

EXTRACTION_METHODS: dict[str, callable] = {
    "region_growing": _extract_region_growing,
    "convex_hull": _extract_convex_hull,
    "meshlab": _extract_meshlab,
}


def extract_facades(
    mesh: "trimesh.Trimesh",
    method: str = "region_growing",
    min_area_m2: float = 1.0,
    **kwargs: object,
) -> list[Facade]:
    """Extract facades from a mesh using the specified method.

    Available methods: region_growing (default), convex_hull, meshlab.
    Falls back to convex_hull if the chosen method returns no results.
    """
    fn = EXTRACTION_METHODS.get(method, _extract_region_growing)
    result = fn(mesh, min_area_m2=min_area_m2, **kwargs)
    if result:
        return result
    if method != "convex_hull":
        result = _extract_convex_hull(mesh, min_area_m2=min_area_m2)
    return result or []


def build_building_from_mesh(
    mesh_data: bytes,
    file_type: str,
    lat: float,
    lon: float,
    height: Optional[float] = None,
    num_stories: int = 1,
    roof_type: str = "flat",
    roof_pitch_deg: float = 0.0,
    name: str = "",
    min_facade_area: float = 1.0,
    method: str = "region_growing",
) -> Building:
    """Create a Building from an OBJ/PLY/STL mesh file.

    Computes a 2D convex hull footprint from the mesh vertices and creates
    wall facades from the footprint edges. Mesh units are assumed to be meters.

    The mesh is centered at the origin; the caller provides GPS coordinates
    (lat/lon) for the building location. Height is auto-detected from the
    mesh bounding box if not provided.

    This is a simplified approach for Phase 1. Full RANSAC plane segmentation
    (extracting individual wall planes from the mesh) is planned for later.
    """
    import io
    import trimesh

    mesh = trimesh.load(io.BytesIO(mesh_data), file_type=file_type)

    # Handle Scene objects (e.g., OBJ with multiple groups)
    if isinstance(mesh, trimesh.Scene):
        meshes = [g for g in mesh.geometry.values() if isinstance(g, trimesh.Trimesh)]
        if not meshes:
            raise ValueError("No valid mesh geometry found in file")
        mesh = trimesh.util.concatenate(meshes)

    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.vertices) == 0:
        raise ValueError("Could not load mesh or mesh is empty")

    # OBJ/GLTF/FBX files typically use Y-up convention.
    # PLY/STL files typically use Z-up.
    # We need Z-up (ENU: x=East, y=North, z=Up) for our coordinate system.
    # Y-up → Z-up rotation: (x, y, z) → (x, -z, y)
    if file_type in ("obj", "glb", "gltf"):
        verts = mesh.vertices.copy()
        mesh.vertices[:, 0] = verts[:, 0]   # x stays
        mesh.vertices[:, 1] = -verts[:, 2]  # y_new = -z_old (forward)
        mesh.vertices[:, 2] = verts[:, 1]   # z_new = y_old (up)

    # Center the mesh at origin
    centroid = mesh.centroid.copy()
    mesh.vertices -= centroid

    # Get building height from mesh bounding box (Z is up after rotation)
    z_min = float(mesh.vertices[:, 2].min())
    z_max = float(mesh.vertices[:, 2].max())
    mesh_height = z_max - z_min

    if mesh_height < 1e-6:
        raise ValueError("Mesh has zero height")

    # Auto-scale: OBJ/STL files from 3D modelling tools often use arbitrary
    # units (cm, inches, etc.). If the user provides a target height, scale
    # the mesh to match. Otherwise auto-detect: if the raw height looks
    # unreasonable (>50m for a building), assume non-meter units and default
    # to 8m.
    if height is not None and height > 0:
        scale = height / mesh_height
    elif mesh_height > 50:
        height = 8.0
        scale = height / mesh_height
    else:
        height = round(mesh_height, 1)
        scale = 1.0

    mesh.vertices *= scale

    # Recompute after scaling
    z_min = float(mesh.vertices[:, 2].min())
    mesh.vertices[:, 2] -= z_min  # shift so ground is at z=0

    name = name or "Mesh building"

    # Try facet-based extraction first (gives real wall/roof planes from mesh).
    # Fall back to convex hull if facets aren't available.
    facades = extract_facades(mesh, method=method, min_area_m2=min_facade_area)

    if facades:
        xs = [float(v[0]) for f in facades for v in f.vertices]
        ys = [float(v[1]) for f in facades for v in f.vertices]
        width = round(max(xs) - min(xs), 1) if xs else 0
        depth = round(max(ys) - min(ys), 1) if ys else 0

        building = Building(
            lat=lat,
            lon=lon,
            width=width,
            depth=depth,
            height=height,
            heading_deg=0.0,
            roof_type=RoofType(roof_type),
            roof_pitch_deg=roof_pitch_deg,
            num_stories=num_stories,
            facades=facades,
            label=name,
        )
    else:
        # Fallback: convex hull footprint + flat roof
        points_2d = [(float(v[0]), float(v[1])) for v in mesh.vertices]
        hull = _convex_hull_2d(points_2d)
        if len(hull) < 3:
            raise ValueError("Mesh footprint has fewer than 3 hull points")
        building = _footprint_to_building(
            enu_coords=hull,
            center_lat=lat,
            center_lon=lon,
            height=height,
            num_stories=num_stories,
            roof_type_str=roof_type,
            roof_pitch_deg=roof_pitch_deg,
            name=name,
        )

    # Attach processed mesh geometry for 3D viewer rendering.
    # Stored as a compact dict of flat arrays (positions + face indices).
    building._mesh_viewer_data = {
        "positions": mesh.vertices.flatten().round(3).tolist(),
        "indices": mesh.faces.flatten().tolist(),
    }

    return building
