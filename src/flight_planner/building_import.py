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


def _extract_facades_from_mesh(
    mesh: "trimesh.Trimesh",
    min_area_m2: float = 1.0,
    cluster_facades: bool = False,
) -> Optional[list[Facade]]:
    """Extract real planar surfaces from a mesh using trimesh facet detection.

    Groups coplanar adjacent faces into facets, then creates a Facade for each
    facet above the minimum area threshold. Returns walls, pitched roof slopes,
    and flat roofs — each with correct normals for waypoint generation.

    Returns None if facet detection fails (caller should fall back to convex hull).
    """
    try:
        facets = mesh.facets
        normals = mesh.facets_normal
    except Exception:
        return None

    if len(facets) == 0:
        return None

    # Compute area per facet and building bounding box for occlusion check
    areas = np.array([float(mesh.area_faces[f].sum()) for f in facets])
    bbox = mesh.bounding_box.extents
    max_extent = float(max(bbox))

    # Building center for normal orientation check
    building_center = mesh.centroid.copy()

    facades: list[Facade] = []
    idx = 0

    for face_indices, normal, area in zip(facets, normals, areas):
        if area < min_area_m2:
            continue

        normal = normal / np.linalg.norm(normal)

        # Get unique vertices for this facet
        vert_indices = np.unique(mesh.faces[face_indices].flatten())
        verts_3d = mesh.vertices[vert_indices]
        facet_center = verts_3d.mean(axis=0)

        # Fix flipped normals: ensure normal points AWAY from building center.
        # Without this, facets with inconsistent winding generate waypoints
        # on the wrong side of the building.
        to_facet = facet_center - building_center
        if np.dot(normal, to_facet) < 0:
            normal = -normal

        nz = abs(normal[2])

        # Skip downward-facing surfaces (floors, ceilings) and ground plane
        if normal[2] < -0.5:
            continue

        # Skip surfaces at ground level
        if np.mean(verts_3d[:, 2]) < 0.3:
            continue

        # Occlusion check: cast a ray from the facet center along its outward
        # normal. If the ray hits another part of the mesh, this surface is
        # blocked (e.g., window reveals, door jambs, recessed features) and
        # a drone cannot photograph it from that direction.
        ray_origin = facet_center + normal * 0.05  # slight offset to avoid self-hit
        hits = mesh.ray.intersects_location(
            ray_origins=[ray_origin],
            ray_directions=[normal],
        )
        if len(hits[0]) > 0:
            hit_dist = float(np.linalg.norm(hits[0][0] - ray_origin))
            if hit_dist < max_extent * 0.5:
                continue  # surface is occluded — skip

        # Classify surface type
        if nz < 0.1:
            # Vertical wall
            azimuth = math.degrees(math.atan2(normal[0], normal[1])) % 360
            if azimuth < 45 or azimuth >= 315:
                direction = "north_wall"
            elif azimuth < 135:
                direction = "east_wall"
            elif azimuth < 225:
                direction = "south_wall"
            else:
                direction = "west_wall"
            component = "21.1"
        elif nz > 0.95:
            # Flat roof
            direction = "roof_flat"
            component = "47.1"
        else:
            # Pitched roof slope
            tilt_deg = round(math.degrees(math.acos(nz)))
            azimuth = math.degrees(math.atan2(normal[0], normal[1])) % 360
            if azimuth < 45 or azimuth >= 315:
                compass = "N"
            elif azimuth < 135:
                compass = "E"
            elif azimuth < 225:
                compass = "S"
            else:
                compass = "W"
            direction = f"roof_{compass}_{tilt_deg}deg"
            component = "47.1"

        # Compute bounding polygon: project vertices onto facet plane, hull, back to 3D
        if abs(normal[2]) < 0.9:
            ref = np.array([0.0, 0.0, 1.0])
        else:
            ref = np.array([1.0, 0.0, 0.0])

        u_axis = np.cross(normal, ref)
        u_axis = u_axis / np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        v_axis = v_axis / np.linalg.norm(v_axis)

        center = verts_3d.mean(axis=0)
        relative = verts_3d - center
        u_coords = relative @ u_axis
        v_coords = relative @ v_axis

        pts_2d = list(zip(u_coords.tolist(), v_coords.tolist()))
        hull_2d = _convex_hull_2d(pts_2d)

        if len(hull_2d) < 3:
            continue

        hull_3d = np.array([
            center + u * u_axis + v * v_axis
            for u, v in hull_2d
        ])

        facades.append(Facade(
            vertices=hull_3d,
            normal=normal,
            component_tag=component,
            label=f"{direction}_{idx}",
            index=idx,
        ))
        idx += 1

    return facades if facades else None


def _cluster_facades(
    facades: list[Facade],
    angle_threshold_deg: float = 20.0,
) -> list[Facade]:
    """Merge facades with similar normals that are spatially close.

    Uses agglomerative clustering: iteratively merge the closest pair of
    facades (by normal angle) whose bounding volumes overlap, until no
    more merges are possible. This eliminates redundant waypoint grids
    from adjacent mesh facets on the same wall/roof plane.
    """
    # Build union-find structure
    parent = list(range(len(facades)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        parent[find(i)] = find(j)

    cos_threshold = math.cos(math.radians(angle_threshold_deg))

    # Check all pairs — merge if normals are similar AND spatially close
    for i in range(len(facades)):
        for j in range(i + 1, len(facades)):
            ni = facades[i].normal
            nj = facades[j].normal

            # Normal similarity: dot product > cos(threshold)
            dot = float(np.dot(ni, nj))
            if dot < cos_threshold:
                continue

            # Spatial proximity: check if closest vertices are within
            # half the facade width (nearby on the same surface)
            min_dist = float("inf")
            for vi in facades[i].vertices:
                for vj in facades[j].vertices:
                    d = float(np.linalg.norm(vi - vj))
                    if d < min_dist:
                        min_dist = d
            max_extent = max(facades[i].width, facades[j].width, 2.0)
            if min_dist < max_extent:
                union(i, j)

    # Group facades by cluster
    clusters: dict[int, list[int]] = {}
    for i in range(len(facades)):
        root = find(i)
        clusters.setdefault(root, []).append(i)

    # Merge each cluster into a single facade
    merged: list[Facade] = []
    idx = 0
    for members in clusters.values():
        if len(members) == 1:
            f = facades[members[0]]
            merged.append(Facade(
                vertices=f.vertices,
                normal=f.normal,
                component_tag=f.component_tag,
                label=f.label,
                index=idx,
            ))
        else:
            # Area-weighted average normal
            total_area = 0.0
            avg_normal = np.zeros(3)
            all_verts = []
            component = facades[members[0]].component_tag
            for m in members:
                f = facades[m]
                area = f.width * f.height
                avg_normal += f.normal * area
                total_area += area
                all_verts.append(f.vertices)
                if f.component_tag != "47.1":
                    component = f.component_tag

            avg_normal /= np.linalg.norm(avg_normal)
            combined_verts = np.vstack(all_verts)

            # Recompute convex hull in the plane of the merged facade
            nz = abs(avg_normal[2])
            if nz < 0.9:
                ref = np.array([0.0, 0.0, 1.0])
            else:
                ref = np.array([1.0, 0.0, 0.0])
            u_axis = np.cross(avg_normal, ref)
            u_axis /= np.linalg.norm(u_axis)
            v_axis = np.cross(avg_normal, u_axis)
            v_axis /= np.linalg.norm(v_axis)

            center = combined_verts.mean(axis=0)
            relative = combined_verts - center
            u_coords = relative @ u_axis
            v_coords = relative @ v_axis

            pts_2d = list(zip(u_coords.tolist(), v_coords.tolist()))
            hull_2d = _convex_hull_2d(pts_2d)
            if len(hull_2d) < 3:
                # Can't form hull, keep first facade
                f = facades[members[0]]
                merged.append(Facade(vertices=f.vertices, normal=f.normal,
                                     component_tag=f.component_tag, label=f.label, index=idx))
            else:
                hull_3d = np.array([center + u * u_axis + v * v_axis for u, v in hull_2d])

                # Label from dominant direction
                azimuth = math.degrees(math.atan2(avg_normal[0], avg_normal[1])) % 360
                if nz > 0.3:
                    tilt = round(math.degrees(math.acos(min(nz, 1.0))))
                    compass = "N" if azimuth < 45 or azimuth >= 315 else "E" if azimuth < 135 else "S" if azimuth < 225 else "W"
                    label = f"roof_{compass}_{tilt}deg" if nz < 0.95 else "roof_flat"
                else:
                    label = ("north" if azimuth < 45 or azimuth >= 315 else
                             "east" if azimuth < 135 else
                             "south" if azimuth < 225 else "west") + "_wall"

                merged.append(Facade(
                    vertices=hull_3d,
                    normal=avg_normal,
                    component_tag=component,
                    label=f"{label}_{idx}",
                    index=idx,
                ))
        idx += 1

    return merged


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
    facades = _extract_facades_from_mesh(mesh, min_area_m2=min_facade_area)

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
