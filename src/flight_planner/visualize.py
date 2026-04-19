"""Data preparation helpers for visualization.

Converts Building/Waypoint objects into JSON-serializable dicts
consumed by the React frontend (Three.js 3D viewer and Leaflet map).
"""

from __future__ import annotations

import math

from .models import Building, meters_per_deg, Waypoint


DIRECTION_COLORS = {
    "north": "#2563eb",  # blue
    "east": "#16a34a",   # green
    "south": "#dc2626",  # red
    "west": "#ca8a04",   # amber
    "roof": "#0891b2",   # teal
    "other": "#9333ea",  # purple
}


def _facade_direction(normal: list[float]) -> str:
    """Classify a facade by cardinal direction from its normal vector."""
    nz = abs(normal[2])
    if nz > 0.3:
        return "roof"
    azimuth = math.degrees(math.atan2(normal[0], normal[1])) % 360
    if azimuth < 45 or azimuth >= 315:
        return "north"
    elif azimuth < 135:
        return "east"
    elif azimuth < 225:
        return "south"
    else:
        return "west"


def _facade_color(normal: list[float]) -> str:
    return DIRECTION_COLORS.get(_facade_direction(normal), DIRECTION_COLORS["other"])


def compute_facade_coverage(
    building: Building,
    waypoints: list[Waypoint],
) -> list[dict]:
    """Per-facade inspection-quality metrics for the coverage diff panel.

    For each facade returns: area, waypoint count, mean |gimbal pitch|,
    mean perpendicularity (cos of angle between -facade_normal and camera
    forward; 1.0 = perfectly perpendicular), and mean stand-off distance.
    """
    import numpy as _np

    # Group waypoints by facade_index (skip transitions — they have no photo
    # and their gimbal angle isn't aligned to any facade).
    wps_by_facade: dict[int, list[Waypoint]] = {}
    for wp in waypoints:
        if getattr(wp, "is_transition", False):
            continue
        wps_by_facade.setdefault(wp.facade_index, []).append(wp)

    out: list[dict] = []
    for facade in building.facades:
        wps = wps_by_facade.get(facade.index, [])
        area = float(facade.width * facade.height)
        if not wps:
            out.append({
                "facade_index": facade.index,
                "label": facade.label,
                "area_m2": round(area, 1),
                "waypoint_count": 0,
                "mean_pitch_abs_deg": None,
                "mean_perpendicularity": None,
                "mean_distance_m": None,
            })
            continue

        normal = _np.asarray(facade.normal, dtype=_np.float64)
        centroid = _np.asarray(facade.center, dtype=_np.float64)
        pitch_abs_sum = 0.0
        perp_sum = 0.0
        dist_sum = 0.0
        for wp in wps:
            h = math.radians(wp.heading_deg)
            # Camera yaw overrides heading when set; KMZ imports leave
            # gimbal_yaw_deg=None (camera follows aircraft).
            if wp.gimbal_yaw_deg is not None:
                h = math.radians(wp.gimbal_yaw_deg)
            p = math.radians(wp.gimbal_pitch_deg)
            # ENU: x=E, y=N. Heading 0 = north = +Y, clockwise.
            fwd = _np.array([
                math.sin(h) * math.cos(p),
                math.cos(h) * math.cos(p),
                math.sin(p),
            ])
            # Perpendicularity: camera forward should oppose facade normal.
            perp = float(_np.dot(fwd, -normal))
            perp_sum += perp
            pitch_abs_sum += abs(wp.gimbal_pitch_deg)
            # Perpendicular distance from waypoint to facade plane.
            pos = _np.array([wp.x, wp.y, wp.z])
            dist_sum += float(abs(_np.dot(pos - centroid, normal)))

        n = len(wps)
        out.append({
            "facade_index": facade.index,
            "label": facade.label,
            "area_m2": round(area, 1),
            "waypoint_count": n,
            "mean_pitch_abs_deg": round(pitch_abs_sum / n, 1),
            "mean_perpendicularity": round(perp_sum / n, 3),
            "mean_distance_m": round(dist_sum / n, 1),
        })
    return out


def _escape_html(text: str) -> str:
    """Escape HTML special characters."""
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#x27;")
    )


def prepare_leaflet_data(
    building: Building,
    waypoints: list[Waypoint],
    mission_area_poly: list[tuple[float, float]] | None = None,
) -> dict:
    """Prepare JSON-serializable data for the 2D Leaflet satellite map viewer.

    ``mission_area_poly`` is an optional list of ``[lat, lon]`` points describing
    an imported DJI mission-area polygon. When present, the frontend renders it
    as a dashed outline distinct from the building footprint.
    """
    center_lat = building.lat
    center_lon = building.lon

    # Building footprint polygon (ground-level vertices of walls)
    ground_coords = []
    for facade in building.facades:
        if abs(facade.normal[2]) < 0.01:  # vertical walls only
            for v in facade.vertices:
                if abs(v[2]) < 0.1:  # ground-level vertices
                    m_lat, m_lon = meters_per_deg(math.radians(center_lat))
                    lat_offset = v[1] / m_lat
                    lon_offset = v[0] / m_lon
                    ground_coords.append([center_lon + lon_offset, center_lat + lat_offset])

    if ground_coords:
        cx = sum(c[0] for c in ground_coords) / len(ground_coords)
        cy = sum(c[1] for c in ground_coords) / len(ground_coords)
        ground_coords.sort(key=lambda c: math.atan2(c[1] - cy, c[0] - cx))
        ground_coords.append(ground_coords[0])  # close polygon

    # Waypoints grouped by facade
    facade_groups: dict[int, list[dict]] = {}
    for wp in waypoints:
        fi = wp.facade_index
        if fi not in facade_groups:
            facade_groups[fi] = []
        facade_groups[fi].append({
            "index": wp.index,
            "lat": wp.lat,
            "lon": wp.lon,
            "alt": round(wp.z, 1),
            "heading": round(wp.heading_deg, 1),
            "gimbal_pitch": round(wp.gimbal_pitch_deg, 1),
            "facade_index": fi,
            "component": wp.component_tag,
            "is_transition": wp.is_transition,
        })

    facade_meta = {}
    for f in building.facades:
        normal_list = f.normal.tolist()
        facade_meta[str(f.index)] = {
            "label": f.label,
            "color": _facade_color(normal_list),
            "direction": _facade_direction(normal_list),
            "azimuth": round(f.azimuth_deg, 0),
            "component": f.component_tag,
        }

    result = {
        "facadeGroups": facade_groups,
        "facadeMeta": facade_meta,
        "flightPath": [[wp.lon, wp.lat] for wp in waypoints],
        "buildingPoly": ground_coords,
        "center": [center_lat, center_lon],
        "buildingLabel": _escape_html(building.label or "Building"),
        "buildingDims": f"{building.width}m x {building.depth}m x {building.height}m",
        "waypointCount": len(waypoints),
        "facadeCount": len(building.facades),
    }
    if mission_area_poly:
        result["missionAreaPoly"] = [[lat, lon] for lat, lon in mission_area_poly]
    return result


def prepare_threejs_data(
    building: Building,
    waypoints: list[Waypoint],
    candidate_facades: list | None = None,
    point_cloud: dict | None = None,
    mission_area: list[tuple[float, float, float]] | None = None,
    mapping_bbox: dict | None = None,
) -> dict:
    """Prepare JSON-serializable data for the 3D Three.js viewer.

    ``point_cloud`` — optional ``{"positions": [...], "colors": [...]}`` flat
    float lists in ENU coordinates (colors 0..1). Rendered as ``THREE.Points``.

    ``mission_area`` — optional list of ``(x, y, z)`` ENU vertices forming the
    imported DJI mission-area polygon. Rendered as a dashed ground outline.

    ``mapping_bbox`` — optional ``{"center": [x, y, z], "axes": [[...], [...], [...]]}``
    oriented bbox in ENU meters, read from the KMZ's 3D-Tiles ``tileset.json``
    (DJI's "Mapping" volume). ``None`` when the KMZ ships no tileset.
    """
    facade_data = []
    for f in building.facades:
        normal_list = f.normal.tolist()
        facade_data.append({
            "vertices": f.vertices.tolist(),
            "normal": normal_list,
            "label": f.label,
            "index": f.index,
            "component": f.component_tag,
            "color": _facade_color(normal_list),
            "direction": _facade_direction(normal_list),
        })

    wp_data = []
    for wp in waypoints:
        wp_data.append({
            "x": round(wp.x, 2),
            "y": round(wp.y, 2),
            "z": round(wp.z, 2),
            "heading": round(wp.heading_deg, 1),
            "gimbal_pitch": round(wp.gimbal_pitch_deg, 1),
            "facade_index": wp.facade_index,
            "index": wp.index,
            "component": wp.component_tag,
            "is_transition": wp.is_transition,
        })

    candidate_data = []
    if candidate_facades:
        for f in candidate_facades:
            normal_list = f.normal.tolist()
            candidate_data.append({
                "vertices": f.vertices.tolist(),
                "normal": normal_list,
                "label": f.label,
                "index": f.index,
                "component": f.component_tag,
                "color": "#666677",
                "direction": _facade_direction(normal_list),
            })

    result = {
        "facades": facade_data,
        "candidateFacades": candidate_data,
        "waypoints": wp_data,
        "buildingLabel": _escape_html(building.label or "Building"),
        "buildingDims": f"{building.width}m x {building.depth}m x {building.height}m",
        "buildingHeight": building.height,
    }
    if point_cloud is not None:
        result["pointCloud"] = point_cloud
    if mission_area:
        result["missionArea"] = {
            "vertices": [[float(x), float(y), float(z)] for x, y, z in mission_area],
        }
    if mapping_bbox is not None:
        result["mappingBox"] = mapping_bbox
    return result
