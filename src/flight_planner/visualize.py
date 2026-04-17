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
) -> dict:
    """Prepare JSON-serializable data for the 3D Three.js viewer.

    ``point_cloud`` — optional ``{"positions": [...], "colors": [...]}`` flat
    float lists in ENU coordinates (colors 0..1). Rendered as ``THREE.Points``.

    ``mission_area`` — optional list of ``(x, y, z)`` ENU vertices forming the
    imported DJI mission-area polygon. Rendered as a dashed ground outline.
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
    return result
