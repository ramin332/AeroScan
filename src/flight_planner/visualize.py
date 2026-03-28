"""Data preparation helpers for visualization.

Converts Building/Waypoint objects into JSON-serializable dicts
consumed by the React frontend (Three.js 3D viewer and Leaflet map).
"""

from __future__ import annotations

import math

from .models import Building, Waypoint


def _facade_color(index: int) -> str:
    colors = [
        "#2563eb", "#dc2626", "#16a34a", "#ca8a04",
        "#9333ea", "#0891b2", "#ea580c", "#6d28d9",
        "#0d9488", "#65a30d",
    ]
    return colors[index % len(colors)]


def _escape_html(text: str) -> str:
    """Escape HTML special characters."""
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#x27;")
    )


def prepare_leaflet_data(building: Building, waypoints: list[Waypoint]) -> dict:
    """Prepare JSON-serializable data for the 2D Leaflet satellite map viewer."""
    center_lat = building.lat
    center_lon = building.lon

    # Building footprint polygon (ground-level vertices of walls)
    ground_coords = []
    for facade in building.facades:
        if abs(facade.normal[2]) < 0.01:  # vertical walls only
            for v in facade.vertices:
                if abs(v[2]) < 0.1:  # ground-level vertices
                    lat_offset = v[1] / (111132.92 - 559.82 * math.cos(2 * math.radians(center_lat)))
                    lon_offset = v[0] / (111412.84 * math.cos(math.radians(center_lat)))
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
        facade_meta[str(f.index)] = {
            "label": f.label,
            "color": _facade_color(f.index),
            "azimuth": round(f.azimuth_deg, 0),
            "component": f.component_tag,
        }

    return {
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


def prepare_threejs_data(building: Building, waypoints: list[Waypoint]) -> dict:
    """Prepare JSON-serializable data for the 3D Three.js viewer."""
    facade_data = []
    for f in building.facades:
        facade_data.append({
            "vertices": f.vertices.tolist(),
            "normal": f.normal.tolist(),
            "label": f.label,
            "index": f.index,
            "component": f.component_tag,
            "color": _facade_color(f.index),
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

    return {
        "facades": facade_data,
        "waypoints": wp_data,
        "buildingLabel": _escape_html(building.label or "Building"),
        "buildingDims": f"{building.width}m x {building.depth}m x {building.height}m",
        "buildingHeight": building.height,
    }
