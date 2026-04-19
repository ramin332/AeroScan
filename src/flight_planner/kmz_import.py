"""Import DJI Smart3D KMZ files back into AeroScan.

DJI's "mappingObject" / Smart3D Explore missions ship as KMZ bundles that
contain — beyond the usual WPML mission — a reference point cloud + mesh of
the scanned building. This module:

1. Parses `wpmz/template.kml` (mission config + mission-area polygon).
2. Parses `wpmz/waylines.wpml` (per-waypoint placemarks).
3. Loads `wpmz/res/ply/<name>/cloud.ply` (reference point cloud).
4. Reads `wpmz/res/ply/<name>/sfm_geo_desc.json` (local-ENU origin).
5. Runs Poisson surface reconstruction on the point cloud to produce a
   triangle mesh suitable for `build_building_from_mesh()`.

Output is consumed by the ``/api/import-kmz`` endpoint.
"""

from __future__ import annotations

import io
import json
import math
import os
import tempfile
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree as ET

import numpy as np

from ._profiling import timed
from .models import Facade, meters_per_deg

WPML_NS = "{http://www.dji.com/wpmz/1.0.6}"
KML_NS = "{http://www.opengis.net/kml/2.2}"


@dataclass
class SmartObliquePose:
    """One pose of a DJI Smart3D rosette. Yaw is an offset relative to the
    aircraft heading (DJI always uses ``gimbalHeadingYawBase=aircraft`` inside
    ``startSmartOblique`` actions). Pitch is absolute from horizon.
    """

    pitch_deg: float
    yaw_offset_deg: float
    roll_deg: float = 0.0


@dataclass
class ParsedWaypoint:
    """A single waypoint as read from waylines.wpml (WGS84 frame)."""

    index: int
    lon: float
    lat: float
    alt_egm96: float           # absolute altitude (EGM96), meters
    heading_deg: float         # waypointHeadingAngle, 0=N clockwise
    gimbal_pitch_deg: float    # relative to horizon, negative = looking down
    speed_ms: float = 2.0
    # Raw gimbal yaw reading from waypointGimbalYawAngle. Interpretation
    # depends on ``gimbal_yaw_base``: when "aircraft" the value is an offset
    # from the aircraft heading; when "north" the value is a world-frame angle
    # (0 = north, clockwise).
    gimbal_yaw_raw_deg: float = 0.0
    gimbal_heading_mode: str = "smoothTransition"
    gimbal_yaw_base: str = "aircraft"  # "aircraft" or "north"
    # 5-pose SmartOblique rosette attached to this waypoint by the covering
    # action group (see ``actionGroupStartIndex`` / ``actionGroupEndIndex``).
    # Empty when the waypoint isn't inside any SmartOblique action group.
    smart_oblique_poses: list[SmartObliquePose] = field(default_factory=list)


@dataclass
class ImportedKmz:
    """Fully parsed contents of a DJI Smart3D KMZ."""

    name: str                                          # derived from filename or wpmz folder
    ref_lat: float
    ref_lon: float
    ref_alt: float                                     # from sfm_geo_desc.json (or takeOffRefPoint)
    waypoints: list[ParsedWaypoint]
    mission_area_wgs84: list[tuple[float, float, float]]   # (lon, lat, alt)
    mission_config_raw: dict
    point_cloud_ply: Optional[bytes]                   # raw PLY bytes, or None
    # 3D-Tiles tileset.json contents (``root.boundingVolume.box`` = 12 floats,
    # ``root.transform`` = 16 floats, column-major ECEF→local). This is the
    # authoritative "Mapping" OBB shown in DJI's Capture Quality Report UI.
    # ``None`` when the KMZ ships no point-cloud tileset (autoExplore).
    mapping_bbox_raw: Optional[dict] = None


# ---------------------------------------------------------------------------
# KMZ archive parsing
# ---------------------------------------------------------------------------


def _find_members(zf: zipfile.ZipFile) -> dict[str, str]:
    """Locate the key files inside a DJI KMZ.

    Returns a dict with keys: template, waylines, pointcloud, geo_desc, tileset
    (any may be missing).
    """
    members: dict[str, str] = {}
    for name in zf.namelist():
        lower = name.lower()
        if lower.endswith("template.kml"):
            members["template"] = name
        elif lower.endswith("waylines.wpml"):
            members["waylines"] = name
        elif lower.endswith("cloud.ply"):
            members["pointcloud"] = name
        elif lower.endswith("sfm_geo_desc.json"):
            members["geo_desc"] = name
        elif lower.endswith("/tileset.json") or lower.endswith("\\tileset.json"):
            members["tileset"] = name
    return members


def _parse_tileset(xml_bytes: bytes) -> Optional[dict]:
    """Parse a 3D-Tiles ``tileset.json`` and return the Mapping OBB payload.

    The 12-float ``box`` is ``[cx, cy, cz, ux, uy, uz, vx, vy, vz, wx, wy, wz]``
    in the tile's local frame; ``transform`` is a column-major 4×4 that maps
    local → ECEF. Returns ``None`` if the expected fields are absent.
    """
    try:
        doc = json.loads(xml_bytes.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    root = doc.get("root") or {}
    box = (root.get("boundingVolume") or {}).get("box")
    transform = root.get("transform")
    if not (isinstance(box, list) and len(box) == 12):
        return None
    if not (isinstance(transform, list) and len(transform) == 16):
        return None
    return {
        "box": [float(v) for v in box],
        "transform": [float(v) for v in transform],
    }


def _parse_template(xml_bytes: bytes) -> tuple[dict, list[tuple[float, float, float]], Optional[tuple[float, float, float]]]:
    """Parse template.kml.

    Returns (mission_config_raw, mission_area_wgs84, takeoff_ref_lla).
    """
    root = ET.fromstring(xml_bytes)

    mission_cfg: dict = {}
    mc = root.find(f".//{WPML_NS}missionConfig")
    if mc is not None:
        for child in mc:
            tag = child.tag.replace(WPML_NS, "")
            mission_cfg[tag] = (child.text or "").strip()

    # Placemark.Point.coordinates → polygon of lon,lat,alt triples
    poly: list[tuple[float, float, float]] = []
    point_el = root.find(f".//{KML_NS}Placemark/{KML_NS}Point/{KML_NS}coordinates")
    if point_el is not None and point_el.text:
        for line in point_el.text.strip().splitlines():
            parts = line.strip().split(",")
            if len(parts) >= 2:
                lon = float(parts[0])
                lat = float(parts[1])
                alt = float(parts[2]) if len(parts) >= 3 else 0.0
                poly.append((lon, lat, alt))

    # Overlap / height / speed under Folder
    for el in root.iter():
        tag = el.tag.replace(WPML_NS, "").replace(KML_NS, "")
        if tag in ("globalShootHeight", "autoFlightSpeed", "orthoCameraOverlapH",
                   "orthoCameraOverlapW", "height"):
            if el.text:
                mission_cfg[tag] = el.text.strip()

    # Takeoff ref point "lat,lon,alt"
    takeoff: Optional[tuple[float, float, float]] = None
    ref_el = root.find(f".//{WPML_NS}takeOffRefPoint")
    if ref_el is not None and ref_el.text:
        parts = [float(p) for p in ref_el.text.strip().split(",")]
        if len(parts) >= 3:
            takeoff = (parts[0], parts[1], parts[2])

    return mission_cfg, poly, takeoff


def _parse_smart_oblique_groups(
    root: ET.Element,
) -> list[tuple[int, int, list[SmartObliquePose]]]:
    """Scan action groups for ``startSmartOblique`` actions.

    Returns a list of ``(start_index, end_index, poses)`` tuples. Each DJI
    Smart3D mission ships one or more action groups that cover ranges of
    waypoints with a 5-pose rosette (pitch/yaw/roll triples). The covered
    waypoints execute all 5 poses at each station — so a single
    ``waypointGimbalPitchAngle`` on the placemark is only the *transitional*
    pose, not the real photo angles.
    """
    groups: list[tuple[int, int, list[SmartObliquePose]]] = []
    for ag in root.iter(f"{WPML_NS}actionGroup"):
        # Only SmartOblique action groups matter here.
        if ag.find(f".//{WPML_NS}actionActuatorFunc[.='startSmartOblique']") is None:
            is_smart = False
            for func in ag.iter(f"{WPML_NS}actionActuatorFunc"):
                if (func.text or "").strip() == "startSmartOblique":
                    is_smart = True
                    break
            if not is_smart:
                continue

        start_el = ag.find(f"{WPML_NS}actionGroupStartIndex")
        end_el = ag.find(f"{WPML_NS}actionGroupEndIndex")
        if start_el is None or end_el is None:
            continue
        try:
            start_idx = int((start_el.text or "0").strip())
            end_idx = int((end_el.text or "0").strip())
        except ValueError:
            continue

        poses: list[SmartObliquePose] = []
        for pt in ag.iter(f"{WPML_NS}smartObliquePoint"):
            p_el = pt.find(f"{WPML_NS}smartObliqueEulerPitch")
            y_el = pt.find(f"{WPML_NS}smartObliqueEulerYaw")
            r_el = pt.find(f"{WPML_NS}smartObliqueEulerRoll")
            try:
                pitch = float((p_el.text if p_el is not None else "0").strip())
                yaw = float((y_el.text if y_el is not None else "0").strip())
                roll = float((r_el.text.strip() if (r_el is not None and r_el.text) else "0"))
            except (AttributeError, ValueError):
                continue
            poses.append(SmartObliquePose(
                pitch_deg=pitch, yaw_offset_deg=yaw, roll_deg=roll,
            ))
        if poses:
            groups.append((start_idx, end_idx, poses))
    return groups


def _parse_waylines(xml_bytes: bytes) -> list[ParsedWaypoint]:
    """Parse waylines.wpml into a list of ParsedWaypoint."""
    root = ET.fromstring(xml_bytes)
    waypoints: list[ParsedWaypoint] = []

    # Mission-wide gimbal yaw base comes from the first ``gimbalRotate`` action
    # we can find. DJI Smart3D photogrammetry missions set this once in a
    # startActionGroup and leave it for the entire flight. "aircraft" = yaw
    # relative to aircraft heading; "north" = absolute world-frame yaw.
    default_yaw_base = "aircraft"
    for yaw_base_el in root.iter(f"{WPML_NS}gimbalHeadingYawBase"):
        if yaw_base_el.text:
            default_yaw_base = yaw_base_el.text.strip() or "aircraft"
            break

    smart_oblique_groups = _parse_smart_oblique_groups(root)

    for placemark in root.iter(f"{KML_NS}Placemark"):
        coords_el = placemark.find(f"{KML_NS}Point/{KML_NS}coordinates")
        if coords_el is None or not coords_el.text:
            continue
        parts = [p.strip() for p in coords_el.text.strip().split(",")]
        if len(parts) < 2:
            continue
        lon = float(parts[0])
        lat = float(parts[1])

        def _f(tag: str, default: float = 0.0) -> float:
            el = placemark.find(f"{WPML_NS}{tag}")
            if el is None or el.text is None:
                return default
            try:
                return float(el.text.strip())
            except ValueError:
                return default

        def _i(tag: str, default: int = 0) -> int:
            return int(_f(tag, default))

        alt = _f("executeHeight", 0.0)
        heading = 0.0
        gimbal_pitch = 0.0

        hp = placemark.find(f"{WPML_NS}waypointHeadingParam")
        if hp is not None:
            h_el = hp.find(f"{WPML_NS}waypointHeadingAngle")
            if h_el is not None and h_el.text is not None:
                try:
                    heading = float(h_el.text.strip())
                except ValueError:
                    pass

        gimbal_yaw_raw = 0.0
        gimbal_heading_mode = "smoothTransition"
        gp = placemark.find(f"{WPML_NS}waypointGimbalHeadingParam")
        if gp is not None:
            p_el = gp.find(f"{WPML_NS}waypointGimbalPitchAngle")
            if p_el is not None and p_el.text is not None:
                try:
                    gimbal_pitch = float(p_el.text.strip())
                except ValueError:
                    pass
            y_el = gp.find(f"{WPML_NS}waypointGimbalYawAngle")
            if y_el is not None and y_el.text is not None:
                try:
                    gimbal_yaw_raw = float(y_el.text.strip())
                except ValueError:
                    pass
            m_el = gp.find(f"{WPML_NS}waypointGimbalHeadingMode")
            if m_el is not None and m_el.text is not None:
                gimbal_heading_mode = m_el.text.strip() or "smoothTransition"

        speed = _f("waypointSpeed", 2.0)

        wp_index = _i("index", len(waypoints))
        poses_for_wp: list[SmartObliquePose] = []
        for s, e, grp_poses in smart_oblique_groups:
            if s <= wp_index <= e:
                poses_for_wp = list(grp_poses)
                break

        waypoints.append(ParsedWaypoint(
            index=wp_index,
            lon=lon, lat=lat, alt_egm96=alt,
            heading_deg=heading,
            gimbal_pitch_deg=gimbal_pitch,
            speed_ms=speed,
            gimbal_yaw_raw_deg=gimbal_yaw_raw,
            gimbal_heading_mode=gimbal_heading_mode,
            gimbal_yaw_base=default_yaw_base,
            smart_oblique_poses=poses_for_wp,
        ))

    return waypoints


@timed("parse_kmz")
def parse_kmz(data: bytes, name: str = "") -> ImportedKmz:
    """Parse a DJI Smart3D KMZ into its structured pieces.

    If ``data`` is itself a KMZ that contains another KMZ inside it
    (DJI's `autoExplore` folder ships that way), the inner KMZ is recursively
    loaded.
    """
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        members = _find_members(zf)

        # Fallback: single inner KMZ (e.g. autoExplore/<uuid>.kmz)
        if not members.get("template") and not members.get("waylines"):
            inner_kmz = next(
                (n for n in zf.namelist() if n.lower().endswith(".kmz")), None
            )
            if inner_kmz:
                return parse_kmz(zf.read(inner_kmz), name=name or Path(inner_kmz).stem)

        if not members.get("waylines"):
            raise ValueError("KMZ does not contain wpmz/waylines.wpml — not a DJI WPML mission")

        # --- template.kml ---
        mission_cfg: dict = {}
        poly: list[tuple[float, float, float]] = []
        takeoff: Optional[tuple[float, float, float]] = None
        if members.get("template"):
            mission_cfg, poly, takeoff = _parse_template(zf.read(members["template"]))

        # --- waylines.wpml ---
        waypoints = _parse_waylines(zf.read(members["waylines"]))

        # --- sfm_geo_desc.json (preferred ENU origin) ---
        ref_lat = ref_lon = ref_alt = 0.0
        if members.get("geo_desc"):
            geo = json.loads(zf.read(members["geo_desc"]).decode("utf-8"))
            gps = geo.get("ref_GPS", {})
            ref_lat = float(gps.get("latitude", 0.0))
            ref_lon = float(gps.get("longitude", 0.0))
            ref_alt = float(gps.get("altitude", 0.0))
        elif takeoff is not None:
            ref_lat, ref_lon, ref_alt = takeoff
        elif waypoints:
            ref_lat = waypoints[0].lat
            ref_lon = waypoints[0].lon
            ref_alt = waypoints[0].alt_egm96

        # --- cloud.ply bytes ---
        point_cloud_ply: Optional[bytes] = None
        if members.get("pointcloud"):
            point_cloud_ply = zf.read(members["pointcloud"])

        # --- tileset.json (3D-Tiles Mapping OBB) ---
        mapping_bbox_raw: Optional[dict] = None
        if members.get("tileset"):
            mapping_bbox_raw = _parse_tileset(zf.read(members["tileset"]))

    resolved_name = name or "imported_kmz"
    return ImportedKmz(
        name=resolved_name,
        ref_lat=ref_lat,
        ref_lon=ref_lon,
        ref_alt=ref_alt,
        waypoints=waypoints,
        mission_area_wgs84=poly,
        mission_config_raw=mission_cfg,
        point_cloud_ply=point_cloud_ply,
        mapping_bbox_raw=mapping_bbox_raw,
    )


# ---------------------------------------------------------------------------
# WGS84 → ENU helpers
# ---------------------------------------------------------------------------


def waypoints_to_enu(
    waypoints: list[ParsedWaypoint],
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> list[dict]:
    """Convert ParsedWaypoint list into local-ENU waypoint dicts.

    Returned dicts match the shape used by ``models.Waypoint`` fields that the
    viewer cares about (x, y, z, lat, lon, alt, heading_deg, gimbal_pitch_deg).
    """
    m_per_lat, m_per_lon = meters_per_deg(math.radians(ref_lat))
    out: list[dict] = []
    for wp in waypoints:
        x = (wp.lon - ref_lon) * m_per_lon
        y = (wp.lat - ref_lat) * m_per_lat
        z = wp.alt_egm96 - ref_alt

        # Resolve the gimbal yaw to an absolute world-frame angle so downstream
        # code (gimbal rewrite, 3D viewer arrows) can compare it to facade
        # normals directly. DJI WPML:
        #   gimbal_yaw_base="aircraft" → raw yaw is an offset from aircraft heading
        #   gimbal_yaw_base="north"    → raw yaw is already absolute (0=N, CW)
        if wp.gimbal_yaw_base == "north":
            abs_yaw = wp.gimbal_yaw_raw_deg
        else:
            abs_yaw = wp.heading_deg + wp.gimbal_yaw_raw_deg
        # Normalize to [-180, 180]
        abs_yaw = ((abs_yaw + 180.0) % 360.0) - 180.0

        # Resolve each SmartOblique rosette pose to an absolute world-frame yaw
        # so the viewer can render ghost arrows without knowing the yaw base.
        # Inside ``startSmartOblique`` actions DJI always uses aircraft-relative
        # yaw offsets, so we add the heading regardless of the placemark's yaw
        # base.
        poses_abs: list[dict] = []
        for pose in wp.smart_oblique_poses:
            pose_yaw = wp.heading_deg + pose.yaw_offset_deg
            pose_yaw = ((pose_yaw + 180.0) % 360.0) - 180.0
            poses_abs.append({
                "pitch": pose.pitch_deg,
                "yaw": pose_yaw,
                "yaw_offset": pose.yaw_offset_deg,
                "roll": pose.roll_deg,
            })

        out.append({
            "x": x, "y": y, "z": z,
            "lat": wp.lat, "lon": wp.lon, "alt": wp.alt_egm96,
            "heading_deg": wp.heading_deg,
            "gimbal_pitch_deg": wp.gimbal_pitch_deg,
            "gimbal_yaw_deg": abs_yaw,
            "speed_ms": wp.speed_ms,
            "index": wp.index,
            "smart_oblique_poses": poses_abs,
        })
    return out


def polygon_to_enu(
    polygon_wgs84: list[tuple[float, float, float]],
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> list[tuple[float, float, float]]:
    """Convert mission-area polygon (lon,lat,alt triples) to ENU."""
    m_per_lat, m_per_lon = meters_per_deg(math.radians(ref_lat))
    return [
        ((lon - ref_lon) * m_per_lon, (lat - ref_lat) * m_per_lat, alt - ref_alt)
        for lon, lat, alt in polygon_wgs84
    ]


def tight_footprint_from_cloud_xy(
    points_xyz: np.ndarray,
    low_pct: float = 30.0,
    high_pct: float = 70.0,
) -> list[tuple[float, float, float]] | None:
    """Convex hull of mid-height cloud points → tight building footprint.

    DJI mission polygons are generous (often 10–20 m larger than the building
    in each direction) so clipping the reconstructed mesh against the polygon
    still leaves a lot of terrain. Mid-height points almost exclusively live
    on building walls — their 2D convex hull gives us a much tighter clip
    region. Returns a CCW-oriented (x,y,0) polygon, or None if the cloud is
    too sparse.
    """
    pts = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    if len(pts) < 3:
        return None
    z_lo = float(np.percentile(pts[:, 2], low_pct))
    z_hi = float(np.percentile(pts[:, 2], high_pct))
    mid = pts[(pts[:, 2] > z_lo) & (pts[:, 2] < z_hi)]
    if len(mid) < 3:
        mid = pts
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(mid[:, :2])
    except Exception as exc:
        print(f"[kmz_import] tight_footprint_from_cloud_xy hull failed: {exc}")
        return None
    poly_xy = mid[hull.vertices, :2]
    sa = 0.5 * float(np.sum(
        poly_xy[:, 0] * np.roll(poly_xy[:, 1], -1)
        - np.roll(poly_xy[:, 0], -1) * poly_xy[:, 1]
    ))
    if sa < 0:
        poly_xy = poly_xy[::-1]
    return [(float(x), float(y), 0.0) for x, y in poly_xy]


def _points_in_polygon_xy(points_xy: np.ndarray, polygon_xy: np.ndarray) -> np.ndarray:
    """Vectorised even-odd ray-cast. ``points_xy`` (N,2), ``polygon_xy`` (M,2)."""
    n_pts = len(points_xy)
    inside = np.zeros(n_pts, dtype=bool)
    m = len(polygon_xy)
    for i in range(m):
        p1 = polygon_xy[i]
        p2 = polygon_xy[(i + 1) % m]
        y_between = (p1[1] > points_xy[:, 1]) != (p2[1] > points_xy[:, 1])
        if not y_between.any():
            continue
        dy = p2[1] - p1[1]
        if abs(dy) < 1e-12:
            continue
        x_cross = p1[0] + (p2[0] - p1[0]) * (points_xy[:, 1] - p1[1]) / dy
        crosses = y_between & (points_xy[:, 0] < x_cross)
        inside ^= crosses
    return inside


@timed("clip_mesh_to_polygon_xy")
def clip_mesh_to_polygon_xy(
    mesh_bytes: bytes,
    polygon_enu: list[tuple[float, float, float]],
    margin_m: float = 1.0,
) -> bytes:
    """Remove mesh faces whose centroid falls outside the mission-area polygon.

    The DJI mission polygon encloses exactly the building we care about. Faces
    outside (sidewalks, neighbouring structures, reconstruction noise) just
    confuse region-growing. A small outward ``margin_m`` keeps edge-wall
    triangles when the polygon is drawn tight against the building.

    Returns PLY bytes of the clipped mesh. Falls back to the input mesh if
    clipping would leave fewer than 100 triangles (polygon likely mis-scaled).
    """
    import io
    import trimesh

    poly = np.array(polygon_enu, dtype=float)
    if len(poly) > 1 and np.allclose(poly[0, :2], poly[-1, :2]):
        poly = poly[:-1]
    poly_xy = poly[:, :2]
    if len(poly_xy) < 3:
        return mesh_bytes

    # Expand outward by margin_m from the polygon centroid so border faces
    # aren't clipped off (simple Minkowski-style scale; good enough for roughly
    # convex mission polygons).
    center = poly_xy.mean(axis=0)
    vecs = poly_xy - center
    scales = 1.0 + margin_m / np.maximum(np.linalg.norm(vecs, axis=1), 0.1)
    poly_xy_expanded = center + vecs * scales[:, None]

    m = trimesh.load(io.BytesIO(mesh_bytes), file_type="ply", force="mesh", process=False)
    if len(m.faces) == 0:
        return mesh_bytes
    centroids_xy = m.triangles.mean(axis=1)[:, :2]
    keep = _points_in_polygon_xy(centroids_xy, poly_xy_expanded)
    n_keep = int(keep.sum())
    print(f"[kmz_import] clip_mesh_to_polygon_xy: {len(m.faces)} → {n_keep} triangles")
    if n_keep < 100:
        return mesh_bytes  # polygon probably wrong frame; don't clip

    clipped = trimesh.Trimesh(vertices=m.vertices, faces=m.faces[keep], process=False)
    clipped.remove_unreferenced_vertices()
    return clipped.export(file_type="ply")


# ---------------------------------------------------------------------------
# Point-cloud RANSAC plane segmentation (DJI photogrammetry-aware)
# ---------------------------------------------------------------------------


@timed("facades_from_pointcloud_cgal")
def facades_from_pointcloud_cgal(
    points_enu: np.ndarray,
    polygon_enu: list[tuple[float, float, float]] | None,
    *,
    algorithm: str = "region_growing",
    min_points: int = 40,
    epsilon: float = 0.05,
    cluster_epsilon: float = 0.25,
    normal_threshold: float = 0.92,
    probability: float = 0.002,
    rg_k_neighbors: int = 12,
    wall_normal_z_max: float = 0.35,
    roof_normal_z_min: float = 0.70,
    min_wall_length_m: float = 0.6,
    min_wall_area_m2: float = 0.5,
    min_roof_area_m2: float = 0.5,
    min_tilted_area_m2: float = 0.4,
    ground_skip_m: float = 1.0,
    jet_neighbors: int = 18,
    min_density_per_m2: float = 25.0,
    bbox_percentile: float = 5.0,
    enable_coplanar_merge: bool = False,
) -> list[Facade]:
    """Extract facades using CGAL Shape Detection + Verdie parallel-snap.

    Multi-plane segmentation tuned for **NEN-2767 building inspection** —
    biased toward many small facets (dormers, sills, balconies, parapets)
    over few large planes, because the inspection gimbal needs to square
    up on small defect targets, not just the main walls.

    Algorithms (via ``CGAL::Shape_detection``):
      * ``algorithm="region_growing"`` (default) — grows regions locally
        from high-planarity seeds, k-nearest neighbors, stopping when a
        candidate point fails the ε / normal-agreement test. Each point
        belongs to at most one region. By construction this produces
        many small, connected, non-overlapping facets. This is CGAL's
        native answer to "many small local surfaces."
      * ``algorithm="efficient_RANSAC"`` — **Schnabel et al. 2007**,
        greedy global detection. Faster on huge clouds but biased toward
        a few large dominant planes. Use for LOD-style reconstruction.

    Post-processing:
      * **Parallel-snap** (Verdie et al. 2015, "LOD Generation for Urban
        Scenes") — planes whose normals agree within 5° share one refit
        normal, weighted by inlier count. Fixes slight misalignment that
        would otherwise make rectangles slice through each other. CGAL's
        C++ ``regularize_planes`` isn't in the Python bindings, so the
        snap is done in numpy. Companion coplanar-merge is OFF by default
        because it consolidates spatially-separated small facets — the
        opposite of what the inspection gimbal needs.
      * **PCA-oriented rectangle** per region — each plane's bounding
        rectangle is built in its own plane frame (5/95 percentile clamp
        on PCA u/v axes), so pitched roofs, dormers, chamfers, and
        canopies keep their real tilt instead of being axis-aligned.
      * **Density gate** — ghost planes with < ``min_density_per_m2``
        inlier density are dropped.
      * Classification into wall / roof / tilted is only for tagging
        and gimbal-pitch selection — geometry uses each plane's own
        normal, not a forced vertical/horizontal.

    Pipeline:
      1. Clip cloud to ``polygon_enu`` (XY).
      2. Drop bottom ``ground_skip_m`` of Z range (street/terrain).
      3. Insert into ``CGAL::Point_set_3``, estimate normals with
         ``jet_estimate_normals``.
      4. Run region-growing (or efficient-RANSAC).
      5. Fit plane per region via SVD.
      6. Parallel-snap regularization.
      7. Density gate + PCA-oriented rectangle + classification.
    """
    try:
        from CGAL.CGAL_Kernel import Point_3
        from CGAL.CGAL_Point_set_3 import Point_set_3
        from CGAL.CGAL_Shape_detection import efficient_RANSAC, region_growing
        from CGAL.CGAL_Point_set_processing_3 import jet_estimate_normals
    except ImportError as e:
        raise RuntimeError(f"CGAL Python bindings not available: {e}")

    import re as _re

    pts_all = np.asarray(points_enu, dtype=np.float64).reshape(-1, 3)
    if len(pts_all) < min_points * 2:
        return []

    if polygon_enu is not None and len(polygon_enu) >= 3:
        poly = np.asarray(polygon_enu, dtype=float)
        if len(poly) > 1 and np.allclose(poly[0, :2], poly[-1, :2]):
            poly = poly[:-1]
        mask = _points_in_polygon_xy(pts_all[:, :2], poly[:, :2])
        pts_all = pts_all[mask]
        if len(pts_all) < min_points * 2:
            return []

    ground_z = float(np.percentile(pts_all[:, 2], 5.0))
    top_z = float(np.percentile(pts_all[:, 2], 98.0))
    pts_all = pts_all[pts_all[:, 2] >= ground_z + ground_skip_m]
    if len(pts_all) < min_points * 2:
        return []

    ps = Point_set_3()
    ps.add_normal_map()
    for p in pts_all:
        ps.insert(Point_3(float(p[0]), float(p[1]), float(p[2])))
    jet_estimate_normals(ps, jet_neighbors)

    shape_map = ps.add_int_map("aeroscan_shape")
    algo_name = algorithm.lower().strip()
    if algo_name == "region_growing":
        # CGAL region_growing — grows regions locally from high-planarity
        # seeds. Each point belongs to at most one region; regions stop
        # at the first neighbor that doesn't match (distance +
        # normal-agreement). Produces many small connected regions by
        # design — the native answer to "many small facets" without
        # manual subdivision.
        n_regions = region_growing(
            ps, shape_map,
            min_points=int(min_points),
            epsilon=float(epsilon),
            cluster_epsilon=float(cluster_epsilon),
            normal_treshold=float(normal_threshold),  # sic: CGAL typo
            k=int(rg_k_neighbors),
        )
        descriptions = [f"region {i}" for i in range(int(n_regions))]
    else:
        # Efficient-RANSAC (Schnabel 2007) — greedy global detection.
        # Faster on huge clouds but biased toward a few large dominant
        # planes. Keep as an alternative for LOD-style reconstruction.
        descriptions = efficient_RANSAC(
            ps, shape_map,
            min_points=int(min_points),
            epsilon=float(epsilon),
            cluster_epsilon=float(cluster_epsilon),
            normal_threshold=float(normal_threshold),
            probability=float(probability),
            planes=True,
        )

    # Bucket point indices by shape/region id.
    n_pts = ps.number_of_points()
    buckets: dict[int, list[int]] = {}
    for i in range(n_pts):
        sid = shape_map.get(i)
        if sid < 0:
            continue
        buckets.setdefault(sid, []).append(i)

    # Fit a plane per bucket via SVD (works for both algorithms; region
    # growing doesn't return plane equations, and efficient_RANSAC's
    # parser was fragile on non-English locales).
    plane_params: dict[int, tuple[np.ndarray, float]] = {}
    for sid, idxs in buckets.items():
        if len(idxs) < 3:
            continue
        pts_sub = pts_all[np.asarray(idxs, dtype=np.int64)]
        centroid = pts_sub.mean(axis=0)
        _, _, vh = np.linalg.svd(pts_sub - centroid, full_matrices=False)
        normal = vh[2]
        normal /= float(np.linalg.norm(normal)) or 1.0
        d = float(np.dot(normal, centroid))
        plane_params[sid] = (normal, d)

    # ---- Plane regularization (Verdie et al. 2015, LOD-Generation) ----
    # CGAL's C++ regularize_planes isn't exposed in the Python bindings,
    # so we do the two most visually-important steps here:
    #   (1) Orientation clustering + parallel snap — planes whose normals
    #       agree within `parallel_tol_deg` share one refit normal
    #       (weighted mean by inlier count).
    #   (2) Coplanar merge — within an orientation cluster, planes whose
    #       signed-distance d values agree within `coplanar_tol_m` pool
    #       their inliers. This removes duplicate hypotheses fit to the
    #       same wall with a 5 mm offset (the "plates through each other"
    #       artifact of raw Schnabel RANSAC).
    parallel_cos = float(np.cos(np.radians(5.0)))
    coplanar_tol = max(0.10, float(epsilon) * 1.5)

    # Keep only buckets that actually received inliers.
    live = [sid for sid, ix in buckets.items() if len(ix) >= min_points]

    # Group shape ids by normal similarity (greedy union-find).
    groups: list[list[int]] = []
    assigned: set[int] = set()
    for sid in live:
        if sid in assigned:
            continue
        n_a, _ = plane_params[sid]
        grp = [sid]
        assigned.add(sid)
        for sid_b in live:
            if sid_b in assigned:
                continue
            n_b, _ = plane_params[sid_b]
            if abs(float(np.dot(n_a, n_b))) >= parallel_cos:
                grp.append(sid_b)
                assigned.add(sid_b)
        groups.append(grp)

    # For each group: (1) snap to common normal, (2) merge coplanar subs.
    regularized: dict[int, tuple[np.ndarray, float, list[int]]] = {}
    for grp in groups:
        weights = np.array([len(buckets[s]) for s in grp], dtype=float)
        # Flip normals inside the group to align before averaging.
        n0 = plane_params[grp[0]][0]
        aligned_normals = []
        for s in grp:
            n_s = plane_params[s][0].copy()
            if float(np.dot(n_s, n0)) < 0:
                n_s = -n_s
            aligned_normals.append(n_s)
        nstack = np.stack(aligned_normals)
        common_n = (nstack * weights[:, None]).sum(axis=0)
        cnl = float(np.linalg.norm(common_n))
        if cnl > 1e-9:
            common_n = common_n / cnl
        else:
            common_n = n0

        # Signed-distance (plane eq: n·x = d) per sub-plane using the
        # common normal, computed from inliers for robustness.
        sub_entries = []
        for s in grp:
            inl = pts_all[np.asarray(buckets[s], dtype=np.int64)]
            d_s = float(np.mean(inl @ common_n))
            sub_entries.append((d_s, s))

        # Coplanar merge is OFF by default — we *want* many small facets
        # (NEN-2767 inspection targets). The merge step consolidates
        # spatially-separated facets on the same wall plane into one big
        # plane, which is the opposite of what the gimbal needs. Enable
        # only when the goal is LOD-style reconstruction.
        sub_entries.sort(key=lambda t: t[0])
        if enable_coplanar_merge:
            merged: list[tuple[float, list[int]]] = []
            for d_s, s in sub_entries:
                if merged and abs(merged[-1][0] - d_s) <= coplanar_tol:
                    prev_d, prev_ids = merged[-1]
                    prev_w = sum(len(buckets[pid]) for pid in prev_ids)
                    new_w = len(buckets[s])
                    new_d = (prev_d * prev_w + d_s * new_w) / (prev_w + new_w)
                    prev_ids.append(s)
                    merged[-1] = (new_d, prev_ids)
                else:
                    merged.append((d_s, [s]))
        else:
            merged = [(d_s, [s]) for d_s, s in sub_entries]

        # Emit one regularized plane per merged cluster.
        for d_final, ids in merged:
            pooled: list[int] = []
            for s in ids:
                pooled.extend(buckets[s])
            # Pick a representative sid (biggest sub-plane gets to own it).
            rep = max(ids, key=lambda x: len(buckets[x]))
            regularized[rep] = (common_n, d_final, pooled)

    merged_count = len(live) - len(regularized)
    if merged_count > 0:
        print(
            f"[cgal facades] regularization: {len(live)} raw planes → "
            f"{len(regularized)} after parallel-snap + coplanar-merge "
            f"({merged_count} merged; Verdie 2015)"
        )

    cloud_centroid = pts_all.mean(axis=0)
    facades: list[Facade] = []
    wall_index = 0
    roof_index = 0
    tilted_index = 0
    rejected_small = 0

    for sid, (reg_normal, _reg_d, pooled_idxs) in regularized.items():
        if len(pooled_idxs) < min_points:
            continue
        inl_pts = pts_all[np.asarray(pooled_idxs, dtype=np.int64)]
        normal = reg_normal.copy()
        nlen = float(np.linalg.norm(normal))
        if nlen < 1e-9:
            continue
        normal = normal / nlen
        nz = abs(float(normal[2]))

        # Orient normal outward (away from cloud centroid).
        plane_centroid = inl_pts.mean(axis=0)
        if np.dot(normal, plane_centroid - cloud_centroid) < 0:
            normal = -normal

        # Build an in-plane orthonormal basis (u_dir, v_dir).
        # For near-vertical walls prefer u_dir horizontal (X-Y plane) so v_dir
        # stays close to world-up — keeps "wall rectangles" upright.
        # For anything tilted/horizontal fall back to a PCA basis on the plane.
        world_up = np.array([0.0, 0.0, 1.0])
        horiz = np.array([normal[1], -normal[0], 0.0])
        hnorm = float(np.linalg.norm(horiz))
        if nz <= wall_normal_z_max and hnorm > 1e-6:
            u_dir = horiz / hnorm
            v_dir = np.cross(normal, u_dir)
            v_dir /= float(np.linalg.norm(v_dir)) or 1.0
        else:
            # Project inliers into the plane and PCA for best in-plane axes.
            centered = inl_pts - plane_centroid
            centered = centered - np.outer(centered @ normal, normal)
            if len(centered) >= 3:
                _, _, vh = np.linalg.svd(centered, full_matrices=False)
                u_dir = vh[0]
                u_dir = u_dir - float(np.dot(u_dir, normal)) * normal
                u_len = float(np.linalg.norm(u_dir))
                if u_len < 1e-6:
                    continue
                u_dir /= u_len
            else:
                ref = world_up if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
                u_dir = np.cross(normal, ref)
                u_dir /= float(np.linalg.norm(u_dir)) or 1.0
            v_dir = np.cross(normal, u_dir)
            v_dir /= float(np.linalg.norm(v_dir)) or 1.0

        # Oriented bounding rectangle in the plane frame.
        #
        # Size-adaptive trim: percentile clamps remove stragglers on
        # large planes (roofs, walls with noisy edges) but squeeze real
        # inliers off the edge of small facets (sills, dormers). So:
        #   - Small inlier sets → use true min/max (no trim).
        #   - Large inlier sets → use percentile trim to reject noise.
        # The switch point scales with plane area so both extremes fit
        # their target cleanly.
        rel = inl_pts - plane_centroid
        us = rel @ u_dir
        vs = rel @ v_dir
        u_lo_raw, u_hi_raw = float(us.min()), float(us.max())
        v_lo_raw, v_hi_raw = float(vs.min()), float(vs.max())
        raw_area = max((u_hi_raw - u_lo_raw) * (v_hi_raw - v_lo_raw), 1e-6)

        if raw_area < 2.0:
            # Small facet — trust the inliers, use raw extents.
            u_min, u_max = u_lo_raw, u_hi_raw
            v_min, v_max = v_lo_raw, v_hi_raw
        else:
            # Large facet — trim percentile outliers. Use a tighter trim
            # for bigger planes since noise-driven stretching scales with
            # plane extent.
            pct_scale = min(1.0, raw_area / 20.0)
            pct_lo = float(bbox_percentile) * (0.4 + 0.6 * pct_scale)
            pct_hi = 100.0 - pct_lo
            u_min = float(np.percentile(us, pct_lo))
            u_max = float(np.percentile(us, pct_hi))
            v_min = float(np.percentile(vs, pct_lo))
            v_max = float(np.percentile(vs, pct_hi))
        width = u_max - u_min
        height = v_max - v_min
        area = max(width * height, 1e-6)

        # Density gate: reject "ghost planes" where a handful of inliers
        # got spread across a huge rectangle floating in the air. Only
        # keep planes with enough inliers per m² to be a real surface.
        inlier_density = len(inl_pts) / area
        if inlier_density < min_density_per_m2:
            rejected_small += 1
            continue

        # Classify + size gate.
        if nz <= wall_normal_z_max:
            # Min-dimension check uses the smaller of (min_wall_length,
            # 0.4m floor) so we keep small sills, balconies, dormer
            # walls — all targets for the inspection gimbal.
            min_dim = min(min_wall_length_m, 0.4)
            if (
                min(width, height) < min_dim
                or max(width, height) < min_wall_length_m
                or area < min_wall_area_m2
            ):
                rejected_small += 1
                continue
            tag, label_pref = "21.1", f"wall_{wall_index}"
            wall_index += 1
        elif nz >= roof_normal_z_min:
            if area < min_roof_area_m2:
                rejected_small += 1
                continue
            tag, label_pref = "27.1", f"roof_{roof_index}"
            roof_index += 1
        else:
            # Tilted plane (dormer, pitched roof slope, canopy, chamfer).
            if area < min_tilted_area_m2:
                rejected_small += 1
                continue
            tag, label_pref = "27.2", f"tilted_{tilted_index}"
            tilted_index += 1

        base = plane_centroid + u_min * u_dir + v_min * v_dir
        corners = np.array([
            base,
            base + width * u_dir,
            base + width * u_dir + height * v_dir,
            base + height * v_dir,
        ])
        facades.append(Facade(
            vertices=corners,
            normal=normal,
            component_tag=tag,
            label=label_pref,
            index=len(facades),
        ))

    print(
        f"[cgal facades] {algo_name} detected {len(descriptions)} shapes → "
        f"{len(regularized)} regularized → {len(facades)} facets "
        f"({wall_index} walls + {roof_index} roofs + {tilted_index} tilted; "
        f"{rejected_small} rejected) from {len(pts_all)} pts "
        f"(ε={epsilon}, cluster_ε={cluster_epsilon}, k={rg_k_neighbors})"
    )
    return facades


def _ransac_2d_lines(
    points_xy: np.ndarray,
    *,
    distance_threshold: float = 0.25,
    min_inliers: int = 250,
    max_lines: int = 20,
    iters_per_line: int = 400,
    seed: int = 42,
) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Iteratively fit 2D lines via RANSAC; returns [(inlier_idx, centroid, direction), ...].

    ``direction`` is a unit 2D tangent; the implicit line passes through
    ``centroid`` with normal perpendicular to ``direction``.
    """
    rng = np.random.default_rng(seed)
    n = len(points_xy)
    if n < min_inliers:
        return []
    alive = np.ones(n, dtype=bool)
    results: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for _line_i in range(max_lines):
        active_idx = np.where(alive)[0]
        if len(active_idx) < min_inliers:
            break
        active_pts = points_xy[active_idx]
        best_inl: np.ndarray | None = None
        best_normal: np.ndarray | None = None
        best_centroid: np.ndarray | None = None
        for _ in range(iters_per_line):
            i0, i1 = rng.choice(len(active_pts), size=2, replace=False)
            p0 = active_pts[i0]
            p1 = active_pts[i1]
            d = p1 - p0
            dl = float(np.linalg.norm(d))
            if dl < 1e-6:
                continue
            d /= dl
            norm = np.array([-d[1], d[0]])
            dist = np.abs((active_pts - p0) @ norm)
            inl = np.where(dist <= distance_threshold)[0]
            if best_inl is None or len(inl) > len(best_inl):
                best_inl = inl
                best_normal = norm
                best_centroid = p0
        if best_inl is None or len(best_inl) < min_inliers:
            break
        # SVD refinement on inlier set
        inl_pts = active_pts[best_inl]
        c = inl_pts.mean(axis=0)
        _, _, vh = np.linalg.svd(inl_pts - c, full_matrices=False)
        line_dir = vh[0]
        norm_ref = np.array([-line_dir[1], line_dir[0]])
        dist = np.abs((active_pts - c) @ norm_ref)
        inl_refined = np.where(dist <= distance_threshold)[0]
        if len(inl_refined) < min_inliers:
            break
        global_inl = active_idx[inl_refined]
        results.append((global_inl, c, line_dir))
        alive[global_inl] = False
    return results


@timed("facades_from_pointcloud_ransac")
def facades_from_pointcloud_ransac(
    points_enu: np.ndarray,
    polygon_enu: list[tuple[float, float, float]] | None,
    *,
    wall_distance_threshold: float = 0.25,
    wall_min_inliers: int = 300,
    max_walls: int = 20,
    wall_slice_low_frac: float = 0.25,
    wall_slice_high_frac: float = 0.90,
    roof_distance_threshold: float = 0.30,
    roof_min_inliers: int = 500,
    roof_normal_z_min: float = 0.85,
    ground_skip_m: float = 1.0,
    min_wall_length_m: float = 2.0,
    min_wall_area_m2: float = 6.0,
    min_roof_area_m2: float = 8.0,
    num_iterations: int = 500,
) -> list[Facade]:
    """Extract facades via vertical-prior RANSAC on a DJI point cloud.

    Walls are treated as 2D line fits on a top-down projection of a
    mid-height Z slice — exploits the vertical-wall prior, so the fit is
    low-DOF and robust to photogrammetric noise. Roofs come from one
    horizontal-plane 3D RANSAC on the upper part of the cloud.

    Pipeline:
      1. Clip cloud to ``polygon_enu`` (XY).
      2. Compute ground_z / top_z percentiles; drop points below
         ``ground_z + ground_skip_m``.
      3. Wall pass: take points with Z in
         [low_frac, high_frac] × (top_z - ground_z). Project to XY, run
         iterative 2D-line RANSAC. Each line becomes a wall extruded from
         ground_z to top_z.
      4. Roof pass: run 3D plane RANSAC on points with Z > high_frac;
         accept only if |n_z| ≥ ``roof_normal_z_min``.
    """
    import open3d as o3d  # deferred: heavy import

    pts_all = np.asarray(points_enu, dtype=np.float64).reshape(-1, 3)
    if len(pts_all) < wall_min_inliers * 2:
        return []

    if polygon_enu is not None and len(polygon_enu) >= 3:
        poly = np.asarray(polygon_enu, dtype=float)
        if len(poly) > 1 and np.allclose(poly[0, :2], poly[-1, :2]):
            poly = poly[:-1]
        mask = _points_in_polygon_xy(pts_all[:, :2], poly[:, :2])
        pts_all = pts_all[mask]
        if len(pts_all) < wall_min_inliers * 2:
            return []

    ground_z = float(np.percentile(pts_all[:, 2], 5.0))
    top_z = float(np.percentile(pts_all[:, 2], 98.0))
    height_range = max(1.0, top_z - ground_z)
    pts_all = pts_all[pts_all[:, 2] >= ground_z + ground_skip_m]
    if len(pts_all) < wall_min_inliers * 2:
        return []

    facades: list[Facade] = []

    # --- Wall pass: 2D line RANSAC on mid-height slice ----------------------
    z_low = ground_z + wall_slice_low_frac * height_range
    z_high = ground_z + wall_slice_high_frac * height_range
    slice_mask = (pts_all[:, 2] >= z_low) & (pts_all[:, 2] <= z_high)
    slice_xyz = pts_all[slice_mask]
    wall_index = 0
    if len(slice_xyz) >= wall_min_inliers:
        slice_xy = slice_xyz[:, :2]
        cloud_centroid_xy = slice_xy.mean(axis=0)
        lines = _ransac_2d_lines(
            slice_xy,
            distance_threshold=wall_distance_threshold,
            min_inliers=wall_min_inliers,
            max_lines=max_walls,
            iters_per_line=num_iterations,
        )
        for inl_idx, line_c, line_dir in lines:
            inl_pts = slice_xy[inl_idx]
            # Project onto line to get length extent
            proj = (inl_pts - line_c) @ line_dir
            p_min = float(np.percentile(proj, 2))
            p_max = float(np.percentile(proj, 98))
            length = p_max - p_min
            if length < min_wall_length_m:
                continue
            h = top_z - ground_z
            if length * h < min_wall_area_m2:
                continue
            # Outward normal: away from cloud centroid
            normal_xy = np.array([-line_dir[1], line_dir[0]])
            if np.dot(normal_xy, line_c - cloud_centroid_xy) < 0:
                normal_xy = -normal_xy
            normal = np.array([float(normal_xy[0]), float(normal_xy[1]), 0.0])
            end0 = line_c + p_min * line_dir
            end1 = line_c + p_max * line_dir
            corners = np.array([
                [end0[0], end0[1], ground_z],
                [end1[0], end1[1], ground_z],
                [end1[0], end1[1], top_z],
                [end0[0], end0[1], top_z],
            ])
            facades.append(Facade(
                vertices=corners,
                normal=normal,
                component_tag="21.1",
                label=f"wall_{wall_index}",
                index=len(facades),
            ))
            wall_index += 1

    # --- Roof pass: one 3D horizontal plane at the top ---------------------
    roof_mask = pts_all[:, 2] >= ground_z + wall_slice_high_frac * height_range
    roof_xyz = pts_all[roof_mask]
    roof_index = 0
    if len(roof_xyz) >= roof_min_inliers:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(roof_xyz)
        try:
            (a, b, c, _d), inl = pcd.segment_plane(
                distance_threshold=roof_distance_threshold,
                ransac_n=3,
                num_iterations=num_iterations,
            )
            if len(inl) >= roof_min_inliers:
                nrm = np.array([a, b, c], dtype=float)
                nrm_len = float(np.linalg.norm(nrm))
                if nrm_len > 1e-6:
                    nrm /= nrm_len
                    if abs(nrm[2]) >= roof_normal_z_min:
                        roof_pts = np.asarray(pcd.points)[inl]
                        x_min, x_max = float(np.percentile(roof_pts[:, 0], 2)), float(np.percentile(roof_pts[:, 0], 98))
                        y_min, y_max = float(np.percentile(roof_pts[:, 1], 2)), float(np.percentile(roof_pts[:, 1], 98))
                        if (x_max - x_min) * (y_max - y_min) >= min_roof_area_m2:
                            z_mean = float(np.mean(roof_pts[:, 2]))
                            corners = np.array([
                                [x_min, y_min, z_mean],
                                [x_max, y_min, z_mean],
                                [x_max, y_max, z_mean],
                                [x_min, y_max, z_mean],
                            ])
                            facades.append(Facade(
                                vertices=corners,
                                normal=np.array([0.0, 0.0, 1.0]),
                                component_tag="27.1",
                                label=f"roof_{roof_index}",
                                index=len(facades),
                            ))
                            roof_index += 1
        except Exception as e:
            print(f"[ransac facades] roof plane fit failed: {e}")

    print(
        f"[ransac facades] extracted {len(facades)} facades "
        f"({wall_index} walls + {roof_index} roofs) from {len(pts_all)} pts"
    )
    return facades


# ---------------------------------------------------------------------------
# Polygon → Facade synthesis (validated against reference point cloud)
# ---------------------------------------------------------------------------


@timed("facades_from_polygon")
def facades_from_polygon(
    polygon_enu: list[tuple[float, float, float]],
    points_enu: np.ndarray,
    *,
    slab_depth_m: float = 5.0,
    sample_step_m: float = 1.0,
    min_hit_fraction: float = 0.40,
    min_edge_length_m: float = 1.5,
    z_top_pct: float = 95.0,
    z_ground_pct: float = 5.0,
) -> tuple[list[Facade], dict]:
    """Build wall + flat-roof Facade objects from the DJI mission-area polygon.

    The polygon comes from the WPML template and encloses the area DJI already
    decided to scan — so its edges correspond to real building walls. Each
    polygon edge is validated against the reference point cloud:

    - Sample points along the edge at ``sample_step_m`` intervals.
    - For each sample, count cloud points in a slab extending inward by
      ``slab_depth_m`` (i.e. into the building), above ground level.
    - Edges with a hit rate below ``min_hit_fraction`` are dropped — that's
      typically a polygon side that cuts across a courtyard or open space.

    Returns (facades, diagnostics). The diagnostics include per-edge hit
    rates so the UI can surface "this edge of the polygon has no wall".
    """
    pts = np.asarray(points_enu, dtype=float).reshape(-1, 3)
    if len(pts) == 0 or len(polygon_enu) < 3:
        return [], {"edges": [], "ground_z": 0.0, "top_z": 0.0, "height": 0.0}

    poly = np.array(polygon_enu, dtype=float)
    if len(poly) > 1 and np.allclose(poly[0, :2], poly[-1, :2]):
        poly = poly[:-1]
    poly_xy = poly[:, :2]

    # Force CCW winding (signed area > 0 when viewed from +Z).
    signed_area = 0.5 * float(np.sum(
        poly_xy[:, 0] * np.roll(poly_xy[:, 1], -1)
        - np.roll(poly_xy[:, 0], -1) * poly_xy[:, 1]
    ))
    if signed_area < 0:
        poly_xy = poly_xy[::-1].copy()

    # Height from the full cloud (DJI-scanned cloud is already focused on the
    # building; no need for strict point-in-polygon filtering).
    ground_z = float(np.percentile(pts[:, 2], z_ground_pct))
    top_z = float(np.percentile(pts[:, 2], z_top_pct))
    height = max(3.0, top_z - ground_z)

    facades: list[Facade] = []
    edge_diags: list[dict] = []
    n = len(poly_xy)

    for i in range(n):
        p1 = poly_xy[i]
        p2 = poly_xy[(i + 1) % n]
        edge_vec = p2 - p1
        L = float(np.linalg.norm(edge_vec))
        if L < min_edge_length_m:
            edge_diags.append({"edge": i, "length": L, "hit_rate": 0.0, "dropped": True})
            continue

        direction = edge_vec / L
        inward = np.array([-direction[1], direction[0]])  # CCW → left of direction is inside
        outward = -inward

        n_samples = max(2, int(round(L / sample_step_m)))
        step = L / n_samples
        hits = 0
        inward_distances: list[float] = []
        # Exclude ground clutter but keep near-ground wall bases. 20% up from
        # the lowest point is generous enough to keep the first story of a wall.
        z_lo = ground_z + 0.2 * max(height, 1.0)
        above_ground = (pts[:, 2] > z_lo) & (pts[:, 2] < top_z + 1.0)
        for k in range(n_samples):
            sample_xy = p1 + (k + 0.5) * step * direction
            rel = pts[:, :2] - sample_xy
            d_in = rel @ inward
            d_tan = rel @ direction
            mask = (
                (d_in > -0.5) & (d_in < slab_depth_m)
                & (np.abs(d_tan) < 0.6 * step)
                & above_ground
            )
            if mask.any():
                hits += 1
                inward_distances.append(float(np.median(d_in[mask])))

        hit_rate = hits / n_samples
        edge_diags.append({
            "edge": i,
            "length": L,
            "hit_rate": hit_rate,
            "mean_inward_m": float(np.mean(inward_distances)) if inward_distances else None,
        })

        if hit_rate < min_hit_fraction:
            continue

        normal_3d = np.array([outward[0], outward[1], 0.0])
        normal_3d /= max(float(np.linalg.norm(normal_3d)), 1e-9)
        # Four corners, CCW viewed from outside: p1_bot, p2_bot, p2_top, p1_top.
        verts = np.array([
            [p1[0], p1[1], ground_z],
            [p2[0], p2[1], ground_z],
            [p2[0], p2[1], top_z],
            [p1[0], p1[1], top_z],
        ])
        facades.append(Facade(
            vertices=verts,
            normal=normal_3d,
            component_tag="21.1",
            label=f"wall_{len(facades):02d}",
            index=len(facades),
        ))

    # Flat roof covering the polygon interior at the measured top Z.
    roof_verts = np.column_stack([poly_xy, np.full(len(poly_xy), top_z)])
    facades.append(Facade(
        vertices=roof_verts,
        normal=np.array([0.0, 0.0, 1.0]),
        component_tag="27.1",
        label="roof",
        index=len(facades),
    ))

    diagnostics = {
        "edges": edge_diags,
        "ground_z": ground_z,
        "top_z": top_z,
        "height": float(height),
        "dropped": sum(1 for d in edge_diags if d.get("hit_rate", 0.0) < min_hit_fraction),
        "wall_count": len(facades) - 1,
    }
    return facades, diagnostics


# ---------------------------------------------------------------------------
# Point cloud handling (Open3D)
# ---------------------------------------------------------------------------


@timed("load_pointcloud")
def load_pointcloud(ply_bytes: bytes) -> "o3d.geometry.PointCloud":  # type: ignore[name-defined]
    """Load PLY bytes into an Open3D point cloud."""
    import open3d as o3d  # deferred: heavy import

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tmp:
        tmp.write(ply_bytes)
        tmp_path = tmp.name
    try:
        pcd = o3d.io.read_point_cloud(tmp_path)
    finally:
        Path(tmp_path).unlink(missing_ok=True)

    if len(pcd.points) == 0:
        raise ValueError("Point cloud PLY is empty")
    return pcd


@timed("pointcloud_to_viewer_arrays")
def pointcloud_to_viewer_arrays(
    pcd: "o3d.geometry.PointCloud",  # type: ignore[name-defined]
    max_points: int = 250_000,
) -> tuple[list[float], list[float]]:
    """Subsample + flatten a point cloud to (positions, colors) flat lists.

    Colors are in 0..1 range. Subsampling uses voxel-down-sampling first, then a
    random prune to the hard cap. ``max_points=250_000`` keeps the payload
    under ~6 MB over the wire (JSON float lists).
    """
    import open3d as o3d  # noqa: F401

    n = len(pcd.points)
    if n > max_points:
        # Target voxel size such that voxel_down_sample produces ~max_points.
        # Volumetric heuristic: scale by cube root of ratio.
        bbox = pcd.get_axis_aligned_bounding_box()
        extent = np.asarray(bbox.get_extent())
        vol = max(float(np.prod(extent)), 1e-6)
        approx_voxel = (vol / max_points) ** (1.0 / 3.0)
        if approx_voxel > 0.001:
            pcd = pcd.voxel_down_sample(voxel_size=approx_voxel)

    pts = np.asarray(pcd.points, dtype=np.float32)
    if len(pts) > max_points:
        idx = np.random.default_rng(42).choice(len(pts), size=max_points, replace=False)
        pts = pts[idx]
        colors = (
            np.asarray(pcd.colors, dtype=np.float32)[idx]
            if pcd.has_colors() else None
        )
    else:
        colors = (
            np.asarray(pcd.colors, dtype=np.float32)
            if pcd.has_colors() else None
        )

    if colors is None or len(colors) != len(pts):
        # Fallback gray
        colors = np.full((len(pts), 3), 0.6, dtype=np.float32)

    return (
        pts.flatten().round(3).tolist(),
        colors.flatten().round(3).tolist(),
    )


@timed("pointcloud_to_mesh_cgal_alpha_wrap")
def pointcloud_to_mesh_cgal_alpha_wrap(
    points: np.ndarray,
    *,
    alpha: float = 0.3,
    offset: float = 0.08,
) -> bytes:
    """Reconstruct a watertight mesh via CGAL ``alpha_wrap_3``.

    Unlike Open3D's ``create_from_point_cloud_alpha_shape`` — which spews
    "invalid tetra in TetraMesh" warnings on noisy DJI clouds and produces
    a non-manifold jagged surface — ``alpha_wrap_3`` is the modern CGAL
    recommendation: it shrink-wraps the point set producing a watertight,
    orientable, 2-manifold surface that's ready to use without smoothing
    or manual repair.

    Parameters
    ----------
    alpha : distance between consecutive wrap probes. Smaller = more
        detail but also more shrink-wrap hugging of noise.
    offset : how far the wrap stays away from the input points (tolerance
        to noise). Typical rule of thumb: offset ≈ point spacing × 1.5.
    """
    from CGAL.CGAL_Alpha_wrap_3 import alpha_wrap_3, Point_3_Vector
    from CGAL.CGAL_Kernel import Point_3
    from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
    import tempfile
    import trimesh

    pts = Point_3_Vector()
    for p in points:
        pts.append(Point_3(float(p[0]), float(p[1]), float(p[2])))

    poly = Polyhedron_3()
    alpha_wrap_3(pts, float(alpha), float(offset), poly)

    with tempfile.NamedTemporaryFile(suffix=".off", delete=False) as tf:
        off_path = tf.name
    try:
        poly.write_to_file(off_path)
        mesh = trimesh.load(off_path, force="mesh")
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            raise RuntimeError(
                f"alpha_wrap_3 produced empty mesh (alpha={alpha}, offset={offset})"
            )
        ply_bytes = mesh.export(file_type="ply")
    finally:
        try:
            os.unlink(off_path)
        except OSError:
            pass

    print(
        f"[kmz_import] alpha_wrap_3: {len(points)} pts → "
        f"{len(mesh.vertices)}V / {len(mesh.faces)}F "
        f"(α={alpha}, offset={offset}, watertight={mesh.is_watertight})"
    )
    return ply_bytes


@timed("pointcloud_to_mesh_ply")
def pointcloud_to_mesh_ply(
    pcd: "o3d.geometry.PointCloud",  # type: ignore[name-defined]
    alpha: float = 0.5,
    target_points: int = 120_000,
    min_voxel_size: float = 0.03,
    max_voxel_size: float = 0.20,
    voxel_size_override: float | None = None,
    smooth_iterations: int = 12,
    decimation_ratio: float = 0.35,
    use_cgal_alpha_wrap: bool = True,
    aw_alpha_override: float | None = None,
    aw_offset_override: float | None = None,
) -> bytes:
    """Reconstruct a triangle mesh from a point cloud; return PLY bytes.

    With ``use_cgal_alpha_wrap=True`` (default), dispatches to
    :func:`pointcloud_to_mesh_cgal_alpha_wrap` — a watertight, manifold
    shrink-wrap that doesn't need smoothing or decimation. Falls back
    silently to the legacy Open3D alpha-shape path if CGAL isn't
    available.

    Legacy Open3D path (``use_cgal_alpha_wrap=False``):
    Uses **alpha-shape** reconstruction (not Poisson). Open3D's Poisson
    implementation hard-aborts the process on noisy DJI Smart3D clouds
    ("Failed to close loop" → SIGABRT in the C++ layer), so we can't catch
    and recover from it. Alpha shape is slower but robust and produces a
    mesh that's good enough for facade extraction.

    Voxel size is chosen **adaptively** to target ~``target_points`` after
    downsampling — alpha-shape tetrahedralization is O(n²) so input size
    dominates runtime. ~40K points keeps Mijande-scale buildings under ~15s.
    """
    import open3d as o3d  # deferred

    n_in = len(pcd.points)
    if voxel_size_override is not None and voxel_size_override > 0:
        voxel_size = float(voxel_size_override)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"[kmz_import] {n_in} → {len(pcd.points)} points (voxel={voxel_size:.3f}m, manual)")
    elif n_in > target_points:
        bbox = pcd.get_axis_aligned_bounding_box()
        extent = np.asarray(bbox.get_extent())
        vol = max(float(np.prod(extent)), 1e-6)
        voxel_size = (vol / target_points) ** (1.0 / 3.0)
        voxel_size = max(min_voxel_size, min(max_voxel_size, voxel_size))
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"[kmz_import] {n_in} → {len(pcd.points)} points (voxel={voxel_size:.3f}m, auto)")
    else:
        voxel_size = float(min_voxel_size)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Prefer CGAL alpha_wrap_3: watertight, manifold, no warning spam, no
    # need for Taubin + decimation afterward. Alpha/offset are scaled from
    # the chosen voxel size so denser clouds get proportionally finer detail.
    if use_cgal_alpha_wrap:
        try:
            pts = np.asarray(pcd.points, dtype=np.float64)
            if len(pts) < 50:
                raise RuntimeError("not enough points after downsample")
            # alpha ≈ 2× voxel spacing (finer probe keeps small features
            # like sills, dormers, parapets); offset ≈ 1× voxel spacing
            # (tight wrap so the mesh hugs geometry, not noise haze).
            aw_alpha = (
                float(aw_alpha_override) if aw_alpha_override and aw_alpha_override > 0
                else max(0.06, min(0.60, voxel_size * 2.0))
            )
            aw_offset = (
                float(aw_offset_override) if aw_offset_override and aw_offset_override > 0
                else max(0.03, min(0.20, voxel_size * 1.0))
            )
            return pointcloud_to_mesh_cgal_alpha_wrap(
                pts, alpha=aw_alpha, offset=aw_offset,
            )
        except Exception as e:
            print(f"[kmz_import] CGAL alpha_wrap_3 unavailable ({e}); falling back to Open3D alpha shape")

    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    mesh = None
    # Open3D's alpha-shape routine spams "invalid tetra in TetraMesh" warnings
    # on every borderline tetrahedron in noisy DJI Smart3D clouds — they're
    # informational (the offending tetra is discarded), not errors. Silence
    # them at the C++ verbosity level so the log stays readable.
    try:
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        if len(mesh.triangles) == 0:
            mesh = None
    except Exception as e:
        print(f"[kmz_import] Alpha shape failed ({e}); falling back to ball pivoting")
        mesh = None

    if mesh is None or len(mesh.triangles) == 0:
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = float(np.mean(distances)) if len(distances) else 0.1
        radii = o3d.utility.DoubleVector([avg_dist * 1.5, avg_dist * 3, avg_dist * 6])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)

    if len(mesh.triangles) == 0:
        raise ValueError("All mesh reconstruction strategies produced an empty mesh")

    # Clean up the raw alpha-shape output before smoothing — Taubin on a mesh
    # with duplicate or degenerate triangles amplifies noise.
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()

    # Taubin (λ=0.5, μ=-0.53) smoothing flattens the jagged alpha-shape walls
    # into near-planar surfaces without the global shrinkage that plain
    # Laplacian smoothing causes. 10-15 passes is the sweet spot for
    # DJI Smart3D clouds: walls become flat enough that region growing
    # merges them into one facade each.
    if smooth_iterations > 0:
        try:
            mesh = mesh.filter_smooth_taubin(number_of_iterations=int(smooth_iterations))
        except Exception as e:
            print(f"[kmz_import] Taubin smoothing skipped: {e}")

    # Quadric decimation collapses coplanar faces after smoothing — this is
    # what actually lets region growing merge a wall into a single region
    # (many input faces with the same normal → fewer output faces with the
    # exact same normal).
    if 0.0 < decimation_ratio < 1.0 and len(mesh.triangles) > 2000:
        target = max(2000, int(len(mesh.triangles) * decimation_ratio))
        try:
            mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target)
        except Exception as e:
            print(f"[kmz_import] Decimation skipped: {e}")

    # Fill small holes — alpha-shape reconstructions leave window-sized gaps
    # that fragment walls into several region-growing components. Use trimesh
    # (pure Python) rather than Open3D's tensor fill_holes, which can SIGABRT
    # on malformed meshes from noisy Smart3D clouds.
    try:
        import trimesh as _tm
        verts = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.triangles)
        if len(verts) > 0 and len(faces) > 0:
            tm_mesh = _tm.Trimesh(vertices=verts, faces=faces, process=False)
            tm_mesh.fill_holes()
            if len(tm_mesh.faces) > len(faces):
                mesh.vertices = o3d.utility.Vector3dVector(np.asarray(tm_mesh.vertices))
                mesh.triangles = o3d.utility.Vector3iVector(np.asarray(tm_mesh.faces))
    except Exception as e:
        print(f"[kmz_import] Hole filling skipped: {e}")

    mesh.compute_vertex_normals()

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        o3d.io.write_triangle_mesh(tmp_path, mesh, write_ascii=False)
        return Path(tmp_path).read_bytes()
    finally:
        Path(tmp_path).unlink(missing_ok=True)
