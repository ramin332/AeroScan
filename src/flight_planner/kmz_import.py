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
import tempfile
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree as ET

import numpy as np

from ._profiling import timed
from .models import meters_per_deg

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


# ---------------------------------------------------------------------------
# KMZ archive parsing
# ---------------------------------------------------------------------------


def _find_members(zf: zipfile.ZipFile) -> dict[str, str]:
    """Locate the key files inside a DJI KMZ.

    Returns a dict with keys: template, waylines, pointcloud, geo_desc (any may be missing).
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
    return members


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


@timed("pointcloud_to_mesh_ply")
def pointcloud_to_mesh_ply(
    pcd: "o3d.geometry.PointCloud",  # type: ignore[name-defined]
    alpha: float = 0.5,
    target_points: int = 40_000,
    min_voxel_size: float = 0.05,
    max_voxel_size: float = 0.40,
    voxel_size_override: float | None = None,
    smooth_iterations: int = 12,
    decimation_ratio: float = 0.35,
) -> bytes:
    """Reconstruct a triangle mesh from a point cloud; return PLY bytes.

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
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    mesh = None
    try:
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

    mesh.compute_vertex_normals()

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        o3d.io.write_triangle_mesh(tmp_path, mesh, write_ascii=False)
        return Path(tmp_path).read_bytes()
    finally:
        Path(tmp_path).unlink(missing_ok=True)
