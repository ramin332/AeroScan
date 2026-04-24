"""REST API endpoints for the AeroScan debug server."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Literal, Optional

from fastapi import APIRouter, Form, HTTPException, UploadFile
from fastapi.responses import Response
from pydantic import BaseModel, Field
from sqlalchemy import func

from .._profiling import get_recorded as _get_phase_timings, start_recording as _start_phase_recording
from ..building_import import build_building_from_geojson, build_building_from_mesh, decode_mesh_viewer_data, extract_facades


def _kmz_extract_best_facades(
    clipped_mesh,
    polygon_enu,
    points_xyz,
    *,
    min_area_m2: float = 5.0,
    fd_overrides: dict | None = None,
):
    """Extract facades from a clipped KMZ mesh / point cloud.

    Priority chain, best-first for DJI Smart3D data:
    1. CGAL Efficient-RANSAC (Schnabel 2007) — gold-standard multi-plane
       detection with connectivity-aware clustering. Native C++ via SWIG.
    2. Vertical-prior 2D-line RANSAC on the point cloud (our fallback).
    3. MeshLab plane segmentation on the clipped mesh.
    4. Mesh region growing.
    5. Polygon-derived walls (last resort, boxy).

    ``fd_overrides`` forwards CGAL Shape-Detection knobs (epsilon,
    cluster_epsilon, min_points, density/area gates, normal_threshold)
    from the UI through to ``facades_from_pointcloud_cgal``. ``None``
    values are dropped so library defaults apply.
    """
    fd_kwargs = {k: v for k, v in (fd_overrides or {}).items() if v is not None}
    try:
        from ..kmz_import import facades_from_pointcloud_cgal as _ffc
        cgal_out = _ffc(points_xyz, polygon_enu, **fd_kwargs)
        if len(cgal_out) >= 3:
            print(f"[kmz facades] cgal region_growing → {len(cgal_out)} facades")
            return cgal_out
    except Exception as _exc:
        print(f"[kmz facades] cgal efficient_RANSAC raised: {_exc}")

    try:
        from ..kmz_import import facades_from_pointcloud_ransac as _ffr
        ransac_out = _ffr(points_xyz, polygon_enu)
        if len(ransac_out) >= 3:
            print(f"[kmz facades] pointcloud_ransac → {len(ransac_out)} facades")
            return ransac_out
    except Exception as _exc:
        print(f"[kmz facades] pointcloud_ransac raised: {_exc}")

    for method in ("meshlab", "region_growing"):
        try:
            out = extract_facades(clipped_mesh, method=method, min_area_m2=min_area_m2)
        except Exception as _exc:  # pragma: no cover — optional dep missing
            print(f"[kmz facades] {method} extraction raised: {_exc}")
            continue
        if len(out) >= 3:
            print(f"[kmz facades] {method} → {len(out)} facades (min_area={min_area_m2})")
            return out
    # Last-resort polygon fallback (boxy walls, but guarantees a mission).
    from ..kmz_import import facades_from_polygon as _ffp
    facades, _ = _ffp(polygon_enu, points_xyz)
    print(f"[kmz facades] polygon fallback → {len(facades)} facades")
    return facades


def _expand_polygon_xy(polygon_enu, margin_m: float):
    """Return the polygon expanded outward by ``margin_m`` (axis-aligned bbox)."""
    import numpy as _np
    if not polygon_enu or len(polygon_enu) < 3 or margin_m <= 0:
        return polygon_enu
    arr = _np.asarray(polygon_enu, dtype=float).reshape(-1, 3)
    centroid = arr[:, :2].mean(axis=0)
    out = []
    for x, y, z in arr:
        dx = x - centroid[0]
        dy = y - centroid[1]
        r = (dx * dx + dy * dy) ** 0.5
        if r < 1e-6:
            out.append((float(x), float(y), float(z)))
            continue
        scale = (r + margin_m) / r
        out.append((float(centroid[0] + dx * scale), float(centroid[1] + dy * scale), float(z)))
    return out


def _filter_facades_by_dji_bbox(
    facades,
    mission_area_wgs84: list | None,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
    *,
    margin_m: float = 2.0,
):
    """Restrict facades to the DJI RC Plus mission-area polygon.

    The ``mission_area_wgs84`` polygon (from the KMZ ``template.kml``) is the
    authoritative bound: it's what the RC Plus shows on-controller as the
    mapped region. A small ``margin_m`` is added so facades lying just on
    the polygon edge (common when the building footprint hugs the mission
    boundary) still survive. Returns the input list unchanged if no polygon
    is available (degraded KMZ).
    """
    import numpy as _np
    from ..kmz_import import polygon_to_enu as _poly_to_enu
    if not mission_area_wgs84:
        print("[kmz facades] no mission_area_wgs84 — skipping filter")
        return facades
    poly_enu = _poly_to_enu(mission_area_wgs84, ref_lat, ref_lon, ref_alt)
    poly_enu = _expand_polygon_xy(poly_enu, margin_m)
    if not poly_enu or len(poly_enu) < 3:
        return facades
    poly = _np.asarray(poly_enu, dtype=float).reshape(-1, 3)[:, :2]

    def _inside(x: float, y: float) -> bool:
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
            j = i
        return inside

    kept = [f for f in facades if _inside(*_np.asarray(f.vertices).mean(axis=0)[:2])]
    print(f"[kmz facades] RC Plus polygon filter (margin={margin_m}m): {len(facades)} → {len(kept)}")
    return kept


def _tighten_clip_polygon(mission_polygon_enu, points_xyz):
    """Return the tightest polygon to clip the reconstructed mesh against.

    Uses the convex hull of mid-height cloud points when available (tight to
    the actual walls), else falls back to the loose DJI mission-area polygon.
    """
    from ..kmz_import import tight_footprint_from_cloud_xy as _tight
    tight = _tight(points_xyz)
    if tight is not None and len(tight) >= 3:
        print(f"[kmz facades] tight cloud-hull footprint: {len(tight)} verts (mission polygon had {len(mission_polygon_enu)})")
        return tight
    return mission_polygon_enu
from ..reconstruct import delete_task as delete_sim_task, get_task as get_sim_task, get_task_result as get_sim_result, list_tasks as list_sim_tasks, start_simulation_async
from ..building_presets import (
    l_shaped_block,
    large_apartment_block,
    pitched_roof_house,
    simple_box,
)
from ..camera import compute_distance_for_gsd, compute_footprint, get_camera
from ..geometry import build_rectangular_building, generate_mission_waypoints
from ..kmz_builder import build_kmz_bytes
from ..models import (
    AlgorithmConfig,
    CAMERAS,
    CameraName,
    ExclusionZone,
    GIMBAL_PAN_MAX_DEG,
    GIMBAL_PAN_MIN_DEG,
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    INSPECTION_SPEED_MS,
    MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN,
    MAX_SPEED_MS,
    MAX_WAYPOINTS_PER_MISSION,
    MIN_ALTITUDE_M,
    MissionConfig,
    RoofType,
)
from ..validate import validate_mission
from ..visualize import prepare_leaflet_data, prepare_threejs_data, compute_facade_coverage
from .database import BuildingRecord, FacadeCacheRecord, get_db
from .state import session

router = APIRouter()

# --- Upload task tracking (in-memory, same pattern as simulation) ---

_upload_tasks: dict[str, dict] = {}
_upload_lock = threading.Lock()

# --- Facade-extraction cache ---
# Keyed by (building_id, fingerprint of extraction-affecting params). Stores
# the decoded trimesh.Trimesh + extracted facades list. Avoids re-running
# region-growing facade extraction every /generate when the user is only
# tweaking mission parameters (GSD, camera, speed, etc.).
_facade_cache: dict[str, tuple[object, list, tuple[float, float, float, float]]] = {}
_facade_cache_lock = threading.Lock()
_FACADE_CACHE_MAX = 8  # bounded LRU — each entry ~ mesh size

# --- Decoded-mesh cache ---
# Keyed by building_id. Caches the trimesh.Trimesh after vertex/index decode
# and (lazily) its ray tree. Decoding a KMZ mesh + building trimesh's ray-tree
# costs hundreds of ms per request; with a per-building lock we can reuse the
# same mesh object across sequential /generate calls safely. The lock ensures
# only one /generate uses a given mesh at a time — trimesh's ray internals
# aren't safe for concurrent access, but sequential reuse works fine.
_mesh_cache: dict[str, object] = {}
_mesh_cache_locks: dict[str, threading.Lock] = {}
_mesh_cache_meta_lock = threading.Lock()
_MESH_CACHE_MAX = 3  # bounded — each entry can be tens of MB


def _mesh_cache_lock_for(building_id: str) -> threading.Lock:
    """Get-or-create the lock for a given building_id (serializes same-building /generate)."""
    with _mesh_cache_meta_lock:
        lk = _mesh_cache_locks.get(building_id)
        if lk is None:
            lk = threading.Lock()
            _mesh_cache_locks[building_id] = lk
        return lk


def _mesh_cache_get(building_id: str):
    with _mesh_cache_meta_lock:
        return _mesh_cache.get(building_id)


def _mesh_cache_put(building_id: str, mesh_obj) -> None:
    with _mesh_cache_meta_lock:
        if building_id in _mesh_cache:
            del _mesh_cache[building_id]
        _mesh_cache[building_id] = mesh_obj
        while len(_mesh_cache) > _MESH_CACHE_MAX:
            evict_id = next(iter(_mesh_cache))
            _mesh_cache.pop(evict_id)
            _mesh_cache_locks.pop(evict_id, None)


def _mesh_cache_invalidate(building_id: str) -> None:
    with _mesh_cache_meta_lock:
        _mesh_cache.pop(building_id, None)
        _mesh_cache_locks.pop(building_id, None)


def _facade_cache_key(building_id: str, extraction_method: str, min_facade_area: float, algo) -> str:
    import hashlib as _h
    fp = (
        extraction_method,
        round(min_facade_area, 3),
        round(algo.region_growing_angle_deg, 2),
        round(algo.downward_face_threshold, 2),
        round(algo.ground_level_threshold_m, 3),
        round(algo.occlusion_ray_offset_m, 4),
        round(algo.occlusion_hit_fraction, 3),
        round(algo.flat_roof_normal_threshold, 3),
        round(algo.wall_normal_threshold, 3),
        round(algo.min_mesh_faces, 0),
    )
    return f"{building_id}:{_h.md5(repr(fp).encode()).hexdigest()}"


def _facades_to_json(facades, bbox) -> str:
    """Serialize extracted facades + bbox to a JSON-safe payload."""
    import numpy as _np
    return json.dumps({
        "bbox": [float(v) for v in bbox],
        "facades": [
            {
                "vertices": _np.asarray(f.vertices, dtype=_np.float64).tolist(),
                "normal": _np.asarray(f.normal, dtype=_np.float64).tolist(),
                "component_tag": f.component_tag,
                "label": f.label,
                "index": int(f.index),
            }
            for f in facades
        ],
    })


def _facades_from_json(payload: str):
    """Inverse of _facades_to_json. Returns (facades, bbox)."""
    import numpy as _np
    from ..models import Facade as _Facade
    data = json.loads(payload)
    facades = [
        _Facade(
            vertices=_np.asarray(fd["vertices"], dtype=_np.float64),
            normal=_np.asarray(fd["normal"], dtype=_np.float64),
            component_tag=fd.get("component_tag", ""),
            label=fd.get("label", ""),
            index=int(fd.get("index", 0)),
        )
        for fd in data["facades"]
    ]
    bbox = tuple(data["bbox"])
    return facades, bbox


def _facade_cache_get(key: str):
    with _facade_cache_lock:
        hit = _facade_cache.get(key)
    if hit is not None:
        return hit
    # Cold: check persistent store.
    db = get_db()
    try:
        row = db.query(FacadeCacheRecord).filter_by(cache_key=key).first()
        if row is None:
            return None
        try:
            value = _facades_from_json(row.payload_json)
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            # Corrupt row — drop it so we re-extract.
            db.delete(row)
            db.commit()
            return None
    finally:
        db.close()
    # Promote to in-memory cache.
    with _facade_cache_lock:
        _facade_cache[key] = value
        while len(_facade_cache) > _FACADE_CACHE_MAX:
            _facade_cache.pop(next(iter(_facade_cache)))
    return value


def _facade_cache_put(key: str, value) -> None:
    with _facade_cache_lock:
        if key in _facade_cache:
            del _facade_cache[key]
        _facade_cache[key] = value
        while len(_facade_cache) > _FACADE_CACHE_MAX:
            _facade_cache.pop(next(iter(_facade_cache)))
    # Persist for next server start. Keyed by the full cache_key, so different
    # extraction params for the same building each get their own row.
    facades, bbox = value
    building_id = key.split(":", 1)[0]
    try:
        payload = _facades_to_json(facades, bbox)
    except (TypeError, ValueError):
        return  # JSON-encoding failure shouldn't break the request path
    db = get_db()
    try:
        existing = db.query(FacadeCacheRecord).filter_by(cache_key=key).first()
        if existing is not None:
            existing.payload_json = payload
        else:
            db.add(FacadeCacheRecord(
                cache_key=key, building_id=building_id, payload_json=payload,
            ))
        db.commit()
    finally:
        db.close()


SNAPSHOT_MODES = ("dji", "inspection")


# ---- Smart recomputation cache for the refine-KMZ slider loop ----
#
# Refine has two expensive phases:
#   1. Mesh reconstruction (alpha_wrap_3): depends on voxel + aw_alpha + aw_offset
#   2. CGAL Shape-Detection: depends on mesh + fd_* params
#
# The user often tweaks one phase's params without touching the other. Keying
# each phase separately lets a slider change in phase 2 hit the cache for
# phase 1 (and vice versa), so only the changed stage re-runs.
_kmz_mesh_cache: dict[tuple, object] = {}
_kmz_mesh_cache_lock = threading.Lock()
_KMZ_MESH_CACHE_MAX = 6   # per-building × a few slider settings ≈ plenty

_kmz_facade_runtime_cache: dict[tuple, object] = {}
_kmz_facade_runtime_cache_lock = threading.Lock()
_KMZ_FACADE_RUNTIME_CACHE_MAX = 16


def _kmz_mesh_key(building_id: str, voxel_size: float, aw_alpha, aw_offset) -> tuple:
    return (
        building_id,
        round(float(voxel_size), 4),
        None if aw_alpha is None else round(float(aw_alpha), 4),
        None if aw_offset is None else round(float(aw_offset), 4),
    )


def _kmz_facade_runtime_key(mesh_key: tuple, fd: dict) -> tuple:
    def _r(v, nd=4):
        return None if v is None else round(float(v), nd)
    return (
        mesh_key,
        _r(fd.get("epsilon")),
        _r(fd.get("cluster_epsilon")),
        None if fd.get("min_points") is None else int(fd["min_points"]),
        _r(fd.get("min_wall_area_m2")),
        _r(fd.get("min_roof_area_m2")),
        _r(fd.get("min_density_per_m2")),
        _r(fd.get("normal_threshold")),
    )


def _kmz_mesh_cache_get(key):
    with _kmz_mesh_cache_lock:
        return _kmz_mesh_cache.get(key)


def _kmz_mesh_cache_put(key, value):
    with _kmz_mesh_cache_lock:
        if key in _kmz_mesh_cache:
            del _kmz_mesh_cache[key]
        _kmz_mesh_cache[key] = value
        while len(_kmz_mesh_cache) > _KMZ_MESH_CACHE_MAX:
            _kmz_mesh_cache.pop(next(iter(_kmz_mesh_cache)))


def _kmz_mesh_cache_invalidate(building_id: str):
    with _kmz_mesh_cache_lock:
        for k in [k for k in _kmz_mesh_cache if k[0] == building_id]:
            del _kmz_mesh_cache[k]


def _kmz_facade_runtime_cache_get(key):
    with _kmz_facade_runtime_cache_lock:
        return _kmz_facade_runtime_cache.get(key)


def _kmz_facade_runtime_cache_put(key, value):
    with _kmz_facade_runtime_cache_lock:
        if key in _kmz_facade_runtime_cache:
            del _kmz_facade_runtime_cache[key]
        _kmz_facade_runtime_cache[key] = value
        while len(_kmz_facade_runtime_cache) > _KMZ_FACADE_RUNTIME_CACHE_MAX:
            _kmz_facade_runtime_cache.pop(next(iter(_kmz_facade_runtime_cache)))


def _kmz_facade_runtime_cache_invalidate(building_id: str):
    with _kmz_facade_runtime_cache_lock:
        for k in [k for k in _kmz_facade_runtime_cache if k[0][0] == building_id]:
            del _kmz_facade_runtime_cache[k]


def _snapshot_encode_response(response: dict) -> str | None:
    """Gzip+b64 encode a GenerateResponse-shaped dict for DB storage."""
    import base64
    import gzip
    try:
        return base64.b64encode(
            gzip.compress(json.dumps(response).encode("utf-8"))
        ).decode("ascii")
    except (TypeError, ValueError):
        return None


def _snapshot_decode_response(gz_b64: str) -> dict | None:
    import base64
    import gzip
    try:
        return json.loads(gzip.decompress(base64.b64decode(gz_b64)))
    except (ValueError, OSError, json.JSONDecodeError):
        return None


def _snapshots_get(props: dict) -> dict:
    """Return (and migrate) the snapshots dict from a BuildingRecord's props.

    Old format stored ``last_response_gz_b64`` / ``last_settings`` flat. When
    we see that, fold it into ``snapshots['dji']`` so existing records light
    up the new mode system without a migration script.
    """
    snapshots = dict(props.get("snapshots") or {})
    if not snapshots and props.get("last_response_gz_b64"):
        snapshots["dji"] = {
            "response_gz_b64": props["last_response_gz_b64"],
            "settings": props.get("last_settings") or {},
            "version_id": props.get("last_version_id"),
        }
    return snapshots


def _snapshot_save(
    building_id: str,
    mode: str,
    response: dict,
    settings: dict,
    version_id: str,
    *,
    extra_props: dict | None = None,
) -> None:
    """Write ``snapshots[mode]`` into the BuildingRecord. Atomic, one DB trip."""
    if mode not in SNAPSHOT_MODES:
        raise ValueError(f"Unknown snapshot mode: {mode!r}")
    gz_b64 = _snapshot_encode_response(response)
    if gz_b64 is None:
        return
    db = get_db()
    try:
        rec = db.query(BuildingRecord).filter_by(id=building_id).first()
        if rec is None:
            return
        props = json.loads(rec.properties_json or "{}")
        snapshots = _snapshots_get(props)
        snapshots[mode] = {
            "response_gz_b64": gz_b64,
            "settings": settings,
            "version_id": version_id,
        }
        props["snapshots"] = snapshots
        props["active_mode"] = mode
        # Keep the flat last_* keys in sync with the dji snapshot for back-
        # compat with any code still reading them. Drop them for other modes.
        if mode == "dji":
            props["last_response_gz_b64"] = gz_b64
            props["last_settings"] = settings
            props["last_version_id"] = version_id
        if extra_props:
            props.update(extra_props)
        rec.properties_json = json.dumps(props)
        db.commit()
    finally:
        db.close()


def _facade_cache_invalidate(building_id: str) -> None:
    """Drop all cached entries for a building (called when it's deleted or refined)."""
    with _facade_cache_lock:
        for k in [k for k in _facade_cache if k.startswith(f"{building_id}:")]:
            del _facade_cache[k]
    db = get_db()
    try:
        db.query(FacadeCacheRecord).filter_by(building_id=building_id).delete()
        db.commit()
    finally:
        db.close()


def _set_upload_progress(task_id: str, progress: float, message: str, **extra: object) -> None:
    with _upload_lock:
        if task_id in _upload_tasks:
            _upload_tasks[task_id].update(progress=progress, message=message, **extra)


# --- Pydantic models ---


class BuildingParams(BaseModel):
    lat: float = 53.2012
    lon: float = 5.7999
    width: float = Field(20.0, ge=1.0, le=200.0)
    depth: float = Field(10.0, ge=1.0, le=200.0)
    height: float = Field(8.0, ge=1.0, le=100.0)
    heading_deg: float = Field(0.0, ge=0.0, le=360.0)
    roof_type: Literal["flat", "pitched"] = "flat"
    roof_pitch_deg: float = Field(0.0, ge=0.0, le=60.0)
    num_stories: int = Field(1, ge=1, le=20)


class MissionParams(BaseModel):
    target_gsd_mm_per_px: float = Field(2.0, ge=0.5, le=10.0)
    camera: Literal["wide", "medium_tele", "telephoto"] = "wide"
    front_overlap: float = Field(0.60, ge=0.0, le=0.95)
    side_overlap: float = Field(0.50, ge=0.0, le=0.95)
    flight_speed_ms: float = Field(2.0, ge=0.5, le=5.0)
    obstacle_clearance_m: float = Field(2.0, ge=1.0, le=20.0)
    mission_name: str = "AeroScan Inspection"
    # Advanced tunable constraints
    gimbal_pitch_margin_deg: float = Field(5.0, ge=0.0, le=15.0)
    min_photo_distance_m: float = Field(1.5, ge=0.5, le=5.0)
    yaw_rate_deg_per_s: float = Field(60.0, ge=30.0, le=120.0)
    # Flight mode
    stop_at_waypoint: bool = False  # False = fly-through (faster, M4E mech shutter)
    # XML-slimming knobs: skip gimbalRotate / rotateYaw actions whose pose is
    # within threshold of the previously emitted one. Cuts XML ~50% on dense
    # facade sweeps where pose is constant within a row.
    gimbal_dedup_threshold_deg: float = Field(2.0, ge=0.0, le=45.0)
    heading_dedup_threshold_deg: float = Field(5.0, ge=0.0, le=45.0)
    # DJI Pilot 2 safety defaults (operator can adjust before flying)
    rc_lost_action: str = "go_home"
    finish_action: str = "return_home"
    takeoff_security_height_m: float = Field(5.0, ge=1.2, le=1500.0)


class AlgorithmParams(BaseModel):
    """Tunable algorithm parameters (not backed by hardware specs)."""
    # Flight time estimation
    hover_time_per_wp_s: float = Field(1.0, ge=0.0, le=10.0)
    takeoff_landing_overhead_s: float = Field(60.0, ge=0.0, le=300.0)
    battery_warning_threshold: float = Field(0.80, ge=0.5, le=0.99)
    battery_info_threshold: float = Field(0.65, ge=0.3, le=0.95)
    gimbal_near_limit_deg: float = Field(-80.0, ge=-90.0, le=0.0)
    # Geometry / grid generation
    facade_edge_inset_m: float = Field(0.1, ge=0.0, le=1.0)
    transition_altitude_margin_m: float = Field(2.0, ge=0.0, le=10.0)
    roof_normal_threshold: float = Field(0.5, ge=0.1, le=0.9)
    min_altitude_m: float = Field(2.0, ge=0.5, le=10.0)
    # Mesh import
    default_building_height_m: float = Field(8.0, ge=1.0, le=100.0)
    min_mesh_faces: int = Field(4, ge=1, le=100)
    downward_face_threshold: float = Field(-0.3, ge=-1.0, le=0.0)
    ground_level_threshold_m: float = Field(0.3, ge=0.0, le=5.0)
    occlusion_ray_offset_m: float = Field(0.05, ge=0.001, le=1.0)
    occlusion_hit_fraction: float = Field(0.5, ge=0.1, le=1.0)
    flat_roof_normal_threshold: float = Field(0.95, ge=0.5, le=1.0)
    wall_normal_threshold: float = Field(0.3, ge=0.01, le=0.9)
    auto_scale_height_threshold_m: float = Field(50.0, ge=10.0, le=500.0)
    auto_scale_target_height_m: float = Field(8.0, ge=1.0, le=100.0)
    region_growing_angle_deg: float = Field(15.0, ge=1.0, le=45.0)
    # Surface sampling
    surface_sample_count: int = Field(2000, ge=100, le=50000)
    surface_dedup_radius_m: float = Field(0.5, ge=0.1, le=5.0)
    surface_dedup_max_angle_deg: float = Field(30.0, ge=5.0, le=90.0)
    # Waypoint LOS occlusion
    enable_waypoint_los: bool = True
    los_tolerance_m: float = Field(0.5, ge=0.1, le=2.0)
    los_min_visible_ratio: float = Field(0.4, ge=0.1, le=1.0)
    # Grid density
    grid_density: float = Field(1.0, ge=0.25, le=4.0)
    # Path optimization
    enable_path_dedup: bool = True
    enable_path_tsp: bool = True
    enable_sweep_reversal: bool = True
    dedup_max_gimbal_diff_deg: float = Field(20.0, ge=5.0, le=90.0)
    tsp_method: Literal["auto", "nearest_neighbor", "greedy", "simulated_annealing", "threshold_accepting"] = "auto"
    # Path collision checking
    enable_path_collision_check: bool = True
    path_collision_margin_m: float = Field(0.5, ge=0.1, le=3.0)
    # KMZ export
    min_waypoint_height_m: float = Field(2.0, ge=0.5, le=10.0)

    def to_algorithm_config(self) -> AlgorithmConfig:
        return AlgorithmConfig(**self.model_dump())


class ExclusionZoneParam(BaseModel):
    """A zone in ENU coordinates — box or arbitrary polygon."""
    id: str = ""
    label: str = ""
    center_x: float = 0.0
    center_y: float = 0.0
    center_z: float = 0.0
    size_x: float = Field(5.0, ge=0.1, le=500.0)
    size_y: float = Field(5.0, ge=0.1, le=500.0)
    size_z: float = Field(10.0, ge=0.1, le=500.0)
    zone_type: Literal["no_fly", "no_inspect", "inclusion"] = "no_fly"
    polygon_vertices: Optional[list[tuple[float, float]]] = None


class GenerateRequest(BaseModel):
    preset: Optional[Literal[
        "simple_box", "pitched_roof_house", "l_shaped_block", "large_apartment_block"
    ]] = None
    building_id: Optional[str] = None
    building: BuildingParams = BuildingParams()
    mission: MissionParams = MissionParams()
    algorithm: AlgorithmParams = AlgorithmParams()
    min_facade_area: float = Field(1.0, ge=0.1, le=50.0)
    extraction_method: str = "region_growing"
    waypoint_strategy: str = "facade_grid"  # "facade_grid" or "surface_sampling"
    disabled_facades: list[int] = []
    enabled_candidates: list[int] = []  # indices of rejected candidates to force-include
    exclusion_zones: list[ExclusionZoneParam] = []


class BuildingUploadRequest(BaseModel):
    name: str = "Uploaded building"
    geojson: dict
    height: float = Field(8.0, ge=1.0, le=100.0)
    num_stories: int = Field(1, ge=1, le=20)
    roof_type: Literal["flat", "pitched"] = "flat"
    roof_pitch_deg: float = Field(0.0, ge=0.0, le=60.0)


# --- Preset map ---

PRESET_MAP = {
    "simple_box": simple_box,
    "pitched_roof_house": pitched_roof_house,
    "l_shaped_block": l_shaped_block,
    "large_apartment_block": large_apartment_block,
}

PRESET_DEFAULTS = {
    "simple_box": {"width": 20.0, "depth": 10.0, "height": 8.0, "heading_deg": 0, "roof_type": "flat", "roof_pitch_deg": 0, "num_stories": 3},
    "pitched_roof_house": {"width": 30.0, "depth": 10.0, "height": 6.0, "heading_deg": 45, "roof_type": "pitched", "roof_pitch_deg": 30, "num_stories": 2},
    "l_shaped_block": {"width": 25.0, "depth": 10.0, "height": 9.0, "heading_deg": 0, "roof_type": "flat", "roof_pitch_deg": 0, "num_stories": 3},
    "large_apartment_block": {"width": 60.0, "depth": 12.0, "height": 18.0, "heading_deg": 15, "roof_type": "flat", "roof_pitch_deg": 0, "num_stories": 6},
}


# --- Building CRUD endpoints ---


@router.post("/buildings")
def create_building(req: BuildingUploadRequest):
    """Upload a GeoJSON building footprint and store it."""
    # Validate by parsing the GeoJSON into a Building
    try:
        building = build_building_from_geojson(
            req.geojson,
            height=req.height,
            num_stories=req.num_stories,
            roof_type=req.roof_type,
            roof_pitch_deg=req.roof_pitch_deg,
            name=req.name,
        )
    except (ValueError, KeyError, IndexError) as e:
        raise HTTPException(status_code=400, detail=f"Invalid GeoJSON: {e}")

    record = BuildingRecord(
        name=req.name,
        source_type="geojson",
        geometry_data=json.dumps(req.geojson),
        lat=building.lat,
        lon=building.lon,
        height=req.height,
        num_stories=req.num_stories,
        roof_type=req.roof_type,
        roof_pitch_deg=req.roof_pitch_deg,
        heading_deg=0.0,
        properties_json=json.dumps({
            "width": building.width,
            "depth": building.depth,
        }),
    )

    db = get_db()
    try:
        db.add(record)
        db.commit()
        db.refresh(record)
        return record.to_dict()
    finally:
        db.close()


@router.post("/buildings/upload-file")
async def upload_building_file(
    file: UploadFile,
    name: str = Form(""),
    lat: float = Form(53.2012),
    lon: float = Form(5.7999),
    height: float = Form(0),
    num_stories: int = Form(1),
    roof_type: str = Form("flat"),
    roof_pitch_deg: float = Form(0.0),
    min_facade_area: float = Form(1.0),
):
    """Upload an OBJ/PLY/STL mesh file — returns a task_id for progress polling.

    The heavy mesh processing runs in a background thread. Poll
    GET /buildings/upload-status/{task_id} for progress and result.
    """
    if not file.filename:
        raise HTTPException(status_code=400, detail="No file provided")

    ext = file.filename.rsplit(".", 1)[-1].lower()
    supported = {"obj": "obj", "ply": "ply", "stl": "stl", "glb": "glb", "gltf": "gltf"}
    if ext not in supported:
        raise HTTPException(status_code=400, detail=f"Unsupported format: .{ext}. Use .obj, .ply, or .stl")

    file_data = await file.read()
    build_name = name or file.filename.rsplit(".", 1)[0] or "Mesh building"
    file_size_mb = len(file_data) / (1024 * 1024)

    task_id = str(uuid.uuid4())
    with _upload_lock:
        _upload_tasks[task_id] = {
            "status": "processing",
            "progress": 0.0,
            "message": f"Reading {file_size_mb:.0f} MB mesh…",
            "result": None,
            "error": None,
        }

    def _process() -> None:
        _start_phase_recording()
        try:
            _set_upload_progress(task_id, 0.05, "Loading mesh into memory…")

            building = build_building_from_mesh(
                mesh_data=file_data,
                file_type=supported[ext],
                lat=lat,
                lon=lon,
                height=height if height > 0 else None,
                num_stories=num_stories,
                roof_type=roof_type,
                roof_pitch_deg=roof_pitch_deg,
                name=build_name,
                min_facade_area=min_facade_area,
                progress_callback=lambda p, msg: _set_upload_progress(task_id, 0.05 + p * 0.85, msg),
            )

            _set_upload_progress(task_id, 0.92, "Saving to database…")

            # Convert footprint to GeoJSON for storage
            from ..models import meters_per_deg as _m_per_deg
            m_per_lat, m_per_lon = _m_per_deg(math.radians(building.lat))
            footprint_coords = []
            for facade in building.facades:
                if abs(facade.normal[2]) < 0.01:
                    v0 = facade.vertices[0]
                    lon_v = building.lon + v0[0] / m_per_lon
                    lat_v = building.lat + v0[1] / m_per_lat
                    footprint_coords.append([round(lon_v, 8), round(lat_v, 8)])
            if footprint_coords:
                footprint_coords.append(footprint_coords[0])
            footprint_geojson = {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [footprint_coords]},
                "properties": {
                    "name": build_name,
                    "height": building.height,
                    "num_stories": num_stories,
                    "source": f"mesh_{ext}",
                },
            }

            record = BuildingRecord(
                name=build_name,
                source_type=f"mesh_{ext}",
                geometry_data=json.dumps(footprint_geojson),
                lat=building.lat,
                lon=building.lon,
                height=building.height,
                num_stories=num_stories,
                roof_type=roof_type,
                roof_pitch_deg=roof_pitch_deg,
                heading_deg=0.0,
                properties_json=json.dumps({
                    "width": building.width,
                    "depth": building.depth,
                    "mesh_format": ext,
                    "auto_height": building.height,
                    "mesh_viewer": getattr(building, "_mesh_viewer_data", None),
                }),
            )

            db = get_db()
            try:
                db.add(record)
                db.commit()
                db.refresh(record)
                result = record.to_dict()  # to_dict strips heavy blobs by default
                _new_building_id = record.id
            finally:
                db.close()

            # Warm facade cache so the first /generate is a cache hit.
            # min_facade_area here comes from the upload form (user-tunable on
            # this path); we also warm for the Pydantic default so a subsequent
            # /generate with its own default (1.0) still hits cache.
            if building.facades:
                try:
                    from ..models import AlgorithmConfig as _AlgoCfg
                    _warm_algo = _AlgoCfg()
                    _bbox = (building.width, building.depth, 0.0, 0.0)
                    for _area in {min_facade_area, 1.0}:
                        _warm_key = _facade_cache_key(_new_building_id, "region_growing", _area, _warm_algo)
                        _facade_cache_put(_warm_key, (building.facades, _bbox))
                except Exception:
                    pass

            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="complete", progress=1.0, message="Done", result=result,
                    phase_timings=_get_phase_timings(),
                )

        except Exception as e:
            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="error", progress=0.0, message=str(e), error=str(e),
                    phase_timings=_get_phase_timings(),
                )

    threading.Thread(target=_process, daemon=True).start()
    return {"task_id": task_id, "status": "processing", "message": f"Processing {file_size_mb:.0f} MB mesh…"}


@router.get("/buildings/upload-status/{task_id}")
def get_upload_status(task_id: str):
    """Poll upload processing progress."""
    with _upload_lock:
        task = _upload_tasks.get(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Upload task not found")
    return task


@router.post("/import-kmz")
async def import_kmz(
    file: UploadFile,
    voxel_size: float | None = Form(default=None),
    import_mode: Literal["raw", "facades"] = Form(default="facades"),
):
    """Import a DJI Smart3D KMZ mission into AeroScan.

    The KMZ is parsed, its reference point cloud runs through Poisson surface
    reconstruction, the resulting mesh is stored as an ``UploadedBuilding`` (so
    the user can re-run /generate on it with different mission params), and the
    imported waypoints are stored as a new MissionVersion. Heavy work runs in a
    background thread — poll ``/buildings/upload-status/{task_id}`` (reuses the
    existing task polling endpoint).
    """
    if not file.filename:
        raise HTTPException(status_code=400, detail="No file provided")
    if not file.filename.lower().endswith(".kmz"):
        raise HTTPException(status_code=400, detail="Expected a .kmz file")

    file_data = await file.read()
    base_name = file.filename.rsplit(".", 1)[0] or "imported_kmz"
    file_size_mb = len(file_data) / (1024 * 1024)

    task_id = str(uuid.uuid4())
    with _upload_lock:
        _upload_tasks[task_id] = {
            "status": "processing",
            "progress": 0.0,
            "message": f"Reading {file_size_mb:.1f} MB KMZ…",
            "result": None,
            "error": None,
        }

    def _process() -> None:
        _start_phase_recording()
        try:
            import base64
            import gzip
            import hashlib
            from ..kmz_import import (
                parse_kmz,
                waypoints_to_enu,
                polygon_to_enu,
                load_pointcloud,
                pointcloud_to_viewer_arrays,
                pointcloud_to_mesh_ply,
                facades_from_polygon,
                clip_mesh_to_polygon_xy,
                resolve_capture_intrinsics,
            )
            from ..models import ActionType, CameraAction, CameraName as _CameraName, MissionConfig

            _set_upload_progress(task_id, 0.03, "Parsing KMZ archive…")
            parsed = parse_kmz(file_data, name=base_name)
            # Initial import uses a coarse voxel for speed — Optimize refines later.
            effective_voxel = float(voxel_size) if voxel_size and voxel_size > 0 else 0.30
            vox_tag = f":v{effective_voxel:.4f}"
            # In raw mode we never reconstruct, so the voxel tag is irrelevant — keep
            # the hash stable across voxel changes so re-imports hit the same row.
            kmz_hash = hashlib.sha256(file_data).hexdigest() + (vox_tag if import_mode == "facades" else ":raw")

            if not parsed.point_cloud_ply:
                raise ValueError("KMZ does not contain a reference point cloud (wpmz/res/ply/.../cloud.ply)")

            record_source_type = "kmz_raw" if import_mode == "raw" else "kmz_import"

            # --- Cache lookup: have we processed this exact KMZ before? ---
            db = get_db()
            try:
                cached = (
                    db.query(BuildingRecord)
                    .filter_by(source_type=record_source_type)
                    .all()
                )
                cached = next(
                    (r for r in cached
                     if (json.loads(r.properties_json or "{}")).get("kmz_hash") == kmz_hash),
                    None,
                )
                cached_props = json.loads(cached.properties_json) if cached else {}
            finally:
                db.close()

            pc_positions: list[float]
            pc_colors: list[float]

            if import_mode == "raw":
                _set_upload_progress(task_id, 0.20, f"Loading point cloud ({len(parsed.point_cloud_ply)//1024} KB)…")
                pcd = load_pointcloud(parsed.point_cloud_ply)
                _set_upload_progress(task_id, 0.55, "Subsampling point cloud for viewer…")
                pc_positions, pc_colors = pointcloud_to_viewer_arrays(pcd, max_points=250_000)

                from ..models import Building as _Building, RoofType as _RoofType
                # Building bounds come from the RC Plus mission-area polygon +
                # point-cloud Z extent — the same volume the "Mapping box" toggle
                # plots in the viewer.
                import numpy as _np_raw
                area_enu_raw = polygon_to_enu(
                    parsed.mission_area_wgs84,
                    parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
                )
                if area_enu_raw:
                    area_arr = _np_raw.asarray(area_enu_raw, dtype=float).reshape(-1, 3)[:, :2]
                    bbox_w = float(area_arr[:, 0].max() - area_arr[:, 0].min())
                    bbox_d = float(area_arr[:, 1].max() - area_arr[:, 1].min())
                else:
                    bbox_w = bbox_d = 0.0
                try:
                    pc_z = _np_raw.asarray(pcd.points, dtype=_np_raw.float32)[:, 2]
                    bbox_h = float(pc_z.max() - pc_z.min()) if pc_z.size else 0.0
                except Exception:
                    bbox_h = 0.0
                building = _Building(
                    lat=parsed.ref_lat, lon=parsed.ref_lon, ground_altitude=parsed.ref_alt,
                    width=bbox_w, depth=bbox_d, height=bbox_h,
                    heading_deg=0.0,
                    roof_type=_RoofType.FLAT, roof_pitch_deg=0.0,
                    num_stories=1, facades=[],
                )
                building.label = parsed.name
                mesh_bytes = None
                building_id = cached.id if cached else None
            elif cached and cached_props.get("mesh_viewer") and cached_props.get("pc_b64"):
                _set_upload_progress(task_id, 0.50, "Reusing cached reconstruction…")
                raw_pc = gzip.decompress(base64.b64decode(cached_props["pc_b64"]))
                import numpy as _np
                pc_arr = _np.frombuffer(raw_pc, dtype=_np.float32)
                half = pc_arr.size // 2
                pc_positions = pc_arr[:half].tolist()
                pc_colors = pc_arr[half:].tolist()

                # Rebuild Building from cached mesh_viewer data (fast, no Open3D).
                import numpy as _np2
                verts, faces = decode_mesh_viewer_data(cached_props["mesh_viewer"])
                # Re-encode to PLY so we can reuse build_building_from_mesh's full pipeline
                import trimesh
                tm = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
                mesh_bytes = tm.export(file_type="ply")
                building = build_building_from_mesh(
                    mesh_data=mesh_bytes, file_type="ply",
                    lat=parsed.ref_lat, lon=parsed.ref_lon, name=parsed.name,
                    preserve_position=True,
                )
                building.label = parsed.name
                building_id = cached.id

                _set_upload_progress(task_id, 0.82, "Tight-clipping mesh to building + extracting facades…")
                area_enu_cached = polygon_to_enu(
                    parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
                )
                import io as _io
                import numpy as _np_cached
                pc_xyz_cached = _np_cached.asarray(pc_positions, dtype=_np_cached.float32).reshape(-1, 3)
                clip_poly_cached = _tighten_clip_polygon(area_enu_cached, pc_xyz_cached)
                clipped_bytes = clip_mesh_to_polygon_xy(mesh_bytes, clip_poly_cached)
                _mesh_clipped = trimesh.load(
                    _io.BytesIO(clipped_bytes), file_type="ply", force="mesh", process=False,
                )
                building.facades = _kmz_extract_best_facades(_mesh_clipped, clip_poly_cached, pc_xyz_cached)
                building.facades = _filter_facades_by_dji_bbox(
                    building.facades,
                    parsed.mission_area_wgs84,
                    parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
                )
            else:
                _set_upload_progress(task_id, 0.15, f"Loading point cloud ({len(parsed.point_cloud_ply)//1024} KB)…")
                pcd = load_pointcloud(parsed.point_cloud_ply)

                _set_upload_progress(task_id, 0.25, "Subsampling point cloud for viewer…")
                pc_positions, pc_colors = pointcloud_to_viewer_arrays(pcd, max_points=250_000)

                _set_upload_progress(task_id, 0.35, f"Reconstructing coarse mesh (voxel={effective_voxel:.2f}m)…")
                mesh_bytes = pointcloud_to_mesh_ply(pcd, voxel_size_override=effective_voxel)

                _set_upload_progress(task_id, 0.65, "Building footprint from mesh…")
                building = build_building_from_mesh(
                    mesh_data=mesh_bytes,
                    file_type="ply",
                    lat=parsed.ref_lat,
                    lon=parsed.ref_lon,
                    name=parsed.name,
                    preserve_position=True,
                )
                building.label = parsed.name
                building_id = None

                _set_upload_progress(task_id, 0.80, "Tight-clipping mesh to building + extracting facades…")
                area_enu_fresh = polygon_to_enu(
                    parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
                )
                import io as _io
                import trimesh as _trimesh2
                import numpy as _np_fresh
                pc_xyz_fresh = _np_fresh.asarray(pcd.points, dtype=_np_fresh.float32)
                clip_poly_fresh = _tighten_clip_polygon(area_enu_fresh, pc_xyz_fresh)
                clipped_bytes = clip_mesh_to_polygon_xy(mesh_bytes, clip_poly_fresh)
                _mesh_clipped = _trimesh2.load(
                    _io.BytesIO(clipped_bytes), file_type="ply", force="mesh", process=False,
                )
                building.facades = _kmz_extract_best_facades(_mesh_clipped, clip_poly_fresh, pc_xyz_fresh)
                building.facades = _filter_facades_by_dji_bbox(
                    building.facades,
                    parsed.mission_area_wgs84,
                    parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
                )

            _set_upload_progress(task_id, 0.88, "Saving imported building…")

            from ..models import meters_per_deg as _m_per_deg
            m_per_lat, m_per_lon = _m_per_deg(math.radians(building.lat))
            footprint_coords = []
            for facade in building.facades:
                if abs(facade.normal[2]) < 0.01:
                    v0 = facade.vertices[0]
                    lon_v = building.lon + v0[0] / m_per_lon
                    lat_v = building.lat + v0[1] / m_per_lat
                    footprint_coords.append([round(lon_v, 8), round(lat_v, 8)])
            # Raw imports skip facade extraction, so no footprint can be derived
            # from facades. Fall back to the DJI mission-area polygon (~20-pt
            # polygon shipped in template.kml), then to the point-cloud/mesh
            # bbox projected onto WGS84. Either way /api/generate needs a valid
            # ≥3-vertex polygon to rebuild the building on demand.
            if len(footprint_coords) < 3 and parsed.mission_area_wgs84:
                footprint_coords = [
                    [round(lon, 8), round(lat, 8)]
                    for lon, lat, _ in parsed.mission_area_wgs84
                ]
            if len(footprint_coords) < 3:
                # Last-resort bbox from point cloud positions
                try:
                    import numpy as _np_fb
                    if pc_positions:
                        arr = _np_fb.asarray(pc_positions, dtype=_np_fb.float32).reshape(-1, 3)
                        xmin, ymin = float(arr[:, 0].min()), float(arr[:, 1].min())
                        xmax, ymax = float(arr[:, 0].max()), float(arr[:, 1].max())
                        corners_enu = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
                        footprint_coords = [
                            [round(building.lon + x / m_per_lon, 8),
                             round(building.lat + y / m_per_lat, 8)]
                            for x, y in corners_enu
                        ]
                except Exception:
                    pass
            if footprint_coords and footprint_coords[0] != footprint_coords[-1]:
                footprint_coords.append(footprint_coords[0])
            footprint_geojson = {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [footprint_coords]},
                "properties": {"name": parsed.name, "height": building.height, "source": record_source_type},
            }

            camera_intrinsics = resolve_capture_intrinsics(parsed)

            if building_id is None:
                import numpy as _np3
                pc_arr = _np3.concatenate([
                    _np3.asarray(pc_positions, dtype=_np3.float32),
                    _np3.asarray(pc_colors, dtype=_np3.float32),
                ])
                pc_b64 = base64.b64encode(gzip.compress(pc_arr.tobytes())).decode("ascii")

                record = BuildingRecord(
                    name=parsed.name,
                    source_type=record_source_type,
                    geometry_data=json.dumps(footprint_geojson),
                    lat=building.lat,
                    lon=building.lon,
                    height=building.height,
                    num_stories=1,
                    roof_type="flat",
                    roof_pitch_deg=0.0,
                    heading_deg=0.0,
                    properties_json=json.dumps({
                        "width": building.width,
                        "depth": building.depth,
                        "mesh_format": "ply",
                        "auto_height": building.height,
                        "mesh_viewer": getattr(building, "_mesh_viewer_data", None),
                        "ref_alt": parsed.ref_alt,
                        "ref_lat": parsed.ref_lat,
                        "ref_lon": parsed.ref_lon,
                        "kmz_hash": kmz_hash,
                        "pc_b64": pc_b64,
                        "ply_b64": base64.b64encode(gzip.compress(parsed.point_cloud_ply)).decode("ascii"),
                        "mission_area_wgs84": parsed.mission_area_wgs84,
                        "waypoints_raw": [
                            {
                                "index": wp.index,
                                "lat": wp.lat, "lon": wp.lon, "alt_egm96": wp.alt_egm96,
                                "heading_deg": wp.heading_deg,
                                "gimbal_pitch_deg": wp.gimbal_pitch_deg,
                                "gimbal_yaw_raw_deg": wp.gimbal_yaw_raw_deg,
                                "gimbal_heading_mode": wp.gimbal_heading_mode,
                                "gimbal_yaw_base": wp.gimbal_yaw_base,
                                "speed_ms": wp.speed_ms,
                                "smart_oblique_poses": [
                                    {"pitch": p.pitch_deg, "yaw_offset": p.yaw_offset_deg, "roll": p.roll_deg}
                                    for p in wp.smart_oblique_poses
                                ],
                            } for wp in parsed.waypoints
                        ],
                        "mission_config_raw": parsed.mission_config_raw,
                        "camera_intrinsics": camera_intrinsics,
                        "voxel_size": effective_voxel,
                    }),
                )
                db = get_db()
                try:
                    db.add(record)
                    db.commit()
                    db.refresh(record)
                    building_id = record.id
                finally:
                    db.close()

            # Warm the facade cache with the facades we already extracted during
            # build_building_from_mesh. Without this, the first /generate call
            # on this building re-runs extract_facades from scratch (region
            # growing + HPR) for the same params — a ~second re-run of work we
            # just did. The AlgorithmParams pydantic defaults match
            # AlgorithmConfig defaults, so this key matches an unmodified
            # /generate request.
            if building_id is not None and building.facades:
                try:
                    from ..models import AlgorithmConfig as _AlgoCfg
                    _warm_algo = _AlgoCfg()
                    _warm_key = _facade_cache_key(building_id, "region_growing", 1.0, _warm_algo)
                    _facade_cache_put(
                        _warm_key,
                        (building.facades, (building.width, building.depth, 0.0, 0.0)),
                    )
                except Exception:
                    pass  # best-effort; not worth failing the import for

            # --- Build waypoints in local ENU ---
            enu_wps = waypoints_to_enu(parsed.waypoints, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt)
            from ..models import Waypoint
            waypoints: list[Waypoint] = []
            for i, w in enumerate(enu_wps):
                wp = Waypoint(
                    x=w["x"], y=w["y"], z=w["z"],
                    lat=w["lat"], lon=w["lon"], alt=w["alt"],
                    heading_deg=w["heading_deg"],
                    gimbal_pitch_deg=w["gimbal_pitch_deg"],
                    gimbal_yaw_deg=w.get("gimbal_yaw_deg"),
                    speed_ms=w["speed_ms"],
                    facade_index=-1,
                    index=i,
                    actions=[CameraAction(
                        action_type=ActionType.TAKE_PHOTO,
                        camera=_CameraName.WIDE,
                    )],
                )
                waypoints.append(wp)

            # Mission area polygon in ENU + WGS84 (lat, lon)
            area_enu = polygon_to_enu(parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt)
            area_latlon = [(lat, lon) for lon, lat, _ in parsed.mission_area_wgs84]

            _set_upload_progress(task_id, 0.94, "Building viewer data…")

            # MissionConfig from parsed mission_config where possible
            raw_cfg = parsed.mission_config_raw or {}
            def _fget(key: str, default: float) -> float:
                try:
                    return float(raw_cfg.get(key, default))
                except (TypeError, ValueError):
                    return default
            config = MissionConfig(
                flight_speed_ms=_fget("autoFlightSpeed", 2.0),
                mission_name=f"Imported: {parsed.name}",
            )

            # Summary (light — no validation; these are DJI-native waypoints)
            total_path_m = 0.0
            for i in range(1, len(waypoints)):
                dx = waypoints[i].x - waypoints[i - 1].x
                dy = waypoints[i].y - waypoints[i - 1].y
                dz = waypoints[i].z - waypoints[i - 1].z
                total_path_m += math.sqrt(dx * dx + dy * dy + dz * dz)
            est_time_s = total_path_m / max(config.flight_speed_ms, 0.1)
            summary = {
                "waypoint_count": len(waypoints),
                "inspection_waypoints": len(waypoints),
                "transition_waypoints": 0,
                "photo_count": len(waypoints),
                "facade_count": len(building.facades),
                "camera_distance_m": camera_intrinsics["distance_m"],
                "photo_footprint_m": [0.0, 0.0],
                "total_path_m": round(total_path_m, 1),
                "estimated_flight_time_s": round(est_time_s),
                "facade_waypoint_counts": {},
                "transitions": [],
                "camera": camera_intrinsics,
                "source": record_source_type,
                "facade_coverage": compute_facade_coverage(building, waypoints),
            }

            threejs_data = prepare_threejs_data(
                building, waypoints,
                point_cloud={"positions": pc_positions, "colors": pc_colors},
                mission_area=area_enu,
            )
            # Stamp DJI SmartOblique rosette poses onto viewer waypoint dicts so
            # the frontend can render all 5 original gimbal directions per
            # waypoint (not just the transitional pitch/yaw).
            vw = threejs_data.get("waypoints", []) if isinstance(threejs_data, dict) else []
            for i, wd in enumerate(vw):
                if i < len(enu_wps):
                    poses = enu_wps[i].get("smart_oblique_poses") or []
                    if poses:
                        wd["smart_oblique_poses"] = [
                            {"pitch": round(p["pitch"], 1), "yaw": round(p["yaw"], 1)}
                            for p in poses
                        ]
                    # Needed for shutter-rate shot simulation in the viewer:
                    # shots per pass segment = dist / speed / shutter_interval.
                    wd["speed_ms"] = round(float(enu_wps[i].get("speed_ms", 2.0)), 3)
            # Attach raw reconstructed mesh so the viewer can render it.
            # Forward the already-gzipped+base64 buffers: re-serializing via
            # .tolist() + JSON for a multi-hundred-k-face mesh is the single
            # slowest step in KMZ import and freezes the frontend during parse.
            raw_mesh = getattr(building, "_mesh_viewer_data", None)
            if raw_mesh and "positions_b64" in raw_mesh and "indices_b64" in raw_mesh:
                threejs_data["rawMesh"] = {
                    "positions_b64": raw_mesh["positions_b64"],
                    "indices_b64": raw_mesh["indices_b64"],
                    "n_vertices": raw_mesh.get("n_vertices"),
                    "n_faces": raw_mesh.get("n_faces"),
                }

            leaflet_data = prepare_leaflet_data(
                building, waypoints,
                mission_area_poly=area_latlon,
            )

            viewer_data = {"threejs": threejs_data, "leaflet": leaflet_data}

            version = session.store(
                building_params={
                    "lat": building.lat, "lon": building.lon,
                    "width": building.width, "depth": building.depth,
                    "height": building.height, "heading_deg": 0.0,
                    "roof_type": "flat", "roof_pitch_deg": 0.0, "num_stories": 1,
                },
                mission_params={
                    "target_gsd_mm_per_px": 2.0,
                    "camera": "wide",
                    "flight_speed_ms": config.flight_speed_ms,
                    "mission_name": config.mission_name,
                },
                building=building,
                waypoints=waypoints,
                config=config,
                summary=summary,
                viewer_data=viewer_data,
                selection={"building_id": building_id},
            )

            response = {
                "version_id": version.version_id,
                "timestamp": version.timestamp,
                "summary": summary,
                "viewer_data": viewer_data,
                "validation": [],
                "can_export": True,
                "config_snapshot": {
                    "building_id": building_id,
                    "building": version.building_params,
                    "mission": version.mission_params,
                },
                "building_id": building_id,
                "imported_name": parsed.name,
            }

            # Persist as the initial DJI snapshot so "Open" / "switch to DJI"
            # later hits the fast load path without needing a refine first.
            if building_id:
                try:
                    _snapshot_save(building_id, "dji", response, {}, version.version_id)
                except Exception as _snap_exc:
                    print(f"[kmz import] snapshot save failed: {_snap_exc}")

            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="complete", progress=1.0, message="Import complete", result=response,
                    phase_timings=_get_phase_timings(),
                )

        except Exception as e:
            import traceback
            traceback.print_exc()
            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="error", progress=0.0, message=str(e), error=str(e),
                    phase_timings=_get_phase_timings(),
                )

    threading.Thread(target=_process, daemon=True).start()
    return {"task_id": task_id, "status": "processing", "message": f"Importing {file_size_mb:.1f} MB KMZ…"}


@router.get("/buildings")
def list_buildings():
    """List all uploaded buildings, with lightweight snapshot metadata."""
    db = get_db()
    try:
        records = db.query(BuildingRecord).order_by(BuildingRecord.created_at.desc()).all()
        out = []
        for r in records:
            d = r.to_dict()
            # Re-parse heavy props to derive snapshot availability without
            # shipping the blobs back to the client.
            try:
                heavy = json.loads(r.properties_json or "{}")
            except (json.JSONDecodeError, TypeError):
                heavy = {}
            snaps = _snapshots_get(heavy)
            d["available_modes"] = sorted(
                m for m in snaps if snaps[m].get("response_gz_b64")
            )
            d["active_mode"] = heavy.get("active_mode") or (
                d["available_modes"][0] if d["available_modes"] else None
            )
            out.append(d)
        return {"buildings": out}
    finally:
        db.close()


class SaveSnapshotRequest(BaseModel):
    version_id: str
    settings: dict | None = None


@router.post("/buildings/{building_id}/snapshots/{mode}")
def save_building_snapshot(building_id: str, mode: str, req: SaveSnapshotRequest):
    """Persist a MissionVersion as the named-mode snapshot for a building.

    The frontend calls this after a generate / rewrite completes so the mode
    can be re-loaded instantly later. Fetches the version's response shape
    out of the in-memory ``session`` store, compresses, and writes.
    """
    if mode not in SNAPSHOT_MODES:
        raise HTTPException(status_code=400, detail=f"Unknown mode: {mode!r}")
    version = session.get(req.version_id)
    if version is None:
        raise HTTPException(status_code=404, detail="Version not found")

    # Reconstruct the GenerateResponse-shaped dict from the in-memory version.
    response = {
        "version_id": version.version_id,
        "timestamp": version.timestamp,
        "summary": version.summary,
        "viewer_data": version.viewer_data,
        "validation": [],
        "can_export": True,
        "config_snapshot": {
            "building_id": building_id,
            "building": version.building_params,
            "mission": version.mission_params,
        },
        "building_id": building_id,
    }
    _snapshot_save(
        building_id, mode, response, req.settings or {}, version.version_id,
    )
    return {"ok": True, "mode": mode, "version_id": version.version_id}


@router.delete("/buildings/{building_id}/snapshots/{mode}")
def delete_building_snapshot(building_id: str, mode: str):
    """Drop one named snapshot (e.g. discard the inspection path)."""
    if mode not in SNAPSHOT_MODES:
        raise HTTPException(status_code=400, detail=f"Unknown mode: {mode!r}")
    db = get_db()
    try:
        rec = db.query(BuildingRecord).filter_by(id=building_id).first()
        if rec is None:
            raise HTTPException(status_code=404, detail="Building not found")
        props = json.loads(rec.properties_json or "{}")
        snaps = _snapshots_get(props)
        removed = snaps.pop(mode, None)
        props["snapshots"] = snaps
        if mode == "dji":
            props.pop("last_response_gz_b64", None)
            props.pop("last_settings", None)
            props.pop("last_version_id", None)
        if props.get("active_mode") == mode:
            remaining = [m for m in snaps if snaps[m].get("response_gz_b64")]
            props["active_mode"] = remaining[0] if remaining else None
        rec.properties_json = json.dumps(props)
        db.commit()
        return {"ok": True, "removed": bool(removed)}
    finally:
        db.close()


@router.get("/buildings/{building_id}")
def get_building(building_id: str):
    """Get a specific uploaded building."""
    db = get_db()
    try:
        record = db.query(BuildingRecord).filter_by(id=building_id).first()
        if not record:
            raise HTTPException(status_code=404, detail="Building not found")
        return record.to_dict()
    finally:
        db.close()


class RefineKmzRequest(BaseModel):
    voxel_size: float = Field(..., gt=0.01, le=1.0)
    # CGAL alpha_wrap_3 overrides (None → auto-derived from voxel_size).
    aw_alpha: float | None = Field(default=None, gt=0.005, le=2.0)
    aw_offset: float | None = Field(default=None, gt=0.001, le=1.0)
    # CGAL Shape-Detection facade extraction overrides (None → library defaults).
    # Bounds are inclusive on both sides so slider steps can land on the
    # boundary without triggering 422 validation errors.
    fd_epsilon: float | None = Field(default=None, ge=0.005, le=0.5)
    fd_cluster_epsilon: float | None = Field(default=None, ge=0.02, le=2.0)
    fd_min_points: int | None = Field(default=None, ge=5, le=1000)
    fd_min_wall_area_m2: float | None = Field(default=None, ge=0.05, le=20.0)
    fd_min_roof_area_m2: float | None = Field(default=None, ge=0.05, le=20.0)
    fd_min_density_per_m2: float | None = Field(default=None, ge=1.0, le=500.0)
    fd_normal_threshold: float | None = Field(default=None, ge=0.5, le=0.999)


@router.post("/buildings/{building_id}/refine-kmz")
def refine_kmz_building(building_id: str, req: RefineKmzRequest):
    """Re-reconstruct a KMZ-imported building at a finer voxel size.

    Reuses the raw PLY bytes stored on the BuildingRecord so the user
    doesn't have to re-upload. Updates mesh_viewer in place, invalidates
    the facade cache for this building, and emits a new MissionVersion.
    Heavy work runs in a background thread — poll the shared upload-status.
    """
    db = get_db()
    try:
        record = db.query(BuildingRecord).filter_by(id=building_id).first()
        if not record:
            raise HTTPException(status_code=404, detail="Building not found")
        props = json.loads(record.properties_json or "{}")
    finally:
        db.close()

    if not props.get("ply_b64"):
        raise HTTPException(
            status_code=400,
            detail="This building was not imported from a KMZ; cannot refine.",
        )

    voxel_size = float(req.voxel_size)
    task_id = str(uuid.uuid4())
    with _upload_lock:
        _upload_tasks[task_id] = {
            "status": "processing",
            "progress": 0.0,
            "message": f"Refining at voxel={voxel_size:.3f}m…",
            "result": None,
            "error": None,
        }

    def _process() -> None:
        try:
            import base64
            import gzip
            from ..kmz_import import load_pointcloud, pointcloud_to_mesh_ply, pointcloud_to_viewer_arrays
            from ..models import ActionType, CameraAction, CameraName as _CameraName, MissionConfig, Waypoint

            ply_bytes = gzip.decompress(base64.b64decode(props["ply_b64"]))

            ref_lat = float(props.get("ref_lat", record.lat))
            ref_lon = float(props.get("ref_lon", record.lon))
            ref_alt = float(props.get("ref_alt", 0.0))
            name = record.name

            # --- PHASE 1: mesh reconstruction (cached on voxel + aw_* params) ---
            mesh_key = _kmz_mesh_key(building_id, voxel_size, req.aw_alpha, req.aw_offset)
            cached_mesh = _kmz_mesh_cache_get(mesh_key)
            if cached_mesh is not None:
                (mesh_bytes, pcd, pc_positions, pc_colors, building,
                 clip_poly_ref, clipped_mesh_trimesh, pc_xyz_ref) = cached_mesh
                _set_upload_progress(task_id, 0.70, "Mesh cache hit — skipping reconstruction.")
            else:
                _set_upload_progress(task_id, 0.10, "Loading point cloud…")
                pcd = load_pointcloud(ply_bytes)

                _set_upload_progress(task_id, 0.25, f"Reconstructing at voxel={voxel_size:.3f}m…")
                mesh_bytes = pointcloud_to_mesh_ply(
                    pcd,
                    voxel_size_override=voxel_size,
                    aw_alpha_override=(float(req.aw_alpha) if req.aw_alpha else None),
                    aw_offset_override=(float(req.aw_offset) if req.aw_offset else None),
                )

                _set_upload_progress(task_id, 0.55, "Subsampling point cloud for viewer…")
                pc_positions, pc_colors = pointcloud_to_viewer_arrays(pcd, max_points=250_000)

                _set_upload_progress(task_id, 0.65, "Building footprint from mesh…")
                building = build_building_from_mesh(
                    mesh_data=mesh_bytes, file_type="ply",
                    lat=ref_lat, lon=ref_lon, name=name,
                    preserve_position=True,
                )
                building.label = name

                _set_upload_progress(task_id, 0.72, "Tight-clipping mesh to building…")
                from ..kmz_import import (
                    polygon_to_enu as _polygon_to_enu,
                    clip_mesh_to_polygon_xy as _clip_mesh_to_polygon_xy,
                )
                mission_area_wgs84_cache = [tuple(p) for p in props.get("mission_area_wgs84", [])]
                area_enu_refine = _polygon_to_enu(mission_area_wgs84_cache, ref_lat, ref_lon, ref_alt)
                import io as _io
                import trimesh as _trimesh3
                import numpy as _np_ref
                pc_xyz_ref = _np_ref.asarray(pcd.points, dtype=_np_ref.float32)
                clip_poly_ref = _tighten_clip_polygon(area_enu_refine, pc_xyz_ref)
                clipped_bytes_ref = _clip_mesh_to_polygon_xy(mesh_bytes, clip_poly_ref)
                clipped_mesh_trimesh = _trimesh3.load(
                    _io.BytesIO(clipped_bytes_ref), file_type="ply", force="mesh", process=False,
                )
                _kmz_mesh_cache_put(mesh_key, (
                    mesh_bytes, pcd, pc_positions, pc_colors, building,
                    clip_poly_ref, clipped_mesh_trimesh, pc_xyz_ref,
                ))

            # --- PHASE 2: facade extraction (cached on fd_* params + mesh_key) ---
            fd_overrides_refine = {
                "epsilon": req.fd_epsilon,
                "cluster_epsilon": req.fd_cluster_epsilon,
                "min_points": req.fd_min_points,
                "min_wall_area_m2": req.fd_min_wall_area_m2,
                "min_roof_area_m2": req.fd_min_roof_area_m2,
                "min_density_per_m2": req.fd_min_density_per_m2,
                "normal_threshold": req.fd_normal_threshold,
            }
            facade_key = _kmz_facade_runtime_key(mesh_key, fd_overrides_refine)
            cached_facades = _kmz_facade_runtime_cache_get(facade_key)
            if cached_facades is not None:
                _set_upload_progress(task_id, 0.82, "Facade cache hit — skipping CGAL.")
                building.facades = cached_facades
            else:
                _set_upload_progress(task_id, 0.78, "Extracting facades (CGAL)…")
                building.facades = _kmz_extract_best_facades(
                    clipped_mesh_trimesh, clip_poly_ref, pc_xyz_ref,
                    fd_overrides=fd_overrides_refine,
                )
                _kmz_facade_runtime_cache_put(facade_key, building.facades)
            building.facades = _filter_facades_by_dji_bbox(
                building.facades,
                props.get("mission_area_wgs84"),
                ref_lat, ref_lon, ref_alt,
            )

            # --- Pre-compute pc_b64 now; DB write happens after response is built. ---
            _set_upload_progress(task_id, 0.85, "Updating building record…")
            import numpy as _np3
            pc_arr = _np3.concatenate([
                _np3.asarray(pc_positions, dtype=_np3.float32),
                _np3.asarray(pc_colors, dtype=_np3.float32),
            ])
            pc_b64 = base64.b64encode(gzip.compress(pc_arr.tobytes())).decode("ascii")

            # Facade cache is keyed on (building_id, extraction_method, ...) — invalidate.
            _facade_cache_invalidate(building_id)

            # --- Rebuild waypoints + mission area from stored raw data ---
            waypoints_raw = props.get("waypoints_raw", [])
            mission_area_wgs84 = [tuple(p) for p in props.get("mission_area_wgs84", [])]
            mission_cfg_raw = props.get("mission_config_raw", {}) or {}

            from ..kmz_import import ParsedWaypoint, SmartObliquePose, waypoints_to_enu, polygon_to_enu
            parsed_wps = [
                ParsedWaypoint(
                    index=int(w.get("index", i)),
                    lon=float(w["lon"]), lat=float(w["lat"]),
                    alt_egm96=float(w["alt_egm96"]),
                    heading_deg=float(w["heading_deg"]),
                    gimbal_pitch_deg=float(w["gimbal_pitch_deg"]),
                    speed_ms=float(w.get("speed_ms", 2.0)),
                    gimbal_yaw_raw_deg=float(w.get("gimbal_yaw_raw_deg", 0.0)),
                    gimbal_heading_mode=str(w.get("gimbal_heading_mode", "smoothTransition")),
                    gimbal_yaw_base=str(w.get("gimbal_yaw_base", "aircraft")),
                    smart_oblique_poses=[
                        SmartObliquePose(
                            pitch_deg=float(p.get("pitch", 0.0)),
                            yaw_offset_deg=float(p.get("yaw_offset", 0.0)),
                            roll_deg=float(p.get("roll", 0.0)),
                        )
                        for p in (w.get("smart_oblique_poses") or [])
                    ],
                )
                for i, w in enumerate(waypoints_raw)
            ]
            enu_wps = waypoints_to_enu(parsed_wps, ref_lat, ref_lon, ref_alt)
            waypoints: list[Waypoint] = []
            for i, w in enumerate(enu_wps):
                waypoints.append(Waypoint(
                    x=w["x"], y=w["y"], z=w["z"],
                    lat=w["lat"], lon=w["lon"], alt=w["alt"],
                    heading_deg=w["heading_deg"],
                    gimbal_pitch_deg=w["gimbal_pitch_deg"],
                    gimbal_yaw_deg=w.get("gimbal_yaw_deg"),
                    speed_ms=w["speed_ms"],
                    facade_index=-1, index=i,
                    actions=[CameraAction(action_type=ActionType.TAKE_PHOTO, camera=_CameraName.WIDE)],
                ))

            area_enu = polygon_to_enu(mission_area_wgs84, ref_lat, ref_lon, ref_alt)
            area_latlon = [(lat, lon) for lon, lat, _ in mission_area_wgs84]

            def _fget(key: str, default: float) -> float:
                try: return float(mission_cfg_raw.get(key, default))
                except (TypeError, ValueError): return default
            config = MissionConfig(
                flight_speed_ms=_fget("autoFlightSpeed", 2.0),
                mission_name=f"Refined ({voxel_size:.2f}m): {name}",
            )

            total_path_m = 0.0
            for i in range(1, len(waypoints)):
                dx = waypoints[i].x - waypoints[i-1].x
                dy = waypoints[i].y - waypoints[i-1].y
                dz = waypoints[i].z - waypoints[i-1].z
                total_path_m += math.sqrt(dx*dx + dy*dy + dz*dz)

            from ..kmz_import import (
                ImportedKmz as _ImportedKmz,
                resolve_capture_intrinsics as _resolve_intr,
            )
            refine_intrinsics = _resolve_intr(_ImportedKmz(
                name=name, ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt,
                waypoints=parsed_wps, mission_area_wgs84=mission_area_wgs84,
                mission_config_raw=mission_cfg_raw, point_cloud_ply=None,
            ))
            summary = {
                "waypoint_count": len(waypoints),
                "inspection_waypoints": len(waypoints),
                "transition_waypoints": 0,
                "photo_count": len(waypoints),
                "facade_count": len(building.facades),
                "camera_distance_m": refine_intrinsics["distance_m"],
                "photo_footprint_m": [0.0, 0.0],
                "total_path_m": round(total_path_m, 1),
                "estimated_flight_time_s": round(total_path_m / max(config.flight_speed_ms, 0.1)),
                "facade_waypoint_counts": {},
                "transitions": [],
                "camera": refine_intrinsics,
                "source": "kmz_refine",
                "voxel_size": voxel_size,
                "facade_coverage": compute_facade_coverage(building, waypoints),
            }

            threejs_data = prepare_threejs_data(
                building, waypoints,
                point_cloud={"positions": pc_positions, "colors": pc_colors},
                mission_area=area_enu,
            )
            vw = threejs_data.get("waypoints", []) if isinstance(threejs_data, dict) else []
            for i, wd in enumerate(vw):
                if i < len(enu_wps):
                    poses = enu_wps[i].get("smart_oblique_poses") or []
                    if poses:
                        wd["smart_oblique_poses"] = [
                            {"pitch": round(p["pitch"], 1), "yaw": round(p["yaw"], 1)}
                            for p in poses
                        ]
                    # Needed for shutter-rate shot simulation in the viewer:
                    # shots per pass segment = dist / speed / shutter_interval.
                    wd["speed_ms"] = round(float(enu_wps[i].get("speed_ms", 2.0)), 3)
            raw_mesh = getattr(building, "_mesh_viewer_data", None)
            if raw_mesh and "positions_b64" in raw_mesh and "indices_b64" in raw_mesh:
                threejs_data["rawMesh"] = {
                    "positions_b64": raw_mesh["positions_b64"],
                    "indices_b64": raw_mesh["indices_b64"],
                    "n_vertices": raw_mesh.get("n_vertices"),
                    "n_faces": raw_mesh.get("n_faces"),
                }
            leaflet_data = prepare_leaflet_data(building, waypoints, mission_area_poly=area_latlon)
            viewer_data = {"threejs": threejs_data, "leaflet": leaflet_data}

            version = session.store(
                building_params={
                    "lat": building.lat, "lon": building.lon,
                    "width": building.width, "depth": building.depth,
                    "height": building.height, "heading_deg": 0.0,
                    "roof_type": "flat", "roof_pitch_deg": 0.0, "num_stories": 1,
                },
                mission_params={
                    "target_gsd_mm_per_px": 2.0, "camera": "wide",
                    "flight_speed_ms": config.flight_speed_ms,
                    "mission_name": config.mission_name,
                },
                building=building, waypoints=waypoints, config=config,
                summary=summary, viewer_data=viewer_data,
                selection={"building_id": building_id},
            )

            response = {
                "version_id": version.version_id,
                "timestamp": version.timestamp,
                "summary": summary,
                "viewer_data": viewer_data,
                "validation": [],
                "can_export": True,
                "config_snapshot": {
                    "building_id": building_id,
                    "building": version.building_params,
                    "mission": version.mission_params,
                },
                "building_id": building_id,
                "imported_name": name,
                "voxel_size": voxel_size,
            }

            dji_settings = {
                "voxel_size": voxel_size,
                "aw_alpha": float(req.aw_alpha) if req.aw_alpha is not None else None,
                "aw_offset": float(req.aw_offset) if req.aw_offset is not None else None,
                "fd_epsilon": float(req.fd_epsilon) if req.fd_epsilon is not None else None,
                "fd_cluster_epsilon": float(req.fd_cluster_epsilon) if req.fd_cluster_epsilon is not None else None,
                "fd_min_points": int(req.fd_min_points) if req.fd_min_points is not None else None,
                "fd_min_wall_area_m2": float(req.fd_min_wall_area_m2) if req.fd_min_wall_area_m2 is not None else None,
                "fd_min_roof_area_m2": float(req.fd_min_roof_area_m2) if req.fd_min_roof_area_m2 is not None else None,
                "fd_min_density_per_m2": float(req.fd_min_density_per_m2) if req.fd_min_density_per_m2 is not None else None,
                "fd_normal_threshold": float(req.fd_normal_threshold) if req.fd_normal_threshold is not None else None,
            }

            # Also persist mesh / point-cloud / geometry alongside the snapshot,
            # and sync the record's height field.
            extra = {
                "mesh_viewer": getattr(building, "_mesh_viewer_data", None),
                "pc_b64": pc_b64,
                "voxel_size": voxel_size,
                "width": building.width,
                "depth": building.depth,
                "auto_height": building.height,
            }
            _snapshot_save(
                building_id, "dji", response, dji_settings, version.version_id,
                extra_props=extra,
            )
            db2 = get_db()
            try:
                rec = db2.query(BuildingRecord).filter_by(id=building_id).first()
                if rec is not None:
                    rec.height = building.height
                    db2.commit()
            finally:
                db2.close()

            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="complete", progress=1.0,
                    message=f"Refinement at {voxel_size:.3f}m complete", result=response,
                )
        except Exception as e:
            import traceback
            traceback.print_exc()
            with _upload_lock:
                _upload_tasks[task_id].update(
                    status="error", progress=0.0, message=str(e), error=str(e),
                )

    threading.Thread(target=_process, daemon=True).start()
    return {"task_id": task_id, "status": "processing"}


@router.post("/buildings/{building_id}/load")
def load_building(building_id: str, mode: str | None = None):
    """Instantly re-hydrate a saved building's snapshot for the given mode.

    ``mode`` picks which path to restore — ``dji`` (DJI original trajectory
    with tuned facades) or ``inspection`` (generated NEN-2767 boustrophedon).
    When omitted, uses the record's ``active_mode`` (falls back to ``dji``).

    Emits a fresh ``MissionVersion`` so the existing KMZ / rewrite endpoints
    work against the loaded state.
    """
    db = get_db()
    try:
        record = db.query(BuildingRecord).filter_by(id=building_id).first()
        if not record:
            raise HTTPException(status_code=404, detail="Building not found")
        props = json.loads(record.properties_json or "{}")
        name = record.name
        lat = float(record.lat)
        lon = float(record.lon)
    finally:
        db.close()

    snapshots = _snapshots_get(props)
    resolved_mode = mode or props.get("active_mode") or "dji"
    if resolved_mode not in SNAPSHOT_MODES:
        raise HTTPException(status_code=400, detail=f"Unknown mode: {resolved_mode!r}")

    snap = snapshots.get(resolved_mode)
    if not snap or not snap.get("response_gz_b64"):
        raise HTTPException(
            status_code=409,
            detail=f"No '{resolved_mode}' snapshot for this building — generate it first.",
        )

    stored_response = _snapshot_decode_response(snap["response_gz_b64"])
    if stored_response is None:
        raise HTTPException(status_code=500, detail="Cached snapshot corrupt")

    # Re-emit as a fresh MissionVersion so downstream endpoints (KMZ download,
    # rewrite-gimbals) have a live version_id to target. Waypoints + building
    # geometry come from stored raw data — no reconstruction needed.
    from ..kmz_import import ParsedWaypoint, SmartObliquePose, waypoints_to_enu
    from ..models import ActionType, CameraAction, CameraName as _CameraName, MissionConfig, Waypoint

    ref_lat = float(props.get("ref_lat", lat))
    ref_lon = float(props.get("ref_lon", lon))
    ref_alt = float(props.get("ref_alt", 0.0))
    waypoints_raw = props.get("waypoints_raw", [])
    parsed_wps = [
        ParsedWaypoint(
            index=int(w.get("index", i)),
            lon=float(w["lon"]), lat=float(w["lat"]),
            alt_egm96=float(w["alt_egm96"]),
            heading_deg=float(w["heading_deg"]),
            gimbal_pitch_deg=float(w["gimbal_pitch_deg"]),
            speed_ms=float(w.get("speed_ms", 2.0)),
            gimbal_yaw_raw_deg=float(w.get("gimbal_yaw_raw_deg", 0.0)),
            gimbal_heading_mode=str(w.get("gimbal_heading_mode", "smoothTransition")),
            gimbal_yaw_base=str(w.get("gimbal_yaw_base", "aircraft")),
            smart_oblique_poses=[
                SmartObliquePose(
                    pitch_deg=float(p.get("pitch", 0.0)),
                    yaw_offset_deg=float(p.get("yaw_offset", 0.0)),
                    roll_deg=float(p.get("roll", 0.0)),
                )
                for p in (w.get("smart_oblique_poses") or [])
            ],
        )
        for i, w in enumerate(waypoints_raw)
    ]
    enu_wps = waypoints_to_enu(parsed_wps, ref_lat, ref_lon, ref_alt)
    waypoints: list[Waypoint] = []
    for i, w in enumerate(enu_wps):
        waypoints.append(Waypoint(
            x=w["x"], y=w["y"], z=w["z"],
            lat=w["lat"], lon=w["lon"], alt=w["alt"],
            heading_deg=w["heading_deg"],
            gimbal_pitch_deg=w["gimbal_pitch_deg"],
            gimbal_yaw_deg=w.get("gimbal_yaw_deg"),
            speed_ms=w["speed_ms"],
            facade_index=-1, index=i,
            actions=[CameraAction(action_type=ActionType.TAKE_PHOTO, camera=_CameraName.WIDE)],
        ))

    # Minimal Building stub — full geometry lives in viewer_data.rawMesh already.
    from ..models import Building
    building_stub = Building(
        lat=ref_lat, lon=ref_lon,
        width=float(props.get("width", 10.0)),
        depth=float(props.get("depth", 10.0)),
        height=float(props.get("auto_height", record.height or 10.0)),
        heading_deg=0.0,
        facades=[], label=name,
    )
    config = MissionConfig(
        flight_speed_ms=2.0,
        mission_name=f"Loaded: {name}",
    )

    version = session.store(
        building_params={
            "lat": ref_lat, "lon": ref_lon,
            "width": building_stub.width, "depth": building_stub.depth,
            "height": building_stub.height, "heading_deg": 0.0,
            "roof_type": "flat", "roof_pitch_deg": 0.0, "num_stories": 1,
        },
        mission_params={
            "target_gsd_mm_per_px": 2.0, "camera": "wide",
            "flight_speed_ms": config.flight_speed_ms,
            "mission_name": config.mission_name,
        },
        building=building_stub, waypoints=waypoints, config=config,
        summary=stored_response.get("summary", {}),
        viewer_data=stored_response.get("viewer_data", {}),
        selection={"building_id": building_id},
    )

    # Swap the freshly-minted version_id into the returned response so the
    # frontend's version tracking stays coherent.
    response = dict(stored_response)
    response["version_id"] = version.version_id
    response["timestamp"] = version.timestamp
    config_snap = dict(response.get("config_snapshot", {}))
    config_snap["building_id"] = building_id
    response["config_snapshot"] = config_snap
    response["building_id"] = building_id

    # Persist the newly-active mode so next reload defaults to it.
    if props.get("active_mode") != resolved_mode:
        db2 = get_db()
        try:
            rec = db2.query(BuildingRecord).filter_by(id=building_id).first()
            if rec is not None:
                p2 = json.loads(rec.properties_json or "{}")
                p2["active_mode"] = resolved_mode
                rec.properties_json = json.dumps(p2)
                db2.commit()
        finally:
            db2.close()

    return {
        "result": response,
        "settings": snap.get("settings") or {},
        "mode": resolved_mode,
        "available_modes": sorted(m for m in snapshots if snapshots[m].get("response_gz_b64")),
    }


@router.delete("/buildings/{building_id}")
def delete_building(building_id: str):
    """Delete an uploaded building."""
    db = get_db()
    try:
        record = db.query(BuildingRecord).filter_by(id=building_id).first()
        if not record:
            raise HTTPException(status_code=404, detail="Building not found")
        db.delete(record)
        db.commit()
        _facade_cache_invalidate(building_id)
        _kmz_mesh_cache_invalidate(building_id)
        _kmz_facade_runtime_cache_invalidate(building_id)
        return {"deleted": building_id}
    finally:
        db.close()


# --- Existing endpoints ---


@router.get("/config")
def get_config():
    """Return all configurable parameters with defaults and ranges."""
    algo_defaults = AlgorithmParams()
    algo_schema = AlgorithmParams.model_json_schema()
    algo_fields = {}
    for name, prop in algo_schema.get("properties", {}).items():
        algo_fields[name] = {
            "default": getattr(algo_defaults, name),
            "min": prop.get("minimum") if "minimum" in prop else prop.get("exclusiveMinimum"),
            "max": prop.get("maximum") if "maximum" in prop else prop.get("exclusiveMaximum"),
            "type": prop.get("type", "number"),
        }
    return {"algorithm": algo_fields}


@router.post("/benchmark-tsp")
def benchmark_tsp(request: GenerateRequest):
    """Run all TSP methods on the same building and return comparison."""
    algo_base = request.algorithm.to_algorithm_config()

    # Build the building (same logic as /generate)
    if request.building_id:
        db = get_db()
        try:
            record = db.query(BuildingRecord).filter_by(id=request.building_id).first()
            if not record:
                raise HTTPException(status_code=404, detail="Building not found")
            props = json.loads(record.properties_json) if record.properties_json else {}
            raw_mesh = props.get("mesh_viewer")
            if raw_mesh:
                from ..building_import import extract_facades
                import trimesh
                import numpy as np
                positions, indices = decode_mesh_viewer_data(raw_mesh)
                # process=False: the mesh was already cleaned (fix_normals,
                # merge_vertices) at upload time. Skipping the default pipeline
                # saves a non-trivial amount of work for large meshes.
                mesh_obj = trimesh.Trimesh(vertices=positions, faces=indices, process=False)
                facades = extract_facades(mesh_obj, method=request.extraction_method, min_area_m2=request.min_facade_area, algo=algo_base)
                if facades:
                    xs = [float(v[0]) for f in facades for v in f.vertices]
                    ys = [float(v[1]) for f in facades for v in f.vertices]
                    from ..models import Building as BuildingModel
                    building = BuildingModel(
                        lat=record.lat, lon=record.lon,
                        width=round(max(xs) - min(xs), 1), depth=round(max(ys) - min(ys), 1),
                        height=record.height, facades=facades, label=record.name,
                    )
                else:
                    geojson = json.loads(record.geometry_data)
                    building = build_building_from_geojson(geojson, height=record.height, name=record.name)
            else:
                geojson = json.loads(record.geometry_data)
                building = build_building_from_geojson(geojson, height=request.building.height, name=record.name)
        finally:
            db.close()
    elif request.preset and request.preset in PRESET_MAP:
        building = PRESET_MAP[request.preset](lat=request.building.lat, lon=request.building.lon)
    else:
        building = build_rectangular_building(
            lat=request.building.lat, lon=request.building.lon,
            width=request.building.width, depth=request.building.depth,
            height=request.building.height, heading_deg=request.building.heading_deg,
            roof_type=RoofType(request.building.roof_type),
            roof_pitch_deg=request.building.roof_pitch_deg, num_stories=request.building.num_stories,
        )

    camera_name = CameraName(request.mission.camera)
    config = MissionConfig(
        target_gsd_mm_per_px=request.mission.target_gsd_mm_per_px,
        camera=camera_name,
        front_overlap=request.mission.front_overlap,
        side_overlap=request.mission.side_overlap,
        flight_speed_ms=request.mission.flight_speed_ms,
        obstacle_clearance_m=request.mission.obstacle_clearance_m,
        gimbal_pitch_margin_deg=request.mission.gimbal_pitch_margin_deg,
        min_photo_distance_m=request.mission.min_photo_distance_m,
        yaw_rate_deg_per_s=request.mission.yaw_rate_deg_per_s,
        gimbal_dedup_threshold_deg=request.mission.gimbal_dedup_threshold_deg,
        heading_dedup_threshold_deg=request.mission.heading_dedup_threshold_deg,
    )

    methods = ["nearest_neighbor", "greedy", "simulated_annealing", "threshold_accepting", "auto"]
    results = []

    for method in methods:
        algo = AlgorithmConfig(**{**algo_base.__dict__, "tsp_method": method})
        t0 = time.perf_counter()
        wps, stats = generate_mission_waypoints(building, config, algo)
        t1 = time.perf_counter()
        opt = stats.get("optimization", {})
        results.append({
            "method": method,
            "time_ms": round((t1 - t0) * 1000, 1),
            "waypoints": len(wps),
            "transit_before_m": round(opt.get("transit_distance_before_m", 0), 1),
            "transit_after_m": round(opt.get("transit_distance_after_m", 0), 1),
            "transit_saved_m": round(opt.get("transit_saved_m", 0), 1),
            "facades_reversed": len(opt.get("facades_reversed", [])),
            "merged": opt.get("waypoints_merged", 0),
        })

    # Sort by transit_after_m ascending
    results.sort(key=lambda r: r["transit_after_m"])

    return {"benchmark": results, "facade_count": len(building.facades)}


@router.get("/presets")
def get_presets():
    return {"presets": PRESET_DEFAULTS}


@router.get("/drone")
def get_drone():
    """Return DJI Matrice 4E hardware specifications."""
    cameras = {}
    for name, spec in CAMERAS.items():
        cameras[name.value] = {
            "focal_length_mm": spec.focal_length_mm,
            "sensor_width_mm": spec.sensor_width_mm,
            "sensor_height_mm": spec.sensor_height_mm,
            "image_width_px": spec.image_width_px,
            "image_height_px": spec.image_height_px,
            "fov_deg": spec.fov_deg,
            "min_interval_s": spec.min_interval_s,
        }
    return {
        "name": "DJI Matrice 4E",
        "cameras": cameras,
        "gimbal": {
            "tilt_min_deg": GIMBAL_TILT_MIN_DEG,
            "tilt_max_deg": GIMBAL_TILT_MAX_DEG,
            "pan_min_deg": GIMBAL_PAN_MIN_DEG,
            "pan_max_deg": GIMBAL_PAN_MAX_DEG,
        },
        "flight": {
            "max_speed_ms": MAX_SPEED_MS,
            "inspection_speed_ms": INSPECTION_SPEED_MS,
            "min_altitude_m": MIN_ALTITUDE_M,
            "max_waypoints": MAX_WAYPOINTS_PER_MISSION,
            "max_flight_time_manifold_min": MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN,
        },
    }


@router.post("/generate")
def generate(request: GenerateRequest):
    t_start = time.perf_counter()

    algo = request.algorithm.to_algorithm_config()

    # Build the building from one of three sources: uploaded, preset, or custom box
    raw_mesh = None  # raw 3D mesh data for viewer (mesh uploads only)
    mesh_obj = None  # trimesh object for LOS occlusion checks
    kmz_source_type: str | None = None  # set to "kmz_raw"/"kmz_import" for KMZ buildings
    kmz_mission_area_wgs84: list | None = None
    kmz_ref_lat: float = 0.0
    kmz_ref_lon: float = 0.0
    kmz_ref_alt: float = 0.0

    if request.building_id:
        db = get_db()
        try:
            # Targeted fetch: only the columns we need + the mesh_viewer sub-document.
            # Parsing the full properties_json is expensive (can contain multi-MB
            # ply_b64 / pc_b64 blobs from KMZ imports); json_extract lets SQLite
            # slice out just the mesh_viewer object before we ever touch Python.
            row = (
                db.query(
                    BuildingRecord.id,
                    BuildingRecord.name,
                    BuildingRecord.lat,
                    BuildingRecord.lon,
                    BuildingRecord.height,
                    BuildingRecord.num_stories,
                    BuildingRecord.roof_type,
                    BuildingRecord.roof_pitch_deg,
                    BuildingRecord.source_type,
                    BuildingRecord.geometry_data,
                    func.json_extract(BuildingRecord.properties_json, "$.mesh_viewer").label("mesh_viewer_str"),
                    func.json_extract(BuildingRecord.properties_json, "$.mission_area_wgs84").label("mission_area_str"),
                    func.json_extract(BuildingRecord.properties_json, "$.ref_lat").label("ref_lat"),
                    func.json_extract(BuildingRecord.properties_json, "$.ref_lon").label("ref_lon"),
                    func.json_extract(BuildingRecord.properties_json, "$.ref_alt").label("ref_alt"),
                )
                .filter(BuildingRecord.id == request.building_id)
                .first()
            )
            if not row:
                raise HTTPException(status_code=404, detail="Building not found")

            raw_mesh = json.loads(row.mesh_viewer_str) if row.mesh_viewer_str else None
            if row.source_type and row.source_type.startswith("kmz_"):
                kmz_source_type = row.source_type
                kmz_mission_area_wgs84 = json.loads(row.mission_area_str) if row.mission_area_str else None
                kmz_ref_lat = float(row.ref_lat if row.ref_lat is not None else row.lat)
                kmz_ref_lon = float(row.ref_lon if row.ref_lon is not None else row.lon)
                kmz_ref_alt = float(row.ref_alt if row.ref_alt is not None else 0.0)

            if raw_mesh:
                # Reconstruct mesh from stored vertices/indices and re-run
                # facet detection with the requested min_facade_area threshold.
                # Only the facades list is cached (expensive); the trimesh.Trimesh
                # is re-decoded fresh per request — trimesh's ray/rtree internals
                # are not thread-safe across concurrent /generate calls.
                from ..building_import import extract_facades
                import trimesh
                import numpy as np

                positions, indices = decode_mesh_viewer_data(raw_mesh)
                # process=False: the mesh was already cleaned (fix_normals,
                # merge_vertices) at upload time. Skipping the default pipeline
                # saves a non-trivial amount of work for large meshes.
                mesh_obj = trimesh.Trimesh(vertices=positions, faces=indices, process=False)

                cache_key = _facade_cache_key(row.id, request.extraction_method, request.min_facade_area, algo)
                cached = _facade_cache_get(cache_key)
                if cached is not None:
                    facades, bbox = cached
                    width, depth = bbox[0], bbox[1]
                else:
                    facades = extract_facades(mesh_obj, method=request.extraction_method, min_area_m2=request.min_facade_area, algo=algo)
                    xs = [float(v[0]) for f in facades for v in f.vertices] if facades else []
                    ys = [float(v[1]) for f in facades for v in f.vertices] if facades else []
                    width = round(max(xs) - min(xs), 1) if xs else 0
                    depth = round(max(ys) - min(ys), 1) if ys else 0
                    _facade_cache_put(cache_key, (facades, (width, depth, 0.0, 0.0)))

                if facades:
                    from ..models import Building as BuildingModel
                    building = BuildingModel(
                        lat=row.lat,
                        lon=row.lon,
                        width=width,
                        depth=depth,
                        height=row.height,
                        facades=facades,
                        label=row.name,
                    )
                else:
                    # Fallback to GeoJSON footprint
                    geojson = json.loads(row.geometry_data)
                    building = build_building_from_geojson(geojson, height=row.height, name=row.name)
            else:
                # GeoJSON-only building (no mesh data).
                # Raw KMZ imports may have a degenerate polygon (saved before
                # the footprint fallback fix landed). Reject with a clear
                # message instead of bubbling a generic ValueError — the user
                # should either re-import the KMZ in facades mode or re-upload
                # so the new fallback synthesises a valid footprint.
                geojson = json.loads(row.geometry_data)
                try:
                    coords = geojson.get("geometry", {}).get("coordinates", [[]])
                    ring = coords[0] if coords else []
                    if len(ring) > 1 and ring[0] == ring[-1]:
                        ring = ring[:-1]
                    if len(ring) < 3:
                        raise HTTPException(
                            status_code=400,
                            detail=(
                                "Building has no inspectable footprint. This is a raw KMZ "
                                "import with no facade data — either re-import in 'facades' "
                                "mode, or re-upload so a footprint can be derived from the "
                                "DJI mission area."
                            ),
                        )
                except HTTPException:
                    raise
                except Exception:
                    pass
                building = build_building_from_geojson(
                    geojson,
                    height=request.building.height,
                    num_stories=request.building.num_stories,
                    roof_type=request.building.roof_type,
                    roof_pitch_deg=request.building.roof_pitch_deg,
                    name=row.name,
                )
        finally:
            db.close()
    elif request.preset and request.preset in PRESET_MAP:
        building = PRESET_MAP[request.preset](
            lat=request.building.lat,
            lon=request.building.lon,
        )
    else:
        building = build_rectangular_building(
            lat=request.building.lat,
            lon=request.building.lon,
            width=request.building.width,
            depth=request.building.depth,
            height=request.building.height,
            heading_deg=request.building.heading_deg,
            roof_type=RoofType(request.building.roof_type),
            roof_pitch_deg=request.building.roof_pitch_deg,
            num_stories=request.building.num_stories,
        )

    # Build the mission config
    camera_name = CameraName(request.mission.camera)
    config = MissionConfig(
        target_gsd_mm_per_px=request.mission.target_gsd_mm_per_px,
        camera=camera_name,
        front_overlap=request.mission.front_overlap,
        side_overlap=request.mission.side_overlap,
        flight_speed_ms=request.mission.flight_speed_ms,
        obstacle_clearance_m=request.mission.obstacle_clearance_m,
        mission_name=request.mission.mission_name,
        gimbal_pitch_margin_deg=request.mission.gimbal_pitch_margin_deg,
        min_photo_distance_m=request.mission.min_photo_distance_m,
        yaw_rate_deg_per_s=request.mission.yaw_rate_deg_per_s,
        stop_at_waypoint=request.mission.stop_at_waypoint,
        gimbal_dedup_threshold_deg=request.mission.gimbal_dedup_threshold_deg,
        heading_dedup_threshold_deg=request.mission.heading_dedup_threshold_deg,
        rc_lost_action=request.mission.rc_lost_action,
        finish_action=request.mission.finish_action,
        takeoff_security_height_m=request.mission.takeoff_security_height_m,
    )

    t_building = time.perf_counter()

    # KMZ buildings: restrict planning to facades inside DJI's declared mapped
    # volume (3D-Tiles tileset.json Mapping OBB). Runs before candidate-facade
    # enablement so the filter applies to the extracted set only — a user can
    # still force-enable a candidate outside the OBB if they want.
    if kmz_source_type is not None:
        building.facades = _filter_facades_by_dji_bbox(
            building.facades,
            kmz_mission_area_wgs84,
            kmz_ref_lat, kmz_ref_lon, kmz_ref_alt,
        )

    # Append enabled candidate facades (rejected during extraction but force-included by user)
    if request.enabled_candidates:
        from ..building_import import last_rejected_candidates
        enabled_set = set(request.enabled_candidates)
        next_idx = max((f.index for f in building.facades), default=-1) + 1
        for candidate in last_rejected_candidates:
            if candidate.index in enabled_set:
                from copy import deepcopy
                added = deepcopy(candidate)
                added.index = next_idx
                added.label = added.label.replace("candidate_", "added_")
                building.facades.append(added)
                next_idx += 1

    # Convert exclusion zone params to domain objects
    zones = [
        ExclusionZone(
            id=z.id, label=z.label,
            center_x=z.center_x, center_y=z.center_y, center_z=z.center_z,
            size_x=z.size_x, size_y=z.size_y, size_z=z.size_z,
            zone_type=z.zone_type,
            polygon_vertices=z.polygon_vertices,
        )
        for z in request.exclusion_zones
    ]

    # Generate waypoints (pass mesh for LOS checks / surface sampling on mesh buildings)
    waypoints, generation_stats = generate_mission_waypoints(
        building, config, algo, mesh=mesh_obj,
        waypoint_strategy=request.waypoint_strategy,
        disabled_facades=request.disabled_facades,
        exclusion_zones=zones,
    )

    t_waypoints = time.perf_counter()

    # Compute summary with path metrics
    camera_spec = get_camera(camera_name)
    distance = compute_distance_for_gsd(camera_spec, config.target_gsd_mm_per_px)
    footprint = compute_footprint(camera_spec, distance)

    n_inspection = sum(1 for wp in waypoints if not wp.is_transition)
    n_transition = sum(1 for wp in waypoints if wp.is_transition)
    n_photos = sum(
        sum(1 for a in wp.actions if a.action_type.value == "takePhoto")
        for wp in waypoints
    )

    # Compute total path length and per-segment distances
    total_path_m = 0.0
    transitions = []
    prev_facade_idx = -1
    for i in range(1, len(waypoints)):
        dx = waypoints[i].x - waypoints[i - 1].x
        dy = waypoints[i].y - waypoints[i - 1].y
        dz = waypoints[i].z - waypoints[i - 1].z
        seg_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        total_path_m += seg_dist

        # Track facade transitions
        if waypoints[i].facade_index != prev_facade_idx and not waypoints[i].is_transition:
            if prev_facade_idx >= 0:
                heading_change = abs(waypoints[i].heading_deg - waypoints[i - 1].heading_deg)
                if heading_change > 180:
                    heading_change = 360 - heading_change
                transitions.append({
                    "from_facade": prev_facade_idx,
                    "to_facade": waypoints[i].facade_index,
                    "heading_change_deg": round(heading_change),
                })
            prev_facade_idx = waypoints[i].facade_index

    # Estimate flight time (inspection WPs at inspection speed, transit at transit speed)
    est_time_s = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i].x - waypoints[i - 1].x
        dy = waypoints[i].y - waypoints[i - 1].y
        dz = waypoints[i].z - waypoints[i - 1].z
        seg_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = waypoints[i].speed_ms or config.flight_speed_ms
        est_time_s += seg_dist / speed
        if not waypoints[i].is_transition:
            est_time_s += algo.hover_time_per_wp_s

    # Per-facade waypoint counts
    facade_wp_counts = {}
    for wp in waypoints:
        if not wp.is_transition:
            facade_wp_counts[wp.facade_index] = facade_wp_counts.get(wp.facade_index, 0) + 1

    # Horizontal and vertical FOV from sensor geometry
    h_fov_deg = round(2 * math.degrees(math.atan(camera_spec.sensor_width_mm / (2 * camera_spec.focal_length_mm))), 1)
    v_fov_deg = round(2 * math.degrees(math.atan(camera_spec.sensor_height_mm / (2 * camera_spec.focal_length_mm))), 1)

    summary = {
        "waypoint_count": len(waypoints),
        "inspection_waypoints": n_inspection,
        "transition_waypoints": n_transition,
        "photo_count": n_photos,
        "facade_count": len(building.facades),
        "camera_distance_m": round(distance, 1),
        "photo_footprint_m": [round(footprint.width_m, 1), round(footprint.height_m, 1)],
        "total_path_m": round(total_path_m, 1),
        "estimated_flight_time_s": round(est_time_s),
        "transitions": transitions,
        "facade_waypoint_counts": facade_wp_counts,
        "camera": {
            "name": camera_name.value,
            "fov_h_deg": h_fov_deg,
            "fov_v_deg": v_fov_deg,
            "distance_m": round(distance, 2),
            "focal_length_mm": camera_spec.focal_length_mm,
        },
        "facade_coverage": compute_facade_coverage(building, waypoints),
    }
    # Stamp the original KMZ source type so the sidebar keeps the DJI/inspection
    # mode toggle visible after regeneration.
    if kmz_source_type is not None:
        summary["source"] = kmz_source_type

    t_summary = time.perf_counter()

    # Validate mission against hardware constraints
    validation = validate_mission(
        waypoints, config, building, algo,
        exclusion_zones=zones, generation_stats=generation_stats,
    )
    validation_data = [
        {"severity": v.severity, "code": v.code, "message": v.message,
         "waypoint_indices": v.waypoint_indices, "facade_index": v.facade_index}
        for v in validation
    ]
    has_errors = any(v.severity == "error" for v in validation)

    # Prepare viewer data (include rejected facade candidates for manual enabling)
    from ..building_import import last_rejected_candidates
    threejs_data = prepare_threejs_data(
        building, waypoints,
        candidate_facades=last_rejected_candidates,
    )
    if raw_mesh:
        # Forward the already-gzipped+base64 mesh blobs directly when present:
        # decoding to Python lists and re-serializing as JSON costs ~1 GB of
        # intermediate Python objects for a 100k-face mesh, and the frontend
        # has to parse all that JSON back. The browser can gunzip the base64
        # payload to a Float32Array natively via DecompressionStream.
        if "positions_b64" in raw_mesh and "indices_b64" in raw_mesh:
            threejs_data["rawMesh"] = {
                "positions_b64": raw_mesh["positions_b64"],
                "indices_b64": raw_mesh["indices_b64"],
                "n_vertices": raw_mesh.get("n_vertices"),
                "n_faces": raw_mesh.get("n_faces"),
            }
        elif "positions" in raw_mesh and "indices" in raw_mesh:
            threejs_data["rawMesh"] = {"positions": raw_mesh["positions"], "indices": raw_mesh["indices"]}

    viewer_data = {
        "threejs": threejs_data,
        "leaflet": prepare_leaflet_data(building, waypoints),
    }

    # Store version
    version = session.store(
        building_params=request.building.model_dump(),
        mission_params=request.mission.model_dump(),
        building=building,
        waypoints=waypoints,
        config=config,
        summary=summary,
        viewer_data=viewer_data,
        algo=algo,
        selection={
            "building_id": request.building_id,
            "disabled_facades": request.disabled_facades,
            "enabled_candidates": request.enabled_candidates,
            "exclusion_zones": [z.model_dump() for z in request.exclusion_zones],
        },
    )

    t_end = time.perf_counter()

    # Extraction stats from mesh import (if applicable)
    from ..building_import import last_extraction_stats
    extraction = dict(last_extraction_stats) if last_extraction_stats else None

    perf = {
        "total_ms": round((t_end - t_start) * 1000, 1),
        "building_ms": round((t_building - t_start) * 1000, 1),
        "waypoints_ms": round((t_waypoints - t_building) * 1000, 1),
        "summary_ms": round((t_summary - t_waypoints) * 1000, 1),
        "validate_ms": round((t_end - t_summary) * 1000, 1),
        "generation": generation_stats,
        "extraction": extraction,
        "validation_counts": {
            "errors": sum(1 for v in validation if v.severity == "error"),
            "warnings": sum(1 for v in validation if v.severity == "warning"),
            "info": sum(1 for v in validation if v.severity == "info"),
        },
    }

    return {
        "version_id": version.version_id,
        "timestamp": version.timestamp,
        "summary": summary,
        "viewer_data": viewer_data,
        "validation": validation_data,
        "can_export": not has_errors,
        "perf": perf,
        "config_snapshot": {
            "building": request.building.model_dump(),
            "mission": request.mission.model_dump(),
            "algorithm": request.algorithm.model_dump(),
            "building_id": request.building_id,
            "preset": request.preset,
            "disabled_facades": request.disabled_facades,
            "enabled_candidates": request.enabled_candidates,
            "exclusion_zones": [z.model_dump() for z in request.exclusion_zones],
        },
    }


@router.get("/versions")
def list_versions():
    return {"versions": session.list_versions()}


@router.get("/versions/{version_id}")
def get_version(version_id: str):
    version = session.get(version_id)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")
    return {
        "version_id": version.version_id,
        "timestamp": version.timestamp,
        "summary": version.summary,
        "viewer_data": version.viewer_data,
        "config_snapshot": {
            "building": version.building_params,
            "mission": version.mission_params,
            **(version.selection or {}),
        },
    }


@router.get("/versions/{version_id}/kmz")
def download_kmz(version_id: str):
    version = session.get(version_id)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")

    kmz_bytes = build_kmz_bytes(version.waypoints, version.config, version.algo)
    filename = f"{version.config.mission_name.replace(' ', '_')}_{version_id}.kmz"

    return Response(
        content=kmz_bytes,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


class RewriteGimbalsRequest(BaseModel):
    max_distance_m: float = Field(default=60.0, gt=0.0, le=500.0)
    pitch_margin_deg: float = Field(default=2.0, ge=0.0, le=20.0)
    preserve_heading: bool = True
    # If False, the perpendicular gimbal rewrite is skipped entirely — useful
    # when the DJI gimbals already point correctly and the caller only wants
    # to strip the SmartOblique rosette + cap speed.
    rewrite_angles: bool = True
    # NEN-2767 enrichment: cap flight speed + collapse SmartOblique rosette to
    # a single per-waypoint photo. Null/False means keep the DJI defaults.
    flight_speed_ms: float | None = Field(default=3.0, ge=0.5, le=15.0)
    strip_smart_oblique: bool = True


@router.post("/versions/{version_id}/rewrite-gimbals")
def rewrite_gimbals(version_id: str, req: RewriteGimbalsRequest | None = None):
    """Rewrite gimbal angles for every waypoint in the given version so the
    camera is perpendicular to the nearest outward-facing facade.

    Produces a brand-new MissionVersion (the original is kept). Requires the
    source version to have facades — raw KMZ imports must go through facade
    extraction first.
    """
    from dataclasses import replace as _dc_replace

    from ..gimbal_rewrite import rewrite_gimbals_perpendicular

    version = session.get(version_id)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")

    params = req or RewriteGimbalsRequest()
    # Facades are only required when rewriting angles. Strip-rosette-only
    # (rewrite_angles=False) just clones the waypoints and edits actions/speed,
    # so a raw-KMZ version without extracted facades is fine.
    if params.rewrite_angles and not version.building.facades:
        raise HTTPException(
            status_code=400,
            detail="Source version has no facades. Run facade extraction first "
                   "(re-import the KMZ in Facades mode, or run /generate).",
        )
    if params.rewrite_angles:
        new_waypoints = rewrite_gimbals_perpendicular(
            waypoints=version.waypoints,
            facades=version.building.facades,
            max_distance_m=params.max_distance_m,
            pitch_margin_deg=params.pitch_margin_deg,
            preserve_heading=params.preserve_heading,
        )
    else:
        # Preserve DJI gimbals exactly — just clone the waypoints so the
        # downstream speed/rosette edits produce a new version.
        new_waypoints = [_dc_replace(w) for w in version.waypoints]

    # NEN-2767 enrichment: override speed + strip SmartOblique rosette so every
    # waypoint captures a single perpendicular photo. Trajectory stays DJI's.
    if params.flight_speed_ms is not None or params.strip_smart_oblique:
        from ..models import ActionType as _ActionType, CameraAction as _CameraAction, CameraName as _CameraName
        for w in new_waypoints:
            if params.flight_speed_ms is not None:
                w.speed_ms = float(params.flight_speed_ms)
            if params.strip_smart_oblique:
                w.actions = [_CameraAction(action_type=_ActionType.TAKE_PHOTO, camera=_CameraName.WIDE)]

    threejs_data = prepare_threejs_data(version.building, new_waypoints)
    # Preserve the original point cloud + mission area if they were attached.
    src = version.viewer_data.get("threejs", {}) if isinstance(version.viewer_data, dict) else {}
    for key in ("pointCloud", "missionArea", "rawMesh"):
        if key in src:
            threejs_data[key] = src[key]

    # Area polygon for Leaflet: reuse the source if it's in the leaflet view.
    leaflet_data = prepare_leaflet_data(version.building, new_waypoints)
    src_leaflet = version.viewer_data.get("leaflet", {}) if isinstance(version.viewer_data, dict) else {}
    if "missionArea" in src_leaflet:
        leaflet_data["missionArea"] = src_leaflet["missionArea"]

    def _gimbal_stats(wps):
        pitches = [float(w.gimbal_pitch_deg) for w in wps]
        yaws = [float(w.gimbal_yaw_deg) if w.gimbal_yaw_deg is not None else float(w.heading_deg) for w in wps]
        if not pitches:
            return {"count": 0}
        pitches_s = sorted(pitches)
        return {
            "count": len(pitches),
            "pitch_mean": round(sum(pitches) / len(pitches), 2),
            "pitch_min": round(min(pitches), 2),
            "pitch_max": round(max(pitches), 2),
            "pitch_median": round(pitches_s[len(pitches_s) // 2], 2),
            "yaw_unique": len(set(round(y, 1) for y in yaws)),
        }

    before_stats = _gimbal_stats(version.waypoints)
    after_stats = _gimbal_stats(new_waypoints)

    # Per-waypoint before/after so the UI can render a diff table and render
    # ghost arrows in the 3D viewer pointing the old camera direction.
    def _yaw(w):
        return float(w.gimbal_yaw_deg) if w.gimbal_yaw_deg is not None else float(w.heading_deg)

    gimbal_diff: list[dict] = []
    n = min(len(version.waypoints), len(new_waypoints))
    for i in range(n):
        b = version.waypoints[i]
        a = new_waypoints[i]
        gimbal_diff.append({
            "index": int(a.index),
            "pitch_before": round(float(b.gimbal_pitch_deg), 1),
            "pitch_after": round(float(a.gimbal_pitch_deg), 1),
            "yaw_before": round(_yaw(b), 1),
            "yaw_after": round(_yaw(a), 1),
            "facade_index": int(a.facade_index),
        })

    # Attach before-angles to the viewer waypoint list so Viewer3D can render
    # ghost arrows at each waypoint pointing the original camera direction.
    try:
        wp_data = threejs_data.get("waypoints", []) if isinstance(threejs_data, dict) else []
        for i, wd in enumerate(wp_data):
            if i < len(gimbal_diff):
                wd["pitch_before"] = gimbal_diff[i]["pitch_before"]
                wd["yaw_before"] = gimbal_diff[i]["yaw_before"]
    except Exception:
        pass

    updated_summary = dict(version.summary)
    updated_summary["source"] = "kmz_gimbal_rewrite"
    updated_summary["parent_version_id"] = version.version_id
    updated_summary["gimbal_before"] = before_stats
    updated_summary["gimbal_after"] = after_stats
    updated_summary["gimbal_diff"] = gimbal_diff
    updated_summary["facade_coverage"] = compute_facade_coverage(version.building, new_waypoints)

    new_version = session.store(
        building_params=version.building_params,
        mission_params={**version.mission_params, "mission_name": f"NEN-2767: {version.config.mission_name}"},
        building=version.building,
        waypoints=new_waypoints,
        config=_dc_replace(version.config, mission_name=f"NEN-2767: {version.config.mission_name}"),
        summary=updated_summary,
        viewer_data={"threejs": threejs_data, "leaflet": leaflet_data},
        algo=version.algo,
        selection=version.selection,
    )

    return {
        "version_id": new_version.version_id,
        "parent_version_id": version.version_id,
        "rewritten_count": sum(1 for w in new_waypoints if w.facade_index >= 0),
        "total_waypoints": len(new_waypoints),
        "timestamp": new_version.timestamp,
        "summary": updated_summary,
        "viewer_data": new_version.viewer_data,
    }


@router.delete("/versions/{version_id}")
def delete_version(version_id: str):
    if not session.delete(version_id):
        raise HTTPException(status_code=404, detail="Version not found")
    return {"deleted": version_id}


@router.delete("/versions")
def delete_all_versions():
    count = session.clear()
    return {"deleted": count}


@router.get("/analyze/{version_id}")
def analyze_version(version_id: str):
    """Analyze a generated mission for quality issues.

    Returns structured diagnostics about waypoint quality, path efficiency,
    coverage gaps, and potential improvements. Designed for dev tooling.
    """
    version = session.get(version_id)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")

    waypoints = version.waypoints
    building = version.building
    config = version.config

    issues: list[dict] = []
    stats: dict = {}

    # --- 1. Duplicate / near-duplicate waypoints ---
    near_dupes = []
    for i in range(len(waypoints)):
        for j in range(i + 1, min(i + 5, len(waypoints))):  # check nearby in sequence
            a, b = waypoints[i], waypoints[j]
            if a.is_transition or b.is_transition:
                continue
            dist = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
            if dist < 0.5:
                near_dupes.append({
                    "wp_a": i, "wp_b": j, "distance_m": round(dist, 2),
                    "same_facade": a.facade_index == b.facade_index,
                    "heading_diff": round(abs(a.heading_deg - b.heading_deg) % 360, 1),
                })
    if near_dupes:
        issues.append({
            "type": "near_duplicate_waypoints",
            "severity": "warning",
            "count": len(near_dupes),
            "message": f"{len(near_dupes)} waypoint pairs within 0.5m of each other",
            "details": near_dupes[:10],
        })

    # --- 2. Heading vs gimbal misalignment ---
    # The camera arrow should point toward the facade. If heading and gimbal
    # direction don't align with the facade normal, the photo won't be perpendicular.
    misaligned = []
    for wp in waypoints:
        if wp.is_transition or wp.facade_index < 0:
            continue
        if wp.facade_index >= len(building.facades):
            continue
        facade = building.facades[wp.facade_index]
        # Expected heading: opposite of facade normal
        expected_heading = math.degrees(math.atan2(facade.normal[0], facade.normal[1])) % 360
        expected_heading = (expected_heading + 180) % 360
        heading_diff = abs(wp.heading_deg - expected_heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        if heading_diff > 30:
            misaligned.append({
                "wp": wp.index, "facade": wp.facade_index,
                "actual_heading": round(wp.heading_deg, 1),
                "expected_heading": round(expected_heading, 1),
                "diff_deg": round(heading_diff, 1),
            })
    if misaligned:
        issues.append({
            "type": "heading_misalignment",
            "severity": "warning",
            "count": len(misaligned),
            "message": f"{len(misaligned)} waypoints with heading >30° off from facade normal",
            "details": misaligned[:10],
        })

    # --- 3. Back-and-forth detection ---
    # Find sequences where the drone reverses direction (angle > 150°)
    reversals = []
    for i in range(2, len(waypoints)):
        a, b, c = waypoints[i - 2], waypoints[i - 1], waypoints[i]
        if a.is_transition or b.is_transition or c.is_transition:
            continue
        dx1, dy1 = b.x - a.x, b.y - a.y
        dx2, dy2 = c.x - b.x, c.y - b.y
        len1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
        len2 = math.sqrt(dx2 * dx2 + dy2 * dy2)
        if len1 < 0.1 or len2 < 0.1:
            continue
        cos_angle = (dx1 * dx2 + dy1 * dy2) / (len1 * len2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.degrees(math.acos(cos_angle))
        if angle > 150:
            reversals.append({
                "wp": i - 1, "angle_deg": round(angle, 1),
                "facade": b.facade_index,
            })
    if reversals:
        issues.append({
            "type": "path_reversal",
            "severity": "info",
            "count": len(reversals),
            "message": f"{len(reversals)} sharp reversals (>150°) in flight path",
            "details": reversals[:10],
        })

    # --- 4. Facade coverage analysis ---
    facade_coverage = {}
    for f in building.facades:
        wps_on_facade = [wp for wp in waypoints if wp.facade_index == f.index and not wp.is_transition]
        facade_coverage[f.index] = {
            "label": f.label,
            "component": f.component_tag,
            "area_m2": round(f.width * f.height, 1),
            "waypoint_count": len(wps_on_facade),
            "covered": len(wps_on_facade) > 0,
        }
    uncovered = [fc for fc in facade_coverage.values() if not fc["covered"]]
    if uncovered:
        issues.append({
            "type": "uncovered_facades",
            "severity": "warning",
            "count": len(uncovered),
            "message": f"{len(uncovered)} facades have no waypoints (not inspected)",
            "details": uncovered,
        })

    # --- 5. Flight efficiency ---
    inspection_wps = [wp for wp in waypoints if not wp.is_transition]
    transition_wps = [wp for wp in waypoints if wp.is_transition]
    total_dist = sum(
        math.sqrt(
            (waypoints[i].x - waypoints[i - 1].x) ** 2 +
            (waypoints[i].y - waypoints[i - 1].y) ** 2 +
            (waypoints[i].z - waypoints[i - 1].z) ** 2
        )
        for i in range(1, len(waypoints))
    )
    inspect_dist = 0.0
    transit_dist = 0.0
    for i in range(1, len(waypoints)):
        d = math.sqrt(
            (waypoints[i].x - waypoints[i - 1].x) ** 2 +
            (waypoints[i].y - waypoints[i - 1].y) ** 2 +
            (waypoints[i].z - waypoints[i - 1].z) ** 2
        )
        if waypoints[i].is_transition or waypoints[i - 1].is_transition:
            transit_dist += d
        else:
            inspect_dist += d

    stats = {
        "total_waypoints": len(waypoints),
        "inspection_waypoints": len(inspection_wps),
        "transition_waypoints": len(transition_wps),
        "total_path_m": round(total_dist, 1),
        "inspection_path_m": round(inspect_dist, 1),
        "transit_path_m": round(transit_dist, 1),
        "efficiency_pct": round(inspect_dist / total_dist * 100, 1) if total_dist > 0 else 0,
        "facades_total": len(building.facades),
        "facades_covered": sum(1 for fc in facade_coverage.values() if fc["covered"]),
        "facades_uncovered": len(uncovered),
        "facade_coverage": facade_coverage,
    }

    return {
        "version_id": version_id,
        "issues": issues,
        "issue_count": {"warning": sum(1 for i in issues if i["severity"] == "warning"),
                        "info": sum(1 for i in issues if i["severity"] == "info")},
        "stats": stats,
    }


# ---------------------------------------------------------------------------
# Simulate reconstruction
# ---------------------------------------------------------------------------

class SimulateRequest(BaseModel):
    version_id: Optional[str] = None  # uses latest if omitted
    render_scale: float = Field(0.1, ge=0.02, le=0.5)
    voxel_size: float = Field(0.03, ge=0.005, le=0.2)


@router.get("/simulate-reconstruct")
def list_simulations():
    """List all simulation tasks."""
    return {"tasks": list_sim_tasks()}


@router.post("/simulate-reconstruct")
def simulate_reconstruct(request: SimulateRequest):
    """Start a simulated reconstruction from a generated mission.

    Renders synthetic photos from each waypoint, reconstructs via TSDF,
    and reimports the mesh. Runs in background — poll via GET.
    """
    import secrets

    # Resolve version
    vid = request.version_id
    if not vid:
        versions = session.list_versions()
        if not versions:
            raise HTTPException(status_code=400, detail="No mission versions — generate a mission first")
        vid = versions[0]["version_id"]

    version = session.get(vid)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")

    task_id = f"sim_{secrets.token_hex(4)}"

    # Get the raw mesh — the ACTUAL building geometry the drone camera would see.
    # Try three sources in order: viewer_data, config_snapshot building_id → DB, building object.
    import logging
    _log = logging.getLogger(__name__)

    # Resolve building_id once — used for both raw_mesh lookup and KMZ context.
    building_id = None
    if hasattr(version, "building_params"):
        building_id = version.building_params.get("building_id")
    cs = getattr(version, "selection", None) or {}
    if not building_id and isinstance(cs, dict):
        building_id = cs.get("building_id")

    raw_mesh = version.viewer_data.get("threejs", {}).get("rawMesh")
    kmz_context: dict | None = None

    if building_id:
        db = get_db()
        try:
            record = db.query(BuildingRecord).filter_by(id=building_id).first()
            if record and record.properties_json:
                props = json.loads(record.properties_json)
                if not raw_mesh:
                    raw_mesh = props.get("mesh_viewer")
                # KMZ source → re-extract reconstructed facades through the same
                # CGAL + DJI-polygon pipeline that the original import used, so the
                # comparison is apples-to-apples (not "CGAL on points" vs "region
                # growing on mesh", which produces wildly different facade sets).
                if record.source_type and record.source_type.startswith("kmz_"):
                    mission_area = props.get("mission_area_wgs84")
                    if mission_area:
                        # Snapshot per-waypoint Smart3D rosette poses + flight
                        # speed from the original viewer payload. The Waypoint
                        # dataclass doesn't carry these; without re-attaching
                        # them in Phase 4 the simulation viewer would render
                        # plain frustums instead of the rosette overlay.
                        orig_vw = (
                            version.viewer_data.get("threejs", {}).get("waypoints", [])
                        )
                        wp_overlay = [
                            {
                                "smart_oblique_poses": w.get("smart_oblique_poses"),
                                "speed_ms": w.get("speed_ms"),
                            }
                            for w in orig_vw
                        ]
                        kmz_context = {
                            "mission_area_wgs84": mission_area,
                            "ref_lat": float(props.get("ref_lat", record.lat)),
                            "ref_lon": float(props.get("ref_lon", record.lon)),
                            "ref_alt": float(props.get("ref_alt", 0.0)),
                            "waypoint_overlay": wp_overlay,
                        }
        finally:
            db.close()

    if not raw_mesh:
        # Last resort: check building._mesh_viewer_data (set during build_building_from_mesh)
        raw_mesh = getattr(version.building, "_mesh_viewer_data", None)

    if raw_mesh:
        n_verts = len(raw_mesh.get("positions", [])) // 3
        n_faces = len(raw_mesh.get("indices", [])) // 3
        _log.info(f"Simulation: using raw mesh ({n_verts} verts, {n_faces} faces)")
    else:
        _log.info("Simulation: no raw mesh — using facade slab fallback")
    if kmz_context:
        _log.info("Simulation: KMZ source detected — recon will use CGAL extractor + DJI polygon clip")

    start_simulation_async(
        building=version.building,
        waypoints=version.waypoints,
        config=version.config,
        algo=version.algo,
        task_id=task_id,
        version_id=vid,
        render_scale=request.render_scale,
        voxel_size=request.voxel_size,
        raw_mesh=raw_mesh,
        kmz_context=kmz_context,
    )

    return {"task_id": task_id, "status": "started", "source_version": vid}


@router.get("/simulate-reconstruct/{task_id}")
def get_simulation(task_id: str):
    """Poll simulation progress / get results. Checks memory first, then DB."""
    task = get_sim_task(task_id)
    if task:
        resp = {
            "task_id": task.task_id,
            "status": task.status,
            "progress": round(task.progress, 3),
            "message": task.message,
        }
        if task.status == "complete" and task.result:
            resp["result"] = task.result
        elif task.status == "error":
            resp["error"] = task.error
        return resp

    # Not in memory — try database (completed task from a previous session)
    db_result = get_sim_result(task_id)
    if db_result:
        return {
            "task_id": task_id,
            "status": "complete",
            "progress": 1.0,
            "message": "Loaded from database",
            "result": db_result,
        }

    raise HTTPException(status_code=404, detail="Simulation task not found")


@router.get("/simulate-reconstruct/{task_id}/photos")
def get_simulation_photos(task_id: str):
    """Serve the list of rendered photo paths for a simulation."""
    task = get_sim_task(task_id)
    if not task or task.status != "complete" or not task.result:
        raise HTTPException(status_code=404, detail="No completed simulation found")
    return {
        "photos": task.result.get("photos", []),
        "total": task.result.get("photos_total", 0),
        "output_dir": task.result.get("output_dir", ""),
    }


@router.get("/simulate-reconstruct/{task_id}/photo/{wp_index}")
def get_simulation_photo(task_id: str, wp_index: int):
    """Serve a specific rendered photo as PNG."""
    task = get_sim_task(task_id)
    if not task or task.status != "complete" or not task.result:
        raise HTTPException(status_code=404, detail="No completed simulation found")

    output_dir = task.result.get("output_dir", "")
    img_path = Path(output_dir) / "images" / f"wp_{wp_index:04d}.png"
    if not img_path.exists():
        raise HTTPException(status_code=404, detail=f"Photo wp_{wp_index:04d}.png not found")

    return Response(
        content=img_path.read_bytes(),
        media_type="image/png",
        headers={"Content-Disposition": f'inline; filename="wp_{wp_index:04d}.png"'},
    )


@router.delete("/simulate-reconstruct/{task_id}")
def delete_simulation(task_id: str):
    """Delete a simulation task and all its output files."""
    if not delete_sim_task(task_id):
        raise HTTPException(status_code=404, detail="Simulation task not found")
    return {"deleted": task_id}
