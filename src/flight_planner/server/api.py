"""REST API endpoints for the AeroScan debug server."""

from __future__ import annotations

import json
import math
from typing import Literal, Optional

from fastapi import APIRouter, Form, HTTPException, UploadFile
from fastapi.responses import Response
from pydantic import BaseModel, Field

from ..building_import import build_building_from_geojson, build_building_from_mesh
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
    CAMERAS,
    CameraName,
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
from ..visualize import prepare_leaflet_data, prepare_threejs_data
from .database import BuildingRecord, get_db
from .state import session

router = APIRouter()

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
    front_overlap: float = Field(0.80, ge=0.0, le=0.95)
    side_overlap: float = Field(0.70, ge=0.0, le=0.95)
    flight_speed_ms: float = Field(3.0, ge=0.5, le=21.0)
    obstacle_clearance_m: float = Field(2.0, ge=1.0, le=20.0)
    mission_name: str = "AeroScan Inspection"


class GenerateRequest(BaseModel):
    preset: Optional[Literal[
        "simple_box", "pitched_roof_house", "l_shaped_block", "large_apartment_block"
    ]] = None
    building_id: Optional[str] = None
    building: BuildingParams = BuildingParams()
    mission: MissionParams = MissionParams()


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
        properties_json=json.dumps({}),
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
):
    """Upload an OBJ/PLY/STL mesh file and create a building from it."""
    if not file.filename:
        raise HTTPException(status_code=400, detail="No file provided")

    ext = file.filename.rsplit(".", 1)[-1].lower()
    supported = {"obj": "obj", "ply": "ply", "stl": "stl", "glb": "glb", "gltf": "gltf"}
    if ext not in supported:
        raise HTTPException(status_code=400, detail=f"Unsupported format: .{ext}. Use .obj, .ply, or .stl")

    file_data = await file.read()
    build_name = name or file.filename.rsplit(".", 1)[0] or "Mesh building"

    try:
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
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to parse mesh: {e}")

    # Convert the computed footprint back to GeoJSON for storage.
    # This lets the generate endpoint reconstruct the building without
    # needing the original mesh binary.
    import math as _math
    m_per_lat = 111132.92 - 559.82 * _math.cos(2 * _math.radians(building.lat))
    m_per_lon = 111412.84 * _math.cos(_math.radians(building.lat))
    # Extract ground-level wall vertices to rebuild the footprint polygon
    footprint_coords = []
    for facade in building.facades:
        if abs(facade.normal[2]) < 0.01:  # vertical walls only
            v0 = facade.vertices[0]  # first ground vertex
            lon_v = building.lon + v0[0] / m_per_lon
            lat_v = building.lat + v0[1] / m_per_lat
            footprint_coords.append([round(lon_v, 8), round(lat_v, 8)])
    # Close the ring
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
            "mesh_format": ext,
            "auto_height": building.height,
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


@router.get("/buildings")
def list_buildings():
    """List all uploaded buildings."""
    db = get_db()
    try:
        records = db.query(BuildingRecord).order_by(BuildingRecord.created_at.desc()).all()
        return {"buildings": [r.to_dict() for r in records]}
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
        return {"deleted": building_id}
    finally:
        db.close()


# --- Existing endpoints ---


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
    # Build the building from one of three sources: uploaded, preset, or custom box
    if request.building_id:
        db = get_db()
        try:
            record = db.query(BuildingRecord).filter_by(id=request.building_id).first()
            if not record:
                raise HTTPException(status_code=404, detail="Building not found")
            geojson = json.loads(record.geometry_data)
            building = build_building_from_geojson(
                geojson,
                height=request.building.height,
                num_stories=request.building.num_stories,
                roof_type=request.building.roof_type,
                roof_pitch_deg=request.building.roof_pitch_deg,
                name=record.name,
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
    )

    # Generate waypoints
    waypoints = generate_mission_waypoints(building, config)

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
            est_time_s += 1.0  # hover time for photo

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
    }

    # Prepare viewer data
    viewer_data = {
        "threejs": prepare_threejs_data(building, waypoints),
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
    )

    return {
        "version_id": version.version_id,
        "timestamp": version.timestamp,
        "summary": summary,
        "viewer_data": viewer_data,
        "config_snapshot": {
            "building": request.building.model_dump(),
            "mission": request.mission.model_dump(),
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
        },
    }


@router.get("/versions/{version_id}/kmz")
def download_kmz(version_id: str):
    version = session.get(version_id)
    if not version:
        raise HTTPException(status_code=404, detail="Version not found")

    kmz_bytes = build_kmz_bytes(version.waypoints, version.config)
    filename = f"{version.config.mission_name.replace(' ', '_')}_{version_id}.kmz"

    return Response(
        content=kmz_bytes,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@router.delete("/versions/{version_id}")
def delete_version(version_id: str):
    if not session.delete(version_id):
        raise HTTPException(status_code=404, detail="Version not found")
    return {"deleted": version_id}
