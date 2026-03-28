"""REST API endpoints for the AeroScan debug server."""

from __future__ import annotations

from typing import Literal, Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import Response
from pydantic import BaseModel, Field

from ..building_presets import (
    l_shaped_block,
    large_apartment_block,
    pitched_roof_house,
    simple_box,
)
from ..camera import compute_distance_for_gsd, compute_footprint, get_camera
from ..geometry import build_rectangular_building, generate_mission_waypoints
from ..kmz_builder import build_kmz_bytes
from ..models import CameraName, MissionConfig, RoofType
from ..visualize import prepare_leaflet_data, prepare_threejs_data
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
    building: BuildingParams = BuildingParams()
    mission: MissionParams = MissionParams()


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


# --- Endpoints ---


@router.get("/presets")
def get_presets():
    return {"presets": PRESET_DEFAULTS}


@router.post("/generate")
def generate(request: GenerateRequest):
    # Build the building
    if request.preset and request.preset in PRESET_MAP:
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

    # Compute summary
    camera_spec = get_camera(camera_name)
    distance = compute_distance_for_gsd(camera_spec, config.target_gsd_mm_per_px)
    footprint = compute_footprint(camera_spec, distance)
    estimated_flight_time_s = len(waypoints) * 2  # rough: ~2s per waypoint (hover + move)

    summary = {
        "waypoint_count": len(waypoints),
        "facade_count": len(building.facades),
        "camera_distance_m": round(distance, 1),
        "photo_footprint_m": [round(footprint.width_m, 1), round(footprint.height_m, 1)],
        "estimated_flight_time_s": estimated_flight_time_s,
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
