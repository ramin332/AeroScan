"""Example: generate a KMZ mission for a simple rectangular building."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from flight_planner.building_presets import simple_box
from flight_planner.camera import compute_distance_for_gsd, compute_footprint, get_camera
from flight_planner.geometry import generate_mission_waypoints
from flight_planner.kmz_builder import build_kmz
from flight_planner.models import CameraName, MissionConfig

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")


def main():
    # Create building
    building = simple_box()
    print(f"Building: {building.label}")
    print(f"  Dimensions: {building.width}m x {building.depth}m x {building.height}m")
    print(f"  Location: {building.lat:.6f}, {building.lon:.6f}")
    print(f"  Facades: {len(building.facades)}")

    for f in building.facades:
        print(f"    [{f.index}] {f.label}: normal=({f.normal[0]:.2f}, {f.normal[1]:.2f}, {f.normal[2]:.2f}), "
              f"azimuth={f.azimuth_deg:.0f}°, w={f.width:.1f}m, h={f.height:.1f}m")

    # Camera info
    config = MissionConfig(target_gsd_mm_per_px=2.0)
    camera = get_camera(config.camera)
    distance = compute_distance_for_gsd(camera, config.target_gsd_mm_per_px)
    footprint = compute_footprint(camera, distance)

    print(f"\nCamera: {config.camera.value}")
    print(f"  Target GSD: {config.target_gsd_mm_per_px} mm/px")
    print(f"  Required distance: {distance:.1f}m")
    print(f"  Photo footprint: {footprint.width_m:.1f}m x {footprint.height_m:.1f}m")

    # Generate waypoints
    waypoints = generate_mission_waypoints(building, config)
    print(f"\nGenerated {len(waypoints)} waypoints")

    # Show first few waypoints
    for wp in waypoints[:5]:
        print(f"  WP{wp.index}: ({wp.lat:.6f}, {wp.lon:.6f}, {wp.alt:.1f}m) "
              f"heading={wp.heading_deg:.0f}° gimbal_pitch={wp.gimbal_pitch_deg:.0f}° "
              f"facade={wp.facade_index}")

    if len(waypoints) > 5:
        print(f"  ... and {len(waypoints) - 5} more")

    # Generate KMZ
    output_path = os.path.join(OUTPUT_DIR, "simple_box.kmz")
    result_path = build_kmz(waypoints, config, output_path)
    print(f"\nKMZ written to: {result_path}")
    print(f"  File size: {os.path.getsize(result_path)} bytes")


if __name__ == "__main__":
    main()
