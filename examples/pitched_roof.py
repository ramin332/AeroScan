"""Example: generate a KMZ mission for a building with a pitched roof."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from flight_planner.building_presets import pitched_roof_house
from flight_planner.camera import compute_distance_for_gsd, get_camera
from flight_planner.geometry import generate_mission_waypoints
from flight_planner.kmz_builder import build_kmz
from flight_planner.models import MissionConfig

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")


def main():
    building = pitched_roof_house()
    print(f"Building: {building.label}")
    print(f"  Dimensions: {building.width}m x {building.depth}m x {building.height}m (eave)")
    print(f"  Heading: {building.heading_deg}°")
    print(f"  Roof: {building.roof_type.value}, pitch={building.roof_pitch_deg}°")
    print(f"  Facades: {len(building.facades)}")

    for f in building.facades:
        print(f"    [{f.index}] {f.label}: normal=({f.normal[0]:.2f}, {f.normal[1]:.2f}, {f.normal[2]:.2f}), "
              f"tilt_from_vert={f.tilt_from_vertical_deg:.0f}°")

    config = MissionConfig(target_gsd_mm_per_px=2.0)
    waypoints = generate_mission_waypoints(building, config)
    print(f"\nGenerated {len(waypoints)} waypoints")

    # Show waypoints per facade
    facade_counts = {}
    for wp in waypoints:
        facade_counts[wp.facade_index] = facade_counts.get(wp.facade_index, 0) + 1
    for fi, count in sorted(facade_counts.items()):
        facade = building.facades[fi]
        print(f"  Facade {fi} ({facade.label}): {count} waypoints")

    output_path = os.path.join(OUTPUT_DIR, "pitched_roof.kmz")
    result_path = build_kmz(waypoints, config, output_path)
    print(f"\nKMZ written to: {result_path}")


if __name__ == "__main__":
    main()
