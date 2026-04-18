#!/usr/bin/env python3
"""Benchmark a DJI Smart3D KMZ import without the HTTP layer.

Usage:
    .venv/bin/python scripts/bench_kmz.py kmz/MijandeExtra.kmz
    .venv/bin/python scripts/bench_kmz.py kmz/MijandeExtra.kmz --mode raw
    .venv/bin/python scripts/bench_kmz.py kmz/MijandeExtra.kmz --voxel 0.4

Prints per-phase wall-clock timings from flight_planner._profiling. Useful for
Step 0 / Step 2 optimization work where we want to iterate faster than the
upload-UI → background-thread loop allows.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("kmz", type=Path, help="Path to .kmz file")
    p.add_argument(
        "--mode",
        choices=("raw", "facades"),
        default="facades",
        help="raw = parse + point cloud only (Step 1); facades = full pipeline",
    )
    p.add_argument(
        "--voxel",
        type=float,
        default=0.30,
        help="Voxel size for alpha-shape reconstruction (facades mode only)",
    )
    p.add_argument(
        "--no-cache",
        action="store_true",
        help="Ignore output/kmz_cache/ and always re-parse + re-reconstruct",
    )
    args = p.parse_args()

    if not args.kmz.exists():
        print(f"not found: {args.kmz}", file=sys.stderr)
        return 2

    from flight_planner._profiling import start_recording, get_recorded, phase
    from flight_planner.kmz_import import (
        parse_kmz,
        load_pointcloud,
        pointcloud_to_viewer_arrays,
        pointcloud_to_mesh_ply,
    )
    from flight_planner import kmz_cache

    data = args.kmz.read_bytes()
    size_mb = len(data) / (1024 * 1024)
    print(f"loaded {args.kmz.name} ({size_mb:.1f} MB), mode={args.mode}")

    entry = kmz_cache.entry_dir(data)
    use_cache = not args.no_cache

    start_recording()
    wall_start = time.perf_counter()

    parsed = parse_kmz(data, name=args.kmz.stem)
    print(
        f"  parsed: {len(parsed.waypoints)} waypoints, "
        f"{len(parsed.mission_area_wgs84)} polygon verts, "
        f"ref=({parsed.ref_lat:.6f},{parsed.ref_lon:.6f}), "
        f"pc_bytes={len(parsed.point_cloud_ply or b'')//1024} KB"
    )

    # Always write the cheap artefacts on first run
    kmz_cache.write_meta(entry, {
        "name": parsed.name,
        "ref_lat": parsed.ref_lat,
        "ref_lon": parsed.ref_lon,
        "ref_alt": parsed.ref_alt,
        "num_waypoints": len(parsed.waypoints),
        "ply_size_bytes": len(parsed.point_cloud_ply or b""),
    })
    kmz_cache.write_waypoints(entry, parsed.waypoints)
    kmz_cache.write_mission_area(entry, parsed.mission_area_wgs84, parsed.mission_config_raw)

    pcd = None  # loaded lazily if a downstream phase needs it

    def _ensure_pcd():
        nonlocal pcd
        if pcd is None:
            pcd = load_pointcloud(parsed.point_cloud_ply)
            print(f"  point cloud: {len(pcd.points):,} points")
        return pcd

    if parsed.point_cloud_ply:
        cached_pc = kmz_cache.read_pointcloud(entry) if use_cache else None
        if cached_pc is not None:
            with phase("pointcloud_cache_hit"):
                pc_pos_arr, _pc_col, _ = cached_pc
            n_cached_pts = len(pc_pos_arr) // 3 if pc_pos_arr.ndim == 1 else len(pc_pos_arr)
            print(f"  [cache hit] viewer arrays: {n_cached_pts:,} points")
        else:
            _ensure_pcd()
            pc_pos, pc_col = pointcloud_to_viewer_arrays(pcd, max_points=250_000)
            print(f"  viewer arrays: {len(pc_pos)//3:,} points")
            import numpy as _np
            kmz_cache.write_pointcloud(
                entry,
                _np.asarray(pc_pos, dtype=_np.float32),
                _np.asarray(pc_col, dtype=_np.float32),
                max_points=250_000,
            )

        if args.mode == "facades":
            cached_mesh = kmz_cache.read_mesh_ply(entry) if use_cache else None
            if cached_mesh is not None:
                with phase("mesh_cache_hit"):
                    mesh_bytes = cached_mesh
                print(f"  [cache hit] mesh: {len(mesh_bytes)//1024} KB")
            else:
                mesh_bytes = pointcloud_to_mesh_ply(_ensure_pcd(), voxel_size_override=args.voxel)
                kmz_cache.write_mesh_ply(entry, mesh_bytes)
                print(f"  reconstructed mesh: {len(mesh_bytes)//1024} KB")

            from flight_planner.building_import import build_building_from_mesh
            building = build_building_from_mesh(
                mesh_data=mesh_bytes,
                file_type="ply",
                lat=parsed.ref_lat,
                lon=parsed.ref_lon,
                name=parsed.name,
            )
            print(
                f"  building: {len(building.facades)} facades, "
                f"{building.width:.1f}×{building.depth:.1f}×{building.height:.1f} m"
            )

    wall_total = time.perf_counter() - wall_start
    print("\n--- phase timings ---")
    total = 0.0
    for entry in get_recorded():
        label = entry["label"]
        secs = entry["seconds"]
        total += secs
        print(f"  {label:40s}  {secs:7.3f} s")
    print(f"  {'-' * 50}")
    print(f"  {'measured phases total':40s}  {total:7.3f} s")
    print(f"  {'wall-clock total':40s}  {wall_total:7.3f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
