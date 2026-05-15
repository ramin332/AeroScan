#!/usr/bin/env python3
"""Build a KMZ that the dev viewer can import to mirror the Manifold's
exact facade-extraction input, so dev-side tuning maps 1:1 to Manifold
behaviour.

Pipeline (same as flight_planner.cli.augment_mission, steps 2–4 of 7):

1. Load the merged Manifold cloud (already voxel-downsampled to ~10 cm
   when produced by the Manifold blackbox merge tooling).
2. Register to the source KMZ's ENU frame via multi-scale point-to-point
   ICP (flight_planner.manifold.register_to_kmz_frame).
3. Apply the dev-match prep: polygon clip to mission_area_wgs84,
   Z-floor at (5th-percentile-Z + 1 m), revoxel to 10 cm.
4. Bundle the pre-filtered cloud back into a copy of the source KMZ
   (replacing wpmz/res/ply/<name>/cloud.ply, leaving everything else
   byte-identical).

Output is ready to drag into http://localhost:3847 and re-extract
facades through the dev frontend — sliders apply, Sidebar params apply.

Usage:
    python scripts/build_manifold_devmatch_kmz.py \\
        --kmz kmz/Mijande.kmz \\
        --manifold-ply /tmp/manifold-flight16/flight0016-10cm.ply \\
        --out kmz/Mijande_manifold_devmatch.kmz
"""
from __future__ import annotations

import argparse
import io
import json
import struct
import sys
import zipfile
from pathlib import Path

import numpy as np
import open3d as o3d

from flight_planner.kmz_import import _points_in_polygon_xy, parse_kmz, polygon_to_enu
from flight_planner.manifold import register_to_kmz_frame


def _find_member(zf: zipfile.ZipFile, suffix: str) -> str:
    matches = [n for n in zf.namelist() if n.endswith(suffix)]
    if not matches:
        raise SystemExit(f"KMZ has no {suffix}")
    return matches[0]


def _write_ply_binary_xyz(pts: np.ndarray) -> bytes:
    pts = np.asarray(pts, dtype=np.float32)
    n = len(pts)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    return header + pts.tobytes()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kmz", type=Path, required=True,
                    help="Source Smart3D KMZ (provides waypoints, polygon, geo desc).")
    ap.add_argument("--manifold-ply", type=Path, required=True,
                    help="Merged Manifold cloud PLY (10 cm voxel from /blackbox).")
    ap.add_argument("--out", type=Path, required=True,
                    help="Output KMZ path.")
    args = ap.parse_args()

    src_bytes = args.kmz.read_bytes()
    print(f"Source KMZ:    {args.kmz}  ({len(src_bytes):,} bytes)")
    print(f"Manifold PLY:  {args.manifold_ply}  ({args.manifold_ply.stat().st_size:,} bytes)")

    parsed = parse_kmz(src_bytes, name=args.kmz.stem)
    polygon_enu = polygon_to_enu(
        parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
    )
    print(f"  ref WGS84:    lat={parsed.ref_lat:.6f} lon={parsed.ref_lon:.6f} alt={parsed.ref_alt:.2f} m")
    print(f"  polygon:      {len(polygon_enu)} verts")

    # Load both clouds.
    print("Loading clouds…")
    manifold_pc = o3d.io.read_point_cloud(str(args.manifold_ply))
    if not parsed.point_cloud_ply:
        raise SystemExit("Source KMZ has no cloud.ply — cannot ICP.")
    with open("/tmp/_kmz_cloud.ply", "wb") as f:
        f.write(parsed.point_cloud_ply)
    kmz_pc = o3d.io.read_point_cloud("/tmp/_kmz_cloud.ply")
    print(f"  manifold:     {len(manifold_pc.points):,} pts")
    print(f"  kmz target:   {len(kmz_pc.points):,} pts")

    # Register Manifold → KMZ frame.
    print("ICP registering Manifold cloud → KMZ frame…")
    registered, _T, icp_stats = register_to_kmz_frame(manifold_pc, kmz_pc)
    print(f"  coarse yaw    {icp_stats['coarse_yaw_deg']:.0f}°")
    print(f"  final RMSE    {icp_stats['icp_rmse_m']:.3f} m")
    print(f"  final fitness {icp_stats['icp_fitness']:.3f}")

    # Apply dev-match prep (mirrors cli.augment_mission).
    pre_pts = len(registered.points)
    n_after_poly = pre_pts
    z_floor = None

    if polygon_enu and len(polygon_enu) >= 3:
        poly_xy = np.array([(p[0], p[1]) for p in polygon_enu], dtype=np.float64)
        pts = np.asarray(registered.points)
        mask = _points_in_polygon_xy(pts[:, :2], poly_xy)
        if mask.sum() > 0:
            registered = o3d.geometry.PointCloud()
            registered.points = o3d.utility.Vector3dVector(pts[mask])
            n_after_poly = int(mask.sum())

    pts = np.asarray(registered.points)
    if len(pts) > 0:
        z_floor = float(np.percentile(pts[:, 2], 5)) + 1.0
        above = pts[pts[:, 2] > z_floor]
        if len(above) > 0:
            registered = o3d.geometry.PointCloud()
            registered.points = o3d.utility.Vector3dVector(above)

    registered = registered.voxel_down_sample(0.10)

    final_pts = np.asarray(registered.points, dtype=np.float32)
    if z_floor is not None:
        print(f"Dev-match prep:  {pre_pts:,} → polygon-clip {n_after_poly:,} → z>{z_floor:.2f}m & revoxel(10cm) → {len(final_pts):,} pts")
    else:
        print(f"Dev-match prep:  {pre_pts:,} → revoxel(10cm) → {len(final_pts):,} pts")

    # Write filtered cloud back into a copy of the source KMZ.
    print(f"Writing {args.out}…")
    new_cloud_bytes = _write_ply_binary_xyz(final_pts)

    # Find original cloud.ply member to know where to place ours.
    with zipfile.ZipFile(io.BytesIO(src_bytes)) as zin:
        cloud_member = _find_member(zin, "cloud.ply")
        out_buf = io.BytesIO()
        with zipfile.ZipFile(out_buf, "w", zipfile.ZIP_DEFLATED) as zout:
            for info in zin.infolist():
                data = new_cloud_bytes if info.filename == cloud_member else zin.read(info.filename)
                zout.writestr(info, data)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(out_buf.getvalue())
    print(f"  output:       {args.out}  ({args.out.stat().st_size:,} bytes)")
    print(f"  cloud.ply at: {cloud_member}  ({len(new_cloud_bytes):,} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
