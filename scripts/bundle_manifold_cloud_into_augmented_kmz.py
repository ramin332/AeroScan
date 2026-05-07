#!/usr/bin/env python3
"""Variant of bundle_cloud_into_augmented_kmz.py that bundles the
**registered Manifold cloud** instead of the source KMZ's curated cloud.

Used for frame-alignment diagnostics: if you load the result into the dev
viewer and the cloud doesn't line up with where the waypoints expect the
building to be, ICP misregistered the /blackbox cloud (despite a low RMSE
score, it might have aligned to a local minimum).

Re-runs the same ICP + dev-match prep that flight_planner.cli.augment_mission
does, then bundles the resulting cloud (and the source's sfm_geo_desc.json)
into a copy of the augmented KMZ.

Usage:
    python scripts/bundle_manifold_cloud_into_augmented_kmz.py \\
        --input  ~/Desktop/<ts>.augmented.kmz \\
        --source kmz/Mijande.kmz \\
        --manifold-ply /tmp/manifold-flight16/flight0016-10cm.ply \\
        --output ~/Desktop/<ts>.augmented.with_manifold_cloud.kmz
"""
from __future__ import annotations

import argparse
import io
import struct
import sys
import zipfile
from pathlib import Path

import numpy as np
import open3d as o3d

from flight_planner.kmz_import import parse_kmz, polygon_to_enu
from flight_planner.manifold import register_to_kmz_frame


def _write_ply_xyz(pts: np.ndarray) -> bytes:
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


def _find_member(zf: zipfile.ZipFile, suffix: str) -> str | None:
    for n in zf.namelist():
        if n.endswith(suffix):
            return n
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input", type=Path, required=True,
                    help="Augmented KMZ produced by the Manifold (or any KMZ to wrap a cloud into).")
    ap.add_argument("--source", type=Path, required=True,
                    help="Source Smart3D KMZ (provides the ICP target cloud + sfm_geo_desc.json).")
    ap.add_argument("--manifold-ply", type=Path, required=True,
                    help="Merged Manifold cloud PLY (10 cm voxel from /blackbox).")
    ap.add_argument("--output", type=Path, required=True,
                    help="Output KMZ path.")
    args = ap.parse_args()

    src_bytes = args.source.read_bytes()
    parsed = parse_kmz(src_bytes, name=args.source.stem)
    print(f"Source KMZ:    {args.source}  ({len(src_bytes):,} bytes)")
    print(f"Manifold PLY:  {args.manifold_ply}")

    if not parsed.point_cloud_ply:
        raise SystemExit("Source KMZ has no cloud.ply — cannot ICP.")

    # ICP register: same call as cli.augment_mission step 4.
    print("ICP registering Manifold → KMZ frame…")
    manifold_pc = o3d.io.read_point_cloud(str(args.manifold_ply))
    with open("/tmp/_kmz_target.ply", "wb") as f:
        f.write(parsed.point_cloud_ply)
    kmz_pc = o3d.io.read_point_cloud("/tmp/_kmz_target.ply")
    registered, _T, icp_stats = register_to_kmz_frame(manifold_pc, kmz_pc)
    print(f"  coarse yaw    {icp_stats['coarse_yaw_deg']:.0f}°")
    print(f"  final RMSE    {icp_stats['icp_rmse_m']:.3f} m")
    print(f"  final fitness {icp_stats['icp_fitness']:.3f}")
    ty = icp_stats.get('total_yaw_deg')
    tt = icp_stats.get('total_translation_m')
    if isinstance(ty, (int, float)):
        print(f"  total yaw     {ty:.2f}°")
    if isinstance(tt, (int, float)):
        print(f"  total trans   {tt:.3f} m")
    print(f"  registered cloud: {len(registered.points):,} pts")

    # Write registered cloud as PLY.
    cloud_bytes = _write_ply_xyz(np.asarray(registered.points, dtype=np.float32))

    # Pull sfm_geo_desc.json from the source KMZ.
    geo_bytes: bytes | None = None
    geo_member: str | None = None
    with zipfile.ZipFile(io.BytesIO(src_bytes)) as zf:
        cloud_member = _find_member(zf, "cloud.ply")
        if cloud_member is None:
            raise SystemExit("Source KMZ has no cloud.ply member.")
        geo_member = _find_member(zf, "sfm_geo_desc.json")
        if geo_member is not None:
            geo_bytes = zf.read(geo_member)

    # Copy input KMZ + replace cloud.ply with the registered one + add geo desc.
    print(f"Writing {args.output}…")
    with zipfile.ZipFile(args.input, "r") as zin:
        out_buf = io.BytesIO()
        with zipfile.ZipFile(out_buf, "w", zipfile.ZIP_DEFLATED) as zout:
            existing_cloud = _find_member(zin, "cloud.ply")
            existing_geo = _find_member(zin, "sfm_geo_desc.json")
            for info in zin.infolist():
                if info.filename == existing_cloud:
                    continue  # we'll write the new cloud below
                zout.writestr(info, zin.read(info.filename))
            zout.writestr(cloud_member, cloud_bytes)
            if geo_bytes is not None and existing_geo is None:
                zout.writestr(geo_member, geo_bytes)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(out_buf.getvalue())
    print(f"  output:       {args.output}  ({args.output.stat().st_size:,} bytes)")
    print(f"  cloud.ply at: {cloud_member}  ({len(cloud_bytes):,} bytes)")
    if geo_bytes:
        print(f"  geo_desc at:  {geo_member}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
