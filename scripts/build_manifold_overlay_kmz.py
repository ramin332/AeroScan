#!/usr/bin/env python3
"""
Build a synthetic KMZ that overlays a Manifold perception cloud onto an existing
Smart3D KMZ's flight plan. Output is a copy of the source KMZ with its
``wpmz/res/ply/<name>/cloud.ply`` replaced by the Manifold cloud (registered to
the KMZ's ENU frame via ICP). Everything else — waypoints, mission area
polygon, gimbal commands — is byte-identical.

Use this to visually verify, in AeroScan's existing Viewer3D, that the
Manifold's raw perception data matches the curated KMZ cloud at the same site.
Import both files (original + _manifold) through ``/import-kmz`` and toggle
between them.

Usage:
    python scripts/build_manifold_overlay_kmz.py \\
        --kmz kmz/Mijande.kmz \\
        --manifold-ply /tmp/manifold-flight16/flight0016-20cm.ply \\
        --out kmz/Mijande_manifold.kmz
"""
from __future__ import annotations

import argparse
import io
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np
import open3d as o3d

# Canonical home for the registration logic — both this script and
# flight_planner.cli import from here so there's a single implementation.
from flight_planner.manifold import (
    estimate_kmz_cloud_voxel_m,
    register_to_kmz_frame as register_manifold_to_kmz,
)


def find_cloud_ply_member(zf: zipfile.ZipFile) -> str:
    candidates = [n for n in zf.namelist() if n.endswith("cloud.ply")]
    if not candidates:
        raise RuntimeError("Source KMZ has no cloud.ply — required for overlay")
    if len(candidates) > 1:
        print(f"warning: multiple cloud.ply members, using first: {candidates[0]}", file=sys.stderr)
    return candidates[0]


def build_overlay_kmz(
    src_kmz: Path,
    manifold_ply: Path,
    out_kmz: Path,
    *,
    output_voxel_m: float | None = None,
) -> None:
    print(f"Loading source KMZ: {src_kmz}")
    with zipfile.ZipFile(src_kmz) as zf:
        cloud_member = find_cloud_ply_member(zf)
        kmz_cloud_bytes = zf.read(cloud_member)

    print(f"  cloud.ply at: {cloud_member}  ({len(kmz_cloud_bytes):,} bytes)")
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as f:
        f.write(kmz_cloud_bytes)
        kmz_pc_path = f.name
    kmz_pc = o3d.io.read_point_cloud(kmz_pc_path)
    Path(kmz_pc_path).unlink()
    print(f"  KMZ cloud: {len(kmz_pc.points):,} pts")

    # Estimate input cloud density so the output stays similar size.
    estimated_voxel = estimate_kmz_cloud_voxel_m(kmz_pc)
    if output_voxel_m is None:
        output_voxel_m = estimated_voxel
        print(f"  output voxel auto-picked from KMZ density: {output_voxel_m*100:.1f} cm")
    else:
        print(f"  output voxel (user-specified): {output_voxel_m*100:.1f} cm   "
              f"(KMZ-density estimate was {estimated_voxel*100:.1f} cm)")

    print(f"Loading Manifold cloud: {manifold_ply}")
    manifold_pc = o3d.io.read_point_cloud(str(manifold_ply))
    print(f"  Manifold cloud: {len(manifold_pc.points):,} pts")

    print("Registering Manifold → KMZ frame (multi-scale point-to-plane ICP)…")
    registered, T, stats = register_manifold_to_kmz(manifold_pc, kmz_pc)
    print(f"  coarse yaw seed: {stats['coarse_yaw_deg']:.0f}°")
    for s in stats["icp_per_scale"]:
        print(f"    {s['voxel_m']*100:.0f} cm   fitness {s['fitness']:.3f}   RMSE {s['rmse_m']:.3f} m")
    print(f"  final RMSE:  {stats['icp_rmse_m']:.3f} m")
    print(f"  final yaw:   {stats['total_yaw_deg']:.3f}°")
    print(f"  translation: {[round(x, 4) for x in stats['total_translation_m']]} m")
    if stats["icp_per_scale"][1]["fitness"] < 0.5:
        # Check fitness at 20cm scale; finer scales naturally drop fitness
        # because Manifold has detail KMZ doesn't represent.
        print(f"  WARNING: low ICP fitness at 20cm — clouds may not be the same site")

    # Voxel-downsample the registered cloud to match the input KMZ's density
    # so the output KMZ stays similar size to the input. Registration was at
    # full Manifold resolution; only the *baked* version is downsampled.
    output_pc = registered.voxel_down_sample(output_voxel_m)
    print(f"  registered (full): {len(registered.points):,} pts → "
          f"output {len(output_pc.points):,} pts at {output_voxel_m*100:.1f} cm voxel")

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as f:
        registered_path = f.name
    o3d.io.write_point_cloud(registered_path, output_pc)
    registered_bytes = Path(registered_path).read_bytes()
    Path(registered_path).unlink()
    print(f"  output cloud: {len(registered_bytes):,} bytes ({len(output_pc.points):,} pts)")

    print(f"Writing overlay KMZ: {out_kmz}")
    out_kmz.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(src_kmz) as src, zipfile.ZipFile(out_kmz, "w", zipfile.ZIP_DEFLATED) as dst:
        for info in src.infolist():
            if info.filename == cloud_member:
                dst.writestr(info.filename, registered_bytes)
            else:
                dst.writestr(info.filename, src.read(info.filename))
    print(f"  done. {out_kmz} ({out_kmz.stat().st_size:,} bytes)")
    print()
    print(f"Import both into AeroScan via /import-kmz to compare:")
    print(f"  - {src_kmz} (DJI's curated cloud)")
    print(f"  - {out_kmz} (Manifold's raw perception, registered)")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--kmz", type=Path, required=True, help="Source Smart3D KMZ")
    p.add_argument("--manifold-ply", type=Path, required=True, help="Manifold mesh PLY (merged or downsampled)")
    p.add_argument("--out", type=Path, required=True, help="Output KMZ path")
    p.add_argument("--output-voxel-m", type=float, default=None,
                   help="Voxel size for the cloud baked into the output KMZ (m). "
                        "If omitted, auto-estimated from the input KMZ's median "
                        "nearest-neighbour spacing so the output stays a similar "
                        "size to the input.")
    args = p.parse_args()
    build_overlay_kmz(args.kmz, args.manifold_ply, args.out,
                      output_voxel_m=args.output_voxel_m)
    return 0


if __name__ == "__main__":
    sys.exit(main())
