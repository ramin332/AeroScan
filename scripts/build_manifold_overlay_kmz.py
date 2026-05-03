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


def register_manifold_to_kmz(
    manifold_pc: o3d.geometry.PointCloud,
    kmz_pc: o3d.geometry.PointCloud,
    *,
    coarse_yaw_search_deg: tuple = (0, 90, 180, 270),
    icp_max_correspondence_m: float = 2.0,
    icp_max_iterations: int = 50,
) -> tuple[o3d.geometry.PointCloud, np.ndarray, dict]:
    """Find the rigid transform that brings the Manifold cloud into the KMZ
    frame. Returns (registered_cloud, 4x4_transform, stats).

    Strategy: coarse search over four 90° yaw rotations (Manifold's perception
    frame is yaw-aligned to aircraft takeoff heading, not to the KMZ's ENU
    frame), pick the one whose centroid lands closest to the KMZ centroid, then
    refine with ICP point-to-point.
    """
    kmz_centroid = np.asarray(kmz_pc.points).mean(axis=0)

    def rot_z(deg: float) -> np.ndarray:
        a = np.deg2rad(deg)
        T = np.eye(4)
        T[:3, :3] = np.array([
            [np.cos(a), -np.sin(a), 0],
            [np.sin(a),  np.cos(a), 0],
            [0, 0, 1],
        ])
        return T

    best_yaw = 0.0
    best_centroid_err = float("inf")
    for yaw in coarse_yaw_search_deg:
        rotated = np.asarray(manifold_pc.points) @ rot_z(yaw)[:3, :3].T
        err = float(np.linalg.norm(rotated.mean(axis=0) - kmz_centroid))
        if err < best_centroid_err:
            best_centroid_err = err
            best_yaw = yaw

    coarse = manifold_pc.transform(rot_z(best_yaw))
    icp = o3d.pipelines.registration.registration_icp(
        coarse, kmz_pc,
        max_correspondence_distance=icp_max_correspondence_m,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=icp_max_iterations),
    )
    fine_T = icp.transformation
    coarse.transform(fine_T)

    total_T = fine_T @ rot_z(best_yaw)
    stats = {
        "coarse_yaw_deg": float(best_yaw),
        "coarse_centroid_err_m": best_centroid_err,
        "icp_fitness": float(icp.fitness),
        "icp_rmse_m": float(icp.inlier_rmse),
        "total_yaw_deg": float(np.degrees(np.arctan2(total_T[1, 0], total_T[0, 0]))),
        "total_translation_m": total_T[:3, 3].tolist(),
    }
    return coarse, total_T, stats


def find_cloud_ply_member(zf: zipfile.ZipFile) -> str:
    candidates = [n for n in zf.namelist() if n.endswith("cloud.ply")]
    if not candidates:
        raise RuntimeError("Source KMZ has no cloud.ply — required for overlay")
    if len(candidates) > 1:
        print(f"warning: multiple cloud.ply members, using first: {candidates[0]}", file=sys.stderr)
    return candidates[0]


def build_overlay_kmz(src_kmz: Path, manifold_ply: Path, out_kmz: Path) -> None:
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

    print(f"Loading Manifold cloud: {manifold_ply}")
    manifold_pc = o3d.io.read_point_cloud(str(manifold_ply))
    print(f"  Manifold cloud: {len(manifold_pc.points):,} pts")

    print("Registering Manifold → KMZ frame…")
    registered, T, stats = register_manifold_to_kmz(manifold_pc, kmz_pc)
    print(f"  coarse yaw: {stats['coarse_yaw_deg']:.0f}°")
    print(f"  ICP fitness: {stats['icp_fitness']:.3f}, RMSE: {stats['icp_rmse_m']:.3f} m")
    print(f"  total yaw: {stats['total_yaw_deg']:.2f}°, translation: {[round(x, 3) for x in stats['total_translation_m']]} m")
    if stats["icp_fitness"] < 0.5:
        print(f"  WARNING: low ICP fitness — clouds may not be the same site")

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as f:
        registered_path = f.name
    o3d.io.write_point_cloud(registered_path, registered)
    registered_bytes = Path(registered_path).read_bytes()
    Path(registered_path).unlink()
    print(f"  registered cloud: {len(registered_bytes):,} bytes ({len(registered.points):,} pts)")

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
    args = p.parse_args()
    build_overlay_kmz(args.kmz, args.manifold_ply, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
