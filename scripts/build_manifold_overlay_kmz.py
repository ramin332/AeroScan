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


# Multi-scale schedule: (voxel size m, max correspondence m, max iterations)
# Coarse → fine. Each pass narrows the correspondence search and refines
# the transform from the previous pass. Sub-10cm RMSE floor against typical
# Smart3D KMZ clouds (~10cm point spacing); ~9 sec total on flight0016.
_DEFAULT_ICP_SCHEDULE = (
    (0.50, 5.0, 30),
    (0.20, 2.0, 30),
    (0.10, 1.0, 30),
    (0.05, 0.5, 30),
    (0.02, 0.2, 30),
)


def register_manifold_to_kmz(
    manifold_pc: o3d.geometry.PointCloud,
    kmz_pc: o3d.geometry.PointCloud,
    *,
    coarse_yaw_search_deg: tuple = (0, 90, 180, 270),
    schedule: tuple = _DEFAULT_ICP_SCHEDULE,
) -> tuple[o3d.geometry.PointCloud, np.ndarray, dict]:
    """Find the rigid transform that brings the Manifold cloud into the KMZ
    frame. Returns (registered_cloud, 4x4_transform, stats).

    Strategy:
    1. Coarse yaw seed: search 0/90/180/270° rotations around Z (Manifold's
       perception frame is yaw-aligned to aircraft takeoff heading, not to
       the KMZ's ENU frame), pick the one whose centroid is closest to the
       KMZ centroid.
    2. Multi-scale point-to-plane ICP from coarse voxel (50cm) down to fine
       (2cm by default). Point-to-plane uses cloud normals so it converges
       to better minima for surface-aligned data than point-to-point.

    Verified on flight0016/Mijande: ~9 sec, ICP RMSE 0.093 m at 2cm voxel
    (5× tighter than the single-pass point-to-point baseline of 0.47 m).
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

    # Step 1 — coarse yaw seed
    best_yaw = 0.0
    best_centroid_err = float("inf")
    for yaw in coarse_yaw_search_deg:
        rotated = np.asarray(manifold_pc.points) @ rot_z(yaw)[:3, :3].T
        err = float(np.linalg.norm(rotated.mean(axis=0) - kmz_centroid))
        if err < best_centroid_err:
            best_centroid_err = err
            best_yaw = yaw

    coarse = manifold_pc.transform(rot_z(best_yaw))

    # Step 2 — multi-scale point-to-plane ICP. Each pass voxel-downsamples
    # both clouds, estimates normals if missing, and refines the transform
    # from the previous pass.
    T = np.eye(4)
    per_scale_stats: list[dict] = []
    for vox, max_d, max_iter in schedule:
        a = coarse.voxel_down_sample(vox)
        b = kmz_pc.voxel_down_sample(vox)
        if not a.has_normals():
            a.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox * 3, max_nn=30))
        if not b.has_normals():
            b.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox * 3, max_nn=30))
        r = o3d.pipelines.registration.registration_icp(
            a, b,
            max_correspondence_distance=max_d,
            init=T,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
        )
        T = r.transformation
        per_scale_stats.append({
            "voxel_m": vox,
            "fitness": float(r.fitness),
            "rmse_m": float(r.inlier_rmse),
        })

    coarse.transform(T)

    total_T = T @ rot_z(best_yaw)
    stats = {
        "coarse_yaw_deg": float(best_yaw),
        "coarse_centroid_err_m": best_centroid_err,
        "icp_fitness": per_scale_stats[-1]["fitness"],
        "icp_rmse_m": per_scale_stats[-1]["rmse_m"],
        "icp_per_scale": per_scale_stats,
        "total_yaw_deg": float(np.degrees(np.arctan2(total_T[1, 0], total_T[0, 0]))),
        "total_translation_m": total_T[:3, 3].tolist(),
    }
    return coarse, total_T, stats


def estimate_kmz_cloud_voxel_m(kmz_pc: o3d.geometry.PointCloud) -> float:
    """Estimate the median nearest-neighbour spacing of the KMZ cloud, so the
    output cloud baked into the overlay KMZ matches its density (and therefore
    its file size).

    Returns voxel size in meters, clamped to [0.05, 0.30] so we don't ship a
    too-coarse or unnecessarily-fine output.
    """
    pts = np.asarray(kmz_pc.points)
    if len(pts) == 0:
        return 0.10
    # Use ~5K-sample KDTree to estimate nearest-neighbour distance — fast and
    # robust to outliers via median.
    n_sample = min(5000, len(pts))
    rng = np.random.default_rng(42)
    sample = pts[rng.choice(len(pts), size=n_sample, replace=False)]
    tree = o3d.geometry.KDTreeFlann(kmz_pc)
    nn_dists = []
    for p in sample:
        # 2 nearest = self + nearest other
        _, idx, d2 = tree.search_knn_vector_3d(p, 2)
        if len(d2) >= 2:
            nn_dists.append(float(np.sqrt(d2[1])))
    median_nn = float(np.median(nn_dists)) if nn_dists else 0.10
    return float(np.clip(median_nn, 0.05, 0.30))


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
