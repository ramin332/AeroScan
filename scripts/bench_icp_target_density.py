#!/usr/bin/env python3
"""Bench: how coarse can the ICP target cloud be before registration breaks?

Decides whether a downsampled fingerprint of the Smart3D KMZ's cloud.ply fits
in the mission-intent JSON (so the rc-companion can ship it directly), or
whether we need a separate cloud-transport channel (file MOP / depot pre-stage).

For each target-cloud voxel size:
  1. Voxel-downsample the full KMZ cloud
  2. Measure raw + gzipped size of the downsampled PLY
  3. Multi-scale point-to-point ICP from the Manifold cloud against this target
  4. Compare the final transform against the FULL-cloud baseline:
     - rotation delta (axis-angle, deg)
     - translation delta (m)
     - per-axis translation
     - final-stage RMSE

The decision rule: if the rotation delta vs full-cloud baseline is < 0.5° AND
translation delta < 10 cm at a fingerprint size that fits the JSON budget
(say < 100 KB gzipped), then we ship the fingerprint and skip the cloud
transport. Otherwise we fall back to file MOP / depot pre-stage.

Usage (on Manifold):
    mamba run -n aero-scan python scripts/bench_icp_target_density.py \\
        --manifold-flight flight0016 \\
        --kmz-cloud /open_app/dev/data/missions/Mijande.cloud.ply
"""
from __future__ import annotations

import argparse
import gzip
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from flight_planner.manifold import (
    DEFAULT_ICP_SCHEDULE,
    from_manifold,
    register_to_kmz_frame,
)


def transform_delta(T_a: np.ndarray, T_b: np.ndarray) -> tuple[float, float, np.ndarray]:
    """Return (rotation_deg, translation_m, translation_xyz_m) between two
    rigid transforms."""
    # Relative rotation: R_rel = R_a^T @ R_b
    R_rel = T_a[:3, :3].T @ T_b[:3, :3]
    # Axis-angle magnitude from trace
    cos_theta = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
    rot_deg = float(np.degrees(np.arccos(cos_theta)))
    t_rel = T_b[:3, 3] - T_a[:3, 3]
    return rot_deg, float(np.linalg.norm(t_rel)), t_rel


def measure_target_size(target_pc: o3d.geometry.PointCloud) -> tuple[int, int]:
    """Return (raw_bytes, gzip_bytes) for the cloud serialised as binary PLY.

    Mirrors what the rc-companion would put in the JSON intent.
    """
    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as f:
        path = Path(f.name)
    try:
        o3d.io.write_point_cloud(str(path), target_pc, write_ascii=False)
        raw = path.read_bytes()
    finally:
        path.unlink(missing_ok=True)
    gz = gzip.compress(raw, compresslevel=6)
    return len(raw), len(gz)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifold-flight", type=str, default="flight0016")
    p.add_argument("--blackbox-dir", type=Path, default=Path("/blackbox"))
    p.add_argument("--kmz-cloud", type=Path, required=True,
                   help="Full Smart3D KMZ cloud.ply (the registration baseline target).")
    p.add_argument("--target-voxels-cm", type=str, default="1,2,5,10,20,50",
                   help="Comma-separated target voxel sizes in cm.")
    p.add_argument("--manifold-voxel-m", type=float, default=0.10,
                   help="Voxel size for the Manifold source cloud (constant; default 0.10).")
    p.add_argument("--json-out", type=Path, default=None,
                   help="Optional: write structured results as JSON.")
    args = p.parse_args()

    voxels_m = sorted({float(s) / 100.0 for s in args.target_voxels_cm.split(",")})

    print(f"=== ICP target-density bench ===")
    print(f"manifold flight: {args.manifold_flight}   manifold voxel: {args.manifold_voxel_m*100:.0f} cm")
    print(f"kmz cloud (full): {args.kmz_cloud}")
    print(f"target voxels (cm): {[v*100 for v in voxels_m]}")
    print()

    # Load source (Manifold) cloud once — same for every test
    t0 = time.monotonic()
    print(f"[load] Manifold cloud …", flush=True)
    manifold_full = from_manifold(
        args.manifold_flight,
        blackbox_dir=args.blackbox_dir,
        voxel_m=args.manifold_voxel_m,
    )
    print(f"  manifold cloud: {len(manifold_full.points):,} pts  ({time.monotonic()-t0:.1f} s)")

    # Load full KMZ cloud
    t0 = time.monotonic()
    print(f"[load] KMZ cloud (full) …", flush=True)
    kmz_full = o3d.io.read_point_cloud(str(args.kmz_cloud))
    print(f"  kmz cloud (full): {len(kmz_full.points):,} pts  ({time.monotonic()-t0:.1f} s)")

    # Baseline: ICP against the full KMZ cloud
    t0 = time.monotonic()
    print(f"\n[baseline] ICP vs full KMZ cloud …", flush=True)
    full_raw, full_gz = measure_target_size(kmz_full)
    src_copy = o3d.geometry.PointCloud(manifold_full)
    _, T_baseline, baseline_stats = register_to_kmz_frame(src_copy, kmz_full)
    print(f"  baseline RMSE: {baseline_stats['icp_rmse_m']:.4f} m   "
          f"yaw: {baseline_stats['total_yaw_deg']:+.3f}°   "
          f"t: {[round(x,3) for x in baseline_stats['total_translation_m']]} m   "
          f"({time.monotonic()-t0:.1f} s)")
    print(f"  full-cloud size: raw {full_raw/1e6:.2f} MB,  gzip {full_gz/1e6:.2f} MB")

    rows: list[dict] = []
    rows.append({
        "voxel_cm": "FULL",
        "target_pts": len(kmz_full.points),
        "raw_bytes": full_raw,
        "gzip_bytes": full_gz,
        "icp_rmse_m": baseline_stats["icp_rmse_m"],
        "icp_fitness": baseline_stats["icp_fitness"],
        "rot_delta_deg": 0.0,
        "trans_delta_m": 0.0,
        "trans_delta_xyz": [0.0, 0.0, 0.0],
        "elapsed_s": time.monotonic() - t0,
    })

    # For each target voxel, downsample the KMZ cloud and re-ICP
    for vox in voxels_m:
        t0 = time.monotonic()
        print(f"\n[bench] target voxel {vox*100:.0f} cm …", flush=True)
        target = kmz_full.voxel_down_sample(vox)
        raw_b, gz_b = measure_target_size(target)
        print(f"  target: {len(target.points):,} pts   "
              f"raw {raw_b/1024:.1f} KB  gzip {gz_b/1024:.1f} KB")

        src_copy = o3d.geometry.PointCloud(manifold_full)  # fresh copy — register_to_kmz_frame mutates
        _, T_test, stats = register_to_kmz_frame(src_copy, target)
        rot_dd, trans_dd, trans_xyz = transform_delta(T_baseline, T_test)
        elapsed = time.monotonic() - t0
        print(f"  RMSE: {stats['icp_rmse_m']:.4f} m   "
              f"Δrot vs baseline: {rot_dd:.3f}°   "
              f"Δtrans: {trans_dd*100:.1f} cm  ({[round(x*100,1) for x in trans_xyz]} cm)   "
              f"({elapsed:.1f} s)")

        rows.append({
            "voxel_cm": vox * 100,
            "target_pts": len(target.points),
            "raw_bytes": raw_b,
            "gzip_bytes": gz_b,
            "icp_rmse_m": stats["icp_rmse_m"],
            "icp_fitness": stats["icp_fitness"],
            "rot_delta_deg": rot_dd,
            "trans_delta_m": trans_dd,
            "trans_delta_xyz": trans_xyz.tolist(),
            "elapsed_s": elapsed,
        })

    # Summary table
    print()
    print("=" * 100)
    print(f"{'target':>10} {'pts':>10} {'raw KB':>10} {'gzip KB':>10} "
          f"{'RMSE m':>9} {'Δrot°':>8} {'Δtrans cm':>11}")
    print("-" * 100)
    for r in rows:
        target = "FULL" if r["voxel_cm"] == "FULL" else f"{r['voxel_cm']:.0f} cm"
        print(f"{target:>10} {r['target_pts']:>10,} "
              f"{r['raw_bytes']/1024:>10.1f} {r['gzip_bytes']/1024:>10.1f} "
              f"{r['icp_rmse_m']:>9.4f} "
              f"{r['rot_delta_deg']:>8.3f} "
              f"{r['trans_delta_m']*100:>11.1f}")
    print("=" * 100)

    if args.json_out:
        args.json_out.write_text(json.dumps({
            "manifold_flight": args.manifold_flight,
            "manifold_voxel_m": args.manifold_voxel_m,
            "manifold_pts": len(manifold_full.points),
            "kmz_cloud": str(args.kmz_cloud),
            "rows": rows,
        }, indent=2))
        print(f"\nWrote {args.json_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
