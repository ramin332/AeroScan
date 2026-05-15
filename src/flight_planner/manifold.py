"""Manifold-side utilities: load /blackbox perception PLYs and register them
into the Smart3D KMZ frame.

The Manifold's `dji_perception/1/mesh_binary_*.ply` chunks are vertex-only point
clouds in a local meter-scale frame anchored on takeoff and yaw-aligned to the
aircraft's heading at takeoff (verified 2026-05-03, see
docs/architecture/data-locations.md). They are NOT in the same frame as the
Smart3D KMZ — a per-flight rotation around Z is needed to register.

This module is the canonical home for that logic. The viewer-overlay script
``scripts/build_manifold_overlay_kmz.py`` and the CLI ``flight_planner.cli``
both import from here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
import open3d as o3d


# Multi-scale ICP schedule: (voxel size m, max correspondence m, max iterations).
# Coarse → fine. Each pass narrows the correspondence search and refines the
# transform from the previous pass. Sub-10 cm RMSE floor on typical Smart3D KMZ
# clouds (~10 cm point spacing); ~9 sec total on flight0016.
DEFAULT_ICP_SCHEDULE: tuple[tuple[float, float, int], ...] = (
    (0.50, 5.0, 30),
    (0.20, 2.0, 30),
    (0.10, 1.0, 30),
    (0.05, 0.5, 30),
    (0.02, 0.2, 30),
)


def merge_blackbox_plys(
    perception_dir: Path,
    *,
    voxel_m: float | None = 0.10,
) -> o3d.geometry.PointCloud:
    """Load and merge all ``mesh_binary_*.ply`` chunks under a perception
    directory into one point cloud.

    Parameters
    ----------
    perception_dir
        Directory containing ``mesh_binary_*.ply``. Typically
        ``/blackbox/<flight>/dji_perception/1/``.
    voxel_m
        If set, voxel-downsample the merged cloud to this size (meters) to
        bound memory / compute downstream. Set to None to keep full resolution.
        10 cm is the default — matches the typical Smart3D KMZ cloud density
        and is what the existing facade extraction is tuned for.
    """
    perception_dir = Path(perception_dir)
    chunks = sorted(perception_dir.glob("mesh_binary_*.ply"))
    if not chunks:
        raise FileNotFoundError(
            f"No mesh_binary_*.ply files in {perception_dir} — "
            f"is this a Smart3D Auto-Exploration flight directory?"
        )

    merged = o3d.geometry.PointCloud()
    for chunk in chunks:
        pc = o3d.io.read_point_cloud(str(chunk))
        merged += pc

    if voxel_m is not None and voxel_m > 0:
        merged = merged.voxel_down_sample(voxel_m)

    return merged


def estimate_kmz_cloud_voxel_m(kmz_pc: o3d.geometry.PointCloud) -> float:
    """Estimate the median nearest-neighbour spacing of the KMZ cloud.

    Used to size output clouds (e.g. for visual overlay KMZs) so they match
    the input KMZ's density. Clamped to [0.05, 0.30] m.
    """
    pts = np.asarray(kmz_pc.points)
    if len(pts) == 0:
        return 0.10
    n_sample = min(5000, len(pts))
    rng = np.random.default_rng(42)
    sample = pts[rng.choice(len(pts), size=n_sample, replace=False)]
    tree = o3d.geometry.KDTreeFlann(kmz_pc)
    nn_dists: list[float] = []
    for p in sample:
        _, _, d2 = tree.search_knn_vector_3d(p, 2)
        if len(d2) >= 2:
            nn_dists.append(float(np.sqrt(d2[1])))
    median_nn = float(np.median(nn_dists)) if nn_dists else 0.10
    return float(np.clip(median_nn, 0.05, 0.30))


def register_to_kmz_frame(
    manifold_pc: o3d.geometry.PointCloud,
    kmz_pc: o3d.geometry.PointCloud,
    *,
    coarse_yaw_search_deg: Sequence[float] = (0, 90, 180, 270),
    schedule: Sequence[tuple[float, float, int]] = DEFAULT_ICP_SCHEDULE,
) -> tuple[o3d.geometry.PointCloud, np.ndarray, dict]:
    """Find the rigid transform that brings the Manifold cloud into the KMZ
    frame. Returns ``(registered_cloud, 4x4_transform, stats)``.

    Strategy:
      1. Coarse yaw seed — search a few rotations around Z (Manifold's
         perception frame is yaw-aligned to aircraft takeoff heading), pick
         the one whose centroid is closest to the KMZ centroid.
      2. Multi-scale point-to-point ICP from coarse voxel down to fine.

    Why point-to-point and not point-to-plane: the wire-side fingerprint
    shipped from the RC is XYZ-only (no normals — see
    `rc-companion/.../PlyVoxelDownsample.kt`), and the augmenter has to
    estimate normals on a 1 m-spaced ~10K-pt cloud. Estimated normals on a
    cloud that sparse are unreliable, and point-to-plane ICP doesn't just
    fail to refine with bad normals — it actively converges to a wrong
    pose. Measured on flight0016/Mijande: point-to-plane left an 86 cm XY
    drift vs the dense-cloud reference; point-to-point on the same input
    landed within 3 cm.
    """
    kmz_centroid = np.asarray(kmz_pc.points).mean(axis=0)

    def rot_z(deg: float) -> np.ndarray:
        a = np.deg2rad(deg)
        T = np.eye(4)
        T[:3, :3] = np.array([
            [np.cos(a), -np.sin(a), 0.0],
            [np.sin(a),  np.cos(a), 0.0],
            [0.0, 0.0, 1.0],
        ])
        return T

    # Step 1 — coarse yaw seed.
    best_yaw = 0.0
    best_centroid_err = float("inf")
    for yaw in coarse_yaw_search_deg:
        rotated = np.asarray(manifold_pc.points) @ rot_z(yaw)[:3, :3].T
        err = float(np.linalg.norm(rotated.mean(axis=0) - kmz_centroid))
        if err < best_centroid_err:
            best_centroid_err = err
            best_yaw = yaw

    coarse = manifold_pc.transform(rot_z(best_yaw))

    # Step 2 — multi-scale point-to-point ICP.
    T = np.eye(4)
    per_scale_stats: list[dict] = []
    for vox, max_d, max_iter in schedule:
        a = coarse.voxel_down_sample(vox)
        b = kmz_pc.voxel_down_sample(vox)
        r = o3d.pipelines.registration.registration_icp(
            a, b,
            max_correspondence_distance=max_d,
            init=T,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
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


def from_manifold(
    flight_id: str = "the_latest_flight",
    *,
    blackbox_dir: Path = Path("/blackbox"),
    voxel_m: float | None = 0.10,
) -> o3d.geometry.PointCloud:
    """Load the merged perception point cloud for a Manifold flight.

    ``flight_id`` may be the symlink ``the_latest_flight`` or a specific
    ``flightNNNN`` directory name. The returned cloud is in the Manifold's
    local frame (anchored on takeoff, yaw-aligned to takeoff heading) — call
    :func:`register_to_kmz_frame` to align it to a Smart3D KMZ frame before
    facade extraction.
    """
    flight_dir = Path(blackbox_dir) / flight_id
    perception_dir = flight_dir / "dji_perception" / "1"
    if not perception_dir.exists():
        raise FileNotFoundError(
            f"No dji_perception/1 under {flight_dir} — "
            f"is this a Smart3D Auto-Exploration flight?"
        )
    return merge_blackbox_plys(perception_dir, voxel_m=voxel_m)
