"""Ground removal: the fitted plane, the facet gate, and both real clouds.

Context (2026-07-10 flight, van in a parking lot): the old ground removal was a
horizontal cut at ``percentile(z, 5) + 1.0 m``. On a 2.7 m target over sloping
asphalt that destroyed the target's lower body while leaving the ground's noise
tail behind as facets. DJI perception clouds carry ~0.5 m of vertical ground
noise regardless of target (Mijande 0.52 m, parking lot 0.54 m per 2x2 m cell),
so no fixed height cut separates a short object from its ground.

The replacement is two stages: fit the ground plane and clear it by
``ground_clearance_m``, then drop facets that are BOTH horizontal AND low.
Orientation is what makes the facet gate safe — walls and lampposts are
vertical, so they survive at any height.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from flight_planner.kmz_import import (
    facades_from_pointcloud_cgal,
    fit_ground_plane,
    height_above_ground,
    parse_kmz,
    polygon_to_enu,
)

KMZ_DIR = Path(__file__).parent.parent / "kmz"
MIJANDE = KMZ_DIR / "Mijande.kmz"
DATA = Path(__file__).parent / "data"
VAN_PLY = DATA / "van_parking_lot_2026-07-10.ply"
VAN_META = DATA / "van_parking_lot_2026-07-10.json"


def _facet_area(vertices) -> float:
    v = np.asarray(vertices, dtype=float)
    n = np.zeros(3)
    for i in range(len(v)):
        n = n + np.cross(v[i], v[(i + 1) % len(v)])
    return float(np.linalg.norm(n) / 2)


def _centroids(facades) -> np.ndarray:
    return np.array([np.asarray(f.vertices, dtype=float).mean(axis=0) for f in facades])


def _read_binary_ply_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    off = raw.find(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[off:], dtype="<f4").reshape(-1, 3).astype(np.float64)


@pytest.fixture(scope="module")
def van():
    pts = _read_binary_ply_xyz(VAN_PLY)
    meta = json.loads(VAN_META.read_text())
    return pts, meta


@pytest.fixture(scope="module")
def mijande_cloud():
    import os
    import tempfile

    import open3d as o3d

    parsed = parse_kmz(MIJANDE.read_bytes(), name="Mijande")
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as fh:
        fh.write(parsed.point_cloud_ply)
        tmp = fh.name
    pc = o3d.io.read_point_cloud(tmp)
    os.unlink(tmp)
    poly = polygon_to_enu(
        parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt
    )
    return np.asarray(pc.points), poly


# --------------------------------------------------------------------------
# fit_ground_plane
# --------------------------------------------------------------------------


def test_ground_plane_recovers_a_synthetic_slope():
    """A 2.5 deg plane + noise + a box on top: recover the plane, not the box."""
    rng = np.random.default_rng(0)
    slope = math.tan(math.radians(2.5))
    gx, gy = rng.uniform(-20, 20, 20_000), rng.uniform(-20, 20, 20_000)
    gz = slope * gx + rng.normal(0, 0.05, 20_000)
    ground = np.c_[gx, gy, gz]

    bx, by = rng.uniform(0, 6, 4_000), rng.uniform(0, 3, 4_000)
    bz = slope * bx + rng.uniform(0, 3.0, 4_000)
    box = np.c_[bx, by, bz]

    coeffs = fit_ground_plane(np.vstack([ground, box]))

    recovered = math.degrees(math.atan(math.hypot(coeffs[0], coeffs[1])))
    assert abs(recovered - 2.5) < 0.4, f"expected ~2.5 deg, got {recovered:.2f}"

    # Ground sits on the plane; the box stands above it.
    assert abs(np.median(height_above_ground(ground, coeffs))) < 0.10
    assert np.median(height_above_ground(box, coeffs)) > 1.0


def test_ground_plane_holds_when_ground_is_the_majority():
    """Contract: the ground must be the majority of the input cloud.

    ``quantile=0.5`` trims the upper half, so structure occupying less than ~50%
    of the points cannot drag the plane up. At quantile=0.8 even Mijande's
    building floated the fit — that bug ate a third of the cloud.

    This is exactly why ``facades_from_pointcloud_cgal`` fits the plane BEFORE
    the mission-polygon clip: the polygon is drawn tight around the target, so
    inside it the target can be the majority. Outside it, ground dominates.
    """
    rng = np.random.default_rng(1)
    ground = np.c_[
        rng.uniform(-10, 10, 9_000),
        rng.uniform(-10, 10, 9_000),
        rng.normal(0, 0.05, 9_000),
    ]
    box = np.c_[
        rng.uniform(-5, 5, 3_000),
        rng.uniform(-5, 5, 3_000),
        rng.uniform(4.5, 5.5, 3_000),
    ]
    coeffs = fit_ground_plane(np.vstack([ground, box]))
    assert abs(coeffs[2]) < 0.3, f"plane floated to z={coeffs[2]:.2f}, should hug z=0"


def test_ground_plane_survives_sub_surface_noise():
    """Both tails must be trimmed.

    The 2026-07-10 cloud has points 3.5 m below grade. Trimming only the upper
    tail lets those drag the plane down — measured: the van scene sank 1.2 m and
    tilted to 7.6 deg, which is worse than the bug being fixed.
    """
    rng = np.random.default_rng(2)
    ground = np.c_[
        rng.uniform(-10, 10, 10_000),
        rng.uniform(-10, 10, 10_000),
        rng.normal(0, 0.05, 10_000),
    ]
    noise = np.c_[
        rng.uniform(-10, 10, 500),
        rng.uniform(-10, 10, 500),
        rng.uniform(-3.5, -1.0, 500),
    ]
    coeffs = fit_ground_plane(np.vstack([ground, noise]))
    assert abs(coeffs[2]) < 0.15, f"plane sank to z={coeffs[2]:.2f}"
    tilt = math.degrees(math.atan(math.hypot(coeffs[0], coeffs[1])))
    assert tilt < 1.0, f"sub-surface noise tilted the plane to {tilt:.2f} deg"


def test_height_above_ground_sign():
    coeffs = np.array([0.0, 0.0, 5.0])  # flat plane at z=5
    pts = np.array([[0, 0, 7.0], [0, 0, 5.0], [0, 0, 2.0]])
    got = height_above_ground(pts, coeffs)
    assert got[0] == pytest.approx(2.0)
    assert got[1] == pytest.approx(0.0)
    assert got[2] == pytest.approx(-3.0)


# --------------------------------------------------------------------------
# Real clouds
# --------------------------------------------------------------------------


@pytest.mark.skipif(not VAN_PLY.exists(), reason="van fixture not present")
def test_van_scene_ground_is_not_detected_as_facades(van):
    """The 2026-07-10 regression: asphalt must not become facades.

    Before the fix: 80 facets, 29 of them off-target ground covering 75.4 m2 --
    more ground area than van area (65.8 m2).

    'Off-target' here means outside the van's footprint. It is not purely ground:
    the lot has lampposts, which are real vertical objects and are SUPPOSED to
    survive. So this asserts a large reduction, not zero.
    """
    pts, meta = van
    poly = [tuple(p) for p in meta["polygon_enu"]]
    cx, cy = meta["van_centroid_xy"]
    radius = meta["van_radius_m"]

    facades = facades_from_pointcloud_cgal(pts, poly)
    assert facades, "expected the van to still yield facets"

    cent = _centroids(facades)
    areas = np.array([_facet_area(f.vertices) for f in facades])
    off_target = np.hypot(cent[:, 0] - cx, cent[:, 1] - cy) >= radius

    off_area = float(areas[off_target].sum())
    on_area = float(areas[~off_target].sum())

    # Measured at ground_clearance_m=0.4: on 63.8 m2, off 5.7 m2 (was 65.8 / 75.4).
    assert on_area > 55.0, f"van facets eroded: {on_area:.1f} m2 (expect ~63.8)"
    assert off_area < 15.0, f"too much off-target surface survives: {off_area:.1f} m2 (expect ~5.7)"
    assert off_area < on_area, (
        f"off-target area {off_area:.1f} m2 still exceeds van area {on_area:.1f} m2 "
        "-- the ground is winning, as it did before the fix"
    )


@pytest.mark.skipif(not VAN_PLY.exists(), reason="van fixture not present")
def test_van_scene_keeps_no_horizontal_facet_at_ground_level(van):
    """No near-horizontal facet may sit within the facet clearance of the ground."""
    pts, meta = van
    poly = [tuple(p) for p in meta["polygon_enu"]]
    facades = facades_from_pointcloud_cgal(pts, poly)

    plane = fit_ground_plane(pts)
    cent = _centroids(facades)
    height = height_above_ground(cent, plane)
    horizontal = np.abs(np.array([f.normal[2] for f in facades])) >= 0.7

    offenders = int((horizontal & (height < 1.5)).sum())
    assert offenders == 0, f"{offenders} horizontal facets still sit on the ground"


@pytest.mark.skipif(not MIJANDE.exists(), reason="sample KMZ not present")
def test_mijande_building_survives_ground_removal(mijande_cloud):
    """Regression gate: the fix must not eat the building.

    Baseline measured on the pre-fix code (percentile cut): 818 facets, 2098.5 m2.
    The fix legitimately drops a handful of low horizontal facets (ground clutter
    ~1.1-1.5 m up), so the count moves. What must NOT happen is wholesale loss.
    """
    pts, poly = mijande_cloud
    facades = facades_from_pointcloud_cgal(pts, poly)
    total_area = sum(_facet_area(f.vertices) for f in facades)

    # Measured at ground_clearance_m=0.4: 812 facets, 2089.8 m2, 136 walls.
    assert len(facades) > 780, f"facet count collapsed to {len(facades)} (was 818)"
    assert total_area > 2040.0, f"facade area collapsed to {total_area:.1f} m2 (was 2098.5)"

    # Walls are the inspection surface; they are vertical and must be untouched
    # by a gate that only removes horizontal things.
    nz = np.abs(np.array([f.normal[2] for f in facades]))
    walls = int((nz < 0.35).sum())
    assert walls >= 128, f"only {walls} wall facets survived (was 128; measured 136 after fix)"
