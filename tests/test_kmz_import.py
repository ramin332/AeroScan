"""Smoke test for DJI Smart3D KMZ parsing."""

from __future__ import annotations

from pathlib import Path

import pytest

from flight_planner.kmz_import import (
    mapping_bbox_to_enu,
    parse_kmz,
    polygon_to_enu,
    resolve_capture_intrinsics,
    waypoints_to_enu,
)


KMZ_DIR = Path(__file__).parent.parent / "kmz"
MIJANDE = KMZ_DIR / "Mijande.kmz"


@pytest.mark.skipif(not MIJANDE.exists(), reason="sample KMZ not present")
def test_parse_mijande():
    data = MIJANDE.read_bytes()
    parsed = parse_kmz(data, name="Mijande")

    assert parsed.name == "Mijande"
    assert len(parsed.waypoints) > 0, "expected at least one waypoint"
    assert len(parsed.mission_area_wgs84) >= 3, "expected a closed polygon"
    assert parsed.point_cloud_ply is not None and len(parsed.point_cloud_ply) > 1000

    # Ref location: Netherlands, Mijande (~52.417°N, 6.641°E)
    assert 52.3 < parsed.ref_lat < 52.5
    assert 6.5 < parsed.ref_lon < 6.7

    # First waypoint should be nearby the reference
    wp0 = parsed.waypoints[0]
    assert abs(wp0.lat - parsed.ref_lat) < 0.01
    assert abs(wp0.lon - parsed.ref_lon) < 0.01


@pytest.mark.skipif(not MIJANDE.exists(), reason="sample KMZ not present")
def test_waypoints_to_enu_roundtrip_near_origin():
    parsed = parse_kmz(MIJANDE.read_bytes(), name="Mijande")
    enu = waypoints_to_enu(parsed.waypoints, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt)
    # All waypoints should be within ~300 m of the reference point
    for e in enu:
        assert abs(e["x"]) < 300 and abs(e["y"]) < 300


@pytest.mark.skipif(not MIJANDE.exists(), reason="sample KMZ not present")
def test_polygon_to_enu_closes():
    parsed = parse_kmz(MIJANDE.read_bytes(), name="Mijande")
    poly_enu = polygon_to_enu(parsed.mission_area_wgs84, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt)
    assert len(poly_enu) == len(parsed.mission_area_wgs84)
    for x, y, _ in poly_enu:
        assert abs(x) < 300 and abs(y) < 300


# ---------------------------------------------------------------------------
# 3D-Tiles tileset.json (the Mapping OBB)
# ---------------------------------------------------------------------------


def test_parse_mijande_extracts_mapping_bbox_raw(sample_kmz):
    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")

    assert parsed.mapping_bbox_raw is not None
    raw = parsed.mapping_bbox_raw

    box = raw["box"]
    assert len(box) == 12
    assert box[0] == pytest.approx(-11.078995704650879)
    assert box[1] == pytest.approx(35.02486801147461)
    assert box[2] == pytest.approx(8.606666564941406)
    # First half-axis: +X, magnitude 30.02
    assert box[3] == pytest.approx(30.02245330810547)
    assert box[4] == pytest.approx(0.0)
    assert box[5] == pytest.approx(0.0)
    # Second half-axis: +Y, magnitude 54.97
    assert box[7] == pytest.approx(54.972225189208984)
    # Third half-axis: +Z, magnitude 9.69
    assert box[11] == pytest.approx(9.690000534057617)

    transform = raw["transform"]
    assert len(transform) == 16
    # ECEF translation (column-major, last column) — the Netherlands at ~6.6°E/52.4°N
    assert transform[12] == pytest.approx(3872130.485288672, rel=1e-6)
    assert transform[13] == pytest.approx(450822.9109937097, rel=1e-6)
    assert transform[14] == pytest.approx(5031308.761120935, rel=1e-6)


def test_parse_mijande_extra_extracts_mapping_bbox_raw(sample_kmz):
    parsed = parse_kmz(sample_kmz("mijande_extra"), name="MijandeExtra")

    assert parsed.mapping_bbox_raw is not None
    box = parsed.mapping_bbox_raw["box"]
    assert len(box) == 12
    assert box[0] == pytest.approx(-38.71443557739258)
    assert box[3] == pytest.approx(21.86248016357422)
    assert box[7] == pytest.approx(37.45479202270508)
    assert box[11] == pytest.approx(5.62999963760376)


def test_parse_auto_explore_has_no_mapping_bbox(sample_kmz):
    """autoExplore KMZs ship only template.kml + waylines.wpml — no tileset."""
    parsed = parse_kmz(sample_kmz("auto_explore"), name="auto_explore")
    assert parsed.mapping_bbox_raw is None


# ---------------------------------------------------------------------------
# mapping_bbox_to_enu — compose 3D-Tiles ECEF→local with WGS84→ENU ref
# ---------------------------------------------------------------------------


def test_mapping_bbox_to_enu_returns_none_when_raw_is_none():
    assert mapping_bbox_to_enu(None, 0.0, 0.0, 0.0) is None


def test_mapping_bbox_to_enu_mijande_matches_raw_when_ref_matches(sample_kmz):
    """DJI's tileset.transform puts the local-frame origin at the sfm_geo_desc
    ref point and aligns the local axes with ENU. So when the viewer ref equals
    the tileset ref, the ENU OBB is literally ``box[0:3]`` + box half-axes.
    """
    import math as _math

    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")
    enu = mapping_bbox_to_enu(
        parsed.mapping_bbox_raw,
        parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
    )

    assert enu is not None
    cx, cy, cz = enu["center"]
    box = parsed.mapping_bbox_raw["box"]
    assert cx == pytest.approx(box[0], abs=1e-3)
    assert cy == pytest.approx(box[1], abs=1e-3)
    assert cz == pytest.approx(box[2], abs=1e-3)

    axes = enu["axes"]
    assert len(axes) == 3
    # Lengths preserved under rigid transform; and axes are axis-aligned here.
    assert _math.hypot(axes[0][0], axes[0][1]) == pytest.approx(box[3], abs=1e-3)
    assert _math.hypot(axes[1][0], axes[1][1]) == pytest.approx(box[7], abs=1e-3)
    assert axes[2][2] == pytest.approx(box[11], abs=1e-3)


def test_mapping_bbox_to_enu_translates_when_ref_offset(sample_kmz):
    """Moving the viewer ref 10m north should shift the ENU OBB center by -10m Y."""
    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")
    base = mapping_bbox_to_enu(
        parsed.mapping_bbox_raw, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt
    )
    # ~10m north ≈ 10 / 111320 deg of latitude
    shifted = mapping_bbox_to_enu(
        parsed.mapping_bbox_raw,
        parsed.ref_lat + 10.0 / 111_320.0,
        parsed.ref_lon,
        parsed.ref_alt,
    )
    assert shifted["center"][1] == pytest.approx(base["center"][1] - 10.0, abs=0.2)
    assert shifted["center"][0] == pytest.approx(base["center"][0], abs=0.2)
    # Axes are rigid: lengths preserved. Directions drift microscopically
    # because the ENU rotation matrix is lat-dependent; that drift is
    # irrelevant to the viewer.
    import math as _math
    for i in range(3):
        len_base = _math.sqrt(sum(v * v for v in base["axes"][i]))
        len_shifted = _math.sqrt(sum(v * v for v in shifted["axes"][i]))
        assert len_shifted == pytest.approx(len_base, rel=1e-9)


def test_mapping_bbox_to_enu_contains_point_cloud(sample_kmz):
    """≥99% of cloud.ply points (already in LOCAL_ENU) fall inside the ENU OBB."""
    import numpy as np

    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")
    enu = mapping_bbox_to_enu(
        parsed.mapping_bbox_raw, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt
    )
    center = np.array(enu["center"])
    axes = [np.array(a) for a in enu["axes"]]

    from flight_planner.kmz_import import load_pointcloud
    pcd = load_pointcloud(parsed.point_cloud_ply)
    pts = np.asarray(pcd.points)
    assert len(pts) > 10_000

    # Project each point onto each half-axis direction; inside iff |proj| ≤ |axis|.
    inside = np.ones(len(pts), dtype=bool)
    rel = pts - center
    for a in axes:
        mag = np.linalg.norm(a)
        direction = a / mag
        proj = rel @ direction
        inside &= np.abs(proj) <= mag + 1e-6
    frac = inside.sum() / len(pts)
    assert frac >= 0.99, f"Only {frac:.4f} of cloud points lie inside OBB"


# ---------------------------------------------------------------------------
# resolve_capture_intrinsics — payload → lens → FOV
# ---------------------------------------------------------------------------


def test_resolve_capture_intrinsics_mijande_m4e_wide(sample_kmz):
    """Smart3D always flies the wide lens. M4E (payloadEnumValue=88) or M3E
    (99) both resolve to the wide CameraSpec.
    """
    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")
    intr = resolve_capture_intrinsics(parsed)

    # Wide M4E: 2·atan(17.3/(2·12)) ≈ 71.5° H, 2·atan(13/(2·12)) ≈ 56.9° V.
    assert intr["name"] == "wide"
    assert intr["focal_length_mm"] == pytest.approx(12.0)
    assert intr["fov_h_deg"] == pytest.approx(71.5, abs=0.2)
    assert intr["fov_v_deg"] == pytest.approx(56.9, abs=0.2)
    assert intr["distance_m"] > 0.0


def test_resolve_capture_intrinsics_distance_matches_orbit_radius(sample_kmz):
    """Smart3D orbit missions fly roughly concentric around the subject. The
    resolved ``distance_m`` should sit in the waypoint-to-area-centroid range
    (roughly the orbit radius), not a zero stub.
    """
    parsed = parse_kmz(sample_kmz("mijande"), name="Mijande")
    intr = resolve_capture_intrinsics(parsed)

    # Sanity bound: M4E Smart3D shoots at 5–60 m typical standoff. Zero means
    # the function returned the old stub.
    assert 5.0 <= intr["distance_m"] <= 60.0


def test_resolve_capture_intrinsics_auto_explore(sample_kmz):
    """autoExplore KMZs have no mission_area polygon and different payload
    metadata — function must still return non-zero FOV (falls back to wide).
    """
    parsed = parse_kmz(sample_kmz("auto_explore"), name="auto_explore")
    intr = resolve_capture_intrinsics(parsed)

    assert intr["fov_h_deg"] > 0.0
    assert intr["fov_v_deg"] > 0.0
    assert intr["focal_length_mm"] > 0.0
