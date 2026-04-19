"""Smoke test for DJI Smart3D KMZ parsing."""

from __future__ import annotations

from pathlib import Path

import pytest

from flight_planner.kmz_import import parse_kmz, polygon_to_enu, waypoints_to_enu


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
