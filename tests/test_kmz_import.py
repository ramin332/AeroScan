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
