"""Smoke tests for flight_planner.cli (parser + module wiring) and the
mission-intent JSON round-trip.

End-to-end CLI coverage on real Manifold /blackbox data lives outside CI —
those tests run on the Manifold against /blackbox/flightNNNN.
"""
from __future__ import annotations

import json
import zipfile
from pathlib import Path

import pytest

from flight_planner.cli import build_parser
from flight_planner.kmz_import import (
    ImportedKmz,
    ParsedWaypoint,
    SmartObliquePose,
)
from flight_planner.mission_intent import (
    SCHEMA_VERSION,
    imported_kmz_to_intent_dict,
    intent_dict_to_imported_kmz,
    read_intent_json,
    write_intent_json,
)


def _sample_imported_kmz() -> ImportedKmz:
    return ImportedKmz(
        name="TestSite",
        ref_lat=53.123456,
        ref_lon=5.654321,
        ref_alt=12.5,
        waypoints=[
            ParsedWaypoint(
                index=0, lon=5.6541, lat=53.1234, alt_egm96=15.0,
                heading_deg=42.0, gimbal_pitch_deg=-15.0, speed_ms=2.5,
                gimbal_yaw_raw_deg=10.0, gimbal_yaw_base="aircraft",
                gimbal_heading_mode="smoothTransition",
                smart_oblique_poses=[
                    SmartObliquePose(pitch_deg=-10.0, yaw_offset_deg=30.0, roll_deg=0.0),
                    SmartObliquePose(pitch_deg=-30.0, yaw_offset_deg=-30.0, roll_deg=0.0),
                ],
            ),
            ParsedWaypoint(
                index=1, lon=5.6542, lat=53.1235, alt_egm96=15.5,
                heading_deg=43.0, gimbal_pitch_deg=-20.0, speed_ms=2.5,
                gimbal_yaw_raw_deg=12.0, gimbal_yaw_base="north",
            ),
        ],
        mission_area_wgs84=[
            (5.6540, 53.1233, 0.0),
            (5.6543, 53.1233, 0.0),
            (5.6543, 53.1236, 0.0),
            (5.6540, 53.1236, 0.0),
        ],
        mission_config_raw={"autoFlightSpeed": "2.0"},  # dropped by the wire format
        point_cloud_ply=b"PLY-SHOULD-BE-DROPPED",
    )


# ---------------------------------------------------------------------------
# Mission-intent JSON round-trip
# ---------------------------------------------------------------------------


def test_intent_round_trip_preserves_augment_path_fields():
    parsed = _sample_imported_kmz()
    d = imported_kmz_to_intent_dict(parsed)
    assert d["schema_version"] == SCHEMA_VERSION

    back = intent_dict_to_imported_kmz(d)
    assert back.name == parsed.name
    assert back.ref_lat == pytest.approx(parsed.ref_lat)
    assert back.ref_lon == pytest.approx(parsed.ref_lon)
    assert back.ref_alt == pytest.approx(parsed.ref_alt)
    assert len(back.waypoints) == len(parsed.waypoints)
    for orig, got in zip(parsed.waypoints, back.waypoints):
        assert got.index == orig.index
        assert got.lon == pytest.approx(orig.lon)
        assert got.lat == pytest.approx(orig.lat)
        assert got.alt_egm96 == pytest.approx(orig.alt_egm96)
        assert got.heading_deg == pytest.approx(orig.heading_deg)
        assert got.gimbal_pitch_deg == pytest.approx(orig.gimbal_pitch_deg)
        assert got.gimbal_yaw_raw_deg == pytest.approx(orig.gimbal_yaw_raw_deg)
        assert got.gimbal_yaw_base == orig.gimbal_yaw_base
        assert got.gimbal_heading_mode == orig.gimbal_heading_mode
        assert len(got.smart_oblique_poses) == len(orig.smart_oblique_poses)
    assert back.mission_area_wgs84 == [tuple(p) for p in parsed.mission_area_wgs84]


def test_intent_drops_cloud_and_mission_config_raw():
    """Bulk fields intentionally don't go over the wire — Manifold has the
    cloud already (via /blackbox), and mission_config_raw isn't used by the
    augment pipeline."""
    parsed = _sample_imported_kmz()
    back = intent_dict_to_imported_kmz(imported_kmz_to_intent_dict(parsed))
    assert back.point_cloud_ply is None
    assert back.mission_config_raw == {}


def test_intent_rejects_wrong_schema_version():
    parsed = _sample_imported_kmz()
    d = imported_kmz_to_intent_dict(parsed)
    d["schema_version"] = 999
    with pytest.raises(ValueError, match="schema_version"):
        intent_dict_to_imported_kmz(d)


def test_intent_size_fits_mop_budget(tmp_path: Path):
    """Plan budget: gzipped ~30-50 KB for 1233 waypoints — what crosses MOP at
    5 KB/s. Raw JSON is larger (verbose field names) but never goes over the
    wire uncompressed; gzip compresses the repeated keys away."""
    import gzip

    parsed = ImportedKmz(
        name="Bench",
        ref_lat=53.0, ref_lon=5.0, ref_alt=10.0,
        waypoints=[
            ParsedWaypoint(
                index=i, lon=5.0 + i * 1e-5, lat=53.0 + i * 1e-5,
                alt_egm96=15.0, heading_deg=float(i % 360),
                gimbal_pitch_deg=-19.0, speed_ms=2.0,
            )
            for i in range(100)
        ],
        mission_area_wgs84=[(5.0, 53.0, 0.0), (5.001, 53.001, 0.0)],
        mission_config_raw={},
        point_cloud_ply=None,
    )
    out = tmp_path / "bench.intent.json"
    write_intent_json(parsed, out)
    raw_bytes = out.read_bytes()
    gz_size = len(gzip.compress(raw_bytes, compresslevel=6))

    # 100 waypoints — should gzip to ~2 KB; 1233 WP projected ~25 KB gzipped.
    per_wp_gz = gz_size / 100
    predicted_1233_gz = per_wp_gz * 1233
    assert predicted_1233_gz < 60_000, (
        f"projected 1233-WP gzipped intent {predicted_1233_gz:.0f} bytes "
        f"exceeds 60 KB MOP budget headroom (plan target: 30-50 KB gzipped)"
    )


def test_intent_json_file_round_trip(tmp_path: Path):
    parsed = _sample_imported_kmz()
    p = tmp_path / "intent.json"
    write_intent_json(parsed, p, indent=2)
    back = read_intent_json(p)
    assert back.name == parsed.name
    assert len(back.waypoints) == len(parsed.waypoints)


# ---------------------------------------------------------------------------
# CLI parser
# ---------------------------------------------------------------------------


def test_parser_help_runs():
    p = build_parser()
    with pytest.raises(SystemExit):
        p.parse_args(["--help"])


def test_parser_kmz_to_json_minimal():
    p = build_parser()
    ns = p.parse_args([
        "kmz-to-json",
        "--source-kmz", "/tmp/in.kmz",
        "--json-out", "/tmp/out.json",
    ])
    assert ns.cmd == "kmz-to-json"
    assert ns.cloud_out is None
    assert ns.pretty is False


def test_parser_augment_requires_one_source():
    """argparse mutual-exclusion: --mission-json XOR --source-kmz."""
    p = build_parser()
    with pytest.raises(SystemExit):
        p.parse_args([
            "augment-mission",
            "--mission-json", "/x.json",
            "--source-kmz", "/y.kmz",
            "--output-kmz", "/z.kmz",
        ])
    with pytest.raises(SystemExit):
        p.parse_args(["augment-mission", "--output-kmz", "/z.kmz"])


def test_parser_augment_mission_json_path():
    p = build_parser()
    ns = p.parse_args([
        "augment-mission",
        "--mission-json", "/m.json",
        "--icp-target-ply", "/c.ply",
        "--flight-id", "flight0016",
        "--output-kmz", "/out.kmz",
    ])
    assert ns.cmd == "augment-mission"
    assert str(ns.mission_json) == "/m.json"
    assert ns.source_kmz is None
    assert str(ns.icp_target_ply) == "/c.ply"


def test_parser_augment_source_kmz_dev_path():
    p = build_parser()
    ns = p.parse_args([
        "augment-mission",
        "--source-kmz", "/in.kmz",
        "--flight-id", "flight0016",
        "--output-kmz", "/out.kmz",
    ])
    assert ns.source_kmz is not None
    assert ns.mission_json is None
    assert ns.icp_target_ply is None  # picked up from the source KMZ


def test_manifold_module_reexport_from_building_import():
    """from_manifold is exposed both as flight_planner.manifold.from_manifold
    and via building_import for callers that import it alongside the other
    Building constructors."""
    from flight_planner.building_import import from_manifold as bi
    from flight_planner.manifold import from_manifold as mf
    assert bi is mf
