"""Telemetry CSV reader: quaternion → yaw with DJI's formula, per-waypoint
collapse at the stop, diff against the flown KMZ (FC index 1-based vs WPML 0-based)."""
from __future__ import annotations

import io
import math
import zipfile
from pathlib import Path

from flight_planner.tools.flight_telemetry import (
    aircraft_euler_deg, commanded_from_kmz, compare, per_waypoint, read_telemetry,
)


def _q_yaw(yaw_deg: float):
    """Quaternion for a pure yaw about z (NED), w-first."""
    h = math.radians(yaw_deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def test_euler_from_pure_yaw_quaternion_matches_dji_formula():
    for y in (0.0, 45.0, 90.0, -120.0, 179.0):
        p, r, yaw = aircraft_euler_deg(*_q_yaw(y))
        assert abs(yaw - y) < 1e-6 and abs(p) < 1e-6 and abs(r) < 1e-6


def _csv(tmp_path: Path, rows):
    p = tmp_path / "t.csv"
    p.write_text("# header comment\n" + "t_ms,mission_state,wp_index,gimbal_pitch_deg,gimbal_roll_deg,gimbal_yaw_deg,q0,q1,q2,q3,lat_deg,lon_deg,alt_m,sats\n"
                 + "\n".join(",".join(str(x) for x in r) for r in rows) + "\n")
    return p


def test_reader_skips_comment_and_header_and_collapses_per_waypoint(tmp_path):
    q = _q_yaw(30.0)
    rows = []
    t = 0
    for wp, gyaw in ((1, 30.0), (2, 75.0)):            # wp2: gimbal 45° right of the nose
        for k in range(5):
            rows.append((t, 48, wp, -20.0, 0.0, gyaw, *q, 52.4, 4.95, 45.0, 18)); t += 100
    rows.append((t, 0, 2, -20.0, 0.0, 75.0, *q, 52.4, 4.95, 45.0, 18))  # IDLE row must be ignored
    s = read_telemetry(_csv(tmp_path, rows))
    assert len(s) == 11
    a = per_waypoint(s)
    assert [x.wp for x in a] == [1, 2]
    assert abs(a[0].pan) < 1e-6 and abs(a[1].pan - 45.0) < 1e-6
    assert abs(a[0].dwell_s - 0.4) < 1e-9


def _kmz(tmp_path: Path) -> Path:
    def pm(i, heading, pitch=None):
        g = (f"<wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>"
             f"<wpml:gimbalPitchRotateAngle>{pitch}</wpml:gimbalPitchRotateAngle>") if pitch is not None else ""
        return f"<Placemark><wpml:index>{i}</wpml:index><wpml:waypointHeadingAngle>{heading}</wpml:waypointHeadingAngle>{g}</Placemark>"
    wpml = "<kml>" + pm(0, 30.0, -20.0) + pm(1, 40.0) + pm(2, 50.0, -35.0) + "</kml>"
    p = tmp_path / "flown.kmz"
    with zipfile.ZipFile(p, "w") as z:
        z.writestr("wpmz/waylines.wpml", wpml)
    return p


def test_commanded_pitch_carries_forward_over_deduped_waypoints(tmp_path):
    c = commanded_from_kmz(_kmz(tmp_path))
    assert [x["pitch"] for x in c] == [-20.0, -20.0, -35.0]
    assert [x["heading"] for x in c] == [30.0, 40.0, 50.0]


def test_compare_uses_1_based_fc_index_and_reports_errors(tmp_path):
    q = _q_yaw(30.0)
    rows = [(k * 100, 48, 1, -22.0, 0.0, 30.0, *q, 52.4, 4.95, 45.0, 18) for k in range(4)]
    q2 = _q_yaw(52.0)
    rows += [(400 + k * 100, 48, 3, -35.0, 0.0, 111.0, *q2, 52.4, 4.95, 45.0, 18) for k in range(4)]
    rep = compare(per_waypoint(read_telemetry(_csv(tmp_path, rows))), commanded_from_kmz(_kmz(tmp_path)))
    r = {x["wp"]: x for x in rep.rows}
    assert r[1]["cmd_pitch"] == -20.0 and abs(r[1]["pitch_err"] + 2.0) < 1e-6 and abs(r[1]["heading_err"]) < 1e-6  # signed: actual −22 vs cmd −20
    assert r[3]["cmd_heading"] == 50.0 and abs(r[3]["heading_err"] - 2.0) < 1e-6
    assert r[3]["pitch_err"] == 0.0 and rep.pitch_err_p90 == 2.0
    assert abs(r[3]["pan"] - 59.0) < 1e-6 and rep.pan_at_stop == 1
