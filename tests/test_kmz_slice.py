"""Resume after a battery swap = a NEW mission that starts at waypoint N.

PSDK WaypointV3 only has START/STOP/PAUSE/RESUME within a powered session
(dji_waypoint_v3.h) and DJI's breakpoint-resume lives in the Dock/Cloud API,
not in PSDK. So we slice the augmented KMZ ourselves: keep waypoints >= N,
renumber (WPML indices must start at 0 and be sequential), keep the photo
names at their ORIGINAL waypoint numbers so the photo set stays one series
across sorties, and give the new first waypoint an explicit gimbalRotate when
the original relied on the gimbal holding a pose from an earlier waypoint.
"""
from __future__ import annotations

import io
import re
import zipfile
from pathlib import Path

import pytest

from flight_planner.kmz_builder import build_kmz_bytes
from flight_planner.kmz_slice import SliceResult, slice_kmz
from flight_planner.models import ActionType, CameraAction, CameraName, MissionConfig, Waypoint


def _wps(n=8):
    out = []
    for i in range(n):
        out.append(Waypoint(
            x=float(i * 3), y=0.0, z=10.0, lat=53.2 + i * 1e-4, lon=5.8 + i * 1e-4, alt=10.0,
            heading_deg=180.0,
            # pitch changes only at 0 and 5 -> WPs 1-4 and 6-7 rely on the held pose (dedup)
            gimbal_pitch_deg=-20.0 if i < 5 else -40.0,
            speed_ms=3.0, actions=[CameraAction(action_type=ActionType.TAKE_PHOTO, camera=CameraName.WIDE)],
            facade_index=0, component_tag="21.1", index=i,
        ))
    return out


def _kmz(tmp_path: Path, n=8) -> Path:
    p = tmp_path / "full.kmz"
    p.write_bytes(build_kmz_bytes(_wps(n), MissionConfig(stop_at_waypoint=True)))
    return p


def _files(p: Path) -> dict[str, str]:
    with zipfile.ZipFile(p) as z:
        return {n: z.read(n).decode("utf-8") for n in z.namelist()}


def _indices(xml: str) -> list[int]:
    return [int(x) for x in re.findall(r"<wpml:index>(\d+)</wpml:index>", xml)]


def test_slice_keeps_tail_renumbers_and_preserves_photo_names(tmp_path):
    src = _kmz(tmp_path); dst = tmp_path / "resume.kmz"
    r = slice_kmz(src, dst, from_wp=3)
    assert isinstance(r, SliceResult) and r.kept == 5 and r.dropped == 3 and r.first_original_index == 3
    f = _files(dst)
    for name in ("wpmz/template.kml", "wpmz/waylines.wpml"):
        assert _indices(f[name]) == [0, 1, 2, 3, 4], name
    w = f["wpmz/waylines.wpml"]
    assert re.findall(r"<wpml:fileSuffix>(wp\d+)</wpml:fileSuffix>", w) == ["wp3", "wp4", "wp5", "wp6", "wp7"]
    assert [int(x) for x in re.findall(r"<wpml:actionGroupId>(\d+)<", w)] == [0, 1, 2, 3, 4]
    assert [int(x) for x in re.findall(r"<wpml:actionGroupStartIndex>(\d+)<", w)] == [0, 1, 2, 3, 4]
    assert [int(x) for x in re.findall(r"<wpml:actionGroupEndIndex>(\d+)<", w)] == [0, 1, 2, 3, 4]


def test_first_waypoint_gets_the_held_gimbal_pitch(tmp_path):
    """WP3 in the source has no gimbalRotate (pose held from WP0 at -20°).
    After slicing it is the first waypoint and must carry -20° explicitly."""
    src = _kmz(tmp_path); dst = tmp_path / "resume.kmz"
    r = slice_kmz(src, dst, from_wp=3)
    w = _files(dst)["wpmz/waylines.wpml"]
    first = re.search(r"<Placemark>.*?</Placemark>", w, re.S).group(0)
    assert "gimbalRotate" in first
    assert re.search(r"<wpml:gimbalPitchRotateAngle>(-?\d+(\.\d+)?)</wpml:gimbalPitchRotateAngle>", first).group(1).startswith("-20")
    assert r.first_pitch_injected is True


def test_first_waypoint_with_its_own_gimbal_rotate_is_left_alone(tmp_path):
    src = _kmz(tmp_path); dst = tmp_path / "resume.kmz"
    r = slice_kmz(src, dst, from_wp=5)          # WP5 changes pitch to -40 -> has its own action
    w = _files(dst)["wpmz/waylines.wpml"]
    first = re.search(r"<Placemark>.*?</Placemark>", w, re.S).group(0)
    assert first.count("<wpml:actionActuatorFunc>gimbalRotate<") == 1 and r.first_pitch_injected is False
    assert "-40" in re.search(r"<wpml:gimbalPitchRotateAngle>([^<]+)<", first).group(1)


def test_distance_and_duration_are_recomputed_and_polygon_kept(tmp_path):
    src = _kmz(tmp_path); dst = tmp_path / "resume.kmz"
    slice_kmz(src, dst, from_wp=4)
    fs, fd = _files(src), _files(dst)
    ds = float(re.search(r"<wpml:distance>([\d.]+)<", fs["wpmz/waylines.wpml"]).group(1))
    dd = float(re.search(r"<wpml:distance>([\d.]+)<", fd["wpmz/waylines.wpml"]).group(1))
    ts = float(re.search(r"<wpml:duration>([\d.]+)<", fs["wpmz/waylines.wpml"]).group(1))
    td = float(re.search(r"<wpml:duration>([\d.]+)<", fd["wpmz/waylines.wpml"]).group(1))
    # 4 kept waypoints = 3 legs of the fixture's 1e-4° lat/lon steps (~12.97 m each, geodesic)
    assert abs(dd - 3 * 12.97) < 0.5
    assert abs(td / ts - dd / ds) < 1e-6      # duration scaled with distance
    # bundled resources (cloud, geo desc) and non-waypoint placemarks survive untouched
    assert set(fs) == set(fd)


def test_from_wp_out_of_range_is_rejected(tmp_path):
    src = _kmz(tmp_path)
    with pytest.raises(ValueError):
        slice_kmz(src, tmp_path / "x.kmz", from_wp=8)
    with pytest.raises(ValueError):
        slice_kmz(src, tmp_path / "x.kmz", from_wp=0)   # a full restart is not a slice


def test_real_flown_kmz_slices_cleanly():
    src = Path("flight-archive/2026-07-10/app-state/received/20260710T103507Z_331.augmented.lean.kmz")
    if not src.exists():
        pytest.skip("flown KMZ fixture not present")
    out = io.BytesIO
    dst = Path("/tmp/aeroscan_resume_test.kmz")
    r = slice_kmz(src, dst, from_wp=217)
    assert r.kept == 398 - 217 and r.dropped == 217
    w = _files(dst)["wpmz/waylines.wpml"]
    assert _indices(w)[:3] == [0, 1, 2] and _indices(w)[-1] == r.kept - 1
    assert re.findall(r"<wpml:fileSuffix>(wp\d+)</wpml:fileSuffix>", w)[0] == "wp217"
