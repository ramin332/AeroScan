"""The XMP reader has to survive how DJI actually names a two-shot waypoint.

`kmz_builder` labels every takePhoto with a `wpN` fileSuffix, but the aircraft
only puts that label on the FIRST photo of the waypoint; the extra panned frame
comes back unlabelled. Pairing photos to commanded poses by capture order alone
would silently give the panned shot the primary shot's angles -- which is exactly
the comparison the 2026-09-07 flights exist to make.
"""

from __future__ import annotations

import importlib.util
import zipfile
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "read_gimbal_xmp", Path(__file__).resolve().parents[1] / "scripts" / "read_gimbal_xmp.py"
)
rgx = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(rgx)


def test_unlabelled_frames_belong_to_the_previous_waypoint():
    names = [
        "DJI_20260907143250_0001_V_wp0.JPG",
        "DJI_20260907143252_0003_V.JPG",
        "DJI_20260907143255_0004_V_wp1.JPG",
        "DJI_20260907143256_0005_V.JPG",
    ]
    assert rgx.photo_shots(names) == [(0, 0), (0, 1), (1, 0), (1, 1)]


def test_explicit_shot_suffix_is_honoured_when_present():
    assert rgx.photo_shots(["a_wp7.JPG", "b_wp7_1.JPG", "c_wp8.JPG"]) == [(7, 0), (7, 1), (8, 0)]


def test_rosette_capture_has_no_labels_at_all():
    assert rgx.photo_shots(["DJI_0001_V.JPG", "DJI_0002_V.JPG"]) == [None, None]


def _wpml(body: str) -> str:
    return f'<?xml version="1.0"?><kml><Document><Folder>{body}</Folder></Document></kml>'


def _action(aid: int, xml: str) -> str:
    return f"<wpml:action><wpml:actionId>{aid}</wpml:actionId>{xml}</wpml:actionActuatorFuncParam></wpml:action>"


def _rotate(pitch: float, yaw: float | None) -> str:
    yaw_on = 1 if yaw is not None else 0
    return (
        "<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>"
        "<wpml:actionActuatorFuncParam>"
        "<wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>"
        f"<wpml:gimbalPitchRotateAngle>{pitch}</wpml:gimbalPitchRotateAngle>"
        f"<wpml:gimbalYawRotateEnable>{yaw_on}</wpml:gimbalYawRotateEnable>"
        f"<wpml:gimbalYawRotateAngle>{yaw if yaw is not None else 0.0}</wpml:gimbalYawRotateAngle>"
        "<wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>"
    )


def _photo(suffix: str) -> str:
    return (
        "<wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>"
        "<wpml:actionActuatorFuncParam>"
        f"<wpml:fileSuffix>{suffix}</wpml:fileSuffix>"
    )


def _make_kmz(tmp_path: Path, body: str) -> Path:
    kmz = tmp_path / "m.kmz"
    with zipfile.ZipFile(kmz, "w") as z:
        z.writestr("wpmz/waylines.wpml", _wpml(body))
    return kmz


def test_two_shots_at_one_waypoint_carry_their_own_poses(tmp_path):
    body = (
        "<Placemark><wpml:waypointHeadingAngle>10.0</wpml:waypointHeadingAngle>"
        + _action(0, _rotate(-20.0, None))
        + _action(1, _photo("wp0"))
        + _action(2, _rotate(-8.0, 107.75))
        + _action(3, _photo("wp0_1"))
        + "</Placemark>"
    )
    rows = rgx.commanded_from_kmz(_make_kmz(tmp_path, body))

    assert [r["shot"] for r in rows] == [0, 1]
    assert rows[0]["cmd_pitch"] == -20.0
    assert rows[1]["cmd_pitch"] == -8.0
    # The primary shot commands no gimbal yaw (the gimbal follows the nose);
    # only the panned shot does. Conflating them was the bug.
    assert rows[0]["cmd_yaw"] != rows[0]["cmd_yaw"]  # nan
    assert rows[1]["cmd_yaw"] == 107.75
    assert rows[0]["heading"] == 10.0


def test_a_deduped_waypoint_holds_the_previous_pose(tmp_path):
    body = (
        "<Placemark><wpml:waypointHeadingAngle>0.0</wpml:waypointHeadingAngle>"
        + _action(0, _rotate(-30.0, None))
        + _action(1, _photo("wp0"))
        + "</Placemark>"
        "<Placemark><wpml:waypointHeadingAngle>5.0</wpml:waypointHeadingAngle>"
        + _action(0, _photo("wp1"))
        + "</Placemark>"
    )
    rows = rgx.commanded_from_kmz(_make_kmz(tmp_path, body))
    assert [r["wp"] for r in rows] == [0, 1]
    assert rows[1]["cmd_pitch"] == -30.0
