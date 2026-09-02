"""Slice an augmented KMZ so the mission starts at waypoint N — our
"resume after a battery swap".

PSDK WaypointV3 offers START / STOP / PAUSE / RESUME within one powered
session only (``dji_waypoint_v3.h``); DJI's breakpoint-resume with
``break_point{index, state, progress, wayline_id}`` exists in the Dock/Cloud
API, not in PSDK, and Pilot 2's library resume never sees a PSDK-uploaded
mission. A resume is therefore a NEW mission whose first waypoint is N.
That is exactly the upload + START path already flown five times; the FC's
documented validity check on START refuses a malformed file rather than
flying it.

What slicing does, on both ``wpmz/template.kml`` and ``wpmz/waylines.wpml``:

* keep waypoint Placemarks with ``<wpml:index>`` >= N, drop the rest;
* renumber indices from 0 (WPML requires 0-based, sequential) and remap
  ``actionGroupId`` / ``actionGroupStartIndex`` / ``actionGroupEndIndex``;
* keep ``<wpml:fileSuffix>wpNNN</wpml:fileSuffix>`` at the ORIGINAL numbers so
  the photo set stays one series across sorties (photo ↔ waypoint pairing is
  by that label, never by order);
* if the new first waypoint had no ``gimbalRotate`` (the augment omits a
  rotate when the pose moved < 5° — the gimbal was holding an earlier pose),
  inject one with the carried-forward pitch, and point the Folder's
  ``startActionGroup`` prime at the same pitch;
* recompute ``<wpml:distance>``, scale ``<wpml:duration>``.

Everything else in the archive (bundled cloud, sfm_geo_desc, the mission-area
polygon Placemark) is copied byte-for-byte.
"""
from __future__ import annotations

import io
import math
import re
import zipfile
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

_KML_NS = "http://www.opengis.net/kml/2.2"
_K = f"{{{_KML_NS}}}"
# The WPML namespace carries a version (1.0.2 in DJI's docs, 1.0.6 from our
# builder); resolve it per document rather than hardcoding one.
_W = "{http://www.dji.com/wpmz/1.0.6}"


def _detect_wpml_ns(xml: str) -> str:
    m = re.search(r'xmlns:wpml="([^"]+)"', xml)
    return m.group(1) if m else "http://www.dji.com/wpmz/1.0.6"


@dataclass
class SliceResult:
    kept: int
    dropped: int
    first_original_index: int
    first_pitch_injected: bool
    first_pitch_deg: float | None
    distance_m: float | None


def _text(el: ET.Element | None) -> str | None:
    return el.text if el is not None else None


def _waypoint_placemarks(folder: ET.Element) -> list[tuple[int, ET.Element]]:
    out = []
    for pm in folder.findall(f"{_K}Placemark"):
        idx = pm.find(f"{_W}index")
        if idx is not None and idx.text is not None:
            out.append((int(idx.text), pm))
    out.sort(key=lambda t: t[0])
    return out


def _gimbal_pitch_of(pm: ET.Element) -> float | None:
    """Pitch commanded by this waypoint's gimbalRotate action, if any."""
    for act in pm.iter(f"{_W}action"):
        if _text(act.find(f"{_W}actionActuatorFunc")) != "gimbalRotate":
            continue
        p = act.find(f"{_W}actionActuatorFuncParam")
        if p is None or _text(p.find(f"{_W}gimbalPitchRotateEnable")) != "1":
            continue
        ang = _text(p.find(f"{_W}gimbalPitchRotateAngle"))
        if ang is not None:
            return float(ang)
    return None


def _inject_gimbal_rotate(pm: ET.Element, pitch_deg: float) -> None:
    group = pm.find(f"{_W}actionGroup")
    if group is None:
        return
    acts = group.findall(f"{_W}action")
    a = ET.Element(f"{_W}action")
    ET.SubElement(a, f"{_W}actionId").text = "0"
    ET.SubElement(a, f"{_W}actionActuatorFunc").text = "gimbalRotate"
    p = ET.SubElement(a, f"{_W}actionActuatorFuncParam")
    for tag, val in (
        ("payloadPositionIndex", "0"),
        ("gimbalHeadingYawBase", "aircraft"),
        ("gimbalRotateMode", "absoluteAngle"),
        ("gimbalPitchRotateEnable", "1"),
        ("gimbalPitchRotateAngle", f"{pitch_deg:g}"),
        ("gimbalRollRotateEnable", "0"),
        ("gimbalRollRotateAngle", "0"),
        ("gimbalYawRotateEnable", "0"),
        ("gimbalYawRotateAngle", "0"),
        ("gimbalRotateTimeEnable", "0"),
        ("gimbalRotateTime", "0"),
    ):
        ET.SubElement(p, f"{_W}{tag}").text = val
    # insert before the first existing action, then renumber actionIds
    first_pos = list(group).index(acts[0]) if acts else len(group)
    group.insert(first_pos, a)
    for i, act in enumerate(group.findall(f"{_W}action")):
        aid = act.find(f"{_W}actionId")
        if aid is not None:
            aid.text = str(i)


def _coords(pm: ET.Element) -> tuple[float, float, float] | None:
    c = pm.find(f".//{_K}coordinates")
    if c is None or not c.text:
        return None
    parts = c.text.strip().split(",")
    lon, lat = float(parts[0]), float(parts[1])
    h = pm.find(f"{_W}executeHeight")
    if h is None:
        h = pm.find(f"{_W}height")
    return lon, lat, float(h.text) if h is not None and h.text else 0.0


def _path_distance_m(pms: list[ET.Element]) -> float | None:
    pts = [_coords(pm) for pm in pms]
    if any(p is None for p in pts) or len(pts) < 2:
        return None
    total = 0.0
    for (lon0, lat0, h0), (lon1, lat1, h1) in zip(pts, pts[1:]):
        dx = math.radians(lon1 - lon0) * 6378137.0 * math.cos(math.radians((lat0 + lat1) / 2))
        dy = math.radians(lat1 - lat0) * 6378137.0
        total += math.sqrt(dx * dx + dy * dy + (h1 - h0) ** 2)
    return total


def _slice_document(xml: str, from_wp: int) -> tuple[str, SliceResult]:
    global _W
    wpml_ns = _detect_wpml_ns(xml)
    _W = f"{{{wpml_ns}}}"
    ET.register_namespace("", _KML_NS)
    ET.register_namespace("wpml", wpml_ns)
    root = ET.fromstring(xml)
    doc = root.find(f"{_K}Document")
    if doc is None:
        raise ValueError("no <Document>")
    folder = doc.find(f"{_K}Folder")
    if folder is None:
        raise ValueError("no <Folder>")

    wps = _waypoint_placemarks(folder)
    n = len(wps)
    if n == 0:
        raise ValueError("no waypoints")
    if not (1 <= from_wp <= n - 1):
        raise ValueError(f"from_wp must be in 1..{n - 1} (a full restart is not a slice), got {from_wp}")

    # carried-forward pitch at the cut: last explicit gimbalRotate before from_wp
    held: float | None = None
    for old_idx, pm in wps:
        if old_idx >= from_wp:
            break
        p = _gimbal_pitch_of(pm)
        if p is not None:
            held = p

    remap: dict[int, int] = {}
    kept: list[ET.Element] = []
    for old_idx, pm in wps:
        if old_idx < from_wp:
            folder.remove(pm)
            continue
        new_idx = len(kept)
        remap[old_idx] = new_idx
        kept.append(pm)
        pm.find(f"{_W}index").text = str(new_idx)
        for tag in ("actionGroupId", "actionGroupStartIndex", "actionGroupEndIndex"):
            for el in pm.iter(f"{_W}{tag}"):
                if el.text is not None and el.text.strip().lstrip("-").isdigit():
                    old = int(el.text)
                    el.text = str(remap.get(old, max(0, old - from_wp)))

    injected = False
    first_pitch = _gimbal_pitch_of(kept[0])
    if first_pitch is None and held is not None:
        _inject_gimbal_rotate(kept[0], held)
        first_pitch = held
        injected = True

    # Folder-level metadata (waylines.wpml): start-action prime, distance, duration
    if first_pitch is not None:
        sag = folder.find(f"{_W}startActionGroup")
        if sag is not None:
            for act in sag.iter(f"{_W}action"):
                if _text(act.find(f"{_W}actionActuatorFunc")) == "gimbalRotate":
                    ang = act.find(f".//{_W}gimbalPitchRotateAngle")
                    if ang is not None:
                        ang.text = f"{first_pitch:g}"
    dist = _path_distance_m(kept)
    d_el = folder.find(f"{_W}distance")
    t_el = folder.find(f"{_W}duration")
    if d_el is not None and d_el.text and dist is not None:
        try:
            old_d = float(d_el.text)
            if t_el is not None and t_el.text and old_d > 0:
                t_el.text = f"{float(t_el.text) * dist / old_d:.3f}"
        except ValueError:
            pass
        d_el.text = f"{dist:.3f}"

    out = ET.tostring(root, encoding="unicode", xml_declaration=True)
    return out, SliceResult(
        kept=len(kept), dropped=from_wp, first_original_index=from_wp,
        first_pitch_injected=injected, first_pitch_deg=first_pitch, distance_m=dist,
    )


def slice_kmz(src: Path, dst: Path, *, from_wp: int) -> SliceResult:
    """Write ``dst`` = ``src`` restarted at 0-based WPML waypoint ``from_wp``.

    The FC reports ``currentWaypointIndex`` 1-based; to redo the waypoint that
    was in progress when the mission stopped, pass ``fc_index - 1``.
    """
    src, dst = Path(src), Path(dst)
    result: SliceResult | None = None
    with zipfile.ZipFile(src) as zin:
        names = zin.namelist()
        for req in ("wpmz/template.kml", "wpmz/waylines.wpml"):
            if req not in names:
                raise ValueError(f"{src}: missing {req}")
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zout:
            for name in names:
                data = zin.read(name)
                if name in ("wpmz/template.kml", "wpmz/waylines.wpml"):
                    xml, r = _slice_document(data.decode("utf-8"), from_wp)
                    if name.endswith("waylines.wpml"):
                        result = r
                    data = xml.encode("utf-8")
                zout.writestr(name, data)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(buf.getvalue())
    assert result is not None
    return result
