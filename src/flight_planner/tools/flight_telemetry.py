"""Read the on-board flight telemetry CSV written by ``kmz_runner.c`` and diff
it against the flown KMZ, per waypoint.

The CSV (``/open_app/dev/data/received/telemetry/<UTC>.csv``) carries one row
per ``GIMBAL_ANGLES`` sample (10 Hz) while a WaypointV3 mission is active:

    t_ms,mission_state,wp_index,gimbal_pitch_deg,gimbal_roll_deg,gimbal_yaw_deg,
    q0,q1,q2,q3,lat_deg,lon_deg,alt_m,sats

Gimbal angles are ``T_DjiVector3f`` x=pitch y=roll z=yaw in degrees
(``dji_fc_subscription.h``). The aircraft attitude is the raw quaternion;
Euler angles are derived here with DJI's own formula from
``Payload-SDK-Tutorial/.../50.info-management.md`` (``DjiTest_FcSubscription
ReceiveQuaternionCallback``): yaw from north, clockwise positive, NED frame.
**The sign convention of the derived yaw against the JPEG XMP
``FlightYawDegree`` is still unverified in the air** — compare once on the
first telemetry flight and fix ``aircraft_euler_deg`` if they disagree.

Why this exists: until 2026-09-02 the only record of what the gimbal did was
the JPEG XMP on the SD card. This makes the same question answerable from the
Manifold the moment the aircraft lands.
"""
from __future__ import annotations

import csv
import math
import re
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median

MISSION_STATES_ACTIVE = {32, 48, 64, 80, 98}  # TRANS, MISSION, BREAK, RESUME, RETURN_FIRSTPOINT
MISSION_STATE_FLYING = 48


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def aircraft_euler_deg(q0: float, q1: float, q2: float, q3: float) -> tuple[float, float, float]:
    """(pitch, roll, yaw) in degrees from DJI's quaternion, DJI's own formula."""
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -2 * q1 * q3 + 2 * q0 * q2))))
    roll = math.degrees(math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1))
    yaw = math.degrees(math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1))
    return pitch, roll, yaw


@dataclass
class Sample:
    t_ms: int
    state: int
    wp: int
    gimbal_pitch: float
    gimbal_roll: float
    gimbal_yaw: float
    q: tuple[float, float, float, float] | None
    lat: float | None
    lon: float | None
    alt: float | None
    sats: int | None

    @property
    def aircraft_yaw(self) -> float | None:
        return aircraft_euler_deg(*self.q)[2] if self.q else None


def read_telemetry(path: Path) -> list[Sample]:
    rows: list[Sample] = []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            f2 = line.rstrip("\n").split(",")
            if f2[0] == "t_ms":
                continue
            if len(f2) < 14:
                continue
            def fl(s):
                return float(s) if s not in ("", None) else None
            q = tuple(fl(x) for x in f2[6:10])
            rows.append(Sample(
                t_ms=int(float(f2[0])), state=int(f2[1]), wp=int(f2[2]),
                gimbal_pitch=float(f2[3]), gimbal_roll=float(f2[4]), gimbal_yaw=float(f2[5]),
                q=q if None not in q else None,
                lat=fl(f2[10]), lon=fl(f2[11]), alt=fl(f2[12]),
                sats=int(float(f2[13])) if f2[13] not in ("", None) else None,
            ))
    return rows


def commanded_from_kmz(kmz: Path) -> list[dict[str, float | None]]:
    """Per-waypoint commanded heading and gimbal pitch from waylines.wpml,
    carrying the last gimbalRotate pitch forward over deduped waypoints
    (the gimbal holds its pose; the augment omits repeats within 5°)."""
    with zipfile.ZipFile(kmz) as z:
        w = z.read("wpmz/waylines.wpml").decode("utf-8")
    out: list[dict] = []
    last_pitch: float | None = None
    for pm in re.findall(r"<Placemark>(.*?)</Placemark>", w, re.S):
        idx = re.search(r"<wpml:index>(\d+)</wpml:index>", pm)
        if not idx:
            continue
        h = re.search(r"<wpml:waypointHeadingAngle>([-\d.]+)</wpml:waypointHeadingAngle>", pm)
        gp = re.search(r"<wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\s*<wpml:gimbalPitchRotateAngle>([-\d.]+)<", pm)
        if gp:
            last_pitch = float(gp.group(1))
        out.append({"index": int(idx.group(1)), "heading": float(h.group(1)) if h else None, "pitch": last_pitch})
    return out


@dataclass
class WaypointActual:
    wp: int                      # 1-based FC index
    n: int
    gimbal_pitch: float
    gimbal_yaw: float
    aircraft_yaw: float | None
    pan: float | None            # gimbal yaw relative to the nose
    dwell_s: float


def per_waypoint(samples: list[Sample], *, state: int = MISSION_STATE_FLYING, last_k: int = 3) -> list[WaypointActual]:
    """Collapse samples to one pose per waypoint: the median of the last ``last_k``
    samples before the index advanced — i.e. the pose at the stop, where the
    shutter fires in stop mode."""
    by: dict[int, list[Sample]] = {}
    for s in samples:
        if s.state == state:
            by.setdefault(s.wp, []).append(s)
    out = []
    for wp, rows in sorted(by.items()):
        rows.sort(key=lambda r: r.t_ms)
        tail = rows[-last_k:]
        gy = median(r.gimbal_yaw for r in tail)
        ay = [r.aircraft_yaw for r in tail if r.aircraft_yaw is not None]
        ayaw = median(ay) if ay else None
        out.append(WaypointActual(
            wp=wp, n=len(rows),
            gimbal_pitch=median(r.gimbal_pitch for r in tail),
            gimbal_yaw=gy,
            aircraft_yaw=ayaw,
            pan=wrap180(gy - ayaw) if ayaw is not None else None,
            dwell_s=(rows[-1].t_ms - rows[0].t_ms) / 1000.0,
        ))
    return out


@dataclass
class Report:
    waypoints: int
    pan_median: float | None
    pan_p90: float | None
    pan_at_stop: int
    pitch_err_median: float | None
    pitch_err_p90: float | None
    heading_err_median: float | None
    heading_err_p90: float | None
    dwell_median_s: float | None
    rows: list[dict] = field(default_factory=list)


def _p(vals: list[float], q: float) -> float | None:
    if not vals:
        return None
    v = sorted(vals)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def compare(actual: list[WaypointActual], commanded: list[dict] | None, *, pan_stop_deg: float = 59.0) -> Report:
    """Diff actual per-waypoint poses against the KMZ. FC waypoint index is
    1-based; WPML <wpml:index> is 0-based."""
    cmd = {c["index"]: c for c in (commanded or [])}
    pans, perr, herr, dwell, rows = [], [], [], [], []
    for a in actual:
        c = cmd.get(a.wp - 1)
        pe = wrap180(a.gimbal_pitch - c["pitch"]) if c and c.get("pitch") is not None else None
        he = wrap180(a.aircraft_yaw - c["heading"]) if c and c.get("heading") is not None and a.aircraft_yaw is not None else None
        if a.pan is not None:
            pans.append(abs(a.pan))
        if pe is not None:
            perr.append(abs(pe))
        if he is not None:
            herr.append(abs(he))
        dwell.append(a.dwell_s)
        rows.append({"wp": a.wp, "samples": a.n, "dwell_s": round(a.dwell_s, 2),
                     "gimbal_pitch": round(a.gimbal_pitch, 1), "gimbal_yaw": round(a.gimbal_yaw, 1),
                     "aircraft_yaw": None if a.aircraft_yaw is None else round(a.aircraft_yaw, 1),
                     "pan": None if a.pan is None else round(a.pan, 1),
                     "cmd_pitch": None if not c else c.get("pitch"), "cmd_heading": None if not c else c.get("heading"),
                     "pitch_err": None if pe is None else round(pe, 1), "heading_err": None if he is None else round(he, 1)})
    return Report(
        waypoints=len(actual),
        pan_median=_p(pans, 0.5), pan_p90=_p(pans, 0.9), pan_at_stop=sum(1 for p in pans if p >= pan_stop_deg),
        pitch_err_median=_p(perr, 0.5), pitch_err_p90=_p(perr, 0.9),
        heading_err_median=_p(herr, 0.5), heading_err_p90=_p(herr, 0.9),
        dwell_median_s=_p(dwell, 0.5), rows=rows,
    )


def format_report(r: Report) -> str:
    def f(v, unit="°"):
        return "–" if v is None else f"{v:.1f}{unit}"
    lines = [
        f"waypoints with samples : {r.waypoints}",
        f"gimbal pan |yaw−nose|  : median {f(r.pan_median)}  p90 {f(r.pan_p90)}  at the ±60° stop: {r.pan_at_stop}",
        f"pitch error vs command : median {f(r.pitch_err_median)}  p90 {f(r.pitch_err_p90)}",
        f"heading error vs cmd   : median {f(r.heading_err_median)}  p90 {f(r.heading_err_p90)}",
        f"dwell per waypoint     : median {f(r.dwell_median_s, ' s')}",
        "",
        "success on the 2026-07-10 baseline (pan median 51.5°, p90 61.0°, 41/294 at the stop):",
        "  pan near 0°, heading error falling toward the airframe's own tracking error, pitch error small.",
    ]
    return "\n".join(lines)
