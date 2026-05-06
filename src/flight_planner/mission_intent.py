"""Compact mission-intent JSON — the wire format between RC and Manifold.

A Smart3D KMZ is ~15 MB (mostly the cloud, tile pyramid, and mesh.bin). The
augmentation pipeline only needs the *intent*: the reference GPS, the
mission-area polygon, and the per-waypoint pose / action data. Everything
else — the dense cloud, the 3D Tiles pyramid — is large and either already
present on the Manifold (via /blackbox) or not needed at all.

Schema fields mirror :class:`flight_planner.kmz_import.ParsedWaypoint` and
:class:`ImportedKmz` so round-trip via JSON loses nothing the augment path
uses. The ``schema_version`` is bumped on any breaking change.

Typical size: ~30 KB for 1233 waypoints, ~10 KB gzipped — fits the plan's
MOP transport budget (5 KB/s upstream).
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .kmz_import import ImportedKmz, ParsedWaypoint, SmartObliquePose


SCHEMA_VERSION = 1


def imported_kmz_to_intent_dict(parsed: ImportedKmz) -> dict[str, Any]:
    """Project an ImportedKmz down to the JSON-shippable intent.

    Drops ``point_cloud_ply`` — that travels via a separate channel (file
    MOP, rsync, or a future GPS-based registration that needs no cloud).
    Drops ``mission_config_raw`` to keep the wire format minimal — Phase 3
    can add it back if the rc-companion needs to display autoFlightSpeed
    / orthoCameraOverlap on the RC.
    """
    return {
        "schema_version": SCHEMA_VERSION,
        "name": parsed.name,
        "ref": {
            "lat": parsed.ref_lat,
            "lon": parsed.ref_lon,
            "alt": parsed.ref_alt,
        },
        "mission_area_wgs84": [list(p) for p in parsed.mission_area_wgs84],
        "waypoints": [
            {
                "index": wp.index,
                "lon": wp.lon,
                "lat": wp.lat,
                "alt_egm96": wp.alt_egm96,
                "heading_deg": wp.heading_deg,
                "gimbal_pitch_deg": wp.gimbal_pitch_deg,
                "speed_ms": wp.speed_ms,
                "gimbal_yaw_raw_deg": wp.gimbal_yaw_raw_deg,
                "gimbal_heading_mode": wp.gimbal_heading_mode,
                "gimbal_yaw_base": wp.gimbal_yaw_base,
                "smart_oblique_poses": [asdict(p) for p in wp.smart_oblique_poses],
            }
            for wp in parsed.waypoints
        ],
    }


def intent_dict_to_imported_kmz(d: dict[str, Any]) -> ImportedKmz:
    """Inverse of :func:`imported_kmz_to_intent_dict`. The returned
    ImportedKmz has ``point_cloud_ply=None`` and ``mission_config_raw={}``
    — those are intentionally dropped by the wire format.
    """
    schema = d.get("schema_version")
    if schema != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported mission-intent schema_version={schema!r} "
            f"(this build expects {SCHEMA_VERSION})"
        )

    ref = d["ref"]
    waypoints = [
        ParsedWaypoint(
            index=int(w["index"]),
            lon=float(w["lon"]),
            lat=float(w["lat"]),
            alt_egm96=float(w["alt_egm96"]),
            heading_deg=float(w["heading_deg"]),
            gimbal_pitch_deg=float(w["gimbal_pitch_deg"]),
            speed_ms=float(w.get("speed_ms", 2.0)),
            gimbal_yaw_raw_deg=float(w.get("gimbal_yaw_raw_deg", 0.0)),
            gimbal_heading_mode=str(w.get("gimbal_heading_mode", "smoothTransition")),
            gimbal_yaw_base=str(w.get("gimbal_yaw_base", "aircraft")),
            smart_oblique_poses=[
                SmartObliquePose(
                    pitch_deg=float(p["pitch_deg"]),
                    yaw_offset_deg=float(p["yaw_offset_deg"]),
                    roll_deg=float(p.get("roll_deg", 0.0)),
                )
                for p in w.get("smart_oblique_poses", [])
            ],
        )
        for w in d["waypoints"]
    ]
    return ImportedKmz(
        name=str(d.get("name", "")),
        ref_lat=float(ref["lat"]),
        ref_lon=float(ref["lon"]),
        ref_alt=float(ref["alt"]),
        waypoints=waypoints,
        mission_area_wgs84=[tuple(p) for p in d.get("mission_area_wgs84", [])],
        mission_config_raw={},
        point_cloud_ply=None,
    )


def write_intent_json(parsed: ImportedKmz, out_path: Path, *, indent: int | None = None) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(imported_kmz_to_intent_dict(parsed), indent=indent))
    return out_path


def read_intent_json(path: Path) -> ImportedKmz:
    return intent_dict_to_imported_kmz(json.loads(Path(path).read_text()))
