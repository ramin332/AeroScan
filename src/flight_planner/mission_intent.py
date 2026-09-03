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
        settings=coerce_settings(d.get("settings")),
    )



# Planner knobs the RC may set per mission. They ride in the mission-intent JSON
# rather than the augment CLI's argv because the Manifold's C runner builds that
# argv from a fixed list — a new knob would otherwise need a PSDK rebuild and a
# DPK reinstall for every change. Unknown keys are ignored so an older engine
# never chokes on a newer RC, and out-of-range values are clamped rather than
# rejected: a mission that flies with a sane speed beats one that refuses.
SETTING_KEYS: dict[str, tuple[type, float, float]] = {
    # key: (type, min, max)
    "inspection_speed_ms": (float, 0.3, 6.0),
    "target_gsd_mm_per_px": (float, 0.5, 20.0),
    "switch_cost": (float, 0.0, 50.0),
    "max_facade_distance_m": (float, 1.0, 100.0),
    "min_action_dwell_s": (float, 0.0, 10.0),
    # Facade detection. These are the knobs a pilot turns on site when the
    # extractor missed a wall or invented one; ranges match what the CGAL
    # region-growing extractor tolerates before it stops producing anything.
    "fd_min_points": (int, 8, 2000),
    "fd_epsilon_m": (float, 0.01, 0.50),
    # Capped at 0.30 deliberately. Measured on the Manifold 2026-09-03 with the
    # busboom cloud: 0.40 took 3626 s and 0.25 took 16 s for BYTE-IDENTICAL
    # facets (48 either way). Widening the region-growing neighbour search buys
    # nothing here and can strand a pilot in the field for an hour.
    "fd_cluster_epsilon_m": (float, 0.05, 0.30),
    "fd_min_wall_area_m2": (float, 0.1, 50.0),
    "fd_min_density_per_m2": (float, 1.0, 400.0),
    # Ignore facets whose centre sits below this height above the ground. On a
    # low target (a van, a car) the extractor finds tarmac-level facets that
    # pull the aim down and waste frames; on a building it drops kerbs, planters
    # and parked cars so the mission spends its waypoints on the facade.
    "min_facade_height_m": (float, 0.0, 20.0),
    # Photos per stop. The nose aims at the primary wall; extras pan the gimbal
    # within its ±60° travel to walls nothing else photographs. Needs the stop —
    # in fly-through the aircraft is still moving while the sequence runs.
    "shots_per_waypoint": (int, 1, 4),
    "gimbal_pan_window_deg": (float, 0.0, 55.0),
}

#: Detection settings, mapped onto facades_from_pointcloud_cgal keyword names.
FACADE_DETECT_MAP: dict[str, str] = {
    "fd_min_points": "min_points",
    "fd_epsilon_m": "epsilon",
    "fd_cluster_epsilon_m": "cluster_epsilon",
    "fd_min_wall_area_m2": "min_wall_area_m2",
    "fd_min_density_per_m2": "min_density_per_m2",
}


def facade_detect_kwargs(settings: dict[str, Any] | None) -> dict[str, Any]:
    """Translate the RC's fd_* settings into extractor keyword arguments."""
    if not settings:
        return {}
    out: dict[str, Any] = {}
    for key, kw in FACADE_DETECT_MAP.items():
        if key in settings:
            out[kw] = settings[key]
    # Roof and tilted facets share the wall's area floor unless someone asks
    # for something else — one number is what a pilot can reason about on site.
    if "min_wall_area_m2" in out:
        out["min_roof_area_m2"] = out["min_wall_area_m2"]
        out["min_tilted_area_m2"] = out["min_wall_area_m2"] * 0.8
    return out

#: Free-form settings that are not numeric ranges.
SETTING_CHOICES: dict[str, tuple[str, ...]] = {
    "assign_mode": ("viterbi", "greedy"),
}

#: Settings that are simple booleans.
SETTING_FLAGS: tuple[str, ...] = ("stop_at_waypoint",)


def coerce_settings(raw: Any) -> dict[str, Any]:
    """Validate and clamp the RC's planner knobs. Unknown keys are dropped."""
    if not isinstance(raw, dict):
        return {}
    out: dict[str, Any] = {}
    for key, (kind, lo, hi) in SETTING_KEYS.items():
        if key not in raw or raw[key] is None:
            continue
        try:
            value = kind(raw[key])
        except (TypeError, ValueError):
            continue
        out[key] = min(max(value, lo), hi)
    for key, choices in SETTING_CHOICES.items():
        value = raw.get(key)
        if isinstance(value, str) and value in choices:
            out[key] = value
    for key in SETTING_FLAGS:
        if key in raw and raw[key] is not None:
            out[key] = bool(raw[key])
    return out


def write_intent_json(parsed: ImportedKmz, out_path: Path, *, indent: int | None = None) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(imported_kmz_to_intent_dict(parsed), indent=indent))
    return out_path


def read_intent_json(path: Path) -> ImportedKmz:
    return intent_dict_to_imported_kmz(json.loads(Path(path).read_text()))
