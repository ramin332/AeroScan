"""AeroScan CLI for Manifold-resident execution.

Two subcommands:

* ``kmz-to-json`` — split a Smart3D KMZ into a compact mission-intent JSON
  and an ICP-target cloud PLY. Laptop-side dev tool. The JSON is the same
  shape the rc-companion app will produce on the RC.

* ``augment-mission`` — take the mission-intent JSON + a flight ID under
  ``/blackbox/`` (and an ICP target cloud), produce an NEN-2767 augmented
  KMZ ready for ``DjiWaypointV3_UploadKmzFile``.

The augmentation only changes gimbal pitch/yaw and the per-waypoint actions.
DJI's flight-tested trajectory and waypoint spacing are preserved verbatim.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

from .gimbal_rewrite import rewrite_gimbals_perpendicular
from .kmz_builder import build_kmz
from .kmz_import import (
    ImportedKmz,
    _points_in_polygon_xy,
    estimate_facade_detection_defaults,
    facades_from_pointcloud_cgal,
    filter_facades_by_polygon,
    parse_kmz,
    polygon_to_enu,
    tight_footprint_from_cloud_xy,
    waypoints_to_enu,
)
from .manifold import from_manifold, register_to_kmz_frame
from .mission_intent import (
    read_intent_json,
    write_intent_json,
)
from .models import (
    ActionType,
    CameraAction,
    CameraName,
    MissionConfig,
    Waypoint,
)


# NEN-2767 post-processing — matches server endpoint /versions/{id}/rewrite-gimbals
_NEN_INSPECTION_SPEED_MS = 3.0
_NEN_MAX_FACADE_DIST_M = 60.0
_NEN_PITCH_MARGIN_DEG = 2.0

# Anomaly thresholds for preview-side flagging. The 2 m / 1 m fingerprint
# bench surfaced these as the "edge case" categories that warrant pilot
# attention: very-up pitches may be valid overhangs OR spurious upward-facing
# facets from CGAL extraction; very-down pitches may be valid base / floor
# OR ground points that bled into facet extraction. Real thresholds will be
# calibrated on the first few field flights.
_ANOMALY_PITCH_UP_DEG = 25.0
_ANOMALY_PITCH_DOWN_DEG = -85.0


def _gimbal_summary(waypoints: list[Waypoint]) -> dict:
    """Compact stats over the augmented waypoints — what rc-companion needs
    to render the preview screen and what the pilot should glance at before
    approving the mission."""
    pitches = [float(w.gimbal_pitch_deg) for w in waypoints]
    yaws = [
        float(w.gimbal_yaw_deg) if w.gimbal_yaw_deg is not None else float(w.heading_deg)
        for w in waypoints
    ]
    aimed = [w for w in waypoints if w.facade_index >= 0]
    pitch_up = sum(1 for p in pitches if p >= _ANOMALY_PITCH_UP_DEG)
    pitch_down = sum(1 for p in pitches if p <= _ANOMALY_PITCH_DOWN_DEG)
    if pitches:
        ps = sorted(pitches)
        n = len(ps)
        pitch_stats = {
            "min": ps[0],
            "max": ps[-1],
            "median": ps[n // 2],
            "p25": ps[n // 4],
            "p75": ps[3 * n // 4],
        }
    else:
        pitch_stats = {"min": 0.0, "max": 0.0, "median": 0.0, "p25": 0.0, "p75": 0.0}
    return {
        "waypoint_count": len(waypoints),
        "waypoints_aimed_at_facade": len(aimed),
        "waypoints_unmatched": len(waypoints) - len(aimed),
        "pitch_deg": pitch_stats,
        "yaw_unique_deg": len({round(y, 1) for y in yaws}),
        "anomaly_thresholds": {
            "pitch_up_deg": _ANOMALY_PITCH_UP_DEG,
            "pitch_down_deg": _ANOMALY_PITCH_DOWN_DEG,
        },
        "anomaly_counts": {
            "pitch_up": pitch_up,
            "pitch_down": pitch_down,
        },
        "anomaly_indices": {
            "pitch_up": [w.index for w in waypoints
                         if w.gimbal_pitch_deg >= _ANOMALY_PITCH_UP_DEG],
            "pitch_down": [w.index for w in waypoints
                           if w.gimbal_pitch_deg <= _ANOMALY_PITCH_DOWN_DEG],
        },
    }


_PILOT2_CARD_MAX_BYTES = 255  # DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN


def _format_pilot2_card(summary: dict) -> str:
    """Render the augment summary as a 7-line ASCII card that fits in the
    255-byte Pilot 2 floating-window budget. Shown after the KMZ is on the
    aircraft so the pilot can sanity-check before tapping Fly."""
    name = (summary.get("name") or "mission")[:24]
    flight_id = summary.get("flight_id") or "?"
    icp = summary.get("icp") or {}
    rmse = float(icp.get("icp_rmse_m") or 0.0)
    # Translation magnitude is the pilot-meaningful "did the algorithm land
    # near the right spot" signal. Final-scale fitness is misleading: it
    # measures correspondence at the 2 cm voxel resolution and naturally
    # collapses to a small number even on excellent registrations (the noise
    # floor of the dense Manifold cloud + sparse fingerprint pair). Reporting
    # drift in cm matches pilot intuition.
    trans = icp.get("total_translation_m") or [0.0, 0.0, 0.0]
    drift_m = float((trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2) ** 0.5)
    yaw_total = float(icp.get("total_yaw_deg") or 0.0)
    gs = summary.get("gimbal_stats") or {}
    total = int(gs.get("waypoint_count") or summary.get("waypoints_total") or 0)
    aimed = int(gs.get("waypoints_aimed_at_facade") or summary.get("waypoints_aimed") or 0)
    facets = int(summary.get("facades") or 0)
    aim_pct = (100.0 * aimed / total) if total else 0.0
    pitch = gs.get("pitch_deg") or {}
    p_med = float(pitch.get("median") or 0.0)
    p_min = float(pitch.get("min") or 0.0)
    p_max = float(pitch.get("max") or 0.0)
    lines = [
        "AeroScan ready",
        f"{name} -> {flight_id}",
        f"Align drift {drift_m * 100:.0f}cm  RMSE {rmse:.2f}m  yaw {yaw_total:+.0f}",
        f"WPs {total}  Facets {facets}  Aim {aim_pct:.0f}%",
        f"Pitch med {p_med:+.0f}  range {p_min:+.0f}..{p_max:+.0f}",
        "Custom gimbal: ENABLED",
        "Tap [AeroScan Fly] when ready",
    ]
    card = "\n".join(lines)
    if len(card.encode("utf-8")) > _PILOT2_CARD_MAX_BYTES:
        # Drop pitch range first (least critical), then fall back to a 3-line card.
        lines = [
            "AeroScan ready",
            f"{name} -> {flight_id}",
            f"WPs {total}  Aim {aim_pct:.0f}%  Custom gimbal ON",
            "Tap [AeroScan Fly] when ready",
        ]
        card = "\n".join(lines)
    return card


def _load_icp_target(icp_target_ply: Path) -> o3d.geometry.PointCloud:
    pc = o3d.io.read_point_cloud(str(icp_target_ply))
    if len(pc.points) == 0:
        raise SystemExit(f"ICP target cloud is empty: {icp_target_ply}")
    return pc


def _waypoints_from_intent(parsed: ImportedKmz) -> list[Waypoint]:
    enu_wps = waypoints_to_enu(
        parsed.waypoints, parsed.ref_lat, parsed.ref_lon, parsed.ref_alt,
    )
    return [
        Waypoint(
            x=w["x"], y=w["y"], z=w["z"],
            lat=w["lat"], lon=w["lon"], alt=w["alt"],
            heading_deg=w["heading_deg"],
            gimbal_pitch_deg=w["gimbal_pitch_deg"],
            gimbal_yaw_deg=w.get("gimbal_yaw_deg"),
            speed_ms=w["speed_ms"],
            index=i,
            facade_index=-1,
            actions=[CameraAction(action_type=ActionType.TAKE_PHOTO, camera=CameraName.WIDE)],
        )
        for i, w in enumerate(enu_wps)
    ]


def augment_mission(
    intent: ImportedKmz,
    flight_id: str,
    output_kmz: Path,
    icp_target_ply: Path,
    *,
    blackbox_dir: Path = Path("/blackbox"),
    voxel_m: float = 0.10,
    max_facade_distance_m: float = _NEN_MAX_FACADE_DIST_M,
    inspection_speed_ms: float = _NEN_INSPECTION_SPEED_MS,
    pitch_margin_deg: float = _NEN_PITCH_MARGIN_DEG,
    summary_json: Path | None = None,
    log: bool = True,
) -> dict:
    """Run the full intent-JSON → augmented-KMZ pipeline.

    Returns a stats dict; the output KMZ is written to ``output_kmz``.
    """
    def _log(msg: str) -> None:
        if log:
            print(msg, flush=True)

    t0 = time.monotonic()

    _log(f"[1/7] Mission intent: name={intent.name!r}  waypoints={len(intent.waypoints):,}  polygon vertices={len(intent.mission_area_wgs84)}")

    waypoints = _waypoints_from_intent(intent)
    polygon_enu = polygon_to_enu(intent.mission_area_wgs84, intent.ref_lat, intent.ref_lon, intent.ref_alt)

    _log(f"[2/7] Loading Manifold cloud from {blackbox_dir / flight_id}/dji_perception/1/")
    manifold_pc = from_manifold(flight_id, blackbox_dir=blackbox_dir, voxel_m=voxel_m)
    _log(f"      manifold cloud: {len(manifold_pc.points):,} pts (after {voxel_m*100:.0f} cm voxel)")

    _log(f"[3/7] Loading ICP target cloud: {icp_target_ply}")
    kmz_pc = _load_icp_target(icp_target_ply)
    _log(f"      icp target: {len(kmz_pc.points):,} pts")

    _log("[4/7] Registering Manifold → KMZ frame (multi-scale point-to-point ICP)…")
    registered, _T, icp_stats = register_to_kmz_frame(manifold_pc, kmz_pc)
    for s in icp_stats["icp_per_scale"]:
        _log(f"        {s['voxel_m']*100:.0f} cm   fitness {s['fitness']:.3f}   RMSE {s['rmse_m']:.3f} m")
    _log(f"      coarse yaw: {icp_stats['coarse_yaw_deg']:.0f}°   final RMSE: {icp_stats['icp_rmse_m']:.3f} m")

    # Mirror the dev backend's KMZ-import preprocessing exactly:
    #   1. Pass FULL pcd.points to CGAL (no pre-voxel, no pre-Z-floor).
    #   2. Compute a TIGHT polygon = convex hull of mid-height (30–70th
    #      pctile Z) points. Same as dev's _tighten_clip_polygon.
    #   3. Pass tight polygon to CGAL — CGAL clips internally and
    #      runs ground_skip_m=1.0 (drops bottom 1m of Z range).
    #   4. Post-extraction: drop facades whose centroid is outside the
    #      mission_area polygon expanded by 2m.
    #
    # /blackbox-specific note: this cloud is the raw perception scan
    # (~1.7M pts, includes ground), much larger than dev's curated DJI
    # cloud (~416K, building-only). CGAL extracts ~3x more facets from
    # /blackbox (3000 vs 1100). That's fine — verification shows the
    # gimbal picker is robust to facet count: more facets just means the
    # picker has more close candidates per WP, so aim error stays small
    # (max ~9° on Mijande). Don't pre-Z-floor; the extra mass doesn't
    # hurt accuracy.
    points_xyz = np.asarray(registered.points, dtype=np.float64)

    tight_poly = tight_footprint_from_cloud_xy(points_xyz)
    if tight_poly is not None and len(tight_poly) >= 3:
        clip_poly = tight_poly
        _log(f"      tight footprint: {len(tight_poly)} verts (mission polygon had {len(polygon_enu) if polygon_enu else 0})")
    else:
        clip_poly = polygon_enu
        _log("      tight footprint failed; falling back to mission polygon")
    _log(f"      facade-extraction input: {len(points_xyz):,} pts (raw, no Z-floor; CGAL handles ground_skip internally)")

    _log("[5/7] Extracting facades (CGAL region growing)…")
    # Density-aware auto-estimator — same call dev backend uses inside
    # _kmz_extract_best_facades. Scales epsilon/cluster_epsilon/min_points
    # to the cloud's actual NN spacing and surface density. Keep the
    # defaults so the augmenter's facet pool is identical to what the
    # dev viewer re-detects on the bundled cloud — otherwise a user
    # comparing viewer-visible facets against augmenter-picked targets
    # sees the augmenter "skipping" facets that were filtered out by
    # different thresholds.
    fd_kwargs = estimate_facade_detection_defaults(points_xyz)
    fd_kwargs.pop("_estimator", None)
    if fd_kwargs:
        _log("      auto-estimated CGAL knobs: "
             + ", ".join(f"{k}={v}" for k, v in fd_kwargs.items()))
    facades = facades_from_pointcloud_cgal(points_xyz, clip_poly, **fd_kwargs)
    _log(f"      facades after CGAL: {len(facades)}")

    # NOTE: vegetation mask removed — at 1 m fingerprint voxel resolution
    # the 2.5 m presence threshold drops some real building facets in
    # corners/concavities, so the picker falls back to a worse facet there
    # (visible in the dev viewer as gimbals pointing differently than the
    # dev path's `dji_pinned` rewrite which has no such mask). If we want
    # this back, we need either a denser fingerprint over the wire (so a
    # tighter threshold is workable) or a different vegetation-rejection
    # signal (e.g. inlier-coherence, density, or per-facet surface
    # roughness rather than positional proximity to fingerprint).

    # Post-extraction: drop facades whose centroid is outside the WPML
    # mission_area polygon expanded by 2m — same final gate the dev backend
    # applies (server/api.py line 1369).
    facades = filter_facades_by_polygon(
        facades,
        intent.mission_area_wgs84,
        intent.ref_lat, intent.ref_lon, intent.ref_alt,
    )
    _log(f"      facades after polygon filter: {len(facades)}")
    if not facades:
        raise SystemExit(
            "Facade extraction produced 0 facades — cannot augment gimbals. "
            "Possible causes: registered cloud is in the wrong frame, polygon "
            "filter dropped everything, or facade-detection thresholds are too tight."
        )

    _log("[6/7] Rewriting gimbals perpendicular to nearest facade…")
    new_waypoints = rewrite_gimbals_perpendicular(
        waypoints=waypoints,
        facades=facades,
        max_distance_m=max_facade_distance_m,
        pitch_margin_deg=pitch_margin_deg,
        preserve_heading=True,
    )
    # NEN-2767: stop-and-shoot at fixed speed, single TAKE_PHOTO per WP.
    # Matches server.api /versions/{id}/rewrite-gimbals exactly.
    for w in new_waypoints:
        w.speed_ms = inspection_speed_ms
        w.actions = [CameraAction(action_type=ActionType.TAKE_PHOTO, camera=CameraName.WIDE)]

    aimed = sum(1 for w in new_waypoints if w.facade_index >= 0)
    _log(f"      waypoints: {len(new_waypoints)}   re-aimed: {aimed}   unchanged (no facade in range): {len(new_waypoints) - aimed}")

    _log(f"[7/7] Writing augmented KMZ: {output_kmz}")
    config = MissionConfig(
        flight_speed_ms=inspection_speed_ms,
        mission_name=f"NEN-2767: {intent.name}",
        # Disable per-WP gimbal-action dedup. The default MissionConfig drops
        # any gimbalRotate whose pose is within 2° of the previous WP's, which
        # for inspection missions strips the gimbal action from most adjacent
        # WPs aiming at the same wall section. The dev viewer's parser then
        # sees no gimbalRotate for those WPs, falls back to the mission-level
        # default (yaw_base="aircraft", raw_yaw=0), and renders the gimbal
        # arrow pointing along the aircraft heading — which is the "gimbal
        # follows flight direction" artifact users see in the viewer. Setting
        # the threshold to a negative value forces every WP to emit its own
        # explicit gimbalRotate with the absolute pose. ~10-30% larger XML;
        # unambiguous parser/render.
        gimbal_dedup_threshold_deg=-1.0,
        heading_dedup_threshold_deg=-1.0,
    )
    output_kmz = Path(output_kmz)
    output_kmz.parent.mkdir(parents=True, exist_ok=True)
    # Bundle the *registered Manifold cloud* (the actual cloud the facade
    # extraction ran on, in the KMZ frame) into the output KMZ at
    # wpmz/res/ply/<name>/cloud.ply. Aircraft ignores it; the dev viewer
    # uses it for visual reconstruction and "Detect Facades" pass.
    # Bundling the ICP target (the 1 m fingerprint, ~130 KB) instead would
    # leave the dev viewer reconstructing a Swiss-cheese alpha-wrap mesh
    # off ~10K points — facades re-extracted in the viewer don't match
    # what the augmenter actually used. The Manifold cloud at 10 cm voxel
    # is ~1.7M pts ≈ 20 MB binary float xyz; aircraft upload over E-Port
    # USB-C handles that fine.
    bundled_cloud_pts = np.asarray(registered.points, dtype=np.float32)
    _bundled_n = len(bundled_cloud_pts)
    _hdr = (
        b"ply\n"
        b"format binary_little_endian 1.0\n"
        b"comment AeroScan augmenter: registered Manifold cloud in KMZ frame\n"
        + f"element vertex {_bundled_n}\n".encode()
        + b"property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    bundled_cloud = _hdr + bundled_cloud_pts.tobytes()
    _log(f"      bundling registered Manifold cloud: {_bundled_n:,} pts ({len(bundled_cloud):,} bytes)")
    # sfm_geo_desc.json — anchors the dev viewer's ENU origin so waypoints
    # render at the correct altitude. Without this, parse_kmz falls back to
    # using waypoints[0].alt as the origin → every waypoint sinks ~5 m and
    # the flight path appears inside the building. Mirrors the structure
    # Smart3D ships in its KMZs.
    sfm_geo_desc = {
        "cs_type": "LOCAL_ENU_CS",
        "offset": [0.0, 0.0, 0.0],
        "ref_GPS": {
            "altitude": float(intent.ref_alt),
            "latitude": float(intent.ref_lat),
            "longitude": float(intent.ref_lon),
        },
        "rvec": [0.0, 0.0, 0.0],
        "scale": 1.0,
    }
    out_path = build_kmz(
        new_waypoints, config, str(output_kmz),
        bundled_cloud_ply=bundled_cloud,
        mission_area_wgs84=intent.mission_area_wgs84,
        bundled_sfm_geo_desc=sfm_geo_desc,
    )
    out_size = Path(out_path).stat().st_size
    _log(f"      done. {out_size:,} bytes")

    # Lean copy (no bundled cloud) — what we upload to the aircraft via
    # DjiWaypointV3_UploadKmzFile. The full KMZ above has the registered
    # Manifold cloud baked in for rc-companion's preview viewer; the aircraft
    # only needs waypoints + gimbal commands + the mission-area polygon, so
    # stripping the 7 MB cloud cuts the over-air upload from ~10 min to ~30 s
    # and keeps Pilot 2's widget channel from saturating during the transfer.
    lean_path = Path(str(output_kmz).replace(".kmz", ".lean.kmz"))
    build_kmz(
        new_waypoints, config, str(lean_path),
        bundled_cloud_ply=None,
        mission_area_wgs84=intent.mission_area_wgs84,
        bundled_sfm_geo_desc=sfm_geo_desc,
    )
    lean_size = lean_path.stat().st_size
    _log(f"      lean (no cloud) → {lean_path.name} ({lean_size:,} bytes)")

    elapsed = time.monotonic() - t0
    _log(f"Total: {elapsed:.1f} s")

    summary = {
        "schema_version": 1,
        "name": intent.name,
        "flight_id": flight_id,
        "output_kmz": out_path,
        "output_kmz_lean": str(lean_path),
        "output_bytes": out_size,
        "output_bytes_lean": lean_size,
        "waypoints_total": len(new_waypoints),
        "waypoints_aimed": aimed,
        "facades": len(facades),
        "gimbal_stats": _gimbal_summary(new_waypoints),
        "icp": {
            "coarse_yaw_deg": icp_stats["coarse_yaw_deg"],
            "icp_rmse_m": icp_stats["icp_rmse_m"],
            "icp_fitness": icp_stats["icp_fitness"],
            "total_yaw_deg": icp_stats["total_yaw_deg"],
            "total_translation_m": icp_stats["total_translation_m"],
        },
        "icp_per_scale": icp_stats["icp_per_scale"],
        "elapsed_s": elapsed,
    }

    if summary_json is not None:
        summary_json = Path(summary_json)
        summary_json.parent.mkdir(parents=True, exist_ok=True)
        summary_json.write_text(json.dumps(summary, indent=2))
        _log(f"      summary → {summary_json} ({summary_json.stat().st_size:,} bytes)")
        card_path = summary_json.with_suffix(".card.txt")
        card_path.write_text(_format_pilot2_card(summary))
        _log(f"      pilot2 card → {card_path} ({card_path.stat().st_size} bytes)")

    return summary


# ---------------------------------------------------------------------------
# Subcommand handlers
# ---------------------------------------------------------------------------


def _cmd_kmz_to_json(args: argparse.Namespace) -> int:
    """Split a Smart3D KMZ into compact JSON intent + ICP target PLY.

    Produces exactly what rc-companion will produce on the RC, plus the
    cloud.ply pulled out as a sibling file. The Manifold-side flow accepts
    those two paths via ``augment-mission --mission-json … --icp-target-ply …``.
    """
    src: Path = args.source_kmz
    print(f"Parsing {src}", flush=True)
    parsed = parse_kmz(src.read_bytes(), name=args.name or src.stem)
    print(f"  waypoints: {len(parsed.waypoints):,}   polygon: {len(parsed.mission_area_wgs84)} verts   "
          f"cloud.ply: {len(parsed.point_cloud_ply or b''):,} bytes")

    intent_path: Path = args.json_out
    write_intent_json(parsed, intent_path, indent=2 if args.pretty else None)
    intent_size = intent_path.stat().st_size
    print(f"Wrote {intent_path} ({intent_size:,} bytes)")

    if args.cloud_out:
        cloud_path: Path = args.cloud_out
        if not parsed.point_cloud_ply:
            raise SystemExit(f"Source KMZ has no cloud.ply — cannot write {cloud_path}")
        cloud_path.parent.mkdir(parents=True, exist_ok=True)
        cloud_path.write_bytes(parsed.point_cloud_ply)
        print(f"Wrote {cloud_path} ({cloud_path.stat().st_size:,} bytes)")

    return 0


def _cmd_augment_mission(args: argparse.Namespace) -> int:
    intent_json: Optional[Path] = args.mission_json
    source_kmz: Optional[Path] = args.source_kmz

    if (intent_json is None) == (source_kmz is None):
        raise SystemExit(
            "augment-mission requires exactly one of --mission-json (production "
            "shape) or --source-kmz (dev shortcut: parses + extracts cloud.ply "
            "from the KMZ on the fly)."
        )

    icp_target: Optional[Path] = args.icp_target_ply

    if intent_json is not None:
        intent = read_intent_json(intent_json)
        if icp_target is None:
            raise SystemExit(
                "--mission-json mode requires --icp-target-ply (the JSON "
                "intent intentionally does not embed the cloud.ply)."
            )
        cleanup_target: Optional[Path] = None
    else:
        # Dev shortcut: parse the KMZ ourselves, persist its cloud.ply to a
        # tempfile so the registration code path is identical.
        parsed = parse_kmz(source_kmz.read_bytes(), name=source_kmz.stem)
        intent = parsed
        if icp_target is None:
            if not parsed.point_cloud_ply:
                raise SystemExit("--source-kmz has no cloud.ply and no --icp-target-ply override.")
            tf = tempfile.NamedTemporaryFile(suffix=".ply", delete=False)
            tf.write(parsed.point_cloud_ply)
            tf.close()
            icp_target = Path(tf.name)
            cleanup_target = icp_target
        else:
            cleanup_target = None

    try:
        stats = augment_mission(
            intent=intent,
            flight_id=args.flight_id,
            output_kmz=args.output_kmz,
            icp_target_ply=icp_target,
            blackbox_dir=args.blackbox_dir,
            voxel_m=args.voxel_m,
            max_facade_distance_m=args.max_facade_distance_m,
            inspection_speed_ms=args.inspection_speed_ms,
            pitch_margin_deg=args.pitch_margin_deg,
            summary_json=args.summary_json,
            log=not args.json,
        )
    finally:
        if cleanup_target is not None:
            cleanup_target.unlink(missing_ok=True)

    if args.json:
        # stats already has the trimmed icp; print as-is.
        json.dump(stats, sys.stdout, indent=2, default=str)
        sys.stdout.write("\n")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="flight_planner.cli",
        description="AeroScan CLI — Manifold-resident mission augmentation.",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    # kmz-to-json
    k = sub.add_parser(
        "kmz-to-json",
        help="Split a Smart3D KMZ into a compact mission-intent JSON + cloud.ply (laptop-side dev tool).",
    )
    k.add_argument("--source-kmz", type=Path, required=True,
                   help="Smart3D KMZ to split.")
    k.add_argument("--json-out", type=Path, required=True,
                   help="Path to write the mission-intent JSON.")
    k.add_argument("--cloud-out", type=Path, default=None,
                   help="Optional: write cloud.ply alongside the JSON (used by "
                        "augment-mission as the ICP target).")
    k.add_argument("--name", type=str, default=None,
                   help="Override the mission name (default: derived from KMZ filename).")
    k.add_argument("--pretty", action="store_true",
                   help="Pretty-print the JSON. Default: compact (production).")
    k.set_defaults(func=_cmd_kmz_to_json)

    # augment-mission
    am = sub.add_parser(
        "augment-mission",
        help="Take a mission-intent JSON + Manifold flight; produce an NEN-2767 augmented KMZ.",
    )
    src_group = am.add_mutually_exclusive_group(required=True)
    src_group.add_argument("--mission-json", type=Path, default=None,
                           help="Mission-intent JSON (production shape; rc-companion produces this).")
    src_group.add_argument("--source-kmz", type=Path, default=None,
                           help="Smart3D KMZ (dev shortcut: parses to intent on the fly).")
    am.add_argument("--icp-target-ply", type=Path, default=None,
                    help="Path to the KMZ's cloud.ply (or any cloud in the KMZ frame). "
                         "Required with --mission-json. With --source-kmz, defaults to the "
                         "KMZ's embedded cloud.ply.")
    am.add_argument("--flight-id", type=str, default="the_latest_flight",
                    help="Manifold flight directory name under --blackbox-dir "
                         "(default: the_latest_flight symlink).")
    am.add_argument("--output-kmz", type=Path, required=True,
                    help="Path to write the augmented KMZ.")
    am.add_argument("--blackbox-dir", type=Path, default=Path("/blackbox"),
                    help="Manifold blackbox root (default: /blackbox).")
    am.add_argument("--voxel-m", type=float, default=0.10,
                    help="Voxel-downsample size for the Manifold cloud (m). Default 0.10.")
    am.add_argument("--max-facade-distance-m", type=float, default=_NEN_MAX_FACADE_DIST_M,
                    help=f"Max waypoint→facade distance for re-aiming (m). "
                         f"Waypoints further away keep their original gimbal. "
                         f"Default {_NEN_MAX_FACADE_DIST_M}.")
    am.add_argument("--inspection-speed-ms", type=float, default=_NEN_INSPECTION_SPEED_MS,
                    help=f"Per-waypoint flight speed (m/s). Default {_NEN_INSPECTION_SPEED_MS}.")
    am.add_argument("--pitch-margin-deg", type=float, default=_NEN_PITCH_MARGIN_DEG,
                    help=f"Margin from gimbal hardware pitch limits (deg). Default {_NEN_PITCH_MARGIN_DEG}.")
    am.add_argument("--summary-json", type=Path, default=None,
                    help="Optional: write a structured summary JSON alongside the output KMZ. "
                         "kmz_runner.c reads this and ships it back to rc-companion in the "
                         "PRVW preview frame so the pilot can review before approving.")
    am.add_argument("--json", action="store_true",
                    help="Emit a JSON stats blob on stdout instead of human progress lines.")
    am.set_defaults(func=_cmd_augment_mission)

    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
