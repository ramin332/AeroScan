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
    facades_from_pointcloud_cgal,
    parse_kmz,
    polygon_to_enu,
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

    _log("[4/7] Registering Manifold → KMZ frame (multi-scale point-to-plane ICP)…")
    registered, _T, icp_stats = register_to_kmz_frame(manifold_pc, kmz_pc)
    for s in icp_stats["icp_per_scale"]:
        _log(f"        {s['voxel_m']*100:.0f} cm   fitness {s['fitness']:.3f}   RMSE {s['rmse_m']:.3f} m")
    _log(f"      coarse yaw: {icp_stats['coarse_yaw_deg']:.0f}°   final RMSE: {icp_stats['icp_rmse_m']:.3f} m")

    # Match dev frontend's input characteristics before facade extraction.
    # /blackbox is everything-within-sensor-range (1.7M pts after 10 cm voxel
    # — 4× denser than dev's curated KMZ cloud and full of ground/surrounding
    # noise). DJI's KMZ cloud is pre-trimmed to building-only at ~10 cm.
    # Without these three steps the CGAL extractor sees too much non-building
    # mass and emits ~5× more roof+ground facets than walls (369 walls vs
    # 1821 roofs vs 689 tilted on Mijande), which biases the gimbal picker
    # toward roofs and produces too-much-up/too-much-down complaints.
    pre_pts = len(registered.points)
    n_after_poly = pre_pts
    z_floor = None
    # 1) Polygon clip in XY: drop scan noise outside the mission area.
    if polygon_enu and len(polygon_enu) >= 3:
        poly_xy = np.array([(p[0], p[1]) for p in polygon_enu], dtype=np.float64)
        pts = np.asarray(registered.points)
        mask = _points_in_polygon_xy(pts[:, :2], poly_xy)
        if mask.sum() > 0:
            registered = o3d.geometry.PointCloud()
            registered.points = o3d.utility.Vector3dVector(pts[mask])
            n_after_poly = int(mask.sum())
    # 2) Z-floor: drop ground points (5th-percentile Z + 1m buffer is a
    # robust "ground level" estimate that survives outliers).
    pts = np.asarray(registered.points)
    if len(pts) > 0:
        z_floor = float(np.percentile(pts[:, 2], 5)) + 1.0
        above = pts[pts[:, 2] > z_floor]
        if len(above) > 0:
            registered = o3d.geometry.PointCloud()
            registered.points = o3d.utility.Vector3dVector(above)
    # 3) Revoxel to dev's target density (~10 cm) so CGAL's region-growing
    # k-NN sees comparable neighborhood density. The 10 cm voxel earlier
    # was pre-ICP; this one shapes the extractor input.
    registered = registered.voxel_down_sample(0.10)
    _log(
        f"      dev-match prep: {pre_pts:,} pts → polygon-clip {n_after_poly:,} → "
        f"z>{z_floor:.2f}m & revoxel(10cm) → {len(registered.points):,} pts"
        if z_floor is not None
        else f"      dev-match prep: {pre_pts:,} → {len(registered.points):,} pts (polygon clip + revoxel(10cm))"
    )

    _log("[5/7] Extracting facades (CGAL region growing)…")
    # Polygon already pre-applied above as a hard XY clip — pass None so the
    # extractor's centroid-based polygon test doesn't double-filter.
    facades = facades_from_pointcloud_cgal(
        np.asarray(registered.points, dtype=np.float64),
        None,
    )
    _log(f"      facades: {len(facades)}")
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
    )
    output_kmz = Path(output_kmz)
    output_kmz.parent.mkdir(parents=True, exist_ok=True)
    # Bundle the ICP target cloud (the same cloud the augment used) into the
    # output KMZ at wpmz/res/ply/<name>/cloud.ply. Aircraft ignores it;
    # makes the artifact self-contained so it can be imported directly into
    # the dev frontend (server.api._process expects a Smart3D-style KMZ
    # with the cloud bundled). Adds the icp-target's bytes to the output —
    # for a 1 m voxel fingerprint that's ~150 KB; for a full curated cloud
    # it's ~13 MB.
    bundled_cloud = icp_target_ply.read_bytes()
    out_path = build_kmz(new_waypoints, config, str(output_kmz),
                         bundled_cloud_ply=bundled_cloud)
    out_size = Path(out_path).stat().st_size
    _log(f"      done. {out_size:,} bytes")

    elapsed = time.monotonic() - t0
    _log(f"Total: {elapsed:.1f} s")

    summary = {
        "schema_version": 1,
        "name": intent.name,
        "flight_id": flight_id,
        "output_kmz": out_path,
        "output_bytes": out_size,
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
