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
    facades_from_pointcloud_cgal,
    parse_kmz,
    polygon_to_enu,
    waypoints_to_enu,
)
from .manifold import from_manifold, register_to_kmz_frame
from .mission_intent import (
    SCHEMA_VERSION,
    imported_kmz_to_intent_dict,
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

    _log("[5/7] Extracting facades (CGAL region growing)…")
    facades = facades_from_pointcloud_cgal(
        np.asarray(registered.points, dtype=np.float64),
        polygon_enu if polygon_enu else None,
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
    out_path = build_kmz(new_waypoints, config, str(output_kmz))
    out_size = Path(out_path).stat().st_size
    _log(f"      done. {out_size:,} bytes")

    elapsed = time.monotonic() - t0
    _log(f"Total: {elapsed:.1f} s")

    return {
        "name": intent.name,
        "flight_id": flight_id,
        "output_kmz": out_path,
        "output_bytes": out_size,
        "waypoints_total": len(new_waypoints),
        "waypoints_aimed": aimed,
        "facades": len(facades),
        "icp": icp_stats,
        "elapsed_s": elapsed,
    }


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
            log=not args.json,
        )
    finally:
        if cleanup_target is not None:
            cleanup_target.unlink(missing_ok=True)

    if args.json:
        out = {k: v for k, v in stats.items() if k != "icp"}
        out["icp"] = {
            "coarse_yaw_deg": stats["icp"]["coarse_yaw_deg"],
            "icp_rmse_m": stats["icp"]["icp_rmse_m"],
            "icp_fitness": stats["icp"]["icp_fitness"],
        }
        json.dump(out, sys.stdout, indent=2)
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
