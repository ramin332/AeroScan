#!/usr/bin/env python3
"""End-to-end validation: does the full augment pipeline produce the same
gimbal angles when fed a 2 m voxel fingerprint vs the full cloud?

The ICP-target-density bench (scripts/bench_icp_target_density.py) verified
that the 2 m fingerprint yields a registration transform within 0.013° of the
full cloud. This script verifies that downstream the full pipeline (registered
cloud → facade extraction → waypoint→facade match → gimbal computation) also
ends up at the same gimbal angles.

Pipeline stages tested:
1. Build a 2 m voxel-downsampled cloud from a full KMZ cloud.ply.
2. Run augment-mission against it.
3. Run augment-mission against the full cloud (baseline).
4. Diff the per-waypoint gimbal pitch / yaw between the two outputs.

Pass: > 99 % of waypoints have |Δpitch| < 0.5° AND |Δyaw| < 0.5°.

Usage (on Manifold):
    mamba run -n aero-scan python scripts/validate_2m_voxel_e2e.py \\
        --intent /open_app/dev/data/missions/Mijande.intent.json \\
        --full-cloud /open_app/dev/data/missions/Mijande.cloud.ply \\
        --voxel-m 2.0 \\
        --flight-id flight0016 \\
        --workdir /open_app/dev/data/missions/validate_2m
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

import open3d as o3d


WPML_NS = "{http://www.dji.com/wpmz/1.0.6}"
KML_NS = "{http://www.opengis.net/kml/2.2}"


def extract_gimbals_from_kmz(kmz_path: Path) -> list[dict]:
    """Pull (waypoint_index, pitch, yaw, lon, lat, alt) from a built KMZ.
    Yaw is reported as the per-waypoint absolute angle (post-augment).
    """
    with zipfile.ZipFile(kmz_path) as zf:
        wpml = zf.read("wpmz/waylines.wpml")
    root = ET.fromstring(wpml)

    out: list[dict] = []
    for placemark in root.iter(f"{KML_NS}Placemark"):
        idx_el = placemark.find(f"{WPML_NS}index")
        if idx_el is None:
            continue
        coords_el = placemark.find(f"{KML_NS}Point/{KML_NS}coordinates")
        lon = lat = alt = None
        if coords_el is not None and coords_el.text:
            parts = coords_el.text.strip().split(",")
            if len(parts) >= 2:
                lon, lat = float(parts[0]), float(parts[1])
        gimbal_pitch = None
        gimbal_yaw = None
        for action in placemark.iter(f"{WPML_NS}action"):
            funcparam = action.find(f"{WPML_NS}actionActuatorFuncParam")
            if funcparam is None:
                continue
            pitch_el = funcparam.find(f"{WPML_NS}gimbalPitchRotateAngle")
            yaw_el = funcparam.find(f"{WPML_NS}gimbalYawRotateAngle")
            if pitch_el is not None and pitch_el.text is not None:
                try:
                    gimbal_pitch = float(pitch_el.text)
                except ValueError:
                    pass
            if yaw_el is not None and yaw_el.text is not None:
                try:
                    gimbal_yaw = float(yaw_el.text)
                except ValueError:
                    pass
        height_el = placemark.find(f"{WPML_NS}executeHeight")
        try:
            alt = float(height_el.text) if height_el is not None and height_el.text else None
        except ValueError:
            alt = None
        out.append({
            "index": int(idx_el.text),
            "lon": lon, "lat": lat, "alt": alt,
            "gimbal_pitch_deg": gimbal_pitch,
            "gimbal_yaw_deg": gimbal_yaw,
        })
    return out


def diff_gimbals(baseline: list[dict], test: list[dict]) -> dict:
    """Per-waypoint disagreement stats. Yaw uses circular distance."""
    by_idx_b = {wp["index"]: wp for wp in baseline}
    by_idx_t = {wp["index"]: wp for wp in test}
    common = sorted(set(by_idx_b) & set(by_idx_t))
    if not common:
        raise SystemExit("No matching waypoint indices between the two KMZs")

    pitch_deltas: list[float] = []
    yaw_deltas: list[float] = []
    over_05 = 0
    over_10 = 0
    over_50 = 0
    for i in common:
        b, t = by_idx_b[i], by_idx_t[i]
        if b["gimbal_pitch_deg"] is None or t["gimbal_pitch_deg"] is None:
            continue
        if b["gimbal_yaw_deg"] is None or t["gimbal_yaw_deg"] is None:
            continue
        dp = abs(t["gimbal_pitch_deg"] - b["gimbal_pitch_deg"])
        dy = abs(((t["gimbal_yaw_deg"] - b["gimbal_yaw_deg"]) + 180.0) % 360.0 - 180.0)
        pitch_deltas.append(dp)
        yaw_deltas.append(dy)
        big = max(dp, dy)
        if big >= 0.5:
            over_05 += 1
        if big >= 1.0:
            over_10 += 1
        if big >= 5.0:
            over_50 += 1

    n = len(pitch_deltas)
    if n == 0:
        raise SystemExit("No gimbal-pair samples to compare")
    pitch_sorted = sorted(pitch_deltas)
    yaw_sorted = sorted(yaw_deltas)

    return {
        "n_waypoints_compared": n,
        "n_baseline_only": len(by_idx_b) - len(common),
        "n_test_only": len(by_idx_t) - len(common),
        "pitch_max": pitch_sorted[-1],
        "pitch_p99": pitch_sorted[int(0.99 * n)],
        "pitch_p95": pitch_sorted[int(0.95 * n)],
        "pitch_median": pitch_sorted[n // 2],
        "yaw_max": yaw_sorted[-1],
        "yaw_p99": yaw_sorted[int(0.99 * n)],
        "yaw_p95": yaw_sorted[int(0.95 * n)],
        "yaw_median": yaw_sorted[n // 2],
        "n_over_0_5_deg": over_05,
        "n_over_1_0_deg": over_10,
        "n_over_5_0_deg": over_50,
        "pct_over_0_5_deg": 100.0 * over_05 / n,
    }


def voxel_downsample_ply(src: Path, dst: Path, voxel_m: float) -> tuple[int, int]:
    pc = o3d.io.read_point_cloud(str(src))
    n_in = len(pc.points)
    pc_down = pc.voxel_down_sample(voxel_m)
    n_out = len(pc_down.points)
    dst.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(dst), pc_down, write_ascii=False)
    return n_in, n_out


def run_augment(intent: Path, cloud: Path, flight_id: str, output: Path) -> None:
    cmd = [
        sys.executable, "-m", "flight_planner.cli", "augment-mission",
        "--mission-json", str(intent),
        "--icp-target-ply", str(cloud),
        "--flight-id", flight_id,
        "--output-kmz", str(output),
    ]
    print(f"  $ {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, check=True)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--intent", type=Path, required=True)
    p.add_argument("--full-cloud", type=Path, required=True)
    p.add_argument("--voxel-m", type=float, default=2.0)
    p.add_argument("--flight-id", type=str, default="flight0016")
    p.add_argument("--workdir", type=Path, required=True)
    args = p.parse_args()

    args.workdir.mkdir(parents=True, exist_ok=True)
    fingerprint = args.workdir / f"cloud.{int(args.voxel_m*100)}cm.ply"
    out_full = args.workdir / "augmented.full.kmz"
    out_test = args.workdir / f"augmented.{int(args.voxel_m*100)}cm.kmz"

    t0 = time.monotonic()
    print(f"[1/4] Voxel-downsample {args.full_cloud} → {fingerprint} at {args.voxel_m} m …")
    n_in, n_out = voxel_downsample_ply(args.full_cloud, fingerprint, args.voxel_m)
    fp_size = fingerprint.stat().st_size
    print(f"      {n_in:,} → {n_out:,} pts   ({fp_size:,} bytes raw)")

    print(f"\n[2/4] Augment with FULL cloud (baseline)…")
    t = time.monotonic()
    run_augment(args.intent, args.full_cloud, args.flight_id, out_full)
    print(f"      {out_full.stat().st_size:,} bytes  ({time.monotonic()-t:.1f} s)")

    print(f"\n[3/4] Augment with {args.voxel_m} m fingerprint…")
    t = time.monotonic()
    run_augment(args.intent, fingerprint, args.flight_id, out_test)
    print(f"      {out_test.stat().st_size:,} bytes  ({time.monotonic()-t:.1f} s)")

    print(f"\n[4/4] Diff gimbals between the two outputs…")
    baseline_g = extract_gimbals_from_kmz(out_full)
    test_g = extract_gimbals_from_kmz(out_test)
    print(f"      baseline waypoints w/ gimbal: {sum(1 for w in baseline_g if w['gimbal_pitch_deg'] is not None):,}")
    print(f"      test     waypoints w/ gimbal: {sum(1 for w in test_g if w['gimbal_pitch_deg'] is not None):,}")
    stats = diff_gimbals(baseline_g, test_g)

    print()
    print("=" * 70)
    print(f"E2E VALIDATION:  full cloud  vs  {args.voxel_m} m fingerprint")
    print("=" * 70)
    print(f"  waypoints compared:    {stats['n_waypoints_compared']:,}")
    print(f"  pitch  delta (deg):    median {stats['pitch_median']:.3f}   p95 {stats['pitch_p95']:.3f}   p99 {stats['pitch_p99']:.3f}   max {stats['pitch_max']:.3f}")
    print(f"  yaw    delta (deg):    median {stats['yaw_median']:.3f}   p95 {stats['yaw_p95']:.3f}   p99 {stats['yaw_p99']:.3f}   max {stats['yaw_max']:.3f}")
    print(f"  waypoints w/ |Δ| ≥ 0.5°: {stats['n_over_0_5_deg']:,}  ({stats['pct_over_0_5_deg']:.2f}%)")
    print(f"  waypoints w/ |Δ| ≥ 1.0°: {stats['n_over_1_0_deg']:,}")
    print(f"  waypoints w/ |Δ| ≥ 5.0°: {stats['n_over_5_0_deg']:,}")
    print("=" * 70)
    print(f"Total elapsed: {time.monotonic()-t0:.1f} s")

    # Pass criterion: at least 99 % of waypoints within 0.5° on both axes
    pass_pct = 100.0 - stats["pct_over_0_5_deg"]
    if pass_pct >= 99.0:
        print(f"\nPASS — {pass_pct:.2f}% of waypoints within 0.5° on both axes.")
        return 0
    else:
        print(f"\nFAIL — only {pass_pct:.2f}% of waypoints within 0.5° "
              f"(threshold 99 %). Investigate.")
        return 2


if __name__ == "__main__":
    sys.exit(main())
