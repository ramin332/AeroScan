#!/usr/bin/env python3
"""Verify an augmented KMZ against the bundled point cloud.

For every waypoint, compute the gimbal look ray and check what (if
anything) the camera is actually aimed at:

* hit / miss        — is there a cloud point within the camera frustum?
* aim error         — angle between (look ray) and (vector from WP to
                      nearest cloud point), so ~0° means dead-on.
* nearest distance  — how far is the nearest hit point from the WP?
* roll-up stats     — % aimed at building, % at empty sky/ground,
                      pitch/yaw distribution, adjacent-WP smoothness.

Inputs come straight from the .with_cloud.kmz the dev viewer loads, so
this checks what the user is actually seeing.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import numpy as np


WPML_NS = "{http://www.dji.com/wpmz/1.0.6}"
KML_NS = "{http://www.opengis.net/kml/2.2}"


def _findtext(el, paths):
    for p in paths:
        x = el.find(p)
        if x is not None and x.text is not None:
            return x.text.strip()
    return None


def _parse_geo_desc(zf: zipfile.ZipFile) -> tuple[float, float, float]:
    for name in zf.namelist():
        if name.lower().endswith("sfm_geo_desc.json"):
            data = json.loads(zf.read(name))
            ref = data.get("ref_GPS") or data.get("refGPS") or {}
            return (
                float(ref["latitude"]),
                float(ref["longitude"]),
                float(ref["altitude"]),
            )
    raise SystemExit("No sfm_geo_desc.json in KMZ — cannot anchor cloud to WPs.")


def _wgs84_to_enu(
    lat: float, lon: float, alt: float,
    ref_lat: float, ref_lon: float, ref_alt: float,
) -> tuple[float, float, float]:
    """Quick local ENU conversion. Good for sub-km radii."""
    R = 6378137.0
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    east = dlon * R * math.cos(math.radians(ref_lat))
    north = dlat * R
    up = alt - ref_alt
    return east, north, up


def _parse_waypoints(zf: zipfile.ZipFile, ref_lat, ref_lon, ref_alt):
    """Returns list of dicts: {x,y,z (ENU), pitch, yaw, heading, idx}."""
    for name in zf.namelist():
        if name.lower().endswith("waylines.wpml") or name.lower().endswith("template.kml"):
            tree = ET.parse(io.BytesIO(zf.read(name)))
            root = tree.getroot()
            placemarks = root.iter(f"{KML_NS}Placemark")
            wps = []
            for pm in placemarks:
                # Coords
                coords_el = pm.find(f".//{KML_NS}coordinates")
                if coords_el is None or coords_el.text is None:
                    continue
                parts = coords_el.text.strip().split(",")
                if len(parts) < 2:
                    continue
                lon = float(parts[0])
                lat = float(parts[1])
                # Altitude: prefer ellipsoidHeight (absolute), else executeHeight.
                alt_str = _findtext(pm, [
                    f".//{WPML_NS}ellipsoidHeight",
                    f".//{WPML_NS}executeHeight",
                ])
                alt = float(alt_str) if alt_str else 0.0

                # Heading
                heading_str = _findtext(pm, [
                    f".//{WPML_NS}waypointHeadingAngle",
                    f".//{WPML_NS}aircraftHeading",
                ])
                heading = float(heading_str) if heading_str else 0.0

                # Gimbal pitch (waypoint-inline)
                pitch_str = _findtext(pm, [
                    f".//{WPML_NS}gimbalPitchAngle",
                    f".//{WPML_NS}waypointGimbalPitchAngle",
                ])
                # Gimbal yaw — search inline AND inside actionGroup actions
                yaw_str = _findtext(pm, [
                    f".//{WPML_NS}gimbalYawAngle",
                    f".//{WPML_NS}waypointGimbalYawAngle",
                ])

                # Action-based gimbal pose (DJI's primary path for Smart3D and
                # for our augmenter when gimbalRotate actions are emitted)
                if pitch_str is None or yaw_str is None:
                    for action in pm.iter(f"{WPML_NS}action"):
                        func_el = action.find(f"{WPML_NS}actionActuatorFunc")
                        if func_el is None:
                            continue
                        if func_el.text != "gimbalRotate":
                            continue
                        param = action.find(f"{WPML_NS}actionActuatorFuncParam")
                        if param is None:
                            continue
                        if pitch_str is None:
                            p = param.find(f"{WPML_NS}gimbalPitchRotateAngle")
                            if p is not None and p.text is not None:
                                pitch_str = p.text.strip()
                        if yaw_str is None:
                            y = param.find(f"{WPML_NS}gimbalYawRotateAngle")
                            if y is not None and y.text is not None:
                                yaw_str = y.text.strip()

                pitch = float(pitch_str) if pitch_str else 0.0
                yaw = float(yaw_str) if yaw_str else heading

                x, y, z = _wgs84_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
                wps.append(dict(
                    x=x, y=y, z=z,
                    pitch=pitch, yaw=yaw, heading=heading,
                    lat=lat, lon=lon, alt=alt,
                ))
            if wps:
                return wps
    raise SystemExit("No placemarks with coords in KMZ.")


def _parse_ply_xyz(zf: zipfile.ZipFile) -> np.ndarray:
    """Read the bundled cloud.ply, return Nx3 float32 ENU."""
    for name in zf.namelist():
        if name.lower().endswith("cloud.ply"):
            data = zf.read(name)
            # Binary little-endian PLY parser. We only need the XYZ vertices.
            header_end = data.find(b"end_header\n")
            if header_end < 0:
                raise SystemExit(f"PLY {name} has no end_header — bad file.")
            header = data[:header_end].decode("ascii", errors="replace")
            n_vertices = 0
            stride = 0
            xyz_offsets = []
            in_vertex = False
            cur_offset = 0
            for line in header.splitlines():
                line = line.strip()
                if line.startswith("element vertex"):
                    n_vertices = int(line.split()[2])
                    in_vertex = True
                    cur_offset = 0
                    xyz_offsets = []
                    stride = 0
                elif line.startswith("element ") and not line.startswith("element vertex"):
                    in_vertex = False
                elif line.startswith("property") and in_vertex:
                    parts = line.split()
                    ptype = parts[1]
                    pname = parts[-1]
                    size = {"float": 4, "double": 8, "uchar": 1, "uint8": 1, "char": 1, "int8": 1, "ushort": 2, "uint16": 2, "short": 2, "int16": 2, "uint": 4, "uint32": 4, "int": 4, "int32": 4}[ptype]
                    if pname in ("x", "y", "z"):
                        xyz_offsets.append((pname, cur_offset, size, ptype))
                    cur_offset += size
                    stride = cur_offset
            if not n_vertices:
                raise SystemExit("PLY: no vertices declared.")
            if len(xyz_offsets) != 3:
                raise SystemExit(f"PLY: did not find x/y/z float properties in vertex element (found {xyz_offsets}).")
            body = data[header_end + len(b"end_header\n"):]
            arr = np.frombuffer(body[: n_vertices * stride], dtype=np.uint8)
            arr = arr.reshape(n_vertices, stride)
            xyz = np.empty((n_vertices, 3), dtype=np.float32)
            for ax, (pname, offset, size, ptype) in enumerate(sorted(xyz_offsets, key=lambda t: "xyz".index(t[0]))):
                if ptype == "float":
                    xyz[:, ax] = arr[:, offset:offset + 4].view(np.float32).reshape(-1)
                elif ptype == "double":
                    xyz[:, ax] = arr[:, offset:offset + 8].view(np.float64).astype(np.float32).reshape(-1)
                else:
                    raise SystemExit(f"PLY xyz unexpected type {ptype}")
            return xyz
    raise SystemExit("No cloud.ply in KMZ.")


def _look_dir(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    """Convert (yaw bearing from north, pitch from horizontal) → unit vector
    in ENU (x=East, y=North, z=Up). Camera looks along this ray.
    """
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    horiz = math.cos(pitch_rad)
    return np.array([
        horiz * math.sin(yaw_rad),  # east
        horiz * math.cos(yaw_rad),  # north
        math.sin(pitch_rad),        # up
    ], dtype=np.float64)


def _angular_diff_deg(a: float, b: float) -> float:
    """Smallest signed angle a - b on the circle, in degrees."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kmz", type=Path,
                    help="Augmented KMZ with bundled cloud.ply + sfm_geo_desc.json")
    ap.add_argument("--max-aim-deg", type=float, default=15.0,
                    help="Aim error threshold in degrees: WPs whose nearest "
                         "in-frustum cloud point is within this angle of the "
                         "look ray are 'on target'. Default 15°.")
    ap.add_argument("--frustum-deg", type=float, default=45.0,
                    help="Half-angle of the cone defining 'in front of camera'. "
                         "Points outside the cone are not candidates. Default 45°.")
    ap.add_argument("--max-range-m", type=float, default=60.0,
                    help="Maximum cloud-point range to consider as a hit. "
                         "Default 60 m.")
    ap.add_argument("--show-worst", type=int, default=10,
                    help="Print the N worst-aimed WPs.")
    args = ap.parse_args()

    with zipfile.ZipFile(args.kmz, "r") as zf:
        ref_lat, ref_lon, ref_alt = _parse_geo_desc(zf)
        wps = _parse_waypoints(zf, ref_lat, ref_lon, ref_alt)
        cloud = _parse_ply_xyz(zf)

    print(f"KMZ:        {args.kmz}")
    print(f"Ref WGS84:  lat={ref_lat:.6f} lon={ref_lon:.6f} alt={ref_alt:.2f} m")
    print(f"Waypoints:  {len(wps):,}")
    print(f"Cloud pts:  {len(cloud):,}")
    print()

    cloud64 = cloud.astype(np.float64)
    cone_cos = math.cos(math.radians(args.frustum_deg))
    aim_threshold_cos = math.cos(math.radians(args.max_aim_deg))

    rows = []
    for i, wp in enumerate(wps):
        wp_xyz = np.array([wp["x"], wp["y"], wp["z"]], dtype=np.float64)
        look = _look_dir(wp["yaw"], wp["pitch"])
        delta = cloud64 - wp_xyz
        dist = np.linalg.norm(delta, axis=1)
        # Only consider points within range and in the forward cone.
        valid = (dist > 0.5) & (dist <= args.max_range_m)
        if not valid.any():
            rows.append(dict(idx=i, hit=False, aim_deg=None, range_m=None, wp=wp))
            continue
        d_valid = dist[valid]
        delta_valid = delta[valid]
        cos_with_look = (delta_valid @ look) / d_valid
        in_cone = cos_with_look >= cone_cos
        if not in_cone.any():
            rows.append(dict(idx=i, hit=False, aim_deg=None, range_m=None, wp=wp))
            continue
        # Score: prefer points closest to the look ray (highest cos), break
        # ties by closer range.
        scored = cos_with_look[in_cone] - 0.001 * (d_valid[in_cone] / args.max_range_m)
        best = int(np.argmax(scored))
        best_cos = float(cos_with_look[in_cone][best])
        best_range = float(d_valid[in_cone][best])
        aim_deg = math.degrees(math.acos(min(1.0, max(-1.0, best_cos))))
        rows.append(dict(idx=i, hit=True, aim_deg=aim_deg, range_m=best_range, wp=wp))

    # Aggregate
    n = len(rows)
    n_no_hit = sum(1 for r in rows if not r["hit"])
    hits = [r for r in rows if r["hit"]]
    on_target = [r for r in hits if r["aim_deg"] <= args.max_aim_deg]
    print(f"=== Aim ===")
    print(f"  no in-cone cloud point ({args.frustum_deg:.0f}°/{args.max_range_m:.0f} m): {n_no_hit}/{n} ({100*n_no_hit/n:.1f}%)")
    print(f"  on-target (aim ≤ {args.max_aim_deg:.0f}°):                          {len(on_target)}/{n} ({100*len(on_target)/n:.1f}%)")
    if hits:
        aims = np.array([r["aim_deg"] for r in hits])
        ranges = np.array([r["range_m"] for r in hits])
        print(f"  aim error: median {np.median(aims):.1f}°  mean {aims.mean():.1f}°  p90 {np.percentile(aims, 90):.1f}°  max {aims.max():.1f}°")
        print(f"  range:     median {np.median(ranges):.1f} m  mean {ranges.mean():.1f} m  p90 {np.percentile(ranges, 90):.1f} m  max {ranges.max():.1f} m")

    # Pitch/yaw distribution
    pitches = np.array([w["pitch"] for w in wps])
    print()
    print(f"=== Pitch/yaw distribution ===")
    print(f"  pitch:  min {pitches.min():.1f}°  median {np.median(pitches):.1f}°  max {pitches.max():.1f}°")
    print(f"          extreme up   (>+25°):  {int((pitches > 25).sum())}")
    print(f"          extreme down (<-85°):  {int((pitches < -85).sum())}")
    print(f"          horizontal-ish (-30..+10°): {int(((pitches > -30) & (pitches < 10)).sum())}  ({100*((pitches > -30) & (pitches < 10)).sum()/n:.1f}%)")

    # Smoothness — adjacent-WP jumps
    print()
    print(f"=== Adjacent-WP smoothness (lower = smoother gimbal track) ===")
    pitch_jumps = np.abs(np.diff(pitches))
    yaws = np.array([w["yaw"] for w in wps])
    yaw_jumps = np.array([abs(_angular_diff_deg(yaws[i], yaws[i-1])) for i in range(1, n)])
    print(f"  pitch Δ between adjacent WPs:  median {np.median(pitch_jumps):.1f}°  p90 {np.percentile(pitch_jumps, 90):.1f}°  max {pitch_jumps.max():.1f}°")
    print(f"  yaw Δ between adjacent WPs:    median {np.median(yaw_jumps):.1f}°  p90 {np.percentile(yaw_jumps, 90):.1f}°  max {yaw_jumps.max():.1f}°")
    print(f"  WPs with |Δyaw| > 30° from prev:  {int((yaw_jumps > 30).sum())}  ({100*(yaw_jumps > 30).sum()/(n-1):.1f}%)")

    # Worst aimed
    if hits and args.show_worst > 0:
        print()
        print(f"=== {min(args.show_worst, len(hits))} worst-aimed WPs ===")
        worst = sorted(hits, key=lambda r: -r["aim_deg"])[:args.show_worst]
        for r in worst:
            wp = r["wp"]
            print(f"  WP[{r['idx']:4d}]  aim {r['aim_deg']:5.1f}°  range {r['range_m']:5.1f} m  "
                  f"pitch {wp['pitch']:+6.1f}°  yaw {wp['yaw']:+7.1f}°  "
                  f"alt {wp['alt']:.1f}m")

    # No-hit WPs sample
    no_hits = [r for r in rows if not r["hit"]]
    if no_hits and args.show_worst > 0:
        print()
        print(f"=== {min(args.show_worst, len(no_hits))} sample no-hit WPs (camera aimed at empty space) ===")
        for r in no_hits[:args.show_worst]:
            wp = r["wp"]
            print(f"  WP[{r['idx']:4d}]  pitch {wp['pitch']:+6.1f}°  yaw {wp['yaw']:+7.1f}°  alt {wp['alt']:.1f}m")


if __name__ == "__main__":
    main()
