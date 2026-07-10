#!/usr/bin/env python3
"""Read ACTUAL gimbal/aircraft angles from DJI JPEGs and diff them against a KMZ.

The WPML is mission *intent*. The angles the gimbal actually held at each shutter
are stamped into every JPEG's XMP by the flight controller. When the pilot says
"the gimbal ended up 45 degrees off," this is the only file that can confirm or
refute it -- the KMZ cannot, because it records what we asked for, not what
happened.

Needs no exiftool and no internet: XMP is a plaintext packet inside the JPEG.

Usage
-----
    # Just dump what the aircraft actually did:
    python scripts/read_gimbal_xmp.py /Volumes/DJI/DCIM/100MEDIA

    # Diff against the mission that flew (matches Nth photo to Nth photo-action):
    python scripts/read_gimbal_xmp.py /Volumes/DJI/DCIM/100MEDIA \
        --kmz flight-archive/2026-07-10/app-state/received/<mission>.augmented.lean.kmz

    # CSV for plotting:
    python scripts/read_gimbal_xmp.py <dir> --csv out.csv

Photo->waypoint pairing is BY ORDER, which assumes one photo per waypoint and no
dropped frames. That holds for our augmented missions (one takePhoto per WP) but
NOT for DJI's own Smart3D rosette captures. If the counts disagree the script
says so and refuses to pretend the pairing is meaningful.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import zipfile
from pathlib import Path

_XMP_START = b"<x:xmpmeta"
_XMP_END = b"</x:xmpmeta>"

_FIELDS = (
    "GimbalPitchDegree",
    "GimbalYawDegree",
    "GimbalRollDegree",
    "FlightPitchDegree",
    "FlightYawDegree",
    "FlightRollDegree",
)


def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def read_xmp_angles(jpeg: Path) -> dict[str, float] | None:
    """Extract drone-dji:* angles from a JPEG's XMP packet. None if absent."""
    blob = jpeg.read_bytes()
    start = blob.find(_XMP_START)
    if start < 0:
        return None
    end = blob.find(_XMP_END, start)
    if end < 0:
        return None
    xmp = blob[start : end + len(_XMP_END)].decode("utf-8", errors="replace")

    out: dict[str, float] = {}
    for field in _FIELDS:
        # DJI writes these either as attributes or as elements, depending on model.
        m = re.search(rf'drone-dji:{field}\s*=\s*"([+-]?[\d.]+)"', xmp)
        if m is None:
            m = re.search(rf"<drone-dji:{field}>([+-]?[\d.]+)</drone-dji:{field}>", xmp)
        if m is not None:
            out[field] = float(m.group(1))
    return out or None


def commanded_from_kmz(kmz: Path) -> list[dict[str, float]]:
    """Per-waypoint commanded (gimbal yaw, pitch, heading), carrying pose forward.

    A deduped waypoint emits no gimbalRotate; the gimbal holds its last-commanded
    pose. Carrying forward is what the aircraft actually does, so it is what we
    must compare the photo against.
    """
    with zipfile.ZipFile(kmz) as z:
        name = next(n for n in z.namelist() if n.endswith("waylines.wpml"))
        xml = z.read(name).decode(errors="replace")

    rows: list[dict[str, float]] = []
    last_yaw = last_pitch = None
    for pm in re.split(r"<Placemark>", xml)[1:]:
        h = re.search(r"<wpml:waypointHeadingAngle>([-\d.]+)</wpml:waypointHeadingAngle>", pm)
        gy = re.search(r"<wpml:gimbalYawRotateAngle>([-\d.]+)</wpml:gimbalYawRotateAngle>", pm)
        gp = re.search(r"<wpml:gimbalPitchRotateAngle>([-\d.]+)</wpml:gimbalPitchRotateAngle>", pm)
        base = re.search(r"<wpml:gimbalHeadingYawBase>([a-z]+)</wpml:gimbalHeadingYawBase>", pm)
        if gy is not None and (base is None or base.group(1) == "north"):
            last_yaw = float(gy.group(1))
        if gp is not None:
            last_pitch = float(gp.group(1))
        if "takePhoto" not in pm:
            continue
        rows.append(
            {
                "cmd_yaw": last_yaw if last_yaw is not None else math.nan,
                "cmd_pitch": last_pitch if last_pitch is not None else math.nan,
                "heading": float(h.group(1)) if h else math.nan,
            }
        )
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("photo_dir", type=Path, help="Directory of DJI JPEGs (DCIM/100MEDIA)")
    ap.add_argument("--kmz", type=Path, default=None, help="Flown KMZ to diff against")
    ap.add_argument("--csv", type=Path, default=None, help="Write per-photo rows here")
    args = ap.parse_args()

    jpegs = sorted(
        p for p in args.photo_dir.iterdir() if p.suffix.lower() in (".jpg", ".jpeg")
    )
    if not jpegs:
        print(f"No JPEGs in {args.photo_dir}", file=sys.stderr)
        return 1

    rows = []
    missing = 0
    for p in jpegs:
        a = read_xmp_angles(p)
        if a is None:
            missing += 1
            continue
        rows.append({"file": p.name, **a})

    print(f"photos: {len(jpegs)}   with XMP angles: {len(rows)}   without: {missing}")
    if not rows:
        print("No drone-dji XMP found. Are these DJI originals (not re-encoded)?", file=sys.stderr)
        return 1

    gy = [r["GimbalYawDegree"] for r in rows if "GimbalYawDegree" in r]
    gp = [r["GimbalPitchDegree"] for r in rows if "GimbalPitchDegree" in r]
    fy = [r["FlightYawDegree"] for r in rows if "FlightYawDegree" in r]

    def stat(name, v):
        if not v:
            print(f"  {name}: absent")
            return
        s = sorted(v)
        print(f"  {name}: min {s[0]:+7.1f}  median {s[len(s)//2]:+7.1f}  max {s[-1]:+7.1f}")

    print("\n=== ACTUAL, from XMP ===")
    stat("gimbal pitch", gp)
    stat("gimbal yaw  ", gy)
    stat("aircraft yaw", fy)

    if gy and fy and len(gy) == len(fy):
        pan = [_wrap180(y - h) for y, h in zip(gy, fy)]
        ap_ = [abs(x) for x in pan]
        over = sum(1 for x in ap_ if x > 60.0)
        s = sorted(ap_)
        print("\n=== ACTUAL gimbal pan relative to the airframe ===")
        print(f"  |pan|: median {s[len(s)//2]:5.1f}°  p90 {s[int(0.9*len(s))]:5.1f}°  max {s[-1]:5.1f}°")
        print(f"  beyond the +-60° hardware pan limit: {over} / {len(ap_)} ({100*over/len(ap_):.1f}%)")
        if over:
            print("  -> the gimbal was being asked for pan it does not have. It saturates here.")

    if args.kmz:
        cmd = commanded_from_kmz(args.kmz)
        print(f"\n=== vs COMMANDED ({args.kmz.name}) ===")
        print(f"  photo actions in KMZ: {len(cmd)}   photos on disk: {len(rows)}")
        if len(cmd) != len(rows):
            print("  counts differ — refusing to pair by order; the diff would be fiction.")
            print("  (Check for dropped frames, or a rosette capture with >1 photo/WP.)")
        else:
            dy = [_wrap180(r["GimbalYawDegree"] - c["cmd_yaw"]) for r, c in zip(rows, cmd)
                  if "GimbalYawDegree" in r and not math.isnan(c["cmd_yaw"])]
            dp = [r["GimbalPitchDegree"] - c["cmd_pitch"] for r, c in zip(rows, cmd)
                  if "GimbalPitchDegree" in r and not math.isnan(c["cmd_pitch"])]
            for nm, d in (("yaw", dy), ("pitch", dp)):
                if not d:
                    continue
                s = sorted(abs(x) for x in d)
                print(f"  |actual - commanded| {nm}: median {s[len(s)//2]:5.1f}°  p90 {s[int(0.9*len(s))]:5.1f}°  max {s[-1]:5.1f}°")
            # Drift check: does the error grow across the flight?
            if len(dy) >= 20:
                q = len(dy) // 4
                first = sum(abs(x) for x in dy[:q]) / q
                last = sum(abs(x) for x in dy[-q:]) / q
                print(f"\n  yaw error, first quarter: {first:.1f}°   last quarter: {last:.1f}°")
                if last > first * 1.8 and last > 10.0:
                    print("  -> ERROR GROWS THROUGH THE FLIGHT. This is drift, not mis-aim.")
                else:
                    print("  -> no systematic growth; the error is not accumulating.")

    if args.csv:
        with args.csv.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["file", *_FIELDS])
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in ["file", *_FIELDS]})
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
