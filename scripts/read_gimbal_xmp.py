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
    # The XMP packet sits in the JPEG header, so reading the whole 6 MB frame to
    # find it costs ~100x more I/O than it needs to -- which matters when the
    # photos are still on the SD card. Read the head, widen only if we must.
    with jpeg.open("rb") as fh:
        blob = fh.read(512 * 1024)
        start = blob.find(_XMP_START)
        end = blob.find(_XMP_END, start) if start >= 0 else -1
        if start < 0 or end < 0:
            blob = blob + fh.read()
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


def photo_shots(names: list[str]) -> list[tuple[int, int] | None]:
    """Map each photo name to the (waypoint, shot) it came from.

    `kmz_builder` labels every takePhoto with a `wpN` fileSuffix, and DJI appends
    it to the filename -- but only for the FIRST photo of a waypoint. When a
    waypoint shoots twice (the gimbal pans between the shots), the extra frames
    come back with no suffix at all:

        DJI_..._0001_V_wp0.JPG   <- waypoint 0, shot 0
        DJI_..._0003_V.JPG       <- waypoint 0, shot 1  (panned)
        DJI_..._0004_V_wp1.JPG   <- waypoint 1, shot 0

    So the suffix marks where a waypoint begins and unlabelled frames belong to
    the waypoint that preceded them. Names must be in capture order. Photos taken
    before any label (a DJI rosette capture has no labels at all) map to None.
    """
    out: list[tuple[int, int] | None] = []
    wp: int | None = None
    shot = 0
    for name in names:
        m = re.search(r"_wp(\d+)(?:_(\d+))?(?:\.[A-Za-z0-9]+)?$", name)
        if m is not None:
            wp = int(m.group(1))
            shot = int(m.group(2)) if m.group(2) else 0
        elif wp is not None:
            shot += 1
        out.append((wp, shot) if wp is not None else None)
    return out


def _action_blocks(placemark: str) -> list[str]:
    """The placemark's <wpml:action> blocks, in execution order."""
    blocks = re.findall(r"<wpml:action>.*?</wpml:action>", placemark, re.S)

    def action_id(b: str) -> int:
        m = re.search(r"<wpml:actionId>(\d+)</wpml:actionId>", b)
        return int(m.group(1)) if m else 0

    return sorted(blocks, key=action_id)


def commanded_from_kmz(kmz: Path) -> list[dict[str, object]]:
    """Commanded pose at each takePhoto, walking actions in execution order.

    A waypoint that shoots twice rotates the gimbal between the shots, so the two
    photos carry different commanded poses. Reading the placemark as a whole (the
    old behaviour) gave both shots the first rotation's angles and silently made
    the panned shot look like a duplicate of the primary one.

    A deduped waypoint emits no gimbalRotate at all; the gimbal holds its last
    pose, so state is carried forward across waypoints.
    """
    with zipfile.ZipFile(kmz) as z:
        name = next(n for n in z.namelist() if n.endswith("waylines.wpml"))
        xml = z.read(name).decode(errors="replace")

    rows: list[dict[str, object]] = []
    last_yaw: float | None = None
    last_pitch: float | None = None

    for wp_index, pm in enumerate(re.split(r"<Placemark>", xml)[1:]):
        h = re.search(r"<wpml:waypointHeadingAngle>([-\d.]+)</wpml:waypointHeadingAngle>", pm)
        heading = float(h.group(1)) if h else math.nan

        for block in _action_blocks(pm):
            if "<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>" in block:
                gp = re.search(r"<wpml:gimbalPitchRotateAngle>([-\d.]+)<", block)
                pitch_on = re.search(r"<wpml:gimbalPitchRotateEnable>(\d)<", block)
                if gp is not None and (pitch_on is None or pitch_on.group(1) == "1"):
                    last_pitch = float(gp.group(1))

                yaw_on = re.search(r"<wpml:gimbalYawRotateEnable>(\d)<", block)
                base = re.search(r"<wpml:gimbalHeadingYawBase>([a-z]+)<", block)
                gy = re.search(r"<wpml:gimbalYawRotateAngle>([-\d.]+)<", block)
                if yaw_on is not None and yaw_on.group(1) == "1" and gy is not None:
                    # Only a north-referenced yaw is comparable to XMP GimbalYawDegree.
                    last_yaw = float(gy.group(1)) if (base is None or base.group(1) == "north") else None
                elif yaw_on is not None and yaw_on.group(1) == "0":
                    # Yaw deliberately not commanded: the gimbal follows the nose.
                    last_yaw = None
                continue

            if "<wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>" not in block:
                continue

            suffix = re.search(r"<wpml:fileSuffix>([^<]*)</wpml:fileSuffix>", block)
            label = suffix.group(1).strip() if suffix else ""
            shot = 0
            if label:
                tail = label.split("_")
                shot = int(tail[1]) if len(tail) > 1 and tail[1].isdigit() else 0
            rows.append(
                {
                    "suffix": label or None,
                    "wp": wp_index,
                    "shot": shot,
                    "cmd_yaw": last_yaw if last_yaw is not None else math.nan,
                    "cmd_pitch": last_pitch if last_pitch is not None else math.nan,
                    "heading": heading,
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
    shots = photo_shots([p.name for p in jpegs])
    for p, key in zip(jpegs, shots):
        a = read_xmp_angles(p)
        if a is None:
            missing += 1
            continue
        rows.append({"file": p.name, "wp": key[0] if key else None,
                     "shot": key[1] if key else None, **a})

    print(f"photos: {len(jpegs)}   with XMP angles: {len(rows)}   without: {missing}")
    if not rows:
        print("No drone-dji XMP found. Are these DJI originals (not re-encoded)?", file=sys.stderr)
        return 1

    labelled = [r for r in rows if r["wp"] is not None]
    per_shot: dict[int, list[dict]] = {}
    for r in labelled:
        per_shot.setdefault(r["shot"], []).append(r)
    if per_shot:
        counts = ", ".join(f"shot {k}: {len(v)}" for k, v in sorted(per_shot.items()))
        print(f"waypoint-labelled: {len(labelled)}  ({counts})")
    else:
        print("no wp labels in the filenames — a DJI rosette capture, or not one of ours")

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

    def pan_report(label, subset):
        pans = [_wrap180(r["GimbalYawDegree"] - r["FlightYawDegree"]) for r in subset
                if "GimbalYawDegree" in r and "FlightYawDegree" in r]
        if not pans:
            return None
        a = sorted(abs(x) for x in pans)
        over = sum(1 for x in a if x > 60.0)
        print(f"  {label}: |pan| median {a[len(a)//2]:5.1f}°  p90 {a[int(0.9*len(a))]:5.1f}°  "
              f"max {a[-1]:5.1f}°   beyond ±60°: {over}/{len(a)}")
        return pans

    if gy and fy:
        print("\n=== ACTUAL gimbal pan relative to the airframe ===")
        print("  (we command no gimbal yaw on primary shots, so shot 0 should sit near 0°;")
        print("   an extra shot pans deliberately, so shot 1+ should NOT be near 0°)")
        pan_report("all photos     ", rows)
        for k, v in sorted(per_shot.items()):
            pan_report(f"shot {k}         ", v)

    if args.kmz:
        cmd = commanded_from_kmz(args.kmz)
        by_key = {(c["wp"], c["shot"]): c for c in cmd}
        print(f"\n=== vs COMMANDED ({args.kmz.name}) ===")
        print(f"  photo actions in KMZ: {len(cmd)}   photos on disk: {len(rows)}")

        paired = [(r, by_key[(r["wp"], r["shot"])]) for r in labelled
                  if (r["wp"], r["shot"]) in by_key]
        if paired:
            shot_photos = {(r["wp"], r["shot"]) for r in labelled}
            never_shot = [k for k in by_key if k not in shot_photos]
            print(f"  paired by waypoint label: {len(paired)} of {len(cmd)} planned shots"
                  f"   planned-but-missing: {len(never_shot)}")
            if never_shot:
                print(f"    first missing: {sorted(never_shot)[:10]}")
        else:
            print("  cannot pair by label — falling back to capture order")
            if len(cmd) != len(rows):
                print("  counts differ too; refusing to pair. The diff would be fiction.")
                paired = []
            else:
                paired = list(zip(rows, cmd))

        def diff_report(label, pairs):
            dp = [r["GimbalPitchDegree"] - c["cmd_pitch"] for r, c in pairs
                  if "GimbalPitchDegree" in r and not math.isnan(c["cmd_pitch"])]
            dy = [_wrap180(r["GimbalYawDegree"] - c["cmd_yaw"]) for r, c in pairs
                  if "GimbalYawDegree" in r and not math.isnan(c["cmd_yaw"])]
            if not dp and not dy:
                return
            print(f"  {label}  (n={len(pairs)})")
            for nm, d in (("pitch", dp), ("yaw  ", dy)):
                if not d:
                    print(f"    |actual-commanded| {nm}: never commanded here")
                    continue
                s = sorted(abs(x) for x in d)
                # Say how many of the pairs actually carried a command for this
                # axis. We command gimbal yaw only on a panned shot, so a yaw
                # figure drawn from a handful of waypoints is not a fleet number.
                print(f"    |actual-commanded| {nm}: median {s[len(s)//2]:5.1f}°  "
                      f"p90 {s[int(0.9*len(s))]:5.1f}°  max {s[-1]:5.1f}°"
                      f"   (commanded on {len(d)}/{len(pairs)})")

        if paired:
            diff_report("all shots", paired)
            by_shot: dict[int, list] = {}
            for r, c in paired:
                by_shot.setdefault(c["shot"], []).append((r, c))
            if len(by_shot) > 1:
                for k, v in sorted(by_shot.items()):
                    diff_report(f"shot {k}", v)

            dy_all = [_wrap180(r["GimbalYawDegree"] - c["cmd_yaw"]) for r, c in paired
                      if "GimbalYawDegree" in r and not math.isnan(c["cmd_yaw"])]
            if len(dy_all) >= 20:
                q = len(dy_all) // 4
                first = sum(abs(x) for x in dy_all[:q]) / q
                last = sum(abs(x) for x in dy_all[-q:]) / q
                print(f"\n  yaw error, first quarter: {first:.1f}°   last quarter: {last:.1f}°")
                if last > first * 1.8 and last > 10.0:
                    print("  -> ERROR GROWS THROUGH THE FLIGHT. This is drift, not mis-aim.")
                else:
                    print("  -> no systematic growth; the error is not accumulating.")

    if args.csv:
        with args.csv.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["file", "wp", "shot", *_FIELDS])
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in ["file", "wp", "shot", *_FIELDS]})
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
