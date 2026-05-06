#!/usr/bin/env python3
"""Post-bundle a cloud.ply from a source Smart3D KMZ into an existing
augmented KMZ so the dev frontend's /api/import-kmz endpoint can load it.

Why this exists: `flight_planner.cli augment-mission` now bundles the cloud
by default (kmz_builder.build_kmz's bundled_cloud_ply arg). But the augmented
KMZs the Manifold produced *before* that change (e.g. tonight's first run)
are flight-plan-only — no cloud — and the dev-frontend import refuses them
with "KMZ does not contain a reference point cloud".

This script takes:
  --input    augmented KMZ (no cloud)
  --source   the Smart3D source KMZ that has the cloud, OR --cloud-ply
  --output   augmented KMZ + cloud bundled, ready for /api/import-kmz

Usage:
    python scripts/bundle_cloud_into_augmented_kmz.py \\
        --input  output/manifold_augmented/<ts>.augmented.kmz \\
        --source kmz/Mijande.kmz \\
        --output output/manifold_augmented/<ts>.augmented.with_cloud.kmz
"""
from __future__ import annotations

import argparse
import io
import sys
import zipfile
from pathlib import Path


def find_cloud_in_kmz(kmz_bytes: bytes) -> tuple[bytes, str] | None:
    """Return (cloud_ply_bytes, original_member_name) for the first
    cloud.ply found in a Smart3D KMZ, or None."""
    with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as zf:
        for name in zf.namelist():
            if name.lower().endswith("cloud.ply"):
                return zf.read(name), name
    return None


def bundle(input_kmz: Path, cloud_ply: bytes, cloud_member_path: str, output_kmz: Path) -> int:
    """Copy input KMZ to output, adding the cloud at cloud_member_path.

    cloud_member_path should be a Smart3D-style path like
    `wpmz/res/ply/<mission>/cloud.ply` so the dev frontend's _find_cloud_ply
    matcher (any name ending in 'cloud.ply') picks it up.
    """
    out_buf = io.BytesIO()
    with zipfile.ZipFile(input_kmz, "r") as zin, \
         zipfile.ZipFile(out_buf, "w", zipfile.ZIP_DEFLATED) as zout:
        # Copy existing entries unchanged
        for info in zin.infolist():
            zout.writestr(info, zin.read(info.filename))
        # Add the cloud
        zout.writestr(cloud_member_path, cloud_ply)
    output_kmz.parent.mkdir(parents=True, exist_ok=True)
    output_kmz.write_bytes(out_buf.getvalue())
    return len(out_buf.getvalue())


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True,
                   help="Augmented KMZ (no cloud).")
    src_group = p.add_mutually_exclusive_group(required=True)
    src_group.add_argument("--source", type=Path, default=None,
                           help="Smart3D KMZ to extract cloud.ply from.")
    src_group.add_argument("--cloud-ply", type=Path, default=None,
                           help="Standalone cloud.ply file.")
    p.add_argument("--output", type=Path, required=True,
                   help="Output KMZ path (input + bundled cloud).")
    p.add_argument("--member-name", type=str, default="wpmz/res/ply/mission/cloud.ply",
                   help="Path inside the output KMZ to write cloud.ply at "
                        "(default mirrors Smart3D structure).")
    args = p.parse_args()

    if args.source is not None:
        src_bytes = args.source.read_bytes()
        found = find_cloud_in_kmz(src_bytes)
        if found is None:
            print(f"ERROR: no cloud.ply in {args.source}", file=sys.stderr)
            return 1
        cloud_ply, original_path = found
        print(f"Source KMZ:    {args.source}  ({len(src_bytes):,} bytes)")
        print(f"  cloud.ply at: {original_path}  ({len(cloud_ply):,} bytes)")
        # Reuse the Smart3D path so it visually matches a real Smart3D KMZ
        member_path = original_path
    else:
        cloud_ply = args.cloud_ply.read_bytes()
        print(f"Cloud:         {args.cloud_ply}  ({len(cloud_ply):,} bytes)")
        member_path = args.member_name

    in_size = args.input.stat().st_size
    print(f"Input KMZ:     {args.input}  ({in_size:,} bytes)")
    out_size = bundle(args.input, cloud_ply, member_path, args.output)
    print(f"Output KMZ:    {args.output}  ({out_size:,} bytes)")
    print(f"  cloud.ply at: {member_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
