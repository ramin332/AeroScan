"""Inspect a DJI-exported KMZ to discover drone/payload enum values.

DJI does not publish WPML drone/payload enum tables for the Matrice 4 series.
The authoritative values come from a KMZ exported by DJI Pilot 2 itself with
the target drone connected. This tool extracts those values and (optionally)
rewrites the provisional constants in ``kmz_builder.py`` to match.

Usage
-----

    python -m flight_planner.tools.inspect_kmz path/to/dji_export.kmz
    python -m flight_planner.tools.inspect_kmz path/to/dji_export.kmz --patch
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass
from pathlib import Path

_KML_NS = "http://www.opengis.net/kml/2.2"
_WPML_NS_PATTERN = re.compile(r"http://www\.dji\.com/wpmz/(\d+\.\d+\.\d+)")


@dataclass
class KmzEnums:
    drone_enum: int
    drone_sub_enum: int
    payload_enum: int
    payload_sub_enum: int
    wpml_version: str
    template_type: str | None


def _read_template_xml(kmz_path: Path) -> str:
    with zipfile.ZipFile(kmz_path) as zf:
        if "wpmz/template.kml" not in zf.namelist():
            raise ValueError(
                f"{kmz_path}: missing wpmz/template.kml — not a DJI WPML KMZ"
            )
        return zf.read("wpmz/template.kml").decode("utf-8")


def _find_int(parent: ET.Element, wpml_ns: str, tag: str) -> int | None:
    el = parent.find(f"{{{wpml_ns}}}{tag}")
    if el is None or el.text is None:
        return None
    try:
        return int(el.text.strip())
    except ValueError:
        return None


def inspect_kmz(kmz_path: Path) -> KmzEnums:
    """Extract drone + payload enum values from a DJI-exported KMZ."""
    xml = _read_template_xml(kmz_path)

    match = _WPML_NS_PATTERN.search(xml)
    if not match:
        raise ValueError(f"{kmz_path}: no DJI WPML namespace found")
    wpml_version = match.group(1)
    wpml_ns = match.group(0)

    root = ET.fromstring(xml)

    drone_info = root.find(f".//{{{wpml_ns}}}droneInfo")
    if drone_info is None:
        raise ValueError(f"{kmz_path}: missing <wpml:droneInfo>")
    drone_enum = _find_int(drone_info, wpml_ns, "droneEnumValue")
    if drone_enum is None:
        raise ValueError(f"{kmz_path}: missing <wpml:droneEnumValue>")
    drone_sub_enum = _find_int(drone_info, wpml_ns, "droneSubEnumValue") or 0

    payload_info = root.find(f".//{{{wpml_ns}}}payloadInfo")
    if payload_info is None:
        raise ValueError(f"{kmz_path}: missing <wpml:payloadInfo>")
    payload_enum = _find_int(payload_info, wpml_ns, "payloadEnumValue")
    if payload_enum is None:
        raise ValueError(f"{kmz_path}: missing <wpml:payloadEnumValue>")
    payload_sub_enum = _find_int(payload_info, wpml_ns, "payloadSubEnumValue") or 0

    template_type_el = root.find(f".//{{{wpml_ns}}}templateType")
    template_type = template_type_el.text if template_type_el is not None else None

    return KmzEnums(
        drone_enum=drone_enum,
        drone_sub_enum=drone_sub_enum,
        payload_enum=payload_enum,
        payload_sub_enum=payload_sub_enum,
        wpml_version=wpml_version,
        template_type=template_type,
    )


def _print_report(kmz_path: Path, enums: KmzEnums) -> None:
    print(f"Inspected: {kmz_path}")
    print(f"  WPML version:       {enums.wpml_version}")
    print(f"  Template type:      {enums.template_type}")
    print(f"  droneEnumValue:     {enums.drone_enum}")
    print(f"  droneSubEnumValue:  {enums.drone_sub_enum}")
    print(f"  payloadEnumValue:   {enums.payload_enum}")
    print(f"  payloadSubEnumValue:{enums.payload_sub_enum}")


_CONSTANT_LINE = re.compile(
    r"^(?P<name>M4E_(?:DRONE|PAYLOAD)(?:_SUB)?_ENUM)\s*=\s*\d+"
)


def patch_kmz_builder(enums: KmzEnums, builder_path: Path) -> list[tuple[str, int, int]]:
    """Rewrite the M4E_* constants in kmz_builder.py. Returns list of (name, old, new)."""
    new_values = {
        "M4E_DRONE_ENUM": enums.drone_enum,
        "M4E_DRONE_SUB_ENUM": enums.drone_sub_enum,
        "M4E_PAYLOAD_ENUM": enums.payload_enum,
        "M4E_PAYLOAD_SUB_ENUM": enums.payload_sub_enum,
    }
    changes: list[tuple[str, int, int]] = []
    text = builder_path.read_text()
    new_lines: list[str] = []
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip()
        indent = line[: len(line) - len(stripped)]
        m = _CONSTANT_LINE.match(stripped)
        if m and m.group("name") in new_values:
            name = m.group("name")
            new_val = new_values[name]
            # Extract old value
            old_match = re.search(rf"{name}\s*=\s*(\d+)", line)
            old_val = int(old_match.group(1)) if old_match else -1
            # Preserve any trailing comment
            comment_match = re.search(r"(\s*#.*)$", line.rstrip("\n"))
            trailing = comment_match.group(1) if comment_match else ""
            trailing_newline = "\n" if line.endswith("\n") else ""
            new_lines.append(f"{indent}{name} = {new_val}{trailing}{trailing_newline}")
            if old_val != new_val:
                changes.append((name, old_val, new_val))
        else:
            new_lines.append(line)
    if changes:
        builder_path.write_text("".join(new_lines))
    return changes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kmz", type=Path, help="Path to DJI-exported KMZ")
    parser.add_argument(
        "--patch",
        action="store_true",
        help="Rewrite the M4E_* constants in kmz_builder.py with the discovered values",
    )
    parser.add_argument(
        "--builder",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "kmz_builder.py",
        help="Path to kmz_builder.py (default: auto-detected from this file's location)",
    )
    args = parser.parse_args(argv)

    if not args.kmz.exists():
        print(f"error: {args.kmz} not found", file=sys.stderr)
        return 2

    try:
        enums = inspect_kmz(args.kmz)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    _print_report(args.kmz, enums)

    if not args.patch:
        print("\nRun with --patch to update kmz_builder.py constants.")
        return 0

    if not args.builder.exists():
        print(f"error: {args.builder} not found", file=sys.stderr)
        return 2

    changes = patch_kmz_builder(enums, args.builder)
    if not changes:
        print(f"\nNo changes needed — kmz_builder.py already matches.")
    else:
        print(f"\nPatched {args.builder}:")
        for name, old, new in changes:
            print(f"  {name}: {old} → {new}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
