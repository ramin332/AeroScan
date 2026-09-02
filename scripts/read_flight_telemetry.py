#!/usr/bin/env python3
"""Read the on-board telemetry CSV and diff it against the flown KMZ.

    .venv/bin/python scripts/read_flight_telemetry.py <telemetry.csv> \\
        [--kmz <flown>.augmented.lean.kmz] [--csv-out per_wp.csv]

The CSV comes from the Manifold: /open_app/dev/data/received/telemetry/<UTC>.csv
(pull with scp). No SD card needed. See flight_planner/tools/flight_telemetry.py.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

from flight_planner.tools.flight_telemetry import (
    commanded_from_kmz, compare, format_report, per_waypoint, read_telemetry,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("telemetry", type=Path)
    ap.add_argument("--kmz", type=Path, default=None, help="flown augmented KMZ (lean is fine)")
    ap.add_argument("--csv-out", type=Path, default=None)
    ap.add_argument("--state", type=int, default=48, help="mission state to keep (48 = MISSION)")
    a = ap.parse_args()
    samples = read_telemetry(a.telemetry)
    if not samples:
        print("no samples in", a.telemetry)
        return 1
    print(f"{len(samples)} samples, states {sorted({s.state for s in samples})}, waypoints {min(s.wp for s in samples)}..{max(s.wp for s in samples)}")
    actual = per_waypoint(samples, state=a.state)
    cmd = commanded_from_kmz(a.kmz) if a.kmz else None
    rep = compare(actual, cmd)
    print(format_report(rep))
    if a.csv_out and rep.rows:
        with open(a.csv_out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rep.rows[0].keys()))
            w.writeheader(); w.writerows(rep.rows)
        print("per-waypoint rows →", a.csv_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
