"""Quality metrics for an augmented KMZ.

Reads an augmented KMZ (waypoints + bundled cloud + bundled sfm_geo_desc),
re-runs the augmenter's gimbal-rewrite on the same cloud while sweeping
parameters, and prints a tabulated quality report you can iterate against.

Usage
-----

    # Score a single augmented KMZ
    python -m flight_planner.tools.aim_quality path/to/augmented.kmz

    # Sweep a parameter and tabulate metrics for each value
    python -m flight_planner.tools.aim_quality path/to/augmented.kmz \
        --sweep wall_bonus:0.1,0.3,0.5,0.7,1.0
    python -m flight_planner.tools.aim_quality path/to/augmented.kmz \
        --sweep max_distance_m:10,15,20,30,60
    python -m flight_planner.tools.aim_quality path/to/augmented.kmz \
        --sweep min_roof_area:0.5,1.0,2.0,3.0,5.0

Metrics
-------

* ``aimed%``        - percent of WPs the picker found a facet for (rest stay
                       at the original Smart3D pose, often pitch=0)
* ``hit_cloud%``    - percent of WPs whose look ray actually intersects the
                       point cloud within 0.5 m of any cloud point
* ``pick_dist_m``   - 3D distance from WP to picked facet centroid
                       (lower is better - picker found a near facet)
* ``pitch_deg``     - gimbal pitch distribution; negative = looking down
* ``anomalies``     - count of WPs whose pitch hit a gimbal hardware limit
                       (-85 / +25 with the configured margin)

These metrics give you a fast feedback loop without re-deploying to the
Manifold. Run the augmenter once to produce the KMZ, then sweep here.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

from ..gimbal_rewrite import rewrite_gimbals_perpendicular
from ..kmz_import import (
    facades_from_pointcloud_cgal,
    estimate_facade_detection_defaults,
    filter_facades_by_polygon,
    parse_kmz,
    tight_footprint_from_cloud_xy,
)


_PITCH_UP_ANOMALY_DEG = 25.0
_PITCH_DN_ANOMALY_DEG = -85.0


@dataclass
class Sweep:
    name: str
    values: list[float]


def _parse_sweep(spec: str) -> Sweep:
    name, _, csv = spec.partition(":")
    if not csv:
        raise SystemExit(f"--sweep expects 'name:v1,v2,...' got {spec!r}")
    return Sweep(name=name.strip(), values=[float(v) for v in csv.split(",")])


def _load_kmz(kmz_path: Path):
    """Parse the KMZ; return (intent, cloud o3d.PointCloud)."""
    raw = kmz_path.read_bytes()
    intent = parse_kmz(raw, name=kmz_path.stem)
    if not intent.point_cloud_ply:
        raise SystemExit(
            f"{kmz_path} has no bundled cloud.ply - "
            "this tool needs an augmented KMZ produced after the bundle-cloud fix."
        )
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tf:
        tf.write(intent.point_cloud_ply); tf.flush()
        pc = o3d.io.read_point_cloud(tf.name)
    return intent, pc


def _waypoints_enu(intent):
    from ..cli import _waypoints_from_intent
    return _waypoints_from_intent(intent)


def _ray_hits_cloud(wp_pos: np.ndarray, look_dir: np.ndarray, look_len: float,
                    kdt: o3d.geometry.KDTreeFlann, radius_m: float = 0.5) -> bool:
    """Walk along the look ray; return True if any sample is within
    radius_m of a cloud point."""
    if look_len < 1e-6:
        return False
    n_samples = max(3, int(look_len / 0.5))
    for s in range(1, n_samples + 1):
        sample = wp_pos + look_dir * (s / n_samples) * look_len * 0.95
        n_close, _i, _d2 = kdt.search_radius_vector_3d(sample, radius_m)
        if n_close > 5:
            return True
    return False


def _score_run(wps_orig, wps_new, facets, kdt) -> dict:
    pitches, pick_dists, hits, picked_count = [], [], 0, 0
    for orig, new in zip(wps_orig, wps_new):
        if new.facade_index < 0 or new.facade_index >= len(facets):
            continue
        picked_count += 1
        f = facets[new.facade_index]
        pos = np.array([orig.x, orig.y, orig.z])
        center = np.asarray(f.center)
        look = center - pos
        d = float(np.linalg.norm(look))
        pick_dists.append(d)
        pitches.append(new.gimbal_pitch_deg)
        if d > 1e-6 and _ray_hits_cloud(pos, look / d, d, kdt):
            hits += 1

    n = len(wps_orig)
    return {
        "n_waypoints": n,
        "n_aimed": picked_count,
        "n_facets": len(facets),
        "aimed_pct": 100.0 * picked_count / n if n else 0.0,
        "hit_cloud_pct": 100.0 * hits / picked_count if picked_count else 0.0,
        "pick_dist_m": _stat(pick_dists),
        "pitch_deg": _stat(pitches),
        "anomalies_up": sum(1 for p in pitches if p > _PITCH_UP_ANOMALY_DEG),
        "anomalies_dn": sum(1 for p in pitches if p < _PITCH_DN_ANOMALY_DEG),
    }


def _stat(xs: list[float]) -> dict:
    if not xs:
        return {"min": 0.0, "p25": 0.0, "med": 0.0, "p75": 0.0, "max": 0.0}
    a = np.asarray(xs)
    return {
        "min": float(a.min()),
        "p25": float(np.percentile(a, 25)),
        "med": float(np.median(a)),
        "p75": float(np.percentile(a, 75)),
        "max": float(a.max()),
    }


def _print_row(label: str, m: dict) -> None:
    print(
        f"  {label:>22}  "
        f"facets={m['n_facets']:>4}  "
        f"aimed={m['aimed_pct']:>5.1f}%  "
        f"hit_cloud={m['hit_cloud_pct']:>5.1f}%  "
        f"pick_dist med={m['pick_dist_m']['med']:>5.1f}m p75={m['pick_dist_m']['p75']:>5.1f}m  "
        f"pitch med={m['pitch_deg']['med']:>+6.1f} p25={m['pitch_deg']['p25']:>+6.1f} p75={m['pitch_deg']['p75']:>+6.1f}  "
        f"anom={m['anomalies_up'] + m['anomalies_dn']:>3}"
    )


def _score_config(intent, pc, *, max_distance_m=60.0, wall_bonus=0.5,
                  min_roof=0.5, min_tilted=0.4) -> dict:
    pts = np.asarray(pc.points, dtype=np.float64)
    fd = estimate_facade_detection_defaults(pts)
    fd.pop("_estimator", None)
    fd["min_roof_area_m2"] = float(min_roof)
    fd["min_tilted_area_m2"] = float(min_tilted)
    facets = facades_from_pointcloud_cgal(
        pts, tight_footprint_from_cloud_xy(pts), **fd,
    )
    facets = filter_facades_by_polygon(
        facets, intent.mission_area_wgs84,
        intent.ref_lat, intent.ref_lon, intent.ref_alt,
    )

    wps_orig = _waypoints_enu(intent)
    wps_new = rewrite_gimbals_perpendicular(
        wps_orig, facets,
        max_distance_m=float(max_distance_m),
        pitch_margin_deg=2.0,
        preserve_heading=True,
        wall_distance_bonus=float(wall_bonus),
    )
    kdt = o3d.geometry.KDTreeFlann(pc)
    return _score_run(wps_orig, wps_new, facets, kdt)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("kmz", type=Path, help="Augmented KMZ to score")
    p.add_argument("--sweep", default=None,
                   help="Parameter sweep: NAME:v1,v2,... where NAME is "
                        "wall_bonus, max_distance_m, min_roof_area, or min_tilted_area")
    p.add_argument("--json", action="store_true",
                   help="Emit JSON instead of a table")
    args = p.parse_args(argv)

    intent, pc = _load_kmz(args.kmz)
    print(f"Loaded {args.kmz}")
    print(f"  waypoints: {len(intent.waypoints)}  cloud: {len(pc.points):,} pts")
    print()

    base = dict(max_distance_m=60.0, wall_bonus=0.5, min_roof=0.5, min_tilted=0.4)

    if args.sweep:
        sw = _parse_sweep(args.sweep)
        if sw.name not in {"wall_bonus", "max_distance_m", "min_roof_area", "min_tilted_area"}:
            raise SystemExit(f"unknown sweep parameter {sw.name!r}")
        param_name = "min_roof" if sw.name == "min_roof_area" else (
            "min_tilted" if sw.name == "min_tilted_area" else sw.name
        )
        print(f"Sweeping {sw.name} over {sw.values} (others at defaults: {base})")
        print()
        rows = []
        for v in sw.values:
            cfg = {**base, param_name: v}
            m = _score_config(intent, pc, **cfg)
            rows.append({"value": v, **m})
            _print_row(f"{sw.name}={v}", m)
        if args.json:
            print(json.dumps(rows, indent=2))
    else:
        m = _score_config(intent, pc, **base)
        if args.json:
            print(json.dumps(m, indent=2))
        else:
            _print_row("(defaults)", m)

    return 0


if __name__ == "__main__":
    sys.exit(main())
