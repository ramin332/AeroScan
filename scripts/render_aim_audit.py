#!/usr/bin/env python3
"""Pre-flight aim audit: draw what the facade picker chose, over the real cloud.

Runs the same facet extraction + assignment the augment runs, then renders a
plan view and a side view with every aim line; red = picks that disagree with
their ±3 neighbours or aim reversals >90°. Prints the audit numbers and the
facets used. Use it BEFORE pushing a mission — it is how the 2026-07-10
"points at the wrong part" problem was diagnosed offline on 2026-09-02.

    .venv/bin/python scripts/render_aim_audit.py \\
        --intent flight-archive/2026-07-10/app-state/missions/<id>/intent.json \\
        --icp-target flight-archive/2026-07-10/app-state/missions/<id>/cloud.ply \\
        --blackbox-dir flight-archive/2026-07-10 --flight-id flight0072 \\
        --out output/aim-audit.png [--assign-mode greedy] [--max-facade-distance-m 60]
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

from flight_planner import cli
from flight_planner.camera import compute_distance_for_gsd, get_camera
from flight_planner.gimbal_rewrite import aim_audit, plane_groups, rewrite_gimbals_perpendicular
from flight_planner.kmz_import import facades_from_pointcloud_cgal, filter_facades_by_polygon, polygon_to_enu
from flight_planner.manifold import from_manifold, register_to_kmz_frame
from flight_planner.mission_intent import read_intent_json
from flight_planner.models import CameraName, MissionConfig


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--intent", type=Path, required=True)
    ap.add_argument("--icp-target", type=Path, required=True)
    ap.add_argument("--blackbox-dir", type=Path, default=Path("/blackbox"))
    ap.add_argument("--flight-id", default="the_latest_flight")
    ap.add_argument("--out", type=Path, required=True, help="PNG path; a .json with the numbers is written beside it")
    ap.add_argument("--assign-mode", choices=("viterbi", "greedy"), default=cli._NEN_ASSIGN_MODE)
    ap.add_argument("--switch-cost", type=float, default=cli._NEN_SWITCH_COST)
    ap.add_argument("--max-facade-distance-m", type=float, default=None)
    ap.add_argument("--voxel-m", type=float, default=0.10)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    reach = args.max_facade_distance_m
    if reach is None:
        reach = float(compute_distance_for_gsd(get_camera(CameraName.WIDE),
                                               MissionConfig().target_gsd_mm_per_px * cli._NEN_MAX_STANDOFF_GSD_FACTOR))
    intent = read_intent_json(args.intent)
    wps = cli._waypoints_from_intent(intent)
    pc = from_manifold(args.flight_id, blackbox_dir=args.blackbox_dir, voxel_m=args.voxel_m)
    reg, _T, _ = register_to_kmz_frame(pc, cli._load_icp_target(args.icp_target))
    pts = np.asarray(reg.points)
    poly = polygon_to_enu(intent.mission_area_wgs84, intent.ref_lat, intent.ref_lon, intent.ref_alt)
    tight = cli.tight_footprint_from_cloud_xy(pts) if hasattr(cli, "tight_footprint_from_cloud_xy") else poly
    fac = filter_facades_by_polygon(facades_from_pointcloud_cgal(pts, tight),
                                    intent.mission_area_wgs84, intent.ref_lat, intent.ref_lon, intent.ref_alt)
    for i, f in enumerate(fac):
        f.index = i
    out = rewrite_gimbals_perpendicular(wps, fac, max_distance_m=reach, pitch_margin_deg=cli._NEN_PITCH_MARGIN_DEG,
                                        switch_ratio=cli._NEN_SWITCH_RATIO, preserve_heading=False,
                                        assign_mode=args.assign_mode, switch_cost=args.switch_cost)
    audit = aim_audit(out, fac, far_standoff_m=reach)
    pick = np.array([w.facade_index for w in out]); n = len(out)
    W = np.array([[w.x, w.y, w.z] for w in out])
    NZ = np.array([abs(float(f.normal[2])) for f in fac])
    kind = lambda i: "wall" if NZ[i] <= .35 else ("roof" if NZ[i] >= .7 else "tilted")
    KC = {"wall": "#2a78d6", "roof": "#d03b3b", "tilted": "#eda100"}
    # Flags are target-aware: two slices of one wall are one target (plane
    # group), so hopping between them is neither a dispute nor a reversal.
    grp = plane_groups(fac)
    g = np.array([grp[p] if p >= 0 else -1 for p in pick])
    k = 3; disputed = set()
    for i in range(n):
        win = [g[j] for j in range(max(0, i - k), min(n, i + k + 1)) if j != i and g[j] >= 0]
        if g[i] >= 0 and win:
            m, c = Counter(win).most_common(1)[0]
            if c >= k and g[i] != m:
                disputed.add(i)
    hd = np.array([w.heading_deg for w in out]); dh = np.abs((np.diff(hd) + 180) % 360 - 180)
    rev = {i for i in np.where(dh > 90)[0] + 1 if g[i] != g[i - 1]}
    used = Counter(int(p) for p in pick if p >= 0)

    rng = np.random.default_rng(0)
    sub = pts[rng.choice(len(pts), min(60000, len(pts)), replace=False)]
    xmin, xmax = W[:, 0].min() - 8, W[:, 0].max() + 8; ymin, ymax = W[:, 1].min() - 8, W[:, 1].max() + 8
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 17), gridspec_kw={"height_ratios": [1.15, 1]})
    for a in (ax, ax2):
        a.set_facecolor("#fcfcfb")
    sel = (sub[:, 0] > xmin) & (sub[:, 0] < xmax) & (sub[:, 1] > ymin) & (sub[:, 1] < ymax)
    ax.scatter(sub[sel, 0], sub[sel, 1], s=0.5, c=np.clip(sub[sel, 2], -1, 6), cmap="Greys", alpha=.35, linewidths=0)
    for f in fac:
        v = np.asarray(f.vertices, float); kd = kind(f.index); u = used.get(f.index, 0) > 0
        ax.add_patch(Polygon(v[:, :2], closed=True, fill=u, fc=KC[kd], ec=KC[kd], alpha=.28 if u else 1, lw=.8 if u else .4))
        if used.get(f.index, 0) >= 5:
            c = v.mean(0); ax.text(c[0], c[1], f"{f.index}\n{used[f.index]}wp", fontsize=7, ha="center", va="center", color="#1c2430")
    ax.plot(W[:, 0], W[:, 1], color="#9aa3ad", lw=.6, zorder=3)
    for i in range(n):
        if pick[i] < 0:
            continue
        c = fac[pick[i]].center; bad = i in disputed or i in rev
        ax.plot([W[i, 0], c[0]], [W[i, 1], c[1]], color="#d03b3b" if bad else "#5c6675", lw=1.2 if bad else .35, alpha=.9 if bad else .5, zorder=4 if bad else 2)
    ax.scatter(W[:, 0], W[:, 1], s=6, c=np.where(pick >= 0, "#1c2430", "#9aa3ad"), zorder=5)
    for i in sorted(disputed | rev)[:60]:
        ax.annotate(str(i + 1), (W[i, 0], W[i, 1]), fontsize=6, color="#d03b3b", xytext=(2, 2), textcoords="offset points")
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect("equal"); ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title(f"{intent.name} · {args.assign_mode} · reach {reach:.1f} m — {len(fac)} facets (blue wall · red roof · amber tilted; filled = used)\n"
                 f"switches {audit['switches']}  flips>90° {audit['reversals_gt90']}  blips {audit['single_blips']}  far {audit['far_picks']}  "
                 f"unaimed {audit['unaimed']} (grey dots)   red lines = {len(disputed)} disputed + {len(rev)} reversals", fontsize=10)
    sel2 = (sub[:, 0] > xmin) & (sub[:, 0] < xmax)
    ax2.scatter(sub[sel2, 0], sub[sel2, 2], s=0.5, c="#9aa3ad", alpha=.3, linewidths=0)
    for f in fac:
        v = np.asarray(f.vertices, float); kd = kind(f.index); u = used.get(f.index, 0) > 0
        ax2.add_patch(Polygon(v[:, [0, 2]], closed=True, fill=u, fc=KC[kd], ec=KC[kd], alpha=.28 if u else 1, lw=.8 if u else .4))
    for i in range(n):
        if pick[i] < 0:
            continue
        c = fac[pick[i]].center; bad = i in disputed or i in rev
        ax2.plot([W[i, 0], c[0]], [W[i, 2], c[2]], color="#d03b3b" if bad else "#5c6675", lw=1.2 if bad else .35, alpha=.9 if bad else .45)
    ax2.scatter(W[:, 0], W[:, 2], s=6, c="#1c2430", zorder=5)
    ax2.set_xlim(xmin, xmax); ax2.set_ylim(-1.5, W[:, 2].max() + 2); ax2.set_aspect("equal"); ax2.set_xlabel("east (m)"); ax2.set_ylabel("up (m)")
    ax2.set_title("Side view (looking north)", fontsize=10)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(args.out, dpi=110)

    def area(f):
        v = np.asarray(f.vertices, float); c = v.mean(0)
        return float(sum(np.linalg.norm(np.cross(v[i] - c, v[(i + 1) % len(v)] - c)) for i in range(len(v))) / 2)
    rows = [{"facet": fi, "waypoints": cnt, "kind": kind(fi), "area_m2": round(area(fac[fi]), 1),
             "center": [round(float(x), 1) for x in fac[fi].center]} for fi, cnt in used.most_common()]
    args.out.with_suffix(".json").write_text(json.dumps({"audit": audit, "facets_used": rows}, indent=1))
    print(json.dumps(audit, indent=1))
    print(f"{'facet':>5} {'#wp':>4} {'kind':<6} {'m²':>6}  centre")
    for r in rows[:25]:
        print(f"{r['facet']:>5} {r['waypoints']:>4} {r['kind']:<6} {r['area_m2']:6.1f}  {r['center']}")
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
