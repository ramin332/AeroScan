# Ground-plane facade fix — design

**Date:** 2026-07-10
**Status:** approved, ready for implementation plan
**Origin:** 2026-07-10 second custom flight (`test10-7`), van in an empty parking lot

## Problem

`facades_from_pointcloud_cgal` separates object from ground with a horizontal
height cut:

```python
ground_z = float(np.percentile(pts_all[:, 2], 5.0))
pts_all = pts_all[pts_all[:, 2] >= ground_z + ground_skip_m]   # ground_skip_m = 1.0
```

This assumes the target is tall relative to both the ground's slope and the
cloud's noise. On the 2026-07-10 flight the target was a ~2.7 m van on a
2.50°-sloping asphalt lot. Measured inside the DJI `mission_area_wgs84`
polygon (44,342 points):

| quantity | value |
|---|---|
| `ground_z` (5th pct) | 0.41 m |
| `top_z` (98th pct) | 3.14 m |
| `height_range` | **2.73 m** (a building is 10–30 m) |
| cut = `ground_z + 1.0` | 1.41 m |
| van points **destroyed** by the cut | 43% |
| asphalt points **surviving** the cut | 28% |

The surviving asphalt is near-horizontal, so `roof_normal_z_min = 0.7`
classifies it as roof. Extraction on the flown cloud yields **80 facets, of
which 53 are `roof_*` at z ≈ 1.5–1.6 m with `normal_z = −1.00`** — outward
normals pointing down into the earth. Of 141.2 m² of "facade", only ~17.4 m²
(13 wall facets) is the van. The augmenter then aims the gimbal at tarmac.

A horizontal cut cannot fix this. Across the 13 m polygon a 2.50° slope spans
0.57 m of Z **by geometry alone**, before noise (asphalt Z actually ranges
−3.56 → +2.50 m). No single constant separates a sloped ground from a short
object. Retuning `ground_skip_m` retunes the bug.

## Requirement

Remove the ground. Whatever stands above it is the inspection target. A van
and a house are the same problem at different scales — there is no
"short target" special case. Asphalt must never become a facade.

The van is a **test stand-in**, not a target class. Buildings are the product.
The van remains a valuable regression fixture *because* it is short: correct
ground removal yields the van's facets and nothing else; incorrect removal is
immediately visible as downward-normal facets at ground level.

## Approach

Replace the percentile height cut with a **robust plane fit plus clearance
band**.

Rejected alternatives:

- **Local grid min-Z height field.** Handles undulating terrain, but a grid
  cell entirely interior to a large roof has `min_z` equal to the roof, so the
  roof deletes itself. Correcting that requires a progressive-TIN /
  morphological filter (CSF, PDAL) — a new dependency and real complexity. Its
  failure mode on buildings is worse than the plane fit's failure mode on
  terrain.
- **Auto-scale `ground_skip_m` from `height_range`.** Smallest diff, but
  inherits the original defect: a horizontal cut through a sloped ground.

## Design

### 1. `_ground_plane_keep_mask(pts, clearance_m) -> np.ndarray[bool]`

New private helper in `kmz_import.py`. Iteratively reweighted plane fit:

1. Least-squares fit `z = ax + by + c` over all points.
2. Keep points whose residual is below the 80th percentile.
3. Refit. Repeat 5 iterations.
4. Return `residual > clearance_m` — keeps points **above** the plane by more
   than the clearance, and drops everything below it (sub-surface noise).

Pure numpy, no new dependency. Validated on the flown cloud: converges to
2.50° tilt, and 39.5% of points land within 0.30 m of the fitted plane — the
asphalt, identified as one surface.

### 2. Both extractors swap the cut

`facades_from_pointcloud_cgal` (line ~991) and
`facades_from_pointcloud_ransac` (line ~1425):

```python
# was: pts_all = pts_all[pts_all[:, 2] >= ground_z + ground_skip_m]
pts_all = pts_all[_ground_plane_keep_mask(pts_all, ground_clearance_m)]
```

`ground_z` / `top_z` / `height_range` remain — they are used downstream for
classification and are not removed.

### 3. `ground_skip_m` is removed, not deprecated

It has no callers outside these two functions (verified: only two definitions,
two use sites, and two stale comments in `cli.py:245,266`). Keeping both knobs
would leave ambiguity about which one wins. The `cli.py` comments must be
updated in the same change — they currently describe the deleted behavior.

### 4. New parameter `ground_clearance_m: float = 0.35`

Exposed as `fd_ground_clearance_m` through the existing `fd_*` chain, which
every sibling facade-detection knob already uses:

```
kmz_import kwarg
  → api.py Pydantic Field (pattern: api.py:1734)   ge=0.05, le=2.0
  → client.ts body mapping (pattern: client.ts:262)
  → store.ts settings patch (pattern: store.ts:492)
  → Sidebar facade-detection panel
```

`ground_skip_m` was the **only** facade-detection parameter absent from this
chain, so this change also closes a dev-platform gap (CLAUDE.md: anything
user-facing is settable in the UI).

Default 0.35 m: the fit puts 39.5% of points within 0.30 m of the plane (the
asphalt); 0.35 m clears it with margin while preserving the van's lower body,
which begins ~0.4 m above the plane. The value is intended to be tuned in the
UI against real clouds, not defended as a constant.

## Testing

The golden fixture is written and passing **before** the extractor changes, so
the refactor either preserves Mijande or fails loudly.

1. **`test_mijande_facade_count_golden`** — pins the *current* Mijande facade
   count and total area. This is the regression gate. Written first, against
   unmodified code.
2. **`test_van_cloud_rejects_ground`** — on the 2026-07-10 flown cloud: assert
   zero facets with `normal_z < −0.7` (the downward-normal ground signature),
   and every facet centroid above the fitted ground plane. Fails today with 53
   offenders; passes after the fix.
3. **`test_ground_plane_fit_handles_slope`** — synthetic 2.5° plane + gaussian
   noise + a box. Assert the box survives and the plane does not.

**Fixture:** the flown cloud is 277k points / 3.3 MB — too large to commit. It
is decimated to ~15k points and committed as
`tests/data/van_parking_lot_2026-07-10.ply`. The extractor only sees 15,341
points after the polygon clip, so no information relevant to the test is lost.
The polygon is committed alongside it as JSON.

## Known risk

A genuinely non-planar site (terraces, steep slope, a building on a hill) will
fit one plane through varying terrain and either retain terrain or eat a
building's base. **The Mijande golden test does not cover this** — Mijande's
ground is planar. This risk is accepted in exchange for the simpler design.

The failure is visible with the same diagnostic used to find the original bug:
facets at ground level with downward normals. If such a site appears, add a
`ground_removal_method` enum with CSF/PDAL as a second method behind the same
interface, designed against that real cloud.

## Out of scope

Two symptoms observed on the 2026-07-10 flight are **not** addressed here and
are not caused by facade detection:

- **Low photo count** (143 photos where DJI shoots ~770 over the same path).
  Each intent waypoint carries a 5-pose `smart_oblique_poses` rosette; the
  augmented KMZ emits one `takePhoto` per waypoint, collapsing the rosette.
- **Slow flight.** The heading rewrite yaws the airframe: heading Δ p90 88.7°,
  max 167.1°, with 35% of waypoints demanding >30° of yaw from the previous.

The flight path itself is DJI Smart3D's, not AeroScan's — `augment_mission`
rewrites gimbal pitch/yaw and heading on DJI's waypoints and does not generate
the path. Fixing facade detection changes where the gimbal looks, not where
the aircraft flies.
