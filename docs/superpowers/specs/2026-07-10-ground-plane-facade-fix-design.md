# Ground-plane facade fix — design (as built)

**Date:** 2026-07-10
**Status:** IMPLEMENTED — landed as commit `d4cd1cb`, flown 2026-07-10 afternoon.
**Origin:** 2026-07-10 morning van scan (`test10-7`), a van in an empty parking lot.

> This doc has been rewritten after implementation to describe what was
> **actually built and measured**. The original design proposed a
> single-stage point-level plane clearance that did **not** work; that
> reasoning trail is preserved under "Superseded — what the original design
> got wrong" at the bottom.

## Problem

`facades_from_pointcloud_cgal` separated object from ground with a horizontal
height cut:

```python
ground_z = float(np.percentile(pts_all[:, 2], 5.0))
pts_all = pts_all[pts_all[:, 2] >= ground_z + ground_skip_m]   # ground_skip_m = 1.0
```

This assumes the target stands tall relative to both the ground's slope and
the cloud's noise. DJI perception clouds carry **~0.5 m of vertical ground
noise regardless of target size** — measured per 2×2 m cell: Mijande 0.52 m,
the 2026-07-10 parking lot 0.54 m. A fixed height cut therefore cannot
separate a short object from sloping ground.

On the 2026-07-10 morning flight the target was a ~2.7 m van on a 2.5°-sloping
asphalt lot. The cut **destroyed 43% of the van** (its lower body) and **kept
28% of the asphalt**. Extraction produced **80 facets, of which 29 were ground
facets covering 75.4 m²** — *more* ground area than the van's own area
(65.8 m²). The augmenter then aimed the gimbal at tarmac.

A horizontal cut cannot fix this. Across the ~13 m polygon a 2.5° slope spans
~0.57 m of Z **by geometry alone**, before noise (the asphalt's Z actually
ranged roughly −3.5 → +2.5 m because the reconstruction has sub-surface
points). No single constant separates a sloped ground from a short object.
Retuning `ground_skip_m` just retunes the bug.

### Why the obvious "ground signature" test does NOT work

An early hypothesis was that ground facets are identifiable by their outward
normal pointing down (`normal_z < −0.7`) and that a test could simply assert
"zero such facets." **That predicate is wrong and useless as a gate:**
Mijande — a correct, ground-free reconstruction — has **171 facets with
`normal_z < −0.7`** (roof underside facets, eaves, soffits, and other
legitimately down-facing structure). Down-facing normals are not a ground
signature. What distinguishes ground is being **near-horizontal AND close to
the fitted ground plane** — orientation *and* height together, tested against
a fitted plane, not against Z alone.

## Requirement

Remove the ground. Whatever stands above it is the inspection target. A van
and a house are the same problem at different scales — there is no
"short target" special case. Asphalt must never become a facade. The van is a
**test stand-in**, not a target class; it stays a valuable regression fixture
*because* it is short — correct ground removal yields the van's facets and
almost nothing else, and incorrect removal is immediately visible as
low, horizontal facets.

## Design — as built (two stages)

Point-level clearance alone left **22 ground facets**; the facet gate alone
left **10**. Neither is sufficient by itself. The shipped solution runs both.

### Stage 1 — `fit_ground_plane()` (point-level)

New public helper in `kmz_import.py` (`fit_ground_plane`, `kmz_import.py:895`).
Iteratively **trimmed** least squares fit of `z = ax + by + c`:

- `quantile=0.5`, `low_quantile=0.05`, `iterations=6`.
- Each pass refits only on points whose residual lies **between** the 5th and
  50th percentile, then recomputes residuals. This trims **both** tails: the
  upper tail is the structure standing on the ground, the lower tail is
  sub-surface reconstruction noise (the 2026-07-10 cloud has points ~3.5 m
  below grade). Trimming only the top drags the plane down into that noise —
  measured on the van scene, the plane sank 1.2 m and tilted to 7.6°, worse
  than the bug.
- `quantile=0.5`, **not 0.8**: at 0.8 the fit still sees enough of a building's
  mass to float upward. Measured on Mijande, q=0.8 kept only 56% of the cloud
  above a 0.35 m clearance where the old height cut kept 80% — it was eating
  the building.

The plane is fitted **before the mission-polygon clip**. The DJI
`mission_area_wgs84` polygon is drawn tight around the target, so inside it the
target can outvote the ground; the wider pre-clip cloud gives the fit a stable
majority of ground points (contract: ground must be >~50% of the input cloud).

Points are then kept where `height_above_ground(pts) > ground_clearance_m`
(`kmz_import.py:1073`). `height_above_ground()` returns the signed height of a
point above the fitted plane.

### Stage 2 — `_reject_ground_facets()` (facet-level gate)

New private helper (`_reject_ground_facets`, `kmz_import.py:940`), applied after
region growing produces the facade list (`kmz_import.py:1382`). For each facet:

- compute its centroid's height above the fitted plane;
- treat it as horizontal if `|normal_z| >= roof_normal_z_min` (0.70);
- **drop it only if it is both horizontal AND within
  `ground_facet_clearance_m` of the plane.**

Testing **orientation as well as height** is what makes this safe: a wall is
vertical, so it survives at any height, and so does a lamppost. Only
horizontal-and-low facets go, which is exactly what tarmac is. This catches the
residual ~0.5 m noise tail that region growing fits rectangles to and that no
point-level clearance small enough to preserve a short target can remove.

### Parameters

- `ground_clearance_m: float = 0.4` (point-level, Stage 1).
- `ground_facet_clearance_m: float = 1.5` (facet gate, Stage 2).
- `ground_skip_m` is **removed** (no external callers). The `facades_from_
  pointcloud_ransac` path takes the same two-stage treatment
  (`kmz_import.py:1471`, `:1515`).

## Measured results (at `ground_clearance_m=0.4`, vs the pre-fix baseline)

| scene | metric | before | after |
|---|---|---|---|
| Mijande | facets | 818 | **812** |
| Mijande | walls | 128 | **136** |
| Mijande | total area | 2098.5 m² | **2089.8 m²** |
| van (bench fixture) | on-target area | 65.8 m² | **63.8 m²** |
| van (bench fixture) | off-target area | 75.4 m² | **5.7 m²** |
| van (actually-flown cloud) | off-target area | 75.4 m² | **11.2 m²** |

Mijande keeps **more** walls than before (the plane fit stops the old cut from
occasionally shaving a wall base). The van's off-target surface drops ~13×.
The full suite is **153 passed**.

## Testing (as built)

`tests/test_ground_removal.py` — **7 tests**:

1. `test_ground_plane_recovers_a_synthetic_slope`
2. `test_ground_plane_holds_when_ground_is_the_majority`
3. `test_ground_plane_survives_sub_surface_noise`
4. `test_height_above_ground_sign`
5. `test_van_scene_ground_is_not_detected_as_facades` — the flown cloud: assert
   on-target van area survives (>55 m²) and off-target surface collapses
   (<15 m²).
6. `test_van_scene_keeps_no_horizontal_facet_at_ground_level`
7. `test_mijande_building_survives_ground_removal` — the regression gate: assert
   facets `> 780`, area `> 2040 m²`, walls `>= 128`. It **asserts bounds, not
   equality** — the count legitimately moves 818 → 812, so a golden equality
   assertion would be wrong.

**Fixture:** `tests/data/van_parking_lot_2026-07-10.ply` — the real
2026-07-10 cloud at **full density**. It is *not* decimated: decimating drops
it below CGAL's `min_density_per_m2` gate and yields zero facets, so a decimated
fixture would test nothing.

## Known risk (accepted)

A genuinely non-planar site (terraces, a steep slope, a building on a hill)
will fit one plane through varying terrain and either retain terrain or eat a
building's base. The Mijande golden test does not cover this — Mijande's ground
is planar. This risk is accepted in exchange for the simpler design. If such a
site appears, add a `ground_removal_method` enum with a progressive-TIN /
morphological filter (CSF/PDAL) as a second method behind the same interface,
designed against that real cloud.

## Still open after this change

- **UI plumbing.** `ground_clearance_m` / `ground_facet_clearance_m` are
  **hardcoded**, not yet exposed through the `fd_*` UI chain (kmz_import kwarg →
  api.py Pydantic Field → client.ts → store.ts → Sidebar). The CLAUDE.md
  dev-platform rule wants every user-facing knob settable in the UI, so this is
  a remaining gap. The original design assumed this plumbing would land in the
  same change; it did not.

## Out of scope (not caused by facade detection)

`augment_mission` does **not** generate the flight path. It takes DJI Smart3D's
waypoints from the RC-exported KMZ and rewrites **only** gimbal pitch/yaw plus
aircraft heading; the spiral-around-the-target pattern is DJI's. Fixing facade
detection changes where the gimbal **looks**, never where the aircraft
**flies**. So these observed symptoms are separate work:

- **Low photo count** — each intent waypoint carries a 5-pose
  `smart_oblique_poses` rosette; the augmented KMZ emits one `takePhoto` per
  waypoint, collapsing the rosette (143 photos over a ~220 m path, 1.54 m
  spacing, where DJI Smart3D shoots ~0.65 m).
- **Heading/gimbal coupling** — the airframe heading barely turns, so required
  gimbal yaw can exceed the gimbal's pan range and it clamps.

---

## Superseded — what the original design got wrong

The pre-implementation design (approved 2026-07-10, before the code was built)
made three claims that turned out to be wrong. They are kept here so the
reasoning trail survives.

1. **"53 of 80 facets were tarmac."** Wrong — a miscount. 53 was the number of
   facets *near* the van, not the ground facets. The true figure is **29 of 80
   ground facets, 75.4 m²**.
2. **"The ground signature is `normal_z < −0.7` (downward normals), and the
   test asserts zero such facets."** Wrong and useless as a predicate — Mijande
   (correct) has **171** such facets. Ground is distinguished by
   *near-horizontal AND near the fitted plane*, not by a down-facing normal.
   See "Why the obvious 'ground signature' test does NOT work" above.
3. **"Point-level plane clearance alone (clearance 0.35, quantile 0.8) is the
   fix."** Wrong on both counts:
   - clearance-alone left **22 ground facets** (the ~0.5 m noise tail survives
     any clearance small enough to keep a short target) — hence the added
     facet gate (Stage 2);
   - `quantile=0.8` let the building float the fit upward (kept only 56% of
     Mijande above clearance) — the shipped fit uses `quantile=0.5` and trims
     **both** tails.
4. **"The golden test asserts equality of the Mijande facet count."**
   Impossible — the count legitimately moves 818 → 812. The shipped test
   asserts bounds, not equality.
