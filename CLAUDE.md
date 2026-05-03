# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AeroScan is a NEN-2767 building inspection flight planner that generates DJI WPML-compliant KMZ waypoint missions for the DJI Matrice 4E drone. Full-stack app: Python/FastAPI backend + React/TypeScript/Three.js frontend.

We are also building a **dev platform** alongside this tool. All configuration and settings must be exposed and controllable from the frontend — the only thing that lives purely in backend code is the deployed logic itself. If a parameter, option, or behavior can be user-facing, it should be settable in the UI.

## Commands

### Development

```bash
./run.sh                    # Start both backend (:8111) and frontend (:3847)
python -m flight_planner    # Backend only (uvicorn on :8111)
cd frontend && npx vite --port 3847  # Frontend only
```

### Testing

```bash
pytest                      # Run all tests (testpaths and pythonpath configured in pyproject.toml)
pytest tests/test_geometry.py -v              # Single test file
pytest tests/test_geometry.py::test_name -v   # Single test
```

### Frontend

```bash
cd frontend
pnpm install        # Install dependencies
pnpm run build      # TypeScript check + Vite production build
pnpm run lint       # ESLint
```

### Install (Python)

```bash
pip install -e ".[dev,server]"      # Core + test + server deps
pip install -e ".[dev,server,mesh]" # Also include mesh import (open3d, trimesh)
```

## Architecture

### Backend (`src/flight_planner/`)

**Entry point**: `__main__.py` starts uvicorn on port 8111.

**Core pipeline**: Building → Facades → Waypoint Grid → Validation → KMZ Export

- **`models.py`** — Dataclasses for everything: `Building`, `Facade`, `Waypoint`, `CameraSpec`, `MissionConfig`. Matrice 4E hardware constants are hardcoded here (gimbal limits, camera specs, flight constraints).
- **`geometry.py`** — The computational core. `build_rectangular_building()` creates facades (walls + roof). `generate_waypoints_for_facade()` builds a boustrophedon (lawnmower) grid on each facade plane using a local (u, v) coordinate frame. `generate_mission_waypoints()` orchestrates the full pipeline: per-facade grid or surface sampling → path optimization → exclusion zone filtering → interior waypoint filtering (convex hull) → WGS84 conversion. Supports `disabled_facades` and `exclusion_zones`.
- **`camera.py`** — GSD, footprint, and grid spacing calculations from sensor/focal-length params.
- **`kmz_builder.py`** — Wraps the `djikmz` library to produce DJI Pilot 2 compatible KMZ files.
- **`building_import.py`** — Parses GeoJSON footprints and 3D mesh files (OBJ/PLY/STL/GLB/GLTF via trimesh) into Building objects. Facade extraction uses region growing on the face adjacency graph. Interior faces are filtered via visibility ray casting (ray from outside the bounding box toward facade center — if another face blocks it, the facade is interior). `trimesh.repair.fix_normals()` is called on mesh load to ensure consistent winding.
- **`validate.py`** — Checks mission constraints (max waypoints, altitude limits, gimbal bounds, flight time). Returns typed `ValidationIssue` objects with error/warning/info severity.
- **`visualize.py`** — Serializes facades + waypoints to JSON for Three.js and Leaflet viewers.
- **`reconstruct.py`** — Simulated photogrammetric reconstruction pipeline. Renders synthetic photos from mission waypoints using pyrender (subprocess for macOS GPU thread safety), fuses depth maps via Open3D TSDF with realistic noise (depth σ=5mm, pose σ=1cm/0.15° simulating MVS+SfM error), then reimports the reconstructed mesh through `build_building_from_mesh()`. Uses the raw uploaded mesh for rendering (not extracted facade planes). Results are persisted to SQLite (`SimulationRecord` table). Also exports COLMAP-compatible format (`cameras.txt`/`images.txt`/`points3D.txt`) for offline full reconstruction with COLMAP/ODM. Quality presets control render_scale (5-20%) and voxel_size (2-8cm).

**Server** (`server/`):
- **`api.py`** — FastAPI router under `/api`. Key endpoints: `POST /generate` (mission generation), building CRUD, version history, KMZ download, simulation CRUD (`POST /simulate-reconstruct`, `GET /simulate-reconstruct/{id}`, `DELETE /simulate-reconstruct/{id}`).
- **`database.py`** — SQLAlchemy with SQLite (`aeroscan.db`). Tables: `BuildingRecord` (uploaded buildings), `SimulationRecord` (reconstruction results — persists across restarts).
- **`state.py`** — In-memory `MissionVersion` store (cleared on restart, max 100 versions). Stores selection state (disabled facades, enabled candidates, exclusion zones) per version.

### Frontend (`frontend/src/`)

- **`store.ts`** — Zustand store: building params, mission params, generated results, version history, UI state. All API calls live here.
- **`api/client.ts`** — Typed fetch wrapper mirroring backend routes.
- **`api/types.ts`** — TypeScript interfaces that mirror the Python Pydantic/dataclass models exactly.
- **`components/Viewer3D.tsx`** — Three.js canvas via react-three/fiber. Three view modes (Selection/Plan/Flight) controlled by a segmented toggle. FacadeMesh uses two-pass rendering (invisible depth-only pass + transparent visual pass) for proper occlusion. Facades are clickable to toggle disabled state. Exclusion zones are draggable via drei PivotControls.
- **`components/Sidebar.tsx`** — Building config (presets vs. upload), mission parameters, file upload (drag-drop), stats, validation display, version history. Facade list with checkboxes, exclusion zone list with type selector.
- **`components/MapView.tsx`** — Leaflet satellite map with waypoints and flight path. Leaflet-draw integration for drawing exclusion zones (rectangles + polygons). Zone type selector (inclusion/no-fly/no-inspect).
- **`components/DroneAnimation.tsx`** — Animated drone flying the mission path.

### Coordinate Systems

- **ENU (East-North-Up)**: Local coordinate system relative to building reference GPS point. Used for all geometry calculations.
- **WGS84**: Global lat/lon/alt for GPS waypoints and KMZ export.
- **Three.js transform**: ENU (x=E, y=N, z=Up) → Three.js (x=right, y=up, z=toward camera). Applied in `Viewer3D.tsx`.

### Data Flow

1. User configures building (preset or upload) + mission params in Sidebar
2. Frontend POSTs to `/api/generate` with `BuildingParams` + `MissionParams` + `disabled_facades` + `exclusion_zones`
3. Backend: builds facades → filters interior faces (visibility ray test) → generates waypoint grids → path optimization → exclusion zone filtering → interior waypoint filtering → validates → serializes viewer data
4. Response includes: waypoints, summary metrics, validation issues, Three.js/Leaflet data, version ID, config snapshot (including selection state)
5. Frontend renders 3D view (Selection/Plan/Flight modes) + map + simulation tab; user can download KMZ via `/api/versions/{id}/kmz`

### Simulation Pipeline

1. User clicks quality preset (Super Fast/Fast/Medium/High) in Sidebar → `POST /api/simulate-reconstruct`
2. Backend spawns subprocess: exports raw mesh as PLY → renders RGB+depth from each waypoint via pyrender → adds realistic noise → fuses via TSDF → decimates → exports reconstructed PLY + COLMAP format
3. Reimports reconstructed mesh through `build_building_from_mesh()` → facade extraction → new mission generation
4. Result (viewer data, comparison metrics, photo metadata) saved to SQLite
5. Frontend Simulation tab shows all runs with switcher, comparison stats, and 3D viewer in Flight mode

**Key design decisions:**
- **pyrender** for rendering (not Open3D) — Open3D's OffscreenRenderer requires EGL which doesn't work on macOS. Pyrender uses pyglet's native macOS OpenGL.
- **Subprocess** for GPU rendering — macOS requires GPU contexts on the main thread; uvicorn handlers run on worker threads. Subprocess gets its own main thread.
- **OpenGL→OpenCV matrix conversion** — pyrender uses OpenGL convention (Y-up, -Z-forward), TSDF uses OpenCV (Y-down, Z-forward). Conversion: `w2c_cv = flip @ inv(c2w)` where flip=diag(1,-1,-1,1). The flip must be **pre-multiplied** (applied to camera-space output).
- **Raw mesh for rendering** — the drone camera sees the actual building, not extracted facade planes. Raw mesh is fetched from viewer_data or database.

## Key Domain Concepts

- **NEN-2767**: Dutch building inspection standard requiring systematic facade coverage.
- **GSD (Ground Sample Distance)**: Target resolution in mm/pixel — drives camera-to-facade distance.
- **Boustrophedon pattern**: Lawnmower sweep across each facade with row reversals.
- **Facade normal**: Outward-facing direction vector used to compute camera standoff position and gimbal angles.
- **NL-SfB codes**: Component classification tags on facades (e.g., "21.1" for brick walls).
- **Exterior vs interior faces**: Mesh models (especially non-photogrammetry) can have interior geometry (room walls, floors). Interior faces must be filtered during facade extraction. The correct approach is visibility ray casting — cast from outside the bounding box toward the face center; if the face is the first hit, it's exterior.

## Facade Extraction & Waypoint Safety

The mesh → facade → waypoint pipeline has multiple safety layers:

1. **`trimesh.repair.fix_normals()`** on mesh load — consistent face winding
2. **Centroid-based normal orientation** — initial outward guess
3. **Visibility ray test** — cast from far outside along ±normal to verify the facade is reachable from outside the building. Interior faces (not visible from any direction) are rejected
4. **Envelope post-filter** (`_extract_region_growing` in `building_import.py`) — after region growing produces the facade list, probe each facade at `center + normal * 2m` and ray-parity test against the full mesh (`RaycastingScene.count_intersections`). If the probe is inside the envelope, the facade's "outward" normal was flipped wrong (typical on L/U-shaped buildings where centroid-based flipping is ambiguous in concavities) and the facet is demoted to a `candidate_interior_*` candidate. Works on non-watertight photogrammetry meshes — don't gate on `mesh.is_watertight`. Caught ~11 interior facets per MijandeExtra import.
5. **Per-facade normal check** — before generating waypoints, verify normal orientation via visibility ray
6. **Open3D ray-parity containment filter** — replaces convex hull. Uses `RaycastingScene.count_intersections()` (odd = inside) which correctly handles non-convex buildings (L-shaped, U-shaped, courtyards). Works on non-watertight meshes.
7. **LOS (line-of-sight) check** — per-waypoint ray cast to verify clear view to facade surface. LOS sample-offset reach is clamped to 40% of the facade's extent so narrow sills/parapets don't sample off-facade space and spuriously report "not visible".
8. **Direct-line inter-facade transits** — `generate_mission_waypoints` concatenates facade groups into a single trajectory without inserting unconditional outward+altitude transit waypoints between every facade pair. The old behaviour routed outward +2m and up +2m between every facade regardless of geometry, producing zig-zag paths that frequently left the DJI polygon without any safety benefit when the direct line was already clear. Now the only transits are the collision-resolution detours (#9).
9. **Path segment collision check** — for each consecutive waypoint pair, casts a ray along the flight path segment and checks for mesh intersections. Collisions are resolved by inserting altitude detour waypoints routed outward from the building. Unresolvable collisions are flagged as validation errors.
10. **Polygon standoff clamp** (`_clamp_standoff_to_polygon` in `geometry.py`) — for KMZ-sourced missions, before generating the per-facade grid, cast a ray from the facade centroid along the outward normal and find where it exits the DJI `mission_area_wgs84` polygon; clamp the standoff distance to `(exit - 0.5m buffer)` with a floor at `max(min_photo_distance_m, obstacle_clearance_m)`. Without this, edge-hugging facades generate WPs at nominal GSD-driven standoff that all land outside the polygon and get clipped by #11, leaving the facade uncovered. Accepts a tighter-than-nominal GSD for those facades in exchange for actual coverage.
11. **Mapping-polygon WP clip** (`_clip_waypoints_to_dji_bbox` in `server/api.py`) — for KMZ-sourced missions, drop inspection waypoints whose XY falls outside the DJI `mission_area_wgs84` polygon (2m margin, matches the facade filter). Transit waypoints bypass the clip — short slews across the edge are unavoidable when facades hug the boundary. Emits `mapping_polygon_clipped` validation warning listing affected facade indices.
12. **Raw point-cloud obstacle filter** (`_filter_waypoints_by_pointcloud` in `server/api.py`) — for KMZ-sourced missions, KDTree over the raw DJI `cloud.ply`. Any WP whose `obstacle_clearance_m` ball contains a cloud point outside the 60° viewing cone toward its target facade is dropped. Catches thin obstacles the alpha-wrap mesh smooths away (wires, branches, fences, railings, antennas). Transit WPs use the full clearance ball with no cone relief. Runs *after* the polygon clip so we don't waste KDTree queries on WPs destined to be clipped anyway. Emits `pointcloud_obstacle` validation warning.
13. **Coverage-health warning** (`validate.py`) — facades ≥ 2m² that end up with zero inspection WPs after the full pipeline surface as a `facades_uncovered` validation warning, so zero-photo walls are visible at plan time instead of at flight time.

### Mesh containment
The old `mesh.convex_hull.contains()` approach failed for non-convex buildings (concave regions were incorrectly classified as "inside"). Open3D ray-parity (`count_intersections`, odd = inside) correctly handles all geometries including non-watertight photogrammetry meshes. `mesh.contains()` from trimesh still requires watertight meshes — avoid it.

## DJI KMZ Import & Facade Detection on Raw Point Clouds

DJI Smart3D / AutoExplore KMZ missions ship a point cloud (sparse, noisy) plus a `mission_area_wgs84` polygon. The inspection pipeline has to produce NEN-2767-grade facades from this. `src/flight_planner/kmz_import.py` handles it:

### Pipeline (raw KMZ → facades)

1. **Point cloud clip** — keep only points inside `mission_area_wgs84`; drop the bottom ~1m of Z to strip terrain.
2. **Mesh reconstruction** (`pointcloud_to_mesh_ply`) — default is **CGAL `alpha_wrap_3`** (Cohen-Steiner et al.), which produces a **watertight, orientable, 2-manifold** shrink-wrap in a single call. No smoothing or decimation pass needed afterward. Open3D `create_from_point_cloud_alpha_shape` is retained as a fallback but silenced to verbosity level Error (it spams "invalid tetra in TetraMesh" warnings on noisy DJI clouds). Tuning: `alpha ≈ 2× voxel_size` and `offset ≈ 1× voxel_size` — tight wrap preserves small features like sills, dormers, and parapets. Coarser ratios smooth those away.
3. **CGAL Shape Detection** (`facades_from_pointcloud_cgal`). Default `algorithm="region_growing"` via `CGAL::Shape_detection::region_growing` — grows regions locally from high-planarity seeds using k-nearest neighbors, stopping at the first neighbor that fails the ε / normal-agreement test. Each point belongs to at most one region, so regions are connected and non-overlapping — CGAL's native answer to "many small facets." `algorithm="efficient_RANSAC"` (Schnabel et al. 2007, "Efficient RANSAC for Point-Cloud Shape Detection") remains as an alternative for LOD-style reconstruction where fewer/bigger dominant planes are wanted. In both cases, normals are estimated upstream via `jet_estimate_normals` and the per-region plane equation is refit via SVD on its inliers.
4. **Parallel-snap regularization** — Verdie et al. 2015 ("LOD Generation for Urban Scenes"). CGAL's C++ `regularize_planes` isn't exposed in the Python bindings, so the parallel-snap step is done inline in numpy: planes whose normals agree within 5° share one refit normal (weighted by inlier count). Fixes the "plates slicing through each other" artifact where slightly-misaligned regions would otherwise render as overlapping rectangles. Coplanar-merge (the companion step) is OFF by default — for NEN-2767 inspection we *want* spatially-separated facets on the same wall plane to stay separate, so the gimbal can square up on each one.
5. **Density gate** — reject any plane with inlier density < `min_density_per_m2` (default 25). Kills "ghost planes" where scattered inliers span a huge floating rectangle.
6. **PCA-oriented rectangle** — each region's bounding rectangle is built in its own plane frame (PCA u/v axes, 5/95 percentile clamp), so pitched roofs, dormers, chamfers, and canopies keep their real tilt instead of being flattened to an axis-aligned XY box.
7. **RC Plus mission-polygon filter** — `_filter_facades_by_dji_bbox` drops any facade whose XY centroid falls outside the DJI `mission_area_wgs84` polygon (from `template.kml`), expanded by a 2m margin so facades hugging the polygon edge still survive. This polygon is what the RC Plus shows on-controller as the mapped region and is the single source of truth for facade planning scope — the same polygon drives the viewer's "Mapping box" toggle via `PolygonBoxView`. Applied at KMZ import, refine, AND at `/generate` so every regeneration (including the NEN-2767 inspection mission) honours the same gate.

### Why region_growing over efficient_RANSAC for inspection

`efficient_RANSAC` extracts the largest plane first and claims all its inliers before moving on, so a noisy wall tends to steal all its neighbors' points and get fit as one giant rectangle. The natural lever to "get more facets" on the RANSAC side is to reduce `cluster_epsilon`, but that often just produces the same giant plane split at noise gaps. Region growing gives the locality guarantee for free: a region stops where the surface bends, where the inliers thin out, or where the k-NN neighborhood runs out of good candidates. Small connected surfaces become small regions; the inspection gimbal gets a separate target per facet. This is why we avoided manual grid-subdivision — the CGAL API already handles it.

### Small-feature bias (NEN-2767)

Defaults are tuned for **many small facets over few big ones** because the inspection gimbal has to square up on small defect targets (window ledges, balcony panels, dormer walls), not just whole walls:

- `min_points=40`, `epsilon=0.05m`, `cluster_epsilon=0.25m` — fine-grained plane extraction.
- `min_wall_area_m2=0.5`, `min_roof_area_m2=0.5`, `min_tilted_area_m2=0.4` — accept small facets.
- Wall dim floor: one edge ≥ 0.4m, longest ≥ 0.6m (sills, parapets survive).
- `min_density_per_m2=25` — tight enough to kill floaters but loose enough to keep small dense facets.

### Voxel / alpha_wrap / RANSAC must agree on scale

These knobs are interlocked. For reasonable DJI Smart3D clouds:

| knob | default | rule of thumb |
|---|---|---|
| voxel_size (auto) | 0.03–0.20m | adaptive to target 120K points |
| alpha_wrap α | 2 × voxel | smaller = more detail, noisier |
| alpha_wrap offset | 1 × voxel | tighter = hugs noise; looser = smoother |
| ransac ε | ≈ voxel | plane-fit tolerance |
| ransac cluster_ε | ≈ 4–5 × voxel | max inlier gap |
| regularize parallel_tol | 5° | snap near-parallel to exact |
| regularize coplanar_tol | 1.5 × ε | merge near-coplanar |

### What `mission_area_wgs84` actually is (and isn't)

The polygon in `template.kml` is the **mapping target** (the on-RC approved region the DJI Smart3D pilot photographed), not a flight envelope. On MijandeExtra, ≈20% of DJI's own waypoints land *outside* the polygon because the drone orbits the target at standoff distance. Don't read the polygon as "the drone stayed in here" — it didn't, and neither can any custom inspection path that wants to photograph the same walls.

What we use the polygon for:
- **Facade scoping**: `_filter_facades_by_dji_bbox` drops facets whose centroid is outside (with a 2m margin) so the inspection mission only plans for walls the pilot actually mapped.
- **Viewer rendering**: `PolygonBoxView` renders the polygon's XY extent as a translucent box ("Mapping box" toggle) so the pilot can visually compare custom-path coverage against DJI's scope. The box persists across the DJI-path / Custom-path mode switch because `/api/generate` plumbs `mission_area_wgs84` into both the Three.js `missionArea` and Leaflet `missionAreaPoly` payloads on every regeneration (not just at import time).
- **Per-facade standoff clamp** (layer 10 in Facade Extraction & Waypoint Safety above): pull the camera in toward the wall when the nominal GSD-driven standoff would push it outside the polygon, rather than generating a WP that gets clipped.
- **Inspection-WP clip** (layer 11): drop any inspection WP whose XY lands outside the polygon. Transit WPs bypass — short slews across the edge are unavoidable when facades hug the boundary.

The authoritative safety source for "where the drone can physically fly" is the raw `cloud.ply`, not the polygon. Layer 12 (point-cloud obstacle filter) handles that.

### Auto-populating the sidebar from the DJI KMZ

On import, `_derive_dji_mission_seeds` pulls site-proven values off the KMZ and stores them in `MissionVersion.mission_params`, which flows through `config_snapshot.mission` on every `/generate` response:

| Sidebar field | Derived from | Notes |
|---|---|---|
| `flight_speed_ms` | `autoFlightSpeed` (template.kml) | Already wired — used in DJI-replay mode. Not representative of actual flight speed (see Smart3D rosette section below). |
| `front_overlap` | `orthoCameraOverlapW` | DJI stores 0–100, we convert to 0–1 fraction. |
| `side_overlap` | `orthoCameraOverlapH` | Same conversion. |
| `obstacle_clearance_m` | median of DJI-WP perpendicular distance to nearest facade plane (top 10% dropped as transit outliers), clamped to [1, 15]m | Site-proven standoff — what the aircraft actually flew at successfully. |

Plus a `dji_extracted` sub-dict carrying the raw values for UI display. The sidebar renders a `DJI-extracted: front X%, side Y%, observed standoff Zm` hint under the overlap sliders when an imported KMZ is active.

On entering Custom-path mode for the first time (`generateInspectionMission` in `store.ts`), the mission seed is `{ ...NEN2767_MISSION, ...dji_seeds }` — the NEN-2767 preset supplies inspection-specific values (GSD, speed, gimbal margin) while DJI seeds override site-specific ones (overlap, clearance). Subsequent slider edits stick (no re-seeding).

## Smart3D rosette pitch — what the KMZ encodes vs what the drone shoots

DJI Smart3D missions have two layers of "where the camera was pointing," and they are NOT the same:

### What the KMZ encodes (mission *intent*)

Three pitch fields appear in `wpmz/waylines.wpml`:

| Field | Meaning | Range observed in our KMZs |
|---|---|---|
| `smartObliqueEulerPitch` | Per-pose rosette pitch, 5 poses per `startSmartOblique` action | **−90° to +30°** (MijandeExtra tops out at +20°) |
| `waypointGimbalPitchAngle` | Single gimbal pitch for non-rosette WPs | always negative (e.g. MijandeExtra: −89° to −10°) |
| `gimbalPitchRotateAngle` | Action-driven gimbal rotation target (rare) | mission-specific |

**Convention:** `0° = horizontal forward`, `−90° = nadir (straight down)`, positive = looking up. Matches DJI's standard WPML convention.

**Hardware bound:** Matrice 4E gimbal soft/controllable limit is **−90° to +35°** (mechanical −140° to +50°). `models.py` `GIMBAL_TILT_MIN/MAX_DEG` matches. **No KMZ in `/kmz/` exceeds +30°** — there is no "straight up to the sky" pose anywhere in our sample missions.

In MijandeExtra specifically: 584 waypoints, only 248 have a rosette block; the rest are transit WPs with a single `waypointGimbalPitchAngle`. Per-WP rosette structure is consistently `[center, +Δ, +Δ, −Δ, −Δ]` with ±30° yaw offsets — a 5-pose fan around the WP heading.

### What the drone actually shoots (capture *reality*)

`smartObliqueCycleMode=unlimited` means the gimbal keeps cycling through the active rosette while flying *between* capture WPs. Combined with action-triggered shutter, the photo count >> rosette pose count (MijandeExtra: 1333 photos / 248 rosette WPs ≈ 5.4 photos per rosette). **You cannot determine which physical photo index corresponds to which pose from the KMZ alone** — that mapping depends on runtime gimbal slew speed, flight speed, and shutter timing.

### `waypointSpeed`/`autoFlightSpeed` in WPML are NOT the actual flight speed

DJI Smart3D missions report `autoFlightSpeed=0.7 m/s` and per-WP `waypointSpeed=1.0 m/s` in the WPML, but the drone actually flies the inspection passes at ~2 m/s. Empirical calibration from MijandeExtra:

- Total inspection-pass length ≈ 850 m
- DJI-reported total photos: 1333
- DJI-reported total pass time: 421.7 s
- → effective speed ≈ 2.02 m/s, inter-shot spacing ≈ **0.64 m**

Using the WPML speed × shutter interval to compute "shots per segment" yields ~5×-too-dense and ~2.4×-too-long output. The viewer's `OriginalGimbalArrows` (`Viewer3D.tsx`) samples by **distance** (`DJI_SMART3D_INTER_SHOT_DIST_M = 0.65`) instead of by time, sidestepping the bad speed field. The pose cycle counter walks continuously across the whole flight without resetting at WP boundaries — `cycleMode=unlimited` cycles 0→1→2→3→4→0 forever, with each `startSmartOblique` block only updating *which direction* poses 0-4 point, not restarting the counter.

### MijandeExtra rosette-template drift

248 overlapping `startSmartOblique` action groups, each spanning 2-3 WPs, 69 unique pose templates. Pitches drift smoothly across waypoints (e.g. `−37,−7,−67` → `−35,−5,−65` → `−21,9,−51`...) as DJI re-aims the rosette to track the building during flight. **Don't reset the pose-cycle counter on template change** — DJI just hot-swaps which way each pose-index points, the cycle keeps walking.

### Ground truth lives in the JPEG EXIF/XMP

The actual gimbal angles at exposure are stamped into every captured JPEG by DJI's flight controller:

- `drone-dji:GimbalPitchDegree` — actual pitch at shutter
- `drone-dji:GimbalYawDegree` — actual yaw at shutter
- `drone-dji:GimbalRollDegree` — actual roll at shutter (always ~0)
- `drone-dji:FlightPitchDegree` / `FlightYawDegree` / `FlightRollDegree` — aircraft body angles

If a user reports "this photo looks straight up but the KMZ doesn't say so," the resolution path is:

1. Read the JPEG XMP, not the WPML — XMP has the truth
2. If XMP confirms a pitch outside the rosette spec, DJI's runtime is overriding the mission (e.g. autonomous calibration shots, gimbal overshoot during slew). The WPML alone cannot model this.
3. The frontend's `OriginalGimbalArrows` / `RosetteDiagnostic` (`Viewer3D.tsx`) renders mission-intent poses from `waypoint.smart_oblique_poses`. Visualizing actual capture orientation would require ingesting the photos' XMP — not currently implemented.

## KMZ Execution Transport (Pilot 2 / PSDK / Manifold)

AeroScan is the **authoring surface**: building geometry in, WPML-compliant KMZ out. The KMZ then needs an **execution transport** to reach the Matrice 4E. See `docs/architecture/kmz-flow.md` for the full writeup; the constraints that drive the current and future architectures:

- **Today's path is manual SD-card sideload** into Pilot 2's mission library. Zero PSDK, works today, is what `kmz_builder.py` targets. Don't assume anything beyond this is in place.
- **There is no PSDK API to inject a KMZ into Pilot 2's mission library.** User-facing library import is SD-card only. MOP (`dji_mop_channel.h`) is a PSDK↔MSDK/OSDK peer channel, not a Pilot 2 back-channel.
- **PSDK Custom Widgets are live-link only** — they render on Pilot 2's live-flight view while the aircraft is powered and linked, and are not visible from the pre-flight mission library screen.
- **The DJI-sanctioned on-drone automation path is Manifold + PSDK**: `DjiWaypointV3_UploadKmzFile(bytes, len)` uploads the KMZ directly to the aircraft, `DjiWaypointV3_Action(START|STOP|PAUSE|RESUME)` controls execution, and a Custom Widget on Pilot 2's live view gives the pilot a tap-to-fly button. Reference sample: `samples/sample_c/module_sample/waypoint_v3/` in the PSDK source. In this shape Pilot 2 is the monitoring surface, not the mission chooser — the pilot does not browse the KMZ from the library.
- **Smart 3D Capture / Explore's output is not known to be an interceptable KMZ.** DJI's public docs describe Smart 3D as a single end-to-end automated mission (first-pass sparse cloud → on-RC facade route → autonomous flight) and do not document any KMZ export step. Any "Smart3D-first" architecture is gated on a device test confirming an accessible `.kmz` / `.wpml` on the RC or aircraft filesystem after a Smart 3D run. Until that test runs, the source-of-KMZ story stays AeroScan-first; `kmz_import.py` already accepts a Smart3D KMZ containing a *point cloud* as planner input, which is a distinct (and verified) path.

## Smart3D outputs on the Manifold (`/blackbox/`) — verified 2026-05-03

A read-only investigation of the Manifold (`ssh dji@<manifold>`) shows that **the Smart Auto-Exploration mesh is on the Manifold's filesystem in plain PLY**, not just on the RC. This collapses the "we need to wirelessly transport a 20-100 MB KMZ from RC to Manifold" framing — for the mesh, no transport is needed.

```
/blackbox/the_latest_flight/         (DJI-maintained symlink → /blackbox/flightNNNN/)
├── camera/
├── dji_mcu/                         (encrypted MCU log)
├── dji_perception/1/
│   ├── mesh_binary_*.ply            ← BUILDING GEOMETRY: ~50 chunks × ~10 MB,
│   │                                  binary little-endian PLY, vertex+normal
│   │                                  point cloud (no faces), local meter-scale
│   │                                  ENU frame anchored on takeoff. ~1 GB / flight.
│   ├── *.enc                        (encrypted: expl_plan.bin.enc, raw_data.enc,
│   │                                 vp.log.enc — opaque)
│   └── vp_storage.json              (volume layout, NOT mission data)
├── psdk/                            (PSDK app log, plain text)
├── system/
├── latest_Smart3DExplore            (1-byte marker — flight ran Smart3D)
└── Smart3DExplore_dynamic_ch_v2.json  (IPC channel config, NOT mission data)
```

What this means for the planner:

- **Mesh source:** AeroScan can pull `mesh_binary_*.ply` directly from `/blackbox/the_latest_flight/dji_perception/1/` over SSH (LAN at depot, USB-tether to M4E debug port at `192.168.42.120` in the field — DJI documents this for PCs in `Payload-SDK-Tutorial/docs/en/40.manifold-quick-start/04.development-environment-setup/01.hardware-environment-setup.md` line 69; hot-plug not supported, cable before aircraft power-on).
- **Density vs KMZ cloud:** Manifold has the unfiltered cloud, ~18.8 M points across all chunks for a typical flight. The KMZ's `cloud.ply` is a curated/decimated version (~416 K points). The existing `facades_from_pointcloud_cgal` pipeline runs unchanged on the merged Manifold cloud (with a 10 cm voxel downsample) and produces ~2.8× more facades than from the same KMZ — denser source, finer structural detail.
- **Flight plan source:** The flight plan (waypoints, gimbal commands, capture actions) is **not** on the Manifold in any plaintext form. `expl_plan.bin.enc` is encrypted and opaque. No `.kmz` or `.wpml` exists outside the PSDK sample tree. The flight plan therefore still has to come from the **Smart3D KMZ on the RC**, which the pilot exports via USB-MTP cable to the laptop in the existing manual workflow.
- **Flight identification:** Use `/blackbox/the_latest_flight` (DJI-maintained symlink); don't try to track `flightNNNN` numbers.

The architectural consequence: **AeroScan reads mesh from Manifold, reads flight plan from RC-exported KMZ, augments only gimbal pitch/yaw per waypoint, and pushes the modified KMZ back to the Manifold over the same wired link.** The wireless MOP transport (the rc-companion app) is preserved as a **control-message channel only** for sub-500 KB payloads (mission selection commands, status pulls). See `docs/architecture/rc-companion-summary-exec.md` for the full architecture and `docs/architecture/rc-companion-bringup.md` for the investigation that drove the pivot.
