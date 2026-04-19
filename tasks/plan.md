# Implementation Plan: DJI Mapping bbox + Frustum parity

## Overview

Align the AeroScan KMZ import viewer with the DJI Smart 3D Capture quality report shown on the RC Plus, on two axes:

1. **Bounding box.** Replace the custom XY-derived bbox (`server/api.py` lines 973–988; `Viewer3D.tsx` lines 886–914) with the oriented bounding box DJI ships inside the KMZ — the `boundingVolume.box` at `wpmz/res/ply/<name>/points/tileset.json` (DJI/OGC 3D Tiles format: `[cx, cy, cz, hx, hy, hz]` written as 12 floats = center + 3 half-axis vectors).
2. **Frustum.** The per-waypoint camera frustum in the 3D viewer uses a hardcoded `fovHDeg = 84, fovVDeg = 56` fallback (`Viewer3D.tsx:224-225`) and is further suppressed on the imported-KMZ path because `App.tsx:185` passes `cameraFov={null}`. DJI's quality report draws frustums with the real capture lens's intrinsics — wider/narrower than what we show. Extract the real intrinsics (payload + focal length from WPML) and plumb them through for imported KMZs.

## Resolved decisions (2026-04-19)

1. **Target = 3D Tiles OBB in `tileset.json`.** `boundingVolume.box` at `wpmz/res/ply/<name>/points/tileset.json` — 12 floats = center + 3 half-axis vectors in the tile's local frame, with a 4×4 `root.transform` mapping to ECEF. User confirmed with light uncertainty; Task 2's cloud-containment check is the backstop.
2. **Golden reference = `MijandeExtra.kmz`.** Has the same KMZ layout as `Mijande.kmz` (template.kml + waylines.wpml + tileset.json + cloud.ply). All acceptance screenshots in Phase 4 use it as primary.
3. **Derived bbox is removed, not kept as fallback.** The `sceneCentroid/sceneRadius` mesh-points math (`Viewer3D.tsx:886-914`) and the XY-bbox loop in `server/api.py:973-988` go away. KMZs without a tileset (the `autoExplore/*.kmz` samples) must still produce something sensible — we degrade to computing an AABB from the point-cloud extent *only* when `mappingBox` is truly absent, and tag it so the UI can surface the difference. This is a degraded mode, not a co-equal path.
4. **RC Plus reference screenshots captured.** `IMG_8718` (top-down) and `IMG_8719` (oblique) saved in `tasks/artifacts/rc-plus/` (HEIC + JPG). Both show `MijandeExtra.kmz`: 1333 photos, 100% progress, Mechanical Shutter enabled, RTK Fix Single. **Observations from the reference**:
   - The DJI UI has **two independent layer toggles** on the right rail: one for the "Mapping" bounding-box **block volume** and one for the camera **frustums**. In these two screenshots the block toggle is **off** and the frustum toggle is **on** — that's why the bbox isn't visible here. A bbox-on reference screenshot is still needed to match its visual style (color, edge thickness, opacity).
   - When toggled on, DJI renders the Mapping OBB as an opaque/semi-transparent rectangular block, not a thin wireframe. We should match that when our toggle is on.
   - Frustums: 4-sided pyramids, apex-at-camera, much *shorter* than our `frustumLen = 2.0` (visually ~0.3–0.5m). All 1333 drawn simultaneously (no sparse sampling).
   - Frustum color: white, slightly transparent. Point cloud color: purple.
   - Subject-facing: all frustums fan outward from orbit positions toward the building — shape/orientation matches our existing math; only scale and density differ.

## Assumptions still in play

1. **"Frustum in the quality report" = per-pose photo cone** drawn from the real capture camera's intrinsics. The M4E wide lens is ~84° diagonal; medium tele and telephoto are much narrower. `waylines.wpml` advertises the payload (`<wpml:payloadEnumValue>88</wpml:payloadEnumValue>` = M4E) but doesn't say which of the 3 lenses is active during mapping. Smart3D mapping missions always use the wide lens, so we default to that — **→ confirm against the RC Plus screenshot** once it's available.
2. **Goal is visual parity, not pixel-perfect recreation.** The DJI report is a black-box reference; we match its *shape and scale* using the real intrinsics, not reverse-engineer its renderer.
3. **Dev-platform rule (CLAUDE.md) applies.** Frustum intrinsics (FOV + draw distance) must be exposed as sidebar controls, not buried constants.

## Architecture Decisions

- **Extract tileset OBB server-side, serialize to ENU for the viewer.** The frontend shouldn't handle the ECEF→local-ENU transform chain; do it once in Python using the existing `sfm_geo_desc.json` ref and the tileset's 4×4 `transform`. Output: `{center: [x,y,z], axes: [[3], [3], [3]]}` in ENU meters. This keeps the three.js side dumb.
- **DJI OBB is the only authoritative bbox.** When `tileset.json` is present, the OBB *replaces* the mesh-points-derived bbox used for scene framing and rendering. When absent, a degraded AABB from the point-cloud extent is emitted with a `bboxSource: "degraded_aabb"` tag so the UI (and telemetry) can flag it. No user-facing toggle — the server picks the best available source.
- **Camera intrinsics payload.** Extend the `CameraFOV` contract (already in `frontend/src/api/types.ts:159-160` and the `summary.camera` block) to cover KMZ imports too. For imported KMZs, backend resolves the intrinsics from the payload+lens identifiers; frontend drops the 84°/56° hardcode.
- **Frustum override in the dev panel.** Expose `fov_h_deg`, `fov_v_deg`, `distance_m` as editable numbers in Sidebar → "Camera Frustum" (collapsed by default). Lets the user A/B against the RC Plus screenshot without a code change.
- **No new endpoint.** Everything piggybacks on `/api/import-kmz` and the existing viewer payload (`prepare_threejs_data` in `visualize.py`).

## Dependency Graph

```
kmz_import.py
    ├── parse tileset.json               (Task 1)
    │       │
    │       └── ENU transform of OBB     (Task 2)
    │
    └── resolve capture intrinsics        (Task 5)

visualize.py / server/api.py
    ├── emit mappingBox + bbox_source    (Task 3)       ← depends on 1,2
    └── emit cameraFov for KMZ imports    (Task 6)       ← depends on 5

frontend/src/api/types.ts
    └── MappingBox + CameraFOV types     (Task 4, 7)

frontend/src/components/Viewer3D.tsx
    ├── render OBB + use for scene bounds (Task 4)     ← depends on 3
    └── drop hardcoded FOV, use real     (Task 7)      ← depends on 6

frontend/src/components/Sidebar.tsx
    ├── bbox source toggle                (Task 8)     ← depends on 4
    └── frustum override controls         (Task 8)     ← depends on 7

Checkpoint + visual comparison            (Task 9)     ← depends on 1–8
```

## Task List

### Phase 1: Mapping bbox extraction (vertical slice — end-to-end rendering of DJI OBB)

#### Task 1: Parse `tileset.json` from DJI KMZ

**Description:** Add a `_parse_tileset` function to `src/flight_planner/kmz_import.py` that locates `wpmz/res/ply/<name>/points/tileset.json` inside the archive, reads the JSON, and returns the root `boundingVolume.box` (12 floats) plus the 4×4 `transform` and the tileset's own local-ENU frame metadata. Extend `ImportedKmz` with a new optional field `mapping_bbox_raw: Optional[dict]` carrying `{box, transform, asset_up_axis}`. Absence of the file (autoExplore KMZs may lack it) is not an error — field stays `None`.

**Acceptance criteria:**
- [ ] `parse_kmz(Mijande.kmz).mapping_bbox_raw["box"]` returns 12 floats matching the values in `tileset.json`
- [ ] `parse_kmz(autoExplore/*.kmz).mapping_bbox_raw` is `None` without raising
- [ ] Nested KMZ fallback path (inner `.kmz`) also populates the field when the inner archive has a tileset

**Verification:**
- [ ] New unit test: `tests/test_kmz_import.py::test_parse_tileset_mijande`
- [ ] `pytest tests/test_kmz_import.py -v`
- [ ] Run `python -c "from flight_planner.kmz_import import parse_kmz; print(parse_kmz(open('kmz/Mijande.kmz','rb').read()).mapping_bbox_raw)"` and eyeball the output

**Dependencies:** None

**Files likely touched:**
- `src/flight_planner/kmz_import.py`
- `tests/test_kmz_import.py` (new)
- `kmz/Mijande.kmz` (read-only fixture)

**Estimated scope:** S (1–2 files)

---

#### Task 2: Transform the OBB into the ENU frame used by the viewer

**Description:** Add `mapping_bbox_to_enu(raw: dict, ref_lat, ref_lon, ref_alt) -> dict` in `kmz_import.py` that (a) assembles the 3D Tiles ECEF→local transform, (b) composes it with WGS84-ref→ENU (using the existing pyproj helpers already used elsewhere in the codebase), and (c) returns `{center: [x,y,z], axes: [[3],[3],[3]]}` where `axes[i]` is a half-axis vector in ENU meters. Axes are non-axis-aligned (oriented box) so keep them as vectors, do not decompose.

**Acceptance criteria:**
- [ ] For `Mijande.kmz`: the returned `center` is within ~1m of the point cloud centroid (ENU), and `|axes[i]|` roughly matches the cloud extent along each principal direction (cross-check with `sfm_geo_desc.json` + `summary.info.refMaxHeight/refMinHeight`)
- [ ] The 8 OBB corners, when plotted, enclose ≥99% of `cloud.ply` points (after ref-ENU conversion)
- [ ] Returns `None` when `raw is None`

**Verification:**
- [ ] Unit test: loads `Mijande.kmz`, computes ENU OBB, asserts cloud containment ≥99%
- [ ] `pytest tests/test_kmz_import.py::test_mapping_bbox_to_enu -v`

**Dependencies:** Task 1

**Files likely touched:**
- `src/flight_planner/kmz_import.py`
- `tests/test_kmz_import.py`

**Estimated scope:** S

---

#### Task 3: Plumb the ENU OBB into the viewer payload + retire the derived bbox

**Description:** In `src/flight_planner/server/api.py`'s `/api/import-kmz` handler (raw + reconstruction branches), call `mapping_bbox_to_enu` after parsing. Extend `prepare_threejs_data` in `visualize.py` to accept a `mapping_bbox: dict | None` kwarg and emit it as `result["mappingBox"]` with `result["bboxSource"] = "dji_mapping"`. When tileset is absent, fall back to a point-cloud AABB with `bboxSource = "degraded_aabb"`. Delete the XY bbox loop at `api.py:973-988` — it's no longer the source of truth.

**Acceptance criteria:**
- [ ] Importing `MijandeExtra.kmz` yields `viewer_data.threejs.mappingBox` (center + 3 axis vectors) with `bboxSource == "dji_mapping"`
- [ ] Importing `autoExplore/*.kmz` yields an AABB `mappingBox` with `bboxSource == "degraded_aabb"`
- [ ] The old XY bbox computation is gone (grep for `bbox_w = float(maxs[0] - mins[0])` returns nothing)
- [ ] `/api/generate` responses are unaffected (mappingBox only populated on import path)

**Verification:**
- [ ] `pytest tests/test_server.py -k "kmz_import" -v`
- [ ] Manual: `curl -F file=@kmz/Mijande.kmz localhost:8111/api/import-kmz | jq .viewer_data.threejs.mappingBox`

**Dependencies:** Tasks 1, 2

**Files likely touched:**
- `src/flight_planner/server/api.py`
- `src/flight_planner/visualize.py`
- `tests/test_server.py`

**Estimated scope:** S

---

#### Task 4: Render the OBB as a toggleable block volume + use it for scene bounds

**Description:** Match DJI's UX: the Mapping OBB is both (a) the authoritative scene-bounds volume AND (b) a visible block that the user can toggle on/off. Two pieces:
1. **Scene bounds.** Replace the `sceneCentroid/sceneRadius` `useMemo` (lines 886–914) so it reads directly from `data.mappingBox` (center + half-axis diagonal as radius). `CameraReframe`, `OrbitControls`, and any Three.js frustum-culling defaults use this.
2. **Visible block.** Add a `MappingBoxView` component that renders the OBB as a semi-transparent rectangular block (mesh, not just edges — matches DJI's rendering). Toggle state lives in `store.ts` as `showMappingBox` and is wired to a sidebar control in Task 8. Default: off (matches DJI's default).
For `bboxSource === "degraded_aabb"`, draw a thin wireframe with a distinct warning color when the toggle is on, and show a "DJI tileset missing" badge in the status line.

Types: `ThreeJSData.mappingBox: MappingBox` and `bboxSource: "dji_mapping" | "degraded_aabb"`.

**Acceptance criteria:**
- [ ] Importing `MijandeExtra.kmz`: camera framing matches the RC Plus top-down view's centering and zoom (see `IMG_8718.jpg`)
- [ ] On data change, camera reframes around the DJI OBB (not the old mesh-points bbox)
- [ ] Sidebar toggle (implemented in Task 8) shows/hides the block without reloading
- [ ] Visual style matches DJI when a bbox-on reference screenshot is provided
- [ ] For `autoExplore/*.kmz`, the fallback AABB uses a distinct warning color + badge
- [ ] The old mesh-points bbox code path is gone (grep for the old mins/maxs loop returns nothing)

**Verification:**
- [ ] `pnpm run build` (typecheck + vite build) passes
- [ ] `pnpm run lint` passes
- [ ] Manual: open http://localhost:3847, import `MijandeExtra.kmz`, confirm the default orbital view centers the building correctly (no dead space, no off-center)
- [ ] Manual: toggle the block on — block wraps the point cloud correctly
- [ ] Side-by-side comparison screenshot saved to `tasks/artifacts/bbox-compare.png` — AeroScan's framing and block vs the RC Plus reference (block-on version pending from user)

**Dependencies:** Task 3

**Files likely touched:**
- `frontend/src/components/Viewer3D.tsx`
- `frontend/src/api/types.ts`

**Estimated scope:** M (3 files, UI + typecheck loop)

---

### Checkpoint A: Bounding box parity

- [ ] `pytest` all green
- [ ] `pnpm run build` + `pnpm run lint` clean
- [ ] Visual: DJI OBB renders for `Mijande.kmz` / `Slochteren.kmz` / `Woonhuis.kmz`; degrades gracefully for `autoExplore/*.kmz`
- [ ] Screenshot comparison with the RC Plus Capture quality report shows the **same box** (position + orientation, ±small cloud-padding)
- [ ] **Review with human before starting Phase 2**

---

### Phase 2: Frustum parity (vertical slice — real camera intrinsics end-to-end)

#### Task 5: Resolve DJI Smart 3D capture intrinsics server-side

**Description:** Add `resolve_capture_intrinsics(parsed: ImportedKmz) -> dict` to `kmz_import.py`. Inspects `parsed.mission_config_raw.payloadEnumValue` (88 = M4E) and any per-waypoint focal-length hints in `waylines.wpml` (e.g. `<wpml:focusDistance>`, `<wpml:focalLength>` if present — scan the sample KMZs first). Falls back to the M4E **wide lens** defaults already encoded in `src/flight_planner/models.py` (the `CameraSpec` for the 12mm wide). Returns `{fov_h_deg, fov_v_deg, distance_m, focal_length_mm, lens_name}`. This mirrors the shape already used by `summary.camera` in `/api/generate`.

**Acceptance criteria:**
- [ ] For all sample KMZs, returns a dict with non-zero `fov_h_deg`/`fov_v_deg`
- [ ] Logs the resolution path (KMZ-stated vs payload-default vs hardcoded fallback) at INFO level
- [ ] Matches the `CameraSpec` values in `models.py` for the M4E wide lens when no KMZ-level override is present

**Verification:**
- [ ] Unit test: resolve intrinsics for each KMZ in `kmz/` and assert non-zero fields
- [ ] `pytest tests/test_kmz_import.py::test_resolve_capture_intrinsics -v`

**Dependencies:** None (parallel to Phase 1)

**Files likely touched:**
- `src/flight_planner/kmz_import.py`
- `tests/test_kmz_import.py`

**Estimated scope:** S

---

#### Task 6: Emit `summary.camera` for imported KMZs

**Description:** In the KMZ-import handler in `server/api.py`, call `resolve_capture_intrinsics` and attach its result to the response under `summary.camera` (same shape `/api/generate` already produces — see `api.py:2421`). Today the import path returns a zeroed-out stub (`api.py:1258-1259`, `api.py:1722`) which is why `App.tsx:185` passes `cameraFov={null}`. Remove that `null`.

**Acceptance criteria:**
- [ ] Importing `Mijande.kmz` returns `summary.camera.fov_h_deg > 0`, `fov_v_deg > 0`
- [ ] `App.tsx:185` can now pass `cameraFov={viewerData.summary.camera}` (drop the `null`)
- [ ] `/api/generate` camera payload unchanged

**Verification:**
- [ ] `pytest tests/test_server.py -k "kmz_import" -v`
- [ ] Manual: curl import endpoint, check `.summary.camera` has real numbers

**Dependencies:** Task 5

**Files likely touched:**
- `src/flight_planner/server/api.py`
- `frontend/src/App.tsx` (one-line change: drop the `null`)

**Estimated scope:** S

---

#### Task 7: Fix the frustum renderer to use real intrinsics + scale + density

**Description:** In `Viewer3D.tsx`, three changes driven by the RC Plus reference (`IMG_8718.jpg` / `IMG_8719.jpg`):
1. Remove the hardcoded `fovHDeg = 84, fovVDeg = 56` (lines 224–225) from `OriginalGimbalArrows`; route through the `cameraFov` prop like `CameraArrows` already does.
2. **Shrink `frustumLen` from 2.0m to ~0.4m** and remove the `halfW * 0.25 / halfH * 0.25` shrink factor (lines 226–227) — DJI's reference draws small pyramids at the true base size. Expose the value via the sidebar override (Task 8).
3. **Remove the sparse-sampling stride** (`frustumStride` at line 158) for KMZ imports — DJI draws all 1333 frustums. Keep stride only for `/api/generate` paths where waypoint counts can blow past 10k. Gate on a prop.

Also audit the `forward/right/up` basis math (Viewer3D.tsx:240–250, 186–190) against DJI's convention (heading 0 = N, CW; pitch 0 = horizon, -90 = straight down) and add a short comment block documenting it. If flipped, fix the sign.

**Acceptance criteria:**
- [ ] Imported `MijandeExtra.kmz`: frustum wireframes visually match `IMG_8719.jpg` — same pyramid size, same density (all 1333 visible), same apex-at-camera orientation
- [ ] No hardcoded 84/56 remains in the file
- [ ] Frustum orientation matches photo capture direction — top edge points away from ground for pitch > -90°
- [ ] `/api/generate` path still sparse-samples for waypoint sets > 500 (performance preserved)

**Verification:**
- [ ] `pnpm run build` + `pnpm run lint`
- [ ] Visual: A/B against RC Plus reference — capture at `tasks/artifacts/frustum-compare.png`
- [ ] Pick one waypoint in `MijandeExtra.kmz`, extract its heading+pitch from `waylines.wpml`, compute expected frustum corners by hand, compare with rendered positions (within 5 cm at `distance_m = 0.4`)
- [ ] Performance: rendering 1333 frustums stays at 60fps on the dev machine

**Dependencies:** Task 6

**Files likely touched:**
- `frontend/src/components/Viewer3D.tsx`
- `frontend/src/App.tsx` (drop `cameraFov={null}`)

**Estimated scope:** M

---

### Phase 3: UI controls (dev-platform surface)

#### Task 8: Sidebar controls — bbox visibility toggle + frustum overrides

**Description:** Add a collapsible "Scene Layers" section to `frontend/src/components/Sidebar.tsx` mirroring the DJI UI's right-rail layer toggles:
- Toggle: `Show Mapping bbox` (drives the `MappingBoxView` mesh from Task 4). Default: off.
- Toggle: `Show camera frustums` (drives `CameraArrows` / `OriginalGimbalArrows`). Default: on.
- Number inputs: `Frustum H FOV`, `Frustum V FOV`, `Frustum distance (m)` — prefilled from `cameraFov`, writeable.
- Reset button: restores server defaults.
- Read-only status line: `Bbox source: DJI Mapping | Degraded AABB (DJI tileset missing)` — informational.

Wire through `store.ts` as `sceneLayers: { showMappingBox, showFrustums, frustumFovH, frustumFovV, frustumDistance }`. Viewer3D consumes these when present, otherwise falls back to server payload. No backend change.

**Acceptance criteria:**
- [ ] Mapping-bbox toggle shows/hides the block without reloading — matches DJI UX
- [ ] Frustum toggle shows/hides all frustum wireframes
- [ ] Changing FOV/distance live-updates the frustum wireframe
- [ ] Reset restores server defaults
- [ ] Bbox source status reflects the backend value
- [ ] State survives view-mode switches (Selection/Plan/Flight) but resets on new KMZ upload

**Verification:**
- [ ] `pnpm run build` + `pnpm run lint`
- [ ] Manual: toggle each control, confirm viewer response

**Dependencies:** Tasks 4, 7

**Files likely touched:**
- `frontend/src/components/Sidebar.tsx`
- `frontend/src/store.ts`
- `frontend/src/components/Viewer3D.tsx` (read overrides)

**Estimated scope:** M

---

### Phase 4: Verification

#### Task 9: Visual regression vs RC Plus quality report (primary: MijandeExtra)

**Description:** Golden reference is `MijandeExtra.kmz`. Capture:
1. The RC Plus Capture quality report screenshot (user-supplied, saved in `tasks/artifacts/rc-plus/`) — blocked pending the `IMG_8719.HEIC` drop.
2. A matching screenshot of AeroScan's viewer after import.
3. A short note in `tasks/artifacts/comparison.md`: bbox match? frustum match? outstanding differences?

Secondary checks: repeat for `Mijande`, `Slochteren`, `Woonhuis`, `NewSmart3DExploreTask2` when RC Plus captures are available. This is not an automated test — it's the acceptance artifact for the whole initiative.

**Acceptance criteria:**
- [ ] `MijandeExtra.kmz` has paired RC Plus ↔ AeroScan screenshots with overlap/difference called out
- [ ] `comparison.md` identifies any remaining deltas with a proposed fix
- [ ] Outstanding deltas either fixed (add task) or documented as known limitation
- [ ] Secondary KMZs captured where RC Plus references are available

**Verification:**
- [ ] Human review of `tasks/artifacts/comparison.md`

**Dependencies:** Tasks 1–8

**Files likely touched:**
- `tasks/artifacts/*` (new)

**Estimated scope:** S (documentation)

---

### Checkpoint B: Final

- [ ] All tests green (`pytest` + `pnpm run build` + `pnpm run lint`)
- [ ] Visual comparison shows bbox and frustum match the RC Plus quality report for all 5 sample KMZs
- [ ] Sidebar controls work
- [ ] Ready for review

## Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Tileset's ECEF `transform` uses a non-standard convention (Z-up vs Y-up); wrong transform = OBB lands in the wrong place | High | Task 2 includes a cloud-containment assertion (≥99% of `cloud.ply` inside the OBB). If that fails, the transform is wrong — debug before Task 3. |
| User's "Mapping" actually refers to the `mappingObject` placemark polygon, not the tileset OBB | Medium | Surfaced as Assumption 1. Confirm before writing code. |
| DJI doesn't expose capture-time focal length in WPML; fallback to M4E-wide is wrong for tele missions | Medium | Task 5 logs the resolution path so mismatches are visible. Expose an override in Task 8. |
| Frustum orientation math (heading+pitch → forward) has a subtle sign bug that only shows up for tilted gimbals | Medium | Task 7 acceptance requires hand-computed waypoint match within 5 cm. |
| `NewSmart3DExploreTask2.kmz` / `autoExplore` subfolder may have a different tileset layout | Low | Task 1's `_find_members` already handles inner-KMZ recursion — extend the tileset search identically, test against the `autoExplore/` sample. |
| Breaking the `/api/generate` path while adding fields to the viewer payload | Low | All new fields are optional and only populated on the import path. Existing tests (`test_server.py::test_generate`) guard the non-import path. |

## Open Questions (remaining)

1. **Need a bbox-on reference screenshot.** `IMG_8718`/`IMG_8719` have the Mapping-block toggle off. A screenshot with it on would let Task 4 match DJI's visual style exactly (opacity, color, edge treatment). Not a blocker — we can ship our own style and refine later.
2. **Does DJI's `mapping_rotation` in `template.kml` affect the OBB?** Mijande's value is `0,0,0` — no rotation. Re-check on a building where the mission polygon is rotated before shipping. (Low risk; Task 2's cloud-containment assertion will catch a mis-applied rotation.)
3. **Photos in MijandeExtra that "look straight up" on the RC Plus** (user-reported indices 1234, 1251). Confirmed (2026-04-19) that no pitch field in the KMZ exceeds +20°, M4E gimbal max is +35°, and DJI's Matrice 4E cannot physically point straight up. Resolution requires reading the JPEG XMP (`drone-dji:GimbalPitchDegree`) — see Task 11 in `todo.md`. The frontend rendering math is verified correct against the WPML data.

## Resolved findings (2026-04-19)

- **Frustum orientation transform is correct.** Investigated user reports of "all frustums looking down." Verified end-to-end: WPML parser → `waypoints_to_enu` → `prepare_threejs_data` → API stamping → `WaypointData.smart_oblique_poses` → `OriginalGimbalArrows`. No clamping, no sign error. Perceived downward bias is real-data bias: 74/1098 rosette poses are at exactly −90° (nadir), and only 30/1098 are above 0°.
- **MijandeExtra mission max upward pitch is +20°.** Across all 248 rosette WPs in the file, the steepest single positive pose is +20° (at WPs 8, 370, 423). On a 4 m diagnostic frustum that's only ~1.4 m of vertical lift — visually modest, not a bug. Documented in CLAUDE.md ("Smart3D rosette pitch — what the KMZ encodes vs what the drone shoots").
- **`smartObliqueCycleMode=unlimited` requires shutter-interval simulation for visual parity with DJI.** Drawing 5 frustums per WP under-renders by ~5× vs the actual ~1333 photos. New Task 10 in `todo.md`.
