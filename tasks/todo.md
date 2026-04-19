# Todo — DJI Mapping bbox + Frustum parity

Source plan: [plan.md](./plan.md)
Golden reference: `MijandeExtra.kmz`

## Phase 1 — Mapping bbox (tileset OBB replaces derived bbox)

- [x] **Task 1** — Parse `tileset.json` from the KMZ; add `ImportedKmz.mapping_bbox_raw`. (S) — `src/flight_planner/kmz_import.py`
- [x] **Task 2** — `mapping_bbox_to_enu()` — compose 3D-Tiles ECEF→local with WGS84→ENU ref. (S) — `kmz_import.py`
- [x] **Task 3** — Plumb ENU OBB through `prepare_threejs_data`; delete derived XY bbox at `api.py:973-988`. (S) — `server/api.py`, `visualize.py`
- [x] **Task 4** — Render OBB as toggleable block volume + replace `sceneCentroid/sceneRadius` with mappingBox-driven version. (M) — `Viewer3D.tsx`, `api/types.ts`

### Checkpoint A

- [ ] `pytest` green, `pnpm run build` + `pnpm run lint` clean
- [ ] OBB renders for `MijandeExtra`/`Mijande`/`Slochteren`/`Woonhuis`; `autoExplore/*` imports with no bbox (already rejected by the cloud.ply precondition — no fallback needed)
- [x] Old derived-bbox code fully removed (grep guard: `bbox_w = float(maxs[0] - mins[0])` absent)
- [ ] RC Plus vs AeroScan screenshot agrees on box position+orientation (blocked on screenshot)
- [ ] **Human review before Phase 2**

## Phase 2 — Frustum parity

- [x] **Task 5** — `resolve_capture_intrinsics()` — payload → lens → FOV. (S) — `kmz_import.py`
- [x] **Task 6** — Emit `summary.camera` for imported KMZs; drop `cameraFov={null}`. (S) — `server/api.py`, `App.tsx`
- [x] **Task 7** — Drop hardcoded 84/56; use real FOV; audit orientation math. (M) — `Viewer3D.tsx`

## Phase 3 — Dev-platform controls

- [x] **Task 8** — Sidebar "Scene Layers": bbox visibility toggle + frustum visibility toggle + FOV/distance overrides + reset + bbox-source status. (M) — `Sidebar.tsx`, `store.ts`, `Viewer3D.tsx`

## Phase 4 — Verification

- [ ] **Task 9** — RC Plus ↔ AeroScan screenshot pair for `MijandeExtra.kmz` + `comparison.md`; secondary KMZs where references exist. (S) — `tasks/artifacts/`

### Checkpoint B

- [ ] All tests green
- [ ] Visual parity confirmed on `MijandeExtra.kmz`
- [ ] Sidebar overrides work end-to-end
- [ ] Ready for review

## Phase 5 — Frustum density (rosette → continuous shutter)

Current `OriginalGimbalArrows` (`Viewer3D.tsx`) draws **5 frustums per WP** (one per rosette pose). DJI's RC Plus quality report draws **one frustum per actual photo** — 1333 for MijandeExtra — sampled along the flight path at the shutter rate while the rosette cycles continuously (`smartObliqueCycleMode=unlimited`). Match that.

- [x] **Task 10** — Replace per-WP rosette draw with shutter-interval simulation along the flight path. (M) — `Viewer3D.tsx`
  - **Switched to distance-based sampling.** Time-based using `waypointSpeed`/`autoFlightSpeed` was wrong: WPML reports 0.7–1.0 m/s but the drone actually flies ~2 m/s, producing 5×-too-dense output. Use `DJI_SMART3D_INTER_SHOT_DIST_M = 0.65` instead (calibrated from MijandeExtra: 850 m / 1333 photos = 0.64 m/photo).
  - **Cycle counter walks continuously across all WPs and never resets.** Initial implementation reset on rosette template change, but in MijandeExtra DJI re-issues a slightly-tilted rosette every 2–3 WPs (248 overlapping groups, 69 unique templates with smoothly drifting pitches) — resetting on template change reproduced the "all 5 poses at every WP" artifact exactly. `cycleMode=unlimited` only updates *which direction* poses 0-4 point; the cycle counter keeps walking.

- [ ] **Task 10b** — Per CLAUDE.md dev-platform rule, expose `DJI_SMART3D_INTER_SHOT_DIST_M` as a sidebar slider (default 0.65 m, range 0.1–2.0). Lets users tune frustum density per mission without code changes. (S — `store.ts`, `Sidebar.tsx`, `Viewer3D.tsx`)

## Investigation backlog (data-side, non-blocking)

- [ ] **Task 11** — User reports specific photos (e.g. 1234, 1251 of 1333) on the RC Plus appear to point near-vertical, but no pitch field in `MijandeExtra.kmz` exceeds +20°. Resolution requires reading the JPEGs' XMP (`drone-dji:GimbalPitchDegree`) — see CLAUDE.md "Smart3D rosette pitch — what the KMZ encodes vs what the drone shoots". (S — wait for user to drop JPEGs into `tasks/artifacts/rc-plus/`)

## Nice-to-have (non-blocking)

- [ ] One more RC Plus screenshot with the Mapping-block toggle **on** — lets Task 4 match DJI's block style (color/opacity) exactly. Current screenshots have it toggled off.
