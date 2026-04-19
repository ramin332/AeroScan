# SPEC: AeroScan

Project-level specification. Complements `CLAUDE.md` (architecture narrative) and
`README.md` (user-facing description). Update this file when any of the six core
areas changes.

## Objective

AeroScan is a NEN-2767 building inspection flight planner. It turns a building
description (parametric preset, uploaded footprint, uploaded mesh, or imported
DJI Smart3D KMZ point cloud) into a DJI WPML-compliant KMZ waypoint mission
for the DJI Matrice 4E.

**Primary user:** an inspection operator or mission planner who needs
NEN-2767-grade facade coverage without hand-editing flight paths.

**Success looks like:** given a building, the user configures mission parameters
in the UI, generates a mission, reviews facades/waypoints/path in 3D + on a
satellite map, fixes any validation errors, and downloads a KMZ that DJI Pilot 2
accepts and flies without modification.

**Secondary objective — dev platform principle.** Every parameter, option, and
behavior that can be user-facing must be exposed and controllable from the
frontend. Only deployed backend logic lives purely in backend code. If a knob
exists, it belongs in the UI.

## Tech Stack

**Backend**
- Python `>=3.12` (enforced by `pyproject.toml`; venv created by `run.sh`)
- FastAPI + uvicorn (port 8111)
- SQLAlchemy 2 + SQLite (`aeroscan.db` in repo root)
- numpy, scipy, pyproj, networkx, rtree, trimesh, `djikmz`
- Optional `mesh` extra: open3d, pyrender, fast-simplification, pymeshlab, cgal

**Frontend**
- React 19 + TypeScript 5.9 + Vite 8 (port 3847)
- Zustand 5 (state), react-three/fiber + drei + three (3D),
  react-leaflet + leaflet-draw (map)
- pnpm for installs (lockfile present); `npx vite` for dev

## Commands

```bash
# Full dev stack (backend :8111 + frontend :3847, recreates venv if broken)
./run.sh

# Backend only
python -m flight_planner                   # uvicorn with reload, watcher scoped to src/

# Frontend only
cd frontend && pnpm install                 # install
cd frontend && npx vite --port 3847         # dev server (matches run.sh)
cd frontend && pnpm run build               # tsc -b && vite build
cd frontend && pnpm run lint                # eslint .

# Install (Python)
pip install -e ".[dev,server]"              # core + tests + server
pip install -e ".[dev,server,mesh]"         # + mesh import / reconstruction

# Tests
pytest                                      # all (testpaths=tests, pythonpath=src)
pytest tests/test_geometry.py -v            # single file
pytest tests/test_geometry.py::test_name -v # single test
```

## Project Structure

```
src/flight_planner/         Python package — backend logic
  __main__.py               uvicorn entrypoint; watcher excludes heavy artifacts
  models.py                 Dataclasses: Building, Facade, Waypoint, MissionConfig, CameraSpec
  geometry.py               Facade build + waypoint grid + path pipeline
  camera.py                 GSD / footprint / grid spacing
  building_import.py        GeoJSON footprints + mesh (OBJ/PLY/STL/GLB/GLTF) import
  building_presets.py       Parametric preset buildings
  kmz_builder.py            KMZ export via djikmz
  kmz_import.py             DJI Smart3D KMZ → point cloud → facades
  kmz_cache.py              KMZ byte cache
  optimize.py               Path optimization
  validate.py               Mission constraint checks (ValidationIssue)
  visualize.py              Serialize facades/waypoints to Three.js + Leaflet JSON
  reconstruct.py            Simulated photogrammetric reconstruction
  gimbal_rewrite.py         Gimbal angle post-processing
  _profiling.py             Phase timing instrumentation
  server/
    __init__.py             FastAPI app factory
    api.py                  /api routes (mission gen, buildings, versions, sim, KMZ)
    database.py             SQLAlchemy models (BuildingRecord, SimulationRecord)
    state.py                In-memory MissionVersion store (max 100)

frontend/src/               React + TypeScript
  App.tsx, main.tsx, app.css
  store.ts                  Zustand store — all API calls live here
  api/
    client.ts               Typed fetch wrapper
    types.ts                TS mirrors of backend dataclasses
  components/
    Viewer3D.tsx            Three.js canvas (Selection/Plan/Flight modes)
    MapView.tsx             Leaflet satellite + drawable exclusion zones
    Sidebar.tsx             All mission/building config UI
    Stats.tsx, VersionList.tsx, DroneInfo.tsx, DroneAnimation.tsx
    PerfPanel.tsx, PhaseTimingsHud.tsx

tests/                      pytest (pythonpath=src)
  test_geometry.py, test_camera.py, test_kmz_builder.py,
  test_kmz_import.py, test_gimbal_rewrite.py,
  test_server.py, test_integration.py

examples/                   Runnable scripts: simple_box.py, l_shaped.py, pitched_roof.py
scripts/                    Dev utilities (bench_kmz.py)
kmz/                        Sample DJI KMZ inputs (gitignored)
output/                     Generated artifacts (gitignored)
sim_output/                 Simulation/reconstruction artifacts (gitignored)
aeroscan.db                 SQLite persistence (gitignored)

CLAUDE.md                   Architecture narrative, domain concepts, pipeline details
SPEC.md                     This file
README.md                   User-facing overview
pyproject.toml              Package config + pytest config
run.sh                      Dev entrypoint
```

## Code Style

**Python** — dataclass-first models, type hints on public functions, small
focused modules. Constants (hardware limits, Matrice 4E specs) live in
`models.py`. No comments explaining *what* the code does; only *why* when the
reason isn't obvious from reading it.

```python
# src/flight_planner/models.py — shape of the codebase
@dataclass
class CameraSpec:
    sensor_width_mm: float
    sensor_height_mm: float
    focal_length_mm: float
    image_width_px: int
    image_height_px: int

    def gsd_mm(self, distance_m: float) -> float:
        """Ground sample distance at a given standoff."""
        return distance_m * self.sensor_width_mm / (self.focal_length_mm * self.image_width_px / 1000)
```

**TypeScript** — interfaces in `api/types.ts` mirror Python dataclasses
field-for-field. State lives in `store.ts` (Zustand); components are
presentational where possible. React 19, function components, hooks.

```ts
// frontend/src/api/types.ts — mirrors Python exactly
export interface Waypoint {
  lat: number;
  lon: number;
  alt_m: number;
  gimbal_pitch_deg: number;
  gimbal_yaw_deg: number;
  heading_deg: number;
  facade_id: string | null;
}
```

**Naming**
- Python: `snake_case` for functions/vars, `PascalCase` for dataclasses,
  module names lowercase.
- TS: `camelCase` for vars/functions, `PascalCase` for components and types,
  component files `PascalCase.tsx`.

**Units** — always suffix numeric fields/args with units: `_m`, `_mm`, `_deg`,
`_px`, `_m2`. Coordinate systems: ENU for local geometry, WGS84 for export.

## Testing Strategy

Testing is not a priority gate right now. The bar is: **the math has to be
correct, and we lean on established packages instead of reinventing them.**
Tests exist where they already exist and can be added when they genuinely help,
but "ship with a test" is not a rule here.

- **Framework:** pytest, configured in `pyproject.toml`
  (`testpaths=["tests"]`, `pythonpath=["src"]`).
- **Location:** `test_<module>.py` per backend module under `tests/`;
  `test_integration.py` covers the end-to-end pipeline;
  `test_server.py` covers FastAPI routes via `httpx`.
- **Fixtures** live in `tests/conftest.py`.
- **Frontend:** no test framework. Type checking (`tsc -b` via `pnpm run build`)
  and lint (`pnpm run lint`) are the current gates.
- **What matters instead:**
  - Geometry, camera, gimbal, and containment math must be provably correct —
    verify against first principles or a reference implementation before
    shipping.
  - Use a trusted package rather than hand-rolling numerical or geometric
    primitives. numpy, scipy, trimesh, open3d, CGAL, networkx, pyproj are
    already in the stack — reach for them before writing your own.

## Boundaries

### Always do
- Expose every user-configurable parameter in the frontend UI — no backend-only
  knobs for things a user would want to change. This is a dev platform;
  presuming settings/defaults on the user's behalf is not ok.
- Prefer an established package (numpy, scipy, trimesh, open3d, CGAL, networkx,
  pyproj, djikmz, three.js, leaflet, …) over a hand-rolled equivalent.
- Verify the math. Geometry/camera/gimbal/containment code must be correct
  against first principles or a trusted reference — not "looks right."
- Keep `frontend/src/api/types.ts` in sync with Python dataclasses when
  backend models change.
- Unit-suffix numeric fields/args (`_m`, `_mm`, `_deg`, …).
- Run `pytest` before committing backend changes; run `pnpm run build` (tsc +
  vite) before committing frontend changes.
- Filter interior mesh faces via visibility ray casting (see CLAUDE.md —
  "Facade Extraction & Waypoint Safety") — do not bypass the safety layers.
- Use Open3D ray-parity for mesh containment, not `mesh.contains()` or
  convex-hull containment.
- Treat `CLAUDE.md` as authoritative for architectural detail; update it when
  the pipeline changes.

### Ask first
- Adding a Python dependency, especially one not already in an existing extra.
  `mesh` extras are heavy (open3d, cgal, pymeshlab) — justify before expanding.
- Changing the SQLite schema (`BuildingRecord`, `SimulationRecord`) or
  introducing a migration story (there isn't one today).
- Changing the fixed ports (`8111` backend, `3847` frontend).
- Swapping or upgrading core libraries (three.js, leaflet, djikmz, trimesh,
  open3d, FastAPI).
- Changing the `alpha_wrap` / RANSAC / region_growing tuning in `kmz_import.py`
  — these knobs are interlocked (see CLAUDE.md table) and tuned for NEN-2767
  small-feature bias.
- Removing or relaxing any facade/waypoint safety layer (normal check, LOS,
  containment, path collision).

### Never do
- Commit `aeroscan.db`, `.venv/`, `node_modules/`, `output/`, `sim_output/`,
  or `kmz/*` (all gitignored — keep it that way).
- Commit secrets, API keys, or DJI account credentials.
- Introduce a backend-only setting that a user might want to change without
  also exposing it in the Sidebar UI.
- Use Open3D's `OffscreenRenderer` for photo simulation — it requires EGL and
  is broken on macOS. Use pyrender in a subprocess (see CLAUDE.md).
- Skip `trimesh.repair.fix_normals()` on mesh load.
- Delete failing tests to make the suite green. Fix them, or ask.
- Reinvent geometric, numerical, or coordinate-system primitives that an
  established package already provides correctly.

## Success Criteria

AeroScan is "working" when all of the following hold on `main`:
- `pytest` passes cleanly with `[dev,server,mesh]` installed.
- `cd frontend && pnpm run build` passes (TypeScript + Vite).
- `./run.sh` starts both servers and the UI loads at `http://localhost:3847`.
- A preset building → mission generation → KMZ download round-trip succeeds.
- An uploaded mesh → facade extraction → mission → KMZ round-trip succeeds.
- A DJI Smart3D KMZ import → facade extraction → mission round-trip succeeds.
- Generated KMZ imports into DJI Pilot 2 without errors.
