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
4. **Per-facade normal check** — before generating waypoints, verify normal orientation via visibility ray
5. **Open3D ray-parity containment filter** — replaces convex hull. Uses `RaycastingScene.count_intersections()` (odd = inside) which correctly handles non-convex buildings (L-shaped, U-shaped, courtyards). Works on non-watertight meshes.
6. **LOS (line-of-sight) check** — per-waypoint ray cast to verify clear view to facade surface
7. **Path segment collision check** — for each consecutive waypoint pair, casts a ray along the flight path segment and checks for mesh intersections. Collisions are resolved by inserting altitude detour waypoints routed outward from the building. Unresolvable collisions are flagged as validation errors.

### Mesh containment
The old `mesh.convex_hull.contains()` approach failed for non-convex buildings (concave regions were incorrectly classified as "inside"). Open3D ray-parity (`count_intersections`, odd = inside) correctly handles all geometries including non-watertight photogrammetry meshes. `mesh.contains()` from trimesh still requires watertight meshes — avoid it.
