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

**Server** (`server/`):
- **`api.py`** — FastAPI router under `/api`. Key endpoints: `POST /generate` (mission generation), building CRUD, version history, KMZ download.
- **`database.py`** — SQLAlchemy with SQLite (`aeroscan.db`). Single table: `BuildingRecord`.
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
5. Frontend renders 3D view (Selection/Plan/Flight modes) + map; user can download KMZ via `/api/versions/{id}/kmz`

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
5. **Convex hull waypoint filter** — final safety net removes any waypoint inside the mesh's convex hull
6. **LOS (line-of-sight) check** — per-waypoint ray cast to verify clear view to facade surface

### Mesh containment pitfall
`mesh.contains()` requires a watertight mesh to work. Photogrammetry meshes are never watertight. Use `mesh.convex_hull.contains()` for reliable inside/outside tests, or use ray-based visibility tests which work on any mesh.
