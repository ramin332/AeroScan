# AeroScan Flight Planner

NEN-2767 building inspection flight planner that generates DJI WPML-compliant KMZ waypoint missions for the **DJI Matrice 4E**. Full-stack application with a Python/FastAPI backend and React/TypeScript/Three.js frontend.

Plan systematic facade inspections, visualize waypoint grids in 3D and on a satellite map, simulate drone flights, and export missions ready for DJI Pilot 2.

## Features

- **Preset & custom buildings** — Quick-start with built-in Dutch housing presets (simple box, pitched roof, L-shaped, apartment block) or import your own geometry via GeoJSON footprints or 3D mesh files (OBJ, PLY, STL, GLB, GLTF)
- **Automatic facade extraction** — Visibility ray casting to detect exterior faces, even on complex photogrammetry meshes with interior geometry
- **Boustrophedon waypoint grids** — Lawnmower sweep across each facade with configurable GSD, overlap, and camera parameters
- **Three camera lenses** — Wide (12 mm), medium tele (19 mm), telephoto (41.8 mm) with real Matrice 4E sensor specs
- **3D viewer** — Three.js scene with Selection, Plan, and Flight view modes; click facades to include/exclude them
- **Satellite map** — Leaflet view with waypoints, flight path, and drawing tools for exclusion/inclusion zones
- **Drone flight animation** — Watch the planned mission play out in 3D
- **Path optimization** — TSP-based route optimization to minimize flight time
- **Mission validation** — Checks against Matrice 4E hardware limits (max waypoints, altitude, gimbal range, flight time)
- **Version history** — Compare iterations of mission plans side by side
- **KMZ export** — DJI WPML-compliant files for DJI Pilot 2
- **Simulated reconstruction** — Synthetic photogrammetry rendering and depth-map fusion

## Quick Start

### Prerequisites

- Python 3.10+
- Node.js 18+ with pnpm

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/AeroScan.git
cd AeroScan

# Python backend
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev,server]"

# For 3D mesh import support (optional)
pip install -e ".[dev,server,mesh]"

# Frontend
cd frontend
pnpm install
cd ..
```

### Run

```bash
./run.sh
```

This starts both servers:

| Service  | URL                          |
|----------|------------------------------|
| Frontend | http://localhost:3847         |
| Backend  | http://localhost:8111         |
| API docs | http://localhost:8111/docs    |

Or run them separately:

```bash
# Backend only
python -m flight_planner

# Frontend only
cd frontend && npx vite --port 3847
```

## Usage

### Web Interface

1. Open http://localhost:3847
2. Choose a building preset or upload a GeoJSON/mesh file in the sidebar
3. Configure mission parameters (GSD, camera, overlap, speed)
4. Click **Generate** to compute the waypoint grid
5. Use the 3D viewer to inspect facades and waypoints (Selection/Plan/Flight modes)
6. Toggle facades on/off by clicking them in 3D or the sidebar list
7. Draw exclusion zones on the satellite map
8. Download the KMZ file for DJI Pilot 2

### CLI Examples

```bash
python examples/simple_box.py      # Rectangular building, flat roof
python examples/pitched_roof.py    # 30-degree pitched roof
python examples/l_shaped.py        # L-shaped building
```

Generated `.kmz` files are written to `output/`. Open them in Google Earth Pro or import into DJI Pilot 2.

## Architecture

```
src/flight_planner/
├── __main__.py          # Entry point (uvicorn on :8111)
├── models.py            # Dataclasses: Building, Facade, Waypoint, CameraSpec, MissionConfig
│                        #   Matrice 4E hardware constants (gimbal, cameras, flight limits)
├── geometry.py          # Computational core: facade generation, boustrophedon grids,
│                        #   path optimization, exclusion filtering, WGS84 conversion
├── camera.py            # GSD, footprint, and grid spacing calculations
├── kmz_builder.py       # DJI WPML KMZ generation via djikmz
├── building_presets.py  # Preset Dutch housing geometries
├── building_import.py   # GeoJSON + mesh parsing (trimesh), facade extraction with
│                        #   region growing and visibility ray casting
├── validate.py          # Mission constraint validation
├── optimize.py          # TSP path optimization
├── reconstruct.py       # Simulated photogrammetry reconstruction
├── visualize.py         # JSON serialization for Three.js and Leaflet viewers
└── server/
    ├── __init__.py      # FastAPI app with CORS
    ├── api.py           # REST endpoints: /api/generate, building CRUD, KMZ download
    ├── database.py      # SQLAlchemy + SQLite (aeroscan.db)
    └── state.py         # In-memory mission version store

frontend/src/
├── main.tsx             # React entry point
├── App.tsx              # Root layout
├── store.ts             # Zustand store: state, API calls, version history
├── api/
│   ├── client.ts        # Typed fetch wrapper
│   └── types.ts         # TypeScript interfaces mirroring backend models
└── components/
    ├── Viewer3D.tsx     # Three.js canvas (Selection/Plan/Flight modes)
    ├── Sidebar.tsx      # Building config, mission params, stats, validation
    ├── MapView.tsx      # Leaflet satellite map with waypoints and drawing tools
    ├── DroneAnimation.tsx  # Animated drone flight
    ├── DroneInfo.tsx    # Drone specs display
    ├── Stats.tsx        # Mission statistics
    ├── VersionList.tsx  # Version history panel
    └── PerfPanel.tsx    # Performance benchmarks

tests/
├── test_camera.py       # GSD and footprint calculations
├── test_geometry.py     # Facade generation, waypoint grids
├── test_kmz_builder.py  # KMZ output validation
├── test_integration.py  # End-to-end pipeline
└── test_server.py       # API endpoint tests
```

### Core Pipeline

```
Building → Facade Extraction → Waypoint Grid → Path Optimization → Validation → KMZ Export
```

1. **Building** is defined by preset, GeoJSON footprint, or 3D mesh
2. **Facades** are extracted with outward normals; interior faces filtered via visibility ray casting
3. **Waypoint grids** are generated per facade using a boustrophedon (lawnmower) pattern in a local (u, v) coordinate frame
4. **Path optimization** orders waypoints to minimize total flight distance (TSP)
5. **Filtering** removes waypoints inside exclusion zones or inside the building's convex hull
6. **Validation** checks against hardware limits (altitude, gimbal range, max waypoints, flight time)
7. **KMZ export** converts ENU coordinates to WGS84 and builds a DJI WPML-compliant file

### Coordinate Systems

| System   | Usage                                | Convention            |
|----------|--------------------------------------|-----------------------|
| ENU      | All geometry calculations            | East, North, Up       |
| WGS84    | GPS waypoints, KMZ export            | Latitude, longitude, altitude |
| Three.js | 3D viewer rendering                  | x=right, y=up, z=toward camera |

### Safety Layers

The pipeline has multiple safeguards to prevent waypoints from being placed inside or behind buildings:

1. `trimesh.repair.fix_normals()` ensures consistent face winding on mesh load
2. Centroid-based normal orientation provides initial outward guess
3. Visibility ray test verifies each facade is reachable from outside the bounding box
4. Per-facade normal check before waypoint generation
5. Convex hull filter removes any waypoint inside the mesh volume
6. Line-of-sight check verifies clear view from each waypoint to its facade

## API

The backend exposes a REST API documented at http://localhost:8111/docs (Swagger UI).

Key endpoints:

| Method | Path                        | Description                     |
|--------|-----------------------------|---------------------------------|
| POST   | `/api/generate`             | Generate mission waypoints      |
| GET    | `/api/versions`             | List mission versions           |
| GET    | `/api/versions/{id}/kmz`    | Download KMZ for a version      |
| POST   | `/api/buildings`            | Create building from upload     |
| GET    | `/api/buildings`            | List saved buildings            |
| POST   | `/api/simulate-reconstruct` | Start photogrammetry simulation |

## Testing

```bash
# All tests
pytest

# Single file
pytest tests/test_geometry.py -v

# Single test
pytest tests/test_geometry.py::test_name -v
```

## Key Domain Concepts

- **NEN-2767** — Dutch standard for condition assessment of building elements, requiring systematic visual inspection of all exterior facades
- **GSD (Ground Sample Distance)** — Target photo resolution in mm/pixel; drives the required camera-to-facade distance
- **Boustrophedon pattern** — Back-and-forth lawnmower sweep across a facade surface to ensure full coverage
- **Facade normal** — Outward-facing direction vector used to compute camera standoff position and gimbal angles
- **NL-SfB codes** — Dutch building component classification (e.g., "21.1" for external walls)
- **DJI WPML** — DJI's Waypoint Mission Language, the XML-based format inside KMZ files consumed by DJI Pilot 2

## Hardware Target

**DJI Matrice 4E** specs encoded in `models.py`:

| Parameter              | Value           |
|------------------------|-----------------|
| Cameras                | Wide (12 mm), Medium Tele (19 mm), Telephoto (41.8 mm) |
| Gimbal tilt range      | -90 to +35 deg |
| Gimbal pan range       | -60 to +60 deg |
| Max speed              | 21 m/s          |
| Inspection speed       | 2 m/s           |
| Min altitude           | 2 m             |
| Max altitude           | 6,000 m         |
| Max waypoints/mission  | 65,535          |
| Flight time (manifold) | 32 min          |

## Tech Stack

**Backend**: Python 3.10+, FastAPI, SQLAlchemy, NumPy, pyproj, trimesh, djikmz

**Frontend**: React 19, TypeScript, Vite, Three.js (react-three/fiber), Leaflet (react-leaflet), Zustand

## License

Proprietary. All rights reserved.
