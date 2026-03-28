# AeroScan Flight Planner

NEN-2767 inspection flight planner — generates DJI WPML-compliant KMZ waypoint missions for automated building inspection with the Matrice 4E.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## Usage

```bash
python examples/simple_box.py      # rectangular building, flat roof
python examples/pitched_roof.py    # 30° pitched roof
python examples/l_shaped.py        # L-shaped building
```

Generated `.kmz` files are written to `output/`. Open them in Google Earth Pro or import into DJI Pilot 2.

## Tests

```bash
pytest tests/ -v
```

## Project structure

```
src/flight_planner/
├── models.py           # dataclasses: Building, Facade, Waypoint, CameraAction
├── camera.py           # GSD calculations, footprint, overlap-to-spacing
├── geometry.py         # facade segmentation, normal computation, waypoint grids
├── kmz_builder.py      # DJI WPML KMZ file generation
├── building_presets.py  # preset Dutch housing geometries
└── visualize.py        # matplotlib 3D visualization
```
