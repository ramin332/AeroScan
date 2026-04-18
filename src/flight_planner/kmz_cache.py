"""Disk cache for parsed KMZ artefacts — dev-only shortcut.

Keyed by ``sha256(kmz_bytes)``, so every unique KMZ gets its own directory
under ``output/kmz_cache/``. Artefacts are stored in formats we trust:

- ``meta.json`` — cache entry metadata (name, ENU origin, timestamps)
- ``waypoints.json`` — parsed waypoints (one dict per waypoint)
- ``mission_area.json`` — area polygon + mission_config_raw
- ``pointcloud.npz`` — numpy ``savez`` with ``positions``/``colors``/``max_points``
- ``mesh.ply`` — reconstructed mesh (facades mode only)
- ``facades.json`` — extracted facades (populated by Step 2)

All artefacts are plain JSON or raw numpy arrays, and :func:`numpy.load`
is always called with ``allow_pickle=False`` so a corrupted cache cannot
execute arbitrary code.

This is strictly a development accelerator — production imports should still
go through the full pipeline. The module is only referenced from
``scripts/bench_kmz.py`` and the ``/api/import-kmz`` warm path.
"""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

import numpy as np

_CACHE_DIR = Path(__file__).resolve().parents[2] / "output" / "kmz_cache"


def _dir_for(sha: str) -> Path:
    return _CACHE_DIR / sha


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def entry_dir(kmz_bytes: bytes) -> Path:
    """Directory where artefacts for this KMZ will be cached. Created lazily."""
    d = _dir_for(sha256_bytes(kmz_bytes))
    d.mkdir(parents=True, exist_ok=True)
    return d


# ---------------------------------------------------------------------------
# Meta / waypoints / mission area
# ---------------------------------------------------------------------------


def write_meta(entry: Path, payload: dict[str, Any]) -> None:
    payload = {**payload, "cached_at": time.time()}
    (entry / "meta.json").write_text(json.dumps(payload, indent=2))


def read_meta(entry: Path) -> dict[str, Any] | None:
    p = entry / "meta.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())


def _to_plain(obj: Any) -> Any:
    if is_dataclass(obj) and not isinstance(obj, type):
        return asdict(obj)
    return obj


def write_waypoints(entry: Path, waypoints: list[Any]) -> None:
    (entry / "waypoints.json").write_text(
        json.dumps([_to_plain(w) for w in waypoints], indent=None)
    )


def read_waypoints(entry: Path) -> list[dict] | None:
    p = entry / "waypoints.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())


def write_mission_area(
    entry: Path,
    polygon_wgs84: list[tuple[float, float, float]],
    mission_config: dict[str, Any],
) -> None:
    (entry / "mission_area.json").write_text(json.dumps({
        "polygon_wgs84": [list(p) for p in polygon_wgs84],
        "mission_config": mission_config,
    }))


def read_mission_area(entry: Path) -> dict[str, Any] | None:
    p = entry / "mission_area.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())


# ---------------------------------------------------------------------------
# Point cloud (numpy savez — raw arrays only)
# ---------------------------------------------------------------------------


def write_pointcloud(
    entry: Path,
    positions: np.ndarray,
    colors: np.ndarray,
    max_points: int,
) -> None:
    np.savez(
        entry / "pointcloud.npz",
        positions=np.asarray(positions, dtype=np.float32),
        colors=np.asarray(colors, dtype=np.float32),
        max_points=np.asarray([max_points], dtype=np.int64),
    )


def read_pointcloud(entry: Path) -> tuple[np.ndarray, np.ndarray, int] | None:
    p = entry / "pointcloud.npz"
    if not p.exists():
        return None
    with np.load(p, allow_pickle=False) as z:
        positions = np.asarray(z["positions"], dtype=np.float32)
        colors = np.asarray(z["colors"], dtype=np.float32)
        max_points = int(z["max_points"][0])
    return positions, colors, max_points


# ---------------------------------------------------------------------------
# Reconstructed mesh (PLY bytes — not proprietary)
# ---------------------------------------------------------------------------


def write_mesh_ply(entry: Path, ply_bytes: bytes) -> None:
    (entry / "mesh.ply").write_bytes(ply_bytes)


def read_mesh_ply(entry: Path) -> bytes | None:
    p = entry / "mesh.ply"
    if not p.exists():
        return None
    return p.read_bytes()


# ---------------------------------------------------------------------------
# Facade cache (populated by Step 2; read by future runs)
# ---------------------------------------------------------------------------


def write_facades(entry: Path, facades_json: list[dict]) -> None:
    (entry / "facades.json").write_text(json.dumps(facades_json))


def read_facades(entry: Path) -> list[dict] | None:
    p = entry / "facades.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())
