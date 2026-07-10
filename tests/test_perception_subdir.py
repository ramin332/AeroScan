"""The Smart3D perception mesh is not always under `dji_perception/1`.

Observed on the real /blackbox ring buffer, 2026-07-10:

    flight0065  10 mesh chunks under dji_perception/1
    flight0070  10 mesh chunks under dji_perception/1
    flight0066   5 mesh chunks under dji_perception/2
    flight0072  13 mesh chunks under dji_perception/2   <- today's fresh scan

`from_manifold` hardcoded `dji_perception/1`, so flight0072's fresh mesh was
invisible and the resolver silently fell back to flight0070's 15-hour-old mesh
under subdir 1. Both 2026-07-10 afternoon missions were therefore planned
against the previous evening's geometry while a fresh scan of the actual target
sat unused on the aircraft. That also explains why the mesh staleness gate
appeared not to fire: it never saw the newer mesh.

Resolution rule: search every `dji_perception/*/` subdirectory, and when more
than one holds chunks, take the one whose newest chunk is newest. Silently
picking the lowest-numbered directory is how this bug happened.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from flight_planner.manifold import resolve_perception_dir


def _write_ply(path: Path, n: int = 8) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.zeros((n, 3), dtype="<f4")
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    path.write_bytes(header.encode() + pts.tobytes())


def test_finds_mesh_under_subdir_one(tmp_path):
    flight = tmp_path / "flight0070"
    _write_ply(flight / "dji_perception" / "1" / "mesh_binary_0.ply")
    assert resolve_perception_dir(flight).name == "1"


def test_finds_mesh_under_subdir_two(tmp_path):
    """flight0072's real layout. This is the case that was silently broken."""
    flight = tmp_path / "flight0072"
    _write_ply(flight / "dji_perception" / "2" / "mesh_binary_0.ply")
    (flight / "dji_perception" / "1").mkdir(parents=True, exist_ok=True)  # exists, empty
    assert resolve_perception_dir(flight).name == "2"


def test_prefers_the_newest_mesh_when_several_subdirs_have_one(tmp_path):
    import os
    import time

    flight = tmp_path / "flight0099"
    old = flight / "dji_perception" / "1" / "mesh_binary_0.ply"
    new = flight / "dji_perception" / "3" / "mesh_binary_0.ply"
    _write_ply(old)
    _write_ply(new)
    past = time.time() - 3600
    os.utime(old, (past, past))
    assert resolve_perception_dir(flight).name == "3", "picked a stale subdir"


def test_empty_subdirs_do_not_count(tmp_path):
    flight = tmp_path / "flight0071"
    (flight / "dji_perception" / "1").mkdir(parents=True)
    (flight / "dji_perception" / "2").mkdir(parents=True)
    with pytest.raises(FileNotFoundError, match="No mesh_binary"):
        resolve_perception_dir(flight)


def test_missing_perception_dir_raises(tmp_path):
    flight = tmp_path / "flight0064"
    flight.mkdir()
    with pytest.raises(FileNotFoundError):
        resolve_perception_dir(flight)
