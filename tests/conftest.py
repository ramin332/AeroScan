"""Shared pytest fixtures for AeroScan tests."""

from __future__ import annotations

from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[1]
_KMZ_DIR = _REPO_ROOT / "kmz"

# Short names → filenames in kmz/
KMZ_SAMPLES = {
    "mijande": "Mijande.kmz",
    "mijande_extra": "MijandeExtra.kmz",
    "slochteren": "Slochteren.kmz",
    "smart3d": "NewSmart3DExploreTask2.kmz",
    "woonhuis": "Woonhuis.kmz",
    "auto_explore": "autoExplore/db9b50e2_d904_41a2_8012_e3ee144049dd.kmz",
}


def sample_kmz_bytes(name: str) -> bytes:
    """Return the raw bytes of a sample KMZ by short name.

    Tests that depend on a real KMZ should call this via the ``sample_kmz`` or
    ``sample_kmz_path`` fixtures, which auto-skip when the file is missing
    (``kmz/`` is gitignored and not every dev box has it).
    """
    if name not in KMZ_SAMPLES:
        raise KeyError(f"Unknown KMZ sample {name!r}; known: {sorted(KMZ_SAMPLES)}")
    path = _KMZ_DIR / KMZ_SAMPLES[name]
    if not path.exists():
        pytest.skip(f"KMZ fixture {path} not present")
    return path.read_bytes()


def sample_kmz_resolved_path(name: str) -> Path:
    if name not in KMZ_SAMPLES:
        raise KeyError(f"Unknown KMZ sample {name!r}; known: {sorted(KMZ_SAMPLES)}")
    path = _KMZ_DIR / KMZ_SAMPLES[name]
    if not path.exists():
        pytest.skip(f"KMZ fixture {path} not present")
    return path


@pytest.fixture
def sample_kmz():
    """Return a callable: sample_kmz(name) -> bytes."""
    return sample_kmz_bytes


@pytest.fixture
def sample_kmz_path():
    """Return a callable: sample_kmz_path(name) -> Path."""
    return sample_kmz_resolved_path
