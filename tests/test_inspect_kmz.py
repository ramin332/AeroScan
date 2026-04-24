"""Tests for the inspect_kmz CLI tool."""

import shutil
from pathlib import Path

import pytest

from flight_planner.tools.inspect_kmz import (
    KmzEnums,
    inspect_kmz,
    patch_kmz_builder,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SLOCHTEREN_KMZ = REPO_ROOT / "kmz" / "Slochteren.kmz"


@pytest.mark.skipif(not SLOCHTEREN_KMZ.exists(), reason="reference KMZ not present")
class TestInspectRealKmz:
    def test_extracts_enums_from_dji_kmz(self):
        """Slochteren.kmz is a DJI-native Smart3D export — enums must parse."""
        enums = inspect_kmz(SLOCHTEREN_KMZ)
        assert enums.wpml_version == "1.0.6"
        assert enums.template_type == "mappingObject"
        assert enums.drone_enum == 99
        assert enums.payload_enum == 88
        assert enums.drone_sub_enum == 0
        assert enums.payload_sub_enum == 0


class TestPatchKmzBuilder:
    def test_patch_rewrites_constants(self, tmp_path: Path):
        """--patch rewrites the M4E_* constants in-place and returns the diff."""
        # Copy the real kmz_builder.py to a temp location so we don't mutate it.
        real = REPO_ROOT / "src" / "flight_planner" / "kmz_builder.py"
        fake = tmp_path / "kmz_builder.py"
        shutil.copy(real, fake)

        enums = KmzEnums(
            drone_enum=103,
            drone_sub_enum=2,
            payload_enum=91,
            payload_sub_enum=1,
            wpml_version="1.0.6",
            template_type="waypoint",
        )
        changes = patch_kmz_builder(enums, fake)

        # All four constants should have changed from the provisional values.
        changed_names = {name for name, _, _ in changes}
        assert changed_names == {
            "M4E_DRONE_ENUM",
            "M4E_DRONE_SUB_ENUM",
            "M4E_PAYLOAD_ENUM",
            "M4E_PAYLOAD_SUB_ENUM",
        }

        patched = fake.read_text()
        assert "M4E_DRONE_ENUM = 103" in patched
        assert "M4E_DRONE_SUB_ENUM = 2" in patched
        assert "M4E_PAYLOAD_ENUM = 91" in patched
        assert "M4E_PAYLOAD_SUB_ENUM = 1" in patched
        # Surrounding comments ("# PROVISIONAL — same as M3E") should be preserved.
        assert "PROVISIONAL" in patched

    def test_patch_is_idempotent(self, tmp_path: Path):
        """Running --patch twice with the same values is a no-op the second time."""
        real = REPO_ROOT / "src" / "flight_planner" / "kmz_builder.py"
        fake = tmp_path / "kmz_builder.py"
        shutil.copy(real, fake)

        enums = KmzEnums(
            drone_enum=103,
            drone_sub_enum=0,
            payload_enum=91,
            payload_sub_enum=0,
            wpml_version="1.0.6",
            template_type="waypoint",
        )
        first = patch_kmz_builder(enums, fake)
        second = patch_kmz_builder(enums, fake)
        assert first, "first patch should have changes"
        assert second == [], "second patch should be a no-op"


class TestInspectMalformed:
    def test_missing_template_raises(self, tmp_path: Path):
        """A ZIP without wpmz/template.kml is rejected cleanly."""
        import zipfile

        bad = tmp_path / "empty.kmz"
        with zipfile.ZipFile(bad, "w") as zf:
            zf.writestr("readme.txt", "not a kmz")
        with pytest.raises(ValueError, match="missing wpmz/template.kml"):
            inspect_kmz(bad)
