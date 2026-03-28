"""Tests for the debug server API."""

import io
import zipfile

import pytest
from fastapi.testclient import TestClient

from flight_planner.server import app

client = TestClient(app)


class TestPresets:
    def test_get_presets(self):
        r = client.get("/api/presets")
        assert r.status_code == 200
        presets = r.json()["presets"]
        assert "simple_box" in presets
        assert "pitched_roof_house" in presets
        assert "l_shaped_block" in presets
        assert "large_apartment_block" in presets

    def test_preset_has_dimensions(self):
        r = client.get("/api/presets")
        box = r.json()["presets"]["simple_box"]
        assert box["width"] == 20.0
        assert box["height"] == 8.0


class TestGenerate:
    def test_generate_default(self):
        r = client.post("/api/generate", json={})
        assert r.status_code == 200
        data = r.json()
        assert data["summary"]["waypoint_count"] > 0
        assert "threejs" in data["viewer_data"]
        assert "leaflet" in data["viewer_data"]
        assert "version_id" in data

    def test_generate_with_preset(self):
        r = client.post("/api/generate", json={"preset": "pitched_roof_house"})
        assert r.status_code == 200
        assert r.json()["summary"]["facade_count"] == 6  # 4 walls + 2 roof

    def test_generate_custom_building(self):
        r = client.post("/api/generate", json={
            "building": {"width": 40, "depth": 15, "height": 12},
            "mission": {"target_gsd_mm_per_px": 3.0},
        })
        assert r.status_code == 200
        assert r.json()["summary"]["waypoint_count"] > 0

    def test_generate_pitched_roof(self):
        r = client.post("/api/generate", json={
            "building": {"roof_type": "pitched", "roof_pitch_deg": 30},
        })
        assert r.status_code == 200
        assert r.json()["summary"]["facade_count"] == 6

    def test_generate_returns_config_snapshot(self):
        r = client.post("/api/generate", json={
            "mission": {"target_gsd_mm_per_px": 5.0},
        })
        data = r.json()
        assert data["config_snapshot"]["mission"]["target_gsd_mm_per_px"] == 5.0

    def test_threejs_data_has_facades_and_waypoints(self):
        r = client.post("/api/generate", json={})
        tj = r.json()["viewer_data"]["threejs"]
        assert len(tj["facades"]) > 0
        assert len(tj["waypoints"]) > 0
        assert "vertices" in tj["facades"][0]
        assert "x" in tj["waypoints"][0]

    def test_leaflet_data_has_coords(self):
        r = client.post("/api/generate", json={})
        lf = r.json()["viewer_data"]["leaflet"]
        assert "facadeGroups" in lf
        assert "buildingPoly" in lf
        assert "center" in lf

    def test_different_gsd_changes_waypoint_count(self):
        r1 = client.post("/api/generate", json={"mission": {"target_gsd_mm_per_px": 2.0}})
        r2 = client.post("/api/generate", json={"mission": {"target_gsd_mm_per_px": 5.0}})
        assert r1.json()["summary"]["waypoint_count"] > r2.json()["summary"]["waypoint_count"]


class TestVersions:
    def test_version_created_on_generate(self):
        r = client.post("/api/generate", json={})
        vid = r.json()["version_id"]
        versions = client.get("/api/versions").json()["versions"]
        ids = [v["version_id"] for v in versions]
        assert vid in ids

    def test_get_version(self):
        r = client.post("/api/generate", json={})
        vid = r.json()["version_id"]
        r2 = client.get(f"/api/versions/{vid}")
        assert r2.status_code == 200
        assert r2.json()["version_id"] == vid
        assert "viewer_data" in r2.json()

    def test_get_nonexistent_version(self):
        r = client.get("/api/versions/v_nonexistent")
        assert r.status_code == 404

    def test_delete_version(self):
        r = client.post("/api/generate", json={})
        vid = r.json()["version_id"]
        r2 = client.delete(f"/api/versions/{vid}")
        assert r2.status_code == 200
        r3 = client.get(f"/api/versions/{vid}")
        assert r3.status_code == 404

    def test_kmz_download(self):
        r = client.post("/api/generate", json={})
        vid = r.json()["version_id"]
        r2 = client.get(f"/api/versions/{vid}/kmz")
        assert r2.status_code == 200
        assert r2.headers["content-type"] == "application/octet-stream"
        # Verify it's a valid KMZ (ZIP)
        assert zipfile.is_zipfile(io.BytesIO(r2.content))
        with zipfile.ZipFile(io.BytesIO(r2.content)) as zf:
            assert "wpmz/template.kml" in zf.namelist()


class TestDrone:
    def test_get_drone(self):
        r = client.get("/api/drone")
        assert r.status_code == 200
        data = r.json()
        assert data["name"] == "DJI Matrice 4E"
        assert "wide" in data["cameras"]
        assert data["gimbal"]["tilt_min_deg"] == -90
        assert data["flight"]["max_waypoints"] == 65535

    def test_drone_cameras_have_specs(self):
        r = client.get("/api/drone")
        cam = r.json()["cameras"]["wide"]
        assert cam["focal_length_mm"] == 24
        assert cam["image_width_px"] == 5280


class TestFrontend:
    def test_index_returns_html(self):
        r = client.get("/")
        assert r.status_code == 200
        assert "text/html" in r.headers["content-type"]
        assert "AeroScan" in r.text
