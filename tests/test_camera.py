"""Tests for camera calculations."""

import pytest

from flight_planner.camera import (
    compute_distance_for_gsd,
    compute_footprint,
    compute_grid_spacing,
    compute_gsd,
    get_camera,
)
from flight_planner.models import CAMERAS, CameraName


class TestGSD:
    def test_gsd_at_known_distance(self):
        """At 14.6m, wide camera should give ~2 mm/px GSD."""
        camera = get_camera(CameraName.WIDE)
        gsd = compute_gsd(camera, 14.6)
        assert abs(gsd - 2.0) < 0.1

    def test_gsd_increases_with_distance(self):
        camera = get_camera(CameraName.WIDE)
        gsd_near = compute_gsd(camera, 5.0)
        gsd_far = compute_gsd(camera, 20.0)
        assert gsd_far > gsd_near

    def test_telephoto_finer_gsd_at_same_distance(self):
        """Telephoto should give finer GSD than wide at the same distance."""
        wide = get_camera(CameraName.WIDE)
        tele = get_camera(CameraName.TELEPHOTO)
        gsd_wide = compute_gsd(wide, 10.0)
        gsd_tele = compute_gsd(tele, 10.0)
        assert gsd_tele < gsd_wide


class TestDistanceForGSD:
    def test_wide_2mm_gsd(self):
        """Wide camera at 2mm/px GSD should need ~14.6m distance."""
        camera = get_camera(CameraName.WIDE)
        distance = compute_distance_for_gsd(camera, 2.0)
        # From the report: d = (24 * 0.002 * 5280) / 17.3 ≈ 14.6m
        assert abs(distance - 14.6) < 0.5

    def test_roundtrip(self):
        """Computing distance then GSD should return the original target."""
        camera = get_camera(CameraName.WIDE)
        target_gsd = 2.0
        distance = compute_distance_for_gsd(camera, target_gsd)
        computed_gsd = compute_gsd(camera, distance)
        assert abs(computed_gsd - target_gsd) < 0.001

    def test_medium_tele_1mm_gsd(self):
        """Medium tele at 1mm/px GSD.

        d = (70 * 0.001 * 8064) / 9.6 = 58.8m
        """
        camera = get_camera(CameraName.MEDIUM_TELE)
        distance = compute_distance_for_gsd(camera, 1.0)
        assert abs(distance - 58.8) < 1.0


class TestFootprint:
    def test_wide_footprint_at_14_6m(self):
        """At 14.6m, wide camera footprint should be reasonable."""
        camera = get_camera(CameraName.WIDE)
        fp = compute_footprint(camera, 14.6)
        # footprint_width = 14.6 * 17.3 / 24 ≈ 10.5m
        assert abs(fp.width_m - 10.5) < 0.5
        # footprint_height = 14.6 * 13.0 / 24 ≈ 7.9m
        assert abs(fp.height_m - 7.9) < 0.5

    def test_footprint_positive(self):
        for cam_name in CameraName:
            camera = get_camera(cam_name)
            fp = compute_footprint(camera, 10.0)
            assert fp.width_m > 0
            assert fp.height_m > 0


class TestGridSpacing:
    def test_default_overlap(self):
        """With 80% front overlap and 70% side overlap."""
        camera = get_camera(CameraName.WIDE)
        fp = compute_footprint(camera, 14.6)
        h_step, v_step = compute_grid_spacing(fp, front_overlap=0.80, side_overlap=0.70)

        # h_step = 10.5 * 0.3 ≈ 3.15m
        assert abs(h_step - 3.15) < 0.3
        # v_step = 7.9 * 0.2 ≈ 1.58m
        assert abs(v_step - 1.58) < 0.2

    def test_no_overlap(self):
        """With 0% overlap, step equals footprint size."""
        camera = get_camera(CameraName.WIDE)
        fp = compute_footprint(camera, 10.0)
        h_step, v_step = compute_grid_spacing(fp, front_overlap=0.0, side_overlap=0.0)
        assert abs(h_step - fp.width_m) < 0.001
        assert abs(v_step - fp.height_m) < 0.001
