"""Tests for visualize.prepare_threejs_data viewer payload."""

from __future__ import annotations

import numpy as np

from flight_planner.models import Building, Facade, RoofType
from flight_planner.visualize import prepare_threejs_data


def _stub_building() -> Building:
    # One vertical wall facade so prepare_threejs_data has something to iterate.
    facade = Facade(
        index=0,
        vertices=np.array([
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 0.0, 3.0],
            [0.0, 0.0, 3.0],
        ], dtype=np.float64),
        normal=np.array([0.0, -1.0, 0.0], dtype=np.float64),
        label="South",
        component_tag="21.1",
    )
    return Building(
        lat=52.0, lon=6.0, ground_altitude=0.0,
        width=4.0, depth=4.0, height=3.0,
        heading_deg=0.0,
        roof_type=RoofType.FLAT, roof_pitch_deg=0.0,
        num_stories=1, facades=[facade], label="stub",
    )


def test_prepare_threejs_data_emits_mapping_box_when_provided():
    b = _stub_building()
    mapping = {
        "center": [1.0, 2.0, 3.0],
        "axes": [[10.0, 0.0, 0.0], [0.0, 20.0, 0.0], [0.0, 0.0, 5.0]],
    }
    out = prepare_threejs_data(b, [], mapping_bbox=mapping)
    assert out["mappingBox"] == mapping


def test_prepare_threejs_data_no_mapping_box_by_default():
    b = _stub_building()
    out = prepare_threejs_data(b, [])
    assert "mappingBox" not in out
