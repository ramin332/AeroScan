"""Regression tests for ``_filter_facades_by_dji_bbox`` and ``_expand_polygon_xy``.

These guard the "10 → 0" bug where the RC Plus mission polygon was applied
too tightly and dropped every facade whose centroid happened to sit on the
polygon edge. The fix is a small outward expansion (default 2m) that lets
edge-hugging facades survive. Without the expansion, the ``edge_hugging``
test below dropped the only facade — exactly the production symptom.
"""

from __future__ import annotations

import numpy as np
import pytest

import math

from flight_planner.models import Facade, meters_per_deg
from flight_planner.server.api import _expand_polygon_xy, _filter_facades_by_dji_bbox


def _enu_to_wgs84(x: float, y: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Inverse of polygon_to_enu: given ENU (x, y), return (lon, lat)."""
    m_per_lat, m_per_lon = meters_per_deg(math.radians(ref_lat))
    return (ref_lon + x / m_per_lon, ref_lat + y / m_per_lat)


def _facade_at_xy(cx: float, cy: float, *, index: int = 0) -> Facade:
    """Minimal vertical-wall facade centered at (cx, cy, 1.5)."""
    verts = np.array(
        [
            [cx - 0.5, cy, 0.0],
            [cx + 0.5, cy, 0.0],
            [cx + 0.5, cy, 3.0],
            [cx - 0.5, cy, 3.0],
        ],
        dtype=np.float64,
    )
    return Facade(
        vertices=verts,
        normal=np.array([0.0, -1.0, 0.0], dtype=np.float64),
        label=f"f{index}",
        component_tag="21.1",
        index=index,
    )


# ---------------------------------------------------------------------------
# _expand_polygon_xy
# ---------------------------------------------------------------------------


def test_expand_polygon_xy_noop_for_zero_margin():
    poly = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0), (0.0, 10.0, 0.0)]
    assert _expand_polygon_xy(poly, 0.0) == poly


def test_expand_polygon_xy_grows_each_vertex_outward():
    # Square centered at (5, 5); corners are 5·√2 from centroid. A 1m margin
    # scales radius to (5√2 + 1) / 5√2 ≈ 1.1414.
    poly = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0), (0.0, 10.0, 0.0)]
    out = _expand_polygon_xy(poly, 1.0)
    expanded = np.asarray(out)[:, :2]
    for p in expanded:
        r = np.linalg.norm(p - np.array([5.0, 5.0]))
        assert r == pytest.approx(5 * np.sqrt(2) + 1.0, abs=1e-6)


def test_expand_polygon_xy_empty_input_returns_input():
    assert _expand_polygon_xy([], 2.0) == []
    assert _expand_polygon_xy(None, 2.0) is None


# ---------------------------------------------------------------------------
# _filter_facades_by_dji_bbox
# ---------------------------------------------------------------------------


POLY_HALF_M = 5.0  # polygon extends ±5m on each ENU axis
REF_LAT, REF_LON = 52.0, 6.0


def _square_polygon_wgs84() -> list[tuple[float, float, float]]:
    """WGS84 square (as (lon, lat, alt)) whose ENU projection is exactly
    ±POLY_HALF_M on each axis relative to (REF_LAT, REF_LON).
    """
    corners_enu = [
        (-POLY_HALF_M, -POLY_HALF_M),
        (+POLY_HALF_M, -POLY_HALF_M),
        (+POLY_HALF_M, +POLY_HALF_M),
        (-POLY_HALF_M, +POLY_HALF_M),
    ]
    return [(*_enu_to_wgs84(x, y, REF_LAT, REF_LON), 0.0) for (x, y) in corners_enu]


def test_filter_keeps_facade_inside_polygon():
    poly = _square_polygon_wgs84()
    facades = [_facade_at_xy(0.0, 0.0)]  # ENU origin = inside polygon
    kept = _filter_facades_by_dji_bbox(facades, poly, REF_LAT, REF_LON, 0.0)
    assert len(kept) == 1


def test_filter_drops_facade_far_outside_polygon():
    poly = _square_polygon_wgs84()
    # 100m east of the polygon — well beyond any reasonable margin.
    facades = [_facade_at_xy(100.0, 0.0)]
    kept = _filter_facades_by_dji_bbox(facades, poly, REF_LAT, REF_LON, 0.0)
    assert len(kept) == 0


def test_filter_passes_through_when_no_polygon():
    facades = [_facade_at_xy(0.0, 0.0), _facade_at_xy(50.0, 50.0)]
    assert _filter_facades_by_dji_bbox(facades, None, REF_LAT, REF_LON, 0.0) == facades
    assert _filter_facades_by_dji_bbox(facades, [], REF_LAT, REF_LON, 0.0) == facades


def test_filter_edge_hugging_facade_survives_via_margin():
    """Regression guard for the "10 → 0" bug.

    A facade centroid sitting ~1m *outside* the polygon edge (as happens when
    the building footprint hugs the RC Plus mapping polygon) must survive
    thanks to the default 2m margin. With ``margin_m=0`` it would be dropped.
    """
    poly = _square_polygon_wgs84()  # ENU polygon spans ±POLY_HALF_M on each axis
    # Facade 1m east of the polygon edge — along the axis the margin moves the
    # edge by exactly margin_m * (POLY_HALF_M / corner_radius), which at y=0
    # equals margin_m (since the corner-radial vector at the midpoint-of-edge
    # passes through the corner; the edge itself moves by the projection of
    # the corner displacement onto the x-axis, which is POLY_HALF_M/corner_r *
    # margin_m). Keep x_offset within 2m * cos(45°) ≈ 1.41m so the test is
    # robust to the geometry.
    facade = _facade_at_xy(POLY_HALF_M + 1.0, 0.0)

    kept_default = _filter_facades_by_dji_bbox(
        [facade], poly, REF_LAT, REF_LON, 0.0
    )
    assert len(kept_default) == 1, "2m margin should save edge-hugging facade"

    kept_no_margin = _filter_facades_by_dji_bbox(
        [facade], poly, REF_LAT, REF_LON, 0.0, margin_m=0.0
    )
    assert len(kept_no_margin) == 0, "without margin, edge-hugging facade is dropped"
