"""Preset Dutch housing block geometries for testing."""

from __future__ import annotations

from .geometry import build_l_shaped_building, build_rectangular_building
from .models import Building, RoofType

# Leeuwarden coordinates (AeroScan HQ area)
DEFAULT_LAT = 53.2012
DEFAULT_LON = 5.7999


def simple_box(
    lat: float = DEFAULT_LAT,
    lon: float = DEFAULT_LON,
) -> Building:
    """A simple rectangular building: 20m x 10m x 8m, flat roof.

    Typical small Dutch housing block (portiekflat).
    """
    return build_rectangular_building(
        lat=lat, lon=lon,
        width=20.0, depth=10.0, height=8.0,
        heading_deg=0.0,
        roof_type=RoofType.FLAT,
        num_stories=3,
        label="simple_box",
    )


def pitched_roof_house(
    lat: float = DEFAULT_LAT,
    lon: float = DEFAULT_LON,
) -> Building:
    """A residential building with a 30-degree pitched roof.

    Typical Dutch row house block: 30m x 10m x 6m eave, 30° roof pitch.
    """
    return build_rectangular_building(
        lat=lat, lon=lon,
        width=30.0, depth=10.0, height=6.0,
        heading_deg=45.0,  # rotated 45° for testing
        roof_type=RoofType.PITCHED,
        roof_pitch_deg=30.0,
        num_stories=2,
        label="pitched_roof_house",
    )


def l_shaped_block(
    lat: float = DEFAULT_LAT,
    lon: float = DEFAULT_LON,
) -> Building:
    """An L-shaped building made of two rectangular wings.

    Wing 1: 25m x 10m (main wing along north)
    Wing 2: 15m x 10m (extending east from end of wing 1)
    Height: 9m (3 stories), flat roof.
    """
    return build_l_shaped_building(
        lat=lat, lon=lon,
        wing1_width=25.0, wing1_depth=10.0,
        wing2_width=15.0, wing2_depth=10.0,
        height=9.0,
        heading_deg=0.0,
        label="l_shaped_block",
    )


def large_apartment_block(
    lat: float = DEFAULT_LAT,
    lon: float = DEFAULT_LON,
) -> Building:
    """A large apartment block: 60m x 12m x 18m, flat roof.

    Typical Dutch galerijflat (gallery-access apartment building).
    """
    return build_rectangular_building(
        lat=lat, lon=lon,
        width=60.0, depth=12.0, height=18.0,
        heading_deg=15.0,
        roof_type=RoofType.FLAT,
        num_stories=6,
        label="large_apartment_block",
    )
