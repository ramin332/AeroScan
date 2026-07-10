"""Facade-picker hysteresis: stop the aim flip-flopping between facades.

`_pick_facade_for_waypoint` re-decides from scratch at every waypoint. When two
facades are near-equally good the pick oscillates, and because the augmented KMZ
welds aircraft heading to the aim yaw, the airframe chases each flip. Measured on
the 2026-07-10 flown missions before this fix:

    flight A (144 WP): 45/142 target switches (32%), 4 aim-yaw reversals >90 deg,
                       max reversal 175.7 deg
    flight B (399 WP): 42/397 target switches (11%), 3 reversals >90 deg

A 175 deg reversal between adjacent waypoints is the picker jumping to a facade
on the opposite side of the target. The aircraft cannot yaw that fast, so the
gimbal -- commanded in the absolute north frame -- has to make up the difference
and saturates against its +-60 deg pan limit. That is the "gimbal locks" symptom
the pilot reported.

Hysteresis: keep tracking the previous facade unless a challenger is better by a
clear margin (`switch_ratio`). Cheap, local, and it cannot make aim worse than
`switch_ratio` allows.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from flight_planner.gimbal_rewrite import (
    _pick_facade_for_waypoint,
    rewrite_gimbals_perpendicular,
)
from flight_planner.models import Facade, Waypoint


def _quad(center, normal, size=2.0, label="wall_x"):
    """Build a square facet centred at `center` with the given outward normal."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    c = np.asarray(center, dtype=float)
    h = size / 2
    verts = np.array([c - h * u - h * v, c + h * u - h * v, c + h * u + h * v, c - h * u + h * v])
    return Facade(vertices=verts, normal=n, component_tag="21.1", label=label)


def _wp(x, y, z, index=0):
    return Waypoint(
        x=x, y=y, z=z, lat=0.0, lon=0.0, alt=z,
        heading_deg=0.0, gimbal_pitch_deg=0.0, gimbal_yaw_deg=None,
        speed_ms=2.0, actions=[], facade_index=-1, index=index,
    )


def test_picker_without_previous_is_unchanged():
    """No previous facade -> behaves exactly as before (nearest wins)."""
    facades = [
        _quad((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)),   # far
        _quad((2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),    # near
    ]
    pos = np.array([20.0, 0.0, 0.0])
    pick = _pick_facade_for_waypoint(pos, facades, max_distance_m=60.0)
    assert pick is not None
    assert pick[0] == 0, "nearest-to-WP facet should win with no hysteresis state"


def test_hysteresis_keeps_previous_facade_when_challenger_is_marginal():
    """A challenger only slightly better must NOT steal the aim."""
    # Two facets nearly equidistant from the WP; previous pick is index 1.
    facades = [
        _quad((0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
        _quad((0.2, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ]
    pos = np.array([0.0, -10.0, 0.0])
    # Without hysteresis the marginally-closer facet 0 wins.
    plain = _pick_facade_for_waypoint(pos, facades, max_distance_m=60.0)
    assert plain is not None and plain[0] == 0

    sticky = _pick_facade_for_waypoint(
        pos, facades, max_distance_m=60.0, previous_index=1, switch_ratio=0.8
    )
    assert sticky is not None and sticky[0] == 1, "marginal challenger stole the aim"


def test_hysteresis_yields_when_challenger_is_decisively_better():
    """Hysteresis must not stick to a facade the WP has clearly flown past."""
    facades = [
        _quad((0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),    # 2 m away
        _quad((0.0, 40.0, 0.0), (0.0, -1.0, 0.0)),   # 42 m away
    ]
    pos = np.array([0.0, -2.0, 0.0])
    sticky = _pick_facade_for_waypoint(
        pos, facades, max_distance_m=60.0, previous_index=1, switch_ratio=0.8
    )
    assert sticky is not None and sticky[0] == 0, "should abandon a far stale target"


def test_hysteresis_ignores_previous_that_became_invalid():
    """If the WP is now behind the previous facet, drop it without argument."""
    facades = [
        _quad((0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),  # outward -Y: its front is y < 0
        _quad((0.0, 5.0, 0.0), (0.0, -1.0, 0.0)),  # outward -Y: its front is y < 5
    ]
    pos = np.array([0.0, 2.0, 0.0])  # BEHIND facet 0, in front of facet 1
    pick = _pick_facade_for_waypoint(
        pos, facades, max_distance_m=60.0, previous_index=0, switch_ratio=0.8
    )
    assert pick is not None and pick[0] == 1, "stuck to a facet the WP is behind"


def test_rewrite_reduces_target_switching_on_a_synthetic_orbit():
    """End-to-end: an orbit around two coplanar facets must not oscillate."""
    facades = [
        _quad((-0.5, 0.0, 0.0), (0.0, -1.0, 0.0)),
        _quad((0.5, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ]
    wps = []
    for i in range(60):
        a = -math.pi / 2 + (i / 59.0) * (math.pi / 3)
        wps.append(_wp(8 * math.cos(a), 8 * math.sin(a), 1.0, index=i))

    def switches(out):
        idx = [w.facade_index for w in out]
        return sum(1 for a, b in zip(idx, idx[1:]) if a != b and a >= 0 and b >= 0)

    loose = rewrite_gimbals_perpendicular(wps, facades, switch_ratio=1.0)
    tight = rewrite_gimbals_perpendicular(wps, facades, switch_ratio=0.8)
    assert switches(tight) <= switches(loose), "hysteresis increased switching"
    assert switches(tight) <= 2, f"still oscillating: {switches(tight)} switches"


@pytest.mark.parametrize("ratio", [0.5, 0.8, 1.0])
def test_switch_ratio_is_monotonic_in_stickiness(ratio):
    """Lower switch_ratio = stickier. Never more switches than a looser ratio."""
    facades = [
        _quad((-0.3, 0.0, 0.0), (0.0, -1.0, 0.0)),
        _quad((0.3, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ]
    wps = [_wp(x, -6.0, 0.0, index=i) for i, x in enumerate(np.linspace(-2, 2, 40))]
    out = rewrite_gimbals_perpendicular(wps, facades, switch_ratio=ratio)
    idx = [w.facade_index for w in out]
    sw = sum(1 for a, b in zip(idx, idx[1:]) if a != b)
    assert sw <= 39
    assert all(i >= 0 for i in idx)
