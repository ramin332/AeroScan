"""Sequence-level facade assignment: decide the whole aim path at once.

Measured on the real 2026-07-10 busboom waypoints against the fresh flight0072
mesh (2026-09-02, docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md):
the greedy per-waypoint picker with one-step hysteresis produced 76 target
switches, 23 aim reversals >90°, 79 picks that disagreed with their ±3
neighbours, and picks up to 31 m away because `max_distance_m=60` let a
waypoint over empty tarmac grab whatever was within 60 m.

Three fixes, all testable without a flight:
  A. a plausible standoff cap — nothing in range → keep DJI's own pose;
  B. Viterbi over the sequence with a switch cost that grows with the turn,
     and coplanar facets counted as one target (slice-hopping is free);
  C. an aim audit that reports far picks / flips / blips before flying.
"""
from __future__ import annotations

import numpy as np

from flight_planner.gimbal_rewrite import (
    aim_audit,
    assign_facades_viterbi,
    plane_groups,
    rewrite_gimbals_perpendicular,
)
from flight_planner.models import Facade, Waypoint


def _quad(center, normal, size=2.0, label="wall_x"):
    n = np.asarray(normal, float); n = n / np.linalg.norm(n)
    ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, ref); u /= np.linalg.norm(u); v = np.cross(n, u)
    c = np.asarray(center, float); h = size / 2
    verts = np.array([c - h*u - h*v, c + h*u - h*v, c + h*u + h*v, c - h*u + h*v])
    return Facade(vertices=verts, normal=n, component_tag="21.1", label=label)


def _wp(x, y, z=5.0, index=0, pitch=-19.0):
    return Waypoint(x=x, y=y, z=z, lat=0.0, lon=0.0, alt=z, heading_deg=0.0,
                    gimbal_pitch_deg=pitch, gimbal_yaw_deg=None, speed_ms=2.0,
                    actions=[], facade_index=-1, index=index)


def _switches(picks):
    return sum(1 for a, b in zip(picks, picks[1:]) if a != b)


# --- A. standoff cap --------------------------------------------------------

def test_facet_beyond_cap_keeps_djis_own_pose():
    far = _quad((0.0, 30.0, 5.0), (0.0, -1.0, 0.0))          # 30 m north, facing us
    wps = [_wp(0.0, 0.0, index=i, pitch=-33.0) for i in range(3)]
    out = rewrite_gimbals_perpendicular(wps, [far], max_distance_m=15.0, preserve_heading=False)
    assert all(w.facade_index == -1 for w in out)
    assert all(w.gimbal_pitch_deg == -33.0 for w in out)     # DJI's pitch untouched
    assert all(w.heading_deg == 0.0 for w in out)            # DJI's heading untouched


# --- B. sequence assignment ------------------------------------------------

def _corner_scene():
    """Sweep east along y=-6 past a building corner at the origin: wall A faces
    south (x<0 side), wall B faces south too but the sweep also passes wall C
    facing east. Greedy flips near the corner; the sequence should switch once."""
    wall_a = _quad((-6.0, 0.0, 5.0), (0.0, -1.0, 0.0), size=6.0, label="wall_a")
    wall_c = _quad((3.0, 6.0, 5.0), (1.0, 0.0, 0.0), size=6.0, label="wall_c")
    wps = [_wp(x, -6.0, index=i) for i, x in enumerate(np.linspace(-10.0, 12.0, 23))]
    return wps, [wall_a, wall_c]


def test_viterbi_switches_once_at_a_corner():
    wps, facs = _corner_scene()
    picks = assign_facades_viterbi(wps, facs, max_distance_m=30.0, switch_cost=4.0)
    assert -1 not in picks
    assert _switches(picks) == 1
    assert picks[0] == 0 and picks[-1] == 1


def test_viterbi_ignores_a_one_waypoint_decoy():
    """A small facet that is nearest for exactly one waypoint must not steal
    the aim for that waypoint (a 'blip') when the wall is tracked before and after."""
    wall = _quad((0.0, 6.0, 5.0), (0.0, -1.0, 0.0), size=8.0, label="wall_x")
    decoy = _quad((0.0, 2.0, 5.0), (0.0, -1.0, 0.0), size=1.0, label="roof_x")  # 4 m closer, but only near WP 5
    wps = [_wp(x, 0.0, index=i) for i, x in enumerate(np.linspace(-8.0, 8.0, 11))]
    greedy = [w.facade_index for w in rewrite_gimbals_perpendicular(wps, [wall, decoy], max_distance_m=30.0, assign_mode="greedy", switch_ratio=1.0)]
    assert 1 in greedy                                        # greedy takes the decoy
    picks = assign_facades_viterbi(wps, [wall, decoy], max_distance_m=30.0, switch_cost=4.0)
    assert set(picks) == {0}                                  # sequence never leaves the wall


def test_coplanar_slices_form_one_group_and_perpendicular_walls_do_not():
    lo = _quad((0.0, 0.0, 2.0), (0.0, -1.0, 0.0), label="wall_lo")
    hi = _quad((0.0, 0.0, 5.0), (0.0, -1.0, 0.0), label="wall_hi")
    side = _quad((4.0, 4.0, 3.0), (1.0, 0.0, 0.0), label="wall_side")
    g = plane_groups([lo, hi, side])
    assert g[0] == g[1] and g[0] != g[2]


def test_switching_between_slices_of_one_wall_is_free():
    """Two stacked slices of one wall: Viterbi may hop between them (pitch follows
    the nearer slice) — no switch penalty should pin it to the wrong slice."""
    lo = _quad((0.0, 6.0, 1.0), (0.0, -1.0, 0.0), size=2.0, label="wall_lo")
    hi = _quad((0.0, 6.0, 7.0), (0.0, -1.0, 0.0), size=2.0, label="wall_hi")
    wps = [_wp(0.0, 0.0, z=z, index=i) for i, z in enumerate([1.0, 1.0, 7.0, 7.0])]
    picks = assign_facades_viterbi(wps, [lo, hi], max_distance_m=30.0, switch_cost=50.0)
    assert picks == [0, 0, 1, 1]


def test_rewrite_default_mode_is_viterbi_and_greedy_is_selectable():
    wps, facs = _corner_scene()
    v = [w.facade_index for w in rewrite_gimbals_perpendicular(wps, facs, max_distance_m=30.0, preserve_heading=False)]
    g = [w.facade_index for w in rewrite_gimbals_perpendicular(wps, facs, max_distance_m=30.0, preserve_heading=False, assign_mode="greedy")]
    assert _switches(v) == 1
    assert len(g) == len(v)


# --- C. audit ---------------------------------------------------------------

def test_aim_audit_counts_far_picks_flips_and_blips():
    wall = _quad((0.0, 6.0, 5.0), (0.0, -1.0, 0.0), size=8.0, label="wall_x")
    far = _quad((40.0, 0.0, 5.0), (-1.0, 0.0, 0.0), size=8.0, label="wall_far")
    wps = [_wp(x, 0.0, index=i) for i, x in enumerate(np.linspace(-8.0, 8.0, 9))]
    out = rewrite_gimbals_perpendicular(wps, [wall, far], max_distance_m=60.0, preserve_heading=False, assign_mode="greedy", switch_ratio=1.0)
    # force a blip + a flip by hand so the audit has something to count
    out[4] = out[4].__class__(**{**out[4].__dict__, "facade_index": 1, "heading_deg": 120.0})
    a = aim_audit(out, [wall, far], far_standoff_m=15.0)
    assert a["far_picks"] == 1
    assert a["reversals_gt90"] >= 1
    assert a["single_blips"] == 1
    assert a["switches"] == 2
    assert set(a) >= {"far_picks", "reversals_gt90", "switches", "single_blips", "unaimed", "standoff_p90_m"}
