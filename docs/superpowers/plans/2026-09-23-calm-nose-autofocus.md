# Calm Nose, Autofocus, Nearest-Point Picker — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The augment pipeline picks facets by the same nearest point it aims at, turns the nose at a bounded rate while the gimbal does the fine aim, and autofocuses before photos.

**Architecture:** Three independent changes to the existing augment pipeline (`cli.augment_mission` → `gimbal_rewrite.rewrite_gimbals_perpendicular` → `kmz_builder.build_kmz`). The picker change is internal to `gimbal_rewrite.py`. The calm nose turns on an existing, tested code path (`command_gimbal_yaw=True` + `schedule_headings`). Autofocus adds one action type, a small planning module that inserts it, and its WPML serialisation. New knobs ride in the mission-intent `settings` so no RC or DPK rebuild is needed.

**Tech Stack:** Python 3.12, numpy, djikmz (WPML builder), pytest. Run tests with `.venv/bin/python -m pytest`.

**Spec:** `docs/superpowers/specs/2026-09-23-flow-inspection-design.md`

## Global Constraints

- `focus` action fields, exactly: `isPointFocus=0`, `focusX=0.4`, `focusY=0.4`, `focusRegionWidth=0.2`, `focusRegionHeight=0.2`, `isInfiniteFocus=0`, `isCalibrationFocus=0`, `payloadPositionIndex=0`.
- Action order at a photo: gimbal command → `focus` → `takePhoto`.
- `autofocus` setting: int 0–2, default 2 (0 off, 1 on change, 2 every photo). Refocus threshold for mode 1: distance differs by more than 15% from the distance at the last focus, or the facet changed.
- `af_time_s`: float 0–3, default 0.5. `smooth_heading`: int 0–1, default 1.
- Heading schedule: `rate_fraction` 0.5 of 60°/s, gimbal pan capped at ±50° (existing defaults of `schedule_headings` / `rewrite_gimbals_perpendicular`; do not change them).
- `startActionGroup` keeps `setFocusType manual`. Do not change it.
- Extra shots still only go to facets no other waypoint photographs, each once.
- Two-shot waypoints keep stopping. No waypoint splitting, no per-leg speed.
- Comments and commit messages in plain English, matching the surrounding style (why, with measured evidence where it exists).

## Review Focus

- A facet with fewer than 3 vertices must aim at its centroid, never crash (`aim_point` / vectorised frames).
- An unaimed waypoint (`facade_index == -1`) with a photo gets a focus in mode 1 and 2, and never indexes `facades[-1]`.
- A waypoint whose extra-shot list is shorter than its photo count (DJI-left photo actions) must not raise in `insert_focus_actions`.
- Dedupe (`_dedupe_pose_actions`) must leave `focus` actions alone.
- With `smooth_heading=0`, output must equal today's behaviour (no gimbal yaw commanded).

Each of these has a test in the owning task below.

---

### Task 1: Vectorised nearest-point frames

**Files:**
- Modify: `src/flight_planner/gimbal_rewrite.py` (the `aim_point` function added in `570427b`, near the top)
- Test: `tests/test_gimbal_rewrite.py`

**Interfaces:**
- Produces:
  - `_facet_frames(facades: Sequence[Facade], inset_frac: float = 0.25) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]` returning `(C, U, V, LO, HI)`: centres `(F,3)`, in-plane axes `(F,3)` each, clamp lows and highs `(F,2)` as `(u, v)`.
  - `_aim_points(frames, pos: np.ndarray) -> np.ndarray` returning `(F,3)` nearest points, one per facet.
  - `aim_point(facade, pos, inset_frac=0.25) -> np.ndarray` keeps its signature and behaviour.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_gimbal_rewrite.py`:

```python
def test_vectorised_aim_points_match_single_aim_point():
    from flight_planner.gimbal_rewrite import _aim_points, _facet_frames, aim_point

    facs = [
        _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), size=20.0),
        _make_facade(normal=(0.0, 1.0, 0.0), center=(3.0, 4.0, 5.0), size=2.0),
        _make_facade(normal=(0.0, 0.0, 1.0), center=(1.0, 1.0, 8.0), size=6.0),
    ]
    rng = np.random.default_rng(0)
    frames = _facet_frames(facs)
    for _ in range(20):
        pos = rng.uniform(-15, 15, size=3)
        many = _aim_points(frames, pos)
        for i, f in enumerate(facs):
            assert np.allclose(many[i], aim_point(f, pos))


def test_degenerate_facet_aims_at_its_centroid():
    from flight_planner.gimbal_rewrite import aim_point

    f = Facade(vertices=np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), normal=np.array([0.0, 1.0, 0.0]))
    assert np.allclose(aim_point(f, np.array([5.0, 5.0, 5.0])), [1.0, 0.0, 0.0])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_gimbal_rewrite.py -k "vectorised or degenerate" -v`
Expected: FAIL with `ImportError: cannot import name '_aim_points'`.

- [ ] **Step 3: Implement**

Replace the body of `aim_point` and add the two helpers directly above it:

```python
def _facet_frames(facades: Sequence[Facade], inset_frac: float = 0.25):
    """Plane frame per facet for ``_aim_points``: centre, in-plane axes u and v,
    and the clamp box in (u, v), inset from the edges by ``inset_frac``.

    Computed once per mission so the picker can ask "nearest point of every
    facet" per waypoint without rebuilding frames. A facet with fewer than
    three vertices gets a zero box, so it aims at its centroid.
    """
    F = len(facades)
    C = np.zeros((F, 3))
    U = np.zeros((F, 3))
    V = np.zeros((F, 3))
    LO = np.zeros((F, 2))
    HI = np.zeros((F, 2))
    k = 1.0 - inset_frac
    for i, f in enumerate(facades):
        verts = np.asarray(f.vertices, dtype=np.float64).reshape(-1, 3)
        c = verts.mean(axis=0)
        C[i] = c
        n = _unit(np.asarray(f.normal, dtype=np.float64))
        # u horizontal along the facet (or east for a flat roof), v = n × u.
        u = np.cross(np.array([0.0, 0.0, 1.0]), n)
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1.0, 0.0, 0.0])
        u = _unit(u)
        v = _unit(np.cross(n, u))
        U[i], V[i] = u, v
        if len(verts) < 3:
            continue
        rel = verts - c
        pu, pv = rel @ u, rel @ v
        LO[i] = (pu.min() * k, pv.min() * k)
        HI[i] = (pu.max() * k, pv.max() * k)
    return C, U, V, LO, HI


def _aim_points(frames, pos: np.ndarray) -> np.ndarray:
    """Nearest point of every facet to ``pos`` — see ``aim_point``. Shape (F, 3)."""
    C, U, V, LO, HI = frames
    d = np.asarray(pos, dtype=np.float64) - C
    su = np.clip(np.einsum("ij,ij->i", d, U), LO[:, 0], HI[:, 0])
    sv = np.clip(np.einsum("ij,ij->i", d, V), LO[:, 1], HI[:, 1])
    return C + su[:, None] * U + sv[:, None] * V


def aim_point(facade: Facade, pos: np.ndarray, inset_frac: float = 0.25) -> np.ndarray:
    """Point on ``facade`` the camera at ``pos`` should look at.

    (keep the existing docstring text from 570427b unchanged here)
    """
    return _aim_points(_facet_frames([facade], inset_frac), pos)[0]
```

- [ ] **Step 4: Run the whole gimbal test file**

Run: `.venv/bin/python -m pytest tests/test_gimbal_rewrite.py -v`
Expected: all PASS, including the two tests from `570427b` (long wall, beyond the end).

- [ ] **Step 5: Commit**

```bash
git add src/flight_planner/gimbal_rewrite.py tests/test_gimbal_rewrite.py
git commit -m "refactor: vectorise nearest-point aim so the picker can use it"
```

---

### Task 2: Picker, greedy and extra shots by nearest point

**Files:**
- Modify: `src/flight_planner/gimbal_rewrite.py` — `assign_facades_viterbi`, `_pick_facade_for_waypoint`, `assign_extra_shots`
- Test: `tests/test_facade_assignment.py`

**Interfaces:**
- Consumes: `_facet_frames`, `_aim_points` (Task 1).
- Produces: unchanged signatures. `_pick_facade_for_waypoint` now returns `(index, distance_to_aim_point)`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_facade_assignment.py` (check the file's existing imports and helpers first; if it has no square-facade helper, add this one):

```python
import math

import numpy as np

from flight_planner.gimbal_rewrite import (
    _pick_facade_for_waypoint,
    assign_extra_shots,
    assign_facades_viterbi,
)
from flight_planner.models import Facade, Waypoint


def _sq(normal, center, size):
    n = np.array(normal, float); n /= np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, up); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    c = np.array(center, float); h = size / 2.0
    verts = np.array([c - h*u - h*v, c + h*u - h*v, c + h*u + h*v, c - h*u + h*v])
    return Facade(vertices=verts, normal=n, label="test")


# WP 5 m in front of a 20 m wall, 9 m along it from the wall's centre.
# Wall centroid: 10.3 m away. Wall nearest point (inset clamp y=7.5): 5.2 m.
# Small facet near the wall's far end: 8.3 m. Centroid scoring picked the
# small facet; nearest-point scoring picks the wall the camera aims at.
LONG_WALL = _sq((1.0, 0.0, 0.0), (0.0, 0.0, 2.0), 20.0)
SMALL = _sq((1.0, 0.0, 0.0), (0.0, 16.0, 2.0), 1.0)
WP = Waypoint(x=5.0, y=9.0, z=2.0)


def test_viterbi_scores_the_nearest_point_not_the_centroid():
    assert assign_facades_viterbi([WP], [LONG_WALL, SMALL], max_distance_m=30.0) == [0]


def test_greedy_scores_the_nearest_point_not_the_centroid():
    pick = _pick_facade_for_waypoint(np.array([5.0, 9.0, 2.0]), [LONG_WALL, SMALL], 30.0)
    assert pick is not None and pick[0] == 0
    assert abs(pick[1] - math.hypot(5.0, 1.5)) < 1e-6


def test_extra_shot_reach_uses_the_nearest_point():
    # Reach 6 m: the long wall's centroid (10.3 m) is out of reach, its nearest
    # point (5.2 m) is not. Primary is another facet, so the wall is free.
    primary = _sq((1.0, 0.0, 0.0), (0.0, 30.0, 2.0), 1.0)
    wp = Waypoint(x=5.0, y=9.0, z=2.0, heading_deg=-90.0)
    extras = assign_extra_shots(
        [wp], [primary, LONG_WALL], [0],
        max_distance_m=6.0, pan_window_deg=45.0, shots_per_waypoint=2,
        pitch_min=-88.0, pitch_max=33.0,
    )
    assert [fi for fi, _p, _y in extras[0]] == [1]
    assert abs(extras[0][0][2] - math.degrees(math.atan2(-5.0, -1.5))) < 0.5
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_facade_assignment.py -k "nearest_point" -v`
Expected: 3 FAIL (Viterbi returns `[1]`, greedy picks 1, extras list empty).

- [ ] **Step 3: Implement `assign_facades_viterbi`**

Replace `C = np.array([np.asarray(f.center, float) for f in facades]).reshape(F, 3)` with `frames = _facet_frames(facades)`. Inside the per-waypoint loop replace `d3 = P[i] - C` with:

```python
        # Score, gate and bear on the point the camera will actually aim at —
        # the nearest point of each facet — not its centroid. A long wall's
        # centroid is far from a waypoint at one end of it, so the wall used to
        # lose to small facets nearby while the aim would have been head-on.
        d3 = P[i] - _aim_points(frames, P[i])
```

Everything after it (`dist`, `signed`, `valid`, `pitch`, `bearing`) stays as is: it already derives from `d3`. `signed` is unchanged in meaning because the aim point lies on the facet plane, as the centroid does. Leave `plane_groups` using centroids. Update the docstring's first cost sentence to say "3D distance to the facet's nearest point".

- [ ] **Step 4: Implement `_pick_facade_for_waypoint`**

Before the loop add `A = _aim_points(_facet_frames(facades), wp_xyz)`. In the loop replace `dist3d = float(np.linalg.norm(c - wp_xyz))` with `dist3d = float(np.linalg.norm(A[i] - wp_xyz))`. Keep the `signed` test on `c`. Update the docstring: "Returns (facade_index, 3d_distance_to_nearest_point)" and "Sort metric: 3D distance from the WP to the facet's nearest point".

- [ ] **Step 5: Implement `assign_extra_shots`**

Replace `C = np.array([np.asarray(f.center, float) for f in facades])` with `frames = _facet_frames(facades)`. In the per-waypoint body replace `d3 = C - pos` with `d3 = _aim_points(frames, pos) - pos`. Then the per-facet `a = aim_point(facades[i], pos) - pos` added in `570427b` duplicates `d3[i]`; replace those two lines with:

```python
            p = float(min(pitch_max, max(pitch_min, pitch[i])))
            y = float(_wrap180(bearing[i]))
```

(`pitch` and `bearing` are already computed from `d3` above, which is now the nearest-point vector.)

- [ ] **Step 6: Run the full suite**

Run: `.venv/bin/python -m pytest -q -p no:warnings`
Expected: all PASS. If an existing Viterbi/greedy test fails, read it: a test that asserted a centroid-scored choice is now wrong by design only if its geometry matches the case above; otherwise it's a real regression — stop and investigate.

- [ ] **Step 7: Commit**

```bash
git add src/flight_planner/gimbal_rewrite.py tests/test_facade_assignment.py
git commit -m "fix: choose facets by the nearest point the camera aims at"
```

---

### Task 3: Focus action type and focus planning

**Files:**
- Modify: `src/flight_planner/models.py` — `ActionType`
- Create: `src/flight_planner/focus_plan.py`
- Test: `tests/test_focus_plan.py`

**Interfaces:**
- Consumes: `aim_point` (Task 1).
- Produces:
  - `ActionType.FOCUS = "focus"`
  - `insert_focus_actions(waypoints: Sequence[Waypoint], facades: Sequence[Facade], mode: int, *, refocus_change: float = 0.15) -> list[Waypoint]`
  - Constants `AUTOFOCUS_OFF = 0`, `AUTOFOCUS_ON_CHANGE = 1`, `AUTOFOCUS_EVERY_PHOTO = 2`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_focus_plan.py`:

```python
"""Where the mission autofocuses before a photo."""

import numpy as np

from flight_planner.focus_plan import insert_focus_actions
from flight_planner.models import ActionType, CameraAction, Facade, Waypoint


def _wall(x0: float, size: float = 20.0) -> Facade:
    # East-facing wall in the plane x = x0, centred at y=0, z=5.
    h = size / 2.0
    verts = np.array([[x0, -h, 5 - h], [x0, h, 5 - h], [x0, h, 5 + h], [x0, -h, 5 + h]])
    return Facade(vertices=verts, normal=np.array([1.0, 0.0, 0.0]), label="wall_e")


def _photo_wp(x: float, y: float, facet: int, extras=()) -> Waypoint:
    acts = [CameraAction(action_type=ActionType.TAKE_PHOTO)]
    for _ in extras:
        acts += [CameraAction(action_type=ActionType.GIMBAL_ROTATE), CameraAction(action_type=ActionType.TAKE_PHOTO)]
    return Waypoint(x=x, y=y, z=5.0, facade_index=facet, extra_facade_indices=list(extras), actions=acts)


def _kinds(wp: Waypoint) -> list[str]:
    return [a.action_type.value for a in wp.actions]


def test_mode_0_changes_nothing():
    wps = [_photo_wp(6.0, 0.0, 0)]
    out = insert_focus_actions(wps, [_wall(0.0)], 0)
    assert _kinds(out[0]) == ["takePhoto"]


def test_mode_2_focuses_before_every_photo_after_the_gimbal_moves():
    wps = [_photo_wp(6.0, 0.0, 0, extras=[1]), _photo_wp(6.0, 1.0, 0)]
    out = insert_focus_actions(wps, [_wall(0.0), _wall(-3.0)], 2)
    assert _kinds(out[0]) == ["focus", "takePhoto", "gimbalRotate", "focus", "takePhoto"]
    assert _kinds(out[1]) == ["focus", "takePhoto"]


def test_mode_1_reuses_focus_along_the_same_wall_at_the_same_distance():
    wps = [_photo_wp(6.0, y, 0) for y in (0.0, 1.5, 3.0)]
    out = insert_focus_actions(wps, [_wall(0.0)], 1)
    assert [_kinds(w) for w in out] == [["focus", "takePhoto"], ["takePhoto"], ["takePhoto"]]


def test_mode_1_refocuses_when_distance_changes_more_than_15_percent():
    wps = [_photo_wp(6.0, 0.0, 0), _photo_wp(6.8, 0.0, 0), _photo_wp(7.2, 0.0, 0)]
    out = insert_focus_actions(wps, [_wall(0.0)], 1)
    # 6.8 is +13% of 6.0: reuse. 7.2 is +20% of the last FOCUS distance (6.0): refocus.
    assert [_kinds(w) for w in out] == [["focus", "takePhoto"], ["takePhoto"], ["focus", "takePhoto"]]


def test_mode_1_refocuses_when_the_facet_changes():
    wps = [_photo_wp(6.0, 0.0, 0), _photo_wp(6.0, 0.0, 1)]
    out = insert_focus_actions(wps, [_wall(0.0), _wall(0.0)], 1)
    assert [_kinds(w) for w in out] == [["focus", "takePhoto"], ["focus", "takePhoto"]]


def test_unaimed_waypoint_focuses_and_does_not_index_minus_one():
    wps = [_photo_wp(6.0, 0.0, -1), _photo_wp(6.0, 0.0, -1)]
    out = insert_focus_actions(wps, [_wall(0.0)], 1)
    assert [_kinds(w) for w in out] == [["focus", "takePhoto"], ["focus", "takePhoto"]]


def test_more_photos_than_targets_does_not_raise():
    wp = Waypoint(x=6.0, y=0.0, z=5.0, facade_index=0, actions=[
        CameraAction(action_type=ActionType.TAKE_PHOTO),
        CameraAction(action_type=ActionType.TAKE_PHOTO),
    ])
    out = insert_focus_actions([wp], [_wall(0.0)], 2)
    assert _kinds(out[0]) == ["focus", "takePhoto", "focus", "takePhoto"]


def test_input_is_not_mutated():
    wps = [_photo_wp(6.0, 0.0, 0)]
    insert_focus_actions(wps, [_wall(0.0)], 2)
    assert _kinds(wps[0]) == ["takePhoto"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_focus_plan.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'flight_planner.focus_plan'`.

- [ ] **Step 3: Add the action type**

In `src/flight_planner/models.py`, `class ActionType(Enum)`, add after `HOVER = "hover"`:

```python
    # One-shot autofocus on an area of the frame. Emitted by kmz_builder as a
    # WPML `focus` action; see focus_plan.insert_focus_actions.
    FOCUS = "focus"
```

- [ ] **Step 4: Create `src/flight_planner/focus_plan.py`**

```python
"""Where the mission autofocuses before a photo.

Every mission flown before 2026-09-23 set manual focus once at the start and
never focused again. The 2026-09-07 Houten photos show the lens parked near
infinity (LensPosition 29-30 against an infinity position of 27-28) while the
laser put the wall at a median 7.3 m and a p10 of 5.4 m. DJI's own Smart3D
flight over the same house shot at LensPosition 42: its runtime refocuses,
ours did not. The client asked for autofocus.

The WPML `focus` action is a one-shot autofocus on an area of the frame. This
module decides where it goes; kmz_builder serialises it.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Sequence

import numpy as np

from .gimbal_rewrite import aim_point
from .models import ActionType, CameraAction, Facade, Waypoint

AUTOFOCUS_OFF = 0
AUTOFOCUS_ON_CHANGE = 1
AUTOFOCUS_EVERY_PHOTO = 2


def insert_focus_actions(
    waypoints: Sequence[Waypoint],
    facades: Sequence[Facade],
    mode: int,
    *,
    refocus_change: float = 0.15,
) -> list[Waypoint]:
    """Return copies of ``waypoints`` with a FOCUS action before photos.

    ``mode`` 2 focuses before every photo. ``mode`` 1 focuses only when the
    photo's facet differs from the facet at the last focus, or when the planned
    distance to its aim point differs from the distance at the last focus by
    more than ``refocus_change``. Along one wall the distance barely changes,
    so most photos reuse the focus. A photo with no known target (an unaimed
    waypoint keeps DJI's pose) always focuses. ``mode`` 0 returns the input
    unchanged.

    The focus goes directly before its ``takePhoto``, so after any gimbal
    rotate that aims that photo: the lens focuses on what the photo will see.

    Photo i at a waypoint targets ``facade_index`` for i = 0 and
    ``extra_facade_indices[i - 1]`` after that, matching the order
    ``rewrite_gimbals_perpendicular`` builds.
    """
    if mode == AUTOFOCUS_OFF:
        return list(waypoints)

    out: list[Waypoint] = []
    last_facet: int | None = None
    last_dist: float | None = None
    for wp in waypoints:
        targets = [wp.facade_index, *wp.extra_facade_indices]
        pos = np.array([wp.x, wp.y, wp.z], dtype=np.float64)
        actions: list[CameraAction] = []
        shot = 0
        for action in wp.actions:
            if action.action_type == ActionType.TAKE_PHOTO:
                facet = targets[shot] if shot < len(targets) else -1
                dist = (
                    float(np.linalg.norm(aim_point(facades[facet], pos) - pos))
                    if 0 <= facet < len(facades) else None
                )
                focus = (
                    mode == AUTOFOCUS_EVERY_PHOTO
                    or dist is None
                    or last_dist is None
                    or facet != last_facet
                    or abs(dist - last_dist) > refocus_change * last_dist
                )
                if focus:
                    actions.append(CameraAction(action_type=ActionType.FOCUS))
                    last_facet, last_dist = facet, dist
                shot += 1
            actions.append(action)
        out.append(replace(wp, actions=actions))
    return out
```

- [ ] **Step 5: Run tests**

Run: `.venv/bin/python -m pytest tests/test_focus_plan.py -v`
Expected: 8 PASS.

- [ ] **Step 6: Commit**

```bash
git add src/flight_planner/models.py src/flight_planner/focus_plan.py tests/test_focus_plan.py
git commit -m "feat: plan where the mission autofocuses before a photo"
```

---

### Task 4: Serialise `focus` and budget its time in the pass test

**Files:**
- Modify: `src/flight_planner/kmz_builder.py` — imports, `_single_shot_can_pass`, the action loop in `_build_mission`, `build_kmz_bytes` post-processing
- Modify: `src/flight_planner/models.py` — `MissionConfig` (add `af_time_s`)
- Test: `tests/test_kmz_builder.py`, `tests/test_selective_stop.py`

**Interfaces:**
- Consumes: `ActionType.FOCUS` (Task 3).
- Produces: `MissionConfig.af_time_s: float = 0.5`; `_single_shot_can_pass` unchanged signature, now budgets `min_action_dwell_s + af_time_s` for waypoints with a FOCUS action.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_kmz_builder.py`:

```python
def _focus_wpml(wps, config=None) -> str:
    data = build_kmz_bytes(wps, config or MissionConfig())
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        return zf.read("wpmz/waylines.wpml").decode("utf-8")


def test_focus_action_serialises_with_djis_fields_in_order():
    wps = _make_test_waypoints()
    for wp in wps:
        wp.actions = [CameraAction(action_type=ActionType.FOCUS), CameraAction(action_type=ActionType.TAKE_PHOTO)]
    xml = _focus_wpml(wps)
    assert xml.count("<wpml:actionActuatorFunc>focus</wpml:actionActuatorFunc>") == len(wps)
    for tag, val in (("isPointFocus", "0"), ("focusX", "0.4"), ("focusY", "0.4"),
                     ("focusRegionWidth", "0.2"), ("focusRegionHeight", "0.2"),
                     ("isInfiniteFocus", "0"), ("isCalibrationFocus", "0")):
        assert f"<wpml:{tag}>{val}</wpml:{tag}>" in xml, tag
    first = xml.split("<Placemark>")[1]
    assert first.index(">gimbalRotate<") < first.index(">focus<") < first.index(">takePhoto<")


def test_dedupe_leaves_focus_actions_alone():
    wps = _make_test_waypoints()          # identical gimbal pose on every WP: dedupe strips rotates
    for wp in wps:
        wp.actions = [CameraAction(action_type=ActionType.FOCUS), CameraAction(action_type=ActionType.TAKE_PHOTO)]
    xml = _focus_wpml(wps)
    assert xml.count(">focus<") == len(wps)
```

Check the file's imports: add `CameraAction` and `ActionType` from `flight_planner.models` if they aren't imported yet.

Append to `tests/test_selective_stop.py`:

```python
from flight_planner.models import ActionType, CameraAction


def test_focus_time_is_added_to_the_pass_budget():
    wps = _row(spacing=3.0, speed=2.0)          # 1.5 s legs
    for wp in wps:
        wp.actions = [CameraAction(action_type=ActionType.FOCUS), CameraAction(action_type=ActionType.TAKE_PHOTO)]
    cfg = MissionConfig(stop_at_waypoint=True, min_action_dwell_s=1.0, af_time_s=0.6)
    assert _single_shot_can_pass(wps, cfg) == [False] * 4
    cfg = MissionConfig(stop_at_waypoint=True, min_action_dwell_s=1.0, af_time_s=0.4)
    assert _single_shot_can_pass(wps, cfg) == [True] * 4
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_kmz_builder.py tests/test_selective_stop.py -k "focus" -v`
Expected: FAIL (`MissionConfig` has no `af_time_s`; no `focus` in the XML).

- [ ] **Step 3: Add `af_time_s` to `MissionConfig`**

In `src/flight_planner/models.py`, directly after `min_action_dwell_s: float = 1.0` and its comment block:

```python
    # Time one WPML `focus` (one-shot autofocus) takes before its photo. Not
    # documented by DJI; 0.5 s is a first guess that the first flight with
    # autofocus measures. A waypoint that focuses needs min_action_dwell_s +
    # af_time_s on its legs to fly through; otherwise it keeps its stop.
    af_time_s: float = 0.5
```

- [ ] **Step 4: Budget focus time in `_single_shot_can_pass`**

In `src/flight_planner/kmz_builder.py`, in `_single_shot_can_pass`, inside `for i, wp in enumerate(waypoints):` after the `continue` for transitions/extras, add:

```python
        need = config.min_action_dwell_s + (
            config.af_time_s
            if any(a.action_type == ActionType.FOCUS for a in wp.actions) else 0.0
        )
```

and change `if t < config.min_action_dwell_s:` to `if t < need:`. Add one line to the docstring's list: "- a waypoint that autofocuses also needs ``af_time_s`` on those legs."

- [ ] **Step 5: Emit the action**

Add to the imports near the other djikmz imports:

```python
from djikmz.model.action.camera_actions import FocusAction
```

In `_build_mission`'s action loop, add a branch after the `HOVER` branch:

```python
            elif action.action_type == ActionType.FOCUS:
                # One-shot area autofocus on the centre fifth of the frame,
                # after the gimbal has aimed and before the photo.
                wb._actions.append(FocusAction(
                    action_id=0,
                    is_point_focus=0,
                    focus_x=0.4,
                    focus_y=0.4,
                    focus_region_width=0.2,
                    focus_region_height=0.2,
                    is_infinite_focus=0,
                ))
```

- [ ] **Step 6: Add `isCalibrationFocus`**

djikmz does not emit it; DJI's own `focus` actions carry `isCalibrationFocus=0`. Add this function next to `_inject_m4e_enums`:

```python
def _inject_focus_calibration_flag(root: ET.Element) -> None:
    """Add ``isCalibrationFocus=0`` to every ``focus`` action.

    DJI's own focus actions (every Smart3D startActionGroup) carry it; djikmz's
    FocusAction does not. Emit what DJI emits.
    """
    for action in root.iter(f"{{{_WPML_NS}}}action"):
        func = action.find(f"{{{_WPML_NS}}}actionActuatorFunc")
        params = action.find(f"{{{_WPML_NS}}}actionActuatorFuncParam")
        if func is None or func.text != "focus" or params is None:
            continue
        if params.find(f"{{{_WPML_NS}}}isCalibrationFocus") is None:
            ET.SubElement(params, f"{{{_WPML_NS}}}isCalibrationFocus").text = "0"
```

In `build_kmz_bytes`, call it on the template before waylines are generated, right after `_dedupe_pose_actions(template_root, config)`:

```python
    _inject_focus_calibration_flag(template_root)
```

- [ ] **Step 7: Run tests**

Run: `.venv/bin/python -m pytest -q -p no:warnings`
Expected: all PASS. If `test_focus_action_serialises_with_djis_fields_in_order` fails on `focusX>0.4`, print the XML and check how djikmz formats floats; match what it emits in the assertion only if the value is still 0.4.

- [ ] **Step 8: Commit**

```bash
git add src/flight_planner/models.py src/flight_planner/kmz_builder.py tests/test_kmz_builder.py tests/test_selective_stop.py
git commit -m "feat: emit WPML focus actions and budget their time before flying through"
```

---

### Task 5: Settings, calm nose, and wiring into the augment

**Files:**
- Modify: `src/flight_planner/mission_intent.py` — `SETTING_KEYS`
- Modify: `src/flight_planner/cli.py` — `augment_mission` signature, the `rewrite_gimbals_perpendicular` call, after the action-normalising loop, `MissionConfig(...)`, summary, `main` wiring (~line 823)
- Test: `tests/test_cli.py`, `tests/test_gimbal_rewrite.py`

**Interfaces:**
- Consumes: `insert_focus_actions` (Task 3), `MissionConfig.af_time_s`, `_single_shot_can_pass` (Task 4).
- Produces: `augment_mission(..., autofocus: int = 2, af_time_s: float = 0.5, smooth_heading: bool = True)`; summary keys `focus_actions`, `stops`, `pass_throughs`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_cli.py`:

```python
def test_new_settings_are_accepted_and_clamped():
    from flight_planner.mission_intent import coerce_settings

    s = coerce_settings({"autofocus": 5, "af_time_s": 0.8, "smooth_heading": 0})
    assert s == {"autofocus": 2, "af_time_s": 0.8, "smooth_heading": 0}


def test_augment_mission_defaults_to_autofocus_every_photo_and_calm_nose():
    import inspect

    from flight_planner.cli import augment_mission

    p = inspect.signature(augment_mission).parameters
    assert p["autofocus"].default == 2
    assert p["af_time_s"].default == 0.5
    assert p["smooth_heading"].default is True
```

Append to `tests/test_gimbal_rewrite.py`:

```python
def test_calm_nose_keeps_gimbal_pan_inside_50_degrees_and_commands_yaw():
    # Walls on both sides of a straight pass: the target flips 180° mid-row.
    east = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), size=30.0)
    west = _make_facade(normal=(-1.0, 0.0, 0.0), center=(12.0, 0.0, 2.0), size=30.0)
    wps = [Waypoint(x=4.0 if i < 5 else 8.0, y=float(i), z=2.0, speed_ms=1.0) for i in range(10)]
    out = rewrite_gimbals_perpendicular(wps, [east, west], preserve_heading=False, command_gimbal_yaw=True)
    for w in out:
        assert w.gimbal_yaw_deg is not None
        pan = (w.gimbal_yaw_deg - w.heading_deg + 180.0) % 360.0 - 180.0
        assert abs(pan) <= 50.0 + 1e-6


def test_smooth_heading_off_commands_no_gimbal_yaw():
    wall = _make_facade(normal=(1.0, 0.0, 0.0), center=(0.0, 0.0, 2.0), size=10.0)
    out = rewrite_gimbals_perpendicular([Waypoint(x=5.0, y=0.0, z=2.0)], [wall], preserve_heading=False)
    assert out[0].gimbal_yaw_deg is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_cli.py tests/test_gimbal_rewrite.py -k "settings or defaults_to_autofocus or calm_nose or smooth_heading" -v`
Expected: the two `test_cli.py` tests FAIL. The two gimbal tests may already PASS (they pin existing behaviour); that is expected.

- [ ] **Step 3: Add the settings**

In `src/flight_planner/mission_intent.py`, `SETTING_KEYS`, after `"gimbal_pan_window_deg"`:

```python
    # Autofocus before photos: 0 off (manual focus set once at start, the
    # behaviour flown up to 2026-09-23), 1 when the facet or the distance
    # changes by >15%, 2 before every photo. The client asked for autofocus.
    "autofocus": (int, 0, 2),
    # Time budget for one autofocus; unmeasured, so a knob.
    "af_time_s": (float, 0.0, 3.0),
    # 1: the nose turns at a bounded rate and the gimbal yaw does the fine aim
    # (schedule_headings, pan within ±50°). 0: the nose points at each
    # waypoint's target, as flown on 2026-09-23 ("a bit weird").
    "smooth_heading": (int, 0, 1),
```

- [ ] **Step 4: Wire `augment_mission`**

In `src/flight_planner/cli.py`:

1. Add to the `augment_mission` signature after `pan_window_deg: float = 45.0,`:

```python
    autofocus: int = 2,
    af_time_s: float = 0.5,
    smooth_heading: bool = True,
```

2. In the `rewrite_gimbals_perpendicular(...)` call, after `preserve_heading=False,` add:

```python
        # Calm nose: the heading chases each target no faster than half the
        # yaw rate and the gimbal yaw (absolute from north) does the fine aim,
        # pan capped at ±50°. Disabled in July on the claim that the M4E
        # ignores gimbal yaw; the 2026-09-07 XMP retracted that (0.1° median
        # over 256 frames).
        command_gimbal_yaw=smooth_heading,
```

3. Directly after the loop that sets `w.speed_ms = inspection_speed_ms` and single-photo actions, add:

```python
    new_waypoints = insert_focus_actions(new_waypoints, facades, autofocus)
```

and add `from .focus_plan import insert_focus_actions` to the module imports.

4. In `MissionConfig(...)`, after `stop_at_waypoint=stop_at_waypoint,` add `af_time_s=af_time_s,`.

5. After the `validate_mission` loop, add:

```python
    passable = _single_shot_can_pass(new_waypoints, config)
    photo_wps = [i for i, w in enumerate(new_waypoints) if not w.is_transition]
    stops = sum(1 for i in photo_wps if stop_at_waypoint and not passable[i])
    focus_actions = sum(1 for w in new_waypoints for a in w.actions if a.action_type == ActionType.FOCUS)
    _log(f"      stops {stops}   pass-throughs {len(photo_wps) - stops}   "
         f"focus actions {focus_actions} (autofocus mode {autofocus}, {af_time_s:.1f} s each)   "
         f"calm nose {'on' if smooth_heading else 'off'}")
```

and add `_single_shot_can_pass` to the existing `kmz_builder` import (or add `from .kmz_builder import _single_shot_can_pass`).

6. In the `summary` dict, after `"stop_at_waypoint": stop_at_waypoint,` add:

```python
        "stops": stops,
        "pass_throughs": len(photo_wps) - stops,
        "focus_actions": focus_actions,
        "autofocus": autofocus,
        "smooth_heading": bool(smooth_heading),
```

7. In `main`, in the `augment_mission(...)` call built from `st`, after `pan_window_deg=...,` add:

```python
            autofocus=int(st.get("autofocus", 2)),
            af_time_s=float(st.get("af_time_s", 0.5)),
            smooth_heading=bool(st.get("smooth_heading", 1)),
```

- [ ] **Step 5: Run the full suite**

Run: `.venv/bin/python -m pytest -q -p no:warnings`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add src/flight_planner/mission_intent.py src/flight_planner/cli.py tests/test_cli.py tests/test_gimbal_rewrite.py
git commit -m "feat: calm nose and autofocus in the augment, set from the mission intent"
```

---

### Task 6: Bench run on Houten-3 and hand-off

**Files:**
- Create: `docs/flights/2026-09-23-bench/BENCH.md`

- [ ] **Step 1: Augment Houten-3 with the new code**

```bash
mkdir -p /tmp/aeroscan-bench
.venv/bin/python -m flight_planner.cli augment-mission \
  --mission-json flight-archive/2026-09-07/missions/20260907T133135Z_705/intent.json \
  --icp-target-ply flight-archive/2026-09-07/missions/20260907T133135Z_705/cloud.ply \
  --blackbox-dir flight-archive/2026-09-07/blackbox --flight-id flight0082 \
  --output-kmz /tmp/aeroscan-bench/new.kmz --summary-json /tmp/aeroscan-bench/new.summary.json
```

Expected: the log ends with `Total:` and a `stops … pass-throughs … focus actions …` line. If `--icp-target-ply` is rejected, run `.venv/bin/python -m flight_planner.cli augment-mission -h` and use the flag it lists for the ICP target.

- [ ] **Step 2: Compare against flight C**

```bash
.venv/bin/python - <<'EOF'
import json, zipfile, re, statistics
new = json.load(open("/tmp/aeroscan-bench/new.summary.json"))
old = json.load(open("flight-archive/2026-09-07/augment/20260907T133135Z_705.summary.json"))
for k in ("waypoints_total", "photos", "stops", "pass_throughs", "focus_actions"):
    print(k, old.get(k), "->", new.get(k))
print("walls_targeted", old["coverage"]["walls_targeted"], "->", new["coverage"]["walls_targeted"])
def steps(path):
    x = zipfile.ZipFile(path).read("wpmz/waylines.wpml").decode()
    h = [float(v) for v in re.findall(r"<wpml:waypointHeadingAngle>([-\d.]+)<", x)]
    d = sorted(abs((b - a + 180) % 360 - 180) for a, b in zip(h, h[1:]))
    return d[int(len(d) * 0.9)] if d else None
print("heading step p90", steps("flight-archive/2026-09-07/augment/20260907T133135Z_705.augmented.lean.kmz"),
      "->", steps("/tmp/aeroscan-bench/new.lean.kmz"))
EOF
```

Pass criteria (spec, Testing): `walls_targeted` ≥ 71, heading step p90 below C's, `focus_actions` ≈ `photos` in mode 2, no ERROR-severity validation in the new summary. If `walls_targeted` < 71, stop and report: do not deploy.

- [ ] **Step 3: Record the bench result**

Write `docs/flights/2026-09-23-bench/BENCH.md` with the numbers printed in Step 2 (a short table, old vs new), the command used, and one line per spec risk saying it is still unflown. Commit:

```bash
git add docs/flights/2026-09-23-bench/BENCH.md
git commit -m "docs: bench the calm nose and autofocus against Houten flight C"
```

- [ ] **Step 4: Push**

```bash
git push origin main
```

Deploying to the Manifold is a separate, pilot-attended step (`scripts/deploy_to_manifold.sh --host=<ip>` on the site router). It is not part of this plan.
