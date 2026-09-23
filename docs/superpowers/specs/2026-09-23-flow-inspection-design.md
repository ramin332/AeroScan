# Calm nose, autofocus, nearest-point picker — design

Date: 2026-09-23 · Status: approved in conversation · Flown: no

## Goal

One test flight that gives **sharper, better-aimed photos** with a **calm aircraft
nose**, without losing coverage or photos.

Baseline (Houten-3, 2026-09-07, flight C: 2 m/s, stop at every waypoint, 2 shots):
4.6 s per waypoint, 539/539 photos, 71 of 162 walls ≥ 2 m² covered, primary shots
13.6° median off target, lens locked near infinity (`LensPosition` 29–30, infinity
27–28) at a median 7.3 m standoff.

The 2026-09-23 on-site fix (`570427b`) already aims at the nearest point of a facet and
lets single-shot waypoints fly through when their legs allow it. The pilot reported the
aim better and the nose "a bit weird". This design builds on `570427b`.

## Scope

In:
- **Autofocus** — the client wants it. Default: before every photo.
- **Calm nose (idea 5)** — the nose turns at a bounded rate; the gimbal does the fine aim.
- **Picker by nearest point (idea 3)** — the facet choice uses the same point the camera
  aims at.

Out (decided 2026-09-23): splitting two-shot waypoints into two fly-through waypoints,
and the per-leg speed that splitting needed. Two-shot waypoints keep stopping.

## Changes

### 1. Picker by nearest point (`gimbal_rewrite.py`)

`assign_facades_viterbi` scores each facet by the distance from the waypoint to
`aim_point(facet, wp)` instead of to `facet.center`. The outward-side test, the reach
cap, the pitch penalty and the bearing used for the switch cost all use that point.
`_pick_facade_for_waypoint` (greedy mode) and `assign_extra_shots` get the same change,
so every mode agrees with the aim. The switch cost and plane-group logic are unchanged.

Why: a long wall's centroid is far from a waypoint at one end of it, so the wall lost
to small facets nearby even though the camera would aim at the wall right in front.

### 2. Calm nose (`gimbal_rewrite.py`, `cli.py`)

`augment_mission` passes `command_gimbal_yaw=True`. That path already exists:
`schedule_headings()` makes the heading a rate-limited pursuit of each waypoint's aim
bearing (`rate_fraction` 0.5 of 60°/s) and caps the gimbal pan at ±50°, and every
waypoint commands an absolute gimbal yaw. The 2026-09-07 XMP proved the M4E follows
that yaw: median 0.1° over 256 frames. The July "gimbal ignores yaw" conclusion that
disabled this path is retracted.

The explicit `gimbalRotate` before the primary photo (`6e980b7`) stays.

Side effect: a calmer nose means smaller heading steps, so more single-shot waypoints
pass the `_single_shot_can_pass` heading test and fly through.

### 3. Autofocus (`models.py`, `gimbal_rewrite.py`/`cli.py`, `kmz_builder.py`)

New `ActionType.FOCUS`, emitted as a WPML `focus` action: area focus in the centre of
the frame (`isPointFocus=0`, `focusX=focusY=0.4`, `focusRegionWidth=focusRegionHeight=0.2`,
`isInfiniteFocus=0`). It goes after the gimbal command and before each `takePhoto` it
serves. The `startActionGroup` keeps `setFocusType manual`; `focus` is a one-shot
autofocus in that mode, as in DJI's own start group.

Setting `autofocus`:
- `2` = before every photo (default for this test flight)
- `1` = only when the facet changes, or when the planned distance to the aim point
  differs by more than 15% from the distance at the last focus
- `0` = off (the behaviour flown so far: manual focus, never refocused)

Time budget: a waypoint with a focus action needs `min_action_dwell_s + af_time_s` on
its legs to fly through. `_single_shot_can_pass` uses that sum; waypoints that cannot
meet it stop, as today. So autofocus can cost fly-throughs; it never costs photos.

## Settings (mission intent `settings`, `mission_intent.SETTING_KEYS`)

Numeric keys with engine defaults. No RC rebuild is needed for the test flight.

| key | type, range | default | meaning |
|---|---|---|---|
| `autofocus` | int 0–2 | 2 | 0 off, 1 on change, 2 every photo |
| `af_time_s` | float 0–3 | 0.5 | time budget for one autofocus; the flight measures it |
| `smooth_heading` | int 0–1 | 1 | 0 = nose points at the target at every waypoint (flown 2026-09-23) |

## Summary and validation

- Summary gains `focus_actions`, `stops`, `pass_throughs` counts, logged on one line.
- No new validation rule. The existing fly-through gates stay as they are.

## Risks the flight has to answer

- **Whether the FC accepts `focus` at an inspection waypoint.** DJI only uses it in
  `startActionGroup`. If the mission-validity check rejects it, START fails before
  takeoff; fall back to `autofocus=0` (needs a redeploy until the panel has a toggle).
- **How long one autofocus takes on the M4E.** Undocumented; 0.5 s is a guess. Measure
  from photo timestamps.
- **Gimbal-motor overload.** Every waypoint now commands gimbal yaw. 2026-06-12
  tripped HMS at ~3.9 commands/s; the 5° dedupe stays. Watch HMS and the journal.
- **Autofocus hunting on a plain wall.** Area focus on a featureless surface can miss.
  `LensPosition` against the laser distance will show it.

## Testing

- Unit: Viterbi picks a long wall the waypoint faces over a small facet whose centroid
  is nearer; greedy and extra-shot assignment use the same point; `smooth_heading=1`
  keeps |pan| ≤ 50° and commands gimbal yaw on every aimed waypoint; focus placement for
  modes 0/1/2 (order gimbalRotate → focus → takePhoto, one focus per photo in mode 2,
  refocus rules in mode 1); `focus` serialises to the WPML fields above; the pass test
  adds `af_time_s` where a waypoint focuses.
- Bench: augment the Houten-3 archive (`flight-archive/2026-09-07/`) and compare with
  flight C: walls covered (≥ 71), heading step p90, focus count, stops vs pass-throughs.

## After the flight, from the photos (success criteria)

| measure | target |
|---|---|
| photos delivered | all planned |
| aim error, primary and extra | ≤ ~5° median (C's primary: 13.6°) |
| gimbal pan at the ±60° stop | 0 frames |
| heading change per waypoint | visibly calmer; p90 below C's |
| `LensPosition` | follows the laser distance, not stuck near infinity |
| walls ≥ 2 m² covered | ≥ 71 of 162 on the same house |

Pull the journal before power-down (it is volatile) and check the telemetry CSV.

## Rollback

`smooth_heading=0`, `autofocus=0` give the behaviour flown on 2026-09-23. Until the
panel has toggles, field rollback is a checkout of `570427b` + `deploy_to_manifold.sh`.
