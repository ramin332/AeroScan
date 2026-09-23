# Flow inspection: two shots without stopping, calm nose, autofocus — design

Date: 2026-09-23 · Status: approved in conversation, awaiting spec review · Flown: no

## Goal

One test flight that is **faster** than Houten flight C and gives **sharper, better-aimed
photos from different positions** (usable for photogrammetry), without losing coverage.

Baseline (Houten-3, 2026-09-07, flight C: 2 m/s, stop at every waypoint, 2 shots):
4.6 s per waypoint, 21.2 min, 539/539 photos, 71 of 162 walls ≥ 2 m² covered, lens
locked near infinity (`LensPosition` 29–30, infinity 27–28) at a median 7.3 m standoff.

The 2026-09-23 on-site fix (`570427b`) already aims at the nearest point of a facet and
lets single-shot waypoints fly through. This design builds on it.

## What the user asked for

- Autofocus — the client wants it. Default: before every photo. A setting selects
  "only when the distance changes" instead.
- Picker scores the nearest point of a facet, not its centroid (idea 3).
- Two photos per waypoint without stopping, taken from different positions (idea 4).
  Never two photos of the same place: the extra shot still only goes to a facet no other
  waypoint photographs.
- A calm aircraft nose, with the gimbal doing the fine aim (idea 5).
- All flown together as one test.

Added during design, because idea 4 does not work without it: **per-leg speed** (idea 2).

## Pipeline (augment, after registration and facade detection)

Order of steps in `rewrite_gimbals_perpendicular` / `augment_mission`:

1. **Pick (idea 3).** `assign_facades_viterbi` scores each facet by the distance from
   the waypoint to `aim_point(facet, wp)` instead of to `facet.center`. The validity
   test (waypoint on the outward side) and the pitch penalty use the same point. The
   switch cost and the plane-group logic do not change. The greedy picker
   (`_pick_facade_for_waypoint`) gets the same change, so both modes agree.
2. **Assign extra shots.** `assign_extra_shots`, unchanged in rule: only facets no other
   waypoint photographs, each given out once. Its distance and pan tests move to
   `aim_point` too.
3. **Split (idea 4).** For each waypoint that has an extra shot, insert a new waypoint
   on the segment to the next waypoint, at the midpoint. The original waypoint keeps
   the primary shot; the new one takes the extra shot. The new waypoint carries
   `facade_index` = the extra facet and an `is_split` flag. With `shots_per_waypoint`
   = 3 or 4, the extras are spread evenly along the segment (at 1/3 and 2/3, and so
   on). The last waypoint of the mission has no next segment: it keeps its extras as a
   stop-and-shoot chain, as today.
   - Safety: the new waypoint lies on a segment the aircraft flies anyway. It adds no
     new airspace, and the polygon clip and point-cloud obstacle filter still run on
     the result.
4. **Aim.** Every photo waypoint aims at `aim_point` of its facet: pitch and absolute
   yaw from north, pitch clamped to the gimbal limits minus the margin.
5. **Heading (idea 5).** `schedule_headings()` over all waypoints (originals and
   splits) with each waypoint's aim bearing as the target: `rate_fraction` 0.5 of 60°/s,
   gimbal pan capped at ±50°. The gimbal yaw is commanded (`command_gimbal_yaw=True`).
   The 2026-09-07 XMP proved the M4E follows it: median 0.1° over 256 frames. The
   primary-shot `gimbalRotate` added in `6e980b7` becomes the normal per-waypoint
   gimbal command.
6. **Autofocus.** A `focus` action (`isPointFocus=0`, centre area: `focusX=focusY=0.4`,
   width and height 0.2, `isInfiniteFocus=0`) placed after `gimbalRotate` and before
   `takePhoto`. Setting `autofocus`:
   - `2` = before every photo (default for this test flight)
   - `1` = only when the facet changes, or when the planned distance to the aim point
     differs by more than 15% from the distance at the last focus
   - `0` = off (today's behaviour: manual focus, never refocused)

   The `startActionGroup` keeps `setFocusType manual`. The `focus` action is a
   one-shot autofocus in that mode, as in DJI's own start group.
7. **Per-leg speed (idea 2).** For each leg into a photo waypoint:
   `speed = clamp(leg_m / need_s, 0.3, inspection_speed_ms)` where
   `need_s = min_action_dwell_s + (af_time_s if that waypoint focuses else 0)`.
   The outgoing leg is checked the same way, so the slower of the two legs sets the
   waypoint's `speed_ms`. Transit waypoints keep `inspection_speed_ms`.
8. **Turn mode.** Every photo waypoint flies through (`toPointAndPassWithContinuityCurvature`).
   A waypoint keeps `curve_and_stop` only if its time need cannot be met even at
   0.3 m/s (leg shorter than `0.3 × need_s`), if the heading schedule had to yaw
   faster than the rate budget on an adjoining leg (a counted violation in
   `schedule_headings`), or if it still carries more than one photo (the last-waypoint
   case in step 3). `_single_shot_can_pass` from `570427b` is
   generalised to this rule.

## Settings (mission intent `settings`, `mission_intent.SETTING_KEYS`)

New numeric keys, with engine-side defaults. No RC rebuild is needed for the test
flight; panel toggles come later.

| key | type, range | default | meaning |
|---|---|---|---|
| `autofocus` | int 0–2 | 2 | 0 off, 1 on change, 2 every photo |
| `af_time_s` | float 0–3 | 0.5 | time budget for one autofocus; measured on the test flight |
| `split_extra_shots` | int 0–1 | 1 | 0 = the stop-for-extra-shots behaviour flown on 2026-09-23 |
| `smooth_heading` | int 0–1 | 1 | 0 = nose points at the target at every waypoint (today) |

`stop_at_waypoint` keeps its meaning of "stopping is allowed". With it off (fly-through),
`shots_per_waypoint` is still forced to 1, as today.

## Validation

- `extra_shots_need_stop` stays. After splitting, no pass-through waypoint carries two
  photos, so the rule is satisfied by construction.
- The fly-through gates (`action_dwell_too_short`, `heading_step_unreachable`) run on
  the final waypoints whenever any waypoint passes, not only when
  `stop_at_waypoint` is off. Per-leg speed means they should not fire. If they do, the
  test fails.
- New info line in the summary: counts of split waypoints, stops kept, focus actions,
  and the estimated mission time.

## Risks the flight has to answer

- **How long one autofocus takes on the M4E.** Undocumented. 0.5 s is a guess.
  Measure it from photo timestamps against planned leg times.
- **Whether the FC accepts `focus` at an inspection waypoint.** DJI only uses it in
  `startActionGroup`. If the mission-validity check rejects it, START fails. The
  pilot sees that before takeoff, and we fall back to `autofocus=0`.
- **Gimbal-motor overload.** There will be more gimbal commands than on 2026-09-07.
  2026-06-12 tripped the HMS warning at ~3.9 commands/s. The 5° dedupe stays. Watch
  HMS and the journal.
- **Split leg speed.** At 0.3–0.75 m/s, split legs are slow. The per-waypoint
  estimate is ~2 s against 4.6 s for a stop. If it comes out worse, the planned-time
  estimate will say so before flight.

## Testing

- Unit tests per step: nearest-point Viterbi prefers a long wall the waypoint faces
  over a small off-axis facet; split waypoint on the segment, midpoint, carries the
  extra facet; heading schedule keeps |pan| ≤ 50°; focus placement for modes 0/1/2;
  per-leg speed respects `need_s` and the 0.3 m/s floor; turn mode falls back to stop
  only in the three cases of step 8.
- Bench run on the Houten-3 archive (`flight-archive/2026-09-07/`) against flight C:
  waypoint count, split count, stops kept, focus count, estimated time, walls covered.
  The spec is met on the bench if estimated time < C's and walls covered ≥ C's 71.

## After the flight, from the photos (success criteria)

| measure | target |
|---|---|
| photos delivered | all planned (as 276/276 and 539/539 at Houten) |
| seconds per original waypoint | < 4.6 s (C) |
| aim error, primary and extra | ≤ ~5° median (C's primary: 13.6°) |
| gimbal pan at the ±60° stop | 0 frames |
| `LensPosition` | follows the laser distance (not stuck near infinity) |
| walls ≥ 2 m² covered | ≥ 71 of 162 on the same house |

Also pull the journal before power-down (it is volatile) and check the telemetry CSV.

## Rollback

`split_extra_shots=0`, `smooth_heading=0`, `autofocus=0` give the behaviour flown on
2026-09-23. The pilot cannot set these from the RC until the panel has the toggles.
In the field, rollback is `git stash` or a checkout of `570427b` + `deploy_to_manifold.sh`.
