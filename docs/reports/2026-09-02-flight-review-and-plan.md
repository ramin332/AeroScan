# Flight review 2026-07-10 and plan — written 2026-09-02

Two audiences, one document. **Part I** is for direction: what happened, what it
cost, what is fixed, what is still unproven, and the decision we ask for.
**Part II** is for engineers: evidence, file references, the corrections we made
to our own earlier claims, the six changes, the aim-picker diagnosis, and the
exact protocol for the next flight.

Companion visuals:

- Interactive before/after page: <https://claude.ai/code/artifact/40fbd599-65f5-48b2-8894-02db752a576a>
- Aim-picker picture over the real scan: [`2026-09-02-picker-picks.png`](./2026-09-02-picker-picks.png)
- Full flight-review report page (this document with the figures embedded): published alongside this file as the "Busboom Flight Review" artifact.

Source data: `flight-archive/2026-07-10/` (all 2026-07-10 `/blackbox` slots),
`flight-archive/2026-09-02-manifold-pull/` (mission intents, summary cards),
the aircraft itself (`dji@192.168.1.118`, verified byte-identical to the archive).

---

## Part I — For direction

### What we set out to prove

AeroScan takes DJI's own automatic scan of a target, works out the surfaces
worth photographing, and re-aims the drone's camera at them, one sharp
head-on photo per stop, for NEN-2767 building inspection. On 2026-07-10 we
flew five missions around a van and a tree on a parking lot (no building was
available) to test that chain end to end.

### What we learned — in five lines

1. **The drone flew the right path but planned against the wrong scan, every time.**
   Two scans were made that morning; the software only looked in one folder and
   used the older, smaller one. On identical waypoints that meant **9 surfaces
   instead of 181** to aim at. One-line bug, big effect.
2. **26% of the photos we asked for were never taken** — 294 of 398. The aircraft
   flew through each waypoint at ~3 m/s with ~0.6 s to rotate the camera and fire.
3. **The camera did not point where we told it.** From the photos' own metadata:
   the gimbal sat 44° off its command and hit its mechanical stop on 14% of shots.
   The fix was written that evening but has not flown.
4. **Our own dashboard said "Aim 100%"** while the real pointing error was 35°.
   The metric measured "is the target somewhere in the wide-angle frame" — which
   is almost always true and almost never useful.
5. **We had no instrument on board.** The only record of what the camera did was
   the SD card. That is why the questions above took until evening to answer and
   why the card is still the sole evidence.

### What is fixed (code done, tested offline, not yet flown)

| | Fix | Effect expected |
|---|---|---|
| ✅ | Look in every scan folder, take the newest | Plans use the scan you just made |
| ✅ | **Stop at every waypoint** before shooting (DJI's own documented mode) | 398 photos, sharper, camera settled — at ~4× the flight time (≈20 min vs 5) |
| ✅ | Warn at plan time when photos will be skipped, when the aircraft can't turn fast enough, or when resolution misses the target | Problems visible on the pilot's screen *before* take-off |
| ✅ | Replace "Aim 100%" with the real resolution number and a warning count | Honest pre-flight card |
| ✅ | Record camera and aircraft angles on board, 10× per second | Every flight becomes self-verifying, no SD card needed |
| ⏳ | Camera-pointing fix (`2bf3308`) | Written 2026-07-10, **unflown** |

### What remains unproven

Everything above is verified on the ground against the real scan data, not in
the air. The things only a flight can answer: does the camera now follow the
nose; does stopping 398 times on 1.7 m legs behave well; is the 20-minute
estimate right (battery limit with the Manifold is 32 min — modelled at 62%).

### What it will cost

- Flight time per mission: ~5 min → **~20 min** (modelled). Still inside one battery.
- One more test flight, flown as a *measurement* flight: photos judged from
  metadata and the on-board log, not the live video feed.

### The decision we ask for

1. Approve installing the rebuilt on-drone package (built, not installed) and
   committing the code (done, not committed).
2. Approve the next test flight with the protocol in Part II §7, accepting the
   longer flight time.
3. Approve the aim-picker improvement (Part II §5) — the current picker sometimes
   aims at a surface 20–30 m away or flips between surfaces; a plan exists and is
   measurable offline before it flies.
4. **Find the 2026-07-10 SD card** and archive its photos. They are the only
   evidence of the camera-pointing bug and are not backed up.

---

## Part II — For engineers

### 1. Data provenance

| Source | Where | Verified |
|---|---|---|
| Perception mesh, flight0072 (13 chunks, written 11:48–11:53) | `flight-archive/2026-07-10/flight0072/dji_perception/2/` and `/blackbox/flight0072/dji_perception/2/` on the aircraft | md5 of chunks 0 and 12 identical on both sides |
| PSDK runtime log of the 12:36 flight | `flight-archive/2026-07-10/flight0073/psdk/PSDK-0073-01.log` (2446 lines) | mission-state + action-state callbacks parsed |
| Five mission intents + summary cards | `flight-archive/2026-09-02-manifold-pull/{missions,received}/` | pulled 2026-09-02, includes the fifth mission `20260710T111848Z_436` that was missing locally |
| Flown KMZ | `.../received/20260710T103507Z_331.augmented.lean.kmz` | 398 WP, 398 `takePhoto`, 207 `gimbalRotate`, `waypointSpeed=3.0`, all `toPointAndPassWithContinuityCurvature` |
| Photos (294) | SD card `DCIM/DJI_202607101133_009` | **not archived; card not located on 2026-09-02** |

### 2. Findings with evidence

**2.1 Every mission used the wrong scan.** All five summary cards read
`-> flight0070`. Chunk-file mtimes on the aircraft: `flight0070/1` written
2026-07-10 10:18–10:21; `flight0072/2` written 11:48–11:53. Augments at 11:56,
12:35 and 13:18 all resolved flight0070 because `kmzrun_status.c:37,95` globbed
`dji_perception/1/` only. Cost, same 398 waypoints: import path **9 vs 181
facades**; augment path **53 flown vs 221** in the offline re-plan.

**2.2 Photo shortfall.** 398 commanded, 294 on the card. The mission flew at
2.94 m/s (commanded 3.0, `frontend/src/store.ts:42` NEN preset) over a 1.74 m
median leg = **0.58 s per waypoint**; waypoints with `gimbalRotate+takePhoto`
dwelt only 0.12 s longer (0.70 s). Turn mode was pass-through on all 398
waypoints; WPML documents `toPointAndPassWithContinuityCurvature` as "the
aircraft will not stop at the point."

**2.3 Gimbal yaw not honoured** (from the 2026-07-10 evening XMP analysis,
`docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md`): actual pan median
51.5°, p90 61.0° (pinned at the ±60° stop), 44° from the commanded yaw; bearing
error 35.4° median vs 17.6° commanded. Fix `2bf3308` (no yaw command; gimbal
follows the nose) is unflown.

**2.4 `Aim 100%` on all five cards** while measured bearing error was 35.4°.
`cli.py` computed it as "waypoint has a facade assigned".

**2.5 No telemetry.** `kmz_runner.c:1180` subscribed only
`DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT`. `GIMBAL_ANGLES`, `QUATERNION`,
`POSITION_FUSED` were available and unused.

**2.6 Heading reachability.** Commanded heading steps 7.5° median, 28° p90,
82° max at 0.6 s per leg; 18 waypoints needed a turn the airframe cannot finish
in the leg at 60°/s. The measured 14.4° median heading error is consistent.
`schedule_headings()` (`gimbal_rewrite.py:299`) only ran when
`command_gimbal_yaw=True`, i.e. never on the path that now flies.

### 3. Corrections to our own earlier claims — read these

Two statements made during this review (and one in ANALYSIS.md open item 3)
were **wrong**. They are retracted in the docs; recorded here so nobody
re-derives them.

| Claim | Why it was wrong | What is true |
|---|---|---|
| "The mesh was 15 h old / from the previous evening" | Read from the **slot directory** mtime (`flight0070` dir: 2026-07-09 20:27). Slot dir mtimes lie — DJI's bookkeeping touches them. | Chunk-file mtimes: 10:18–10:21 the same morning. The staleness gate logged 253 → 8041 s across the day, all correct. The 6 h gate did its job; only the subdir hardcode was at fault. |
| "104 unfinished actions = 104 missing photos, exact match" | Global totals (837 starts − 733 completions) coincided with 398 − 294. Per waypoint, **251 of 398** have an action with no completion callback, and that share is flat vs leg time (63% on legs < 0.5 s, 69% on legs ≥ 1.0 s). | The FC's action-completion callbacks are an unreliable record (like the RESUME-index artifact). The 104 shortfall is real; the dwell mechanism is the leading hypothesis, not a measurement. Stop mode removes it either way. |
| "This mission shot 5.01 mm/px" | Taken from the import path's nominal camera distance (18.34 m), not per-waypoint standoff. | Offline re-plan on the fresh mesh: median standoff gives **1.97 mm/px**, in spec. The 399-WP mission's 9.23 mm/px at 33.8 m stands. |

### 4. The six changes (laptop repo uncommitted; Manifold repo uncommitted; DPK built, not installed)

| # | Change | Files | Proof |
|---|---|---|---|
| 1 | Flight telemetry: `GIMBAL_ANGLES` (10 Hz, `x=pitch y=roll z=yaw`, deg), `QUATERNION` (10 Hz, raw), `POSITION_FUSED` (5 Hz) → `/open_app/dev/data/received/telemetry/<UTC>.csv`, one row per gimbal sample while a mission is active, closed on IDLE | `aeroscan-psdk/src/manifold3_app/kmz_runner.c` (+168 lines) | `gcc -fsyntax-only -Wall -Wextra` 0 warnings; full `make` clean; `build/dpk/psdk-demo_v01.00.00.00.dpk` rebuilt 2026-09-02 12:08 |
| 2 | Mesh resolver globs `dji_perception/*/` | `kmzrun_status.c:37,95`, `test/test_status.c` | native test `ALL PASS`, including a `/2/`-only slot that wins on mtime |
| 3+4 | Augment defaults to `stop_at_waypoint=True` → WPML `toPointAndStopWithContinuityCurvature` on every inspection WP; `--fly-through` opts out. Fly-through mode gets `action_dwell_too_short` (knob `min_action_dwell_s`, default 1.0 s, models → API → sidebar) and `heading_step_unreachable` warnings | `cli.py`, `validate.py`, `models.py`, `server/api.py`, `frontend/src/{api/types.ts,store.ts,components/Sidebar.tsx}` | 192/192 pytest; 25/25 in the Manifold env after deploy; frontend build clean |
| 5 | `gsd_out_of_spec` warning (25% over target, names the lens that meets it at the same standoff); `validate_mission` now runs on the augment path | `validate.py`, `cli.py` | `tests/test_validate_plan_gates.py` (11 tests) |
| 6 | Pilot 2 card: `Aim %` → `GSD x.x mm/px` + `Stop@WP on/OFF  warn N`; summary JSON carries the issue list | `cli.py` | card tests, ≤ 255 bytes |

Design note on 3+4: `schedule_headings()` stays off on the no-yaw path on
purpose. With no gimbal yaw to absorb residual pan, rate-limiting the heading
would point the camera *away* from the target. Stopping lets the turn complete
instead.

Offline before/after on the real waypoints + fresh mesh (see the artifact):
legs under 1.0 s **312 → 0**; unreachable turns **18 → 0**; execution warnings
**2 → 0**; three unrelated warnings remain (`photo_interval_too_short`,
`too_close_to_surface`, `facades_uncovered`); flight time 295 s actual (model
277 s) → **≈1185 s modelled** (2 m/s² accel + 1.0 s dwell; the planner's own
11.7-min estimate ignores accel and is optimistic); 62% of the 32-min limit.

### 5. Aim-picker diagnosis (approved plan A + B + C, not yet implemented)

Measured by running `rewrite_gimbals_perpendicular` on the real 398 waypoints
against the fresh mesh (201 facets after the tight polygon; 221 with the
augment's polygon):

| metric | value |
|---|---|
| target switches | 76 (30 distinct facets used) |
| aim reversals > 90° / > 150° | 23 / 8 (with hysteresis `switch_ratio=0.8`) |
| picks disagreeing with their ±3-neighbour majority | 79 (20%) |
| picks onto near-horizontal facets (\|nz\| ≥ 0.7) | 75 — car and van roofs on this site |
| pitch < −70° / pitch > 0° | 20 / 13 |
| standoff to picked centre | p50 7.6 m, p90 14.5 m, **max 31.4 m** |
| aim run length | p50 5 WPs; 6 single-WP blips; 16 runs ≤ 2 |

Facets that received ≥ 5 waypoints (id · #WP · flagged picks · kind · m² · centre):

```
 63  44  7  wall   1.6  (25.1, 39.9, 4.5)   tree, slice z 3.7–5.3
197  36  5  wall   1.4  ( 2.7,  7.2, 1.2)   van side
 64  35  3  wall   1.1  (30.4, 38.0, 2.5)   tree
 42  27  6  wall   1.1  (29.8, 39.0, 6.5)   tree, top slice
127  23  5  wall   1.5  (29.1, 37.3, 4.5)   tree
 49  21  2  wall   1.9  ( 7.1,  5.7, 1.5)   van
 33  21  3  wall   1.7  (30.2, 42.0, 3.9)   tree
 72  19  5  wall   1.4  (29.4, 38.8, 5.5)   tree
 31  18  5  roof   1.7  ( 5.0,  7.4, 2.5)   van roof
 71  18  4  wall   1.7  (27.1, 41.2, 6.4)   tree
 28  17  4  roof   1.3  (29.0, 52.0, 2.1)   car roof
118  16  4  wall   1.9  ( 4.6,  8.0, 1.6)   van
 41  14  4  roof   2.0  (15.0, 28.1, 1.7)   car roof
 62  13  1  wall   1.5  (25.1, 39.9, 3.5)   tree
188  11  3  wall   1.2  ( 4.5,  4.7, 1.0)   van
 78  11  1  wall   1.5  (30.7, 41.7, 2.9)   tree
 21   9  2  roof   1.6  (21.5, 28.8, 2.4)   car roof
 22   8  2  roof   3.5  (14.7, 29.1, 1.6)   car roof
 25   8  6  wall   1.4  (28.4, 41.9, 5.8)   tree
 38   5  4  tilted 1.8  (12.5, 25.9, 0.8)   near-ground
```

Three failure modes, all visible in the PNG: (1) grabbing a facet 20–30 m away
when the aircraft is over empty tarmac, because `max_distance_m=60`; (2)
slice-hopping between vertical slices of one surface (the tree: 63/62/42/127/72/
25/71), producing pitch jitter and most of the 76 switches; (3) nearest-wins
choosing a car roof 3 m below, producing near-nadir pitches.

Plan (bounded change in `gimbal_rewrite.py` + CLI/API knobs + tests):

- **A. Standoff cap.** Reach limited to the distance where GSD would be 2× target
  (≈14.6 m on WIDE). No facet inside → keep DJI's own Smart3D pose for that
  waypoint (the existing "no pick" path already does this).
- **B. Sequence assignment.** Viterbi over all waypoints: node cost = weighted 3D
  distance + orientation class + pitch beyond ±45°; transition cost = 0 for the
  same target, else `switch_cost × (1 + turn°/90)`; facets on the same plane
  count as the same target. Greedy mode stays selectable.
- **C. Pre-flight audit.** `scripts/render_aim_audit.py` producing the PNG above;
  card gains `far N · flips N`.

Success criteria on this same data before any flight: reversals > 90° 23 → ≤ 5;
far picks (> cap) → 0; switches 76 → < 30; car-roof picks down. Limit: none of
this knows a car from a wall; on a building that matters less.

### 6. Open items

| | Item | Owner |
|---|---|---|
| 🔴 | Commit both repos (laptop `aero-scan`; Manifold `/open_app/dev`) | you |
| 🔴 | Install the rebuilt DPK: `dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk`, then start in Pilot → Application Management | you |
| ✅ | Deploy laptop augment code to the Manifold (`scripts/deploy_to_manifold.sh --host=192.168.1.118`) — done 2026-09-02, 25 tests pass in the Manifold env | done |
| 🔴 | Archive the 2026-07-10 SD-card photos to `flight-archive/2026-07-10/photos-flight0073/` | you |
| 🟡 | Telemetry reader script (`scripts/read_flight_telemetry.py`: quaternion → yaw, diff vs flown KMZ per WP) | not started |
| 🟡 | Picker A + B + C (§5) | approved, not started |
| 🟡 | Flight-time estimator in `validate.py` ignores accel/decel in stop mode (11.7 min vs 19.8 min modelled) | open |
| 🟡 | `min_action_dwell_s=1.0` is a floor, not a calibration — only matters in fly-through | open |
| ℹ️ | Laptop `aeroscan-psdk` clone (`d7cd77f`) is behind the Manifold (`02c1721` + uncommitted) — pull after committing on the Manifold | you |

### 7. Next flight — measurement protocol

1. Install the DPK; confirm in the Pilot log that `kmz_runner` subscribes
   `GIMBAL_ANGLES` without error.
2. Smart3D Auto-Exploration scan of the target. **Do not power-cycle** afterwards.
3. Augment immediately. On the card check: `-> flightNNNN` is the slot you just
   scanned; `Facets` in the hundreds, not ~50; `Stop@WP on`; `warn` count and
   which codes (summary JSON).
4. Approve, fly. Expect ~20 min and a visibly stop-start flight.
5. After landing, before any reboot: pull `/open_app/dev/data/received/telemetry/*.csv`,
   the `/blackbox/<slot>/psdk/PSDK-*.log`, and the SD-card `DCIM` folder.
6. Judge from data, not the FPV feed:
   - photos on card = 398, each named `_wpNNN`;
   - `scripts/read_gimbal_xmp.py <photos> --kmz <flown>.augmented.lean.kmz`:
     gimbal pan near 0°, bearing error falling toward the airframe's ~14°;
   - telemetry CSV: gimbal pitch/yaw tracking the commanded values at each stop;
     verify the quaternion yaw sign against XMP `FlightYawDegree` once;
   - action starts vs completions in the PSDK log — informative only (see §3).
7. Archive with `scripts/pull_flight_archive.sh`; write the review.


## Addendum (same day) — picker redesign outcome

Implemented and measured offline after this report was written. On the same
398 waypoints and the fresh flight0072 mesh, the new default (Viterbi sequence
assignment, 14.6 m reach, DJI pose kept when nothing is in reach) gives: far
picks 39 → 0, max standoff 31.4 → 14.6 m, single-waypoint blips 1 → 0, target
switches 76 → 54, reversals >90° 23 → 14. The 14 residual reversals are
handovers between the van and parked cars at 9–14 m — a feature of the test
venue (two targets with cars between them), not of the picker. Details and the
switch-cost sweep: `docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md`,
section "Picker redesign". Picture: `scripts/render_aim_audit.py`. Unflown.
