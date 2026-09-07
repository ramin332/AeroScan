---
title: What makes the M4E dawdle at a stop waypoint — WPML / WaypointV3 knob research
date: 2026-09-07
status: research, documentation only. No code changed.
summary: Every arrival / turning / damping / action-trigger knob in WPML, what the local docs actually say, what DJI's own KMZs do, and a ranked list of ways to buy speed back without losing the two-shots-per-position that made 2026-09-07 work.
---

# Stop-at-waypoint dwell — what the docs say, what DJI does, and what we can change

**Research only. Nothing in this report has been implemented or flown.**

---

## 0. Read this first — what "documented" means in this report

The DJI **Cloud API WPML specification** (the authoritative reference for every
`wpml:` element) is **not present anywhere on this machine**. The only DJI-authored
prose available locally is the PSDK tutorial page
`Payload-SDK-Tutorial/docs/en/50.function-overview/10.advanced-function/50.waypoint-mission.md`,
which describes waypoint *types* in one paragraph and never names a `wpml:` element.

So this report grades every claim on a four-level scale, and the grade is stated
on every row:

| Grade | Meaning |
|---|---|
| **DJI-DOC** | Quoted from a DJI-authored document in this repo (only the PSDK tutorial page qualifies). |
| **DJI-EMITTED** | Observed in a KMZ that DJI's own software (Smart3D / AutoExplore / Pilot 2) produced. This is behaviour evidence, not a specification — DJI's choice tells us what it thinks is right, not what the field means. |
| **3RD-PARTY** | Field description from the `djikmz` Python library (`.venv/.../djikmz/`), which we depend on. Several of its docstrings say "from the dji offical doc:" and appear to be verbatim quotes, but the library is community-written and we cannot verify the quote against the source. Treat as a strong hint, not a spec. |
| **INFERRED** | Our reasoning. Explicitly labelled everywhere it appears. |

**Where the WPML spec is not available, "the docs do not say" means "no local DJI
source says." It does not mean DJI has not documented it online.** Anything marked
NOT FOUND below should be checked against `developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/`
before being treated as a real gap.

### Sources used

| Source | Path |
|---|---|
| DJI PSDK tutorial, waypoint mission page | `Payload-SDK-Tutorial/docs/en/50.function-overview/10.advanced-function/50.waypoint-mission.md` |
| DJI PSDK capabilities table | `Payload-SDK-Tutorial/docs/en/40.manifold-quick-start/06.psdk-capabilities-overview.md:78` |
| `djikmz` library models | `/Users/ramin/git/aero-scan/.venv/lib/python3.12/site-packages/djikmz/model/{turn_param,heading_param,waypoint,action_group}.py`, `model/action/{gimbal_actions,camera_actions,movement_actions}.py` |
| DJI Smart3D KMZs (5) | `/Users/ramin/git/aero-scan/kmz/{Mijande,MijandeExtra,Woonhuis,Slochteren,NewSmart3DExploreTask2}.kmz` |
| DJI Smart3D capture route | `~/Documents/Projects/AeroScan/KML/deafad254da341efa290ce4ffada7e0a/NewSmart3DCaptureRoute1.kmz` |
| **DJI AutoExplore planning stub** | `/Users/ramin/git/aero-scan/kmz/autoExplore/db9b50e2_d904_41a2_8012_e3ee144049dd.kmz` |
| **DJI Pilot 2 2D-mapping mission** | `~/Documents/Projects/AeroScan/KML/0d0dd55c4f684499a2d69b82566f4f24/1vanbraakplein.kmz` |
| Our flown missions, 2026-09-07 | `flight-archive/2026-09-07/augment/20260907T131731Z_187.augmented.lean.kmz` (B), `…20260907T133135Z_705.augmented.lean.kmz` (C) |
| Our emitter | `src/flight_planner/kmz_builder.py` |
| Our knobs | `src/flight_planner/models.py:466,479`, `src/flight_planner/validate.py:180-250` |
| PSDK API surface as we use it | `/Users/ramin/git/aeroscan-psdk/docs/psdk/api-notes.md:12-20`, `src/manifold3_app/kmz_runner.c` |

---

## 1. The measurement we are trying to explain

Three consecutive flights, same house (`Houten-3`), same Smart3D scan, same 276
waypoints. Timings from JPEG filename timestamps, first frame to last frame.

| | Flight | Turn mode | Speed | Photos | Span | s/WP | s/photo |
|---|---|---|---|---|---|---|---|
| **A** | DJI Smart3D (DJI's own route) | `toPointAndPassWithContinuityCurvature` | DJI's | 635 | 6.5 min (390 s) | — | **0.61** |
| **B** | ours, fly-through, 1 shot | `toPointAndPassWithContinuityCurvature` | 1.0 m/s | 276 | 6.7 min (402 s) | 1.46 | 1.46 |
| **C** | ours, stop + 2 shots | `toPointAndStopWithContinuityCurvature` | 2.0 m/s | 539 | 21.2 min (1272 s) | **4.61** | 2.36 |

Path geometry, measured directly from the flown KMZs (275 legs, identical in B and C):

```
total path 472 m   leg length: min 0.51  p10 0.84  median 1.63  p90 2.62  max 4.47 m
```

Action chains actually emitted:

| | takePhoto | gimbalRotate | dominant chain |
|---|---|---|---|
| B | 276 | 149 | `takePhoto` |
| C | 539 | 471 | `gimbalRotate → takePhoto → gimbalRotate → takePhoto` (207 of 276 WPs) |

**C costs +870 s over B — +3.15 s per waypoint — while flying its legs at twice
B's commanded speed.**

> **The A/B/C comparison is under-determined.** C differs from B in *three*
> variables at once (turn mode, shots per waypoint, commanded speed). One
> equation, three unknowns. Anything below that splits the 870 s is arithmetic
> on assumptions, and is labelled INFERRED.

---

## 2. Every relevant WPML element

Location column: `E` = observed in a DJI-emitted KMZ, `L` = defined in the `djikmz`
library, `D` = described in a DJI doc in this repo.

### 2.1 Turning and arrival

| Element | Meaning | Values / range | Where found | DJI's own value |
|---|---|---|---|---|
| `wpml:waypointTurnMode` | How the aircraft transits the waypoint. **DJI-DOC** describes the *concepts* (below), **3RD-PARTY** gives the four string values. | `coordinateTurn`, `toPointAndStopWithDiscontinuityCurvature`, `toPointAndStopWithContinuityCurvature`, `toPointAndPassWithContinuityCurvature` | `L` `djikmz/model/turn_param.py:19-24`; `D` `50.waypoint-mission.md:18-26` | **Smart3D: `toPointAndPassWithContinuityCurvature` on 100% of waypoints** (584/584 MijandeExtra, 485/485 Woonhuis, 1618/1618 Slochteren, 1233/1233 Mijande, 711/711 NewSmart3DExploreTask2, 513/513 CaptureRoute1). **Pilot 2 2D map: `coordinateTurn` on 58/60, `toPointAndStopWithDiscontinuityCurvature` on the 2 endpoints.** **AutoExplore stub: `coordinateTurn` on the 2 interior, `toPointAndStopWithDiscontinuityCurvature` on the 2 ends.** |
| `wpml:waypointTurnDampingDist` | "Turn damping distance in meters. Defines how far to the waypoint that the aircraft should turn." **3RD-PARTY**. Required for `coordinateTurn`; `djikmz` notes "from the dji pilot2, this mode [`toPointAndPassWithContinuityCurvature`] does not require damping distance". | float > 0 (m) | `L` `djikmz/model/turn_param.py:40-45, 58-74` | **Smart3D emits it on every waypoint even in pass mode**: MijandeExtra min 0.010 / p10 0.237 / median 0.415 / p90 0.614 / max 1.177 m; Slochteren median 0.506, max 1.278; Woonhuis median 0.415, max 1.372. **Pilot 2 mapping: 0.798 m on all `coordinateTurn` WPs, `10` and `0` on the two stop endpoints.** |
| `wpml:globalWaypointTurnMode` | Mission-level default turn mode. | same four values | `L` `djikmz/model/kml.py:84-88` | Smart3D does not emit it (every WP is explicit). **We emit `toPointAndStopWithDiscontinuityCurvature`** — the `djikmz` default (`kml.py:85`) — and then override per-waypoint. |
| `wpml:useStraightLine` | "Use straight line for waypoint (0: No, 1: Yes) 0/No means trajectory will be a curve." **3RD-PARTY** | 0 / 1 | `L` `djikmz/model/waypoint.py:117-122` | **`1` on 100% of every DJI KMZ inspected** (Smart3D, AutoExplore, Pilot 2 mapping). We also emit `1`. |
| `wpml:globalUseStraightLine` | Mission-level default for the above. | 0 / 1 | `L` `djikmz/model/kml.py:89-94` | Smart3D does not emit it. We emit `1`. |
| `wpml:useGlobalTurnParam` | Whether this waypoint inherits the global turn param. | 0 / 1 | `L` `djikmz/model/waypoint.py:111-116` | Smart3D does not emit it. |
| `wpml:isRisky` | Not described in any local source. | `0` everywhere | `E` Smart3D + AutoExplore | `0` |
| `wpml:waypointWorkType` | Not described in any local source. | `0` everywhere | `E` Smart3D + AutoExplore | `0` |

**The DJI-DOC text on turn behaviour, quoted in full** (`50.waypoint-mission.md:16-26`):

> The waypoint type refers to the way that the drone flies to the waypoint when performing the waypoint mission, including curvature flight, straight flight and coordinated turn.
> * Curvature flight
>   * When the drone performs the flight mission in a continuous curvature, it will not stop when it reaches the designated waypoint.
>   * When the drone performs a flight mission in a continuous curvature, it stops at the waypoint.
>   * When the drone performs a flight mission with a discontinuous curvature, it stops at the waypoint.
> * Straight flight
>   * Straight entry
>   * Straight out
> * Coordinated turn: the drone turns ahead of time before reaching the waypoint

That is the complete DJI-authored description available locally. It maps 1:1 onto
the three `toPointAnd…Curvature` strings plus `coordinateTurn`. **The "five values"
in the task brief resolve to four WPML strings**; the fifth item in DJI's list is
"straight flight (entry/out)", which in WPML is not a turn-mode value at all but
the separate boolean `useStraightLine`.

### 2.2 Heading

| Element | Meaning | Values | Where | DJI's own value |
|---|---|---|---|---|
| `wpml:waypointHeadingMode` | `followWayline` = "nose follows the course direction to the next waypoint"; `manually`; `fixed`; `smoothTransition` = "target yaw angle … given by `waypointHeadingAngle` and transitions evenly to the target yaw angle of the next waypoint during the flight segment"; `towardPOI`. **3RD-PARTY** | 5 values | `L` `djikmz/model/heading_param.py:13-29` | **Smart3D: `smoothTransition` on 100%.** Pilot 2 mapping: `followWayline` on 100%. AutoExplore stub: `followWayline`. We emit `smoothTransition` (`kmz_builder.py:167`). |
| `wpml:waypointHeadingAngle` | Target yaw, degrees. **3RD-PARTY** | −180…180 | `L` `heading_param.py:103-109` | Smart3D emits real per-WP angles. |
| `wpml:waypointHeadingPathMode` | Rotation direction: `clockwise`, `counterClockwise`, `followBadArc` ("rotation … along the shortest path"). **3RD-PARTY** | 3 values | `L` `heading_param.py:35-46` | `followBadArc` on 100% of every DJI KMZ. We emit the same. |
| `wpml:waypointHeadingAngleEnable` | Not described in any local source. | 0 / 1 | `E` Smart3D, AutoExplore | Smart3D: `0` in MijandeExtra / Mijande / Woonhuis / NewSmart3DExploreTask2, **`1` in Slochteren**. **We never emit it.** See §9 UNKNOWNS. |
| `wpml:waypointHeadingPoiIndex`, `wpml:waypointPoiPoint` | POI targeting for `towardPOI`. | — | `L` `heading_param.py:51-86` | zeros in all samples. |

**Heading is part of the arrival cost.** In `smoothTransition` the nose is
interpolating toward the waypoint's commanded angle across the whole leg; with
`toPointAndStop*` the aircraft is stationary while the remainder of that turn
completes. `validate.py:238-247` already models this as `heading_step_unreachable`
using `yaw_rate_deg_per_s` (default 60°/s, `models.py:462`) — **that 60°/s figure
is our assumption, not a documented M4E number** (see §9).

### 2.3 Action triggering — the important table

| Element | Meaning | Values | Where | DJI's own use |
|---|---|---|---|---|
| `wpml:actionTriggerType` | When the action group fires. **3RD-PARTY** enumerates four. | `reachPoint`, `betweenAdjacentPoints`, `multipleTiming`, `multipleDistance` | `L` `djikmz/model/action_group.py:7-13` | see below — **all four are used by DJI somewhere** |
| `wpml:actionTriggerParam` | "Parameter for the action trigger multiple timing and multiple distance; time in second or distance in meter respectively". **3RD-PARTY** | float | `L` `action_group.py:28-33` | **`2` (seconds) in the Pilot 2 mapping mission; `0` in the AutoExplore stub** |
| `wpml:actionGroupStartIndex` / `EndIndex` | Waypoint index range the group applies to. | int | `L` `action_group.py:79-88` | Smart3D `betweenAdjacentPoints` groups span **2–4 waypoints** (MijandeExtra span histogram `{0:2, 1:30, 2:117, 3:89, 4:12}`); the Pilot 2 mapping groups span **0→59, the whole route** |
| `wpml:actionGroupMode` | "only sequence" **3RD-PARTY** | `sequence` | `L` `action_group.py:89-93` | `sequence` everywhere |

**Where each trigger type appears in DJI-authored KMZs (DJI-EMITTED, verbatim counts):**

| Trigger | DJI file | Count | What it triggers |
|---|---|---|---|
| `betweenAdjacentPoints` | MijandeExtra Smart3D | **248** of 250 groups | `startSmartOblique` (the rosette) |
| `betweenAdjacentPoints` | Slochteren Smart3D | **521** of 522 | `startSmartOblique` |
| `betweenAdjacentPoints` | Pilot 2 2D mapping (`1vanbraakplein`) | 1 (spans WP 0→59) | `gimbalAngleLock` + `gimbalRotate(-70°)` + `startTimeLapse(minShootInterval=1.785 s)` |
| **`multipleTiming`** | **Pilot 2 2D mapping** | **1 (spans WP 0→59), `actionTriggerParam=2`** | re-issues `gimbalRotate(-70°)` **every 2 seconds for the whole route** |
| **`multipleDistance`** | **DJI AutoExplore stub** | **1 (spans WP 0→3), `actionTriggerParam=0`** | **`gimbalRotate(pitch −90°, yaw 0) → takePhoto(wide)`** |
| `reachPoint` | Smart3D | 1–2 per mission | `stopSmartOblique` at the last WP |
| `reachPoint` | Pilot 2 2D mapping | 1 (WP 59) | `stopTimeLapse` + `gimbalAngleUnlock` |
| `reachPoint` | **ours (B and C)** | **276 of 276 groups** | our whole capture chain |

**This is the single most important finding in the report and it is DJI-EMITTED, not inferred:
DJI never uses `reachPoint` to take a photo. In every DJI-authored capture mission
inspected, photography is armed once over a waypoint *range* and then fires on a
distance or time cadence, or is handed to a dedicated continuous actuator
(`startSmartOblique` / `startTimeLapse`). We are the only party in this dataset
that puts `takePhoto` behind `reachPoint`.**

### 2.4 Speed

| Element | Meaning | Where | DJI's own value | Ours |
|---|---|---|---|---|
| `wpml:autoFlightSpeed` | Mission cruise speed (m/s). | `E` all files; `L` `task_builder.py` | Smart3D: 0.64–2.10 m/s. **CLAUDE.md records this field as not representative** — MijandeExtra says 0.7 m/s and the aircraft flew ~2.0 m/s. | B `1.0`, C `2.0` |
| `wpml:waypointSpeed` | Per-waypoint speed override. **3RD-PARTY** "Flight speed in m/s" (`waypoint.py:81-85`) | `E`/`L` | Smart3D: `1` on essentially every WP regardless of `autoFlightSpeed` | B `1.0` ×276, C `2.0` ×276 |
| `wpml:globalTransitionalSpeed` | Speed from current position to the first waypoint. | `E`; our `kmz_builder.py:381-385` | Smart3D 4.6–15 m/s | equal to cruise (see §8, minor waste) |
| `wpml:useGlobalSpeed` | Inherit mission speed. | `L` `waypoint.py:86-91` | — | — |

**Neither speed field is authoritative in flight.** Memory `project_flight_speed_overridable_in_pilot`
records that the Pilot 2 fly panel changes mission speed live. Consistent with the
data: flight B was commanded 1.0 m/s and its 472 m / 402 s implies **1.17 m/s**.

### 2.5 Dwell, hover and gimbal timing

| Element | Meaning | Where | DJI's own value | Ours |
|---|---|---|---|---|
| `wpml:hoverTime` (inside `actionActuatorFunc = hover`) | "Hover time in seconds, dji offical doc says float, but in reality it is an integer". **3RD-PARTY** | `L` `djikmz/model/action/movement_actions.py:11-20` | **Present exactly twice per Smart3D mission, both inside `startActionGroup`** (pre-flight camera prime: `gimbalRotate → hover 0.5 → setFocusType → focus → hover 1`). **Zero `hover` actions at any inspection waypoint in any DJI KMZ.** | **Once**, in our `startActionGroup`, `0.5` s (`kmz_builder.py:427`) |
| `wpml:gimbalRotateTimeEnable` | "Enable rotation time limit (0: No, 1: Yes)". **3RD-PARTY** | `L` `gimbal_actions.py:64-69` | **`0` everywhere in every DJI KMZ** | `0` everywhere (`kmz_builder.py:417`) |
| `wpml:gimbalRotateTime` | "Rotation time in seconds". **3RD-PARTY** | `L` `gimbal_actions.py:70-74` | `10` everywhere (inert, because enable = 0) | `10` in the start group, **`0.0` on all 470 per-waypoint rotates** (inert, enable = 0) |
| `wpml:smartObliqueStayTime` | Per-rosette-pose dwell. Not described in any local source; name is self-evident. | `E` Smart3D only | **`0` on all 1098 poses (MijandeExtra), all 917 (Woonhuis), all 2405 (Slochteren)**; NewSmart3DExploreTask2 has 50 non-zero (0.65–1.77 s) out of 1345 | n/a — we do not use `startSmartOblique` |
| `wpml:smartObliqueRunningTime` | Per-pose slew time. Not described locally. | `E` Smart3D only | `0` on essentially all poses | n/a |
| `wpml:smartObliqueCycleMode` | `unlimited` in every Smart3D mission. CLAUDE.md: "the gimbal keeps cycling through the active rosette while flying *between* capture WPs." | `E` | `unlimited` | n/a |
| `wpml:minShootInterval` (inside `startTimeLapse`) | Minimum shutter interval for interval capture. Not described locally; name self-evident. | `E` Pilot 2 mapping | **`1.785` s** | n/a |

### 2.6 The complete WPML vocabulary, and what is *not* in it

Union of every `wpml:` element name across all 9 DJI-authored KMZs and both of our
flown ones (`template.kml` + `waylines.wpml`), sorted:

```
action actionActuatorFunc actionActuatorFuncParam actionGroup actionGroupEndIndex
actionGroupId actionGroupMode actionGroupStartIndex actionId actionTrigger
actionTriggerParam actionTriggerType author autoFlightSpeed autoRerouteInfo
caliFlightEnable cameraFocusType cloudFilePath coordinateMode createTime
dewarpingEnable direction distance droneEnumValue droneInfo droneSubEnumValue
duration efficiencyFlightModeEnable elevationOptimizeEnable ellipsoidHeight
executeHeight executeHeightMode executeRCLostAction exitOnRCLost
facadeWaylineEnable fileSuffix finishAction flyToWaylineMode focusRegionHeight
focusRegionWidth focusX focusY gimbalHeadingYawBase gimbalPitchAngle
gimbalPitchMode gimbalPitchRotateAngle gimbalPitchRotateEnable
gimbalRollRotateAngle gimbalRollRotateEnable gimbalRotateMode gimbalRotateTime
gimbalRotateTimeEnable gimbalYawRotateAngle gimbalYawRotateEnable
globalGimbalPitchMode globalHeight globalRTHHeight globalShootHeight
globalTransitionalSpeed globalUseStraightLine globalWaypointHeadingParam
globalWaypointTurnMode height heightMode hoverTime imageFormat index
isCalibrationFocus isInfiniteFocus isLookAtSceneSet isPointFocus isRisky
mappingHeadingAngle mappingHeadingMode mappingHeadingParam margin minShootInterval
missionAutoRerouteMode missionConfig modelColoringEnable orthoCameraOverlapH
orthoCameraOverlapW orthoLidarOverlapH orthoLidarOverlapW overlap payloadEnumValue
payloadInfo payloadLensIndex payloadParam payloadPositionIndex payloadSubEnumValue
photoSize positioningType quickOrthoMappingEnable realTimeFollowSurfaceByFov
returnMode safetyBottomHeight samplingRate scanningMode shootType
smartObliqueCycleMode smartObliqueEnable smartObliqueEulerPitch
smartObliqueEulerRoll smartObliqueEulerYaw smartObliqueGimbalPitch
smartObliquePoint smartObliqueRunningTime smartObliqueStayTime startActionGroup
takeOffRefPoint takeOffSecurityHeight templateId templateType
transitionalAutoRerouteMode updateTime useGlobalHeadingParam useGlobalHeight
useGlobalPayloadLensIndex useGlobalSpeed useGlobalTurnParam useStraightLine
waylineAvoidLimitAreaMode waylineCoordinateSysParam waylineId
waypointGimbalHeadingMode waypointGimbalHeadingParam waypointGimbalPitchAngle
waypointGimbalYawAngle waypointHeadingAngle waypointHeadingAngleEnable
waypointHeadingMode waypointHeadingParam waypointHeadingPathMode
waypointHeadingPoiIndex waypointPoiPoint waypointSpeed waypointTurnDampingDist
waypointTurnMode waypointTurnParam waypointWorkType
```

**There is no arrival-accuracy element.** No `arrivalTolerance`, no
`positionAccuracy`, no `reachRadius`, no `waypointRadius`, no per-waypoint
`stayTime`. `grep -i` for all of those across the PSDK tutorial tree and the
`djikmz` package returns nothing. **The only per-waypoint dwell control that
exists in WPML is the `hover` action's `hoverTime`, and it can only *add* time,
never subtract it.**

---

## 3. What makes the aircraft slow at a stop waypoint

### 3.1 DOCUMENTED

1. **`toPointAndStopWithContinuityCurvature` means the aircraft stops.**
   DJI-DOC: "When the drone performs a flight mission in a continuous curvature,
   it stops at the waypoint." That is the whole documented mechanism. A stop is a
   decelerate-to-zero, and 276 of them on a 472 m route is by itself a large cost.

2. **`toPointAndPassWithContinuityCurvature` means it does not stop.**
   DJI-DOC: "it will not stop when it reaches the designated waypoint."

3. **`coordinateTurn` turns early.** DJI-DOC: "the drone turns ahead of time
   before reaching the waypoint", parameterised by `waypointTurnDampingDist`
   (3RD-PARTY: "how far to the waypoint that the aircraft should turn").

4. **Nothing else about arrival is documented.** No local DJI source says how
   precisely the aircraft must be positioned before it declares a waypoint
   reached, how long it holds, or how long an action takes.

### 3.2 INFERRED — the time budget

Path: 275 legs, 472 m, median 1.63 m. Flight C span 1272 s.

Stop-and-go travel time, integrated over the *actual* leg-length distribution with
a trapezoidal profile capped at the commanded 2.0 m/s:

| assumed accel | travel | remainder available for dwell |
|---|---|---|
| 1.0 m/s² | 704 s | 568 s → 2.06 s/WP |
| **2.0 m/s²** | **501 s** | **771 s → 2.79 s/WP** |
| 3.0 m/s² | 417 s | 855 s → 3.10 s/WP |
| 4.0 m/s² | 373 s | 899 s → 3.26 s/WP |

**The M4E's waypoint-mode acceleration is NOT DOCUMENTED in any local source.**
The table is therefore a sensitivity analysis, not a result. Taking 2 m/s² as a
mid estimate:

| Component | Estimate | Grade | Basis |
|---|---|---|---|
| **(c) shutter / write** | 539 × 0.5 s = **270 s** (0.98 s/WP) | our own spec constant, not DJI's | `models.py:59` `min_interval_s=0.5` for the WIDE lens. Cross-check: DJI's flight A sustained **0.61 s/photo** for 635 photos while moving, which is consistent with a ~0.5 s floor. |
| **(b) gimbal rotation** | 470 rotations × 0.2–0.6 s = **94–282 s** (0.34–1.02 s/WP) | INFERRED, weakly | No M4E gimbal slew rate exists in any local doc. Upper end anchored on DJI's rosette, which re-points ~30–60° *and* shoots in 0.61 s total. Our pans are smaller: median 14.1°, p90 36.3° (2026-09-07 XMP analysis). |
| **(a) arrival / braking / position hold** | **residual, ~220–410 s (0.8–1.5 s/WP)** | INFERRED | 771 − 270 − (94…282) |
| **(d) fixed per-waypoint FC overhead** | **cannot be separated from (a)** | — | The FC's action state machine emits start/complete callbacks per action (observed in the 2026-07-10 PSDK log, `docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md`). Whether each action round-trips before the next begins is **not documented**. If it does, most of the "arrival" residual is really serialisation. |

**Which of (a)–(d) the docs let us estimate, plainly:**

| | Estimable from docs? |
|---|---|
| (a) arrival / braking / position-hold tolerance | **NO.** No element, no parameter, no prose in any local source. Not exposed, not measurable from the KMZ. |
| (b) gimbal rotation time | **PARTLY.** WPML has `gimbalRotateTimeEnable` + `gimbalRotateTime`, described 3RD-PARTY as a "rotation time limit" in seconds. That is a *knob*. There is **no documented default rate**, so we cannot estimate what it costs today — only cap it. |
| (c) shutter / write | **YES, from our own spec table** (`min_interval_s`), corroborated by DJI's measured 0.61 s/photo. Not from a DJI doc. |
| (d) fixed FC overhead | **NO.** |

### 3.3 The sharpest single fact

**DJI's flight A shot 635 photos in 390 s — 0.61 s per photo — while flying,
with a gimbal that re-points between every shot. Our flight C shot 539 photos in
1272 s — 2.36 s per photo — while stopped. The camera is running at ~26% of the
cadence DJI's own mission achieves on the same aircraft on the same day.**

The camera is not the bottleneck. Neither is the gimbal. **The stop is.** And
because A exists, "two photos from one position with different aims" does **not**
inherently require a stop — DJI gets five differently-aimed poses per position
without ever stopping.

---

## 4. Q2 — `toPointAndStopWithDiscontinuityCurvature` vs `…ContinuityCurvature`

**DOCUMENTED difference:** both stop. The only stated difference is the curvature
of the path (`50.waypoint-mission.md:21-22`):

> * When the drone performs a flight mission in a continuous curvature, it stops at the waypoint.
> * When the drone performs a flight mission with a discontinuous curvature, it stops at the waypoint.

**Settling-time difference: NOT DOCUMENTED. No local source says either is faster.**

**INFERRED, and flagged as speculation:** a curvature-*continuous* path must blend
the incoming and outgoing arcs at the waypoint; a curvature-*discontinuous* one may
fly straight in and pivot. On a 1.63 m median leg, blending has almost no room to
work, so the difference is plausibly negligible. **Do not spend a flight on this
alone.**

**DJI-EMITTED evidence, which cuts the other way from the speculation:** DJI uses
`toPointAndStopWithDiscontinuityCurvature` **only at route endpoints**, and always
with `waypointTurnDampingDist` set to `0` (AutoExplore WP0 and WP3; Pilot 2 mapping
WP0 and WP59). It never uses it mid-route. That is consistent with reading it as
"hard stop, no blending" — i.e. DJI's choice for a terminal point, not a
throughput optimisation.

**Note a live inconsistency in our own output:** our `globalWaypointTurnMode` is
`toPointAndStopWithDiscontinuityCurvature` (inherited from `djikmz`'s default,
`kml.py:85`) while every waypoint overrides it to `…ContinuityCurvature`. Harmless
today because `waypointTurnParam` is emitted on every waypoint, but it means the
global is dead code that would silently take effect if a waypoint ever lost its
override.

---

## 5. Q3 — can actions fire by distance or time instead of by reaching a point?

**YES. Documented as an enum value (3RD-PARTY) and — more importantly —
DEMONSTRATED IN DJI'S OWN KMZs.**

`djikmz/model/action_group.py:7-13`:

```python
class TriggerType(str, Enum):
    REACH_POINT       = "reachPoint"
    BETWEEN_POINTS    = "betweenAdjacentPoints"
    MULTIPLE_TIMING   = "multipleTiming"
    MULTIPLE_DISTANCE = "multipleDistance"
```

`action_group.py:28-33` on `actionTriggerParam`:

> "Parameter for the action trigger multiple timing and multiple distance; time in second or distance in meter respectively"

### 5.1 `multipleDistance` in a DJI-authored file — verbatim

`kmz/autoExplore/db9b50e2_….kmz` → `wpmz/waylines.wpml:42-77`. One action group
spanning waypoints **0 through 3**:

```xml
<wpml:actionGroup>
  <wpml:actionGroupId>0</wpml:actionGroupId>
  <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
  <wpml:actionGroupEndIndex>3</wpml:actionGroupEndIndex>
  <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
  <wpml:actionTrigger>
    <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
    <wpml:actionTriggerParam>0</wpml:actionTriggerParam>
  </wpml:actionTrigger>
  <wpml:action>   <!-- id 0 -->
    <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
    …<wpml:gimbalPitchRotateAngle>-90</wpml:gimbalPitchRotateAngle>…
  </wpml:action>
  <wpml:action>   <!-- id 1 -->
    <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
    …<wpml:payloadLensIndex>wide</wpml:payloadLensIndex>…
  </wpml:action>
</wpml:actionGroup>
```

**Caveat, stated plainly:** this file is a **planning stub, not a flown mission**.
Its `droneEnumValue` is `65535` (wildcard), its coordinates are local metres not
lat/lon, its `wpml:distance` is `10872892` and `wpml:duration` is `1087301.125` —
nonsense values. `actionTriggerParam` is `0`, i.e. the interval is unfilled. It is
genuine DJI-authored WPML proving the **schema** and DJI's intended **pattern**;
it is not proof that the M4E flight controller honours `multipleDistance` in a
real flight.

### 5.2 `multipleTiming` in a real DJI Pilot 2 mission — verbatim

`~/Documents/Projects/AeroScan/KML/0d0dd55c…/1vanbraakplein.kmz` (`templateType =
mapping2d`, 60 waypoints). Three action groups, all spanning the whole route:

| group | trigger | param | actions |
|---|---|---|---|
| 0 | `betweenAdjacentPoints` | — | `gimbalAngleLock`, `gimbalRotate(pitch −70°)`, `startTimeLapse(minShootInterval = 1.78525459766388)` |
| 1 | **`multipleTiming`** | **`2`** | `gimbalRotate(pitch −70°)` — re-issued every 2 s across WP 0→59 |
| 2 | `reachPoint` (WP 59 only) | — | `stopTimeLapse`, `gimbalAngleUnlock` |

Turn modes in that mission: `coordinateTurn` with `waypointTurnDampingDist =
0.798 m` on 58 of 60 waypoints; `toPointAndStopWithDiscontinuityCurvature` with
damping `0` at the two endpoints. `waypointSpeed = 0.8 m/s` on all 60.

**This is a real, Pilot-2-planned, flyable mapping mission that photographs an
entire 60-waypoint route without stopping at a single interior waypoint.** It uses
a continuous shutter actuator armed over a range, a periodic trigger to keep the
gimbal on target, and early-turn damping so the aircraft never has to arrive
precisely anywhere.

### 5.3 What this does and does not buy us

**Buys us:** proof that fly-through capture with a maintained gimbal aim is DJI's
normal pattern, on this aircraft, with these elements.

**Does not buy us — and this is the honest limitation:** `multipleDistance` and
`multipleTiming` repeat **the same action list** at a fixed cadence. They cannot
alternate between two different gimbal aims per position. Our two shots per
waypoint deliberately point at **different facades** (that is exactly why coverage
went 5 → 71 walls). A distance trigger firing `gimbalRotate(aim A) → takePhoto`
every 0.8 m gives us N identical-aim photos, not the A/B pair.

**The fly-through equivalent of our two-shot stop is therefore not a distance
trigger. It is two waypoints.** See option 1 in §7.

---

## 6. Q4 — `hoverTime`

**It is not what is costing us time.**

- **We emit `hover` exactly once**, in `startActionGroup`, `hoverTime = 0.5 s`
  (`kmz_builder.py:427`). Total cost across the whole mission: 0.5 s.
- **No inspection waypoint in any of our flown KMZs contains a `hover` action.**
  Verified: `grep -c hoverTime` = 1 in both flown files.
- **DJI does the same thing.** Every Smart3D KMZ contains exactly two `hoverTime`
  values (`0.5` and `1`), both inside `startActionGroup`, as part of the pre-flight
  camera prime: `gimbalRotate → hover 0.5 → setFocusType → focus → hover 1`. Zero
  `hover` actions at inspection waypoints.

`hoverTime` is documented (3RD-PARTY, `movement_actions.py:11-20`) as "Hover time
in seconds", `gt=0.0`, with the note "dji offical doc says float, but in reality it
is an integer". **It can only add dwell. There is no negative or zero hover, and no
element anywhere that shortens the FC's own arrival hold.**

---

## 7. Ranked options

Each option gives the exact WPML change, the expected effect, and what it costs.
**All are unflown and unvalidated.**

---

### 1. Split every 2-shot waypoint into two waypoints and fly through — *highest expected value*

**Change:** for each inspection waypoint that currently carries
`gimbalRotate(A) → takePhoto → gimbalRotate(B) → takePhoto`, emit **two**
waypoints ~0.4–0.8 m apart along the existing flight direction, each with one aim
and one `takePhoto`, and set every waypoint to:

```xml
<wpml:waypointTurnParam>
  <wpml:waypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:waypointTurnMode>
  <wpml:waypointTurnDampingDist>0.4</wpml:waypointTurnDampingDist>
</wpml:waypointTurnParam>
```

Emitting the damping distance in pass mode is what **DJI itself does on 100% of
Smart3D waypoints** (median 0.415–0.506 m across the sample) — our current output
omits it entirely.

Waypoint count goes 276 → ~539, far under the documented 65535 limit
(`50.waypoint-mission.md:15`).

**Why it should work:** removes 276 stops. Keeps one aim per waypoint, so no
action chain has to serialise two gimbal moves in one dwell. Matches DJI's own
geometry (their median leg is 1.56 m — the same order as our proposed split).

**Risks:**
- Each waypoint's `reachPoint` chain now has only the leg time. At 0.5 m and
  1.0 m/s that is 0.5 s — **below our own `min_action_dwell_s` floor of 1.0 s**
  (`models.py:479`), and the 2026-07-10 flight lost 104 of 398 photos at 0.58 s.
  *But note*: on 2026-09-07 flight B, `action_dwell_too_short` fired on 56
  waypoints at 1.0 m/s and **zero photos were lost**. The floor is conservative.
- Requires an aim-scheduling change in the planner (two waypoints where there was
  one), which is real work, not a KMZ tweak.
- `validate.py:211-220` currently raises a hard **ERROR** (`extra_shots_need_stop`)
  for multi-shot waypoints in fly-through mode. That gate would have to be
  re-scoped, and it exists for a measured reason.

---

### 2. Keep the stop, but only where the second shot actually earns it

**Change:** per-waypoint `waypointTurnMode` — `toPointAndStopWithContinuityCurvature`
only on the waypoints whose second shot covers a facet nothing else covers;
`toPointAndPassWithContinuityCurvature` (+ damping ~0.4 m) everywhere else.

Our planner already knows which waypoints those are: `Waypoint.extra_facade_indices`
(`models.py:162`) is non-empty only where an extra shot was scheduled. On flight C,
**207 of 276** waypoints carried the full 2-shot chain — but the coverage win came
from the *distinct* facets, which is a smaller set.

**Why it should work:** linear saving. Cutting stops by half cuts roughly half of
the ~2.8 s/WP dwell.

**Risks:** none new — it is the current, flown behaviour applied selectively. Lowest
risk of anything here. Smallest ceiling, too.

---

### 3. `coordinateTurn` with damping instead of stopping

**Change:**

```xml
<wpml:waypointTurnMode>coordinateTurn</wpml:waypointTurnMode>
<wpml:waypointTurnDampingDist>0.8</wpml:waypointTurnDampingDist>
```

matching the Pilot 2 mapping mission's 0.798 m exactly.

**Why it might work:** DJI-DOC: "the drone turns ahead of time before reaching the
waypoint." No stop at all, and DJI uses it for its own 60-waypoint mapping route.

**Risks — significant:**
- **INFERRED but important:** with `coordinateTurn` the aircraft cuts the corner
  and may never pass through the waypoint's coordinates. What `reachPoint` then
  means is **NOT DOCUMENTED**. Our photos are geometrically planned — a waypoint
  that is missed by 0.8 m at a 8–11 m standoff shifts the aim by ~5°, and the
  `too_close_to_surface` margins (a DJI waypoint sat 0.3 m from a wall on
  2026-09-07) are not built for corner-cutting.
- `djikmz` **requires** `waypointTurnDampingDist` for this mode
  (`turn_param.py:63-72`), so this cannot be emitted without also choosing a value.
- On a facade sweep with 0.5 m minimum legs, a 0.8 m damping radius is larger than
  the leg. Behaviour in that regime is undefined in every source we have.

---

### 4. Cap the gimbal rotation time explicitly

**Change:** on every `gimbalRotate` action,

```xml
<wpml:gimbalRotateTimeEnable>1</wpml:gimbalRotateTimeEnable>
<wpml:gimbalRotateTime>0.4</wpml:gimbalRotateTime>
```

(currently `enable = 0`, `time = 0.0` on all 470 per-waypoint rotates —
`kmz_builder.py:417-418`, `kmz_slice.py:109-110`).

**Why it might work:** if the FC waits for the gimbal to report "arrived" before
starting the shutter, bounding that wait bounds component (b).

**Risks:**
- **The direction of this parameter is UNKNOWN.** 3RD-PARTY calls it a "rotation
  time *limit*", which reads like a cap — but it could equally be a *target
  duration*, in which case `0.4` on a 5° re-aim would make the mission *slower*,
  and every DJI KMZ setting it to `10` with `enable = 0` is consistent with either
  reading.
- **This is the option that most directly trades away the guarantee the photos
  depend on.** If the cap expires before the gimbal reaches the commanded angle,
  the shutter fires on a moving gimbal at a wrong aim — and the 2026-09-07 XMP
  analysis (median |actual − commanded| pitch **0.0°**, yaw **0.1°**) is exactly
  the thing we would be putting at risk.
- Cheap to test, but **must** be verified against the JPEG XMP, not the WPML.

---

### 5. Try `toPointAndStopWithDiscontinuityCurvature` instead of `…Continuity`

**Change:** one string, per waypoint.

**Why it might work:** speculation only (§4).

**Risk:** low risk, low expected value. **Do not spend a dedicated flight on it** —
fold it into another test if at all.

---

### 6. Not an option, but free: `globalTransitionalSpeed`

We set `globalTransitionalSpeed` to the inspection cruise speed
(`kmz_builder.py:381-385`, so `2.0` on flight C). That is the speed from the
current position to the **first** waypoint. DJI uses **4.6–15 m/s** for the same
field. Raising it costs nothing at the waypoints and saves the transit in. Small,
but it is pure waste today.

---

## 8. Where the docs contradict — or fail to support — what our code assumes

| Our assumption | Where | Status |
|---|---|---|
| "Stop mode means the action chain always completes" (`models.py:470-471`, `validate.py:229`) | comment + gate | **Not documented.** DJI-DOC says the aircraft stops; it says nothing about the FC waiting for actions. On 2026-09-07 it happened to be true (539/539 photos returned) — that is evidence, not a spec. |
| `yaw_rate_deg_per_s = 60.0` (`models.py:462`) drives `heading_step_unreachable` | constant | **Not documented anywhere local.** No M4E yaw-rate figure exists in the PSDK tutorial or the `djikmz` models. |
| `min_action_dwell_s = 1.0` (`models.py:479`) | constant | Self-labelled in the source as "a floor with margin, not a calibrated constant". 2026-09-07 flight B **contradicts** it in the safe direction: 56 waypoints below the floor, 0 photos lost. |
| `min_interval_s = 0.5` for WIDE (`models.py:59`) | constant | Corroborated, not documented: DJI's own flight A sustained 0.61 s/photo. |
| We emit no `waypointTurnDampingDist` at all | `kmz_builder.py:179` → `djikmz` `task_builder.py:143` (only `early_turn` gets one) | **DJI emits it on 100% of Smart3D waypoints even in pass mode.** Our fly-through missions (B, and 2026-07-10) had no damping at all. Whether that changed the flown path is **UNKNOWN**. |
| `globalWaypointTurnMode = toPointAndStopWithDiscontinuityCurvature` | `djikmz/model/kml.py:85` default, unset by us | Inconsistent with every per-waypoint value we emit. Dead code today; a latent trap. |
| We never emit `waypointHeadingAngleEnable` | — | DJI emits it on every Smart3D waypoint (`0` in four missions, `1` in Slochteren). Its meaning is **NOT FOUND**. If it gates whether `waypointHeadingAngle` is honoured, our omission is load-bearing and untested. |
| "There is no runtime cruise speed API in V3; speeds are baked per-waypoint in the WPML" (`kmz_builder.py:7-9`) | comment | **Correct as far as PSDK goes** (§ next), but incomplete: memory `project_flight_speed_overridable_in_pilot` records that the **pilot** can change mission speed live from the Pilot 2 fly panel. Speed in the KMZ is a seed, not a commitment — and flight B's measured 1.17 m/s against a commanded 1.0 m/s is consistent with that. |

---

## 9. Q5 — does PSDK expose any of this at runtime?

**No.** The complete WaypointV3 surface, from the DJI tutorial page
(`50.waypoint-mission.md:201-255`) and our own API notes
(`aeroscan-psdk/docs/psdk/api-notes.md:12-20`):

```c
DjiWaypointV3_Init(void);
DjiWaypointV3_UploadKmzFile(const uint8_t *data, uint32_t dataLen);
DjiWaypointV3_Action(E_DjiWaypointV3Action action);   // START | STOP | PAUSE | RESUME
DjiWaypointV3_RegMissionStateCallback(...);
DjiWaypointV3_RegActionStateCallback(...);            // used in kmz_runner.c
```

That is all of it. **There is no `SetCruiseSpeed`, no arrival-tolerance setter, no
per-waypoint override, no dwell control.** Confirmed against our own app: the only
`DjiWaypointV3_*` symbols anywhere in `aeroscan-psdk/src` are `Init`,
`UploadKmzFile`, `Action(START|PAUSE|RESUME)` and the two callback registrations
(`kmz_runner.c:593, 984, 1028, 1038, 1275-1277`).

**Contrast — Waypoint 2.0 *did* have a runtime speed API**
(`DjiWaypointV2_SetGlobalCruiseSpeed` / `GetGlobalCruiseSpeed`,
`50.waypoint-mission.md:189-199`), but that path "only supports Matrice 300 RTK
and Matrice 350 RTK" (`:84`) and explicitly does **not** support our aircraft
(`:204`). It is not available to us.

**What the callbacks give back:** `T_DjiWaypointV3MissionState` carries `state`,
`currentWaypointIndex`, `wayLineId` (used at `kmz_runner.c:1450-1470`). No
position error, no dwell time, no per-action timing. The action-state callback
gives start/complete events, which is the closest thing we have to a per-action
timer — and the 2026-09-02 review found those completion callbacks to be an
**unreliable record** (`docs/reports/2026-09-02-flight-review-and-plan.md`, §3).

**The KMZ is the only lever.** Everything in §7 has to be authored, uploaded and
flown; none of it can be tuned in the air from the Manifold.

---

## 10. NOT FOUND / UNKNOWN — do not let anyone turn these into facts

1. **The DJI Cloud API WPML specification is not in this repo.** Every element
   meaning marked 3RD-PARTY above comes from `djikmz`, not from DJI. **Check the
   online spec before acting on any of them.** This is the single biggest caveat
   in the report.
2. **There is no arrival-accuracy / position-tolerance parameter anywhere.** Not in
   WPML (full vocabulary in §2.6), not in PSDK. How precisely the M4E must arrive
   before `reachPoint` fires is **not exposed and not documented**. If that hold is
   the dominant cost, **no KMZ change can shorten it** — only not stopping can.
3. **No settling-time difference is documented between the two `…Stop…` modes.**
4. **No M4E acceleration figure** exists in any local source. The §3.2 travel
   budget is a sensitivity table, not a measurement.
5. **No M4E gimbal slew rate** exists in any local source. Component (b) of the
   time budget is the weakest number in this report.
6. **`gimbalRotateTime` direction is unknown** — cap or target? Every DJI KMZ sets
   `enable = 0`, so DJI's own files give no evidence either way.
7. **`multipleDistance` is unproven in flight.** The only DJI file using it is a
   planning stub with wildcard drone enum and `actionTriggerParam = 0`. We have
   **no evidence the M4E FC executes it**, and no evidence about what happens when
   a `multipleDistance` group's range overlaps waypoints that also carry
   `reachPoint` groups.
8. **`waypointHeadingAngleEnable` meaning is unknown.** DJI emits `0` in four
   Smart3D missions and `1` in Slochteren. We never emit it.
9. **`waypointWorkType`, `isRisky` meanings are unknown.** Both `0` in every DJI
   file; we emit neither.
10. **`efficiencyFlightModeEnable` meaning is unknown.** Appears in
    `template.kml` of the AutoExplore stub and the Pilot 2 mapping mission, `0` in
    both. The name is suggestive enough that it should be looked up before the next
    speed experiment.
11. **`missionAutoRerouteMode` / `transitionalAutoRerouteMode` are unknown** (`1`
    in the Pilot 2 mapping mission only).
12. **Whether the FC serialises action start→complete round-trips** — i.e. whether
    component (d) exists at all — is **not documented** and the 2026-07-10
    completion-callback data was found unreliable.
13. **Flight A's route length is unknown** (it is DJI's own Smart3D route, whose
    KMZ for `Houten-3` is not in `kmz/`), so A's 0.61 s/photo cannot be converted
    into a ground speed here.
14. **The 402 s / 1272 s spans are first-photo-to-last-photo**, so they exclude
    takeoff, the transit to WP0, and the return. They are lower bounds on total
    mission time and they undercount by one leg.

---

## 11. Recommended next step

Not a code change — a **measurement that costs one flight**.

Fly the same 276-waypoint mission twice more on the same building:

| run | change vs flight C | isolates |
|---|---|---|
| **D** | stop mode kept, **one** shot per waypoint (drop the pan) | separates the 2-shot chain cost (b + c) from the stop cost (a + d) |
| **E** | fly-through + damping 0.4 m, **one** shot per waypoint, 2.0 m/s | gives the pure travel time for this route at 2 m/s, which is the missing constant in every estimate in §3.2 |

With C, D and E the system of equations closes and the 870 s splits for real
instead of by assumption. Until then, the only claim this report will stand
behind without qualification is the one in §3.3: **DJI photographs the same
building four times faster than we do, without stopping, on the same aircraft,
using `betweenAdjacentPoints` and `multipleTiming`/`multipleDistance` triggers
that we do not use at all.**

---

## 11. Verification against DJI's Smart3D obliques (added after review)

The pilot objected that §5 rests on two files that are not the job we do: a DJI
AutoExplore *planning stub* and a **nadir 2D mapping** mission. Neither photographs
a facade, and neither takes two differently-aimed photos from one position. The
objection is correct, so the claim was re-tested against DJI's **Smart 3D oblique**
missions — the exact mission class flight A flew.

Measured directly from `kmz/` (counts are per file, from `wpmz/waylines.wpml`):

| file | WPs | turn mode | damping | action triggers | photo actuator |
|---|---|---|---|---|---|
| Mijande.kmz | 1233 | 100% `toPointAndPassWithContinuityCurvature` | 1233 | `betweenAdjacentPoints` ×583, `reachPoint` ×2 | `startSmartOblique` ×583 |
| MijandeExtra.kmz | 584 | 100% pass | 584 | `betweenAdjacentPoints` ×248, `reachPoint` ×2 | `startSmartOblique` ×248 |
| Woonhuis.kmz | 485 | 100% pass | 485 | `betweenAdjacentPoints` ×205, `reachPoint` ×2 | `startSmartOblique` ×205 |
| Slochteren.kmz | 1618 | 100% pass | 1618 | `betweenAdjacentPoints` ×521, `reachPoint` ×1 | `startSmartOblique` ×521 |
| NewSmart3DExploreTask2.kmz | 711 | 100% pass | 711 | `betweenAdjacentPoints` ×289, `reachPoint` ×2 | `startSmartOblique` ×289 |
| **Slochteren_custom.kmz** (ours) | 1618 | 100% pass | **0** | **`reachPoint` ×1618** | **`takePhoto` ×1618** |

The §5 conclusion survives on the right file class, and more strongly: **not one
DJI Smart3D waypoint uses a stop turn mode, every one carries damping, and there is
no `takePhoto` action anywhere in any of them.** The two `reachPoint` triggers per
mission are the start/stop action group, never a photo.

### But the mechanism does not transfer, and §5 overstated the prize

DJI achieves fly-through capture with **`startSmartOblique`**, a DJI-internal
actuator that cycles a fixed 5-pose rosette continuously (`smartObliqueCycleMode =
unlimited`; see CLAUDE.md). It cannot be given per-wall computed aims — the whole
point of AeroScan. **We cannot copy how DJI does it.** Note also that DJI uses
neither `multipleDistance` nor `multipleTiming` in any Smart3D mission; those
appear only in the stub and the mapping flight, so §5's ranking rested on the
wrong precedent.

### What actually transfers

1. **Fly-through with `reachPoint` + `takePhoto` already works for us, in the air.**
   Flight B (2026-09-07) flew exactly that and returned **276 of 276** photos in
   6.7 min. This is flown evidence, not inference. The untested part is narrower
   than §5 implied: only whether **two differently-aimed shots** survive fly-through.
2. **The two-waypoint split (option 1) stays the leading candidate** — but on our
   own evidence, not on DJI's precedent.
3. **We emit no `waypointTurnDampingDist`.** `Slochteren_custom.kmz` above is our
   own output: 1618 waypoints, damping 0, against DJI's 100%. This is the cheapest
   change on the list and is independent of everything else.
