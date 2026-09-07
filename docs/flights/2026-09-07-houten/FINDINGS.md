# 2026-09-07 — Houten. Three sites, first real houses, and a clean A/B on stop-and-shoot.

Data pulled from the Manifold at 16:03 the same day.
Archive: `flight-archive/2026-09-07/` (augment outputs, 7 `/blackbox` slots, 24 mesh chunks, 391 MB).
Photos are still on the SD card — see "What is still missing".

## The comparison set for the team (last three photo folders)

All three fly the **same building** (`Houten-3`, `/blackbox/flight0082`), one after the other,
from the same Smart3D scan. Only the flight settings differ.

| | Flight | Route | Speed | Turn mode | Shots/WP | Photos planned |
|---|---|---|---|---|---|---|
| **A** | DJI Smart3D, normal settings | DJI's own | DJI's | DJI's | 5-pose rosette | DJI's |
| **B** | AeroScan `20260907T131731Z_187` | same 276 WPs | **1.0 m/s** | fly-through | **1** | **276** |
| **C** | AeroScan `20260907T133135Z_705` | same 276 WPs | **2.0 m/s** | **stop at WP** | **2** | **539** |

A is the baseline the inspection has always had. B and C are ours, and they differ by
exactly two settings, so the difference between them is measurement, not opinion.

### B vs C — one toggle, 14× the walls

Same mission, same mesh, same 276 waypoints, same 353 detected facets:

| | B (1 m/s, fly-through, 1 shot) | C (2 m/s, stop, 2 shots) |
|---|---|---|
| Photos | 276 | **539** |
| Facets photographed | 20 of 353 | **283 of 353** |
| Walls ≥ 2 m² photographed | 5 of 162 | **71 of 162** |
| Walls ≥ 2 m² with zero photos | 157 | **91** |
| Plan-time warnings | dwell too short ×56, heading unreachable ×2 | none of those |
| Median GSD | 1.85 mm/px | 1.85 mm/px |

**Stop-and-shoot with a second gimbal-panned photo is the coverage lever.** Doubling the
photos multiplied the walls covered by 14×, because the second shot is aimed at a wall no
other waypoint was going to photograph. This is the 2026-09-03 bench prediction (92 → 78
unshot) landing much harder on a real house than it did on the test-venue vans.

Note C flies **twice as fast** as B and still covers 14× more. Speed was not the cost —
the fly-through mode was, and the plan-time warnings said so before the flight:
B was flagged `action_dwell_too_short` on 56 waypoints, C on none.

## All three sites

| Site | Slot | WPs | Facets | Walls ≥2m² | Stop@WP | Photos | Facets shot | Walls shot | GSD mm/px |
|---|---|---|---|---|---|---|---|---|---|
| 0709Houten | flight0079 | 312 | 477 | 230 | off | 312 | 30 | 8 | 1.83 |
| Houten2-0709 | flight0080 | 416 | 542 | 273 | **on** | 730 | 371 | 87 | 2.01 |
| Houten-3 | flight0082 | 276 | 353 | 162 | **on** | 539 | 283 | 71 | 1.85 |

The pattern repeats across sites: stop-mode runs photograph 60–80% of detected facets,
fly-through runs photograph 6%.

## What worked, first time, on real buildings

- **GSD is in spec on the WIDE lens.** 1.83–2.01 mm/px against a 2.0 target, standoff p90
  8.6–11.0 m, max 14.5 m. The pre-flight worry that a house would push standoff past WIDE's
  7.3 m in-spec limit was wrong — the Smart3D orbit hugs these buildings. The
  range-adaptive-lens item (MEDIUM_TELE) is **not** needed at this standoff.
- **Registration is excellent.** ICP RMSE **0.138 m**, residual drift **2–3 cm**, yaw 82–91°,
  across all three sites. Mijande's reference was 0.468 m. Caveat: `icp_fitness` is 0.09–0.11
  where Mijande read 0.986 — expected when the Manifold cloud covers far more ground than the
  mission cloud, but worth confirming rather than assuming.
- **Aim picker held.** `far_picks 0`, `unaimed 0`, reversals 4 (Houten-3) / 15 (Houten2),
  single blips 0–1. Reach 14.65 m was never the binding constraint.
- **Every planner knob set on the RC reached the engine.** Proven by accident: two Houten2
  augments ran with Reach lowered to **10 m** and the result got measurably worse — 16
  waypoints went unaimed (0 at 14.65 m) and facets covered fell 371 → 338. The panel is
  wired end to end.
- **Registration cache works.** Re-augmenting the same site fell from 12.2 s to 8.9 s.
  Augments took 9–29 s total.
- **Continue (battery-swap resume) ran for real.** `20260907T120925Z_003.resume104.lean.kmz`,
  313 waypoints (104..416), 522 photos, original numbering — sliced and uploaded in the field.
- **The warning gates earned their keep.** Fly-through runs correctly raised
  `action_dwell_too_short`, `heading_step_unreachable` and `photo_interval_too_short`;
  the same mission in stop mode raised none of them.

## Findings that need attention

1. **Telemetry CSV was never written.** `/open_app/dev/data/received/telemetry/` does not
   exist. The gimbal-telemetry subscription (`GIMBAL_ANGLES` / `QUATERNION` /
   `POSITION_FUSED`, implemented 2026-09-02) produced nothing across three flights.
   Either the DPK on the aircraft predates it or the subscription failed silently.
   The Manifold repo is at `9d054ae`.
2. **The field session's PSDK/FC log is gone.** `journalctl` on the Manifold is volatile and
   the aircraft power-cycled on the way home; the journal jumps from 09-03 15:22 to 09-07
   16:03. Everything between 11:38 and 15:31 — including any FC error codes and the
   action start/completion counts that would confirm no photos were skipped — is lost.
   **Pull the journal before power-cycling, or make the journal persistent.**
3. **A DJI waypoint sits 0.3 m from a wall** (Houten2, WP15; also WP10 at 1.8 m and WP28 at
   2.0 m on the other sites). `too_close_to_surface` fired. This is DJI's own trajectory —
   we do not move waypoints — but it is worth knowing the Smart3D route comes that close.
4. **Coverage is still under half.** Even at its best, C photographs 71 of 162 walls ≥ 2 m².
   The next lever is the picker (targets per waypoint), not the detector — see the
   2026-09-03 measurements.

## What the photos say (read after the flight, from the JPEG XMP)

All three photo sets were recovered from the SD card and read with
`scripts/read_gimbal_xmp.py`. The WPML is intent; these are the angles the flight
controller stamped into each frame at the shutter.

### Nothing was dropped

| | photos planned | photos on card | missing |
|---|---|---|---|
| B (1 m/s, fly-through, 1 shot) | 276 | **276** | 0 |
| C (2 m/s, stop, 2 shots) | 539 | **539** | 0 |

2026-07-10 lost 104 of 398 at 0.58 s dwell. Stop-at-waypoint and the 1 m/s
fly-through both returned every frame. The `action_dwell_too_short` warning that
fired on B (56 waypoints) did **not** cost photos at 1 m/s — the warning is
calibrated for the 2 m/s case it was derived from and is conservative here.

### The 2026-07-10 gimbal bug is fixed

| |2026-07-10|B (2026-09-07)|
|---|---|---|
| \|gimbal pan\| median | 51.5° | **6.5°** |
| \|gimbal pan\| p90 | 61.0° (at the stop) | **8.8°** |
| frames beyond the ±60° stop | 41 / 294 | **0 / 276** |
| \|actual − commanded\| pitch | corr +0.40 (no better than a frozen gimbal) | **median 0.1°** |

Emitting no gimbal-yaw command (`2bf3308`) did exactly what it was supposed to.

### And the aircraft DOES honour a commanded gimbal yaw

This overturns the 2026-07-10 reading. Flight C's second shot pans the gimbal off
the nose, which means it *must* command absolute-north gimbal yaw — the very
command the July analysis concluded the M4E ignored. It does not ignore it:

| C, panned shot (n=263) | |
|---|---|
| \|actual − commanded\| yaw | **median 0.1°, p90 0.2°** (commanded on 256 of 263) |
| \|actual − commanded\| pitch | median 0.0°, p90 0.1° |
| frames beyond the ±60° stop | **0 of 539** (max pan 46.1°) |

So July's failure was not "the M4E refuses gimbal yaw." Something else was
saturating the pan then — most likely the old `heading := bearing` weld asking for
pan the gimbal did not have, rather than the FC discarding the command.
`schedule_headings()` (rate-limited heading pursuit, currently off) is back on the
table as a result.

The shot-0 yaw figure in the raw report (median 9.2°) is drawn from only 42 of 276
waypoints and compares against a pose carried forward from the previous waypoint's
pan. It is a bookkeeping artifact of the reader, not an aircraft error.

### The three flights side by side, as the camera actually behaved

| | A — DJI Smart3D rosette | B — ours, fly-through | C — ours, stop + 2 shots |
|---|---|---|---|
| Photos | 635 | 276 | 539 |
| Gimbal pitch median | −30.0° | −8.6° | −16.2° |
| Gimbal pitch range | −90.0 … +30.0° | −88.0 … +33.0° | −88.0 … +33.0° |
| \|pan\| median | 28.5° | 6.5° | 14.1° |
| \|pan\| p90 | 34.6° | 8.8° | 36.3° |
| Frames beyond ±60° | 7 | 0 | 0 |

DJI's rosette spends its pitch budget looking down (median −30°); ours looks at
walls (−8.6° / −16.2°). That is the difference between a mission built to
reconstruct a model and one built to inspect a facade.

## What is still missing

**Nothing — the photos are in.** All seven folders were on the card; the last three (A, B, C)
are archived under `flight-archive/2026-09-07/photos/`, and the 2026-07-10 photos that earlier
notes recorded as lost were still on the card too (folders 006–010) — recover those as well.

Reproduce the numbers above with:

```bash
mkdir -p flight-archive/2026-09-07/photos/{A-dji-rosette,B-1ms-flythrough-1shot,C-2ms-stop-2shots}
# copy the last three DCIM/DJI_2026* folders into those three directories, oldest first

# THE measurement — actual gimbal angles at exposure vs what we commanded
.venv/bin/python scripts/read_gimbal_xmp.py flight-archive/2026-09-07/photos/B-1ms-flythrough-1shot \
    --kmz flight-archive/2026-09-07/augment/20260907T131731Z_187.augmented.lean.kmz
.venv/bin/python scripts/read_gimbal_xmp.py flight-archive/2026-09-07/photos/C-2ms-stop-2shots \
    --kmz flight-archive/2026-09-07/augment/20260907T133135Z_705.augmented.lean.kmz
```

Open items after today: the telemetry CSV, the volatile journal, and coverage still under half.
