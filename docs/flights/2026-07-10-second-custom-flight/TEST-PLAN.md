# TEST PLAN — second custom flight, 2026-07-10

Written 2026-07-09, night before, from the 2026-06-12 flight's root-cause
analysis. Read `../2026-06-12-first-custom-flight/ANALYSIS.md` first if you
haven't — this plan assumes its findings.

## What broke last time, one line each

1. **Heading jitter** — WaypointV3's default heading mode points the nose
   toward the *next* waypoint; our boustrophedon sweep zig-zags, so aircraft
   yaw flipped almost every waypoint (~1s cadence, mean swing 11°, 46% of WPs
   swung >5°).
2. **HMS "gimbal motor overload"** — `cli.py` forced
   `gimbal_dedup_threshold_deg=-1.0` / `heading_dedup_threshold_deg=-1.0`
   (dedup fully disabled) as a workaround for a viewer parser bug. Result:
   every one of 581 waypoints fired its own gimbalRotate + rotateYaw action —
   1,745 actions total, ~3.9/s sustained, peaking 14–21/s.

## What changed since (landed on `main`, laptop repo only — see deploy step)

| # | Fix | File(s) |
|---|---|---|
| 1 | Parser carries last-commanded heading/gimbal pose forward instead of resetting to 0 on deduped waypoints | `kmz_import.py` |
| 2 | Removed the forced `-1.0` dedup override — normal 5°/5° dedup applies again | `cli.py` |
| 3 | Fixed stale `gimbal_dedup_threshold_deg` defaults (`2.0` → `5.0`) that never picked up the earlier bump | `server/api.py`, `frontend/store.ts` |
| 4 | **New default, first flight test:** aircraft now faces the facade it's shooting (`heading_deg` = facade bearing) via WPML `smoothTransition` + per-waypoint `waypointHeadingAngle`, instead of DJI's point-toward-next-waypoint default | `kmz_builder.py` (heading emission), `cli.py` (`preserve_heading=False`) |

Fix #4 is the one with real flight-behavior risk — it changes how the
airframe orients itself during autonomous flight and has never flown. Fixes
1–3 are mechanical (action counts / dedup) and lower risk.

146 tests pass (`.venv/bin/python -m pytest tests/`), frontend typechecks
clean (`npx tsc --noEmit`). No hardware validation yet — that's tomorrow.

**Independent adversarial verification (2026-07-09, separate agent, not the
one that wrote the fix) — verdict:**
- **Gimbal overload: CONFIRMED FIXED.** Mechanism verified by executing old
  vs. new code, not just reading diffs — the regression test genuinely
  fails on old code. Residual: the actual thermal/duty-cycle trip point
  inside the FC's HMS logic isn't knowable statically; "no overload
  warning" is a real-flight observable, not a proof.
- **Heading jitter: PARTIALLY FIXED.** The mechanism that caused the
  original bug (WaypointV3's point-toward-next-waypoint default) is
  genuinely eliminated. But the fix aims at a facade *centroid*, which
  pans smoothly rather than holding flat — a real behavior change from
  what ANALYSIS.md assumed ("stable heading per pass"), not itself a bug.
  More importantly: two specific, proven-by-construction paths can still
  reintroduce the *original* jitter (waypoints with no facade in range;
  the facade picker flip-flopping between two facades). Neither is
  checkable without the real point cloud — see Stage 4.5 below, which
  exists specifically to catch this on tomorrow's actual mission.

## Also landed tonight: real pause/resume + fixed widget icons (separate repo)

Unrelated to the jitter/overload bugs, but landed the same night — a pilot
complaint that came up mid-session: the single "Fly" widget's icon was
literally a rewind/skip-back glyph (not play), and there was no way to pause
a mission — tapping the widget mid-flight was silently ignored (`kmz_runner.c`
only ever called `Action(START)`, gated on our own upload-pipeline state
which resets to idle the instant START succeeds).

This lives in the **`aeroscan-psdk` repo** (C app, `/open_app/dev` on the
Manifold — not this `aero-scan` repo):

- Fixed the Fly icon (was ⏪, now ▶).
- New second widget, "AeroScan Pause/Resume" (a switch, not a button) —
  wired to `DjiWaypointV3_Action(PAUSE)` / `Action(RESUME)`, which existed in
  the SDK but were never called anywhere. Gated on the FC's *actual* mission
  state (`MISSION` = flying → pause allowed, `BREAK` = paused → resume
  allowed), not our own app state, so it can't get stuck out of sync.
- Compiled clean on the Manifold's real `aarch64-linux-gnu-gcc` toolchain
  (zero warnings), built into a fresh `.dpk`, and **installed** on the drone
  (`psdk-demo` v01.00.00.00, installed 2026-07-09 19:30:41).
- **Ground-tested:** both widgets confirmed rendering correctly in Pilot 2
  (play icon fixed, pause/resume switch present) with the aircraft powered.
  Tapping pause while idle correctly does nothing (gated to `MISSION` state
  only, as designed) — confirmed 2026-07-09.
- **Not yet tested:** real pause→resume mid-flight — can only be verified
  in the air, tomorrow.
- **Committed + pushed** — `aeroscan-psdk` commit `02c1721` on `main`
  (`d7cd77f..02c1721`). SSH had gone down mid-session, came back after
  `scripts/fix_ssh_perms.sh` on the Manifold; both repos are clean.

## Pre-flight (tonight or at the depot, before the field)

- [ ] **Deploy the fix to the Manifold.** The augment engine runs from
  `/open_app/dev/aero-scan` on the drone (rsync + editable pip install), not
  from this laptop repo directly. Without this step tomorrow's augment runs
  the **old** code and reproduces 2026-06-12 exactly.
  ```bash
  ./scripts/deploy_to_manifold.sh
  # first time only: ssh dji@192.168.1.55 "mamba run -n aero-scan pip install -e /open_app/dev/aero-scan"
  ```
- [ ] `bash scripts/preflight_check_manifold.sh` — read-only, confirms mesh
  state, disk headroom, DPK install, `/open_app/dev` state.
- [ ] Confirm the deploy landed — two checks, both must pass:
  ```bash
  # 1. new heading mechanism present (fix #4)
  ssh dji@192.168.1.55 "grep -c use_global_heading_param /open_app/dev/aero-scan/src/flight_planner/kmz_builder.py"
  # expect a nonzero count — 0 or a grep error means the rsync didn't land

  # 2. old -1.0 dedup override is GONE (fix #2)
  ssh dji@192.168.1.55 "grep -c 'gimbal_dedup_threshold_deg=-1.0' /open_app/dev/aero-scan/src/flight_planner/cli.py"
  # expect 0 — nonzero means you're still running the old cli.py
  ```
  If either check fails, do not fly on unverified code — re-run `deploy_to_manifold.sh`.
- [ ] Re-confirm the four blockers from 2026-06-12 (`TEST-FLIGHT-RUNBOOK.md`) are still true: M4E drone/payload enums (99/88), motors OFF before tapping Fly (let START auto-takeoff — `error_code 778` if armed), mission-area polygon stays out of `waylines.wpml`, Fly-widget state persists across a restart. None of today's changes touch these — should be unaffected, but the runbook says "these will bite again," so don't assume.
- [ ] Charge batteries (≥2), RC, laptop. SD card in aircraft.
- [x] **Ground-tested 2026-07-09:** tapped the pause switch while idle
  (nothing flying) — correctly a no-op, no crash. This is the designed
  behavior (gated on the FC's actual mission state, not a bug).
- [x] **`aeroscan-psdk` committed + pushed** (resolved 2026-07-09, same
  night — SSH came back after `scripts/fix_ssh_perms.sh`). Commit `02c1721`
  on `main`, `d7cd77f..02c1721`. Manifold repo `git status` clean.
- [x] **rc-companion does NOT need rebuilding for tomorrow** (resolved
  2026-07-09, no longer an open question). Confirmed by reading
  `AugmentSession.kt`: it treats the augmented KMZ and summary JSON as
  opaque bytes — it doesn't parse WPML internals, just displays whatever
  the Python augment engine (already fixed, already deployed) puts in the
  summary JSON, which tonight's fixes didn't change the schema of. The
  currently-installed APK already proved the full AUGM→PRVW→EXEC flow on
  2026-06-12. The known gap (no `a37fce1` fail-fast hard-block) just means
  a no-mesh augment fails a step later than it could (`exit 1` at `[2/7]`
  instead of an earlier UI block) — same outcome, not a correctness gap.
- [ ] **The unresolved item from `FLIGHT-REVIEW.md`:** verify the RC Plus 2 writes a DJI flight-record `.txt` for a PSDK WaypointV3 flight (`Internal shared storage/DJI/com.dji.industry.pilot/FlightRecord/`). Still unverified after the last flight. 5-minute check, do it opportunistically — it's the only source of *measured* (not commanded) telemetry, and would let us confirm the fix worked from ground-truth data, not just the KMZ we uploaded.

## Field procedure

Same as `TEST-FLIGHT-RUNBOOK.md` stages 1–4 (scan → switch app → augment →
approve/upload) — no changes there. **New step inserted before Stage 5
(GO/NO-GO):**

### Stage 4.5 — pre-flight pose check (new)

Before tapping Fly, pull the staged/augmented KMZ and run it through the
verifier — this is the gate that would have caught both 2026-06-12 bugs
before takeoff:

```bash
# the augmented KMZ is written wherever rc-companion staged it, or:
#   ssh dji@192.168.1.55 "ls -t /open_app/dev/aero-scan/output/*.kmz | head -1"
# then scp it to the laptop and run:
.venv/bin/python scripts/verify_augmented_kmz.py <path-to-augmented.kmz>
```

Check the **"Aircraft heading (nose) smoothness"** section. **Correction
from an independent adversarial review done tonight (2026-07-09) — the
original guidance here was wrong, don't use it:** ">5% of WPs" alone does
NOT distinguish real jitter from the fix's *intended* behavior. The fix
aims each waypoint's nose at its facade's centroid, which legitimately pans
smoothly across a pass (heading changes almost every WP, in one direction)
— that racks up a similar ">5° swing" percentage to real jitter, proven by
constructing a synthetic clean-pan case and running it through the tool.

What to actually check:

- **`bounce fraction`** (new stat) — the nose visiting a heading, moving
  away, then returning close to where it just was. Validated: 0% on a clean
  pan, 100% on the two known failure modes below. **Elevated (>30%) = real
  back-and-forth, investigate before flying.**
- **Two specific proven-possible failure modes** (found by construction,
  not just theory) that would still show up as real jitter despite the fix:
  1. **Waypoints with no facade in range** (`gimbal_rewrite_perpendicular`
     leaves `facade_index == -1` untouched) keep their *original* imported
     DJI heading — i.e. the old zig-zag, unfixed, for those specific WPs.
  2. **The facade picker flip-flopping** between two different facades on
     adjacent waypoints (near a corner, or where two facets are similarly
     close) — each flip snaps the nose to a different centroid bearing.
  Neither is checkable by inspecting `gimbal_rewrite.py` alone — they only
  show up once you have the *actual* facades from tomorrow's real point
  cloud. **This is exactly what Stage 4.5 exists to catch, on the real
  mission, not a synthetic one.**
- **Neither stat is a clean automatic pass/fail** on real data — the actual
  2026-06-12 flown KMZ (confirmed jittery, HMS overload, pilot abort) only
  scores ~14% bounce fraction, because its real character was a continuous
  wander across many small facade facets, not a strict bounce. If either
  number looks even moderately elevated, or ambiguous, **don't trust the
  threshold — look at which specific waypoints are large deltas** (cross-
  reference `facade_index` per WP) and judge by eye.

Also glance at the existing gimbal-aim stats (`=== Aim ===`, `=== Adjacent-WP
smoothness ===`) — unchanged logic, just confirms the gimbal itself still
aims correctly with the new heading behavior.

### Stage 5 — GO/NO-GO, with one addition

Everything from the runbook applies (pilot on sticks, know the abort path,
watch the floating window for `"Mission: flying — waypoint N"`). Additionally
this flight:

- **Watch the aircraft nose, not just the gimbal.** The whole point of fix #4
  is the airframe should visibly stop zig-zagging and instead hold a steady
  heading through each facade pass, only rotating at pass transitions. If the
  nose is still whipping around like last time, that's a NO-GO signal to
  pause/RTH even if the mission is technically "flying."
- **Listen/watch for HMS "gimbal motor overload."** If it recurs despite the
  Stage 4.5 check passing, that's new information — pause and note the WP
  index range where it happened (cross-reference against the pre-flight
  verifier's yaw/pitch-jump report for that KMZ).
- Since fix #4 is untested in the air: fly the first pass cautious,
  abort-ready, at a facade or two before committing to the full mission if
  conditions allow a staged approach.
- **You now have a real pause button.** Use it deliberately for the staged
  approach above — pause after the first facade pass, confirm the aircraft
  actually holds (`BREAK` state, floating window says "paused"), then resume,
  rather than only relying on RTH/abort if something looks wrong. This
  itself is unflown — first real test of PAUSE/RESUME on this airframe.

## Post-flight — what was actually done (2026-07-10)

- [x] **Archived the full flight slot** with `scripts/pull_flight_archive.sh`
  (not just PSDK logs) into `flight-archive/2026-07-10/` — the whole
  `flight0072` slot (both missions flew in one power session; `flight0071` is
  an app boot with no mission). `pull_flight_archive.sh` pulls the entire slot
  including `dji_perception/1/mesh_binary_*.ply`; `pull_flight_debug.sh` only
  stages PSDK logs + mission-state and hits a permissions wall on the DPK log
  (`dji` is not in group `nvidia`, so the sandboxed DPK's log dir is
  unreadable). Used the archiver. No power-cycle between landing and archiving.
  Re-pulled after power-down because a live slot is still being written during
  rsync.
  - Gotcha hit today: `/blackbox` slot **directory** mtimes lie —
    `flight0065`/`flight0066` looked recent by `ls -dt` but every file inside
    was from 2026-06-12. Rank slots by per-file mtime
    (`find <slot> -type f -newermt <date>`), not `ls -dt`. The
    `the_latest_flight` symlink was correct.
- [x] **Ran `scripts/verify_augmented_kmz.py` on BOTH flown KMZs**
  (`20260710T093952Z_881.augmented.lean.kmz`, 144 WP;
  `20260710T095651Z_712.augmented.lean.kmz`, 399 WP). Gimbal pitch median
  −45.5° → −36.5°, horizontal-ish WPs 20.1% → 36.1%, yaw-heavy (>30°) 35.0% →
  24.5%, aim on-target 98.6%. **Gimbal motor overload: ZERO occurrences**
  (resolved). Heading Δ median 2.3° on the 399-WP mission (airframe barely
  turns — see the heading/gimbal-coupling open item in `ANALYSIS.md`).
- [ ] RC flight-record `.txt` check — still not confirmed (see open item;
  carry forward).
- [x] **Wrote `ANALYSIS.md`** in this directory: what flew, the ground fix's
  measured effect, gimbal-overload resolved, PAUSE/RESUME first flight, and the
  six open items (RESUME-restarts-at-WP-1, Fly-widget-inert-while-paused, mesh
  staleness gate didn't fire, ground params hardcoded, low photo count,
  heading/gimbal coupling). Updated `docs/RESUME-HERE.md`.
- [~] Facade-heading behaviour flew without HMS overload and aimed on-target
  98.6%, but PAUSE/RESUME's index behaviour and the heading/gimbal coupling are
  open — do NOT yet treat the heading rewrite as fully trusted. See `ANALYSIS.md`.

## Known-open items NOT addressed tonight (not blocking, for awareness)

- **In-product flight review.** `FLIGHT-REVIEW.md` recommends a ~1–1.5 day
  AeroScan tab that replays a flown mission from the KMZ + PSDK log timeline,
  reusing `DroneAnimation.tsx`/`Viewer3D.tsx`/`MapView.tsx`. Tonight's answer
  was a CLI tool instead — `scripts/verify_augmented_kmz.py` now runs on any
  WPML KMZ (staged or archived, cloud optional) and reports action counts +
  pose/heading smoothness, skipping the aim-check section when no cloud is
  bundled. No chart/browser needed, works over SSH in the field. Worth
  building the in-product tab too if visual replay becomes a recurring need
  — this CLI tool doesn't replace that, it's the fast/offline complement.
- **RC flight-record existence for PSDK flights** — still unverified (see
  pre-flight checklist item above).
- **`docs/RESUME-HERE.md`** open items unrelated to this fix: rc-companion
  Android build (needs gradle, not on this laptop), mirroring doc updates
  into the Manifold's own `/open_app/dev/docs/` (per repo convention, do this
  after tomorrow's flight alongside the new ANALYSIS doc).
- **FC subscription for measured telemetry on the Manifold** (`kmz_runner.c`
  doesn't subscribe to `DjiFcSubscription` for position/gimbal) — would make
  future reviews not depend on the RC's encrypted flight record. ~0.5 day,
  not needed for tomorrow.
