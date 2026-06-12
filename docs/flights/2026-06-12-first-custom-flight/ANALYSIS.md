# Flight analysis — 2026-06-12 (first custom WaypointV3 flight)

Evidence: plaintext `psdk/PSDK-*.log` in each flight dir (NUL-padded — read with
`tr -d '\000'` / `LANG=C grep -a`), directory layout, file sizes. MCU/diag
telemetry (`*.log.enc`) is DJI-encrypted (`LOGH`/`SLOG` AES) — not readable here.

## Flight identification

Slot numbers are **not** chronological (ring buffer). Real order:
`0064 (13:56) → 0065 (14:11) → 0066 (14:23) → 0067 (15:00)`.

| slot | role | flew? | evidence |
|------|------|-------|----------|
| **0064** | our app ran, **augment FAILED** (used stale `flight0058` mesh, age 9135 s) → no flight | no | `psdk/PSDK-0064-01.log`: `augment-mission exited 1`, `Augment FAILED`. No START. |
| **0065** | **Smart3D auto-exploration SCAN** (built the mesh) + our augment→approve→upload (no START) | uploaded only | 10 `mesh_binary_*.ply`; log: `augmenting flight 'flight0065' (mesh age 185 s)` → `PRVW` → `Pilot APPROVED` → `UploadKmzFile OK 128770 B … 441.augmented.lean.kmz`. |
| **0066** | **OUR CUSTOM FLIGHT — route flown** | **YES (288 wp)** | `restored READY_TO_FLY (…441)` → `Fly tapped` → `Action(START)` → 427× `Mission: flying — waypoint N` up to wp 288. |
| **0067** | idle Smart3D-only boot, our app absent | no | only `APP name: Smart3DExplore`; no `kmz_runner`. |

**Correction to first guesses:** there is no separate "standard route re-flown for
comparison." The pairing is **0065 (DJI scan that mapped the building) ↔ 0066 (our
custom inspection over that same building).** 0064 = failed pre-flight, 0067 = later idle boot.

**Keep:** 0066 (our flight) + 0065 (scan mesh + the upload). **Deletable:** 0064 + 0067 (no flight).

## The flight itself (flight0066)

Two START attempts of the same 288-wp KMZ:
- **Run 1** — START 14:24:40, flew wp1→**139**, aborted/paused 14:26:47.
- **Run 2** — START 14:36:56 (after ~10-min gap), flew wp1→**288**, completed 14:42:11.

The abort-then-refly matches the pilot stopping for the jitter / overload.

## Root cause — heading jitter AND gimbal overload (one shared cause)

The augmented KMZ packs **~288 waypoints (~3 m spacing)**, each carrying **both** a
per-waypoint aircraft-heading change **and** a gimbal-rotate action.

**(a) Heading jitter — structural.** WaypointV3's default heading mode points the
nose toward the *next* waypoint. Our boustrophedon facade sweep zig-zags, so the
toward-next-waypoint heading flips at nearly every waypoint → an aircraft-yaw
setpoint change roughly **every ~1 s** (Run 2: 288 wp in ~295 s). That per-second
re-yawing is the jitter. DJI's own routes use far fewer, smoothly-ordered
waypoints, so they don't jitter.
→ **Fix:** fix/smooth the waypoint heading per facade pass; let the **gimbal** aim, not the airframe.

**(b) "Gimbal motor overload" — rate/duty-cycle, not bad angles.** Each waypoint
fires one+ gimbal-rotate actions. Run 2 alone = **~1150 action events in ~5 min ≈
3.9 gimbal events/s sustained, peaking 14–21/s.** Near-continuous slewing with
almost no dwell for ~5 min is the classic HMS "gimbal motor overload" (thermal /
duty-cycle) trigger. Angles are **not** out of range — the pipeline clamps to the
M4E soft limits (−90°…+35°).
→ **Fix:** fewer waypoints / gimbal actions per metre, add settle/dwell time, use continuous (smooth) gimbal mode.

**Both fixed by the same change:** thin the waypoints and decouple aircraft heading
from per-waypoint aiming.

## Telemetry map (for future debugging)

**Readable now (plaintext):**
- `flightNNNN/psdk/PSDK-NNNN-01.log` — full mission timeline: augment/approve/upload,
  `Fly tapped`, `Action(START)`, per-waypoint `mission state` (16 prep / 32 transfer /
  48 flying / 64 paused-or-done), gimbal/capture action callbacks (group/id, 1=start/5=finish).
  NUL-padded. Gives cadence/structure, **not angle values**.
- `dji_perception/*/mesh_binary_*.ply` — building point cloud (binary LE PLY), scans only.

**Holds the heading/gimbal VALUES but encrypted (DJI Assistant 2 / decryptor needed):**
- `dji_mcu/MCU-*.log.enc` — FC/gimbal stream: actual yaw, gimbal pitch/yaw/roll, HMS warnings incl. "gimbal motor overload."
- `diag/diag-*.log.enc` — full diagnostics (flight0066 = 22 MB, the long flight).

**Best source for COMMANDED angles (already archived here):**
`flown-kmz/20260612T122123Z_441.augmented.lean.kmz` → `wpmz/waylines.wpml` has the
exact per-waypoint heading + gimbal pitch/yaw. Parse consecutive deltas to quantify
both issues precisely. (This is the file the FC actually executed.)
