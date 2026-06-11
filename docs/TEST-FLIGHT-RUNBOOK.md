# TEST FLIGHT RUNBOOK — first AeroScan augment→fly (offline)

> Keep this open on your phone. **No internet in the field** — everything here is self-contained.
> Decision locked: **full chain incl. START**, **laptop carried as cable fallback**.

## What we are testing (priority order)
1. **GO/NO-GO:** does the M4E accept a PSDK `DjiWaypointV3` upload + `Action(START)` and actually fly it? (never done; M4E not explicitly listed for WaypointV3 in DJI docs)
2. Does a **fresh Smart3D scan** land a readable mesh on `/blackbox` and get consumed by the on-Manifold augment? (live data, not canned Mijande)
3. Do **FC safety interlocks** (battery / GPS / home / RTH) fire for a WaypointV3 START? (undocumented — observe)

## THE iron rule
**Do NOT power-cycle the aircraft between the scan and the augment.** A power-cycle creates a new empty `/blackbox` slot and the ~30-slot ring buffer churns the fresh mesh. App-switching in Pilot is safe (no new slot). Scan → land → keep powered → switch app → augment.

---

## Tonight at the depot (LAN + internet — your only chance to verify with tools)
- [ ] Run `bash scripts/preflight_check_manifold.sh` — confirm: disk headroom on `/blackbox` (>~2 GB free), DPK `psdk-demo` installed, app process reachable.
- [ ] Confirm **which APK is on the RC**. Want the **May 25 11:44** build (`rc-companion/app/build/outputs/apk/debug/app-debug.apk`) — it has the readiness banner. The May 1 `lastgood` does NOT. Sideload if wrong.
  - Known gap: installed APK lacks the `a37fce1` fail-fast hard-block (can't rebuild — no gradle/java on laptop). Banner still shows red/green; you just won't be hard-blocked from tapping Augment with no mesh. Acceptable.
- [ ] Charge: aircraft batteries (≥2), RC, laptop. SD card in aircraft (Smart3D writes there + fallback).
- [ ] Laptop fallback ready: AeroScan webapp runs offline (`./run.sh`, backend :8111 / frontend :3847). USB-MTP cable to pull the Smart3D KMZ off the RC. SD card to sideload a fallback KMZ into Pilot 2.

## Field procedure
**Stage 1 — Scan** (proven, low risk)
- Power on → boot app `Smart3DExplore` → fly Smart Auto-Exploration over the building → land → **KEEP POWERED.**

**Stage 2 — Switch to AeroScan**
- Pilot → Application Management → switch active app to **psdk-demo (AeroScan)**.
- Expect: **Fly widget appears** + readiness **banner goes GREEN ("mesh ✓")**. Green here = mesh blocker resolved on live data.

**Stage 3 — Augment** (proven on bench)
- Open rc-companion → pick the Smart3D `.kmz` from RC filesystem → **Augment**.
- Wait ~2–4 min. Preview (PRVW) returns: # waypoints, # facades, # suspect camera poses.

**Stage 4 — Approve + upload** (proven, MD5-verified on bench)
- Review preview stats → **Approve** → EXEC → `UploadKmzFile` → state = `READY_TO_FLY`.

**Stage 5 — GO/NO-GO (the real test)**
- Conditions: open area, clear of people/obstacles, full battery, solid GPS/RTK.
- **Pilot on the sticks. Know the abort path BEFORE tapping: Pilot 2 pause / RTH, and/or `Action(STOP)`.**
- Pilot 2 live view → tap **AeroScan: Fly** → `Action(START)`.
- Watch floating window:
  - `"Mission: flying — waypoint N"` → **GO.** Let it fly; watch camera aim per waypoint.
  - **Refusal** → **NO-GO.** Do not force. **Write down the FC validity error code** (no internet to look it up — record verbatim for next session). A clean refusal is still a successful test result.

## If something breaks (no internet — these are your only moves)
| Symptom | Move |
|---|---|
| Banner stays RED / "no mesh" after switch | Mesh didn't land or got churned. Confirm you did NOT power-cycle. Re-fly a short Smart3D scan, switch, retry. |
| Augment fails at `[2/7]` | Same as above — no mesh. |
| PRVW never returns | MOP transport stalled. Re-open rc-companion, retry Augment. If persists → **laptop cable fallback** (below). |
| Approve stuck on "Approving…" | Known transition bug fixed in `9aa9b53`; if old APK, restart app + retry. |
| START refuses with error code | Record the code. NO-GO. End test; this is a documented outcome. |
| Whole PSDK/radio chain unusable | **Laptop fallback** (below). |

## Laptop cable fallback (bypasses Manifold + PSDK entirely)
Proven "zero-PSDK, SD-card" path:
1. USB-MTP cable RC → laptop. Copy the Smart3D `.kmz` to laptop.
2. `./run.sh`, open frontend, import the Smart3D KMZ (it carries a point cloud) → generate NEN-2767 inspection mission → export KMZ.
3. Write KMZ to SD card → insert in aircraft → Pilot 2 → import into mission library → fly normally (no widget, no PSDK).
This skips the Manifold mesh and uses the KMZ's own cloud — lower fidelity, but it flies.

## Record for next session (write in notes — no internet to save it live)
- Did mesh land + banner go green? (blocker #2)
- Augment time, # waypoints/facades, # suspect poses.
- **START accepted or refused? If refused, the exact FC error code.** (blocker #1 — THE result)
- Did FC interlocks behave (battery/GPS/RTH)? (blocker #3)
- Any camera mis-aim seen during flight.
