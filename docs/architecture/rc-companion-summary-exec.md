# RC Wireless Mission Transfer — Executive Summary

**Date:** 2026-05-03 (supersedes 2026-05-01 version — see "What changed" below)
**Status:** Pivot complete. We are **not** transporting KMZ files over the radio. The Smart3D mesh is already on the Manifold; we read it from there, augment with gimbal aim, and push the modified KMZ back over the same wired path the dev workflow already uses.

---

## What changed since the previous version

The 2026-05-01 version of this document recommended **pausing the wireless transport and pivoting to "generate the mission on the drone."** That recommendation was based on three things we now know are wrong or moot:

1. We thought the KMZ had to come from the RC and be moved to the Manifold.
2. We measured 5 KB/sec over the radio and called it a blocker for the production payload.
3. We treated DJI's Smart Auto-Exploration outputs as opaque.

A read-only investigation of the Manifold's filesystem on 2026-05-03 changed all three:

- The Smart Auto-Exploration **3D mesh is already on the Manifold**, in plain `.ply` files at `/blackbox/the_latest_flight/dji_perception/1/mesh_binary_*.ply`. Per flight: ~1 GB across ~50 chunks of vertex+normal point clouds. We don't need to move it; we just need to read it.
- The Manifold's perception data is **denser than the curated cloud DJI puts in the KMZ** — 4,700 facades extracted vs. 1,651 from the same KMZ's `cloud.ply`.
- The flight plan (waypoints, gimbal angles, capture commands) **is not on the Manifold** — DJI keeps it encrypted in `expl_plan.bin.enc`. The flight plan stays in the KMZ on the RC, which the pilot already exports via USB-MTP cable to the laptop in the existing workflow.

So the architecture flips: **AeroScan reads the mesh from the Manifold (fast, wired), reads the flight plan from the KMZ on the RC (existing manual export), augments only the gimbal aim per waypoint, and pushes the modified KMZ back to the Manifold over the same wired path. The radio is not in the bulk-data path at all.**

---

## What we set out to do

Make it possible to fly a building-inspection mission planned in AeroScan
**without the pilot ever leaving the controller** — no SD-card sideload, no
"download KMZ → walk to controller → swap card → reboot Pilot 2 → import" loop.

## What we actually built and learned

**Working today:**
- Wireless RC ↔ Manifold path over OcuSync (proof-of-concept) — **kept only as a control-message channel**, not for bulk files. Sub-500 KB payloads in <2 minutes; fine for "fly mission X" commands and status pulls.
- The Android app on the controller (`rc-companion/`) registers MSDK V5 cleanly, binds the M4E, and pushes data through DJI's MOP channel to a Manifold-side passive listener (`rc_probe.c`). End-to-end verified.

**Validated by investigation (2026-05-03):**
- Manifold's `/blackbox/` directory keeps per-flight perception data in plain PLY format. 18 GB across 35+ flights. We have SSH access. **No file transport is needed for the mesh — we read it.**
- DJI maintains a `/blackbox/the_latest_flight` symlink that points to the most recent flight directory. **No flight-ID lookup needed.**
- The encrypted `expl_plan.bin.enc` is opaque, so the flight plan still has to come from the KMZ on the RC. The pilot's existing USB-MTP-to-laptop workflow handles that — manual but reliable.

**The core insight:** the right scope is **gimbal augmentation, not mission re-planning**. We take Smart3D's flight path as-is, extract facades from the Manifold mesh, and override only `gimbalPitchAngle` / `gimbalYawAngle` per waypoint to aim at the closest in-view facade. The drone re-flies a path it already knows how to execute.

## Production workflow (after pivot)

1. **In the field:** pilot flies a Smart Auto-Exploration mission as normal. Mesh accumulates in `/blackbox/the_latest_flight/`. Smart3D KMZ ends up on the RC's storage.
2. **At the laptop (depot, hangar, or anywhere with a USB cable):**
   - USB-C cable from the laptop into the M4E aircraft debugging port → Manifold appears at `192.168.42.120` (DJI-documented for PCs). Or, in the lab, use the existing Wi-Fi LAN.
   - Pilot connects RC to laptop via USB-MTP, drags the Smart3D KMZ off (existing workflow).
   - Click "Augment with NEN-2767 gimbals" in AeroScan. Backend reads the latest flight's mesh from the Manifold over the wired link, parses the KMZ for waypoints, computes which facade each waypoint should look at, and writes a modified KMZ.
   - Click "Push to drone" — backend SCPs the modified KMZ to the Manifold's `/open_app/dev/data/received/`.
3. **Pre-takeoff:** disconnect the cable. Power on aircraft. Pilot taps the AeroScan PSDK widget on Pilot 2's live-flight view → drone uploads the augmented KMZ via `DjiWaypointV3_UploadKmzFile` → `DjiWaypointV3_Action(START)` → drone flies the same path with cameras now aimed at facades.

## Why this beats the alternatives

- **Beats SD-card sideload** because nothing has to come off the RC and be physically swapped. The pilot's MTP cable to the laptop replaces the SD-card swap, the laptop pushes the augmented mission to the drone over a USB-C cable to the aircraft debug port, and Pilot 2 flies it.
- **Beats wireless transport** because OcuSync's ~5 KB/sec uplink would need 67 minutes for a 20 MB KMZ. Cable speed is several MB/sec — files in seconds.
- **Beats moving the planner to the Manifold** because we don't have to maintain a Python-on-Tegra-arm64 build. The laptop already has the planner.
- **Beats re-planning the mission from scratch** because the gimbal-augmentation pass is much simpler than NEN-2767 waypoint generation: same path, same actions, only camera angles change.

## What we keep from the proof-of-concept

- The Android RC-companion app (`rc-companion/`) stays buildable and on the shelf for future small-payload uses (status pulls, control commands, in-field tweaks). Not on the production critical path.
- The Manifold-side listener (`rc_probe.c` on the Manifold) gets extended in a follow-up PR to actually persist incoming KMZs to disk (today it only logs hex previews).
- The bring-up document (`rc-companion-bringup.md`) captures every Android- and PSDK-side gotcha so re-engaging this transport later costs hours, not days.

## Decision needed

Confirm we proceed with:
- **Source of truth:** mesh from Manifold (`/blackbox/the_latest_flight/dji_perception/1/`), flight plan from RC-exported KMZ.
- **Transport:** USB-C cable (laptop ↔ M4E aircraft debug port) for SCP push of augmented KMZ. LAN as the depot fallback.
- **AeroScan scope shift:** add a "gimbal-augment" pass on top of Smart3D missions instead of generating NEN-2767 inspection missions from scratch.

The technical proof-of-concept is done. This is now a "where do we focus the next two weeks" question — and the answer is on the gimbal-augmentation pass + the laptop-side ingester from the Manifold, not on more transport plumbing.
