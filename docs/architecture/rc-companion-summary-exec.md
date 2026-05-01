# RC Wireless Mission Transfer — Executive Summary

**Date:** 2026-05-01
**Status:** Proof-of-concept complete. Recommending we **pause** this transport
and pivot to an alternative architecture for production.

---

## What we set out to do

Make it possible to fly a building-inspection mission planned in AeroScan
**without the pilot ever leaving the controller** — no SD-card sideload, no
"download KMZ → walk to controller → swap card → reboot Pilot 2 → import" loop.

Concretely: a small Android app on the DJI RC Plus 2 controller would let the
pilot pick a mission file (KMZ) and send it wirelessly to the on-drone computer
(DJI Manifold 3), which would then fly the mission.

## What we built

End-to-end wireless transport: **controller → drone radio → drone E-Port →
on-drone computer**. The pilot picks a file in our app, the file streams over
the air, and arrives at the drone-side application that we also wrote. We
verified it with a real mission file on real hardware today.

This is the first time we've demonstrated a non-SD-card path for getting
mission data onto a DJI drone in our setup.

## What we learned

**The good:**
- The wireless path works. We proved out every layer of DJI's "MOP" inter-app
  channel — registration, addressing the right device slot, framing, integrity
  check (MD5), clean disconnect. The drone-side app sees real bytes from the
  controller-side app.
- We documented four non-obvious DJI gotchas that cost us time on the way (app
  startup crashes, connection-flicker behavior, an addressing mismatch, and
  the pilot needing to "prime" the radio with DJI's own app once after
  power-up). All written up in `rc-companion-bringup.md` so we don't pay for
  them again.

**The blocker:**
- **Upload speed is hard-capped at ~5 KB/sec by the radio.** This is a
  documented physical-layer limit of DJI's OcuSync uplink, not something we
  can tune in software on either side. It's the same speed regardless of
  reliable vs. unreliable mode.

**What that means in practice:**

| Mission file size | Time to transfer over the radio |
|---|---|
| 50 KB | ~10 sec (acceptable) |
| 500 KB | ~1.5 min (borderline) |
| 1 MB | ~3 min (poor UX) |
| 5 MB | ~15 min (infeasible) |

A typical AeroScan inspection mission for a building of any meaningful size
falls into the multi-MB range once the full waypoint set, photo metadata, and
DJI's required `wpmz/` payload are bundled in. **At 5 KB/sec the pilot would
be sitting on the launch pad for minutes per mission.** That's worse than the
SD-card workflow we're trying to replace.

## Why we're pausing

This transport is not the right vehicle for the bulk-mission-file use case.
Continuing to invest engineering time in this layer (writing the file to disk
on the drone side, validating headers, wiring it into the autopilot's
"Waypoint V3" execution API) buys us a path that is **fundamentally too slow**
for the missions we actually generate. The rest of the work would be solid
engineering, but it would deliver a feature that, once shipped, the pilot
wouldn't choose to use.

## Recommended pivot

The right place to put the mission-generation logic is **on the drone itself**,
on the on-drone computer (Manifold 3). Instead of *generating a KMZ on the
controller and shipping it over the slow radio*, we'd ship a small command
("inspect the building footprint at GPS X, Y, Z with these parameters") and
let the on-drone computer **build the KMZ locally** and hand it to the
autopilot. The radio carries kilobytes of intent, not megabytes of waypoints.

This pivot:
- Sidesteps the OcuSync uplink ceiling entirely.
- Reuses the wireless transport we just proved out — for command/control
  payloads, where 5 KB/sec is plenty.
- Aligns with where DJI clearly designed their architecture to live: the
  Manifold has the CPU, storage, and direct E-Port-USB link to the autopilot;
  the controller is meant to be a thin pilot-facing surface.
- Lets us keep the entire AeroScan planning engine; it just runs on the drone
  rather than on a laptop.

## What we keep

- The Android RC app (`rc-companion/`) is on the shelf, building, and
  installable. It still has value for any future feature that needs the
  controller-to-drone wireless channel for *small* payloads — settings,
  start/stop commands, status pulls, telemetry tagging.
- The drone-side listener (`rc_probe.c` on the Manifold) similarly stays as
  a known-good template for any command-layer feature.
- The bring-up document captures every gotcha so re-engaging this transport
  later costs hours, not days.

## Decision needed

Confirm we're moving the mission-generation engine onto the Manifold 3 as the
primary architecture, with the radio path reserved for control messages only.
The technical proof-of-concept is done; this is a "where do we put our next
two weeks" question.
