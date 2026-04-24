# KMZ Flow: Authoring → Drone Execution

How AeroScan-generated KMZ missions get onto the DJI Matrice 4E, and what DJI's
PSDK actually supports for that last-mile step. Last researched 2026-04-24.

## TL;DR

- Today's production path is **manual sideload**: AeroScan produces a WPML-compliant
  KMZ, the user downloads it, copies it to the RC via SD card, and picks it from
  Pilot 2's mission library. This works today and is what `kmz_builder.py` targets.
- There is **no PSDK API** to inject a KMZ into Pilot 2's mission library. User-facing
  import is SD-card only.
- The DJI-sanctioned on-drone automation path is **`DjiWaypointV3_UploadKmzFile` +
  `DjiWaypointV3_Action(START)` from a Manifold-hosted PSDK app**, with a PSDK
  Custom Widget for pilot control from Pilot 2's live-flight view. This bypasses
  Pilot 2's mission library entirely — Pilot 2 becomes the monitoring surface, not
  the mission chooser.
- Whether DJI's built-in **Smart 3D Capture / Explore** writes an interceptable
  KMZ/WPML after a scan is **not confirmed by any public doc**. Needs a device test
  before any "Smart3D-first" architecture is committed to.

## What DJI's PSDK Supports

### Confirmed (from PSDK headers + tutorials)

| Capability | API / Surface | Notes |
|---|---|---|
| Upload KMZ to aircraft | `DjiWaypointV3_UploadKmzFile(bytes, len)` | From PSDK app on Manifold; aircraft accepts the file directly. |
| Execute / control mission | `DjiWaypointV3_Action(START \| STOP \| PAUSE \| RESUME)` | Mission control from Manifold. |
| Pilot-facing controls | PSDK Custom Widget | Rendered on Pilot 2's live-flight screen only. Buttons / status / text. |
| PSDK ↔ MSDK/OSDK peer comms | `dji_mop_channel.h` (MOP channel) | Peer apps, not Pilot 2. |
| Sample reference | `samples/sample_c/module_sample/waypoint_v3/` in PSDK source | End-to-end "upload + fly" pattern. |

### Not supported

- **No PSDK API to inject into Pilot 2's mission library.** The Waypoint V3 upload
  goes to the aircraft, not Pilot 2's library. User-facing library import is manual
  SD card only.
- **Widgets are live-link only.** A widget renders while the PSDK app is running
  with the aircraft powered and linked. It is **not** visible on Pilot 2's
  pre-flight mission library screen.
- **MOP is not a Pilot 2 back-channel.** It connects PSDK to MSDK/OSDK peer apps,
  not to Pilot 2.

### Unverified

- Does **Smart 3D Capture / Explore** emit a WPML/KMZ to a readable location on the
  RC or aircraft filesystem after the scan?
  - Nothing in DJI's public docs (product pages, Heliguy writeups, PSDK/Cloud-API
    repos) describes Smart 3D as exporting a KMZ into Pilot 2's mission library.
    It is documented as a single end-to-end automated mission: sparse cloud on
    first pass → on-RC facade route derivation → autonomous flight.
  - **Until a device test confirms an accessible file, treat Smart3D-as-KMZ-source
    as an open question.**

## Two Candidate "Source of KMZ" Architectures

### A — AeroScan-first (current)

```
Building geometry (preset / GeoJSON / uploaded mesh / imported DJI Smart3D cloud)
    ↓  AeroScan web app (local, Mac/Linux/Windows)
Waypoint grid + validation + KMZ (WPML)
    ↓  download
    ↓  sideload via SD card → RC
Pilot 2 mission library → fly
```

This is what the codebase implements end-to-end today. `kmz_import.py` already
accepts a DJI Smart3D KMZ containing a point cloud as *input* to the planner, so
there is a partial B→A bridge for the "Smart 3D gave me a point cloud, now plan an
inspection over it" use case.

### B — Smart3D-first (conditional)

```
Smart 3D Capture (autonomous first pass)
    ↓  ??? — does this emit a KMZ we can read?
AeroScan processes it (perpendicular alignment / uniform GSD / NL-SfB tags)
    ↓
Refined KMZ → execution transport (see below)
    ↓
Aircraft flies inspection pass
```

This architecture depends on the Smart 3D run leaving a readable WPML/KMZ on the RC
or aircraft. **Device test gates this path.** If Smart 3D does not expose a file,
the alternatives are (i) an on-Manifold "first pass" scanner we build ourselves, or
(ii) staying on architecture A.

## The Supported On-Drone Execution Shape

Given the PSDK surface, "tap a widget on Pilot 2 → fly the AeroScan-processed
mission" is only possible via a Manifold PSDK app. The flow:

1. AeroScan-processed KMZ lands on Manifold. Transport is arbitrary — USB, ADB,
   MQTT, HTTP pull, Manifold-side build. AeroScan doesn't care, Manifold does.
2. Pilot powers the aircraft and links to the RC. Pilot 2 boots with the PSDK
   widget visible on the live-flight screen.
3. Pilot taps the "Fly AeroScan mission" button in the widget.
4. The PSDK app on Manifold calls `DjiWaypointV3_UploadKmzFile(bytes, len)` with
   the processed KMZ, then `DjiWaypointV3_Action(START)`.
5. Aircraft executes the mission. Pilot 2 keeps showing FPV + telemetry + widget
   status throughout.

**UX trade-off.** In this model, Pilot 2 is the monitoring surface, not the
mission chooser. The pilot cannot browse / preview / edit the KMZ from Pilot 2's
mission library — they tap a widget and go. This is the DJI-sanctioned automation
flow; the pre-flight library path is closed to PSDK.

## What This Means for AeroScan

- The web-app pipeline described in `CLAUDE.md` is the **authoring surface** —
  building geometry in, WPML-compliant KMZ out. That doesn't change.
- The KMZ needs an **execution transport** to reach the aircraft. Two options:
  1. **Manual sideload (today).** Pilot downloads the KMZ from AeroScan, copies
     it to the RC via SD card, picks it from Pilot 2's mission library, flies.
     Zero PSDK work. Zero Manifold. This is the current supported path.
  2. **Manifold + PSDK widget (future).** A PSDK app on Manifold receives the
     KMZ (via whatever transport), renders a widget, uploads via `WaypointV3`
     on demand. Skips sideload and the library screen entirely.
- The Smart3D-as-source story is orthogonal to both of the above and gated on a
  device test.

## Open Questions

1. **Does Smart 3D Capture emit a readable KMZ?** Run a Smart 3D Capture on the
   M4E, check the RC + aircraft filesystems for `.kmz` / `.wpml`. Single test,
   resolves the upstream question.
2. **Which execution transport do we target?** Manual sideload (works today, zero
   engineering) vs. Manifold PSDK app (better UX, real engineering cost). Product
   decision, not a docs one.
3. **Where does the processed KMZ live between "AeroScan produced it" and
   "Manifold needs it"?** If / when we commit to the Manifold path, pick a
   transport: HTTP pull from AeroScan, MQTT push, USB sideload to Manifold, etc.

## Sources

- `dji_waypoint_v3.h` — PSDK C header, `DjiWaypointV3_UploadKmzFile` +
  `DjiWaypointV3_Action`
- PSDK Waypoint Mission tutorial
- PSDK Custom Widget tutorial
- `dji_mop_channel.h` — PSDK ↔ MSDK/OSDK peer channel (not Pilot 2)
- Heliguy: M4E Smart 3D Capture workflow writeup
- DJI WPML format reference
