# Flight archive — 2026-06-12 (first successful AeroScan custom WaypointV3 flight)

Pulled from the Manifold `/blackbox/` ring buffer (volatile — these slots get
reused/churned on the drone, so this copy is the preservation). Each `flightNNNN`
is one DJI M4E flight session (camera/, diag/, dji_mcu/, dji_perception/, psdk/, system/).

## Purpose
Debugging data for two issues observed during our custom flight:
1. **Heading jitter** — aircraft flew the route but jittery; appeared to change
   the aircraft *heading* (yaw), not just the gimbal.
2. **"Gimbal motor overload"** warnings — frequent during flight. Cause unknown
   (too many gimbal commands? bad angles? mechanical?).

## Flights (slot numbers are NOT chronological — real order: 64→65→66→67)
| slot | size | role | keep? |
|------|------|------|-------|
| flight0064 | 10M | our app ran, **augment FAILED** (stale mesh) — no flight | deletable |
| flight0065 | 122M | **Smart3D scan** (built the mesh) + our augment→upload (no START) | **keep** |
| flight0066 | 92M | **OUR CUSTOM FLIGHT** — flew 288 waypoints (2 START attempts) | **keep** |
| flight0067 | 12M | idle Smart3D-only boot, our app absent | deletable |

No separate "standard route" comparison flight exists. Pairing is **0065 (DJI scan
that mapped the building) ↔ 0066 (our custom inspection over it).** Full evidence +
the jitter/gimbal root-cause are in `ANALYSIS.md`.

## TODO / off-machine backup
This is on the laptop only. Back it up off-machine (external/cloud) — the drone
churns `/blackbox`, so if the laptop is lost this data is gone.
