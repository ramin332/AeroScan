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

## Flights (slot numbers are NOT chronological — ring buffer)
| slot | size | mesh chunks | ours / standard? |
|------|------|-------------|------------------|
| flight0064 | 10M | 0 | TBD (see analysis) |
| flight0065 | 122M | 10 | Smart3D auto-exploration SCAN (has mesh) |
| flight0066 | 92M | 0 | TBD |
| flight0067 | 12M | 0 | TBD |

Known: the two relevant flights flew the **same physical route** — one DJI standard
auto-scan route, one our PSDK WaypointV3 custom mission. The identification is in
`ANALYSIS.md` (written by the analysis agent).

## TODO / off-machine backup
This is on the laptop only. Back it up off-machine (external/cloud) — the drone
churns `/blackbox`, so if the laptop is lost this data is gone.
