# Flight analysis — 2026-07-10 (second custom WaypointV3 flight)

Evidence: plaintext `psdk/PSDK-*.log` in `/blackbox/flight0072` (NUL-padded —
read with `tr -d '\000'` / `LANG=C grep -a`), the two flown augmented KMZs, and
the ground-removal bench measurements. MCU/diag telemetry (`*.log.enc`) is
DJI-encrypted — not readable here.

## Flight identification

Both missions flew in **one power session, `/blackbox/flight0072`**. `flight0071`
is an app boot with no mission. A power-cycle creates the slot, not an app
restart, so both uploads + STARTs share flight0072.

**Operational gotcha, cost us time today:** `/blackbox` slot **directory**
mtimes lie. DJI's ring-buffer bookkeeping touches slot dirs without writing
flight data — on 2026-07-10, `flight0065`/`flight0066` showed recent dir mtimes
while **every file inside was from 2026-06-12**. Rank slots by **per-file**
mtime (`find <slot> -type f -newermt <date>`), not by `ls -dt`. The
`the_latest_flight` symlink was correct and is the reliable pointer. Also: a
live slot is still being written while you rsync it — re-pull after power-down.

## The flights themselves (flight0072)

Two missions, back to back, one session:

| # | uploaded | KMZ | WP | START |
|---|---|---|---|---|
| 1 | 11:40:31 | `20260710T093952Z_881.augmented.lean.kmz` | 144 | 11:40:43 |
| 2 | 11:57:38 | `20260710T095651Z_712.augmented.lean.kmz` | 399 | 11:57:46 |

Mission 2 was paused and resumed mid-flight: **PAUSE 12:02:59, RESUME 12:03:06**.
Both worked — first time PAUSE/RESUME has been flown on this airframe.

## What we set out to test, and the result

### 1. Ground-removal fix (commit `d4cd1cb`) — measured effect

The morning van scan exposed the ground bug: the old percentile height cut
(`ground_z + ground_skip_m`) destroyed 43% of a ~2.7 m van on 2.5°-sloping
asphalt and kept 28% of the tarmac, producing **29 ground facets over 75.4 m²**
— more ground area than van area (65.8 m²). The gimbal was aimed at tarmac.

The fix replaces the cut with two stages: `fit_ground_plane()` (iteratively
trimmed least squares, trims both tails, fitted before the polygon clip) +
`_reject_ground_facets()` (drops facets that are both near-horizontal AND close
to the fitted plane). Params `ground_clearance_m=0.4`,
`ground_facet_clearance_m=1.5`; `ground_skip_m` removed. Measured:

| scene | metric | before | after |
|---|---|---|---|
| van (bench fixture) | on-target | 65.8 m² | 63.8 m² |
| van (bench fixture) | off-target | 75.4 m² | **5.7 m²** |
| van (actually-flown cloud) | off-target | 75.4 m² | **11.2 m²** |
| Mijande | facets | 818 | 812 |
| Mijande | walls | 128 | **136** |
| Mijande | total area | 2098.5 m² | 2089.8 m² |

The van's off-target surface dropped ~7–13×; Mijande kept *more* walls than
before. The full suite is **153 passed**. Design + measurements:
`docs/superpowers/specs/2026-07-10-ground-plane-facade-fix-design.md`.

### 2. Gimbal-aim change flew as intended

Across the two missions the gimbal pitch median moved **−45.5° → −36.5°**;
horizontal-ish waypoints **20.1% → 36.1%**; yaw-heavy waypoints (>30°) **35.0%
→ 24.5%**. On-target aim held at **98.6%**. The aim points at structure, not
tarmac — the ground fix's intended effect, confirmed on the flown KMZs.

### 3. "Gimbal motor overload" — RESOLVED

The 2026-06-12 flight tripped HMS "gimbal motor overload" from near-continuous
slewing (~3.9 gimbal events/s sustained). On 2026-07-10 there were **ZERO
occurrences** across both missions. Considered resolved — the dedup + heading
changes that landed since have taken the gimbal duty cycle below the trip point.

### 4. PAUSE / RESUME — first flight, both worked

Mission 2 was paused at 12:02:59 and resumed at 12:03:06; the aircraft honoured
both. This is the first airborne test of the PAUSE/RESUME switch widget
(previously ground-tested idle-only). See the open item below — RESUME behaved
oddly in the callback stream and needs investigation before we trust it.

## Open items (NOT fixed — do not claim otherwise)

1. **RESUME may restart at waypoint 1.** After RESUME at WP 396, DJI's own
   callback emitted `mission state: 80, index: 1` then `index: 2`, and
   **actions fired at WP 2 (photos taken)** before it continued at 396. It is
   unclear whether the aircraft physically flew back to WP 2 or whether this is
   only a callback/index artifact. **Investigate flight0072 diag telemetry
   before trusting PAUSE mid-mission.**

2. **Fly widget inert while paused.** The Fly widget logged `fly tap ignored —
   state=0` while the mission was paused; the pilot tapped it 3× before finding
   the separate Resume switch. The Fly and Resume affordances need to be
   unambiguous in the paused state.

3. **Mesh staleness gate did NOT fire.** No Smart3D scan ran today; both
   missions used **flight0070's mesh from the previous evening (~15 h old)**,
   despite `AEROSCAN_MESH_MAX_AGE_S=21600` (6 h). The 6 h staleness gate should
   have refused a 15 h mesh and did not. Investigate why the gate passed a stale
   mesh — a wrong mesh silently flown is the worst failure mode.

4. **`ground_clearance_m` / `ground_facet_clearance_m` are hardcoded.** They are
   not yet plumbed into the `fd_*` UI chain, which the CLAUDE.md dev-platform
   rule requires for any user-facing knob. `ground_skip_m` was the only
   facade-detection knob missing from that chain before; the new pair inherits
   the gap.

5. **Low photo count.** Each intent WP carries a 5-pose `smart_oblique_poses`
   rosette, but the augmented KMZ emits **one** `takePhoto` per WP — 143 photos
   over a ~220 m path (1.54 m inter-shot) where DJI Smart3D shoots ~0.65 m
   spacing. We are collapsing the rosette to a single frame. This is an
   augmenter/`kmz_builder` issue, not a facade-detection one.

6. **Heading/gimbal coupling.** The airframe heading barely turns (399-WP
   mission: heading Δ **median 2.3°**), so the required gimbal yaw can exceed
   the gimbal's pan range and the gimbal clamps/locks, pointing the wrong way.
   The aim held on-target 98.6% here, but the coupling is a latent failure mode:
   a facade off the airframe's beam can demand more yaw than the gimbal has.

## Architecture reminder (so future readers don't misattribute bugs)

`augment_mission` does **not** generate the flight path. It takes DJI Smart3D's
waypoints from the RC-exported KMZ (`intent.json → waypoints[]`,
`mission_area_wgs84`) and rewrites **only** gimbal pitch/yaw + aircraft heading.
The spiral-around-the-target pattern is DJI's. The ground fix, the gimbal-aim
change, and the heading rewrite all change **where the gimbal looks / where the
nose points**, never **where the aircraft flies**. Low photo count and slow
flight are not facade-detection bugs.

## Evening addendum — a fourth flight, and the real bug

A fourth mission flew at 12:36 (`flight0073`, `20260710T103507Z_331`, 398 WPs,
paused at WP 398). Its photos are folder `DCIM/DJI_202607101133_009` on the SD
card — **the camera clock runs 1 h behind the PSDK log** (11:33–11:41 on the card
= 12:33–12:41 local), and DJI writes our waypoint label into the filename
(`..._V_wp375.JPG`) because `kmz_builder` names each `takePhoto`. Pair photos to
waypoints by that label, never by order: 294 photos cover waypoints 0–376, so ~80
shots never fired (0.58 s legs against a 0.5 s minimum shutter interval).

### RESUME is fine — the earlier concern is withdrawn

The FC reported `index: 1` and `index: 396` **61 ms apart**. Those waypoints are
**46.7 m** apart — 23 s of flying at 2 m/s. The aircraft never went there. It is a
callback artifact. **PAUSE/RESUME is safe to use.**

### The gimbal was not honouring our yaw command

From the 294 photos' XMP (`drone-dji:GimbalYawDegree`, `FlightYawDegree`):

| | commanded | actual |
|---|---|---|
| gimbal pan (\|GimbalYaw − FlightYaw\|) | 0° everywhere | median **51.5°**, p90 **61.0°**, 41/294 at the ±60° stop |
| bearing error to the target | **17.6°** | **35.4°** |
| \|heading − commanded heading\| | — | median 14.4° |
| corr(commanded pitch, actual pitch) | — | **+0.40** (no better than never moving) |

We set `gimbalYawRotateAngle = waypointHeadingAngle` with `gimbalHeadingYawBase=north`,
so the *commanded* pan was 0° at every waypoint. The airframe tracked its commanded
heading to within 14° median, so the gimbal should have sat ~14° off the nose. It sat
at 51°, pinned against its mechanical stop, **44° from the yaw we asked for** — and
our commanded pose pointed at the target twice as accurately as the gimbal ever
achieved. The planner was right; the aircraft did not comply. Yaw is the only gimbal
axis with a travel limit, hence the pilot's "ends up 45° left or right and locks",
and why lap two looked worse: it was already at the stop.

**Fix (`2bf3308`): emit no gimbal yaw command.** `gimbal_yaw_deg=None` leaves yaw
uncommanded, the gimbal follows the nose, and required pan is identically zero — it
cannot saturate. Azimuth comes from the heading, which works. Pitch is still
commanded. `schedule_headings()` (rate-limited heading pursuit with a pan cap) is
implemented and tested but only used when `command_gimbal_yaw=True`, the path to
re-enable if a device test ever shows absolute-north yaw *is* honoured.

### "In frame" is not "aimed" — a wide lens hides bad tracking

The pilot reported this flight looked good: *"it was looking at the car the entire
time."* Both that and the 35.4° median bearing error are true. The WIDE lens has a
**35.8° horizontal half-FOV**, so at the median error the car sits just inside the
frame edge — continuously visible, never centred. At p90 (91.7°) it is out of frame
entirely. The FPV feed is a poor instrument for aim quality; the XMP is the
instrument. Same trap as the 399-WP mission reading "99.7% on-target" while shooting
at 9.23 mm/px.

### Hypotheses tested and killed (do not re-walk these)

- **Gimbal mid-slew at shutter** — `corr(commanded step, error) = −0.03`, and the
  gimbal demonstrably reaches 96.6°/s. It has time and speed to arrive.
- **`gimbalRotateTime=10`** — present on 1 of 207 actions, and disabled
  (`gimbalRotateTimeEnable=0`). Dead config in `kmz_builder.py:406-407`.
- **Photo↔waypoint pairing offset** — no shift in −6..+30 reduces the error.
- **Rigid frame rotation (ICP yaw error)** — resultant length R=0.656, not rigid.
- **"Half the waypoints are blind"** — no. `MissionConfig`'s 5°/5° dedup omits a
  `gimbalRotate` when the pose barely moved; the gimbal holds its last pose. Every
  waypoint resolved a facade. That dedup is what cured the 2026-06-12 overload.

### Data provenance

The 294 photos are the only record of this bug and live on the SD card, not in
`/blackbox` (whose `camera/` directory is empty). Copy them to
`flight-archive/2026-07-10/photos-flight0073/` before the card is reused; without
them, "the fix worked" is unfalsifiable.

## Verdict

- Ground fix: **works**, measured on both bench and flown clouds.
- Facade-picker hysteresis (`aee6451`): **works**, 45→24 target switches, all
  >150° aim reversals gone on the close orbit.
- Gimbal motor overload: **resolved** (zero occurrences).
- PAUSE/RESUME: **flew, and is safe** — the RESUME index concern is withdrawn.
- **Gimbal yaw was never being honoured.** Fixed in `2bf3308`; **unflown**. The next
  flight's photos, through `scripts/read_gimbal_xmp.py`, are the test: success is
  pan near 0° and bearing error falling toward the airframe's own ~14° heading error.
- Still open: the C mesh resolver hardcodes `dji_perception/1`; GSD is unvalidated at
  plan time (this mission: 9.23 mm/px against a 2.0 target); range-adaptive lens.
- **Low photo count is closed as not-a-bug** — DJI's rosette serves multi-view stereo;
  NEN-2767 wants one perpendicular frame, and along-path overlap is already ~84%.
