# RESUME HERE — AeroScan fly-readiness (last updated 2026-07-10, evening)

> Single entry point for picking up the on-drone augment→fly work next session.
> Read this first, then the linked detail docs. Everything is merged to `main`.
> On the Manifold, `cd /open_app/dev`.

## One-line status

**IT FLIES, AND WE NOW MEASURE WHAT IT ACTUALLY DID.** Four missions have flown
(2026-06-12; 2026-07-10 morning + two afternoon + one evening). The day's real
lesson: we spent it tuning what we *ask* the aircraft for, and only at the end read
the JPEG XMP to see what the aircraft *does*. The commands were already better than
the execution. **Instrument before optimising.**

## The ONE next action — reflight and re-measure the gimbal

Deploy `2bf3308` and fly. Then pull the photos and run:

```bash
.venv/bin/python scripts/read_gimbal_xmp.py <photo-dir> --kmz <the-flown>.augmented.lean.kmz
```

**Success = gimbal pan near 0° and bearing error dropping toward the airframe's own
heading error (~14°).** Today it was pan median 51.5°, p90 exactly 61.0° (pinned at
the ±60° stop), and the gimbal sat 44° from the yaw we commanded.

⚠️ **"In frame" is not "aimed."** The pilot reported the last flight looked good —
*"it was looking at the car the entire time"* — and the median bearing error was
35.4°. Both are true: the WIDE lens has a **35.8° horizontal half-FOV**, so the
target sits just inside the frame edge, continuously visible and never centred. At
p90 (91.7°) it is out of frame entirely. **The FPV feed cannot judge aim quality.
The XMP can.** Same trap as the 399-WP mission reading "99.7% on-target" while
shooting at 9.23 mm/px.

⚠️ **Photos are the only record of the bug and they are NOT archived.** They live on
the SD card (`DCIM/DJI_202607101133_009`), not in `/blackbox`. Copy them to
`flight-archive/2026-07-10/photos-flight0073/` before the card is reused, or "the
fix worked" becomes unfalsifiable.

## What 2026-07-10 established (with evidence)

**The gimbal was not honouring our absolute-north yaw command.** From 294 photos'
XMP (waypoint index recovered from the DJI filename, `_wp375.JPG`):

| | commanded | actual |
|---|---|---|
| gimbal pan (\|GimbalYaw − FlightYaw\|) | 0° everywhere | median **51.5°**, p90 **61.0°**, 41/294 at the stop |
| bearing error to the target | **17.6°** | **35.4°** |
| \|heading − commanded heading\| | — | median 14.4° |
| corr(commanded pitch, actual pitch) | — | **+0.40** (no better than never moving) |

The planner was right; the aircraft did not comply. Yaw is the only gimbal axis with
a mechanical limit, so it is the only one that can saturate — hence the pilot's
"ends up 45° left or right and locks", and why lap two looked worse (it was already
at the stop). **Fix (`2bf3308`): emit no gimbal yaw command.** `gimbal_yaw_deg=None`
→ the gimbal follows the nose, required pan is identically zero. Azimuth comes from
the heading, which works. Pitch is still commanded.

`schedule_headings()` (rate-limited heading pursuit with a pan cap) is implemented and
tested but **only used when `command_gimbal_yaw=True`**. If a device test ever shows
absolute-north yaw *is* honoured, that is the path to re-enable — not the old
`heading := bearing` weld.

## Disproved today (do not re-litigate without new evidence)

- **Downward normals (`normal_z < −0.7`) are a ground signature** — no. Mijande has
  171 of them and is correct. Normals come from SVD; the classifier uses `abs(nz)`.
- **Gimbal mid-slew at shutter** — no. `corr(commanded step, error) = −0.03`, and the
  gimbal reaches 96.6°/s. It has time and speed to arrive.
- **`gimbalRotateTime=10` in the WPML** — dead config. Present on 1 of 207 actions and
  disabled (`gimbalRotateTimeEnable=0`). See `kmz_builder.py:406-407`.
- **Photo↔waypoint pairing offset** — no shift in −6..+30 reduces the error.
- **Rigid frame rotation (ICP yaw error)** — resultant length R=0.656, not rigid.
- **RESUME restarts at waypoint 1** — no. The FC reported index 1 and index 396
  **61 ms apart**; they are **46.7 m** apart (23 s of flying). A callback artifact.
  **PAUSE/RESUME is safe to use.**
- **"Half the waypoints are blind"** — no. `MissionConfig`'s 5°/5° dedup omits a
  `gimbalRotate` when the pose barely moved; the gimbal holds its last pose. Every
  waypoint in both afternoon missions resolved a facade. **That dedup is what cured
  the 2026-06-12 gimbal-motor-overload. Leave it alone.**
- **"No Smart3D scan ran on 2026-07-10"** — it did. 13 mesh chunks, under
  `dji_perception/`**`2`**`/`. Everything was globbing subdir `1`.

## Open items

| | Item | Owner |
|---|---|---|
| ✅ | First M4E WaypointV3 upload+START — flew 2026-06-12 + 2026-07-10 | done |
| ✅ | Gimbal motor overload — zero occurrences since | resolved |
| ✅ | Ground removal on short targets — plane fit + facet gate (`d4cd1cb`) | resolved |
| ✅ | Ground params in the UI — `fd_*` chain + sliders (`3579c52`) | resolved |
| ✅ | RESUME index behaviour — callback artifact, PAUSE is safe | resolved |
| ✅ | Facade-picker flip-flop — hysteresis, `switch_ratio=0.8` (`aee6451`) | resolved |
| 🔴 | **Verify the gimbal fix in the air** — fly `2bf3308`, then `read_gimbal_xmp.py` | you |
| 🔴 | **C mesh resolver still hardcodes `dji_perception/1`** (`aeroscan-psdk` `kmzrun_status.c:37,95`). Python side fixed (`fcbad5f`). Until a DPK rebuild ships, force the slot with `AEROSCAN_FLIGHT_ID=flightNNNN` or the augment silently plans against a stale mesh — which is exactly what happened twice on 2026-07-10. | you |
| 🔴 | **GSD is unvalidated at plan time.** The 399-WP mission shot at 33.8 m → **9.23 mm/px** against `target_gsd_mm_per_px = 2.0`. Perfectly aimed, unusable for defect work, and nothing flagged it. A plan-time gate in `validate.py` is ~3 lines. | you |
| 🟡 | **Range-adaptive lens.** M4E has WIDE/MEDIUM_TELE/TELEPHOTO (`models.py:50-81`); we hardcode WIDE. At 33.8 m, MEDIUM_TELE gives **2.12 mm/px** vs WIDE's 9.23 — a 4.4× gain at zero flight-time cost. **Gated on a device test:** can M4E WPML direct a single `takePhoto` to a chosen lens? Smart3D's `template.kml` declares `<wpml:imageFormat>visable</wpml:imageFormat>`; we emit none. | you |
| 🟡 | Fly widget inert while paused (`fly tap ignored — state=0`); pilot tapped 3× before finding Resume | you |
| 🟡 | Mesh staleness gate — root cause was the subdir bug above, not the gate. Re-verify once the C resolver is fixed. | you |
| ⛔ | **Low photo count is NOT a bug.** DJI's 0.65 m spacing + 5-pose rosette serves multi-view stereo. NEN-2767 wants one sharp perpendicular frame per surface, and at 1.54 m spacing along-path overlap is already ~84%. Collapsing the rosette is deliberate. Only revisit if the deliverable becomes a 3D reconstruction. | closed |
| 🟢 | Build the rc-companion in Android Studio (committed, UNBUILT — no gradle on the laptop) | you |
| 📋 | Mirror doc updates into the Manifold `/open_app/dev/docs/` + `INDEX.md` | not done |

## Architecture fact that keeps getting forgotten

`augment_mission` **does not generate the flight path.** It ingests DJI Smart3D's
waypoints from the RC-exported KMZ and rewrites only gimbal pitch/yaw, aircraft
heading, and the per-waypoint actions. The spiral/orbit is DJI's. Facade detection
changes where the gimbal *looks*, never where the aircraft *flies*. Uniform GSD needs
our own trajectory (`geometry.py` has the boustrophedon planner) — a big move, gated
on re-validating safety layers 9–12.

## Instruments (use these before theorising)

- `scripts/read_gimbal_xmp.py` — **actual** gimbal/airframe angles from JPEG XMP,
  diffed against the flown KMZ. Reports pan beyond ±60° and whether error grows
  through the flight. The WPML is intent; only the XMP is truth.
- `scripts/verify_augmented_kmz.py` — action counts, aim, pose/heading smoothness on
  any WPML KMZ. Offline, no cloud needed.
- `scripts/pull_flight_archive.sh` — whole `/blackbox` slots. `pull_flight_debug.sh`
  only takes PSDK logs + mission state and hits a permissions wall on the DPK log
  (`dji` is not in group `nvidia`).

## Resume commands

```bash
# SSH to the Manifold
ssh dji@192.168.1.55

# DEV mode (raw binary, readable logs, fast iterate) — stops the prod apps + runs ours:
cd /open_app/dev && ./run.sh            # foreground; Ctrl-C to stop; reboot/Pilot re-tap restores prod
#   logs: /open_app/dev/Payload-SDK-3.16.0/build/data/logs/latest.log

# PRODUCTION (DPK): build emits build/dpk/psdk-demo_v01.00.00.00.dpk
dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk   # no sudo
#   then start/Set-Auto-start in Pilot → Application Management; logs via Pilot Log Export
#   (DPK runs as user apppsdk-demo — dji can't read its files; journalctl -u psdk-demo.service for console)

# Is a mesh present on the latest flight?  (subdir is NOT always 1 — see gotchas)
find /blackbox/the_latest_flight/dji_perception -name 'mesh_binary_*.ply' 2>/dev/null | wc -l

# Which slot actually holds today's data? Directory mtimes LIE. Use per-file mtime.
for d in /blackbox/flight[0-9]*; do
  n=$(find "$d" -type f -newermt "$(date +%Y-%m-%d) 00:00" 2>/dev/null | wc -l)
  [ "$n" -gt 0 ] && echo "$(basename $d) files_today=$n meshes=$(find "$d/dji_perception" -name 'mesh_binary_*.ply' 2>/dev/null | wc -l)"
done
```

## Key gotchas (learned the hard way)

- **`/blackbox` is a ~30-slot ring buffer** that cycles; a **power-cycle** creates a
  new flight slot (app updates don't). The mesh is evicted as it cycles → **augment
  right after scanning**, and don't burn reboots in between.
- **Slot DIRECTORY mtimes lie.** DJI's bookkeeping touches them without writing flight
  data. On 2026-07-10, `flight0065`/`flight0066` showed same-day dir mtimes while every
  file inside was from 2026-06-12. Rank slots by **per-file** mtime
  (`find <slot> -type f -newermt <date>`), never `ls -dt`. The `the_latest_flight`
  symlink was correct; the mtimes were not. (Silver lining: that misreading recovered
  the 2026-06-12 first-flight data, which we thought lost.)
- **The perception mesh is not always under `dji_perception/1`.** Observed: `1` for
  flight0065/0070, `2` for flight0066/0072. Globbing `1` made a fresh scan invisible
  and the augment silently planned against a 15-hour-old mesh — twice. Search
  `dji_perception/*/` and take the newest chunk. Python fixed (`fcbad5f`); the C
  resolver in `aeroscan-psdk/kmzrun_status.c:37,95` is **still hardcoded**.
- **A live slot is still being written while you rsync it.** Re-pull after power-down.
- **Photos are not in `/blackbox`** (`camera/` is empty). They live on the SD card,
  in `DCIM/DJI_<timestamp>_NNN/`, and the **camera clock runs 1 h behind** the PSDK
  log (UTC vs CEST). DJI encodes our waypoint index in the filename (`..._V_wp375.JPG`)
  because `kmz_builder` labels each `takePhoto` — pair by that, never by order.
- **Dev vs prod:** raw binary (`./run.sh`, user `dji`, readable logs) = debugging;
  **DPK = production** (DJI-recommended; raw-exec is flagged unstable on M4E). Only
  **one app holds the E-Port** at a time — switch in Pilot.
- **No root** on the Manifold (`dji` not a sudoer). DPK install + Pilot need no sudo.
- **Never `pkill -f dji_sdk_demo_on_manifold3` over SSH** — it self-matches and kills
  your session. Use `pkill -x dji_sdk_demo_on` or kill by PID.
- The DJI floating window caps messages at **255 bytes**; widget configs load from a
  path **relative to the binary** (`../widget_file`), bundled via `app.json` `userconfig`.

## Detail docs (read for depth)

- `docs/architecture/manifold-deployment.md` — canonical build/package/install/run/log.
- `docs/architecture/kmz-flow.md` — KMZ execution transport + what `Action(START)` checks.
- `docs/superpowers/plans/2026-05-25-aeroscan-mission-cockpit.md` — cockpit phases + status.
- `docs/superpowers/plans/2026-05-25-manifold-readiness-handshake.md` — PING/STAT + RC banner.
- `CLAUDE.md` → "KMZ Execution Transport" + the `/blackbox` sections.

## Repos & today's commits

- **aero-scan** (laptop, this repo) — rc-companion Kotlin + docs. Today: `1a56e2c`
  (banner), `e66b85c`/`7de5404`/`8738809` (plans+docs), `a37fce1` (fail-fast).
- **aeroscan-psdk** (Manifold `/open_app/dev`, `git@github.com:ramin332/aeroscan-psdk.git`)
  — PSDK C app. Today: `6fd6a34` (DPK packaging), `3f63a84` (mission progress),
  `1906364` (PING/STAT), `84d501d` (comment fix).
</content>
