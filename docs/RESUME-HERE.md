# RESUME HERE — AeroScan fly-readiness (last updated 2026-09-02)

> Single entry point for picking up the on-drone augment→fly work next session.
> Read this first, then the linked detail docs. Everything is merged to `main`.
> On the Manifold, `cd /open_app/dev`.

## One-line status

**IT FLIES, AND WE NOW MEASURE WHAT IT ACTUALLY DID.** Four missions have flown
(2026-06-12; 2026-07-10 morning + two afternoon + one evening). The day's real
lesson: we spent it tuning what we *ask* the aircraft for, and only at the end read
the JPEG XMP to see what the aircraft *does*. The commands were already better than
the execution. **Instrument before optimising.**

## Next time the aircraft is powered on (2026-09-02 leftovers, 2 minutes)

1. `git -C ~/git/aeroscan-psdk fetch manifold main && git merge --ff-only manifold/main && git push` —
   Manifold commit `b844692` (stale-mesh override, widget packaging fix) is not on GitHub yet.
2. **Clear the staged ground-test scenario** before any real flight:
   `ssh dji@192.168.1.118 rm /open_app/dev/data/received/mission_progress.json` — otherwise the
   app offers "Continue from WP 217/398" of the July mission. (Augmenting a new mission also clears it.)
3. Smart3DExplore was left `failed` after the 14:23 app switch; a power-cycle restores it (boot app).

## The ONE next action — reflight and re-measure the gimbal

**First** install the DPK rebuilt on 2026-09-02 (`dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk`) and `git commit` on the Manifold — it carries the mesh-subdir fix and the telemetry recorder — and deploy the laptop repo's augment changes (`scripts/deploy_to_manifold.sh`). Then fly `2bf3308`. Then pull the photos and run:

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
- **"No Smart3D scan ran on 2026-07-10"** — two did: 10:18 (flight0070, subdir `1`)
  and 11:48 (flight0072, 13 chunks, subdir `2`). Everything was globbing subdir `1`.
- **"The mesh was 15 h old / from the previous evening"** — no. Chunk-file mtimes on
  the Manifold say 10:18–10:21 the same morning; the slot *dir* mtime (07-09 20:27)
  lied. The staleness gate logged the correct 253–8041 s. Verified 2026-09-02.

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
| 🟡 | C mesh resolver `dji_perception/1` hardcode — **fixed in code 2026-09-02** on the Manifold (`kmzrun_status.c` globs `dji_perception/*/`; `test_status.c` proves a `/2/`-only slot resolves and wins on mtime). DPK rebuilt (`build/dpk/psdk-demo_v01.00.00.00.dpk`, 12:08) but **NOT installed** and the Manifold repo is **uncommitted**. Until installed, `AEROSCAN_FLIGHT_ID=flightNNNN` still applies. Cost of the bug on 2026-07-10: **9 facades instead of 181**. | you: install + commit |
| ✅ | GSD plan-time gate — `gsd_out_of_spec` in `validate.py` (warns at 25% over target, names the M4E lens that would meet it at the same standoff). `validate_mission` now also runs on the **augment path** (`cli.py`), which never validated before. Card shows `GSD x.x mm/px` instead of `Aim 100%`. 2026-09-02. | resolved |
| 🟡 | **Range-adaptive lens.** M4E has WIDE/MEDIUM_TELE/TELEPHOTO (`models.py:50-81`); we hardcode WIDE. At 33.8 m, MEDIUM_TELE gives **2.12 mm/px** vs WIDE's 9.23 — a 4.4× gain at zero flight-time cost. **Gated on a device test:** can M4E WPML direct a single `takePhoto` to a chosen lens? Smart3D's `template.kml` declares `<wpml:imageFormat>visable</wpml:imageFormat>`; we emit none. | you |
| 🟡 | Fly widget inert while paused (`fly tap ignored — state=0`); pilot tapped 3× before finding Resume | you |
| ✅ | Mesh staleness gate — verified correct on 2026-09-02 from the PSDK logs: it read 253–8041 s for a mesh written 10:21 that morning. The "15 h old" claim came from a lying slot-dir mtime. | resolved |
| 🟡 | 104/398 lost photos (count solid; dwell mechanism is the leading hypothesis, NOT proven — the FC completion callbacks are flat vs leg time, see ANALYSIS addendum) — **addressed in code, unflown.** Augment now defaults to WPML `toPointAndStopWithContinuityCurvature` ("the aircraft will stop at the point") via `stop_at_waypoint=True`; `--fly-through` opts out. Fly-through gets a `action_dwell_too_short` warning driven by the new `min_action_dwell_s` knob (models → api → UI slider). Cost: flight time — 398 stops. Verify on the next flight: action completions == starts in the PSDK log. | you: fly |
| 🟡 | Heading reachability — **addressed by stop-at-waypoint (unflown)**: the aircraft stops, so the turn completes before the shutter. Fly-through gets a `heading_step_unreachable` warning (step / `yaw_rate_deg_per_s` > leg time). `schedule_headings()` deliberately stays off on the no-yaw path: with no gimbal yaw to absorb residual pan, rate-limiting the heading would point the camera *away* from the target. | you: fly |
| 🟡 | **Facade picker redesign — done 2026-09-02, unflown.** `rewrite_gimbals_perpendicular(assign_mode="viterbi")` is the default: whole-sequence assignment (`assign_facades_viterbi`), coplanar slices = one target (`plane_groups`), reach capped at the 2×-GSD standoff (14.6 m WIDE) with DJI's own pose kept when nothing is in reach. Measured on busboom (same mesh, same 398 WPs): far picks **39 → 0**, max standoff **31.4 → 14.6 m**, blips **1 → 0**, target switches **76 → 54**, flips >90° **23 → 14** — the 14 residual are van↔parked-car handovers at 9–14 m (test venue, not the picker). `scripts/render_aim_audit.py` draws it; the card shows `flips N far N`. CLI `--assign-mode/--switch-cost/--max-facade-distance-m`; API `rewrite-gimbals` takes the same as query params. | you: fly |
| ✅ | **RC app "only connects after a Pilot 2 message" — explained + fixed 2026-09-02.** Pilot's enable step *starts* the DPK; MOP binds in <1 s, the RC connected 13–36 s later on every 07-10 session; the rc-companion checked once and then waited for a manual Retry. Now polls every 5 s until Ready (`nextPollDelayMs`), banner gives the Pilot 2 steps. Built locally: `output/rc-companion/rc-companion-debug-2026-09-02.apk` (sideload to the RC). Side effect found: the switch **SEGVs Smart3DExplore** (unit `failed`); re-scan needs Smart3D restarted in Pilot — runbook Stage 2. Untested in the field. | you: sideload |
| 🟡 | **Battery-swap resume ("Continue") — built 2026-09-02, unflown.** Python `slice-kmz` (6 tests, verified on the flown KMZ); Manifold `kmzrun_progress.c` (native test replays the July callbacks) + Pilot 2 widget index 2 + STAT mission fields; DPK rebuilt + installed; RC app shows the mission line and asks twice before a new augment replaces an interrupted mission. Rules: new augment replaces; Fly = whole mission; Continue = from last waypoint, original photo numbering. Runbook section "Battery swap mid-mission". | you: fly |
| 🟡 | Gimbal telemetry — **implemented 2026-09-02, needs DPK install**: `kmz_runner.c` subscribes `GIMBAL_ANGLES` (10 Hz, x=pitch y=roll z=yaw deg), `QUATERNION` (10 Hz, raw), `POSITION_FUSED` (5 Hz) and writes `/open_app/dev/data/received/telemetry/<UTC>.csv` with the current WP index while a mission is active; closed on IDLE. **Follow-up:** a reader script (quaternion → yaw, diff vs the flown KMZ per WP, like `read_gimbal_xmp.py`) — not written yet. Verify the quaternion sign convention against XMP `FlightYawDegree` once. | you: install, then fly |
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
