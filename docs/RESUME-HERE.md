# RESUME HERE — AeroScan fly-readiness (last updated 2026-07-10)

> Single entry point for picking up the on-drone augment→fly work next session.
> Read this first, then the linked detail docs. Everything below is **merged to
> `main`** on both repos (the `feat/manifold-readiness-handshake` branch is merged).
> Start new work from `main`; on the Manifold, `cd /open_app/dev`.

## One-line status

**IT FLIES.** The augment→fly chain has now flown twice: 2026-06-12 (first custom
WaypointV3 flight) and **2026-07-10 (two missions, one power session, `flight0072`;
first PAUSE/RESUME in the air)**. The 2026-06-12 gimbal-motor-overload is **resolved**
(zero occurrences 2026-07-10). The ground-removal bug found on the 2026-07-10 morning
van scan is **fixed** (commit `d4cd1cb`, plane fit + facet gate, 153 tests pass). The
work now is closing the open items below, not proving first flight.

## The ONE next action — investigate the RESUME index behaviour

The 2026-07-10 flight's biggest open question: **RESUME may restart at waypoint 1.**
After RESUME at WP 396, DJI's own callback emitted `mission state: 80, index: 1` then
`index: 2` and **actions fired at WP 2 (photos taken)** before continuing at 396.
Unclear whether the aircraft physically flew there. **Investigate `flight0072` diag
telemetry before trusting PAUSE mid-mission.** See
`docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md`.

Other open items from 2026-07-10 (all in that ANALYSIS.md): Fly widget inert while
paused (`fly tap ignored — state=0`); **mesh staleness gate did NOT fire** (flew a
~15 h mesh despite the 6 h `AEROSCAN_MESH_MAX_AGE_S` gate); `ground_clearance_m` /
`ground_facet_clearance_m` still hardcoded (not in the `fd_*` UI chain); low photo
count (one `takePhoto` per WP collapses the 5-pose rosette); heading/gimbal coupling
(airframe barely turns, gimbal yaw can exceed pan range).

## What works (verified this session)

- **Deployment:** runs as a DJI **DPK** (`psdk-demo`), Pilot-managed, boot-capable.
  Our **AeroScan Fly widget renders** in Pilot (packaging fixed: `.dpk` 159 MB → 432 KB).
- **Full chain wired + code-audited:** `PING/STAT` readiness → `AUGM` → augment
  subprocess → `PRVW` preview → `EXEC` (pilot approve) → `DjiWaypointV3_UploadKmzFile`
  → `READY_TO_FLY` → Fly-widget tap → `Action(START)` → mission-state callback feeds
  the floating window.
- **Readiness handshake working end-to-end** (RC banner ⇄ Manifold PING/STAT, ch 49154).
- **Transport proven** on real hardware (RC → Manifold, 600 KB+ missions staged + augment invoked).

## Open items

| | Item | Owner |
|---|---|---|
| ✅ | **First M4E WaypointV3 upload+START** — FLEW 2026-06-12 + 2026-07-10 (GO) | done |
| ✅ | **Gimbal motor overload** (2026-06-12) — zero occurrences 2026-07-10 | resolved |
| ✅ | **Ground removal on short targets** — plane fit + facet gate, commit `d4cd1cb` | resolved |
| 🔴 | **RESUME may restart at WP 1** — investigate `flight0072` diag before trusting PAUSE | you |
| 🔴 | **Mesh staleness gate did NOT fire** — flew a ~15 h mesh past the 6 h gate | you |
| 🟡 | **Fly widget inert while paused** (`state=0`) — pilot tapped 3× before finding Resume | you |
| 🟡 | **Ground params hardcoded** — plumb `ground_clearance_m`/`ground_facet_clearance_m` into the `fd_*` UI chain | you |
| 🟡 | **Low photo count** — one `takePhoto` per WP collapses the 5-pose rosette | you |
| 🟡 | **WaypointV3 START safety interlocks** (battery/GPS) still undocumented — kept observing | observe |
| 🟢 | **Build the rc-companion in Android Studio** — banner + fail-fast are committed but UNBUILT | you (no gradle on the laptop) |
| 📋 | **Mirror these doc updates into the Manifold `/open_app/dev/docs/` + `INDEX.md`** | controller (not yet done) |

We deliberately did **not** build: HMS/buzzer alerts (RC banner already warns), or our
own pre-flight safety checks (rely on the FC). Keep it simple.

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

# Is a mesh present on the latest flight?
ls /blackbox/the_latest_flight/dji_perception/1/mesh_binary_*.ply 2>/dev/null || echo "NO MESH"
```

## Key gotchas (learned the hard way)

- **`/blackbox` is a ~30-slot ring buffer** that cycles; a **power-cycle** creates a
  new flight slot (app updates don't). The mesh is evicted as it cycles → **augment
  right after scanning**, and don't burn reboots in between.
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
