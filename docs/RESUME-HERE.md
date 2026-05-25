# RESUME HERE — AeroScan fly-readiness (last updated 2026-05-25)

> Single entry point for picking up the on-drone augment→fly work next session.
> Read this first, then the linked detail docs. Everything below is **merged to
> `main`** on both repos (the `feat/manifold-readiness-handshake` branch is merged).
> Start new work from `main`; on the Manifold, `cd /open_app/dev`.

## One-line status

**The whole augment→fly software chain is built, wired, and verified at a desk; it
has NEVER flown.** The only thing between us and a real end-to-end run is a **fresh
Smart3D scan** (there is no mesh on `/blackbox` right now). Code + docs are done and
committed on `main`.

## The ONE next action (resolves both remaining blockers at once)

1. **Fly a Smart3D Auto-Exploration scan** of a building with the M4E.
2. **Immediately** (before many reboots — the mesh is on a ~30-slot ring buffer):
   open the **rc-companion** on the RC → pick the Smart3D `.kmz` → **Augment**.
   - The readiness banner should now go **green** ("mesh ✓"), and the augment runs
     to completion instead of failing at `[2/7]` no-mesh.
3. Review the preview → **Approve & upload** → switch to **Pilot 2** → tap the
   **AeroScan Fly** widget → `DjiWaypointV3_Action(START)`.
4. **Watch for the GO/NO-GO:** does the M4E accept the WaypointV3 upload + START and
   fly it? (Never tested — DJI docs don't explicitly list the M4E for WaypointV3.)
   Watch the floating window for "Mission: flying — waypoint N", and note whether the
   FC applies its own safety interlocks (battery/GPS/home) — that's undocumented for
   WaypointV3 and this is how we find out.

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
| 🔴 | **Mesh** — none on `/blackbox`; needed for a successful augment | fly a Smart3D scan |
| 🔴 | **First M4E WaypointV3 upload+START** — never run on hardware (GO/NO-GO) | the flight above |
| 🟡 | **WaypointV3 START safety interlocks** (battery/GPS) undocumented — verify on that flight | observe |
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
