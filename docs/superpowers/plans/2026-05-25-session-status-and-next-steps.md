# Manifold readiness bring-up — session status & next steps (2026-05-25)

> Handoff doc written before a battery reboot + conversation compaction. Read this
> first to resume. Branch (both repos): **`feat/manifold-readiness-handshake`**.

## One-line status

The Manifold PSDK app **builds, is installed as a DPK, and runs live on the
aircraft**, and the **augment pipeline is now proven end-to-end on real hardware**:
RC → MOP 49154 → handler received a 351 KB mission → venv Python parsed the
584-waypoint MijandeExtra mission → clean, correct no-mesh failure (flight0049
has none). **Deployment is settled: DPK + DJI Pilot 2** (installed & validated
2026-05-25; dev `systemd --user` service removed). The **dev path is now one
command** — `./run.sh` stops both production apps and runs the raw binary with
readable logs (`docs/architecture/manifold-deployment.md` for dev-vs-prod). Widget
clutter (TTS/mic) removed; the AeroScan **Fly** button + status window + dev textbox
remain. **Phase 1 C readiness module built & unit-tested.** Remaining: Phase 2
(PING/STAT), Phase 3 (RC banner), widget-config packaging, and the one real
blocker — a **mesh** for a successful augment.

## What we found (investigation)

1. **Mesh is volatile.** `/blackbox` is a ~30-flight ring buffer. flight0016
   (Mijande) is pruned. **Only flight0019** still has `mesh_binary_*.ply`;
   flight0048 (latest) has none. → augment on the latest flight fails (the exact
   failure from this morning). See memory `manifold-mesh-volatile`. **Still the
   live data blocker.**
2. **DPK is the production deployment** (not raw `systemd`). `dji_app_ctl install
   -i <file>.dpk` works **without** sudo. DJI's docs say the raw-exec path is
   dev-only and may terminate abnormally; production must go through Pilot. (The
   earlier "no root → can't autostart → *must* use DPK" reasoning was **wrong**
   — see #4.)
3. **One app at a time.** Only one DPK app holds the aircraft E-Port link;
   `Smart3DExplore` and our app can't both run. Manual app-switching in Pilot
   (Smart3D → our app, after the scan) is the intended, accepted flow. The boot
   app stays Smart3DExplore.
4. **Auto-start was NOT blocked.** Correction to the prior premise: the dev
   `systemd --user` service **did survive a cold reboot** — the Manifold
   graphically auto-logs-in `dji`, which starts `systemd --user`, which starts
   the enabled service; the app even reconnected to the E-Port. So DPK is chosen
   for DJI-recommended-production + stability + widget support, **not** because
   auto-start was impossible.
5. **Widgets require the DPK path — confirmed.** As a managed DPK app, our PSDK
   **Custom Widgets render and are interactive on Pilot's live view** (the stock
   gimbal-mover widget worked). The earlier raw-`systemd` binary never surfaced
   widgets. → a future "tap → fly" widget (`Action(START)`) requires the DPK
   path. Update (this session): the app already ships a **custom**
   `widget_config.json` (AeroScan **Fly** button); the TTS/microphone clutter came
   from a `speaker` block copied from the sample — **removed** from both en/cn
   configs. Remaining widget work: package `widgets/` into the DPK (binary-relative
   load path, not the hardcoded `/open_app/dev/...`) + the Fly tap→`Action(START)`
   handler (Phase 2.3).
6. **Transport + augment proven end-to-end (dev path, 2026-05-25).** RC → MOP
   49154 → `kmz_runner` received a 351 KB mission payload → staged → launched the
   venv Python → augment parsed the mission (`[1/7] MijandeExtra, 584 waypoints,
   9-vertex polygon`) → `[2/7]` mesh load → clean `FileNotFoundError` (no mesh on
   flight0049) → handler logged `Augment FAILED` (exit 1). The whole chain works;
   only mesh data is missing. **Caveat:** the first connect left a stale/half-open
   MOP channel (`channelHandle … closed` + send retries, "unable to send"); a
   restart cleared it — stale-channel recovery is a hardening TODO.
7. **WaypointV3 upload + `Action(START)` is the sanctioned transport** (docs
   confirmed). By design two human gates: approve in companion → upload; Pilot
   widget tap → START. The **START trigger (Phase 2.3) is not implemented**, and
   the companion shows **stats only, no visual preview**.

## What we built / did this session

- **Spec:** `docs/superpowers/specs/2026-05-25-manifold-service-readiness-handshake-design.md` (committed).
- **Plan:** `docs/superpowers/plans/2026-05-25-manifold-readiness-handshake.md` (committed).
- **Phase 1 DONE** — repo **aeroscan-psdk** (`/open_app/dev`, branch
  `feat/manifold-readiness-handshake`): `src/manifold3_app/kmzrun_status.{c,h}` +
  `test/test_status.c`. PSDK-free, native-gcc unit-tested (ALL PASS). Standalone
  tool reports mesh/env readiness of any flight. Commits `7f39888`, `1c2278c`,
  `f22088f`.
- **Live bring-up verified** — clean build (also emits a `.dpk`), app launched on
  the aircraft: `DjiCore_Init` OK, **MOP 49154 bound + accepting**, widgets
  registered ("fly widget at index 0"), WaypointV3 state callback active, no
  errors.
- **DPK install — DONE & validated (production deployment).** Installed our build
  as a DPK (no sudo):
  `dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk`
  (installs to `/open_app/psdk-demo`; DJI auto-generates a root SYSTEM service
  with `WantedBy=multi-user.target`). Started from Pilot, **Custom Widgets render
  and are interactive on the live view** (verified). Full mechanics in
  `docs/architecture/manifold-deployment.md`.
- **systemd `--user` service — REMOVED.** Deleted
  `~/.config/systemd/user/aeroscan-kmzrunner.service`. It was a dev convenience;
  it actually *did* auto-start across a cold reboot (graphical auto-login →
  `systemd --user`), so the prior "can't autostart" claim was wrong. DPK
  superseded it for production (DJI-recommended + stability + widgets).

## Decisions made

- **Deployment = DPK + Pilot 2** (installed & validated this session). Chosen for
  DJI-recommended-production posture + runtime stability + Custom Widget support
  on Pilot's live view — **not** because `systemd --user` couldn't auto-start
  (it could). The boot auto-start app stays **Smart3DExplore**; the pilot
  manually switches to our app in Pilot after the scan.
- **App identity stays as-is.** Registered triple `USER_APP_NAME="Payload test ap"`,
  `USER_APP_ID="183281"` (+ key/license) must not change — renaming risks
  breaking `DjiCore_Init` auth. The DPK display name `psdk-demo` is cosmetic
  packaging only. (Resolves the earlier "rename to AeroScan vs keep psdk-demo"
  open question: keep the registered identity.)
- Phase 1 knobs: `env_ok` = deep `import flight_planner.manifold` probe (5 s);
  augment subprocess runs under the sandboxed DPK user `app<appname>` (the 775
  perms on `/open_app/dev` + 755 on `/blackbox` let it exec the venv Python and
  traverse the mesh); `n_points` via PLY-header parse.

## Done this session (deployment)

- **DPK install + validation — DONE.** App installed as a DPK (no sudo), started
  from Pilot, widgets confirmed interactive on the live view, augment subprocess
  works under the sandboxed DPK user. App identity kept as the registered triple
  (not renamed). Canonical model: `docs/architecture/manifold-deployment.md`.
- **systemd `--user` service — REMOVED.** No longer the deployment path.
- **`run.sh` is now the one-command dev launcher.** Stops BOTH production apps
  (Smart3DExplore + psdk-demo DPK), builds, runs the raw binary foreground with
  readable logs; banner notes a reboot/Pilot re-tap restores production. (Manifold
  repo, uncommitted.)
- **Transport + augment validated end-to-end on the dev path** (see #6) — the full
  RC→Manifold→augment chain works; fails only on missing mesh.
- **Widget clutter removed.** Dropped the `speaker` (TTS/voice) block from both
  `en/cn widget_config.json` → just the AeroScan Fly button + status window +
  textbox. (Manifold repo, uncommitted.)

## What's left to do (ordered)

1. **Mesh for a successful augment — THE blocker (field task).** Fly a fresh
   Smart3D scan (also answers firmware-persistence H1/H2: does current firmware
   keep the mesh in `/blackbox`?), OR use flight0019's mesh + its source KMZ — but
   flight0019 is at the ring-buffer prune edge (latest is **flight0049**, ~30-flight
   buffer), so **verify it still exists** before relying on it. Everything else
   works; this is the only thing standing between us and a successful augment.
2. **Phase 2 — PSDK PING/STAT wiring** in `kmz_runner.c` (codeable now, no aircraft
   needed). `kmzrun_status` builds clean; add the `is_ping` dispatch + STAT reply.
   End-to-end testable once Phase 3 lands.
3. **Phase 3 — RC-companion Kotlin** (Constants / AugmentFraming PING-STAT /
   StatusSession / HomeViewModel banner / HomeScreen). **User builds in Android
   Studio.** This is also the answer to "how do we see managed-app status" — the
   DPK's logs aren't `dji`-readable over SSH, so the RC banner is the status surface.
4. **Widget-config packaging.** Copy `widgets/` into the DPK + load via a
   binary-relative path (drop the hardcoded `/open_app/dev/src/...`), so a DPK
   shipped to a device without the dev tree still finds its config. Small
   CMake/`setup_psdk.sh` change. (TTS/mic already removed.)
5. **Stale MOP-channel recovery.** The first connect left a half-open channel
   ("unable to send" until a restart). The handler should detect + recover stale
   sessions instead of needing a manual restart.
6. **Phase 2.3 — fly trigger** (Fly widget tap → `Action(START)`, with the human
   gate) + **visual preview** (PNG into the PRVW frame). Deployment side unblocked
   (widgets interactive); the START handler + gate is the work.
7. **Manifold-side docs.** Mirror the dev/prod deployment model into
   `/open_app/dev/docs/` + `INDEX.md` (laptop `docs/` already updated).
8. **Commit the branch work** (both repos, `feat/manifold-readiness-handshake`):
   laptop doc updates; Manifold `run.sh` dev-mode + `widget_config` cleanup are
   uncommitted.

## How to resume (quick commands)

- **Standalone readiness check (no aircraft needed):**
  `ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && gcc -I. test/test_status.c kmzrun_status.c -o /tmp/t && /tmp/t'`
  or the live `/blackbox` smoke against `the_latest_flight` / `flight0019`.
- **Build the Manifold app (dev/debug only):** `ssh dji@192.168.1.55 'cd /open_app/dev && ./run.sh'` builds + execs the raw binary.
  Per DJI docs the raw-exec path is dev-only (may terminate abnormally) — production runs via the DPK; see `docs/architecture/manifold-deployment.md`.
- **Production deployment (DPK):** build emits `build/dpk/psdk-demo_v01.00.00.00.dpk` → `dji_app_ctl install -i <file>.dpk` (no sudo) → start/stop from Pilot → Enter Camera View (icon on left edge; yellow = running). Logs: Pilot → Application Management → Log Export, or `journalctl -u psdk-demo.service` (the DPK runs as `app<appname>`, so `dji` can't read `data/logs/` directly). Full RC steps in `docs/architecture/manifold-deployment.md`.

## Pitfalls / gotchas (learned the hard way)

- **Never `pkill -f dji_sdk_demo_on_manifold3` over SSH** — `-f` matches your own
  SSH command line (it contains that string) and kills your session (exit 255),
  and the app too. Use `pkill -x dji_sdk_demo_on` (15-char comm) or kill by PID.
- **`test_status.c` must stay in `test/` subdir** — `setup_psdk.sh` globs
  `src/manifold3_app/*.c` into the PSDK build; a `main()` there collides with
  `main.c`. (Already moved + committed.)
- **Manifold clock skew ~1 h** → `make` "modification time in the future"
  warnings; a clean build sidesteps it.
- The PSDK app **writes logs relative to its CWD** (`data/logs/`), so launch dir
  matters when hunting the current log.
