# Manifold readiness bring-up — session status & next steps (2026-05-25)

> Handoff doc written before a battery reboot + conversation compaction. Read this
> first to resume. Branch (both repos): **`feat/manifold-readiness-handshake`**.

## One-line status

The Manifold PSDK app **builds and runs live on the aircraft** (binds MOP 49154,
widgets register, RC→Manifold transport works); the **Phase 1 C readiness module
is built & unit-tested**; auto-start must go through the **DJI DPK path** (the
`dji` user has no root). Remaining: install as DPK + set Pilot auto-start, wire
Phase 2 (PING/STAT), build Phase 3 (RC banner), and get a **mesh** for a
successful augment.

## What we found (investigation)

1. **Mesh is volatile.** `/blackbox` is a ~30-flight ring buffer. flight0016
   (Mijande) is pruned. **Only flight0019** still has `mesh_binary_*.ply`;
   flight0048 (latest) has none. → augment on the latest flight fails (the exact
   failure from this morning). See memory `manifold-mesh-volatile`.
2. **No root.** `dji` is not in sudoers (`groups`: dji adm dialout audio video
   render gdm weston-launch jtop). → `systemd --user` **cannot enable linger** →
   cannot auto-start on boot. BUT `dji_app_ctl` works **without** sudo.
3. **One app at a time.** Only one DPK app holds the aircraft E-Port link;
   `Smart3DExplore` and our app can't both run. `run.sh` stops Smart3D first.
   Coexistence not tested.
4. **Widgets.** Pilot showed **tts / record / move** = the stock `psdk-demo`
   DPK's widgets. Our app ran as a *raw process / systemd service*, so Pilot
   never drew its widgets ("AeroScan fly widget not found"). Pilot only surfaces
   widgets from the **managed DPK app** → running as the DPK should fix this.
5. **Transport works.** Live test: RC sent a KMZ (302508 B intent + 57154 B
   cloud) → Manifold **staged** it → augment **invoked** → **FAILED on no-mesh**
   (flight0048). The RC↔Manifold chain is healthy up to the mesh step.
6. **WaypointV3 upload + `Action(START)` is the sanctioned transport** (docs
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
- **systemd `--user` service installed** (`~/.config/systemd/user/aeroscan-kmzrunner.service`),
  enabled, **start/stop demonstrated**. BUT it **cannot auto-start on boot**
  (no linger/root). Keep it for SSH-driven dev; it is NOT the production
  autostart.

## Decisions made

- **Auto-start path = DPK + Pilot** (root-free). ← chosen this session.
- Phase 1 knobs: `env_ok` = deep `import flight_planner.manifold` probe (5 s);
  service user = `dji`; `n_points` via PLY-header parse.

## What's left to do (ordered)

1. **DPK path (the chosen autostart):**
   - Decide app identity: rename to "AeroScan" in `config/dji_sdk_app_info.h`
     (+ rebuild) vs keep "psdk-demo" (installing our `.dpk` updates the existing
     `psdk-demo` app).
   - Install (no sudo): `dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk`.
   - In Pilot → Application Management: start it, **verify OUR widgets appear**
     (incl. fly widget), then **Set as Auto-start on Boot**.
   - **Reboot test:** power-cycle → confirm auto-start + MOP 49154 bind + widgets.
   - Watch: boot timing (E-Port/link readiness vs `DjiCore_Init`), Smart3D ↔
     AeroScan slot switching in Pilot.
2. **Phase 2 — PSDK PING/STAT wiring** in `kmz_runner.c` (plan Phase 2). The
   `kmzrun_status` module already builds clean into the app; just add the
   `is_ping` dispatch + STAT reply. Can't end-to-end test PING until Phase 3.
3. **Phase 3 — RC-companion Kotlin** (plan Phase 3): Constants / AugmentFraming
   (PING/STAT) / StatusSession / HomeViewModel banner / HomeScreen. **User builds
   in Android Studio** (laptop has the AS JBR + cached gradle 8.9, but no
   `java`/`gradle`/`gradlew` on PATH for headless builds).
4. **Mesh for a successful augment** (the data blocker): fly a fresh Smart3D scan
   (also answers the firmware-persistence question H1/H2 — does current firmware
   keep the mesh in `/blackbox`?), OR use flight0019's mesh + its source KMZ.
   Without a mesh, augment correctly fails.
5. **Widget-config fix** if the DPK path doesn't resolve it.
6. **Phase 2.3 fly trigger** (widget tap → `Action(START)`) + **visual preview**
   (render a PNG into the PRVW frame) — later production items.

## How to resume (quick commands)

- **Standalone readiness check (no aircraft needed):**
  `ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && gcc -I. test/test_status.c kmzrun_status.c -o /tmp/t && /tmp/t'`
  or the live `/blackbox` smoke against `the_latest_flight` / `flight0019`.
- **Build the Manifold app:** `ssh dji@192.168.1.55 'cd /open_app/dev && ./run.sh'` (builds + execs).
  Logs: `Payload-SDK-3.16.0/build/data/logs/` (relative to CWD).
- **systemctl --user** needs `XDG_RUNTIME_DIR=/run/user/1000`.

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
