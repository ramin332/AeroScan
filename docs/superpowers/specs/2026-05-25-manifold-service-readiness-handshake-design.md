# Manifold service + RC readiness handshake — design

**Date:** 2026-05-25
**Status:** Design (pending user review)
**Author:** AeroScan / Claude pairing session

## Problem

The on-Manifold augment app (`kmz_runner`, a PSDK binary) is started by hand over
SSH via `/open_app/dev/run.sh`, which cmake-builds then `exec`s the binary. Two
operational gaps make the RC-companion augment workflow not production-ready:

1. **No visibility into Manifold readiness.** The RC-companion just opens a MOP
   pipeline on channel 49154 and hangs if nothing is listening. The pilot has no
   way to know whether (a) the app is up, (b) the augment Python env is healthy,
   or (c) the latest flight even has a mesh to augment. When the mesh is missing
   the augment subprocess dies with a cryptic `exited 1` *after* the pilot has
   already shipped the payload (see `augment-mission` `from_manifold` failure,
   2026-05-25).
2. **No managed lifecycle.** A manual `exec` has no crash recovery, no clean
   start/stop, and no observable status. Restarting means SSH + rebuild.

Underlying reality (verified 2026-05-25, see memory `manifold-mesh-volatile`):
`/blackbox` is a ~30-flight ring buffer; only `flight0019` still holds
`mesh_binary_*.ply`. The latest flight (`flight0048`) has `dji_perception/1/`
but **no mesh**. So "does the latest flight have a mesh" is a live, per-session
question the pilot must be able to see *before* augmenting.

## Goals

- **Readiness handshake (Part B):** RC-companion can ask the Manifold app for a
  status snapshot and render a banner: app up? env healthy? latest flight has a
  mesh (chunks + point count)? `/blackbox` free space?
- **Managed service (Part A):** Run `kmz_runner` under `systemd --user` (DJI's
  sanctioned mechanism for non-DPK, complex-runtime apps) with crash recovery
  and a clean start/stop/status surface. Ships in a mode that is **correct
  regardless of whether the app can coexist with Smart3D**, with boot-enable as a
  documented one-step upgrade gated on a hardware coexistence test.

## Non-goals (separate workstreams, explicitly out of scope here)

- Phase 2.3 fly trigger (widget tap → `DjiWaypointV3_Action(START)`).
- Visual/geometric mission preview before approve (the PNG-in-PRVW idea).
- Mesh durability / harvest-on-flight-complete (fixes the *cause* of missing
  mesh; this spec only *surfaces* it).
- Packaging AeroScan as a DPK / RC-driven app launch.
- Disambiguating *why* recent flights lack a mesh (needs a fresh Smart3D flight).

## Codebases touched

| Change | Repo | Path |
| --- | --- | --- |
| C: PING/STAT frames, status builder | `aeroscan-psdk` (`git@github.com:ramin332/aeroscan-psdk.git`, lives at `/open_app/dev`) | `src/manifold3_app/kmz_runner.{c,h}` |
| systemd `--user` unit + installer + coexistence-test doc | `aeroscan-psdk` | `scripts/`, `docs/` |
| Kotlin: PING/STAT codec, status query, banner | `aero-scan` (this repo) | `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/{mop,ui}/` |

Python engine is unchanged. The `env_ok` probe imports it but adds no code there.

## Architecture overview

```text
RC-companion (Android)                    Manifold (kmz_runner, PSDK C)
─────────────────────                     ────────────────────────────
HomeViewModel.checkStatus()
  → StatusSession.query()
      connect MOP 49154
      send PING  (16B header, body=0) ───▶ HandleConnection: is_ping
                                              BuildStatusJson():
                                                - resolve flight_id
                                                - glob mesh_binary_*.ply
                                                - parse PLY headers → n_points
                                                - statvfs(/blackbox)
                                                - deep env probe (python import, 5s timeout)
      read STAT (16B header + JSON)   ◀───   EncodeHeader(STAT)+SendAll(json)
      parse → ManifoldStatus
  → banner: 🟢 / 🔴 / ⚪
```

The handshake is a **pure read** on the C side — it never touches the augment
state machine, so it is safe to answer at any time (idle, mid-review, etc.).

## Part B — Readiness handshake (wire protocol)

### New frames (same 16-byte header, two new magics)

Header is unchanged: `magic[4] | version u32 LE | body_len u32 LE | reserved[4]`.

- **`PING`** (RC → Manifold): `body_len = 0`. "Send me status."
- **`STAT`** (Manifold → RC): `body_len = N`, body = UTF-8 JSON (single object).

Backward compatibility:

- Old Manifold + new RC: old C sees unknown magic `PING`, logs "bad/unknown
  magic — drop conn". RC's `StatusSession` treats a connection drop / no STAT
  within the deadline as **⚪ Unreachable** (degrades safely).
- New Manifold + old RC: RC never sends PING → no behavior change.
- `version` stays `1`; STAT is additive.

### STAT JSON schema

```json
{
  "app_version":      "string",   // compile-time KMZRUN_APP_VERSION
  "flight_id":        "string",   // what augment WOULD use (env or the_latest_flight)
  "latest_flight":    "string",   // resolved basename, e.g. "flight0048"
  "mesh_present":     false,      // mesh_chunks > 0
  "mesh_chunks":      0,          // count of mesh_binary_*.ply
  "n_points":         0,          // Σ "element vertex N" across chunks (header parse)
  "mesh_bytes":       0,          // Σ file sizes
  "blackbox_free_gb": 42.1,       // statvfs(/blackbox)
  "env_ok":           true,       // deep probe: python -c "import flight_planner.manifold"
  "env_detail":       "ok"        // "ok" | "import failed (exit N)" | "timed out (>5s)"
}
```

`mesh_*` are computed with the **same glob** as `flight_planner.manifold.
merge_blackbox_plys` (`mesh_binary_*.ply`), so `mesh_present` is a faithful
predictor of whether `augment-mission`'s `from_manifold` will succeed.

### C-side computation (`BuildStatusJson`, pure C)

Factor into a testable function `kmzrun_build_status_json(const char *blackbox_dir,
const char *flight_id, const char *venv_python, char *out, size_t outsz)` so it
can be unit-tested against a fixture directory (no `/blackbox` dependency).

- **flight_id:** `getenv("AEROSCAN_FLIGHT_ID")` else `"the_latest_flight"`.
- **latest_flight:** `realpath(<blackbox>/<flight_id>)`, take basename.
- **mesh:** `glob(<blackbox>/<flight_id>/dji_perception/1/mesh_binary_*.ply)`;
  per file: `stat` for bytes; read the ASCII PLY header (≤4 KB) until
  `end_header`, parse `element vertex N`, accumulate. No vertex *data* is read.
- **blackbox_free_gb:** `statvfs(blackbox_dir)` → `f_bavail * f_frsize / 1e9`.
- **env_ok (deep):** fork/exec `venv_python -c "import flight_planner.manifold"`
  (the module the augment actually needs — pulls open3d). Bounded wait of
  **5 s** (open3d import is slow): poll `waitpid(WNOHANG)` + short `nanosleep`;
  on deadline `kill(child, SIGKILL)` and set `env_ok=false, env_detail="timed
  out (>5s)"`. Exit 0 → `true`; non-zero → `false, "import failed (exit N)"`.

The deep probe runs on the accept thread, briefly blocking that connection
(≤5 s). Acceptable: single-peer model, RC is waiting for STAT. (Optimization,
not in v1: cache the probe result for ~30 s.)

### C-side dispatch (`HandleConnection`)

Add `is_ping = !memcmp(hdr, KMZRUN_MAGIC_PING, 4)`. On PING (enforce
`body_len == 0`): build STAT JSON, `EncodeHeader(hdr, KMZRUN_MAGIC_STAT, len)`,
`SendAll(peer, hdr, 16)`, `SendAll(peer, json, len)`; `continue` (keep the
connection open). New defines: `KMZRUN_MAGIC_PING "PING"`,
`KMZRUN_MAGIC_STAT "STAT"`, `KMZRUN_APP_VERSION` (string).

### RC-side (Kotlin)

- **`Constants.kt`:** add `MAGIC_PING = "PING"`, `MAGIC_STAT = "STAT"`.
- **`AugmentFraming.kt`:** add `buildPingFrame(): ByteArray` (header, body 0) and
  `parseStatBody(body: ByteArray): ManifoldStatus` (UTF-8 JSON → data class).
  Add a `ManifoldStatus` data class mirroring the schema.
- **New `StatusSession.kt`** (sibling of `AugmentSession`, same pipeline I/O
  helpers): `suspend fun query(deadlineMs = 15_000): StatusResult`. Connect →
  send PING → bounded `readExactly(HEADER_LEN)` + STAT body → parse → close.
  Bounded read (NOT the augment's 10-min retry): on deadline or hard-close →
  `StatusResult.Unreachable`. Keep it separate from `AugmentSession` so a status
  check can't collide with an in-flight augment; ViewModel must not call it while
  an augment session is open on 49154.
- **`HomeViewModel.kt`:** `checkStatus()` invoked on Home screen entry + a manual
  "Refresh" action; expose a `manifoldStatus: StateFlow<BannerState>`.
- **`HomeScreen.kt`:** render the banner.

### Banner states (priority order)

| State | Condition | Example text |
| --- | --- | --- |
| ⚪ Checking | query in flight | `Checking Manifold…` |
| ⚪ Unreachable | connect fail / no STAT by deadline | `Manifold not reachable — is the app running?` |
| 🔴 EnvError | `env_ok == false` | `Augment env error: import failed (exit 1)` |
| 🔴 NoMesh | `env_ok && !mesh_present` | `flight0048 has no mesh — completed Smart3D run?` |
| 🟢 Ready | `env_ok && mesh_present` | `Ready — flight0048 · 1.2M pts · 12.4 GB free` |

EnvError outranks NoMesh (a broken env blocks augment regardless of mesh).

## Part A — systemd `--user` service

Per DJI FAQ §"How to Configure Auto Startup Using `systemd --user`" — the
sanctioned path for non-DPK, complex-runtime (conda/open3d) apps. Runs as the
`dji` user, outside the `dji_app_mgr` DPK slot.

### Unit (manual-start default) — `~/.config/systemd/user/aeroscan-kmzrunner.service`

```ini
[Unit]
Description=AeroScan KMZ runner (PSDK augment app)
After=network.target

[Service]
Type=simple
EnvironmentFile=-/open_app/dev/aeroscan-kmzrunner.env
# Manual-start default: free the single aircraft link from the scan app first.
# Removed in the boot-enable variant once coexistence is verified.
ExecStartPre=-/system/bin/dji_app_ctl stop Smart3DExplore
ExecStart=/open_app/dev/Payload-SDK-3.16.0/build/bin/dji_sdk_demo_on_manifold3
WorkingDirectory=/open_app/dev/Payload-SDK-3.16.0/build
Restart=on-failure
RestartSec=3
StartLimitIntervalSec=60
StartLimitBurst=5

[Install]
WantedBy=default.target
```

- **Runs the prebuilt binary** — it does not rebuild. Deploy sequence:
  `git pull` (aeroscan-psdk) → build (existing `run.sh` cmake/make steps, or a
  new `build.sh` that stops before `exec`) → `systemctl --user restart
  aeroscan-kmzrunner`. `run.sh` stays as the dev (build+exec) path.
- **`EnvironmentFile=-…`** (leading `-` = optional) lets bench pin
  `AEROSCAN_FLIGHT_ID=flight0019`; default unset → `the_latest_flight`.
- **Linger:** `loginctl enable-linger dji` so the `--user` instance survives SSH
  logout (without it, the service dies when the session ends — critical detail).

### Installer — `aeroscan-psdk:scripts/install_user_service.sh`

`mkdir -p ~/.config/systemd/user` → write unit → `systemctl --user daemon-reload`
→ `loginctl enable-linger dji`. Flags:

- (default) install manual-start variant, do **not** enable at boot.
- `--enable-boot` write the boot variant (drop `ExecStartPre`, keep
  `WantedBy=default.target`) and `systemctl --user enable aeroscan-kmzrunner`.
  **Only run after the coexistence test passes.**

### Coexistence test (the gate for `--enable-boot`)

Decides whether AeroScan can hold the E-Port PSDK link alongside Smart3D (the
docs don't settle it; `run.sh` stopping Smart3D is suggestive, not proof).

1. Power aircraft, link RC. Confirm `Smart3DExplore` is the active/running DPK
   (`dji_app_ctl status`).
2. **Without** stopping it, run the binary directly:
   `/open_app/dev/Payload-SDK-3.16.0/build/bin/dji_sdk_demo_on_manifold3`.
3. Watch the log (`$PSDK_DIR/data/logs/latest.log`):
   - **PASS:** `DjiCore_Init` succeeds **and** `bound MOP channel 49154,
     accepting` appears → coexistence works → boot-enable is viable → run
     installer `--enable-boot` (no `ExecStartPre` kill).
   - **FAIL:** `DjiCore_Init` errors (link/identity busy) or MOP bind fails →
     exclusive → keep manual-start + `ExecStartPre`.
4. Record the result in `aeroscan-psdk:docs/` and set the service mode.

Until the test runs, the default manual-start variant is correct either way.

## Error handling

- **C:** STAT build never aborts the connection. If the perception dir is
  missing, report `mesh_chunks=0, mesh_present=false` (not an error). If a PLY
  header is malformed, skip that file's vertex count but keep its byte size and
  the chunk count. `statvfs` failure → `blackbox_free_gb = -1`.
- **RC:** any connect/read failure or deadline → ⚪ Unreachable, never a crash.
  JSON parse failure → ⚪ Unreachable + log. Status query is idempotent and
  re-runnable via Refresh.
- **Service:** `Restart=on-failure` with a 5-in-60s start-limit guard so a
  crash-looping binary backs off instead of hammering.

## Testing

- **C unit:** `kmzrun_build_status_json` against fixture dirs — (a) a dir with
  two fake `mesh_binary_*.ply` (known vertex counts in headers) → assert chunks
  / n_points / bytes; (b) empty perception dir → `mesh_present=false`; (c)
  missing dir. Stub `venv_python` with a script that exits 0 / exits 1 / sleeps
  10 s → assert `env_ok` + timeout path.
- **C/MOP integration (on device):** small Python MOP client sends PING, asserts
  STAT parses and matches `ls` reality; verify `mesh_present=false` for
  `the_latest_flight` and `true` with `AEROSCAN_FLIGHT_ID=flight0019`.
- **Kotlin:** extend `AugmentFramingTest` with PING build + STAT parse
  round-trip; `ManifoldStatus` JSON parse (incl. malformed → throws, caught by
  session). Banner-state mapping unit test.
- **Service:** `systemctl --user start` → active; `kill -9` the PID → restarts
  within `RestartSec`; SSH logout (with linger) → stays up; `--enable-boot`
  reboot test once coexistence passes.

## Sequencing

1. **Part B first** (handshake) — independent of the coexistence question and
   delivers the highest-value fix (surfaces the missing-mesh failure up front).
   C frames + builder → Kotlin codec + StatusSession → banner.
2. **Part A** (service) — install script + unit (manual-start), then the
   coexistence test, then flip `--enable-boot` if it passes.

## Risks / open questions

- **Coexistence unknown** — resolved by the test above; default mode is safe
  either way.
- **Deep env probe latency** — 5 s worst case on the accept thread. Mitigated by
  the bound + (later) caching. If open3d import routinely exceeds 5 s on the
  Manifold, raise the bound or fall back to a shallow `access()` check; decide
  during implementation from a measured import time.
- **`run.sh` vs service drift** — both must point at the same binary path; the
  installer reads the path from one place to avoid divergence.
