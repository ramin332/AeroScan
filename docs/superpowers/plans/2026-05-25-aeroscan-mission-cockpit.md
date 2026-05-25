# AeroScan Mission Cockpit — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development.
> Steps use checkbox (`- [ ]`) syntax. Branch (both repos): **`feat/manifold-readiness-handshake`**.
> The on-device C lives in the **aeroscan-psdk** repo on the Manifold (`/open_app/dev`,
> edited via scp + built there). The RC-companion Kotlin lives in this laptop repo
> (`rc-companion/`). Mesh data is parked.

## Status — 2026-05-25 (most of this was already built; scope trimmed)

Reading `kmz_runner.c` before building (DRY) revealed the cockpit backend was
~80% already implemented. Actual state:

- **Phase 1 — DONE** (Manifold `6fd6a34`): self-contained DPK (lean `app.json`
  `userconfig` → `widget_file`; `.dpk` 159 MB → 432 KB; exe-relative config load).
  Verified in Pilot: the **AeroScan Fly button renders** (no more stock gimbal mover).
- **Phase 2 — DONE**: `Status()` already pushed every state transition to the
  floating window; added live **mission progress** (Manifold `3f63a84`) — the
  WaypointV3 state callback now shows "Mission: flying — waypoint N" for active
  states. (Verifies live only on a real flight.)
- **Phase 4 — already built**: the Fly button calls `DjiWaypointV3_Action(START)`
  gated on `READY_TO_FLY`; mission + action state callbacks registered.
- **Phase 5 — already started**: `DjiFcSubscription_Init` + `STATUS_FLIGHT`
  (START preconditions).
- **Phase 3 (HMS alerts) + idle-readiness — DROPPED** (kept simple): the
  rc-companion readiness banner already surfaces pre-flight warnings
  (no mesh / env bad / unreachable), so no on-aircraft HMS/buzzer duplication.

**Remaining is the data blocker, not code:** a flight whose `mesh_binary_*.ply`
still exists on `/blackbox`, to exercise augment → upload → fly end-to-end and
verify mission progress on the window live.

**Goal:** Turn the Pilot 2 on-aircraft live view into an AeroScan "mission cockpit" —
a floating-window status line (readiness + augment progress + mission state),
action-bar buttons (Augment, FLY), HMS alerts, and live telemetry — and make the
DPK self-contained so it actually renders *our* widgets.

**Architecture:** PSDK C app on Manifold 3 / M4E. Floating window =
`DjiWidgetFloatingWindow_ShowMessage` (≤255 B, ~1–2 Hz). Buttons via
`DjiWidget_RegHandlerList`. Mission via `DjiWaypointV3_*`. Alerts via
`DjiHmsCustomization_*`. Telemetry via `DjiFcSubscription_*`. Config bundled into
the DPK via `app.json` `userconfig` and loaded from a runtime-relative path.

**Tech stack:** C (PSDK 3.16.0), CMake + `build_dpk.sh`, JSON widget config; the
existing Python augmenter (invoked as a subprocess); Kotlin/Compose (RC companion).

---

## Grounded API reference (from on-device headers — docs were incomplete)

```c
// dji_widget.h
#define DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN 255
T_DjiReturnCode DjiWidgetFloatingWindow_ShowMessage(const char *str);     // push status (≤255 B)
T_DjiReturnCode DjiWidget_RegDefaultUiConfigByDirPath(const char *dir);
T_DjiReturnCode DjiWidget_RegUiConfigByDirPath(lang, screen, const char *dir);
T_DjiReturnCode DjiWidget_RegHandlerList(const T_DjiWidgetHandlerListItem*, uint32_t); // EVERY config index needs a handler or NULL-deref SIGSEGV

// dji_waypoint_v3.h
typedef enum { START=0, STOP=1, PAUSE=2, RESUME=3 } E_DjiWaypointV3Action;
typedef enum { IDLE=0, PREPARE=16, TRANS_MISSION=32, MISSION=48, BREAK=64,
               RESUME=80, RETURN_FIRSTPOINT=98 } E_DjiWaypointV3MissionState;
typedef struct { E_DjiWaypointV3MissionState state; uint32_t wayLineId;
                 uint16_t currentWaypointIndex; } T_DjiWaypointV3MissionState;
T_DjiReturnCode DjiWaypointV3_Init(void);
T_DjiReturnCode DjiWaypointV3_UploadKmzFile(const uint8_t *data, uint32_t dataLen);
T_DjiReturnCode DjiWaypointV3_Action(E_DjiWaypointV3Action action);
T_DjiReturnCode DjiWaypointV3_RegMissionStateCallback(WaypointV3MissionStateCallback);

// dji_hms_customization.h — custom code range 0x1E020000–0x1E02FFFF; persists until eliminated
DjiHmsCustomization_Init();  RegDefaultHmsTextConfigByDirPath / RegHmsTextConfigByDirPath(dir);
DjiHmsCustomization_InjectHmsErrorCode(code, level);   // level: NONE/HINT/WARN/CRITICAL/FATAL
DjiHmsCustomization_EliminateHmsErrorCode(code);
DjiHmsCustomization_AlarmEnhancedCtrl(action, setting); // RC vibration/buzzer — Manifold-3 only
```

**Caveats to verify on-device, not assume:** M4E isn't explicitly listed as
WaypointV3-supported (Manifold-3 implied — device test in Phase 4); telemetry table
has no M4E column (use M4T proxy; prefer `RTK_POSITION`/`GPS_POSITION` over
`POSITION_FUSED`); no SDK gate on `Action(START)` and a new `UploadKmzFile`
silently overwrites — the human gate + overwrite-safety are ours.

## Execution model (device-coupled)

- **Unit-testable (PSDK-free, native gcc — subagent-draftable):** the status-line
  *formatter* (status struct → ≤255 B string), augment-progress parsing
  (`[N/7] …` → percent/label), HMS code/level mapping. Test like `kmzrun_status`.
- **Integrator (device-coupled — controller does on Manifold):** all `Dji*` glue,
  `app.json`/CMake/`setup_psdk.sh` packaging, build + `dji_app_ctl install` + Pilot
  verify. **User** does the on-RC visual/flight verification.
- **Parallel track (laptop, subagent):** RC-companion PING/STAT banner — see
  `2026-05-25-manifold-readiness-handshake.md` Phase 3 (in progress).

---

## Phase 1 — DPK packaging fix (foundational: make the DPK render OUR widgets)

**Files:**
- Manifold: `Payload-SDK-3.16.0/samples/sample_c/platform/linux/manifold3/app_json/app.json` (or a repo-owned copy)
- Manifold: `src/manifold3_app/aeroscan_widgets.c` (load path)
- Manifold: `scripts/setup_psdk.sh` and/or `run.sh` (place config + our app.json)
- Manifold: `src/manifold3_app/widget_file/{en_big_screen,cn_big_screen}/` (rename from `widgets/` to DJI's convention)

- [ ] **1.1** Confirm `build_dpk.sh` userconfig path-flattening: a `userconfig` dir
  entry lands under the app's `/open_app/<app>/` runtime dir; determine the exact
  relative path the running binary must use (`DjiUserUtil_GetCurrentFileDirPath`
  vs `/proc/self/exe` dir vs cwd). Document the resolved path.
- [ ] **1.2** Set `userconfig` to bundle ONLY our config dir(s) (`widget_file`,
  later `hms_text`) — not `../../../../../../samples`. Keep `user_app_id:183281`,
  `name_en:psdk-demo`.
- [ ] **1.3** Change `aeroscan_widgets.c` to load config from the runtime-resolved
  path (works for BOTH dev binary and installed DPK), replacing the hardcoded
  `/open_app/dev/src/manifold3_app/widgets/`. Keep en + cn + default registration.
- [ ] **1.4** Ensure the build places `widget_file/` so both paths resolve: dev
  build copies it next to the binary; DPK bundles it via `userconfig`. Wire into
  `setup_psdk.sh`/CMake.
- [ ] **1.5** Build, `dji_app_ctl install`, start in Pilot → **verify (user):
  Pilot shows the AeroScan Fly button + floating window + textbox, NOT the gimbal
  mover.** Also verify the dev binary (`./run.sh`) still loads the same config.
- [ ] **1.6** Commit (Manifold repo).

## Phase 2 — Floating-window status line (on-aircraft readiness display)

**Files:** Manifold: `src/manifold3_app/cockpit_status.{c,h}` (new),
`test/test_cockpit_status.c` (new, native-gcc), wire into `kmz_runner.c`.

- [ ] **2.1** TDD the PSDK-free formatter `cockpit_status_format(const status fields, char out[256])`
  → e.g. `"mesh ✓ 54chk/1.0M · env ✓ · augment: idle"`, clamped to 255 B. Reuse
  the `kmzrun_status` fields (mesh_present, mesh_chunks, n_points, env_ok) +
  augment phase + mission state. Native-gcc test with fixtures (subagent-draftable).
- [ ] **2.2** Add a ~1 Hz status timer/thread in the app that builds the string and
  calls `DjiWidgetFloatingWindow_ShowMessage`. Gate on
  `DjiWidgetFloatingWindow_GetChannelState`. (Integrator.)
- [ ] **2.3** Feed it: idle shows readiness (recompute `kmzrun_status` periodically
  or on demand); during augment shows progress (Phase 3); during flight shows
  mission state (Phase 4).
- [ ] **2.4** Build/install; **user verifies the status line updates live in Pilot.**
- [ ] **2.5** Commit.

## Phase 3 — Augment from the widget + HMS alerts

**Files:** Manifold: `kmz_runner.c` (Augment button handler + progress parse),
`cockpit_hms.{c,h}` (new), `hms_text/{en,cn}/` config + `app.json` userconfig.

- [ ] **3.1** Add an "Augment Now" button to `widget_config.json` (+ its handler —
  remember every index needs a handler). Handler kicks the existing augment
  subprocess path (or re-runs the last received mission).
- [ ] **3.2** TDD a PSDK-free parser: augmenter stdout line → `{phase 1..7, label}`
  (the augmenter prints `[N/7] …`). Drives the floating-window "augment: 3/7 …".
- [ ] **3.3** HMS: register `hms_text/{en,cn}` config; map results to custom codes
  (0x1E02xxxx): HINT "augment ready", WARN "no mesh / env bad", FATAL "augment
  failed" + `DjiHmsCustomization_AlarmEnhancedCtrl` RC buzz. Eliminate stale codes.
- [ ] **3.4** Build/install; **user verifies** augment-from-widget + alerts.
- [ ] **3.5** Commit.

## Phase 4 — FLY trigger + mission state (the human-gated START)

**Files:** Manifold: `kmz_runner.c` (FLY handler + Arm gate + WaypointV3 wiring).

- [ ] **4.1** Add "Arm" switch + "FLY" button to the config (+ handlers). FLY only
  acts when Arm is on (the human gate — SDK has none).
- [ ] **4.2** Wire `DjiWaypointV3_Init` + `RegMissionStateCallback` (before upload).
  FLY → `DjiWaypointV3_UploadKmzFile(augmented_kmz, len)` → `Action(START)`.
  Surface the FC validity-check error code on failure (HMS + floating window).
- [ ] **4.3** Mission-state callback → floating window: map state enum to text
  (`MISSION` → "flying · wp {currentWaypointIndex}/{total}", total from the KMZ;
  `BREAK` → "paused", `RETURN_FIRSTPOINT` → "returning", etc.).
- [ ] **4.4** **DEVICE TEST (user, props-off/bench first):** confirm M4E accepts
  WaypointV3 upload + START; confirm the state callback streams. THIS IS THE GO/NO-GO
  for the whole fly path on M4E.
- [ ] **4.5** Commit.

## Phase 5 — Telemetry status

**Files:** Manifold: `cockpit_telemetry.{c,h}` (new), wire into the status line.

- [ ] **5.1** `DjiFcSubscription_Init` + subscribe: `STATUS_FLIGHT` (on-ground/in-air),
  `RTK_CONNECT_STATUS`, `RTK_POSITION` (or GPS fallback), `VELOCITY`,
  `BATTERY_SINGLE_INFO_INDEX1`, `FLIGHT_ANOMALY`. (Integrator; verify topic
  availability on M4E.)
- [ ] **5.2** Fold a compact telemetry summary into the floating-window line
  (RTK fix? battery? in-air?) and use it as a START precondition check (warn if no
  RTK / low battery / no home point before FLY).
- [ ] **5.3** Build/install; **user verifies** telemetry shows + START gating works.
- [ ] **5.4** Commit.

## Parallel — RC-companion readiness banner (separate plan, subagent)

Per `2026-05-25-manifold-readiness-handshake.md` Phase 3: PING/STAT framing +
StatusSession + HomeViewModel banner + HomeScreen, consuming the `kmzrun_status`
JSON. **In progress (subagent).** The C-side PING/STAT handler (that plan's Phase 2)
is wired into `kmz_runner.c` to match the banner's protocol; fold it in alongside
Phase 2 here. Also fold in **stale-MOP-channel recovery** (today's "unable to send
until restart") when touching the runner's accept loop.

## Self-review notes

- Every new widget index MUST have a registered handler (NULL → SIGSEGV). Add
  handlers and config entries together.
- Floating-window strings are hard-capped at 255 B — the formatter clamps.
- `Action(START)` has no SDK gate and upload overwrites silently — Arm switch + a
  "mission already uploaded?" guard are ours.
- Keep the registered app identity (`183281` / "Payload test ap") unchanged.
