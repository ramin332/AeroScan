# Manifold PSDK App — Build, Package, Install, Run, Log, Operate

Canonical reference for how the on-drone AeroScan PSDK app (`kmz_runner`, the
augment service that listens on MOP 49154) is deployed to the Manifold 3 mounted
on the M4E's E-Port. Validated live on **2026-05-25**.

> **Repo split.** The PSDK C app and its build/install scripts live in the
> **aeroscan-psdk** repo at `/open_app/dev` on the Manifold, *not* in this
> (`aero-scan`) repo. This doc is the laptop-side reference for the deployment
> model; Manifold-side operational notes live in `/open_app/dev/docs/`. The
> Python augment engine is the only piece this repo ships to the Manifold (via
> `scripts/deploy_to_manifold.sh`, rsynced to `/open_app/dev/aero-scan` and
> editable-installed into the conda env); the PSDK app spawns it as a subprocess.

## TL;DR

- **Production deployment = DJI DPK package**, installed with
  `dji_app_ctl install -i <file>.dpk` (no sudo) and managed through **DJI Pilot
  2**. This is DJI's recommended production path. It is **not** a raw binary
  under a `systemd --user` service.
- The dev `systemd --user` service was a development convenience. It **did**
  survive a cold reboot — auto-start was never actually blocked (see
  [Correction](#correction-the-no-root-cant-autostart-premise-was-wrong)). DPK
  is chosen for DJI-recommended-production + stability + **widget support**, not
  because auto-start was impossible.
- **PSDK Custom Widgets render and are interactive on Pilot's live view only
  under the managed-DPK app.** The raw `systemd` binary never surfaced widgets.
  A future "tap → fly" Pilot widget (`DjiWaypointV3_Action(START)`) therefore
  **requires** the DPK path.
- We do **not** make our app the boot auto-start app. The boot app stays
  **Smart3DExplore**; the pilot manually switches to our app in Pilot after the
  scan (see [Operational workflow](#operational-workflow)).

## Why DPK is the production path

DJI's own tutorial docs (`Payload-SDK-Tutorial/docs/en/…`, in the PSDK source
tree) are explicit:

- `40.manifold-quick-start/03.manifold-platform-capabilities/05.system-tools.md`
  — running the raw binary / `dji_app_ctl` directly is *"intended only for
  development and debugging… Before releasing your application to a production
  environment, be sure to test application installation and startup using DJI
  Pilot."*
- `00.index.md` (Known Issues, Matrice 4T/4E + Manifold 3) — *"manually running
  the PSDK executable may cause abnormal termination"*; DJI's remedy is
  `dji_app_ctl stop Smart3DExplore` (i.e. free the single E-Port slot, then let
  the managed app run).
- `40.manifold-quick-start/03.manifold-platform-capabilities/03.application-management.md`
  — the DPK install/start/stop/auto-start workflow via Pilot; *"only one DPK
  application can run at a time."*

So the three drivers for DPK are: DJI-recommended production posture, runtime
stability (the raw-exec path is documented to terminate abnormally), and PSDK
Custom Widget support on Pilot's live view.

## Correction: the "no root → can't autostart" premise was *wrong*

Earlier session notes (and the spec's "Part A — systemd `--user` service")
reasoned: the `dji` user has no root → can't `loginctl enable-linger` → therefore
`systemd --user` **cannot** auto-start on boot → therefore we **must** use a DPK.

**That premise was false.** The `systemd --user` service actually **did survive a
cold reboot**: the Manifold graphically auto-logs-in the `dji` user, which starts
the `systemd --user` instance, which starts the enabled service — and the app
even reconnected to the M4E E-Port. Auto-start was never the blocker.

DPK is chosen for the reasons above (production posture, stability, widgets), not
because the dev path couldn't auto-start. Any doc repeating the no-root /
can't-autostart reasoning as the *justification* for DPK should be corrected to
this.

## Deployment pipeline

```
aeroscan-psdk source (/open_app/dev)
   │  build (cmake/make; the repo's run.sh also emits a .dpk)
   ▼
build/dpk/psdk-demo_v01.00.00.00.dpk
   │  dji_app_ctl install -i <file>.dpk      (no sudo; updates same name+version in place)
   │  — or — copy .dpk to RC, Pilot → Application Management → "+" → pick .dpk
   ▼
installed to /open_app/<appname>   (here /open_app/psdk-demo)
   │  DJI auto-generates a root SYSTEM service (see Runtime mechanics)
   ▼
Pilot → Enter Camera View → app icon on left edge → tap to start (yellow = running)
   │  optional: Application Management → More (⋯) → Set as Auto-start on Boot
   ▼
app binds MOP 49154, registers widgets, WaypointV3 callbacks active
```

The build was installed this session as:

```
dji_app_ctl install -i /open_app/dev/Payload-SDK-3.16.0/build/dpk/psdk-demo_v01.00.00.00.dpk
```

(no sudo; installing a `.dpk` with the same name+version updates the existing app
in place).

## Runtime mechanics (verified)

A DPK installs to `/open_app/<appname>` (here `/open_app/psdk-demo`). DJI
auto-generates a **root systemd SYSTEM service** at
`/data/overlay/etc/systemd/system/<appname>.service`:

```ini
ExecStart=/open_app/<appname>/bin/<binary>
WorkingDirectory=/open_app/<appname>/
Restart=on-failure
User=app<appname>
Group=nvidia
CPUQuota=640%
MemoryMax=10G
OOMScoreAdjust=-500
WantedBy=multi-user.target          # true boot auto-start, no login needed
```

- `WantedBy=multi-user.target` means the managed service is a real boot-time
  SYSTEM service — it does not depend on the `dji` user logging in (unlike the
  dev `systemd --user` path, which rode the graphical auto-login).
- The app runs as the sandboxed user **`app<appname>`** (not `dji`), with the
  resource limits above.

### Augment subprocess still works under the sandbox

The PSDK app spawns the Python augment engine as a subprocess. Under the
sandboxed `app<appname>` user this still works because the relevant paths are
group/other-readable:

- `/open_app/dev/{,miniforge3/…,/src}` are `775` → `app<appname>` can exec the
  venv Python and `import flight_planner`.
- `/blackbox` is `755` → `app<appname>` can traverse it to read the perception
  mesh.

## Log access reality

Because the managed app runs as `app<appname>`, the **`dji` user cannot read the
app's own files or logs** — `/open_app/psdk-demo` is permission-denied to `dji`.
Two ways to get logs:

1. **Pilot → Application Management → Log Export** (writes the managed app's logs
   to USB/SD). This is the supported managed-app log path.
2. **`journalctl -u psdk-demo.service`** for the app's console/stdout output —
   this works for `dji` because `dji` is in the `adm` group.

This is a change from the dev `systemd --user` / raw-exec path, where the app ran
as `dji` and wrote logs under its CWD (`data/logs/`) which `dji` could read
directly.

## Pilot 2 operation (RC steps)

- **Install / update:** Pilot → **Application Management** → **`+`** (top-right) →
  pick the `.dpk` (copied to the RC via USB/SD beforehand).
- **Start / stop:** Pilot → **Enter Camera View** → app icons on the **left
  edge** → tap to start. Icon turns **yellow** = running; **gray** = stopped.
  **Only one app runs at a time** (the single E-Port PSDK slot).
- **Auto-start:** Application Management → per-app **More (⋯)** → **Set as
  Auto-start on Boot**. Only one app can be the boot app.

## Operational workflow

We do **not** set our app as the boot auto-start app — the boot app stays
**Smart3DExplore**. The intended field flow:

1. Power aircraft, link RC → boot app **Smart3DExplore** starts (auto).
2. Pilot flies the Smart Auto-Exploration scan → perception mesh accumulates in
   `/blackbox/`.
3. In Pilot, **switch to our app** (`psdk-demo`) — tap its icon on the left edge
   of the camera view (Smart3DExplore stops; our app binds MOP 49154).
4. Run the augment from the RC-companion app over MOP.

Manual app-switching between Smart3D and our app is **expected and acceptable** —
it is inherent to DJI's one-app-at-a-time E-Port rule, not a defect.

## Widgets

A managed-DPK app's PSDK **Custom Widgets render and are interactive on Pilot's
live view** — verified this session (the stock gimbal-mover widget worked). The
earlier raw-`systemd` binary did **not** surface widgets in Pilot.

Consequence: the Pilot "fly / start mission" widget (tap →
`DjiWaypointV3_Action(START)`) **requires the DPK path**. As of 2026-05-25 the app
ships a **custom** AeroScan `widget_config.json` with a **Fly button** (the stock
gimbal-mover is gone), and the tap→`Action(START)` handler is **wired**
(`kmz_runner.c`: `HandleApprovalFrame` → upload → `READY_TO_FLY` → Fly tap →
`Action(START)`, gated on readiness; mission/action state callbacks registered).
What remains unproven is on-hardware: the first M4E WaypointV3 upload + START has
never run (no mesh ever reached it). See the cockpit plan
(`docs/superpowers/plans/2026-05-25-aeroscan-mission-cockpit.md`).

## App identity constraint

The DJI-registered auth triple must stay:

```
USER_APP_NAME = "Payload test ap"
USER_APP_ID   = "183281"            (+ key / license)
```

The DPK **display** name (`psdk-demo`) is cosmetic packaging metadata only.
**Renaming the registered identity risks breaking `DjiCore_Init` auth**, so it
must stay as-is unless the app is re-registered with DJI. (This supersedes the
earlier open question of whether to rename the app to "AeroScan".)

## Data caveat: volatile mesh (the ring-buffer model)

`/blackbox` is a **~30-slot rolling ring buffer that cycles** (verified
2026-05-25). The operational model:

- Slots `flight0020`–`flight0049` are **reused**; slot numbers are **not
  chronological** (confirmed by mtimes, not by slot index). Use the
  `the_latest_flight` symlink, never a `flightNNNN` guess.
- **A new flight slot is created by an aircraft POWER-CYCLE, not by app updates.**
  A DPK install / rebuild does **nothing** to `/blackbox`.
- The perception mesh (`mesh_binary_*.ply`) exists for a slot **only if a Smart3D
  Auto-Exploration scan ran in that session**, and it is **evicted as the buffer
  cycles**.
- **Verified 2026-05-25: the previously-only mesh (`flight0019`) has been pruned —
  no flight currently has a mesh.** So a *successful* augment now requires a
  **fresh Smart3D scan** (there is no longer a fallback flight with a mesh).

**Operational rule (intended flow):** fly a Smart3D scan, then **immediately** run
the RC/augment so the fresh mesh is used before the buffer churns it — do not do
many reboots between scan and augment. The readiness handshake
(`docs/superpowers/specs/2026-05-25-manifold-service-readiness-handshake-design.md`)
surfaces "does the latest flight have a mesh" to the pilot before they ship the
augment payload, and the rc-companion is being changed to **refuse to augment when
PING/STAT reports no mesh** rather than upload a KMZ and run a doomed augment. See
memory `manifold-mesh-volatile`.

## See also

- `docs/architecture/kmz-flow.md` — the WaypointV3 execution transport this app
  uses (`DjiWaypointV3_UploadKmzFile` + `Action(START)`).
- `docs/architecture/rc-companion-summary-exec.md` — the RC ↔ Manifold augment
  architecture this app is the Manifold endpoint of.
- `docs/architecture/rc-companion-bringup.md` — the MOP transport bring-up.
- `CLAUDE.md` → "KMZ Execution Transport (Pilot 2 / PSDK / Manifold)".
