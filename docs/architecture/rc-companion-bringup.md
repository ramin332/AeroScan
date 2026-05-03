# RC Companion Bring-Up: First End-to-End MOP Push

How we got `rc-companion/` (Android MSDK V5 app on RC Plus 2) to push a KMZ over
OcuSync → aircraft → E-Port → Manifold via DJI's MOP pipeline, and what was
broken on the way. First successful end-to-end test: 2026-05-01 09:53.

> **Note (2026-05-03):** This document captures *how the RC→Manifold MOP path
> was made to work*. The wireless bulk-transport architecture has since been
> superseded by reading the Smart3D mesh directly off the Manifold's
> `/blackbox/the_latest_flight/dji_perception/1/` (faster, denser, no transport
> needed). See `rc-companion-summary-exec.md` for the current architecture and
> the **Step 0 findings** section at the bottom of this file for the
> investigation that drove the pivot. The MOP path is kept as a control-message
> channel and as a fallback for sub-500 KB payloads.

## TL;DR

- **End-to-end MOP transport from RC to Manifold works.** The Android app picks
  a KMZ, streams it as 1 KB chunks on MOP channel `49152`, and the Manifold's
  passive listener (`rc_probe.c`) receives the bytes plus our 16-byte MD5 trailer
  before a clean disconnect.
- **Throughput is ~5 KB/s, hard-limited by OcuSync uplink** (DJI documents 24-48
  Kbps for MSDK upstream — we measured 6.5 KB/s, on the high side of spec). Not
  a code issue; it's the radio's physical layer ceiling. KMZs over a few MB are
  impractical over this transport.
- **rc_probe currently logs received bytes only — it does not persist them to
  disk.** Extending it to write to `/open_app/dev/data/received/` is the next
  PSDK-side task before this is operationally useful.
- **Three Android-side and one PSDK-side fixes were load-bearing.** Documented
  below so they survive future refactors.

## Architecture (working state)

```
┌────────────────────────┐                 ┌──────────────────┐
│  RC Plus 2 (Android)   │                 │  Matrice 4E      │
│  ───────────────────   │ ──── OcuSync ── │  (aircraft)      │
│  rc-companion APK      │     uplink      │                  │
│  (com.aeroscan.         │  ~24-48 Kbps   │  MOP router      │
│   rccompanion)         │                 │     │            │
│                        │                 │     │  E-Port    │
│  MSDK V5               │                 │     ▼  USB-C     │
│   PipelineManager      │                 │  ┌────────────┐  │
│   .connectPipeline(    │                 │  │ Manifold 3 │  │
│     LEFT_OR_MAIN,      │                 │  │ ────────── │  │
│     channelId=49152,   │                 │  │  PSDK      │  │
│     deviceType=PAYLOAD,│                 │  │  V3.16.0   │  │
│     mode=STABLE)       │                 │  │  rc_probe  │  │
└────────────────────────┘                 │  │  (49152)   │  │
                                           │  └────────────┘  │
                                           └──────────────────┘
```

Critical configuration for the pipeline to work:

| Knob | Value | Why |
|---|---|---|
| `ComponentIndexType` | `LEFT_OR_MAIN` | M4E exposes a single E-Port; this is the canonical slot. |
| `PipelineDeviceType` | **`PAYLOAD`** (not `ONBOARD`) | Manifold runs a **PSDK** application (`dji_sdk_demo_on_manifold3`), so MSDK must address it as a payload, not an onboard computer. M4E has no OSDK pathway. |
| `TransmissionControlType` | `STABLE` | Reliable transport. Both sides must agree (PSDK side uses `DJI_MOP_CHANNEL_TRANS_RELIABLE`). |
| MOP channel ID | `49152` | DJI sample default for "normal" channel; must be ≥ 49152 (user range). |
| MSDK chunk size | 1024 bytes | Per MSDK V5 docs, ≤ 1 KB per `writeData` call on the upstream channel. |

## Bugs Fixed During Bring-Up

### 1. App SIGABRT'd on launch — `JNI_OnLoad` NPE in `libSdkyclx_clx.so`

**Symptom:** `JNI DETECTED ERROR IN APPLICATION: field operation on NULL object`
during `com.cySdkyc.clx.Helper.install(this)` in
`AeroScanApp.attachBaseContext`. App crashed before `setContent`.

**Root cause:** `<uses-feature android:name="android.hardware.usb.accessory"
android:required="true">` in the manifest. DJI's native loader reflects on
package metadata during `JNI_OnLoad`; `required="true"` on a feature an Activity
later claims via intent-filter leaves a JNI lookup returning `null`, which the
native code dereferences without checking. The MSDK V5 sample's manifest never
declares `usb.accessory required="true"` — only `usb.host required="false"`.

**Fix:** Removed the `usb.accessory` `<uses-feature>` line. Left `usb.host
required="false"` — harmless, declares optional support.

### 2. Connected → Disconnected flicker on every USB attach intent

**Symptom:** Banner flipped from `M4E connected (id 0)` back to `MSDK
registered, waiting for aircraft` within a second of binding. MSDK's live-view
HEVC decoder (which V5 always spins up on bind) was visibly tearing down its
surface (`V5_LIVE_VIEW: removeSurface ... currentSize = 0`).

**Root cause:** `USB_ACCESSORY_ATTACHED` was filtered on `MainActivity` with
`launchMode="singleTask"`. The OcuSync radio re-fires the attach intent every
time the SDK polls it. Each intent reordered the task and delivered
`onNewIntent` to the foreground `MainActivity`, which drops MSDK's bound
surface.

**Fix:** Moved the filter to a dedicated `UsbAttachActivity` that does nothing
but `finish()` immediately. The system's mere delivery of the intent to *some*
Activity in the package is what wakes MSDK to bind the OcuSync radio — a
no-op absorber satisfies that without disturbing `MainActivity`.

```xml
<activity
    android:name=".UsbAttachActivity"
    android:exported="true"
    android:launchMode="singleInstance"
    android:noHistory="true"
    android:excludeFromRecents="true"
    android:theme="@android:style/Theme.Translucent.NoTitleBar">
    <intent-filter>
        <action android:name="android.hardware.usb.action.USB_ACCESSORY_ATTACHED" />
    </intent-filter>
    <meta-data
        android:name="android.hardware.usb.action.USB_ACCESSORY_ATTACHED"
        android:resource="@xml/accessory_filter" />
</activity>
```

### 3. MOP `connect` refused — `CONNECTION_REFUSED`

**Symptom:** Banner stable on `M4E connected (id 0)`, but Send failed with
`MOP connect failed: errorCode='CONNECTION_REFUSED', description='The target
device refuses to connect.'`. Manifold's `rc_probe` accept loop never logged
`PEER CONNECTED` — connect was rejected below the application accept layer.

**Root cause:** `MopFileSender` used `PipelineDeviceType.ONBOARD`. ONBOARD is
the slot for OSDK applications on aircraft like the Matrice 300 series.
Manifold here runs **PSDK** (`dji_sdk_demo_on_manifold3`, PSDK V3.16.0), and
M4E has no OSDK pathway over E-Port. The aircraft's MOP router NACKs the
connect at the protocol layer, MSDK reports `CONNECTION_REFUSED`, PSDK never
sees it.

**Fix:** Single-token change in `MopFileSender.kt`:

```kotlin
- private val deviceType: PipelineDeviceType = PipelineDeviceType.ONBOARD,
+ private val deviceType: PipelineDeviceType = PipelineDeviceType.PAYLOAD,
```

### 4. PSDK side missing the listener

**Symptom:** Even with the PAYLOAD fix, `rc_probe` was not in the boot log of
the running PSDK demo. `src/manifold3_app/rc_probe.c` had been deleted from the
Manifold's working tree.

**Fix:** PSDK-side restoration of `rc_probe.c`/`rc_probe.h` and re-wiring of
`AeroscanRcProbe_Init()` in `main.c`. After this, the boot log shows:

```
rc_probe: passive MOP listeners armed (49152, 49153)
rc_probe[mop:49152]: bound, accepting (passive listener — never sends)
rc_probe[mop:49153]: bound, accepting (passive listener — never sends)
```

## Operational Workaround: Pilot 2 Cold-Start Prime

Independent of code: on RC Plus 2, MSDK V5's first cold bind to the M4E often
flickers. Opening **DJI Pilot 2** once after RC power-up primes the OcuSync
radio at the OS level; backgrounding Pilot 2 (home button — not force-stop)
keeps the radio warm. Subsequent AeroScan launches bind cleanly.

This is not something we can fix in app code — it's an artifact of how MSDK V5
binds the radio. It also lines up with the operational flow: the pilot opens
Pilot 2 anyway to tap the PSDK widget for takeoff.

## Performance: OcuSync Uplink Ceiling

Measured on first successful run (2026-05-01 09:37–09:53):

- 1024-byte chunks at ~6.5 chunks/sec → **~6.5 KB/s sustained**
- Matches DJI's documented **24–48 Kbps** ceiling for MSDK upstream
  (reliable *and* unreliable — both capped identically)
- OcuSync is asymmetric: downlink (aircraft→RC) is 16-20 Mbps for video, uplink
  (RC→aircraft) is bandwidth-narrow because it's mostly meant for control.

Practical KMZ-size budget:

| KMZ size | Transfer time |
|---|---|
| 50 KB | ~10 sec |
| 500 KB | ~1.5 min |
| 1 MB | ~3 min |
| 5 MB | ~15 min |

**Implication for the source-of-KMZ architecture.** A few-hundred-KB hand-crafted
NEN-2767 KMZ is fine. A multi-MB KMZ that ships orthos, point clouds, or large
`res/` attachments is not. Strip non-essential entries on the RC before pushing
(`wpmz/template.kml` + `wpmz/waylines.wpml` are the load-bearing ones).

## Open Follow-Ups

1. **Persist received bytes on Manifold.** `rc_probe.c` currently only logs hex
   previews. Mirror the `text_receiver.c` pattern: `fopen(received_path, "wb")`
   on `PEER CONNECTED`, `fwrite(recvBuf, realLen, 1, fp)` on each chunk,
   `fclose` on `peer disconnected`. Filename should come from the AeroScan
   header's `nameLen + name` field (see `Framing.kt`), not from peer-disconnect
   timestamp.
2. **Validate the AeroScan header + MD5 trailer on PSDK side.** Today's
   rc_probe is protocol-agnostic — it would happily accept any byte stream. A
   minimal parse of the `AERS` magic + size + trailing 16-byte digest gives
   us proof-of-correct-receipt and a sanity check before invoking
   `DjiWaypointV3_UploadKmzFile`.
3. **Bridge to Waypoint V3 execution.** Once the KMZ is on the Manifold,
   `DjiWaypointV3_UploadKmzFile(bytes, len)` + `DjiWaypointV3_Action(START)` is
   the documented path to push it onto the aircraft and kick off the mission
   from a PSDK Custom Widget tap. See `docs/architecture/kmz-flow.md`.
4. **KMZ-stripping mode on RC.** Add an option in the file-pick step to drop
   `res/`, optional thumbnails, etc. before sending — typically halves transfer
   time for AeroScan-generated missions.
5. **Estimated-time UI.** The current progress bar shows bytes/total. Adding
   `ETA = (total − sent) / 5 KB/s` lets the pilot know up front whether the
   transfer will be 10 seconds or 3 minutes.

## Reference Logs (2026-05-01)

### Boot — PSDK on Manifold

```
05-01 09:36:21.582  core:        Identify device : manifold3
05-01 09:36:23.091  core:        Identify AircraftType = Matrice 4E,
                                 MountPosition = Extension Port
05-01 09:36:24.598  user:        aeroscan_widgets: registered minimal widget set
05-01 09:36:30.591  user:        text_receiver: listening on MASTER + SLAVE _RC_APP
05-01 09:36:30.591  user:        rc_probe: passive MOP listeners armed (49152, 49153)
05-01 09:36:32.594  user:        rc_probe[mop:49152]: bound, accepting
05-01 09:36:32.594  user:        rc_probe[mop:49153]: bound, accepting
```

### KMZ Push Trace — From RC

```
09:37:57.923  rc_probe[mop:49152]: rx 1024 bytes: 0d 7f 78 4e 2c be ...
09:37:58.067  rc_probe[mop:49152]: rx 1024 bytes: b7 a5 4c a4 b5 a0 ...
... (many similar)
09:53:15.817  rc_probe[mop:49152]: rx 1024 bytes: 2f 4d 69 6a 61 6e 64 65 ...   (= "/MijandeExtra/...", ASCII filename inside zip)
09:53:16.051  rc_probe[mop:49152]: rx  909 bytes: 65 73 2f 70 6c 79 ...        (last data chunk, partial)
09:53:16.113  rc_probe[mop:49152]: rx   16 bytes: 6b ba 6a 53 3a 00 3c a6 ...  (16-byte MD5 trailer per Framing.kt)
09:53:16.137  rc_probe[mop:49152]: peer disconnected
```

ASCII bytes from `/MijandeExtra/points/pointcloud` confirm a real KMZ payload
made it through (ZIP entry path); 16-byte trailer matches our `Framing.kt`
MD5 finish; clean `peer disconnected` confirms MSDK side closed the pipeline
cleanly after sending the trailer.

### MSDK MOP Sender

`rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/mop/MopFileSender.kt`
defaults to:

```kotlin
componentIndex = ComponentIndexType.LEFT_OR_MAIN
channelId      = MopConstants.MOP_CHANNEL_ID    // 49152
deviceType     = PipelineDeviceType.PAYLOAD     // load-bearing on M4E
mode           = TransmissionControlType.STABLE
```

### PSDK MOP Listener

`/open_app/dev/src/manifold3_app/rc_probe.c` on the Manifold:

```c
static const T_ProbeBindSpec s_probeBinds[] = {
    {49152, DJI_MOP_CHANNEL_TRANS_RELIABLE, "mop:49152"},
    {49153, DJI_MOP_CHANNEL_TRANS_RELIABLE, "mop:49153"},
};
```

`DJI_MOP_CHANNEL_TRANS_RELIABLE` on PSDK side ↔ `TransmissionControlType.STABLE`
on MSDK side. Mismatch produces silent transport-layer NACK without reaching
`Accept`.

---

## Step 0 Findings (2026-05-03) — why the architecture pivoted

After the wireless transport above was working, we did a read-only investigation
of the Manifold's filesystem to answer "where does the data we're trying to move
actually live?" Findings here drove the architecture pivot documented in
`rc-companion-summary-exec.md`.

### Manifold blackbox layout

`/blackbox/` contains per-flight directories `flight0006` … `flight0036+`,
owned by root, totalling 18 GB. Each Smart Auto-Exploration flight populates:

```
/blackbox/flightNNNN/
├── camera/                      (empty in samples)
├── dji_mcu/                     (encrypted MCU log + idx)
├── dji_perception/1/
│   ├── mesh_binary_*.ply        ← the building geometry, in 10 MB chunks
│   ├── *.enc                    (encrypted raw data, expl_plan, vp_log, …)
│   └── vp_storage.json          (storage layout config — not the mission)
├── psdk/                        (the running PSDK app's log)
├── system/
├── latest_Smart3DExplore        (1-byte marker — "this flight ran Smart3D")
└── Smart3DExplore_dynamic_ch_v2.json   (IPC channel config — not the mission)
```

DJI maintains symlinks `/blackbox/the_latest_flight → flightNNNN/` so we never
need to know flight numbers — just follow the symlink.

### What the PLY chunks contain

Sample: `/blackbox/flight0016/dji_perception/1/mesh_binary_*.ply`

- 54 PLY files, ~10 MB each (1.0 GB total for one flight).
- Format: PLY binary little-endian, vertex+normal point cloud, no faces.
- Local meter-scale ENU frame anchored on takeoff, **yaw-aligned to the aircraft's heading at takeoff** (not magnetic north / KMZ's ENU). Per-flight rotation around Z is needed to register into the KMZ's frame — verified on flight0016/Mijande: ICP gives fitness 0.986, RMSE 0.468 m, total yaw 85.74°, translation effectively zero. Open3D's `registration_icp` with a coarse-yaw seed (0/90/180/270°) handles this in ~10 s on a 20 cm voxel-downsampled cloud.
- Bbox of merged cloud: ~111 × 60 × 20 m for a multi-building site (Mijande).

### Manifold vs KMZ: same site, two different representations

The thing that surprises is the size delta. Quick numbers, both for Mijande:

| Source | Where | Points | On disk |
|---|---|---|---|
| `kmz/Mijande.kmz` `cloud.ply` | inside KMZ on RC | 416 K | 12.9 MB (zip) |
| `/blackbox/flight0016/dji_perception/1/mesh_binary_*.ply` | on the drone | **18.8 M** | **~1.0 GB** (uncompressed PLY: 6 floats × 4 bytes per point) |

Manifold has **45× more points**. They are **not the same data in two places**:

- **Manifold = raw perception stream.** During the Smart3D flight, the perception system dumps mesh chunks as the drone scans, every few seconds. Unfiltered, dense, noisy. Size scales with scan duration:

| Flight | Chunks | Total |
|---|---|---|
| flight0008 | 15 | 268 MB |
| flight0014 | 64 | 1.4 GB |
| flight0016 (Mijande) | 54 | 1.0 GB |
| flight0019 | 25 | 350 MB |

- **KMZ `cloud.ply` = DJI's post-flight thumbnail.** Decimated to ~10 cm point spacing, sized for Pilot 2's 3D Tiles preview, not for analysis. That's why your `Mijande.kmz` is only 15 MB — it's the pretty version of what the drone actually saw.

### Why we use the Manifold version (and downsample anyway)

For facade extraction, **denser is better** — more points means we resolve smaller features (window sills, parapets, balcony details). On Mijande, the existing `facades_from_pointcloud_cgal` pipeline runs unchanged on the merged Manifold cloud (with a 10 cm voxel downsample) and produces **4,700 facades vs 1,651** from the same KMZ's curated cloud. Same building, 2.8× more structural detail.

**Subtle but important:** the voxel downsample is **only for the CGAL algorithm's runtime**, not for storage or transport. Region-growing on 18.8 M points takes minutes; on 1.7 M points (10 cm voxel) it takes 14 s. The downsample also kills sensor noise (averaging within each 10 cm cube) while preserving structural geometry. Nothing big leaves the Manifold; the augmented KMZ we ship back is the same ~15 MB as the input KMZ.

### Production data flow

```
Manifold raw PLYs (~268 MB – 1.4 GB on Manifold, depending on flight)
   │
   │ rsync over wired link (USB-tether to M4E debug port, or LAN)
   ▼  ~5–30 sec
Laptop merges in memory + voxel-downsamples to 10 cm  (~1.7 M pts in RAM)
   │
   ▼  ~14 sec
CGAL facade extraction → ~4700 facades
   │
   ▼  ICP-register to KMZ frame using KMZ's curated cloud as target
Facades in KMZ-aligned ENU frame
   │
   ▼  per-waypoint: find dominant in-view facade, override gimbal pitch/yaw
Gimbal-augmentation pass (~few sec, 1233 waypoints)
   │
   ▼  re-zip with original waylines.wpml + template.kml + tiles unchanged
Modified KMZ (~15 MB, similar size to original Smart3D KMZ)
   │
   │ scp → Manifold's /open_app/dev/data/received/
   ▼
DjiWaypointV3_UploadKmzFile + DjiWaypointV3_Action(START)
```

`scripts/build_manifold_overlay_kmz.py` builds a synthetic KMZ for visual verification: it replaces `cloud.ply` inside a source KMZ with a Manifold-sourced (and ICP-registered) cloud, leaves the waypoints / polygon / gimbal commands byte-identical, and lets you import both KMZs through `/import-kmz` to toggle between them in Viewer3D.

### What's NOT on the Manifold

Searched comprehensively for the flight plan (waypoints, gimbal commands,
capture actions). **It is not stored unencrypted anywhere on the Manifold:**
- `expl_plan.bin.enc` is encrypted, opaque.
- `Smart3DExplore_dynamic_ch_v2.json` is IPC channel/storage config, not a
  mission.
- `vp_storage.json` is volume layout, not waypoints.
- No `*.kmz` or `*.wpml` exists anywhere on the filesystem outside the PSDK
  sample-code tree.

So the flight plan still has to come from the KMZ on the RC, which the pilot
exports via USB-MTP cable to the laptop in the existing manual workflow. That
manual export is the only viable route for the waypoint stream.

### Implications for the architecture

1. **Mesh transport is solved without code.** AeroScan reads the perception
   PLYs from the Manifold over the wired link the dev workflow already uses
   (`rsync dji@<manifold>:/blackbox/the_latest_flight/dji_perception/1/`).
2. **Flight plan transport stays manual.** Smart3D KMZ → laptop via the pilot's
   existing USB-MTP cable. This was already routine before AeroScan; we don't
   change it.
3. **AeroScan's job becomes a transformation, not a generation.** Combine the
   KMZ's flight plan with the Manifold's mesh, override only gimbal pitch/yaw
   per waypoint, and emit a modified KMZ. The drone re-flies a path it already
   knows.
4. **The MOP wireless path keeps mattering, but only for control messages.**
   At ≤500 KB it's still useful for "fly mission X" commands and status
   pulls. Sub-2-minute payloads, no infrastructure required. This bring-up
   work is preserved in that role.
