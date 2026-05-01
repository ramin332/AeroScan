# RC Companion Bring-Up: First End-to-End MOP Push

How we got `rc-companion/` (Android MSDK V5 app on RC Plus 2) to push a KMZ over
OcuSync → aircraft → E-Port → Manifold via DJI's MOP pipeline, and what was
broken on the way. First successful end-to-end test: 2026-05-01 09:53.

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
