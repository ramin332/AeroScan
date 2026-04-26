# AeroScan RC Companion

Android (Kotlin, MSDK V5) app that runs on the **DJI RC Plus 2** controller paired with the **Matrice 4E**. Pilot picks a `.kmz` from RC storage and streams it over OcuSync → aircraft → E-Port USB → Manifold 3 via DJI's MOP (SDK Interconnection) channel, where the sibling PSDK app picks it up and flies it via Waypoint V3.

This app is a **file-pusher**, full stop. No FPV, no telemetry, no map, no replanner. The full design rationale lives in the build brief; the planning notes live at `~/.claude/plans/within-our-current-repo-magical-metcalfe.md`.

## Status

Scaffold only — code paths compile-clean against MSDK V5 (verify Gradle coords against [DJI's tutorial](https://developer.dji.com/doc/mobile-sdk-tutorial/en/) before first sync). **Not yet validated on real hardware.** Phase 0 verification (sideloadability of the RC Plus 2, MOP `connect()` against M4E firmware, Pilot 2 coexistence) must pass before relying on any of this in the field.

## Prerequisites

- Android Studio (Hedgehog or later)
- JDK 17
- An MSDK V5 app registered at https://developer.dji.com/ — **SDK Type must be `Mobile SDK`** (not Payload SDK), package `com.aeroscan.rccompanion`. The PSDK Manifold app (`App ID 183281`) is a separate registration; do **not** reuse its key here.

## Setup

1. Copy `local.properties.example` to `local.properties` and fill in `DJI_APP_KEY=...`. This file is gitignored — never commit it.
2. Open the `rc-companion/` folder in Android Studio. It will auto-download the Gradle wrapper on first sync.
3. Build & run on a connected Android device or RC Plus 2:
   ```
   ./gradlew :app:assembleDebug
   adb install app/build/outputs/apk/debug/app-debug.apk
   ```

## MOP wire protocol

The MSDK V5 transport (Pipeline.connect / writeData / disconnect) is verified against the official sample at `dji-sdk/Mobile-SDK-Android-V5/SampleCode-V5/android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/{models/MopVM.kt,data/MOPCmdHelper.java}`. The companion API doc is vendored at `../Payload-SDK-Tutorial/docs/en/50.function-overview/00.basic-function/160.sdk-interconnection.md`.

The application-layer framing in `app/src/main/kotlin/com/aeroscan/rccompanion/mop/Framing.kt` is **deliberately simpler than DJI's sample protocol** in `MOPCmdHelper.java` (which has 8-byte command headers, sequence numbers, GBK-encoded 32-byte filenames, ack/nack). For a one-shot KMZ push we only need: header → bytes → MD5. Bytes-on-the-wire compatibility with the Manifold-side PSDK server is mandatory; whichever side adopts a richer protocol must update both. Channel ID is `49152` (see `mop/Constants.kt`).

Component index defaults to `ComponentIndexType.LEFT_OR_MAIN` since the M4E exposes a single E-Port. Verify against the MSDK sample app's connection dropdown if Phase 0 finds the connect call returns "device not found".

## Pilot two-step flow

1. Open AeroScan RC → pick KMZ → Send to Manifold → wait for "Done"
2. Switch to DJI Pilot 2 → tap the PSDK widget on live-flight view → pilot taps confirm to fly

The PSDK side is the chooser-of-mission *and* the flight controller. This app just delivers the bytes.

## What's not in scope here

- Waypoint upload to the aircraft (`DjiWaypointV3_UploadKmzFile`) — PSDK side, sibling repo.
- KMZ parsing or validation — PSDK side validates; we treat the file as opaque bytes + filename + size.
- Any UI that overlaps with Pilot 2 (FPV, telemetry, map). Pilot 2 stays the live-flight surface.
