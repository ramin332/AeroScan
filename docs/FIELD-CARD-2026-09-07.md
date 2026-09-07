# FIELD CARD — 2026-09-07, first real-house flight

> Everything here works with **no internet**. LAN/SSH to the Manifold, MOP to the RC,
> and the laptop webapp are all local. The only thing that needs internet is the
> satellite basemap in the frontend's Map tab — use the 3D tab instead.
> Companion docs (also offline): `docs/TEST-FLIGHT-RUNBOOK.md`, `docs/RESUME-HERE.md`.

## 0. Find the Manifold (DHCP — it moves)

```bash
arp -a | grep -iE 'tegra|dji'          # or try both known IPs
export AEROSCAN_MANIFOLD_HOST=192.168.1.118   # seen at .55 and .118
ssh dji@$AEROSCAN_MANIFOLD_HOST 'hostname; date'
```

## 1. Preflight (aircraft powered, on the LAN)

```bash
bash scripts/preflight_check_manifold.sh --host=$AEROSCAN_MANIFOLD_HOST
```
Prints: mesh on latest flight, every mesh in `/blackbox` with chunk mtimes, disk,
PSDK process, DPK install state, ssh-perm cron, git HEAD, engine deps.

RC app version:
```bash
adb devices
adb shell dumpsys package com.aeroscan.rccompanion | grep lastUpdateTime   # want 2026-09-03
adb install -r output/rc-companion/rc-companion-debug-2026-09-03-3d.apk    # if older
```

## 2. During the day — offline status pulls

```bash
# Which slot actually holds today's data? Slot DIR mtimes LIE. Per-file mtime is truth.
ssh dji@$AEROSCAN_MANIFOLD_HOST 'for d in /blackbox/flight[0-9]*; do
  n=$(find "$d" -type f -newermt "$(date +%Y-%m-%d) 00:00" 2>/dev/null | wc -l)
  [ "$n" -gt 0 ] && echo "$(basename $d) files_today=$n meshes=$(find "$d/dji_perception" -name "mesh_binary_*.ply" 2>/dev/null | wc -l)"
done'

# Mesh present on the latest flight? (subdir is NOT always 1)
ssh dji@$AEROSCAN_MANIFOLD_HOST 'find /blackbox/the_latest_flight/dji_perception -name "mesh_binary_*.ply" | wc -l'

# Mission state the aircraft is holding
ssh dji@$AEROSCAN_MANIFOLD_HOST 'cat /open_app/dev/data/received/mission_progress.json'

# PSDK / FC log — the FC's REAL error code lives here, not in Action(START)'s return
ssh dji@$AEROSCAN_MANIFOLD_HOST "journalctl --since today | sed -E 's/\x1b\[[0-9;]*m//g'" | tail -80
ssh dji@$AEROSCAN_MANIFOLD_HOST "journalctl --since today | grep -iE 'error_code|waypoint_v3|fly tap|ready_to_fly'"

# Is the payload app up?
ssh dji@$AEROSCAN_MANIFOLD_HOST 'systemctl is-active psdk-demo.service; dji_app_ctl list'
```

## 3. After the flight — offline analysis (this is the point of the day)

```bash
# Photos are on the SD CARD, not /blackbox. Camera clock runs 1 h behind the PSDK log.
mkdir -p flight-archive/2026-09-07/photos
cp -r /Volumes/<SDCARD>/DCIM/DJI_2026*/. flight-archive/2026-09-07/photos/

# THE measurement: actual gimbal angles from JPEG XMP vs the flown KMZ
.venv/bin/python scripts/read_gimbal_xmp.py flight-archive/2026-09-07/photos \
    --kmz <the-flown>.augmented.lean.kmz

# The mission itself, offline, no cloud needed
.venv/bin/python scripts/verify_augmented_kmz.py <the-flown>.augmented.lean.kmz
.venv/bin/python scripts/render_aim_audit.py <the-flown>.augmented.lean.kmz

# Telemetry CSV written by the Manifold during the mission (first flight it exists)
scp dji@$AEROSCAN_MANIFOLD_HOST:'/open_app/dev/data/received/telemetry/*.csv' flight-archive/2026-09-07/
.venv/bin/python scripts/read_flight_telemetry.py flight-archive/2026-09-07/<file>.csv

# Whole /blackbox slot (mesh included) — do this AFTER power-down, a live slot is still being written
bash scripts/pull_flight_archive.sh --host=$AEROSCAN_MANIFOLD_HOST
```

## 4. Laptop fallback (no PSDK, no Manifold) — works fully offline

```bash
./run.sh          # backend :8111, frontend :3847 — deps already installed
```
USB-MTP the Smart3D KMZ off the RC → import in the frontend (it carries a point cloud)
→ generate mission → export KMZ → SD card → Pilot 2 mission library → fly normally.
Use the **3D tab**; the Map tab's satellite tiles need internet.

## 5. Never do these

- **Do not power-cycle between the Smart3D scan and the augment.** New empty `/blackbox`
  slot; the ring buffer churns the fresh mesh. App-switching in Pilot is safe.
- **Do not `pkill -f dji_sdk_demo_on_manifold3` over SSH** — it self-matches and kills your
  session. `pkill -x dji_sdk_demo_on` or kill by PID.
- **Do not arm motors before START** — WaypointV3 refuses with `error_code: 778`.
- **Do not reuse the SD card before the photos are copied.** The 2026-07-10 photos were
  never archived and that made the gimbal bug unfalsifiable.
