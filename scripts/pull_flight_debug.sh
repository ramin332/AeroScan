#!/usr/bin/env bash
# Read-only post-flight pull from the Manifold after a Fly-widget / START test.
# Safe: no installs, no kills, no writes to app dirs. Stages TODAY's artifacts
# into /tmp on the drone, tars them, and pulls one bundle to ./flight-debug/.
#
# Usage (laptop on same LAN/AP as the drone, no internet needed):
#   bash scripts/pull_flight_debug.sh
set -u
HOST="dji@192.168.1.118"
OUT="./flight-debug"
mkdir -p "$OUT"

echo "##############################################################"
echo "## 1) STAGE + BUNDLE on the drone (read-only, one ssh)       ##"
echo "##############################################################"
ssh -o ConnectTimeout=8 "$HOST" 'bash -s' <<"REMOTE"
set -u
TODAY=$(date +%Y-%m-%d)
STAGE=/tmp/flightdbg
rm -rf "$STAGE"; mkdir -p "$STAGE/logs" "$STAGE/missions" "$STAGE/received"

# --- access diagnostics: does 'dji' belong to the group that can read the DPK? ---
{
  echo "=== whoami / groups ==="; id
  echo; echo "=== /open_app ==="; ls -la /open_app/
  echo; echo "=== /open_app/psdk-demo (stderr NOT suppressed — perms wall shows here) ==="
  ls -la /open_app/psdk-demo/
  echo; echo "=== psdk-demo log dirs ==="
  ls -la /open_app/psdk-demo/*/data/logs/ 2>&1 | head -20
} > "$STAGE/access.txt" 2>&1
echo "--- access.txt ---"; cat "$STAGE/access.txt"

# --- TODAY's PSDK logs, anywhere readable (find skips dirs it can't enter) ---
echo; echo "=== today's readable DJI*.log ==="
find /open_app -name 'DJI*.log' -newermt "$TODAY 00:00" -readable 2>/dev/null | tee "$STAGE/today_logs.txt"
while read -r f; do [ -n "$f" ] && cp -p "$f" "$STAGE/logs/" 2>/dev/null; done < "$STAGE/today_logs.txt"

# --- today's mission intent (what was augmented) + augmented KMZ(s) ---
echo; echo "=== newest mission intents + augmented KMZs ==="
ls -dt /open_app/dev/data/missions/* 2>/dev/null | head -3 | tee "$STAGE/intents.txt" \
  | while read -r d; do cp -rp "$d" "$STAGE/missions/" 2>/dev/null; done
ls -t /open_app/dev/data/received/*.augmented.kmz 2>/dev/null | head -2 | tee "$STAGE/kmzs.txt" \
  | while read -r k; do cp -p "$k" "$STAGE/received/" 2>/dev/null; done

# --- bundle ---
tar -czf /tmp/flightdbg.tgz -C /tmp flightdbg 2>/dev/null
echo; echo "=== bundle ==="; ls -la /tmp/flightdbg.tgz; du -sh "$STAGE"
REMOTE

echo
echo "##############################################################"
echo "## 2) PULL the bundle (one scp) + unpack                     ##"
echo "##############################################################"
scp -o ConnectTimeout=8 "$HOST:/tmp/flightdbg.tgz" "$OUT/" 2>/dev/null && echo "pulled."
tar -xzf "$OUT/flightdbg.tgz" -C "$OUT/" 2>/dev/null && echo "unpacked -> $OUT/flightdbg/"

echo
echo "##############################################################"
echo "## 3) DECODE locally — the START story                       ##"
echo "##############################################################"
B="$OUT/flightdbg"
echo "=== access (did we get the DPK log, or hit a perms wall?) ==="
sed -n '1,40p' "$B/access.txt" 2>/dev/null
echo
echo "=== logs pulled ==="; ls -la "$B/logs/" 2>/dev/null
echo
echo "=== FC WaypointV3 state stream + our START handler (strip ANSI) ==="
for L in "$B"/logs/*.log; do
  [ -e "$L" ] || continue
  echo "----- $L -----"
  sed -E 's/\x1b\[[0-9;]*m//g' "$L" \
    | grep -nE "Fly tapped|fly tap ignored|DjiWaypointV3_Action|Receive waypoint push state|mission state|Action.*START|valid|reject|refuse|errorCode|0x[0-9a-fA-F]{6,}" \
    | tail -80
done
echo
echo "Done. If access.txt shows 'Permission denied' on psdk-demo, the DPK log"
echo "is unreadable by 'dji' — tell me and we pivot (DPK log->shared dir on next"
echo "deploy, or RC export)."
