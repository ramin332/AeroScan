#!/usr/bin/env bash
# Read-only pre-flight verification of the Manifold. Safe: no installs, no kills, no writes.
# Run from the laptop on the same LAN as the drone:  bash scripts/preflight_check_manifold.sh
set -u
HOST="dji@192.168.1.55"

ssh -o ConnectTimeout=8 "$HOST" 'bash -s' <<"REMOTE"
echo "===== HOST / TIME / UPTIME ====="
hostname; date; uptime

echo; echo "===== MESH on latest flight (THE blocker) ====="
LF=$(readlink -f /blackbox/the_latest_flight 2>/dev/null); echo "the_latest_flight -> ${LF:-?}"
ls -la /blackbox/the_latest_flight/dji_perception/1/mesh_binary_*.ply 2>/dev/null || echo "NO MESH on latest flight"

echo; echo "===== ANY mesh anywhere in /blackbox ====="
ls /blackbox/flight*/dji_perception/1/mesh_binary_*.ply 2>/dev/null | head || echo "NONE anywhere"

echo; echo "===== /blackbox slots (newest mtime first) ====="
ls -dt /blackbox/flight* 2>/dev/null | head -6

echo; echo "===== DISK (need headroom for ~1GB scan) ====="
df -h /blackbox /open_app 2>/dev/null

echo; echo "===== AeroScan / PSDK process running? ====="
ps -eo pid,user,comm,etime 2>/dev/null | grep -iE "dji_sdk_demo_on|psdk|kmz_runner|aeroscan" | grep -v grep || echo "no matching process visible to dji"

echo; echo "===== DPK install state ====="
( dji_app_ctl list 2>&1 || echo "dji_app_ctl list unavailable" ) | head -20

echo; echo "===== built artifacts present ====="
ls -la /open_app/dev/Payload-SDK-3.16.0/build/dji_sdk_demo_on_manifold3 2>/dev/null
ls -la /open_app/dev/Payload-SDK-3.16.0/build/dpk/*.dpk 2>/dev/null || echo "no .dpk built"

echo; echo "===== git HEAD on Manifold (/open_app/dev) ====="
( cd /open_app/dev 2>/dev/null && git log --oneline -3 && git status --short | head ) || echo "no /open_app/dev git"

echo; echo "===== dev log tail (if dev binary was last run) ====="
tail -12 /open_app/dev/Payload-SDK-3.16.0/build/data/logs/latest.log 2>/dev/null || echo "no dev log"

echo; echo "===== AUGMENT engine: Python deps (open3d + CGAL) — augment HARD-FAILS without these ====="
ENGINE=/open_app/dev/aero-scan
# find the interpreter the augment would run under (engine venv if present, else system python3)
PY=$( ls "$ENGINE"/.venv*/bin/python 2>/dev/null | head -1 )
[ -z "$PY" ] && PY=$(command -v python3 2>/dev/null)
echo "interpreter: ${PY:-NONE FOUND}"
if [ -n "$PY" ]; then
  "$PY" - <<'PYEOF' 2>&1 || echo ">>> DEP CHECK FAILED — augment will die at import"
import importlib
for m in ("open3d","CGAL","numpy","trimesh"):
    try:
        importlib.import_module(m); print(f"  ok  {m}")
    except Exception as e:
        print(f"  FAIL {m}: {e.__class__.__name__}: {e}")
import importlib.util as u
print("  ok  flight_planner" if u.find_spec("flight_planner") else "  FAIL flight_planner: not importable (engine not pip-installed?)")
PYEOF
fi
echo "engine dir:"; ls -la "$ENGINE" 2>/dev/null | head -5 || echo "  NO engine dir at $ENGINE (deploy_to_manifold.sh not run?)"

echo; echo "===== C runner argv that spawns the augment (verify it points at the interpreter above) ====="
grep -rnoE "(augment-mission|flight_planner\.cli|python[0-9.]*)" /open_app/dev/Payload-SDK-3.16.0/ 2>/dev/null | grep -v Binary | head -12 || echo "  no augment spawn string found in PSDK tree"

echo; echo "===== readiness MOP channel listener (49154) ====="
( netstat -anp 2>/dev/null | grep -E "49154|4915" | head ) || echo "netstat unavailable"
echo "===== DONE ====="
REMOTE
