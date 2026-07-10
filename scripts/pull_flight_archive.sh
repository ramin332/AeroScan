#!/usr/bin/env bash
# Post-flight: archive a full /blackbox flight slot off the Manifold.
#
# Read-only on the drone: no installs, no deletes, no writes outside /tmp.
# Works with no internet — laptop only needs to be on the same LAN/AP as the
# aircraft (the drone's wifi router is fine).
#
# /blackbox is a ~30-slot ring buffer that CYCLES. Slot numbers are NOT
# chronological, and the perception mesh (dji_perception/1/mesh_binary_*.ply)
# is evicted as the buffer churns. Archive BEFORE power-cycling.
#
# Usage:
#   bash scripts/pull_flight_archive.sh --list                 # show slots, newest first
#   bash scripts/pull_flight_archive.sh                        # pull the newest slot
#   bash scripts/pull_flight_archive.sh --flight=flight0066    # pull a named slot
#   bash scripts/pull_flight_archive.sh --flight=flight0065 --flight=flight0066
#   bash scripts/pull_flight_archive.sh --host=192.168.1.118   # AP handed a different IP
#
# Output: ./flight-archive/<YYYY-MM-DD>/<flightNNNN>/
#         ./flight-archive/<YYYY-MM-DD>/app-state/    (mission intents + augmented KMZs)
#         ./flight-archive/<YYYY-MM-DD>/slots.txt     (the --list output, for provenance)
#
# Re-runnable: rsync resumes partial transfers, never deletes on either side.

set -uo pipefail

HOST="192.168.1.118"
LIST_ONLY=0
FLIGHTS=()
OUT_ROOT="./flight-archive"

for arg in "$@"; do
    case "$arg" in
        --host=*)   HOST="${arg#--host=}" ;;
        --flight=*) FLIGHTS+=("${arg#--flight=}") ;;
        --out=*)    OUT_ROOT="${arg#--out=}" ;;
        --list|-l)  LIST_ONLY=1 ;;
        -h|--help)  sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

SSH="ssh -o ConnectTimeout=8"
TARGET="dji@${HOST}"

echo "### drone: $TARGET"

# ---------------------------------------------------------------------------
# 1) Inventory. Slot numbers lie; mtime doesn't. Mesh presence decides "keep".
# ---------------------------------------------------------------------------
SLOTS=$($SSH "$TARGET" 'bash -s' <<"REMOTE"
set -u
resolved=$(readlink -f /blackbox/the_latest_flight 2>/dev/null | xargs -r basename)
printf '%-14s %-20s %8s %6s %s\n' SLOT MTIME SIZE MESHES NOTE
for d in $(ls -dt /blackbox/flight[0-9]* 2>/dev/null); do
    slot=$(basename "$d")
    mt=$(date -r "$d" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo '?')
    sz=$(du -sh "$d" 2>/dev/null | cut -f1)
    meshes=$(ls "$d"/dji_perception/1/mesh_binary_*.ply 2>/dev/null | wc -l | tr -d ' ')
    note=""
    [ "$slot" = "$resolved" ] && note="<- the_latest_flight"
    [ "$meshes" -gt 0 ] 2>/dev/null && note="$note [MESH]"
    printf '%-14s %-20s %8s %6s %s\n' "$slot" "$mt" "$sz" "$meshes" "$note"
done
REMOTE
)
rc=$?
if [ $rc -ne 0 ] || [ -z "$SLOTS" ]; then
    echo "FAILED to reach $TARGET or list /blackbox (exit $rc)." >&2
    echo "Check the IP: the AP may have handed the drone a different one." >&2
    echo "Try: --host=192.168.1.118   (or arp -a | grep -i dji)" >&2
    exit 1
fi
echo "$SLOTS"

if [ "$LIST_ONLY" -eq 1 ]; then
    echo
    echo "Nothing pulled (--list). Rerun with --flight=<slot> to archive."
    exit 0
fi

# Default: newest slot by mtime (NOT by slot number).
if [ ${#FLIGHTS[@]} -eq 0 ]; then
    newest=$(printf '%s\n' "$SLOTS" | awk 'NR==2 {print $1}')
    if [ -z "$newest" ]; then echo "no flight slots found." >&2; exit 1; fi
    FLIGHTS=("$newest")
    echo
    echo "No --flight given; defaulting to newest by mtime: $newest"
    echo "If a Smart3D scan built the mesh in an EARLIER slot, pull that too."
fi

DAY=$(date +%Y-%m-%d)
OUT="${OUT_ROOT}/${DAY}"
mkdir -p "$OUT"
printf '%s\n' "$SLOTS" > "$OUT/slots.txt"

# ---------------------------------------------------------------------------
# 2) Pull each slot whole. No --delete: never destructive, always resumable.
# ---------------------------------------------------------------------------
for f in "${FLIGHTS[@]}"; do
    echo
    echo "### pulling /blackbox/$f -> $OUT/$f/"
    # --progress, not --info=progress2: macOS ships openrsync, which lacks it.
    rsync -az --partial --progress \
        -e "$SSH" \
        "$TARGET:/blackbox/$f/" "$OUT/$f/" || {
            echo "rsync failed for $f (permissions? disk full?)" >&2; exit 1; }
done

# ---------------------------------------------------------------------------
# 3) App state: what we told the drone to fly, and what it staged.
#    Separate from /blackbox — this is our own DPK's data dir.
# ---------------------------------------------------------------------------
echo
echo "### pulling app state -> $OUT/app-state/"
mkdir -p "$OUT/app-state"
rsync -az --partial -e "$SSH" \
    "$TARGET:/open_app/dev/data/missions/" "$OUT/app-state/missions/" 2>/dev/null \
    || echo "  (no missions/ — ok if augment never ran)"
rsync -az --partial -e "$SSH" \
    --include='*.kmz' --include='*/' --exclude='*' \
    "$TARGET:/open_app/dev/data/received/" "$OUT/app-state/received/" 2>/dev/null \
    || echo "  (no received/ KMZs)"

# ---------------------------------------------------------------------------
# 4) Receipt. What actually landed, so a later reader can trust the archive.
# ---------------------------------------------------------------------------
echo
echo "##############################################################"
echo "## archived -> $OUT"
echo "##############################################################"
du -sh "$OUT"/* 2>/dev/null
echo
echo "=== PSDK logs ==="
find "$OUT" -path '*/psdk/*' -name '*.log' 2>/dev/null | sed "s|^|  |"
echo "=== perception meshes ==="
n=$(find "$OUT" -name 'mesh_binary_*.ply' 2>/dev/null | wc -l | tr -d ' ')
echo "  $n mesh chunk(s)"
[ "$n" -eq 0 ] && echo "  WARNING: no mesh. Either no Smart3D scan ran in this slot," \
                       "or the ring buffer already evicted it."
echo "=== augmented KMZs ==="
find "$OUT/app-state" -name '*.kmz' 2>/dev/null | sed "s|^|  |"
echo
echo "This laptop is now the ONLY copy — /blackbox will churn these slots."
echo "Back it up off-machine before the next power-cycle."
