#!/usr/bin/env bash
# Sync the Python AeroScan engine from this dev repo to the Manifold.
#
#   dev (laptop)  =  /Users/ariana/git/aero-scan
#   run (manifold) =  dji@192.168.1.55:/open_app/dev/aero-scan
#
# After the first deploy, on the Manifold install editable into the conda env:
#
#     mamba run -n aero-scan pip install -e /open_app/dev/aero-scan
#
# Subsequent deploys: just rerun this script. Editable install picks up changes
# instantly (no reinstall needed) since pip records the path, not the contents.
#
# Flags:
#   -n        dry-run (rsync --dry-run, show what would change)
#   --host=H  override target host (default 192.168.1.55)

set -euo pipefail

DRY=""
HOST="192.168.1.55"
for arg in "$@"; do
    case "$arg" in
        -n|--dry-run) DRY="--dry-run" ;;
        --host=*) HOST="${arg#--host=}" ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/"
DEST="dji@${HOST}:/open_app/dev/aero-scan/"

echo "src:  $SRC"
echo "dest: $DEST"
echo

rsync -avz --delete ${DRY} \
    --exclude '.git/' \
    --exclude '.venv/' \
    --exclude '.venv-*/' \
    --exclude 'frontend/' \
    --exclude 'rc-companion/' \
    --exclude 'node_modules/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude '*.pyo' \
    --exclude '.pytest_cache/' \
    --exclude '.ruff_cache/' \
    --exclude '.mypy_cache/' \
    --exclude 'output/' \
    --exclude 'sim_output/' \
    --exclude 'kmz/' \
    --exclude '*.db' \
    --exclude 'dist/' \
    --exclude 'build/' \
    --exclude '*.egg-info/' \
    --exclude '.DS_Store' \
    --exclude 'Payload-SDK-Tutorial/' \
    --exclude '.claude/' \
    "$SRC" "$DEST"
