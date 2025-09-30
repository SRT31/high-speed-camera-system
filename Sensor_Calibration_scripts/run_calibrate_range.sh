#!/usr/bin/env bash
# run_calibrate_range.sh
# Loop over a range of raspiraw frames and run sensor_calibrate_imx219.py on each.
# Usage:
#   ./run_calibrate_range.sh 40 50
# Customization via environment variables (defaults shown):
#   DIR=/dev/shm BASE=out WIDTH=640 HEIGHT=64 BITDEPTH=10 PACKING=raw10 \
#   BAYER=BGGR MODE=auto WB_MODE=auto PY=python3 CAL_SCRIPT=sensor_calibrate_imx219_.py \
#   ./run_calibrate_range.sh 40 50
#
# Examples:
#   # Just run 40..50 with defaults:
#   ./run_calibrate_range.sh 40 50
#   # Different ROI size:
#   WIDTH=320 HEIGHT=64 ./run_calibrate_range.sh 40 50
#   # Force WB ROI later:
#   WB_MODE=roi ./run_calibrate_range.sh 40 50
#   # Dark frame pass (to get -k/-S):
#   MODE=dark ./run_calibrate_range.sh 1 3

set -euo pipefail

START="${1:-40}"
END="${2:-50}"

DIR="${DIR:-/dev/shm}"
BASE="${BASE:-out}"
OUTLOC="${OUTLOC:-/home/project/tuning}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-64}"
BITDEPTH="${BITDEPTH:-10}"
PACKING="${PACKING:-raw10}"
BAYER="${BAYER:-BGGR}"
MODE="${MODE:-auto}"
WB_MODE="${WB_MODE:-auto}"
PY="${PY:-python3}"
CAL_SCRIPT="${CAL_SCRIPT:-sensor_calibrate_imx219_.py}"

echo "[cfg] DIR=$DIR BASE=$BASE WIDTH=$WIDTH HEIGHT=$HEIGHT BITDEPTH=$BITDEPTH PACKING=$PACKING"
echo "[cfg] BAYER=$BAYER MODE=$MODE WB_MODE=$WB_MODE PY=$PY CAL_SCRIPT=$CAL_SCRIPT"
echo "[cfg] Range: $START .. $END"

for i in $(seq "$START" "$END"); do
  num=$(printf "%06d" "$i")
  raw="$DIR/$BASE.$num.raw"
  if [[ ! -f "$raw" ]]; then
    echo "[skip] missing $raw"
    continue
  fi
  report="$OUTLOC/$BASE.$num.json"
  echo "== $raw -> $report =="
  "$PY" "$CAL_SCRIPT" \
    --input "$raw" \
    --width "$WIDTH" --height "$HEIGHT" \
    --bitdepth "$BITDEPTH" --packing "$PACKING" \
    --bayer "$BAYER" --mode "$MODE" --wb-mode "$WB_MODE" \
    --report "$report"
done

echo "[done] processed range $START..$END"
