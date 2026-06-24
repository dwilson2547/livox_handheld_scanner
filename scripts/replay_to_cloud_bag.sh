#!/usr/bin/env bash
#
# replay_to_cloud_bag.sh <session_dir> [cloud_bag_out]
#
# Replay a raw session bag through Point-LIO and record the DESKEWED, world-frame
# /cloud_registered together with /aft_mapped_to_init odometry into a new bag. This
# is the geometry input for build_voxel_map.py — the project doctrine requires the
# deskewed cloud, not raw /livox/lidar single-pose accumulation (which blobs).
#
# Usage:
#   source ~/ros2_ws/install/setup.bash
#   scripts/replay_to_cloud_bag.sh sessions/viewer_fix_20260621_075144
#
set -euo pipefail

SESSION="${1:?usage: replay_to_cloud_bag.sh <session_dir> [cloud_bag_out]}"
SESSION="$(realpath "$SESSION")"
OUT="${2:-$SESSION/cloud_bag}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRINGUP_SHARE="$(ros2 pkg prefix scanner_bringup)/share/scanner_bringup"
PL_CONFIG="$BRINGUP_SHARE/config/point_lio_horizon_dense.yaml"

if [ -e "$OUT" ]; then
  echo "Output $OUT already exists — remove it or pass a different path." >&2
  exit 1
fi

echo "Replaying $SESSION through Point-LIO → recording /cloud_registered + /aft_mapped_to_init"
echo "  cloud bag: $OUT"

LAUNCH_LOG="$(mktemp)"

# Record the deskewed cloud + odom. Records start immediately; the launch then
# plays the bag through Point-LIO which publishes both topics.
ros2 bag record -o "$OUT" /cloud_registered /aft_mapped_to_init &
REC_PID=$!
sleep 2

# NOTE: `ros2 launch ... use_bag:=true` both PLAYS the bag and keeps the Point-LIO
# / meshing nodes alive indefinitely after playback ends — it never self-exits.
# So we run it in the background and watch its log for the bag player finishing,
# then tear the whole group down. (Waiting on the launch directly hangs forever.)
setsid ros2 launch scanner_bringup scanner.launch.py \
  use_bag:=true "bag_path:=$SESSION" \
  record:=false foxglove:=false rviz:=false \
  "point_lio_config:=$PL_CONFIG" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

echo "  waiting for bag playback to finish (Point-LIO replay)…"
# The player is the launch's tracked process #1 (ros2-1); its exit prints this line.
until grep -q "process has finished cleanly" "$LAUNCH_LOG" 2>/dev/null; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  sleep 2
done

# Let Point-LIO emit the final sweeps and the recorder flush, then stop everything.
sleep 3
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true
kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -9 -- "-$LAUNCH_PID" 2>/dev/null || true
rm -f "$LAUNCH_LOG"

echo "Done → $OUT"
echo "Next: python3 $SCRIPT_DIR/build_voxel_map.py $SESSION"
