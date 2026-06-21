#!/usr/bin/env bash
#
# discover_horizon_broadcast_code.sh
# Runs the SDK1 Livox ROS 2 driver in auto-discovery mode so the Horizon's
# broadcast code can be read from the driver's stdout without editing the
# checked-in config by hand.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${REPO_ROOT}/src/scanner_bringup/config/horizon.json"
TMP_CONFIG="$(mktemp /tmp/livox-horizon-discovery.XXXXXX.json)"

cleanup() {
  rm -f "${TMP_CONFIG}"
}
trap cleanup EXIT

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
  exit 1
fi

if [ ! -f "${CONFIG_PATH}" ]; then
  echo "ERROR: Horizon config not found at ${CONFIG_PATH}"
  exit 1
fi

python3 - "${CONFIG_PATH}" "${TMP_CONFIG}" <<'PY'
import json
import sys

src_path, dst_path = sys.argv[1], sys.argv[2]

with open(src_path, "r", encoding="utf-8") as handle:
    cfg = json.load(handle)

for lidar in cfg.get("lidar_config", []):
    lidar["enable_connect"] = False

with open(dst_path, "w", encoding="utf-8") as handle:
    json.dump(cfg, handle, indent=2)
    handle.write("\n")
PY

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
  source "${HOME}/ros2_ws/install/setup.bash"
fi
set -u

cat <<EOF
Auto-discovery mode enabled.

What to watch for:
- "No broadcast code was added to whitelist, swith to automatic connection mode"
- device/broadcast-code lines emitted by livox_ros2_driver

When the Horizon code appears, copy the 15-character code into:
  ${CONFIG_PATH}

Press Ctrl+C after the code is captured.
EOF

exec ros2 run livox_ros2_driver livox_ros2_driver_node --ros-args \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:="${TMP_CONFIG}"
