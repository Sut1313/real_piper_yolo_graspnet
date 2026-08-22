#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi
set -u

ros2 launch real_piper_perception d435i_graspnet_pipeline.launch.py \
  shared_dir:="${SCRIPT_DIR}/shared_camera_data" \
  publish_rate_hz:="${PUBLISH_RATE_HZ:-5.0}"
