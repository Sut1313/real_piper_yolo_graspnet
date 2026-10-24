#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${SCRIPT_DIR}"
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  colcon build --symlink-install \
  --packages-up-to realsense2_camera real_piper_perception real_piper_yolo_graspnet
