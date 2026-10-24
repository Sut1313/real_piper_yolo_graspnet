#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi
set -u

ros2 launch real_piper_yolo_graspnet d435i_yolo_graspnet_pipeline.launch.py \
  shared_dir:="${SCRIPT_DIR}/shared_camera_data" \
  model_path:="${TABLETOP_YOLO_MODEL:-${SCRIPT_DIR}/../tabletop_yolov8seg/models/tabletop_yolov8s_seg_v2.onnx}" \
  target_class:="${TARGET_CLASS:-cup}"
