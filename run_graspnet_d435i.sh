#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="${IMAGE_NAME:-graspnet-baseline:20.04}"
MODE="${1:-gui}"
SHARED_DIR="${SCRIPT_DIR}/shared_camera_data"
SHARED_MOUNT="/shared_camera_data"
CHECKPOINT_PATH="${CHECKPOINT_PATH:-logs/log_rs/checkpoint.tar}"
MIN_DEPTH_M="${MIN_DEPTH_M:-0.15}"
MAX_DEPTH_M="${MAX_DEPTH_M:-1.20}"
ROI="${ROI:-}"
TOP_K="${TOP_K:-50}"
SAVE_DEBUG_OUTPUTS="${SAVE_DEBUG_OUTPUTS:-0}"
SAVE_DEBUG_EVERY="${SAVE_DEBUG_EVERY:-10}"
MAX_SAVED_DEBUG_FRAMES="${MAX_SAVED_DEBUG_FRAMES:-5}"
DOCKER_TTY_ARGS=()
if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

mkdir -p "${SHARED_DIR}"
# 启动前清除残留的暂停标记（2026-08-08：上次会话强杀进程残留导致推理全程冻结）
rm -f "${SHARED_DIR}/pause_inference"

echo "============================================"
echo "Real Piper D435i GraspNet Inference"
echo "  image:       ${IMAGE_NAME}"
echo "  shared_dir:  ${SHARED_DIR} -> ${SHARED_MOUNT}"
echo "  checkpoint:  ${CHECKPOINT_PATH}"
echo "  mode:        ${MODE}"
echo "  depth range: ${MIN_DEPTH_M}..${MAX_DEPTH_M} m"
echo "  roi:         ${ROI:-full}"
echo "  debug save:  ${SAVE_DEBUG_OUTPUTS} every ${SAVE_DEBUG_EVERY}, keep ${MAX_SAVED_DEBUG_FRAMES}"
echo "============================================"

common_args=(
  python inference_from_camera.py
  --checkpoint_path "${CHECKPOINT_PATH}"
  --shared_dir "${SHARED_MOUNT}"
  --num_point "${NUM_POINT:-20000}"
  --num_view "${NUM_VIEW:-300}"
  --collision_thresh "${COLLISION_THRESH:-0.01}"
  --top_k "${TOP_K}"
  --segmentation_mode workspace
  --workspace_name target
  --min_depth_m "${MIN_DEPTH_M}"
  --max_depth_m "${MAX_DEPTH_M}"
  --save_debug_every "${SAVE_DEBUG_EVERY}"
  --max_saved_debug_frames "${MAX_SAVED_DEBUG_FRAMES}"
)

if [[ -n "${ROI}" ]]; then
  common_args+=(--roi "${ROI}")
fi
if [[ "${SAVE_DEBUG_OUTPUTS}" == "1" ]]; then
  common_args+=(--save_debug_outputs)
fi

case "${MODE}" in
  gui)
    if [[ -z "${DISPLAY:-}" ]]; then
      echo "ERROR: DISPLAY is not set. Use '$0 headless' or run from a graphical desktop session." >&2
      exit 1
    fi
    xhost +local:docker >/dev/null 2>&1 || true
    docker run --rm "${DOCKER_TTY_ARGS[@]}" \
      --gpus 'all,"capabilities=compute,utility,graphics"' \
      --ipc=host \
      --network=host \
      -e DISPLAY="${DISPLAY}" \
      -e EGL_PLATFORM=surfaceless \
      -e QT_X11_NO_MITSHM=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${SHARED_DIR}:${SHARED_MOUNT}:rw" \
      -v "${SCRIPT_DIR}/graspnet-baseline/inference_from_camera.py:/workspace/inference_from_camera.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/precheck_match.py:/workspace/precheck_match.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/text_overlay.py:/workspace/text_overlay.py:ro" \
      -v "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc:/workspace/fonts/NotoSerifCJK-Bold.ttc:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_utils.py:/workspace/pointnet2/pointnet2_utils.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_modules.py:/workspace/pointnet2/pointnet2_modules.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pytorch_utils.py:/workspace/pointnet2/pytorch_utils.py:ro" \
      "${IMAGE_NAME}" \
      "${common_args[@]}"
    ;;
  headless)
    docker run --rm \
      --gpus 'all,"capabilities=compute,utility,graphics"' \
      --ipc=host \
      --network=host \
      -e EGL_PLATFORM=surfaceless \
      -v "${SHARED_DIR}:${SHARED_MOUNT}:rw" \
      -v "${SCRIPT_DIR}/graspnet-baseline/inference_from_camera.py:/workspace/inference_from_camera.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/precheck_match.py:/workspace/precheck_match.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/text_overlay.py:/workspace/text_overlay.py:ro" \
      -v "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc:/workspace/fonts/NotoSerifCJK-Bold.ttc:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_utils.py:/workspace/pointnet2/pointnet2_utils.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_modules.py:/workspace/pointnet2/pointnet2_modules.py:ro" \
      -v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pytorch_utils.py:/workspace/pointnet2/pytorch_utils.py:ro" \
      "${IMAGE_NAME}" \
      "${common_args[@]}" \
      --no_display
    ;;
  *)
    echo "Usage: $0 [gui|headless]" >&2
    exit 2
    ;;
esac
