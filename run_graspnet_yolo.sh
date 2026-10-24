#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export SEGMENTATION_MODE=yolo
exec "${SCRIPT_DIR}/run_graspnet_d435i.sh" "$@"
