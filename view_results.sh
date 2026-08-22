#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SHARED_DIR="${SCRIPT_DIR}/shared_camera_data" \
  python3 "${SCRIPT_DIR}/scripts/result_viewer.py"
