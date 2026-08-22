#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_DIR="${OUTPUT_DIR:-${SCRIPT_DIR}/shared_camera_data/output}"
KEEP_FRAMES="${KEEP_FRAMES:-5}"

if [[ ! -d "${OUTPUT_DIR}" ]]; then
  exit 0
fi

python3 - "$OUTPUT_DIR" "$KEEP_FRAMES" <<'PY'
import os
import sys

output_dir = sys.argv[1]
keep_frames = max(int(sys.argv[2]), 0)
frame_ids = set()

for name in os.listdir(output_dir):
    stem, ext = os.path.splitext(name)
    if ext not in ('.npy', '.pcd'):
        continue
    try:
        frame_ids.add(int(stem.rsplit('_', 1)[1]))
    except (IndexError, ValueError):
        continue

stale_ids = sorted(frame_ids)[:-keep_frames]
removed = 0
for frame_id in stale_ids:
    for prefix, ext in (
        ('grasps', '.npy'),
        ('scene', '.pcd'),
        ('objects', '.pcd'),
    ):
        path = os.path.join(output_dir, f'{prefix}_{frame_id:04d}{ext}')
        try:
            os.remove(path)
            removed += 1
        except FileNotFoundError:
            pass

print(f'cleanup_output: removed {removed} files, kept {keep_frames} frame(s)')
PY
