#!/usr/bin/env python3
"""实验一：感知单点标定（2026-08-08 用户要求）。

静止物体连续 N 帧采样候选中心位置（camera → base），统计：
- 均值 / std / min-max 范围 → 感知系统性偏差（均值 vs 实际，需尺量对比）
- 波动幅度 → 感知噪声
关注 z 方向（抓取高度偏差的感知来源量化）。
用法：python3 scripts/perception_stability_test.py [--frames 30] [--interval 2]
"""
import argparse
import csv
import sys
import time

import numpy as np

sys.path.insert(0, 'src/real_piper_perception')
from real_piper_perception.grasp_transform import load_handeye, camera_pose_to_base  # noqa: E402

CSV_PATH = 'shared_camera_data/perception_grasps.csv'
HANDEYE = 'camera_data/handeye_eye_to_hand.yaml'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--interval', type=float, default=2.0)
    args = ap.parse_args()

    handeye = load_handeye(HANDEYE)
    samples = []
    last_pos = None
    for i in range(args.frames):
        with open(CSV_PATH) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            print(f'[{i+1}] 无候选'); time.sleep(args.interval); continue
        # 取最高 score 候选；若与上一帧位置差 > 0.15m 则视为换了目标，跳过
        rows.sort(key=lambda r: float(r['score']), reverse=True)
        for r in rows:
            xyz = (float(r['camera_x']), float(r['camera_y']), float(r['camera_z']))
            Rg = np.array([[float(r[f'r{i}{j}']) for j in range(3)] for i in range(3)])
            pos, _ = camera_pose_to_base(xyz, Rg, handeye)
            if last_pos is not None and np.linalg.norm(pos - last_pos) > 0.15:
                continue  # 跳帧目标（可能是误检/其他物体）
            samples.append(pos)
            last_pos = pos
            print(f'[{i+1}] score={float(r["score"]):.3f} '
                  f'base=({pos[0]*1000:6.1f},{pos[1]*1000:6.1f},{pos[2]*1000:6.1f})mm')
            break
        time.sleep(args.interval)

    if len(samples) < 5:
        print(f'样本不足（{len(samples)}）'); return 1
    arr = np.array(samples)
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)
    rng = arr.max(axis=0) - arr.min(axis=0)
    print('\n=== 统计（30 帧静止物体） ===')
    print(f'均值:  ({mean[0]*1000:6.1f}, {mean[1]*1000:6.1f}, {mean[2]*1000:6.1f}) mm')
    print(f'std:   ({std[0]*1000:6.1f}, {std[1]*1000:6.1f}, {std[2]*1000:6.1f}) mm')
    print(f'范围:  x {rng[0]*1000:.1f}mm  y {rng[1]*1000:.1f}mm  z {rng[2]*1000:.1f}mm')
    print(f'样本数: {len(samples)}')
    print('\n注意：均值 vs 物体实际位置（尺量）的差 = 感知系统性偏差；'
          '范围 = 感知噪声（z 方向是抓取高度偏差的关键）。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
