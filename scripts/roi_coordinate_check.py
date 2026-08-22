#!/usr/bin/env python3
"""ROI 截取是否导致 GraspNet 坐标偏移——数值检验任务（2026-08-16）。

检验问题：推理用 ROI 掩码过滤点，抓取候选坐标（相机系）是否偏移？

三条路径对比（同一帧真实数据，复用推理同款反投影函数）：
  A. 全图路径：完整深度图 + 原始内参反投影（推理的基准坐标）
  B. ROI 路径：与 inference_from_camera.py:482-489 完全一致的掩码逻辑
     （roi_mask &= workspace_mask，图像不裁剪，内参不修改）
  C. 错误 crop 方案（对照）：裁剪图像 [y0:y1, x0:x1] + 内参不补偿
     （cx 不减 x0）——如果未来有人改成真裁剪，偏移量即此数值

输出：A vs B 一致性（应=0）；A vs C 偏移量（对照理论值 ΔX=(x0/fx)·z）。

用法：python3 scripts/roi_coordinate_check.py [--shared-dir ...]
      （推理运行中取当前帧；无 frame 时报错退出）
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'graspnet-baseline' / 'utils'))
from data_utils import CameraInfo, create_point_cloud_from_depth_image  # noqa: E402


# ── 核心逻辑（与推理管线逐行一致） ──────────────────────────────

def load_frame(shared_dir: str) -> dict:
    """读共享目录当前帧（color.png / depth.png / meta.mat），与推理 read_data 同构。"""
    shared = Path(shared_dir)
    color_p = shared / 'color.png'
    depth_p = shared / 'depth.png'
    meta_p = shared / 'meta.mat'
    for p in (color_p, depth_p, meta_p):
        if not p.exists():
            raise FileNotFoundError(f'缺少帧文件: {p}（先启动相机链路）')
    import cv2
    import scipy.io as scio
    color = cv2.imread(str(color_p))
    depth = cv2.imread(str(depth_p), cv2.IMREAD_UNCHANGED)
    meta = scio.loadmat(str(meta_p))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = float(meta['factor_depth'][0][0])
    # 与推理 read_data_from_shared 同构：scale = factor_depth（uint16 mm → m）
    camera = CameraInfo(
        int(color.shape[1]), int(color.shape[0]),
        float(intrinsic[0][0]), float(intrinsic[1][1]),
        float(intrinsic[0][2]), float(intrinsic[1][2]),
        factor_depth)
    return {'color': color, 'depth': depth, 'camera': camera,
            'factor_depth': factor_depth}


def apply_roi_mask(workspace_mask: np.ndarray, roi, h: int, w: int) -> np.ndarray:
    """与 inference_from_camera.py:485-489 逐行一致：ROI 像素掩码按位与。"""
    x_min, y_min, x_max, y_max = roi
    roi_mask = np.zeros_like(workspace_mask, dtype=bool)
    x0, x1 = int(x_min * w), int(x_max * w)
    y0, y1 = int(y_min * h), int(y_max * h)
    roi_mask[y0:y1, x0:x1] = True
    return workspace_mask & roi_mask


def crop_without_compensation(depth: np.ndarray, camera, roi, factor_depth: float):
    """错误对照方案：裁剪图像 + 内参不补偿（cx/cy 未减 x0/y0）。

    返回 (错误点云, (Δx_理论, Δy_理论, 参考深度 z))。理论偏移：
    ΔX = (x0/fx)·z，ΔY = (y0/fy)·z。
    """
    h, w = depth.shape
    x_min, y_min, x_max, y_max = roi
    x0, y0 = int(x_min * w), int(y_min * h)
    x1, y1 = int(x_max * w), int(y_max * h)
    crop = depth[y0:y1, x0:x1].copy()
    # 错误反投影：crop 后像素 (u',v') 未加回 (x0,y0)，内参 cx/cy 未补偿
    # （create_point_cloud 的尺寸断言会拦截，故此处手动实现同一公式）
    xmap, ymap = np.meshgrid(np.arange(crop.shape[1]), np.arange(crop.shape[0]))
    points_z = crop / factor_depth
    cloud_err = np.stack([
        (xmap - camera.cx) * points_z / camera.fx,
        (ymap - camera.cy) * points_z / camera.fy,
        points_z], axis=-1)
    z_ref = float(np.median(crop[crop > 0])) / factor_depth
    return cloud_err, (x0 / camera.fx * z_ref, y0 / camera.fy * z_ref, z_ref)


def run_check(shared_dir: str, roi, verbose: bool = True) -> dict:
    """三路径数值对比。返回报告 dict + 打印数值表。"""
    frame = load_frame(shared_dir)
    depth = frame['depth'].astype(np.float32) / frame['factor_depth']
    camera = frame['camera']

    # A. 全图点云（基准）
    cloud_full = create_point_cloud_from_depth_image(
        frame['depth'], camera, organized=True)
    mask = frame['depth'] > 0

    # B. ROI 掩码路径（推理同款）
    workspace = (depth > 0.15) & (depth < 1.20) & mask
    if roi is not None:
        workspace = apply_roi_mask(workspace, roi, depth.shape[0], depth.shape[1])
    cloud_roi = cloud_full[workspace]           # 完整点云按掩码取点（推理 line 523 同构）
    roi_points = int(workspace.sum())
    full_points = int(mask.sum())

    # A vs B：ROI 内同一批像素在两条路径的坐标差（B 从完整点云取，A 同索引反投影）
    uv = np.argwhere(workspace)                  # [v, u]
    us, vs = uv[:, 1], uv[:, 0]
    zs = depth[vs, us]
    xa = (us - camera.cx) / camera.fx * zs
    ya = (vs - camera.cy) / camera.fy * zs
    xb, yb, zb = cloud_roi[:, 0], cloud_roi[:, 1], cloud_roi[:, 2]
    d_ab = np.max(np.abs(np.column_stack([xa - xb, ya - yb, zs - zb])))

    # 中心点对比（与推理 object_centers_camera 同构）
    center_full = cloud_full[mask].mean(axis=0)
    center_roi = cloud_roi.mean(axis=0)

    # C. 错误 crop 对照
    if roi is not None:
        cloud_err, (dx_theory, dy_theory, z_ref) = crop_without_compensation(
            frame['depth'], camera, roi, frame['factor_depth'])
        valid_err = cloud_err[cloud_err[..., 2] > 0]
        if valid_err.size:
            # 与 A 同像素（crop 内 (u',v') = 原 (u-x0, v-y0)）坐标差
            us_c, vs_c = uv[:, 1] - int(roi[0] * depth.shape[1]), uv[:, 0] - int(roi[1] * depth.shape[0])
            in_crop = (us_c >= 0) & (us_c < cloud_err.shape[1]) & (vs_c >= 0) & (vs_c < cloud_err.shape[0])
            xc = (us_c[in_crop] - camera.cx) / camera.fx * zs[in_crop]
            yc = (vs_c[in_crop] - camera.cy) / camera.fy * zs[in_crop]
            d_ac = np.max(np.abs(np.column_stack([
                xa[in_crop] - xc, ya[in_crop] - yc])))
            mean_dx = float(np.mean(xa[in_crop] - xc))
            mean_dy = float(np.mean(ya[in_crop] - yc))
        else:
            d_ac = mean_dx = mean_dy = float('nan')
    else:
        d_ac = mean_dx = mean_dy = float('nan')
        dx_theory = dy_theory = z_ref = 0.0

    report = {
        'roi': roi,
        'full_points': full_points, 'roi_points': roi_points,
        'max_ab_diff': float(d_ab),
        'center_full': center_full.tolist(),
        'center_roi': center_roi.tolist(),
        'center_diff_m': float(np.max(np.abs(center_full - center_roi))),
        'max_ac_diff_m': float(d_ac),
        'mean_ac_dx_dy': [mean_dx, mean_dy],
        'theory_dx_dy': [dx_theory, dy_theory],
        'z_ref': z_ref,
    }

    if verbose:
        print('=' * 62)
        print('ROI 坐标偏移数值检验（推理同款反投影函数，真实帧）')
        print('=' * 62)
        print(f'ROI (归一化): {roi}')
        print(f'全图有效点: {full_points}   ROI 内点: {roi_points}')
        print()
        print('[A 全图 vs B ROI 掩码]  同像素坐标一致性')
        print(f'  max|Δ|(x,y,z) = {d_ab:.3e} m   (像素级确定性一致，应=0)')
        print(f'  中心点: 全图 {np.round(center_full, 4)}  ROI {np.round(center_roi, 4)}')
        print(f'  中心差 = {np.max(np.abs(center_full - center_roi)):.3e} m')
        print()
        print('[A 全图 vs C 错误crop方案] 对照：若真裁剪且内参不补偿')
        print(f'  max|Δ|(x,y) = {d_ac:.4f} m')
        print(f'  平均偏移 Δx={mean_dx:.4f} m  Δy={mean_dy:.4f} m')
        print(f'  理论值 ΔX=(x0/fx)·z = {dx_theory:.4f} m  '
              f'ΔY=(y0/fy)·z = {dy_theory:.4f} m  (z_ref={z_ref:.3f} m)')
        print()
    return report


def main():
    ap = argparse.ArgumentParser(description='ROI 坐标偏移数值检验')
    ap.add_argument('--shared-dir', default=str(ROOT / 'shared_camera_data'))
    ap.add_argument('--roi', default='', help='覆盖 ROI：x_min,y_min,x_max,y_max 归一化'
                                              '（默认读 roi.json，无则全图）')
    args = ap.parse_args()

    roi = None
    if args.roi:
        roi = tuple(float(v) for v in args.roi.split(','))
    else:
        roi_json = Path(args.shared_dir) / 'roi.json'
        if roi_json.exists():
            data = json.loads(roi_json.read_text())
            roi = (float(data['x_min']), float(data['y_min']),
                   float(data['x_max']), float(data['y_max']))
            print(f'[roi_coordinate_check] 读取动态 ROI: {roi}')

    report = run_check(args.shared_dir, roi)

    # 最终结论（阈值 1e-6m=0.001mm：float32 反投影舍入噪声量级，远低于相机精度）
    print('=' * 62)
    if report['max_ab_diff'] < 1e-6:
        print(f'结论: ROI 掩码路径与全图路径同像素坐标一致'
              f'（max|Δ|={report["max_ab_diff"]:.2e} m = 浮点舍入噪声，'
              f'物理意义=0），ROI 截取不产生坐标偏移。')
    else:
        print(f'结论: 发现不一致 max|Δ|={report["max_ab_diff"]:.3e}，需排查！')
    if roi is not None:
        print(f'对照: 若采用"裁剪图像+内参不补偿"的错误方案，偏移量达 '
              f'{report["mean_ac_dx_dy"][0]:.3f}/{report["mean_ac_dx_dy"][1]:.3f} m '
              f'(理论 {report["theory_dx_dy"][0]:.3f}/{report["theory_dx_dy"][1]:.3f} m)')
    print('=' * 62)


if __name__ == '__main__':
    main()
