"""GraspNet candidate parsing and ranking for real Piper grasping.

Parses the 25-field-per-row CSV written by the GraspNet side into
PerceptionTarget structs, ranks them by score with an 8-pose-bucket
round-robin reorder, and provides a lateral self-consistency gate
(center vs grasp center) as a stand-in for Gazebo ground truth.

Note: the brief calls the format "25 列", but the enumerated column
list and the real shared_camera_data/perception_grasps.csv both have
24 fields per row:
  name(1) + camera_x/y/z(3) + count/score/width/height/depth(5)
  + r00..r22(9) + center_camera_x/y/z(3) + link6_grasp_z/tcp_forward_z/
  source_dist_xy(3) = 24.
EXPECTED_COLS reflects that.
"""

from dataclasses import dataclass
import csv

import numpy as np

EXPECTED_COLS = 24  # brief 枚举列格式（见模块 docstring）


@dataclass
class PerceptionTarget:
    name: str
    xyz_camera: np.ndarray          # shape (3,)
    rotation: np.ndarray            # shape (3,3) 行主序组装的旋转矩阵
    score: float
    width: float
    center_camera: np.ndarray       # shape (3,)
    link6_grasp_z: float
    tcp_forward_z: float
    source_dist_xy: float


def parse_grasp_csv(path: str) -> list[PerceptionTarget]:
    """解析 24 列 CSV；表头跳过；坏行（列数不足/数值错误）跳过并打印警告；空文件返回 []。"""
    cands: list[PerceptionTarget] = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        for lineno, row in enumerate(reader, start=1):
            if lineno == 1:
                continue  # 表头（含列名）
            if len(row) < EXPECTED_COLS:
                print(f'[grasp_candidates] skip bad row {lineno}: '
                      f'expected {EXPECTED_COLS} columns, got {len(row)}')
                continue
            try:
                name = row[0]
                xyz = np.array([float(row[1]), float(row[2]), float(row[3])])
                score = float(row[5])
                width = float(row[6])
                rotation = np.array([
                    [float(row[9]), float(row[10]), float(row[11])],
                    [float(row[12]), float(row[13]), float(row[14])],
                    [float(row[15]), float(row[16]), float(row[17])],
                ])
                center = np.array([float(row[18]), float(row[19]), float(row[20])])
                link6 = float(row[21])
                tcp = float(row[22])
                src = float(row[23])
            except (ValueError, IndexError) as exc:
                print(f'[grasp_candidates] skip bad row {lineno}: {exc}')
                continue
            cands.append(PerceptionTarget(
                name=name, xyz_camera=xyz, rotation=rotation, score=score,
                width=width, center_camera=center, link6_grasp_z=link6,
                tcp_forward_z=tcp, source_dist_xy=src,
            ))
    return cands


def rank_candidates(cands: list[PerceptionTarget]) -> list[PerceptionTarget]:
    """2026-08-14 用户裁决：score 升序排列（低分优先）。

    背景：本场景高分候选系统性不可达（approach 预检几乎全拒，14 轮实测
    可行候选全部 ≤0.69 且多为 ≤0.2），升序让低分（可达概率高）候选优先
    进入被检序列；配合 max_candidates 截断时，被检窗口覆盖可行集而非
    不可达高分集。替代原"分数降序 + 8 姿态桶 round-robin"。
    """
    return sorted(cands, key=lambda c: c.score)


def self_consistency(c: PerceptionTarget, max_center_dist: float = 0.08) -> bool:
    """自洽检查（替代仿真 Gazebo 真值门控）：
    ||center_camera[:2] - xyz_camera[:2]|| <= max_center_dist。
    center_camera 是物体中心、xyz_camera 是抓取中心，横向距离超限说明候选漂移/误检。"""
    diff = c.center_camera[:2] - c.xyz_camera[:2]
    return bool(float(np.linalg.norm(diff)) <= max_center_dist)


def build_precheck_results(ranked: list, approved_indices: list,
                           selected_index) -> dict:
    """预检结果 → 共享目录 json 结构（推理侧可视化过滤用）。

    只含相机系位姿（xyz_camera + rotation，与 CSV 同坐标系），供推理侧
    precheck_match.match_precheck 按几何匹配当前帧候选（ranked 序号与
    GraspGroup 序不同源，不能按序号传递）。

    Returns:
        {'approved': [{'xyz': [..], 'R': [[..]]}, ...],
         'selected': 同上结构或 None,
         'selected_index': int | None}
    """
    def _pose_of(c: PerceptionTarget) -> dict:
        return {'xyz': c.xyz_camera.tolist(), 'R': c.rotation.tolist()}

    # 防御：-1/None 均视为"未选中"（executor 无全过时 selected_index 可能是 -1）
    valid = selected_index is not None and selected_index >= 0
    return {
        'approved': [_pose_of(ranked[i]) for i in approved_indices],
        'selected': (_pose_of(ranked[selected_index]) if valid else None),
        'selected_index': selected_index if valid else None,
    }
