"""precheck_match 纯函数单测（推理侧渲染过滤用，无 torch/open3d 依赖）。

被测模块：graspnet-baseline/precheck_match.py（sys.path 注入，宿主机可跑）。
"""
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'graspnet-baseline'))
import precheck_match as pm  # noqa: E402


def _pose(xyz, rpy):
    """与 precheck_results.json 结构一致：{'xyz': [..], 'R': [[..]]}。"""
    return {'xyz': np.array(xyz, dtype=float),
            'R': Rotation.from_euler('xyz', rpy).as_matrix()}


def test_match_marks_approved_and_selected():
    # 候选 0 与 approved 位姿一致；候选 1 偏离；selected 与候选 2 一致
    poses = [_pose([0.1, 0.2, 0.3], [0, 0, 0]),
             _pose([0.5, 0.5, 0.5], [0, 0, 0]),
             _pose([0.2, 0.1, 0.4], [0.1, 0, 0])]
    xyz = np.array([p['xyz'] for p in poses])
    R = np.array([p['R'] for p in poses])
    approved = [_pose([0.1, 0.2, 0.3], [0, 0, 0])]
    selected = _pose([0.2, 0.1, 0.4], [0.1, 0, 0])
    mask, sel = pm.match_precheck(xyz, R, approved, selected)
    # selected 属于 approved 子集：候选 2 显示（红色）；候选 1 偏离不显示
    assert mask.tolist() == [True, False, True]
    assert sel == 2


def test_selected_match_is_unique_nearest_candidate():
    """宽松阈值命中多个候选时，执行页面只能选择位置/姿态最近的一项。"""
    selected = _pose([0.10, 0.20, 0.30], [0, 0, 0])
    poses = [_pose([0.115, 0.20, 0.30], [0, 0, 0]),
             _pose([0.101, 0.20, 0.30], [0, 0, 0])]
    xyz = np.array([p['xyz'] for p in poses])
    R = np.array([p['R'] for p in poses])
    mask, sel = pm.match_precheck(xyz, R, [], selected)
    assert mask.tolist() == [False, True]
    assert sel == 1


def test_match_threshold_rejects_large_offset():
    xyz = np.array([[0.1, 0.2, 0.3]])
    R = np.eye(3)[np.newaxis]
    approved = [_pose([0.2, 0.2, 0.3], [0, 0, 0])]  # 10cm 偏移 > 2cm 阈值
    mask, sel = pm.match_precheck(xyz, R, approved, None)
    assert mask.tolist() == [False]
    assert sel == -1


def test_match_threshold_rejects_large_rotation():
    # 角度是宽松方向检查（同位置多方向候选都视为同一位置可用）：
    # 95° > 90° 阈值拒绝；30° 接受（同一抓取位置的方向多样性）
    xyz = np.array([[0.1, 0.2, 0.3]])
    R = np.eye(3)[np.newaxis]
    approved = [_pose([0.1, 0.2, 0.3], [0, 0, np.radians(95)])]
    mask, sel = pm.match_precheck(xyz, R, approved, None)
    assert mask.tolist() == [False]
    assert sel == -1
    approved = [_pose([0.1, 0.2, 0.3], [0, 0, np.radians(30)])]
    mask, sel = pm.match_precheck(xyz, R, approved, None)
    assert mask.tolist() == [True]


def test_match_accepts_small_rotation():
    xyz = np.array([[0.1, 0.2, 0.3]])
    R = np.eye(3)[np.newaxis]
    approved = [_pose([0.1, 0.2, 0.3], [0, 0, np.radians(5)])]  # 5° < 15° 阈值
    mask, sel = pm.match_precheck(xyz, R, approved, None)
    assert mask.tolist() == [True]


def test_match_empty_approved():
    xyz = np.array([[0.1, 0.2, 0.3]])
    R = np.eye(3)[np.newaxis]
    mask, sel = pm.match_precheck(xyz, R, [], None)
    assert mask.tolist() == [False]
    assert sel == -1


def test_serialize_roundtrip():
    """executor 侧 JSON 结构 → 匹配侧可读（字段名一致性防线）。"""
    approved = [_pose([0.1, 0.2, 0.3], [0, 0, 0])]
    data = {'approved': [{'xyz': a['xyz'].tolist(), 'R': a['R'].tolist()}
                         for a in approved],
            'selected': None, 'selected_index': None}
    # 直接传 JSON 反序列化后的 dict 结构（字段名一致性防线）
    mask, sel = pm.match_precheck(
        np.array([[0.1, 0.2, 0.3]]), np.eye(3)[np.newaxis],
        data['approved'], None)
    assert mask.tolist() == [True]
