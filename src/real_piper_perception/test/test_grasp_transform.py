from pathlib import Path

import numpy as np
from real_piper_perception.grasp_transform import (
    load_handeye, camera_pose_to_base, graspnet_to_tcp_rotation, GRASPNET_TO_PIPER_TCP_R)

# M3 修复：fixture 路径基于本文件位置（cwd 无关，包目录内运行 pytest 也通过）
FIXTURE = str(Path(__file__).parent / 'fixtures' / 'handeye_fixture.yaml')

def test_load_handeye_returns_4x4():
    m = load_handeye(FIXTURE)
    assert m.shape == (4, 4)
    assert np.allclose(m[3], [0, 0, 0, 1])

def test_camera_to_base_translation():
    m = load_handeye(FIXTURE)
    pos, rot = camera_pose_to_base((0.0, 0.0, 0.0), np.eye(3).flatten(), m)
    assert np.allclose(pos, [0.1, 0.2, 0.3], atol=1e-6)

def test_graspnet_to_tcp_maps_axes():
    # 2026-08-21 语义矩阵：TCP +Z=GraspNet +X，TCP +Y=GraspNet +Y，
    # TCP +X=-GraspNet +Z。
    R = GRASPNET_TO_PIPER_TCP_R
    out = graspnet_to_tcp_rotation(np.eye(3))
    assert np.allclose(out, R, atol=1e-9)
    assert np.allclose(R[:, 2], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(R[:, 1], [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(R[:, 0], [0.0, 0.0, -1.0], atol=1e-9)
    # 正交性
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
