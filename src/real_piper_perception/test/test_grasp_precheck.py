"""Task 6 预检模块几何部分 TDD 单测（不依赖 MoveIt / ROS 节点）。

approach 和 lift 使用基座 Z 高度差。
"""

from pathlib import Path

import numpy as np

from real_piper_perception.grasp_candidates import PerceptionTarget
from real_piper_perception.grasp_transform import (
    GRASPNET_TO_PIPER_TCP_R, camera_pose_to_base, graspnet_to_tcp_rotation, load_handeye)
from real_piper_perception.grasp_precheck import Prechecker

# M3 修复：fixture 路径基于本文件位置（cwd 无关，包目录内运行 pytest 也通过）
FIXTURE = str(Path(__file__).parent / 'fixtures' / 'handeye_fixture.yaml')


def _cand():
    return PerceptionTarget(name='target', xyz_camera=np.array([0.1, 0.0, 0.5]),
                            rotation=np.eye(3), score=0.9, width=0.05,
                            center_camera=np.array([0.1, 0.0, 0.5]),
                            link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)


def _poses(clearance=0.05, lift_z=0.10, open_lift=0.10):
    p = Prechecker(node=None, move_group=None, ssik_script='', params={})
    h = load_handeye(FIXTURE)
    return p.make_stage_poses(_cand(), h, approach_clearance=clearance,
                              lift_z=lift_z, open_gripper_lift_height=open_lift)


def test_make_stage_poses_geometry():
    p = Prechecker(node=None, move_group=None, ssik_script='', params={})
    h = load_handeye(FIXTURE)
    poses = p.make_stage_poses(_cand(), h, approach_clearance=0.05,
                               lift_z=0.10, open_gripper_lift_height=0.10)
    # approach 比 grasp 沿 base +Z 高 5cm。
    delta = np.asarray(poses['grasp']['pos']) - np.asarray(poses['approach']['pos'])
    assert np.allclose(delta, [0.0, 0.0, -0.05], atol=1e-9)
    assert abs(np.linalg.norm(delta) - 0.05) < 1e-9
    # lift 比 grasp 高 open_gripper_lift_height
    assert abs(poses['lift']['pos'][2] - poses['grasp']['pos'][2] - 0.10) < 1e-9
    # 四元数单位模
    for name in ('approach', 'grasp', 'lift'):
        q = np.array(poses[name]['quat'])
        assert abs(np.linalg.norm(q) - 1.0) < 1e-6


def test_make_stage_poses_grasp_position_pure_translation():
    # fixture 为纯平移 (0.1,0.2,0.3) 单位旋转 → grasp 位置 = [0.2, 0.2, 0.8]
    poses = _poses()
    assert np.allclose(poses['grasp']['pos'], [0.2, 0.2, 0.8], atol=1e-9)


def test_make_stage_poses_orientation_is_semantic_tcp():
    # 姿态 = R_cam2base(单位阵) @ eye @ 新 GRASPNET_TO_PIPER_TCP_R
    poses = _poses()
    R_tcp = graspnet_to_tcp_rotation(np.eye(3))
    assert np.allclose(R_tcp, GRASPNET_TO_PIPER_TCP_R, atol=1e-9)
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R_tcp).as_quat()  # [x,y,z,w]
    # 接口约定 quat=[w,x,y,z]
    for name in ('approach', 'grasp', 'lift'):
        assert poses[name]['quat'] == [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
    # 三段姿态一致（仅 z 高度不同）
    for name in ('approach', 'lift'):
        assert poses[name]['quat'] == poses['grasp']['quat']
    # 位移语义不同但姿态保持一致。
    assert poses['approach']['quat'] == poses['grasp']['quat']
    assert poses['lift']['quat'] == poses['grasp']['quat']


def test_make_stage_poses_grasp_matches_transform_chain():
    # make_stage_poses 的 grasp 应等于 transform 链手算结果
    pos_base, rot_base = camera_pose_to_base(_cand().xyz_camera, _cand().rotation,
                                             load_handeye(FIXTURE))
    r_tcp = graspnet_to_tcp_rotation(rot_base)
    assert np.allclose(r_tcp, GRASPNET_TO_PIPER_TCP_R, atol=1e-9)
    assert np.allclose(pos_base, [0.2, 0.2, 0.8], atol=1e-9)


def test_params_defaults():
    p = Prechecker(node=None, move_group=None, ssik_script='', params={})
    assert p.params['graspnet_6d_analytic_only_precheck'] is True
    assert p.params['enable_piper_analytic_ik_planning'] is True
    assert p.params['piper_analytic_ik_max_solutions'] == 8
    assert p.params['piper_analytic_ik_max_solution_plans'] == 6
    assert p.params['planning_time_precheck'] == 1.0
    # 自定义覆盖生效
    p2 = Prechecker(node=None, move_group=None, ssik_script='',
                    params={'graspnet_6d_analytic_only_precheck': False})
    assert p2.params['graspnet_6d_analytic_only_precheck'] is False
    assert p2.params['planning_time_precheck'] == 1.0


def test_run_ssik_missing_script_returns_empty():
    # ssik 脚本路径不存在 → 告警 + []（不抛异常；analytic-only 门控接管）
    p = Prechecker(node=None, move_group=None, ssik_script='no_such_script.py', params={})
    T = np.eye(4)
    T[:3, 3] = [0.3, 0.0, 0.35]
    assert p._run_ssik(T, np.zeros(6), 8) == []


def test_precheck_stages_empty_and_missing():
    p = Prechecker(node=None, move_group=None, ssik_script='', params={})
    ok, results = p.precheck_stages({}, scan_only_report=False)
    assert ok is False
    assert results == {'approach': False, 'grasp': False, 'lift': False}
    # 只有 approach 段（move_group=None → fail-closed False），AND 语义下后续不再尝试
    ok, results = p.precheck_stages({'approach': {'pos': [0.2, 0.2, 0.8], 'quat': [1, 0, 0, 0]}},
                                    scan_only_report=False)
    assert ok is False
    assert results['approach'] is False
    assert results['grasp'] is False and results['lift'] is False
