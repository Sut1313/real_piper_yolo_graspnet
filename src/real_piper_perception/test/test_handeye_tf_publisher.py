"""handeye_tf_publisher 单测：文件 → 参数 → TransformStamped 数据链。

覆盖：正常解析、帧名文件优先、fail-loud 各分支（缺文件/缺字段/det 错/
冗余矩阵不一致）、四元数往返。
"""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_piper_perception.handeye_tf_publisher import load_handeye_tf

FIXTURE = str(Path(__file__).parent / 'fixtures' / 'handeye_fixture.yaml')


def test_fixture_parses_and_default_frames():
    d = load_handeye_tf(FIXTURE)
    ts = d['transform']
    # 纯平移 (0.1, 0.2, 0.3) + 单位旋转
    assert np.allclose(d['R'], np.eye(3), atol=1e-9)
    assert np.allclose(d['t'], [0.1, 0.2, 0.3], atol=1e-9)
    assert ts.header.frame_id == 'base_link'
    assert ts.child_frame_id == 'camera_link'
    assert ts.transform.translation.x == 0.1
    assert ts.transform.translation.y == 0.2
    assert ts.transform.translation.z == 0.3
    # 单位旋转 → 单位四元数 [x,y,z,w]
    assert np.allclose([ts.transform.rotation.x, ts.transform.rotation.y,
                        ts.transform.rotation.z, ts.transform.rotation.w],
                       [0, 0, 0, 1], atol=1e-9)


def test_quaternion_roundtrip_nonidentity(tmp_path):
    """非单位旋转：四元数往返还原 R（数值核对）。"""
    R = np.array([[0.0, -0.82806033, 0.56063899],
                  [-1.0, 0.0, 0.0],
                  [0.0, -0.56063899, -0.82806033]])
    t = [0.072, 0.021, 0.548]
    p = tmp_path / 'handeye_rot.yaml'
    p.write_text(f'R_cam2base:\n  {R.tolist()}\nt_cam2base:\n  {t}\n')
    d = load_handeye_tf(str(p))
    ts = d['transform']
    q = [ts.transform.rotation.x, ts.transform.rotation.y,
         ts.transform.rotation.z, ts.transform.rotation.w]
    R_rt = Rotation.from_quat(q).as_matrix()
    assert np.allclose(R_rt, R, atol=1e-6), f'往返误差: {R_rt - R}'


def test_file_frame_keys_override_defaults(tmp_path):
    """帧名优先取文件键（需求：参数从文件读取，不写死）。"""
    p = tmp_path / 'handeye_frames.yaml'
    p.write_text('R_cam2base:\n  [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n'
                 't_cam2base: [0.0, 0.0, 0.5]\n'
                 'parent_frame: world\n'
                 'child_frame: camera_mount\n')
    d = load_handeye_tf(str(p), default_parent='base_link', default_child='camera_link')
    assert d['transform'].header.frame_id == 'world'
    assert d['transform'].child_frame_id == 'camera_mount'


def test_missing_file_raises(tmp_path):
    with pytest.raises(ValueError, match='读取失败'):
        load_handeye_tf(str(tmp_path / 'nonexistent.yaml'))


def test_missing_fields_raises(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('R_cam2base:\n  [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n')
    with pytest.raises(ValueError, match='缺少'):
        load_handeye_tf(str(p))


def test_bad_det_raises(tmp_path):
    """det=-1 的镜像矩阵（仿真 RotationR 型）拒绝发布。"""
    p = tmp_path / 'mirror.yaml'
    p.write_text('R_cam2base:\n  [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]\n'
                 't_cam2base: [0.0, 0.0, 0.0]\n')
    with pytest.raises(ValueError, match='det'):
        load_handeye_tf(str(p))


def test_inconsistent_redundant_matrix_raises(tmp_path):
    """冗余 T_cam2base 与 R/t 矛盾 → 拒绝（数据自相矛盾）。"""
    p = tmp_path / 'inconsistent.yaml'
    p.write_text('R_cam2base:\n  [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n'
                 't_cam2base: [0.1, 0.2, 0.3]\n'
                 'T_cam2base:\n  [[1.0, 0.0, 0.0, 0.9], [0.0, 1.0, 0.0, 0.2],\n'
                 '   [0.0, 0.0, 1.0, 0.3], [0.0, 0.0, 0.0, 1.0]]\n')
    with pytest.raises(ValueError, match='不一致'):
        load_handeye_tf(str(p))
