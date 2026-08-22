#!/usr/bin/env python3
"""第 0 步：SDK CalFK（DH 参数 FK）vs pinocchio URDF FK 一致性验证（2026-08-08）。

目的：
  1. 确认 piper_sdk 的 DH 参数与项目 URDF 是否一致（决定 SDK FK 能否作为
     独立验证参考 + EndPoseCtrl 欧拉角语义对齐）。
  2. 判定项目 URDF 对应 SDK 的哪个 DH 版本（dh_is_offset=0 无偏 / 1 有 j2/j3 2° 偏）。

输出：每组关节角下 SDK（两版）与 pinocchio 的位置偏差(mm) 与姿态角差(deg)。
纯计算，不动真机。
"""
import sys
from pathlib import Path

import numpy as np

import pinocchio as pin

from piper_sdk.kinematics import C_PiperForwardKinematics

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_URDF = str(_PROJECT_ROOT / 'src' / 'piper_ros' / 'src' /
            'piper_description' / 'urdf' / 'piper_description.urdf')


def pinocchio_fk(joints_rad):
    """URDF FK：返回 link6 的位置(m) 与旋转矩阵。"""
    model = pin.buildModelFromUrdf(_URDF)
    data = model.createData()
    q = np.zeros(model.nq)
    # URDF joint 序（与 SDK joint1..6 对应，由模型 joint 名确认）
    for i, name in enumerate(model.names[1:]):  # 跳过 universe
        if name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'):
            q[model.getJointId(name) - 1] = joints_rad[list(
                ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')).index(name)]
    pin.forwardKinematics(model, data, q)
    T = data.oMi[model.getJointId('joint6')].homogeneous
    return T[:3, 3], T[:3, :3]


def sdk_fk(joints_rad, dh_offset):
    """SDK CalFK：返回 joint6 [x,y,z,r,p,y]（mm/度）并转成 (位置m, 旋转矩阵)。"""
    fk = C_PiperForwardKinematics(dh_is_offset=dh_offset)
    pose = fk.CalFK(list(joints_rad))[-1]  # joint6
    x, y, z, r, p, yaw = pose
    pos = np.array([x, y, z]) / 1000.0
    R = rpy_to_matrix(np.radians(r), np.radians(p), np.radians(yaw))
    return pos, R


def rpy_to_matrix(r, p, y):
    """XYZ 顺序欧拉角（R = Rz·Ry·Rx）→ 旋转矩阵（与 SDK __MatrixToeula 同序）。"""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def rot_angle(R1, R2):
    """两旋转矩阵的轴角差（度）。"""
    R = R1.T @ R2
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(cos_a))


def main():
    # 测试关节角组（弧度）：零位 + 代表性位姿
    test_sets = {
        'zero':            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'zero_j3_90':      [0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0],
        'reach_up':        [0.0, np.pi / 4, np.pi / 4, 0.0, 0.0, 0.0],
        'horizontal':      [np.pi / 6, np.pi / 2, 0.0, 0.0, np.pi / 6, 0.0],
        'downward':        [0.0, np.pi / 2, -np.pi / 6, 0.0, 0.0, 0.0],
        'random1':         [0.3, 0.8, -0.5, 0.2, -0.3, 0.6],
        'random2':         [-0.4, 1.2, 0.7, 0.5, 0.4, -0.2],
    }
    print(f"{'set':<12}{'SDK版':<8}{'pos偏差(mm)':<14}{'姿态角差(deg)':<14}")
    print('-' * 50)
    for name, joints in test_sets.items():
        p_pin, R_pin = pinocchio_fk(joints)
        for off in (0, 1):
            try:
                p_sdk, R_sdk = sdk_fk(joints, off)
                dpos = np.linalg.norm(p_sdk - p_pin) * 1000.0
                dang = rot_angle(R_sdk, R_pin)
                print(f"{name:<12}{off:<8}{dpos:<14.3f}{dang:<14.3f}")
            except Exception as e:
                print(f"{name:<12}{off:<8}ERROR: {e}")


if __name__ == '__main__':
    sys.exit(main())
