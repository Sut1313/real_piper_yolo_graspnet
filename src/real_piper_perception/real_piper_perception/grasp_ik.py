"""Pinocchio 数值 IK（2026-08-07）：朝下抓取位姿 IK 求解。

背景：ssik 解析 IK 对朝下姿态无解（实测 0/8）；MoveIt OMPL goal sampler
（KDL）对朝下目标报 Invalid goal state / Unable to sample any valid state。
Pinocchio 阻尼最小二乘（DLS）数值 IK 对可达位姿可靠迭代收敛。

输出关节角与 ssik 同语义（joint1..joint6 顺序，弧度），供 MoveIt joint
space 规划（_plan_to_joints）——绕开 pose 约束的 IK 困境，规划回到
joint space（可靠 + 碰撞感知）。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_URDF = str(_PROJECT_ROOT / 'src' / 'piper_ros' / 'src' /
            'piper_description' / 'urdf' / 'piper_description.urdf')
_LINK6 = 'link6'
JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')

_MODEL = None
_DATA = None


def _ensure_model():
    global _MODEL, _DATA
    if _MODEL is None:
        _MODEL = pin.buildModelFromUrdf(_URDF)
        _DATA = _MODEL.createData()
    return _MODEL, _DATA


def solve_ik_link6(target_4x4: np.ndarray, seed: np.ndarray | None = None,
                   max_iter: int = 200, tol: float = 1e-6,
                   damp: float = 0.05) -> np.ndarray | None:
    """DLS 数值 IK：目标 link6 位姿（4x4 齐次，基座系）→ 6 关节角。

    Args:
        target_4x4: link6 目标位姿（旋转 + 平移，基座系）。
        seed: 初始关节角（6 维，joint1..6）；None = 当前零位。
        max_iter: 最大迭代次数。
        tol: 6D 误差范数收敛阈值（位置+姿态，单位 m/rad）。
        damp: DLS 阻尼系数 λ。

    Returns:
        np.ndarray(6,) 关节角（joint1..6）；不可达/发散返回 None。
    """
    model, data = _ensure_model()
    nq = model.nq
    q = np.zeros(nq)
    if seed is not None:
        s = np.asarray(seed, dtype=np.float64).reshape(6)
        q[:6] = s

    # 关节限位（URDF 烘焙；joint7/gripper 若有则保持 0）
    lo = model.lowerPositionLimit.copy()
    up = model.upperPositionLimit.copy()
    if nq > 6:
        lo[6:] = 0.0
        up[6:] = 0.0

    target = pin.SE3(np.asarray(target_4x4[:3, :3], dtype=np.float64),
                     np.asarray(target_4x4[:3, 3], dtype=np.float64))
    frame_id = model.getFrameId(_LINK6)

    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        oMf = data.oMf[frame_id]
        err = pin.log6(oMf.inverse() * target).vector  # (6,) 位置+姿态误差
        if float(np.linalg.norm(err)) < tol:
            return q[:6].copy()
        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)  # (6,nq)
        J6 = J[:, :6]
        # DLS: dq = -Jᵀ (J Jᵀ + λ²I)⁻¹ err
        dq = -J6.T @ np.linalg.solve(J6 @ J6.T + damp * damp * np.eye(6), err)
        q[:6] = np.clip(q[:6] + dq, lo[:6], up[:6])
    return None
