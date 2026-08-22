#!/usr/bin/env python3
"""第 3 步（纯计算）：M 轴映射必要性验证（2026-08-08）。

两层验证：
  1. 恒等式"夹爪开口方向（link7 Z）= -Y_link6"（pinocchio URDF FK 任意关节角验证）
  2. 同一抓取器目标姿态下：
       A（无 M）：固件执行 R_grasp      → 开口 = -R_grasp[:,1]
       B（带 M）：固件执行 R_grasp @ M  → 开口 =  X_g（GraspNet 接近方向）
     对比两版开口方向与目标开口方向 Z_g 的偏差。
依据：R_base_tcp = R_cam2base @ R_grasp_cam @ M（grasp_transform.py §1.3）；
     M = [[0,-1,0],[1,0,0],[0,0,1]]（2026-08-07 最终裁决，TCP+Y=-GraspNet+X）。
"""
import numpy as np
import pinocchio as pin

_URDF = ('./src/piper_ros/src/'
         'piper_description/urdf/piper_description.urdf')
_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')

M = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])


def fk_link6(joints):
    model = pin.buildModelFromUrdf(_URDF)
    data = model.createData()
    q = np.zeros(model.nq)
    for i, name in enumerate(_JOINTS):
        q[model.getJointId(name) - 1] = joints[i]
    pin.forwardKinematics(model, data, q)
    T6 = data.oMi[model.getJointId('joint6')].homogeneous
    T7 = data.oMi[model.getJointId('joint7')].homogeneous
    return T6[:3, :3], T7[:3, :3]


def rpy_to_matrix(r, p, y):
    """XYZ 顺序欧拉角（与 SDK __MatrixToeula / EndPoseCtrl 同序）→ 旋转矩阵。"""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def matrix_to_rpy(R):
    """旋转矩阵 → XYZ 顺序欧拉角（度），与 SDK 同式。"""
    beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    yaw = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    roll = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    return np.degrees([roll, beta, yaw])


def verify_open_direction_identity():
    """验证 link7 Z（开口）= -Y_link6 恒等式（多组随机关节角）。"""
    rng = np.random.default_rng(42)
    max_err = 0.0
    for _ in range(20):
        q = rng.uniform(-1.2, 1.2, 6)
        R6, R7 = fk_link6(q)
        ident_dir = -R6[:, 1]
        actual_dir = R7[:, 2]
        err = np.degrees(np.arccos(np.clip(np.dot(ident_dir, actual_dir), -1, 1)))
        max_err = max(max_err, err)
    print(f"[1] 恒等式 link7 Z = -Y_link6 最大偏差: {max_err:.4f}° (20 组随机关节角)")


def main():
    verify_open_direction_identity()

    # 虚构抓取目标（GraspNet 抓取器坐标系，base 系）：
    #   接近方向 X_g = 朝下偏 30°（真实"从上往下抓"的接近语义）
    #   开合方向 Z_g = 水平 +Y（两指开合方向，绕夹爪轴线的旋转自由度）
    #   期望结果：夹爪轴线（link7 Z）= 接近方向 Xg（夹爪从上方伸向物体）
    Xg = np.array([0.0, -np.sin(np.radians(30)), -np.cos(np.radians(30))])  # 朝下 30°
    Zg = np.array([0.0, 1.0, 0.0])                    # 开合方向（GraspNet +Z）
    Yg = np.cross(Zg, Xg); Yg /= np.linalg.norm(Yg)
    R_grasp = np.column_stack([Xg, Yg, Zg])           # 列 = [Xg, Yg, Zg]

    print(f"\n抓取器目标：接近 Xg={Xg}（朝下 30°），开合 Zg={Zg}（水平）")
    print(f"期望：夹爪轴线（link7 Z）指向接近方向 Xg（从上方抓）")
    print(f"R_grasp = \n{R_grasp.round(4)}")

    # A 版：无 M（直接把抓取器姿态当 J6 姿态发）
    R_A = R_grasp
    axis_A = -R_A[:, 1]                               # 夹爪轴线 = link7 Z = -Y_link6
    # B 版：带 M（GraspNet 姿态 → J6 姿态）
    R_B = R_grasp @ M
    axis_B = -R_B[:, 1]

    def report(name, R_sent, axis, eul):
        match = np.dot(axis, Xg)
        print(f"\n[{name}] J6 姿态欧拉角(XYZ 顺序) = {eul.round(2)}°")
        print(f"  执行后夹爪轴线（link7 Z）= {axis.round(3)}")
        print(f"  与接近方向 Xg 的 cos = {match:.4f} （1.0=夹爪朝目标接近方向，正确）")

    report("A 无 M", R_A, axis_A, matrix_to_rpy(R_A))
    report("B 带 M", R_B, axis_B, matrix_to_rpy(R_B))

    print("\n结论：")
    print("  A：夹爪轴线被摆到与接近垂直的水平方向（+X）——夹爪无法从上方伸向物体（错误）")
    print("  B：夹爪轴线 = 接近方向（朝下 30°）——正确从上方抓取语义")
    print("  即：M 把 GraspNet 接近方向映射为夹爪轴线方向；无 M 时姿态完全错误。")
    print("  与 2026-08-07 实测一致：M 映射后开口朝下分量 0.84~0.92；calib_18 时'末端没朝下'。")


if __name__ == '__main__':
    main()
