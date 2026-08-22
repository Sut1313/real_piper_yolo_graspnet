#!/usr/bin/env python3
"""第 4 步：相机全链路（真实候选 → handeye → M → EndPoseCtrl → FK 回验），2026-08-08。

流程：
  1. 读最新 perception_grasps.csv（取第一个高分候选）
  2. 变换链：camera → base（T_cam2base）→ M 轴映射 → J6 目标
  3. 工具偏移补偿：p_j6 = p_tcp - R_j6 @ t_6_7（t_6_7 = (0,0,135.8mm)，URDF 提取）
  4. 欧拉角（XYZ 顺序，与 SDK/固件一致）→ EndPoseCtrl @5% MOVE P
  5. FK 稳定判据等待到位 → 回验（位置/姿态偏差）→ MoveJ 回 zero
安全：5% 速度；回验偏差 >50mm 或 >10° 报失败。
"""
import csv
import sys
import time
import numpy as np

from piper_sdk import C_PiperInterface_V2
import pinocchio as pin

sys.path.insert(0, './src/real_piper_perception')
from real_piper_perception.grasp_transform import load_handeye, camera_pose_to_base, graspnet_to_tcp_rotation  # noqa: E402

CSV_PATH = './shared_camera_data/perception_grasps.csv'
HANDEYE = './camera_data/handeye_eye_to_hand.yaml'
URDF = ('./src/piper_ros/src/'
        'piper_description/urdf/piper_description.urdf')
FACTOR = 1000
POS_TOL_MM = 50.0   # 回验位置容差
RPY_TOL_DEG = 10.0  # 回验姿态容差


def matrix_to_rpy(R):
    """旋转矩阵 → XYZ 顺序欧拉角（度），与 SDK __MatrixToeula 同式。"""
    beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    yaw = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    roll = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    return np.degrees([roll, beta, yaw])


def tool_offset():
    """link6→link7 平移（link6 系），URDF 提取。"""
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    pin.forwardKinematics(model, data, np.zeros(model.nq))
    T6 = data.oMi[model.getJointId('joint6')].homogeneous
    T7 = data.oMi[model.getJointId('joint7')].homogeneous
    return (np.linalg.inv(T6) @ T7)[:3, 3]


def latest_candidate():
    with open(CSV_PATH) as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda r: float(r['score']), reverse=True)
    r = rows[0]
    R_grasp = np.array([[float(r[f'r{i}{j}']) for j in range(3)] for i in range(3)])
    return (float(r['camera_x']), float(r['camera_y']), float(r['camera_z'])), R_grasp, float(r['score'])


def main():
    handeye = load_handeye(HANDEYE)
    xyz, R_grasp, score = latest_candidate()
    print(f"candidate: score={score:.3f} cam_xyz={np.round(xyz,3)}")

    # 变换链
    p_base, R_base_grasp = camera_pose_to_base(xyz, R_grasp, handeye)
    R_j6 = graspnet_to_tcp_rotation(R_base_grasp)   # GraspNet → J6（M 映射）
    t67 = tool_offset()
    p_j6 = p_base - R_j6 @ t67                      # 工具偏移补偿（夹爪中心 → J6）
    rpy = matrix_to_rpy(R_j6)
    print(f"J6 target: pos={np.round(p_j6*1000,1)}mm rpy={np.round(rpy,2)}deg (XYZ 顺序)")

    # 发送（5% MOVE P）
    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print("ERROR: enable timeout"); return 1
    piper.MotionCtrl_2(0x01, 0x00, 5, 0x00)
    time.sleep(0.2)
    vals = [round(v * FACTOR) for v in list(p_j6) + list(rpy)]
    print(f"EndPoseCtrl({vals}) @5%", flush=True)
    piper.EndPoseCtrl(*vals)

    # 等待到位（FK 稳定判据）
    piper.EnableFkCal()
    time.sleep(0.5)
    last = None; stable = 0; t0 = time.time()
    while time.time() - t0 < 180:
        time.sleep(2.0)
        fk = piper.GetFK('feedback')[-1]
        pos = np.array(fk[:3])
        if last is not None and np.linalg.norm(pos - last) < 1.0:
            stable += 1
        else:
            stable = 0
        last = pos
        if stable >= 4:
            break
    print(f"settled after {time.time()-t0:.0f}s", flush=True)

    # 回验
    fk = piper.GetFK('feedback')[-1]
    fk_pos = np.array(fk[:3]); fk_rpy = np.array(fk[3:])
    dev_pos = np.linalg.norm(fk_pos - p_j6 * 1000)
    dev_rpy = np.linalg.norm(fk_rpy - rpy)
    print(f"FK feedback: pos={np.round(fk_pos,1)}mm rpy={np.round(fk_rpy,2)}deg")
    print(f"deviation: pos={dev_pos:.1f}mm rpy={dev_rpy:.1f}deg "
          f"-> {'PASS' if dev_pos < POS_TOL_MM and dev_rpy < RPY_TOL_DEG else 'FAIL'}")

    # 回 zero
    print("returning to zero via MoveJ @5%", flush=True)
    piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)
    time.sleep(0.2)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    last = None; stable = 0; t0 = time.time()
    while time.time() - t0 < 120:
        time.sleep(2.0)
        fk = piper.GetFK('feedback')[-1]
        pos = np.array(fk[:3])
        if last is not None and np.linalg.norm(pos - last) < 1.0:
            stable += 1
        else:
            stable = 0
        last = pos
        if stable >= 4:
            break
    print("back to zero", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
