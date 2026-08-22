#!/usr/bin/env python3
"""候选遍历扫描（2026-08-08）：依次尝试 CSV 候选（score 降序）直到固件接受。

每个候选：变换链 → EndPoseCtrl @5% → 等待 15s → FK 检查是否移动：
  - 拒动（位置不变）：标记不可达，回 zero 继续下一个
  - 移动：等稳定 → FK 回验 → PASS（偏差<50mm/10°）即停，回 zero 收尾
安全：5% 速度；每个候选后 MoveJ 回 zero。
"""
import csv
import sys
import time
from pathlib import Path

import numpy as np

from piper_sdk import C_PiperInterface_V2
import pinocchio as pin

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / 'src' / 'real_piper_perception'))
from real_piper_perception.grasp_transform import load_handeye, camera_pose_to_base, graspnet_to_tcp_rotation  # noqa: E402

CSV_PATH = str(PROJECT_ROOT / 'shared_camera_data' / 'perception_grasps.csv')
HANDEYE = str(PROJECT_ROOT / 'camera_data' / 'handeye_eye_to_hand.yaml')
URDF = str(PROJECT_ROOT / 'src' / 'piper_ros' / 'src' /
           'piper_description' / 'urdf' / 'piper_description.urdf')
FACTOR = 1000
SETTLE_WAIT_S = 15      # 每候选等待判定时间
POS_TOL_MM = 50.0
RPY_TOL_DEG = 10.0
MAX_CANDIDATES = 15


def matrix_to_rpy(R):
    beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    yaw = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    roll = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    return np.degrees([roll, beta, yaw])


def tool_offset():
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    pin.forwardKinematics(model, data, np.zeros(model.nq))
    T6 = data.oMi[model.getJointId('joint6')].homogeneous
    T7 = data.oMi[model.getJointId('joint7')].homogeneous
    return (np.linalg.inv(T6) @ T7)[:3, 3]


def fk_pos(piper):
    try:
        return np.array(piper.GetFK('feedback')[-1][:3])
    except Exception:
        return None


def go_zero(piper):
    piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)
    time.sleep(0.2)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    last = None; stable = 0; t0 = time.time()
    while time.time() - t0 < 120:
        time.sleep(2.0)
        pos = fk_pos(piper)
        if pos is None:
            continue
        if last is not None and np.linalg.norm(pos - last) < 1.0:
            stable += 1
        else:
            stable = 0
        last = pos
        if stable >= 4:
            break


def main():
    handeye = load_handeye(HANDEYE)
    t67 = tool_offset()
    with open(CSV_PATH) as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda r: float(r['score']), reverse=True)

    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print("ERROR: enable timeout"); return 1
    piper.EnableFkCal()
    time.sleep(0.5)

    accepted = 0
    for i, r in enumerate(rows[:MAX_CANDIDATES]):
        score = float(r['score'])
        xyz = (float(r['camera_x']), float(r['camera_y']), float(r['camera_z']))
        Rg = np.array([[float(r[f'r{i}{j}']) for j in range(3)] for i in range(3)])
        p_b, R_b = camera_pose_to_base(xyz, Rg, handeye)
        R_j6 = graspnet_to_tcp_rotation(R_b)
        p_j6 = p_b - R_j6 @ t67
        rpy = matrix_to_rpy(R_j6)
        axis = -R_j6[:, 1]

        print(f"\n[{i+1}] score={score:.3f} axis_down={-axis[2]:.2f} "
              f"J6=({p_j6[0]*1000:.0f},{p_j6[1]*1000:.0f},{p_j6[2]*1000:.0f})mm "
              f"rpy=({rpy[0]:.0f},{rpy[1]:.0f},{rpy[2]:.0f})deg", flush=True)

        start_pos = fk_pos(piper)
        piper.MotionCtrl_2(0x01, 0x00, 5, 0x00)
        time.sleep(0.2)
        vals = [round(v * FACTOR) for v in list(p_j6) + list(rpy)]
        piper.EndPoseCtrl(*vals)

        # 等待 SETTLE_WAIT_S 后判定是否移动
        time.sleep(SETTLE_WAIT_S)
        now_pos = fk_pos(piper)
        if now_pos is None or start_pos is None or np.linalg.norm(now_pos - start_pos) < 5.0:
            print("  -> 固件拒动（不可达），跳过", flush=True)
            go_zero(piper)
            continue

        # 移动了：等稳定 + 回验
        last = None; stable = 0; t0 = time.time()
        while time.time() - t0 < 120:
            time.sleep(2.0)
            pos = fk_pos(piper)
            if pos is None:
                continue
            if last is not None and np.linalg.norm(pos - last) < 1.0:
                stable += 1
            else:
                stable = 0
            last = pos
            if stable >= 4:
                break
        fk = piper.GetFK('feedback')[-1]
        fk_pos_v = np.array(fk[:3]); fk_rpy = np.array(fk[3:])
        dev_pos = np.linalg.norm(fk_pos_v - p_j6 * 1000)
        dev_rpy = np.linalg.norm(fk_rpy - rpy)
        ok = dev_pos < POS_TOL_MM and dev_rpy < RPY_TOL_DEG
        print(f"  -> 执行后 FK=({fk_pos_v[0]:.0f},{fk_pos_v[1]:.0f},{fk_pos_v[2]:.0f})mm "
              f"dev_pos={dev_pos:.0f}mm dev_rpy={dev_rpy:.0f}deg "
              f"-> {'PASS' if ok else 'FAIL'}", flush=True)
        if ok:
            accepted += 1
            print("  ACCEPTED - stopping scan", flush=True)
            go_zero(piper)
            break
        go_zero(piper)

    print(f"\nscan done: {accepted} candidate(s) accepted", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
