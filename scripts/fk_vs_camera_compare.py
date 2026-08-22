#!/usr/bin/env python3
"""FK 与相机感知位置比对（2026-08-08 用户裁决：正运动学比对相机实际识别位置）。

前提：机械臂停在 grasp 位（dry_run_max_stage=grasp 截断后）或任意位姿；
流程：读 /joint_states（真机）→ /compute_fk 求 link7（夹爪）位置；
      读最新 perception_grasps.csv 候选 → handeye → base 系物体感知位置；
      输出 Δxyz = FK(link7) - 感知物体位置（米）。
用法：python3 scripts/fk_vs_camera_compare.py [--candidate N]
"""
import argparse
import csv
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

sys.path.insert(0, 'src/real_piper_perception')
from real_piper_perception.grasp_transform import load_handeye, camera_pose_to_base  # noqa: E402

CSV_PATH = 'shared_camera_data/perception_grasps.csv'
HANDEYE = 'camera_data/handeye_eye_to_hand.yaml'


class FKCompare(Node):
    def __init__(self, candidate_idx):
        super().__init__('fk_vs_camera_compare')
        self.candidate_idx = candidate_idx
        self._joints = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 1)
        self._fk = self.create_client(GetPositionFK, '/compute_fk')

    def _js_cb(self, msg):
        self._joints = msg

    def run(self):
        # 等关节数据
        t0 = time.time()
        while self._joints is None and time.time() - t0 < 10:
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joints is None:
            print('ERROR: /joint_states 无数据'); return 1

        # FK link7
        if not self._fk.wait_for_service(timeout_sec=5.0):
            print('ERROR: /compute_fk 不可用'); return 1
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['link7']
        req.robot_state.joint_state = self._joints
        fut = self._fk.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        resp = fut.result()
        if not resp.pose_stamped:
            print('ERROR: FK 无结果'); return 1
        p_fk = resp.pose_stamped[0].pose.position
        fk_pos = np.array([p_fk.x, p_fk.y, p_fk.z])
        print(f'FK link7 位置: ({fk_pos[0]*1000:.1f}, {fk_pos[1]*1000:.1f}, '
              f'{fk_pos[2]*1000:.1f}) mm')

        # 相机感知位置（最新 CSV 候选）
        handeye = load_handeye(HANDEYE)
        with open(CSV_PATH) as f:
            rows = sorted(csv.DictReader(f),
                          key=lambda r: float(r['score']), reverse=True)
        if self.candidate_idx is not None and self.candidate_idx < len(rows):
            r = rows[self.candidate_idx]
        else:
            r = rows[0]
        xyz = (float(r['camera_x']), float(r['camera_y']), float(r['camera_z']))
        Rg = np.array([[float(r[f'r{i}{j}']) for j in range(3)] for i in range(3)])
        pos_base, _ = camera_pose_to_base(xyz, Rg, handeye)
        print(f'感知物体位置(base): ({pos_base[0]*1000:.1f}, {pos_base[1]*1000:.1f}, '
              f'{pos_base[2]*1000:.1f}) mm  score={float(r["score"]):.3f}')

        # 比对（夹爪 link7 基准 vs 物体感知中心）
        delta = fk_pos - pos_base
        print(f'Δxyz (FK link7 - 感知物体): ({delta[0]*1000:.1f}, {delta[1]*1000:.1f}, '
              f'{delta[2]*1000:.1f}) mm')
        print(f'注：link7 基准 ≠ 两指中心（夹爪几何偏移未计入）；'
              f'z 偏差含 grasp_z_offset 当前值')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--candidate', type=int, default=None)
    args = ap.parse_args()
    rclpy.init()
    node = FKCompare(args.candidate)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
