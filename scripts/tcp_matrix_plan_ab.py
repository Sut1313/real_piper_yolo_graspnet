#!/usr/bin/env python3
"""Plan-only A/B comparison for the legacy and semantic TCP axis mappings.

The script reads one frozen GraspNet CSV, applies the same hand-eye transform
and position offsets to both groups, and changes only ``R_grasp_tcp``.  It
submits MoveGroup goals with ``planning_options.plan_only=True`` through the
project adapter.  It never calls ``execute()`` and never commands the gripper.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

from real_piper_perception.moveit_adapter import MoveGroupInterface


LEGACY_TCP_R = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

SEMANTIC_TCP_R = np.array([
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)

MATRICES = {
    'legacy': LEGACY_TCP_R,
    'semantic': SEMANTIC_TCP_R,
}

ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
BASE_DOWN = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def load_handeye(path: str) -> tuple[np.ndarray, np.ndarray]:
    with open(path) as f:
        data = yaml.safe_load(f)
    r = np.asarray(data['R_cam2base'], dtype=np.float64).reshape(3, 3)
    t = np.asarray(data['t_cam2base'], dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(r)) or not np.all(np.isfinite(t)):
        raise ValueError('hand-eye contains non-finite values')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6):
        raise ValueError('hand-eye rotation is not orthonormal')
    if not np.isclose(np.linalg.det(r), 1.0, atol=1e-6):
        raise ValueError('hand-eye rotation determinant is not +1')
    return r, t


def load_candidates(path: str) -> list[dict]:
    rows = []
    with open(path, newline='') as f:
        for source_index, row in enumerate(csv.DictReader(f)):
            rotation = np.array(
                [float(row[f'r{i}{j}']) for i in range(3) for j in range(3)],
                dtype=np.float64,
            ).reshape(3, 3)
            rows.append({
                'source_index': source_index,
                'name': row['name'],
                'score': float(row['score']),
                'width': float(row['width']),
                'camera_xyz': np.array([
                    float(row['camera_x']),
                    float(row['camera_y']),
                    float(row['camera_z']),
                ], dtype=np.float64),
                'camera_rotation': rotation,
            })
    rows.sort(key=lambda x: x['score'], reverse=True)
    return rows


def pose_message(position: np.ndarray, rotation: np.ndarray) -> Pose:
    quat = Rotation.from_matrix(rotation).as_quat()  # x, y, z, w
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, position)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, quat)
    return msg


def rotation_error_deg(actual: np.ndarray, target: np.ndarray) -> float:
    return float(np.degrees(Rotation.from_matrix(actual @ target.T).magnitude()))


class UrdfFk:
    def __init__(self, urdf_path: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.link6_joint_id = self.model.getJointId('joint6')

    def link6(self, joint_map: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        q = pin.neutral(self.model)
        for name, value in joint_map.items():
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                continue
            idx_q = self.model.joints[joint_id].idx_q
            q[idx_q] = float(value)
        pin.forwardKinematics(self.model, self.data, q)
        placement = self.data.oMi[self.link6_joint_id]
        return np.array(placement.translation), np.array(placement.rotation)


def endpoint_joints(plan) -> dict[str, float]:
    trajectory = plan.trajectory.joint_trajectory
    if not trajectory.points:
        raise ValueError('planned trajectory has no points')
    values = trajectory.points[-1].positions
    return {name: float(value) for name, value in zip(trajectory.joint_names, values)}


def wait_for_feedback(node, timeout_s: float = 10.0) -> JointState:
    event = threading.Event()
    holder: dict[str, JointState] = {}

    def callback(msg: JointState) -> None:
        if all(name in msg.name for name in ARM_JOINTS):
            holder['msg'] = msg
            event.set()

    subscription = node.create_subscription(JointState, '/joint_states_feedback', callback, 10)
    if not event.wait(timeout_s):
        node.destroy_subscription(subscription)
        raise TimeoutError('no /joint_states_feedback received')
    node.destroy_subscription(subscription)
    return holder['msg']


def start_state_from_feedback(msg: JointState) -> tuple[RobotState, dict[str, float]]:
    positions = {name: float(msg.position[msg.name.index(name)]) for name in ARM_JOINTS}
    state = RobotState()
    state.joint_state.name = list(ARM_JOINTS)
    state.joint_state.position = [positions[name] for name in ARM_JOINTS]
    state.joint_state.velocity = [0.0] * len(ARM_JOINTS)
    state.joint_state.effort = [0.0] * len(ARM_JOINTS)
    state.is_diff = False
    return state, positions


def summarize(rows: list[dict]) -> dict:
    planned = [row for row in rows if row['plan_success']]

    def stats(key: str, source=rows):
        values = [float(row[key]) for row in source if row.get(key) is not None]
        if not values:
            return None
        return {
            'min': min(values),
            'max': max(values),
            'mean': sum(values) / len(values),
        }

    return {
        'candidate_count': len(rows),
        'plan_success_count': len(planned),
        'plan_success_rate': len(planned) / len(rows) if rows else 0.0,
        'tcp_forward_dot_graspnet_approach': stats('tcp_forward_dot_graspnet_approach'),
        'abs_finger_opening_dot_graspnet_width': stats(
            'abs_finger_opening_dot_graspnet_width'),
        'approach_down': stats('approach_down'),
        'approach_tilt_deg': stats('approach_tilt_deg'),
        'safe_down_count_at_0_70': sum(row['approach_down'] >= 0.70 for row in rows),
        'planned_position_error_mm': stats('planned_position_error_mm', planned),
        'planned_orientation_error_deg': stats('planned_orientation_error_deg', planned),
        'planning_time_s': stats('planning_time_s'),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', required=True)
    parser.add_argument('--handeye', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--limit', type=int, default=20)
    parser.add_argument('--planning-time', type=float, default=0.5)
    parser.add_argument('--tcp-offset', type=float, default=0.1358)
    parser.add_argument('--grasp-z-offset', type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    handeye_r, handeye_t = load_handeye(args.handeye)
    candidates = load_candidates(args.csv)[:max(1, args.limit)]
    fk = UrdfFk(args.urdf)

    rclpy.init()
    node = rclpy.create_node('tcp_matrix_plan_ab')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    move_group = MoveGroupInterface(
        node, 'arm', base_frame='base_link', end_effector_link='link6')
    move_group.set_planning_time(args.planning_time)
    feedback = wait_for_feedback(node)
    start_state, start_joints = start_state_from_feedback(feedback)

    results: dict[str, list[dict]] = {name: [] for name in MATRICES}
    for rank, candidate in enumerate(candidates):
        camera_r = candidate['camera_rotation']
        base_grasp_r = handeye_r @ camera_r
        tcp_position = handeye_r @ candidate['camera_xyz'] + handeye_t
        tcp_position[2] -= float(args.grasp_z_offset)

        for matrix_name, mapping in MATRICES.items():
            tcp_r = base_grasp_r @ mapping
            tcp_forward = tcp_r[:, 2]
            finger_opening = -tcp_r[:, 1]
            approach_down = float(tcp_forward @ BASE_DOWN)
            tilt_deg = float(math.degrees(math.acos(np.clip(approach_down, -1.0, 1.0))))
            link6_position = tcp_position - tcp_forward * float(args.tcp_offset)
            target_pose = pose_message(link6_position, tcp_r)

            record = {
                'rank': rank,
                'source_index': candidate['source_index'],
                'score': candidate['score'],
                'width': candidate['width'],
                'camera_xyz_m': candidate['camera_xyz'].tolist(),
                'tcp_position_m': tcp_position.tolist(),
                'link6_target_position_m': link6_position.tolist(),
                'tcp_rotation': tcp_r.tolist(),
                'tcp_forward': tcp_forward.tolist(),
                'finger_opening_axis': finger_opening.tolist(),
                'tcp_forward_dot_graspnet_approach': float(
                    tcp_forward @ base_grasp_r[:, 0]),
                'abs_finger_opening_dot_graspnet_width': abs(float(
                    finger_opening @ base_grasp_r[:, 1])),
                'approach_down': approach_down,
                'approach_tilt_deg': tilt_deg,
                'plan_success': False,
                'planning_time_s': None,
                'planned_end_joints': None,
                'planned_position_error_mm': None,
                'planned_orientation_error_deg': None,
            }

            move_group._start_state = start_state  # fixed real feedback for every A/B goal
            move_group.set_pose_target(target_pose, end_effector_link='link6')
            started = time.monotonic()
            try:
                plan = move_group.plan()
            except Exception as exc:  # keep the full A/B set even if one request fails
                plan = None
                record['error'] = f'{type(exc).__name__}: {exc}'
            record['planning_time_s'] = time.monotonic() - started
            if plan is not None:
                end = endpoint_joints(plan)
                actual_p, actual_r = fk.link6(end)
                record['plan_success'] = True
                record['planned_end_joints'] = end
                record['planned_position_error_mm'] = float(
                    np.linalg.norm(actual_p - link6_position) * 1000.0)
                record['planned_orientation_error_deg'] = rotation_error_deg(actual_r, tcp_r)
            results[matrix_name].append(record)
            print(
                f'[{matrix_name:8s}] rank={rank:02d} score={candidate["score"]:.4f} '
                f'align={record["tcp_forward_dot_graspnet_approach"]:+.3f} '
                f'down={approach_down:+.3f} plan={record["plan_success"]}',
                flush=True,
            )

    report = {
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'safety': {
            'plan_only': True,
            'execute_called': False,
            'gripper_commanded': False,
            'move_group_allow_trajectory_execution': False,
        },
        'inputs': {
            'csv': str(Path(args.csv).resolve()),
            'csv_sha256': sha256(args.csv),
            'handeye': str(Path(args.handeye).resolve()),
            'handeye_sha256': sha256(args.handeye),
            'urdf': str(Path(args.urdf).resolve()),
            'candidate_limit': len(candidates),
            'planning_time_s': args.planning_time,
            'tcp_offset_m': args.tcp_offset,
            'grasp_z_offset_m': args.grasp_z_offset,
            'real_start_joints_rad': start_joints,
        },
        'matrices': {name: matrix.tolist() for name, matrix in MATRICES.items()},
        'summary': {name: summarize(rows) for name, rows in results.items()},
        'results': results,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report['summary'], ensure_ascii=False, indent=2), flush=True)
    print(f'Wrote {output}', flush=True)

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
