"""用 MoveIt 执行 SRDF ready 命名位姿（arm 组），2026-08-10。

ready 关节值动态解析自 install 的 piper.srdf（<group_state name="ready"
group="arm">），不硬编码。链路与 grasp_executor 相同：
    move_group 规划(/move_action, plan_only) → /execute_trajectory
    → arm_controller(FakeSystem) → /joint_states → driver remap → CAN → 真机
速度 0.1（与 executor velocity_scale 一致）。
"""
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from real_piper_perception.moveit_adapter import MoveGroupInterface

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRDF_PATH = str(PROJECT_ROOT / 'install' / 'piper_with_gripper_moveit' /
                'share' / 'piper_with_gripper_moveit' / 'config' / 'piper.srdf')
GROUP = 'arm'
POSE_NAME = sys.argv[1] if len(sys.argv) > 1 else 'ready'


def resolve_named_pose(srdf_path: str, group: str, name: str) -> dict:
    """从 SRDF 解析命名位姿 → {关节名: 弧度}。找不到则退出。"""
    root = ET.parse(srdf_path).getroot()
    for gs in root.findall('group_state'):
        if gs.get('name') == name and gs.get('group') == group:
            return {j.get('name'): float(j.get('value'))
                    for j in gs.findall('joint')}
    raise SystemExit(f'SRDF 中找不到 {group} 组的 {name} 命名位姿')


def main() -> int:
    joints = resolve_named_pose(SRDF_PATH, GROUP, POSE_NAME)
    print(f'ready 位姿（解析自 SRDF）: {joints}', flush=True)

    rclpy.init()
    node = rclpy.create_node('moveit_go_ready')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    mg = MoveGroupInterface(node, GROUP, base_frame='base_link')
    mg.set_max_velocity_scaling_factor(0.1)
    mg.set_planning_time(10.0)
    mg.set_start_state_to_current_state()
    mg.set_joint_value_target(joints)

    node.get_logger().info(f'[moveit_go_ready] 规划 ready 位姿（plan_only）...')
    plan = mg.plan()
    if plan is None:
        node.get_logger().error('[moveit_go_ready] 规划失败')
        return 1
    node.get_logger().info('[moveit_go_ready] 规划成功，执行轨迹...')
    t0 = time.time()
    ok = mg.execute(plan)
    print(f'执行耗时: {time.time() - t0:.1f}s, 结果: {"成功" if ok else "失败"}',
          flush=True)
    time.sleep(2.0)
    cur = mg.get_current_joint_values()
    print(f'当前关节角（/joint_states）: {[f"{v:.3f}" for v in cur]}',
          flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
