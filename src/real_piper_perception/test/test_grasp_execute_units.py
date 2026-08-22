"""C2/M1 修复的纯函数 TDD 单测（不依赖 MoveIt / ROS 节点）。

覆盖：
- joint7 命令换算（总开度 ↔ joint7 半行程，URDF prismatic 米制 + 驱动
  gripper_val_mutiple=2 语义；越限钳制）
- TCP→link6 位姿折算（I1：规划端点 link6，右乘逆偏移 T(0,0,-0.1358)，
  TCP_OFFSET_FROM_LINK6_Z 为 URDF 权威值）
- 夹爪反馈"等待新反馈（带超时）"（M1，配合 C1 的 MultiThreadedExecutor）
"""

import threading
import time
from types import SimpleNamespace

import numpy as np

from real_piper_perception.grasp_execute import (
    GRIPPER_JOINT7_LIMIT_M,
    GRIPPER_VAL_MULTIPLE,
    GraspExecutor,
    joint7_cmd_for_opening,
    opening_for_joint7_cmd,
)


# ---------------------------------------------------------------------------
# C2：joint7 命令换算（prismatic 米制、半行程、驱动 ×2 语义）
# ---------------------------------------------------------------------------

def test_joint7_cmd_half_opening():
    # 真实 CSV 物体 0.0482 + margin 0.005 = 0.0532 总开度 → joint7 = 0.0266
    assert abs(joint7_cmd_for_opening(0.0532) - 0.0266) < 1e-9
    assert abs(joint7_cmd_for_opening(0.048) - 0.024) < 1e-9
    # 全开 0.07 总开度（SRDF open=0.035）→ joint7 = 0.035
    assert abs(joint7_cmd_for_opening(0.07) - 0.035) < 1e-9


def test_joint7_cmd_clamp():
    # 越上限：0.08 总开度 > 0.07 → 钳制 0.035（不直发 width+margin，C2 越界问题）
    assert joint7_cmd_for_opening(0.08) == GRIPPER_JOINT7_LIMIT_M
    assert joint7_cmd_for_opening(0.12) == GRIPPER_JOINT7_LIMIT_M
    # 负值 → 0（不反向张开）
    assert joint7_cmd_for_opening(-0.01) == 0.0


def test_opening_for_joint7_cmd():
    assert abs(opening_for_joint7_cmd(0.035) - 0.07) < 1e-9
    assert abs(opening_for_joint7_cmd(0.024) - 0.048) < 1e-9


def test_roundtrip_within_limits():
    # 限位内往返：opening(joint7(opening)) == opening
    for opening in (0.0, 0.01, 0.0482, 0.0532, 0.07):
        j7 = joint7_cmd_for_opening(opening)
        assert opening_for_joint7_cmd(j7) <= GRIPPER_JOINT7_LIMIT_M * GRIPPER_VAL_MULTIPLE
        if j7 < GRIPPER_JOINT7_LIMIT_M - 1e-12:  # 未钳制
            assert abs(opening_for_joint7_cmd(j7) - opening) < 1e-9


# ---------------------------------------------------------------------------
# I1：TCP 位姿 → link6 位姿折算（右乘逆偏移 T(0,0,-0.1358)，纯平移）
# ---------------------------------------------------------------------------

def test_tcp_pose_to_link6_pure_translation():
    ex = GraspExecutor(node=None, move_group=None, params={})
    out = ex._tcp_pose_to_link6(
        {'pos': [0.2, 0.2, 0.8], 'quat': [1.0, 0.0, 0.0, 0.0]})
    # 单位姿态：TCP Z 轴 = 基座 Z → link6 在 TCP 下方 0.1358m（URDF 权威值）
    assert np.allclose(out['pos'], [0.2, 0.2, 0.8 - 0.1358], atol=1e-9)
    assert np.allclose(out['quat'], [1.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_tcp_pose_to_link6_rotated():
    from scipy.spatial.transform import Rotation
    ex = GraspExecutor(node=None, move_group=None, params={})
    # 绕 X 轴 90°：TCP Z 轴 → 基座 -Y，故 link6 沿基座 +Y 偏移 0.1358m
    q = Rotation.from_euler('x', np.pi / 2).as_quat()  # [x,y,z,w]
    out = ex._tcp_pose_to_link6(
        {'pos': [0.2, 0.2, 0.8],
         'quat': [float(q[3]), float(q[0]), float(q[1]), float(q[2])]})
    assert np.allclose(out['pos'], [0.2, 0.2 + 0.1358, 0.8], atol=1e-9)
    # 姿态不变（纯平移偏移）
    assert np.allclose(out['quat'],
                       [float(q[3]), float(q[0]), float(q[1]), float(q[2])],
                       atol=1e-9)


# ---------------------------------------------------------------------------
# M1：等待新反馈（带超时）
# ---------------------------------------------------------------------------

def _exec():
    return GraspExecutor(node=None, move_group=None, params={})


def _feedback_msg(value: float):
    return SimpleNamespace(name=['gripper'], position=[value])


def test_wait_feedback_new_value_arrives():
    ex = _exec()
    arrived = []
    def feeder():
        time.sleep(0.1)
        ex._feedback_cb(_feedback_msg(0.041))
        arrived.append(True)
    t = threading.Thread(target=feeder, daemon=True)
    t.start()
    got = ex._read_gripper_feedback_wait(1.0)
    t.join(timeout=2.0)
    assert arrived and got == 0.041


def test_wait_feedback_timeout_returns_latest():
    ex = _exec()
    ex._feedback_cb(_feedback_msg(0.03))  # 闭合前旧值
    t0 = time.monotonic()
    # 无新反馈 → 超时（0.5s 上限）返回最近缓存，而非 None
    got = ex._read_gripper_feedback_wait(0.5)
    elapsed = time.monotonic() - t0
    assert got == 0.03
    assert 0.4 <= elapsed < 1.0


def test_wait_feedback_timeout_no_data():
    ex = _exec()
    assert ex._read_gripper_feedback_wait(0.3) is None


def test_feedback_is_meters_total_opening():
    # C2：反馈 = 驱动 grippers_angle/1e6 的米制总开度，直接缓存不换算
    ex = _exec()
    ex._feedback_cb(_feedback_msg(0.041))
    assert ex._read_gripper_feedback() == 0.041


def test_stable_feedback_rejects_transient_24mm_and_returns_closed_baseline():
    """回归 2026-08-21 假阳性：69.7→24.0→4.3mm 必须取稳定 4.3mm。"""
    ex = _exec()

    def feeder():
        # 24mm 只短暂停留，小于稳定窗口；随后落到空夹全闭 4.3mm。
        for _ in range(4):
            ex._feedback_cb(_feedback_msg(0.0240))
            time.sleep(0.015)
        for value in (0.0048, 0.0045, 0.0043, 0.0042, 0.0043,
                      0.0043, 0.0042, 0.0043, 0.0043, 0.0043):
            ex._feedback_cb(_feedback_msg(value))
            time.sleep(0.02)

    thread = threading.Thread(target=feeder, daemon=True)
    thread.start()
    got = ex._read_gripper_feedback_stable(
        timeout_s=1.0, stable_window_s=0.12,
        stable_tol_m=0.0008, min_samples=5)
    thread.join(timeout=2.0)
    assert got is not None
    assert abs(got - 0.0043) < 0.0003


def test_stable_feedback_times_out_on_continuous_motion():
    ex = _exec()

    def feeder():
        value = 0.060
        for _ in range(20):
            ex._feedback_cb(_feedback_msg(value))
            value -= 0.002
            time.sleep(0.015)

    thread = threading.Thread(target=feeder, daemon=True)
    thread.start()
    got = ex._read_gripper_feedback_stable(
        timeout_s=0.25, stable_window_s=0.10,
        stable_tol_m=0.0005, min_samples=4)
    thread.join(timeout=1.0)
    assert got is None


# ---------------------------------------------------------------------------
# N1（修复轮 2）：joint7 钳制与判定联动——越限 fail-closed，杜绝误报 success
# ---------------------------------------------------------------------------

class _FakeMoveGroup:
    """MoveGroupInterface 替身：记录调用，plan/笛卡尔规划恒成功（无 ROS 依赖）。"""

    def __init__(self):
        self.calls = []
        self.plan_result = object()
        self.cartesian_fraction = 1.0

    def set_planning_time(self, t):
        self.calls.append(('set_planning_time', t))

    def set_start_state_to_current_state(self):
        self.calls.append(('start_state',))

    def set_joint_value_target(self, target):
        self.calls.append(('joint_target', dict(target)))

    def set_pose_target(self, pose, end_effector_link=None):
        self.calls.append(('pose_target', end_effector_link))

    def compute_cartesian_path(self, poses, eef_step, jump_threshold,
                               avoid_collisions):
        self.calls.append(('cartesian', len(poses)))
        return self.plan_result, self.cartesian_fraction

    def plan(self):
        self.calls.append(('plan',))
        return self.plan_result

    def set_max_velocity_scaling_factor(self, v):
        self.calls.append(('velocity', v))

    def execute(self, plan):
        self.calls.append(('execute', plan))
        return True


def _pose(x=0.4, y=0.0, z=0.2):
    return {'pos': [x, y, z], 'quat': [1.0, 0.0, 0.0, 0.0]}


class _FakePiper:
    """piper_sdk 夹爪接口替身：记录 GripperCtrl 命令（SDK 直控语义，2026-08-07）。"""

    def __init__(self):
        self.calls = []

    def GripperCtrl(self, angle, speed, ctrl, _):
        self.calls.append(angle)


def test_close_gripper_normal_width_uses_sdk_command():
    # SDK 直控（2026-08-07 用户裁决）：cmd 0.0532 总开度 → GripperCtrl(53200)，
    # 0.001mm 单位 = 总开度(米)×1e6，不再走 MoveIt gripper 组（无 joint_target）
    fake = _FakeMoveGroup()
    ex = GraspExecutor(node=None, move_group=None, params={},
                       gripper_move_group=fake)
    piper = _FakePiper()
    ex._piper = piper
    assert ex._close_gripper(0.0532) is True
    assert piper.calls and piper.calls[0] == 53200
    assert not any(c[0] in ('joint_target', 'plan', 'execute')
                   for c in fake.calls)


def test_close_gripper_boundary_at_max_opening_allowed():
    # 边界：cmd = 0.07（= 夹爪最大开度）→ GripperCtrl(70000) 允许执行
    fake = _FakeMoveGroup()
    ex = GraspExecutor(node=None, move_group=None, params={},
                       gripper_move_group=fake)
    piper = _FakePiper()
    ex._piper = piper
    assert ex._close_gripper(0.07) is True
    assert piper.calls[0] == 70000


def test_close_gripper_fail_closed_when_over_limit():
    # N1：物体宽 0.078 → cmd 0.083 > 夹爪最大开度 0.07 → fail-closed，
    # 且完全不触碰 MoveIt / SDK（不 GripperCtrl / plan / execute）
    fake = _FakeMoveGroup()
    ex = GraspExecutor(node=None, move_group=None, params={},
                       gripper_move_group=fake)
    piper = _FakePiper()
    ex._piper = piper
    assert ex._close_gripper(0.083) is False
    assert piper.calls == []
