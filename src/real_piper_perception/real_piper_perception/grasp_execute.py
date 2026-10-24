"""抓取执行序列模块（Task 7）：预位 → approach → grasp → 闭合夹爪 → 判定 → 抬升。

真机 grasp_executor（Task 8）在预检（Task 6 Prechecker.precheck_stages）全过后
调用 :class:`GraspExecutor.execute_pick` 执行完整抓取并用
:func:`grasp_verify.verify_grasp` 判定成功与否。

执行序列（与 brief 逐字对应）：

1. 预位（OMPL plan+execute）——目标 = ``stage_poses['lift']``（grasp 位 +
   open_gripper_lift_height 的抬升预位姿，夹爪张开，同一朝向）。
   brief 未提供 home 位姿参数，stage_poses 也仅有 approach/grasp/lift 三段；
   预位取抬升预位姿作为安全预位目标（物体上方、朝向已对准，后续直线
   下探有足够运动空间）。若 Task 9 联调希望改为固定 home 关节位型，需由
   Task 8 增加 home 参数并改此处（本任务不臆造）。
   注意（M7 deferred）：预位高度 = grasp + open_gripper_lift_height，执行抬升
   高度 = grasp + lift_z，是两个独立参数（默认同为 0.10）；只调其一会使预位
   与终位高度分叉。执行段抬升（lift_z 运动）未经预检（预检 lift 段是 open-lift
   位姿），几何上是子集、风险低。
2. approach 位（笛卡尔直线 compute_cartesian_path，失败回退 OMPL）。
3. grasp 位（OMPL plan+execute）。
4. 闭合夹爪：joint7 关节目标（米，prismatic，半行程）=
   (grasp_width + gripper_margin) / 2——见下方"夹爪单位与机制约定"（C2 修复）。
5. 等待新反馈（≤0.5s，M1）→ 读 /joint_states_feedback 的 gripper 反馈实际
   开度（米制总开度）→ verify_grasp 判定。
6. 成功（夹住）或抓空（没夹住）都执行抬升（grasp 位 + lift_z 的笛卡尔直线，
   不用 stage_poses['lift']——那是预检用的 open-lift 位姿）；失败则保持姿态。

夹爪单位与机制约定（C2 修复，2026-08-06 按真机 URDF/SRDF/驱动/sdk 逐条核实）：
- joint7 是 **prismatic 关节、单位米**，限位 [0, 0.035]，属于独立 gripper 规划组
  （piper.srdf：arm 组 = base_link→link6 仅 joint1-6；gripper 组 = gripper_base→
  link7 仅 joint7）。夹爪闭合必须用独立 gripper MoveGroupInterface（方案 A），
  不能放在 arm 组上（joint7 不在 arm 组，规划必然失败）。
- 手指行程语义：URDF joint7（link7，一指向外 0..0.035m）+ joint8（link8 镜像，
  -0.035..0m）⇒ **指尖总开度 = 2 × joint7**（每指行程 = 总开度/2）。SRDF open
  状态 joint7=0.035（全开，总开度 0.07m）/ close=0（全闭）。
- 命令侧（驱动 piper_ctrl_single_node.py joint_callback，launch 将
  joint_ctrl_single remap 到 /joint_states）：joint7(m) × 1e6 ×
  gripper_val_mutiple → GripperCtrl 行程（0.001mm 单位）；README 真机配置
  gripper_val_mutiple=2 ⇒ GripperCtrl 行程 = 2 × joint7 = 指尖总开度。
- 反馈侧（驱动 PublishArmJointAndGripper：grippers_angle / 1e6）：
  piper_sdk 文档 "grippers_angle: The stroke of the gripper (in 0.001 mm)"——
  **米制总开度**（字段名 "angle" 是历史遗留，实为行程长度）。命令与反馈
  **同为米制总开度**，不存在"弧度 1:1"；判定比较在总开度域（米）进行。
- 物体宽 + margin 必须不超过小夹爪 70mm 总开口；候选选择层先过滤，
  execute_pick 再做一次 fail-closed 防御。实际夹持结果仍由真实反馈判定。
  剩余开→闭映射微标定（指尖-物体实际接触）留 Task 11 阶段 3。

MoveIt2 交互：当前环境未安装 ``moveit_py``（探测：ModuleNotFoundError），
MoveIt 调用为接口占位 + fail-closed（同 Task 6 策略），真实接线在 Task 9
干跑联调（方法名按 moveit_py snake_case API 写，若有出入以 Task 9 实测为准）。
节点/消息导入均延迟或 TYPE_CHECKING，保证无 ROS 环境可导入（单测用）。
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from real_piper_perception.grasp_precheck import Prechecker
from real_piper_perception.grasp_verify import classify_grasp, verify_grasp
from real_piper_perception.gripper_limits import (
    PIPER_SMALL_GRIPPER_MAX_OPENING_M,
    fits_small_gripper,
    opening_m_to_sdk_command,
    required_opening_m,
)

if TYPE_CHECKING:
    from rclpy.node import Node  # 仅类型标注，不强制（node 传 None 时用 print 日志）

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

# 参数键默认值（brief 参数键清单）
PARAM_DEFAULTS = {
    'velocity_scale': 0.02,            # MoveIt 轨迹执行速度缩放
    'gripper_margin': 0.005,           # 夹爪闭合余量（米）
    'gripper_detect_threshold': 0.01,  # 判定阈值（米）
    'lift_z': 0.10,                    # 抬升高度（米，基座系 Z 向上）
    'planning_time_exec': 10.0,        # 执行段规划时间上限（秒）
}

# 执行序列阶段名（失败时返回的 stage 取值；成功/抓空为 ''）
STAGE_ORDER = ('pre_pose', 'approach', 'grasp', 'gripper', 'lift')

# 夹爪关节：MoveIt 规划侧关节名（命令目标）vs /joint_states_feedback 反馈关节名
GRIPPER_JOINT_TARGET = 'joint7'
GRIPPER_JOINT_FEEDBACK = 'gripper'

# 夹爪反馈话题（sensor_msgs/JointState，piper_ros 驱动发布）
FEEDBACK_TOPIC = '/joint_states_feedback'

# 闭合夹爪后等待夹爪稳定/停止的静置时间（秒，brief 原值 0.5s）。
# M1 修复：改"等待新反馈（带超时）"——进入判定前等收到一条更新的反馈，
# 超时则用最近缓存（无数据 → None → 判定按失败处理）。
FEEDBACK_SETTLE_S = 0.5

# 2026-08-21 真机抓空暴露：第一条新反馈可能只是闭合途中的瞬态
#（69.7→24.0→4.3mm）。业务判定必须等待一段连续稳定窗口，并在初判
# grasped 后再做一次保持复核，不能把“新反馈”等同于“稳定反馈”。
FEEDBACK_STABLE_TIMEOUT_S = 3.0
FEEDBACK_STABLE_WINDOW_S = 0.35
FEEDBACK_STABLE_TOL_M = 0.0008
FEEDBACK_STABLE_MIN_SAMPLES = 5

# 空载全开/全闭参考的物理合理区间。除机械异常外，这也会拦截未连接
# SDK 或失效反馈话题产生的默认 0/0 缓存值。
GRIPPER_CALIB_OPEN_MIN_M = 0.050
GRIPPER_CALIB_OPEN_MAX_M = 0.075
GRIPPER_CALIB_CLOSED_MIN_M = 0.000
GRIPPER_CALIB_CLOSED_MAX_M = 0.010
GRIPPER_CALIB_MIN_SPAN_M = 0.030

# ---------------------------------------------------------------------------
# 夹爪（joint7）真机事实常量（C2 修复，2026-08-06 按 URDF/SRDF/驱动/sdk 核实，
# 依据详见模块 docstring"夹爪单位与机制约定"）：
# - URDF piper_description.xacro：joint7 prismatic、米制、限位 [0, 0.035]；
#   joint8 镜像限位 [-0.035, 0] ⇒ 总开度 = 2 × joint7（0.035m/指行程）。
# - piper.srdf：gripper 组 = gripper_base→link7（仅 joint7）。
# - 驱动 joint_callback：命令 joint7(m) × 1e6 × gripper_val_mutiple →
#   GripperCtrl（0.001mm 行程单位）；README 真机配置 mutiple=2。
# - piper_sdk GetArmGripperMsgs：grippers_angle = "stroke of the gripper
#   (in 0.001 mm)"；驱动反馈 grippers_angle/1e6 = 米制总开度。
# ---------------------------------------------------------------------------
GRIPPER_JOINT7_LIMIT_M = 0.035      # URDF joint7 上限（米，每指行程）
GRIPPER_JOINT7_OPEN_M = GRIPPER_JOINT7_LIMIT_M  # SRDF open 状态（全开）
GRIPPER_JOINT7_CLOSE_M = 0.0        # SRDF close 状态（全闭）
# 驱动 gripper_val_mutiple（launch 参数，README 真机配置 =2）：
# GripperCtrl 行程 = joint7(m) × 1e6 × mutiple。本模块命令换算依赖该值，
# 须与驱动 launch 参数一致（若改驱动参数，此处同步修改）。
GRIPPER_VAL_MULTIPLE = 2
# SDK 0.6.2 默认小夹爪范围 [0, 0.07]m，GripperCtrl 单位 0.001mm。
# 使用具名的总开口上限，避免真机逻辑继续隐式依赖 0.035×2 推算。
GRIPPER_MAX_OPENING_M = PIPER_SMALL_GRIPPER_MAX_OPENING_M

# 规划组末端执行器链（I1 修复）：arm 组 tip 为 link6（piper.srdf
# <group name="arm"> chain base_link→link6）。真机 URDF 链接清单 =
# base_link, link1..6, gripper_base, link7, link8——不存在 grasp_tcp
# （grasp_tcp 是仿真独有帧）。位姿目标须先折算回 link6（_tcp_pose_to_link6，
# 与 Prechecker._tcp_to_link6_matrix 同一数学：右乘逆偏移
# T(0,0,-0.1358)，TCP_OFFSET_FROM_LINK6_Z 见 grasp_precheck.py）。
END_EFFECTOR_LINK = 'link6'


# ---------------------------------------------------------------------------
# 夹爪命令/反馈换算纯函数（C2 修复；TDD 单测见 test/test_grasp_execute_units.py）
# ---------------------------------------------------------------------------

def joint7_cmd_for_opening(opening_m: float) -> float:
    """指尖总开度 → joint7 命令（米，prismatic 半行程）。

    URDF 镜像手指（joint7+joint8）⇒ 每指行程 = 总开度/2；驱动
    gripper_val_mutiple=2 将命令翻倍为 GripperCtrl 行程（总开度）。
    越限钳制到 [0, 0.035]（URDF 限位）并返回钳制值（调用方依返回值告警）。
    """
    j7 = float(opening_m) / GRIPPER_VAL_MULTIPLE
    return min(GRIPPER_JOINT7_LIMIT_M, max(0.0, j7))


def opening_for_joint7_cmd(joint7_m: float) -> float:
    """joint7 命令（米）→ 指尖总开度（米）：2 × joint7（驱动 mutiple=2 语义）。"""
    return float(joint7_m) * GRIPPER_VAL_MULTIPLE


def gripper_calibration_is_valid(full_open_m: float,
                                 full_closed_m: float) -> bool:
    """检查空载全开/全闭参考是否足以用于夹持结果判定。"""
    try:
        full_open = float(full_open_m)
        full_closed = float(full_closed_m)
    except (TypeError, ValueError):
        return False
    if not np.isfinite(full_open) or not np.isfinite(full_closed):
        return False
    return (
        GRIPPER_CALIB_OPEN_MIN_M <= full_open <= GRIPPER_CALIB_OPEN_MAX_M
        and GRIPPER_CALIB_CLOSED_MIN_M <= full_closed <= GRIPPER_CALIB_CLOSED_MAX_M
        and full_open - full_closed >= GRIPPER_CALIB_MIN_SPAN_M
    )


# 笛卡尔直线参数（brief：eef_step 0.01、avoid_collisions=True；plan 文档
# jump_threshold 0.0——MoveIt2 该参数已弃用，占位保留 0.0 语义）
CARTESIAN_EEF_STEP = 0.01
CARTESIAN_JUMP_THRESHOLD = 0.0
CARTESIAN_AVOID_COLLISIONS = True

class GraspExecutor:
    """预检通过后的抓取执行器（预位→approach→grasp→闭合→判定→抬升）。

    Args:
        node: rclpy 节点，用于订阅 /joint_states_feedback 缓存 gripper 反馈；
            None 时退化为无订阅（反馈恒为 None → 判定按失败处理）。
        move_group: MoveIt MoveGroupInterface（moveit_py），arm 组；None 为占位，
            MoveIt 交互走 fail-closed + 告警（Task 9 干跑联调接线）。
        gripper_move_group: MoveIt MoveGroupInterface（moveit_py），gripper 组
            （piper.srdf gripper 组 = gripper_base→link7，仅 joint7）。
            C2 修复：夹爪闭合必须用 gripper 组（joint7 不在 arm 组）。
            None 为占位，闭合 fail-closed。
        params: 参数键见 :data:`PARAM_DEFAULTS`（缺省自动补默认值）。
    """

    def __init__(self, node: 'Node' | None, move_group, params: dict,
                 gripper_move_group=None):
        """node: rclpy 节点（订阅 /joint_states_feedback 缓存 gripper 反馈用）；
        move_group: MoveIt MoveGroupInterface（arm 组，可为 None 占位）；
        gripper_move_group: MoveIt MoveGroupInterface（gripper 组，夹爪闭合用，
        C2 修复；可为 None 占位）；
        params 键：velocity_scale(默认0.02)、gripper_margin(默认0.005米)、
        gripper_detect_threshold(默认0.01米)、lift_z(默认0.10米)、planning_time_exec(默认10.0)"""
        self.node = node
        self.move_group = move_group
        self.gripper_move_group = gripper_move_group
        self.params = dict(PARAM_DEFAULTS)
        if params:
            self.params.update(params)

        self._latest_gripper: float | None = None
        self._latest_gripper_stamp: float = float('-inf')  # M1：最近反馈到达时刻
        self._piper = None  # SDK 直控惰性初始化（2026-08-07，参考 piper_sdk demo）
        self._feedback_lock = threading.Lock()
        self._feedback_sub = None
        if node is not None:
            self._setup_feedback_subscription()

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------

    def _log(self, level: str, msg: str) -> None:
        if self.node is not None:
            try:
                getattr(self.node.get_logger(), level)(msg)
                return
            except Exception:
                pass
        print(f'[grasp_execute {level.upper()}] {msg}')

    # ------------------------------------------------------------------
    # 夹爪反馈订阅（/joint_states_feedback，缓存最近 gripper 关节值）
    # ------------------------------------------------------------------

    def _setup_feedback_subscription(self) -> None:
        """节点上订阅 /joint_states_feedback 并缓存 gripper 关节值。

        C1 修复（2026-08-06 实测根因）：反馈订阅必须放在**独立可重入回调组**
        （ReentrantCallbackGroup）。rclpy 默认回调组是 MutuallyExclusive——
        服务回调（grasp_execute）阻塞运行（_run_flow_in_thread 内 join）期间，
        同组订阅回调被组互斥锁住、永不执行（_latest_gripper 恒为 None，夹爪
        判定恒"反馈无数据→failed"）；再配 MultiThreadedExecutor（grasp_executor
        main()）使该组的回调有可用线程。两者缺一不可（单测实测：
        独立组+单线程 executor / 默认组+多线程 executor 均饿死）。

        消息类型 sensor_msgs/JointState 延迟导入：无 ROS 环境（node=None）
        不订阅也不抛错。订阅失败 → 告警，判定按失败处理（无数据）。
        """
        try:
            from rclpy.callback_groups import ReentrantCallbackGroup
            from sensor_msgs.msg import JointState
            self._feedback_cb_group = ReentrantCallbackGroup()
            self._feedback_sub = self.node.create_subscription(
                JointState, FEEDBACK_TOPIC, self._feedback_cb, 10,
                callback_group=self._feedback_cb_group)
            self._log('info', f'已订阅 {FEEDBACK_TOPIC}（夹爪反馈，'
                              f'关节名 {GRIPPER_JOINT_FEEDBACK}，独立可重入回调组）')
        except Exception as exc:
            self._log('warn', f'订阅 {FEEDBACK_TOPIC} 失败（{exc}）：'
                              f'无夹爪反馈时判定按失败处理')

    def _feedback_cb(self, msg) -> None:
        """JointState 回调：缓存名字为 'gripper' 的关节位置（米制总开度）。

        C2 修复：驱动 PublishArmJointAndGripper 把 grippers_angle（sdk 文档：
        "stroke of the gripper (in 0.001 mm)"）除以 1e6 发布 ⇒ 该值已是米制
        指尖总开度（命令/反馈同为米制总开度，非弧度）。"""
        try:
            for name, pos in zip(msg.name, msg.position):
                if name == GRIPPER_JOINT_FEEDBACK:
                    with self._feedback_lock:
                        self._latest_gripper = float(pos)
                        self._latest_gripper_stamp = time.monotonic()
                    return
        except (AttributeError, TypeError, ValueError):
            pass  # 消息结构异常时静默丢弃，保持最近一次有效缓存

    def _read_gripper_feedback(self) -> float | None:
        """最近一次 gripper 反馈开度（米制总开度）；无订阅/无数据返回 None
        （调用方：判定视为失败）。"""
        with self._feedback_lock:
            return self._latest_gripper

    def _read_gripper_feedback_wait(self, timeout_s: float) -> float | None:
        """等待"新反馈"（M1 修复，配合 C1 的 MultiThreadedExecutor）。

        进入时记录最近反馈时刻，轮询至多 timeout_s 直到收到一条更新的 gripper
        反馈（米制总开度）并返回；超时返回最近缓存（无任何数据则 None）。
        避免用闭合前的旧开度判定（反馈流滞后时误判抓空）。
        """
        with self._feedback_lock:
            stamp_at_entry = self._latest_gripper_stamp
            latest = self._latest_gripper
        deadline = time.monotonic() + float(timeout_s)
        while True:
            with self._feedback_lock:
                if self._latest_gripper_stamp > stamp_at_entry:
                    return self._latest_gripper
                latest = self._latest_gripper
            if time.monotonic() >= deadline:
                return latest
            time.sleep(0.02)

    def _read_gripper_feedback_stable(
            self, timeout_s: float = FEEDBACK_STABLE_TIMEOUT_S,
            stable_window_s: float = FEEDBACK_STABLE_WINDOW_S,
            stable_tol_m: float = FEEDBACK_STABLE_TOL_M,
            min_samples: int = FEEDBACK_STABLE_MIN_SAMPLES) -> float | None:
        """等待命令后的新反馈进入稳定窗口，返回窗口中位数。

        只使用进入本方法之后到达的新样本；窗口时间跨度达到
        ``stable_window_s``、样本数达到 ``min_samples`` 且最大开度差不超过
        ``stable_tol_m`` 才算稳定。超时或无稳定窗口返回 None（fail-closed），
        绝不退回闭合途中的首条瞬态或命令前旧缓存。
        """
        timeout_s = max(float(timeout_s), 0.0)
        stable_window_s = max(float(stable_window_s), 0.0)
        stable_tol_m = max(float(stable_tol_m), 0.0)
        min_samples = max(int(min_samples), 2)
        # 与上游已验证真机链一致：SDK 对象只发送 GripperCtrl，反馈统一来自
        # 已 ConnectPort() 的 Piper ROS 驱动发布的 /joint_states_feedback。
        # 不从 self._piper 直读；该惰性命令对象没有启动 CAN 接收线程，其
        # GetArmGripperMsgs() 默认缓存为 0，曾导致 full_open/full_closed=0/0。
        with self._feedback_lock:
            entry_stamp = self._latest_gripper_stamp
        last_stamp = entry_stamp
        samples: list[tuple[float, float]] = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._feedback_lock:
                stamp = self._latest_gripper_stamp
                value = self._latest_gripper
            if stamp > last_stamp and value is not None and np.isfinite(value):
                last_stamp = stamp
                samples.append((stamp, float(value)))
                # 找“以最新样本结尾、振幅仍在容差内”的最长连续后缀。
                # 不能先按时间窗口裁剪：离散采样会让首末跨度总比窗口少
                # 一个周期，导致永远无法达到严格时间门限。
                stable_start = len(samples) - 1
                value_min = value_max = float(value)
                for idx in range(len(samples) - 2, -1, -1):
                    candidate = samples[idx][1]
                    next_min = min(value_min, candidate)
                    next_max = max(value_max, candidate)
                    if next_max - next_min > stable_tol_m:
                        break
                    value_min, value_max = next_min, next_max
                    stable_start = idx
                stable_samples = samples[stable_start:]
                if len(stable_samples) >= min_samples:
                    span = stable_samples[-1][0] - stable_samples[0][0]
                    if span >= stable_window_s:
                        return float(np.median(
                            [sample[1] for sample in stable_samples]))
            time.sleep(0.01)
        return None

    # ------------------------------------------------------------------
    # MoveIt 交互（占位，Task 9 干跑联调）
    # ------------------------------------------------------------------

    @staticmethod
    def _make_pose_msg(pose: dict):
        """geometry_msgs Pose 语义 dict（pos=[x,y,z], quat=[w,x,y,z]）→ Pose 消息。

        geometry_msgs 延迟导入；不可用时返回 None（fail-closed）。
        """
        try:
            from geometry_msgs.msg import Pose
        except ImportError:
            return None
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in pose['pos'])
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = \
            (float(v) for v in pose['quat'])
        return p

    def _tcp_pose_to_link6(self, pose: dict) -> dict:
        """TCP 位姿 → link6 位姿（I1 修复）。

        stage_poses 是 TCP 帧位姿（基座系）；MoveIt 规划端点改为 link6（arm 组
        tip，SRDF 核实；真机 URDF 无 grasp_tcp 帧）后，位姿目标须先折算：
        T_link6 = T_tcp · T(0,0,-0.1358)（右乘逆偏移，与
        Prechecker._tcp_to_link6_matrix 同一数学；0.1358m 为 URDF 权威值
        （joint7/joint8 origin z，06-urdf-tcp-authoritative-plan.md §1）。
        """
        tcp = Prechecker._pose_to_matrix(pose)
        link6 = Prechecker._tcp_to_link6_matrix(tcp)
        pos = link6[:3, 3]
        q = Rotation.from_matrix(link6[:3, :3]).as_quat()  # [x,y,z,w]
        return {'pos': [float(v) for v in pos],
                'quat': [float(q[3]), float(q[0]), float(q[1]), float(q[2])]}

    def _move_linear(self, pose) -> bool:
        """笛卡尔直线移动（approach/抬升段）。

        compute_cartesian_path（eef_step 0.01、avoid_collisions=True、规划失败
        回退 OMPL plan）；执行前 set_max_velocity_scaling_factor(velocity_scale)。

        注意：回退仅限**规划**失败（fraction < 1.0 或笛卡尔规划异常）——
        执行失败直接返回 False 不再重试（执行层失败可能是控制器/硬件问题，
        真机上盲目重试有安全风险）。

        占位（move_group=None 或接口未接线）：fail-closed 返回 False 并告警，
        Task 9 干跑联调接入真实 moveit_py MoveGroupInterface。
        """
        if self.move_group is None:
            self._log('warn', '_move_linear 占位（move_group=None）：fail-closed；'
                              'Task 9 干跑联调')
            return False
        # I1 修复：目标端点 = link6（arm 组 tip），TCP 位姿先折算回 link6
        pose_msg = self._make_pose_msg(self._tcp_pose_to_link6(pose))
        if pose_msg is None:
            self._log('warn', '_move_linear：位姿消息构造失败（geometry_msgs 不可用），'
                              'fail-closed')
            return False
        # 1) 笛卡尔规划（失败 → 回退 OMPL 规划）
        try:
            plan, fraction = self.move_group.compute_cartesian_path(
                [pose_msg], eef_step=CARTESIAN_EEF_STEP,
                jump_threshold=CARTESIAN_JUMP_THRESHOLD,
                avoid_collisions=CARTESIAN_AVOID_COLLISIONS)
        except Exception as exc:
            self._log('warn', f'compute_cartesian_path 未接线或调用失败（{exc}），'
                              f'回退 OMPL；Task 9 干跑联调')
            return self._move_pose(pose)
        if fraction < 1.0 - 1e-9:
            self._log('warn', f'compute_cartesian_path fraction={fraction:.3f} '
                              f'< 1.0（未走完整直线），回退 OMPL')
            return self._move_pose(pose)
        # 2) 执行（失败不回退重试，返回 False）
        try:
            self.move_group.set_max_velocity_scaling_factor(
                float(self.params['velocity_scale']))
            self.move_group.execute(plan)
            return True
        except Exception as exc:
            self._log('warn', f'笛卡尔直线执行失败（{exc}）：不重试，fail-closed；'
                              f'Task 9 干跑联调')
            return False

    def _move_pose(self, pose) -> bool:
        """OMPL plan+execute（预位/grasp 段）；执行前
        set_max_velocity_scaling_factor(velocity_scale)。

        占位（move_group=None 或接口未接线）：fail-closed 返回 False 并告警，
        Task 9 干跑联调接入真实 moveit_py MoveGroupInterface。
        """
        if self.move_group is None:
            self._log('warn', '_move_pose 占位（move_group=None）：fail-closed；'
                              'Task 9 干跑联调')
            return False
        # I1 修复：目标端点 = link6（arm 组 tip），TCP 位姿先折算回 link6
        pose_msg = self._make_pose_msg(self._tcp_pose_to_link6(pose))
        if pose_msg is None:
            self._log('warn', '_move_pose：位姿消息构造失败（geometry_msgs 不可用），'
                              'fail-closed')
            return False
        try:
            self.move_group.set_planning_time(float(self.params['planning_time_exec']))
            self.move_group.set_start_state_to_current_state()
            self.move_group.set_pose_target(pose_msg, end_effector_link=END_EFFECTOR_LINK)
            plan = self.move_group.plan()
            if plan is None:
                return False
            self.move_group.set_max_velocity_scaling_factor(
                float(self.params['velocity_scale']))
            return bool(self.move_group.execute(plan))
        except Exception as exc:
            self._log('warn', f'MoveIt plan+execute 未接线或调用失败（{exc}）：'
                              f'fail-closed；Task 9 干跑联调')
            return False

    def _gripper_set_width(self, cmd_opening_m: float, label: str) -> bool:
        """夹爪宽度设置（张开/闭合共用；SDK 直控 2026-08-07 用户裁决）。

        参考 piper_sdk demo/V2/piper_ctrl_gripper.py：直接调 GripperCtrl——
        range 单位 0.001mm、range = 总开度(米)×1e6；0x02 初始化 → 0x01 位置
        控制，200Hz 持续下发（绝对位置语义，重复下发无害）。绕开 MoveIt
        gripper 组 / mock / 驱动转发链路（驱动 gripper_forward=false）。
        张开目标 = 最大开度（URDF 总开度 0.07m=70000 单位，真机到物理限位停），
        不按物体宽度换算（用户裁决）。

        N1 修复（钳制与判定联动）：命令目标越限时直接 fail-closed 返回 False，
        不钳制后续跑（宽物体+margin 超行程时夹爪触不到，钳制会误报 success
        导致抬升掉件）。
        """
        opening_m = float(cmd_opening_m)
        if opening_m < 0.0 or opening_m > GRIPPER_MAX_OPENING_M:
            self._log('error',
                      f'物体宽度超出夹爪行程：cmd_opening={opening_m:.4f} m '
                      f'（SDK 小夹爪最大 {GRIPPER_MAX_OPENING_M:.3f} m），'
                      f'fail-closed 中止夹爪{label}（N1）')
            return False
        try:
            if self._piper is None:
                from piper_sdk import C_PiperInterface
                self._piper = C_PiperInterface('can0')
                # demo 初始化：0x02 失能清错 → 0x01 使能（位置控制）
                self._piper.GripperCtrl(0, 1000, 0x02, 0)
                self._piper.GripperCtrl(0, 1000, 0x01, 0)
            angle = opening_m_to_sdk_command(opening_m)
            t_end = time.time() + 1.0
            while time.time() < t_end:
                self._piper.GripperCtrl(angle, 1000, 0x01, 0)
                time.sleep(0.005)  # 200Hz（demo 节奏）
            self._log('info', f'夹爪{label}：SDK GripperCtrl({angle}, 1000, 0x01, 0) '
                              f'（总开度 {opening_m*1000:.1f} mm）')
            return True
        except Exception as exc:
            self._log('warn', f'夹爪{label} SDK 直控失败（{exc}）：fail-closed')
            return False

    def _open_gripper(self) -> bool:
        """张开夹爪到最大开度（2026-08-07 用户裁决：不按宽度换算，直接最大）。"""
        return self._gripper_set_width(GRIPPER_MAX_OPENING_M, '张开')

    def _close_gripper(self, cmd_opening_m: float) -> bool:
        """闭合夹爪到命令宽度（SDK 直控，见 _gripper_set_width）。"""
        return self._gripper_set_width(cmd_opening_m, '闭合')

    # ------------------------------------------------------------------
    # 核心 API
    # ------------------------------------------------------------------

    def _fail(self, stage: str, cmd_width=None, actual_width=None) -> dict:
        """失败返回（保持姿态，不再继续执行后续阶段）。

        I3 修复：统一回传 cmd_width（实际命令总开度，米），
        结果记录与内部判定用同一数值。
        """
        return {'status': 'failed', 'actual_width': actual_width, 'stage': stage,
                'cmd_width': cmd_width}

    def execute_pick(self, stage_poses: dict, grasp_width: float,
                     max_stage: str | None = None) -> dict:
        # Task 10 单段验证：max_stage 指定后仅执行到该段（open/pre_pose/
        # approach/grasp/gripper/lift），完成后返回 dry_stop（臂停在段末位姿，
        # 不执行后续段）。None = 完整序列。
        """执行完整抓取序列：
        1. 预位 → 2. approach 位（笛卡尔直线 compute_cartesian_path）→ 3. grasp 位（OMPL plan+execute）
        → 4. 闭合夹爪（C2：独立 gripper 组，joint7 目标 = (grasp_width + margin)/2 米）
        → 5. 等待新反馈（≤0.5s，M1）→ 读 gripper 反馈实际开度 → verify_grasp 判定
        → 6. 抬升已暂时关闭（2026-08-14 用户裁决：抬升目标同 pre_pose 不可达）；
           判定后直接返回，executor 收尾 joint-space 回 ready
        返回 dict: {'status': 'success'|'grasp_empty'|'failed',
                    'actual_width': float, 'stage': str(失败时的阶段名, 成功为''),
                    'cmd_width': float}（cmd_width = 实际命令总开度，米；I3 修复）
        stage_poses: Task 6 Prechecker.make_stage_poses 的产出
        {'approach': {'pos': [x,y,z], 'quat': [w,x,y,z]}, 'grasp': ..., 'lift': ...}
        （lift 是预检用的 open-lift 位姿；执行抬升目标 = grasp 位 + lift_z）

        Note（C2 修复后的单位事实，依据模块 docstring）:
            - 命令总开度 cmd_width = grasp_width + gripper_margin（米）；
              joint7 命令 = cmd_width / 2（prismatic 半行程，驱动 ×2 语义）。
            - 判定在"总开度域（米）"进行：反馈实际总开度 < cmd_width -
              gripper_detect_threshold ⇒ 夹住（success）；≈cmd_width ⇒ 抓空
              （grasp_empty）；无反馈数据（/joint_states_feedback 未订阅/未收到）
              ⇒ failed（不可判定）。命令与反馈同为米制总开度，无单位换算。
            - actual_width 单位与反馈一致（米制总开度）；失败于抬升前为 None，
              抬升段失败时为已测得的实际开度。
            - 本方法阻塞运行（含 0.5s 静置），调用方（Task 8 服务回调）
              需在后台线程执行（配合 MultiThreadedExecutor，C1 修复）。
        """
        for key in ('approach', 'grasp', 'lift'):
            if key not in stage_poses:
                self._log('error', f'stage_poses 缺少 {key} 段，中止执行')
                return self._fail('setup')

        try:
            cmd_width = required_opening_m(
                float(grasp_width), float(self.params['gripper_margin']))
        except (TypeError, ValueError) as exc:
            self._log('error', f'候选夹爪宽度非法（{exc}），执行前 fail-closed')
            return self._fail('gripper_width')
        if not fits_small_gripper(grasp_width, self.params['gripper_margin']):
            self._log(
                'error',
                f'候选所需开口超出小夹爪行程：grasp_width={float(grasp_width):.4f} m, '
                f'margin={float(self.params["gripper_margin"]):.4f} m, '
                f'required={cmd_width:.4f} m > {GRIPPER_MAX_OPENING_M:.3f} m，'
                '执行前 fail-closed')
            return self._fail('gripper_width', cmd_width)
        self._log('info', f'execute_pick: grasp_width={float(grasp_width):.4f} m, '
                          f'cmd_width={cmd_width:.4f} m（总开度；joint7 命令 = '
                          f'{joint7_cmd_for_opening(cmd_width):.4f} m 半行程）')

        # Task 10 单段验证截断（max_stage 段完成后停，返回 dry_stop）
        _seq = ['open', 'pre_pose', 'approach', 'grasp', 'gripper', 'lift']
        _stop_idx = _seq.index(max_stage) if max_stage in _seq else len(_seq)

        def _dry_stop():
            return {'status': 'dry_stop', 'actual_width': None,
                    'stage': str(max_stage), 'cmd_width': cmd_width}

        # 0) 张开夹爪到最大（2026-08-07 用户裁决：不按宽度换算；SDK 直控）
        if not self._open_gripper():
            return self._fail('gripper_open', cmd_width)
        if 0 >= _stop_idx:
            return _dry_stop()

        # 0.5) 判定参考标定：每次执行前实测全开/全闭。
        full_open = self._read_gripper_feedback_stable()
        if full_open is None:
            self._log('warn', '全开反馈在超时内未稳定，标定 fail-closed')
            return self._fail('gripper_calib', cmd_width)
        if not self._gripper_set_width(0.0, '标定闭合'):
            return self._fail('gripper_calib', cmd_width)
        full_closed = self._read_gripper_feedback_stable()
        if full_closed is None:
            self._log('warn', '全闭反馈在超时内未稳定，标定 fail-closed')
            return self._fail('gripper_calib', cmd_width)
        if not self._open_gripper():
            return self._fail('gripper_open', cmd_width)
        if not gripper_calibration_is_valid(full_open, full_closed):
            self._log(
                'error',
                f'夹爪参考标定无效：full_open={full_open*1000:.1f}mm, '
                f'full_closed={full_closed*1000:.1f}mm, '
                f'span={(full_open-full_closed)*1000:.1f}mm；'
                f'要求 full_open={GRIPPER_CALIB_OPEN_MIN_M*1000:.0f}..'
                f'{GRIPPER_CALIB_OPEN_MAX_M*1000:.0f}mm、'
                f'full_closed={GRIPPER_CALIB_CLOSED_MIN_M*1000:.0f}..'
                f'{GRIPPER_CALIB_CLOSED_MAX_M*1000:.0f}mm、'
                f'span>={GRIPPER_CALIB_MIN_SPAN_M*1000:.0f}mm，fail-closed')
            return self._fail('gripper_calib', cmd_width)
        self._log('info', f'判定参考标定：full_open={full_open*1000:.1f}mm '
                          f'full_closed={full_closed*1000:.1f}mm')

        # 1) 预位：2026-08-14 用户裁决关闭——lift 位姿（grasp + 10cm 同姿态）
        #    系统性超出可达域（预检只查 approach+grasp，lift 是盲区），pre_pose
        #    规划必失败（OMPL "Unable to sample any valid states"）。直接从当前
        #    位姿走 approach 线性段（失败自动回退 OMPL）。
        #    max_stage='pre_pose' 截断语义保留（无运动，直接 dry_stop）。
        if 1 >= _stop_idx:
            return _dry_stop()

        # 2) approach 位：笛卡尔直线（失败自动回退 OMPL）
        if not self._move_linear(stage_poses['approach']):
            return self._fail('approach', cmd_width)
        if 2 >= _stop_idx:
            return _dry_stop()

        # 3) grasp 位：OMPL plan+execute
        if not self._move_pose(stage_poses['grasp']):
            return self._fail('grasp', cmd_width)
        if 3 >= _stop_idx:
            return _dry_stop()

        # 4) 闭合夹爪：完全闭合（2026-08-08 用户裁决——判定基于全闭理论值，
        #    不依赖候选宽度；物体挡在行程内时夹爪停在物体宽处）
        if not self._close_gripper(0.0):
            return self._fail('gripper', cmd_width)

        # 5) 等待连续稳定反馈 → 初判（总开度域，米）。第一条新反馈可能是
        # 闭合途中的瞬态，不能用于业务判定。
        actual_width = self._read_gripper_feedback_stable()
        if actual_width is None:
            self._log('warn', '夹爪反馈在超时内未形成稳定窗口，判定失败并保持姿态')
            return self._fail('gripper', cmd_width)
        status, detail = classify_grasp(actual_width, full_open, full_closed)
        self._log('info', f'grasp verdict: actual_width={actual_width:.4f} m '
                          f'（总开度，闭合命令=完全闭合）, '
                          f'full_open={full_open:.4f}, full_closed={full_closed:.4f}, '
                          f'detail={detail} → {status}')

        # 初判抓住后，在 grasp 姿态继续保持一个完整稳定窗口并二次分类。
        # 当前 lift 暂停，至少用保持复核阻止瞬态假阳性写入 success。
        if status == 'success':
            held_width = self._read_gripper_feedback_stable()
            if held_width is None:
                self._log('warn', '抓住初判后保持复核无稳定反馈，fail-closed')
                return self._fail('gripper_hold', cmd_width)
            held_status, held_detail = classify_grasp(
                held_width, full_open, full_closed)
            self._log('info', f'grasp hold verdict: actual_width={held_width:.4f} m, '
                              f'detail={held_detail} → {held_status}')
            actual_width = held_width
            status = held_status

        # 6) 抬升暂时关闭；判定后直接返回，由 executor 收尾回 zero。
        return {'status': status, 'actual_width': actual_width, 'stage': '',
                'cmd_width': cmd_width}
