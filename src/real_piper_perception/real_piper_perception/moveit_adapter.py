"""moveit_py MoveGroupInterface 的 action/service 适配实现（2026-08-07）。

humble 无 moveit_py / moveit_commander（apt 与 pip 均无）→ 手写适配器，
直接调用 move_group 节点的 action/service（moveit_msgs）：
  - /move_group              (moveit_msgs/action/MoveGroup)          规划
  - /execute_trajectory      (moveit_msgs/action/ExecuteTrajectory)  执行
  - /compute_cartesian_path  (moveit_msgs/srv/GetCartesianPath)      笛卡尔直线
  - /get_current_state       (moveit_msgs/srv/GetCurrentState)       当前关节

实现 grasp_execute.py / grasp_precheck.py 实际用到的 moveit_py snake_case
方法子集（鸭子类型注入，调用侧零改动）：
  get_current_joint_values / get_planning_time / set_planning_time /
  set_start_state_to_current_state / set_joint_value_target(list|dict) /
  set_pose_target / plan / execute / compute_cartesian_path /
  set_max_velocity_scaling_factor
"""
from __future__ import annotations

import time

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

# SRDF：arm 组 = base_link→link6（关节 joint1..joint6）；gripper 组 = gripper_base→link7
_GROUP_JOINTS = {
    'arm': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    'gripper': ['joint7'],
}

# 规划/执行 action 等待超时（秒）；move_group 默认规划 10s，留足余量
_ACTION_TIMEOUT_S = 60.0
_SERVICE_TIMEOUT_S = 10.0

def _make_box_primitive(x: float, y: float, z: float) -> SolidPrimitive:
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(x), float(y), float(z)]
    return box


class _PlanResult:
    """compute_cartesian_path 返回的 plan 对象：调用侧只需 .trajectory。"""

    __slots__ = ('trajectory',)

    def __init__(self, trajectory):
        self.trajectory = trajectory


class MoveGroupInterface:
    """moveit_py MoveGroupInterface 兼容层（move_group action/service 后端）。"""

    def __init__(self, node, group_name: str, base_frame: str = 'base_link',
                 end_effector_link: str | None = None):
        self._node = node
        self._group_name = group_name
        self._end_effector_link = end_effector_link
        self._joint_names = list(_GROUP_JOINTS.get(group_name, []))
        self._planning_time = 5.0
        self._velocity_scale = 1.0
        self._start_state: RobotState | None = None
        self._joint_target: dict[str, float] | None = None
        self._pose_target = None  # (Pose, eef_link)
        self._joint_state: JointState | None = None  # /joint_states 最新缓存

        # MoveIt2 humble 的 move_group 节点 action server 名是 /move_action
        #（MoveGroupMoveAction capability；/move_group 是旧名，servers=0）
        # 2026-08-07：所有异步组件（action/service/订阅）放入独立 Reentrant
        # 回调组——避免与 node 默认 MutuallyExclusive 组串行导致的 action
        # goal response 回调延迟（executor 干跑实测 10s 超时，MTE 下仍可能）。
        self._cb_group = ReentrantCallbackGroup()
        self._mg_action = ActionClient(node, MoveGroup, '/move_action',
                                       callback_group=self._cb_group)
        self._exec_action = ActionClient(node, ExecuteTrajectory,
                                         '/execute_trajectory',
                                         callback_group=self._cb_group)
        self._cart_service = node.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self._cb_group)
        # moveit_msgs 无 GetCurrentState 服务：订阅 /joint_states 缓存
        #（与 move_group 的 planning scene monitor 数据同源）
        node.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 1,
                                 callback_group=self._cb_group)

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_state = msg

    def _wait_future(self, future, timeout_s: float) -> bool:
        """轮询等待 rclpy Future 完成（不依赖当前线程 spin——MTE 回调线程安全；
        rclpy Future.result() 不支持 timeout 关键字）。异常视为失败。"""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                self._node.get_logger().warn(
                    f'action future 等待超时（{timeout_s}s）：{type(future).__name__}')
                return False
            time.sleep(0.05)
        return future.exception() is None

    def _joint_value(self, name: str) -> float:
        if self._joint_state is None:
            raise RuntimeError('/joint_states 尚无数据（订阅缓存为空）')
        names = list(self._joint_state.name)
        if name not in names:
            raise KeyError(f'/joint_states 中无关节 {name}')
        return float(self._joint_state.position[names.index(name)])

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------

    def get_current_joint_values(self) -> list[float]:
        """当前关节角（arm 组 joint1..6 顺序 / gripper 组 joint7）。"""
        return [self._joint_value(name) for name in self._joint_names]

    def get_planning_time(self) -> float:
        return self._planning_time

    def set_planning_time(self, seconds: float) -> None:
        self._planning_time = float(seconds)

    def set_max_velocity_scaling_factor(self, factor: float) -> None:
        self._velocity_scale = float(factor)

    def set_start_state_to_current_state(self) -> None:
        """从 /joint_states 缓存构造 RobotState 显式作为规划起点。"""
        if self._joint_state is None:
            self._start_state = None  # 无缓存 → 让 move_group 用自身状态
            return
        state = RobotState()
        state.joint_state.name = list(self._joint_state.name)
        state.joint_state.position = list(self._joint_state.position)
        state.joint_state.velocity = list(self._joint_state.velocity)
        state.joint_state.effort = list(self._joint_state.effort)
        self._start_state = state

    # ------------------------------------------------------------------
    # 目标设定
    # ------------------------------------------------------------------

    def set_joint_value_target(self, target) -> bool:
        """list（按组关节顺序）或 dict {关节名: 值}。返回 True 表示已接受。"""
        if isinstance(target, dict):
            self._joint_target = {k: float(v) for k, v in target.items()}
        else:
            values = [float(v) for v in target]
            if len(values) != len(self._joint_names):
                raise ValueError(
                    f'{self._group_name} 组需要 {len(self._joint_names)} 个关节值，'
                    f'收到 {len(values)}')
            self._joint_target = dict(zip(self._joint_names, values))
        self._pose_target = None
        return True

    def set_pose_target(self, pose, end_effector_link: str | None = None) -> bool:
        self._pose_target = (pose, end_effector_link or self._end_effector_link)
        self._joint_target = None
        return True

    # ------------------------------------------------------------------
    # 规划 / 执行
    # ------------------------------------------------------------------

    def plan(self):
        """MoveGroup action 规划。返回 MoveGroup.Result.plan（含 .trajectory）；
        失败返回 None（调用侧 bool(plan) 判空）。"""
        request = MotionPlanRequest()
        request.group_name = self._group_name
        request.planner_id = ''
        request.num_planning_attempts = 1
        request.allowed_planning_time = self._planning_time
        request.max_velocity_scaling_factor = self._velocity_scale
        request.max_acceleration_scaling_factor = 1.0
        if self._start_state is not None:
            request.start_state = self._start_state
        if self._joint_target is not None:
            constraints = Constraints()
            for name, value in self._joint_target.items():
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = value
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            request.goal_constraints.append(constraints)
        elif self._pose_target is not None:
            pose, eef_link = self._pose_target
            constraints = Constraints()
            # MoveIt2 无 PoseConstraint：位置 + 姿态两段约束（MoveIt1 兼容层）
            posc = PositionConstraint()
            posc.header.frame_id = 'base_link'  # ⚠ 2026-08-07 修复：缺 frame 导致
            # 约束区域在错误坐标系解释，规划终点 z 偏差 22cm（实测）
            posc.link_name = eef_link or ''
            posc.target_point_offset.x = 0.0
            posc.target_point_offset.y = 0.0
            posc.target_point_offset.z = 0.0
            posc.constraint_region.primitive_poses.append(pose)
            posc.constraint_region.primitives.append(
                _make_box_primitive(0.02, 0.02, 0.02))  # ±1cm 位置容差
            posc.weight = 1.0
            constraints.position_constraints.append(posc)
            oric = OrientationConstraint()
            # PositionConstraint 与 OrientationConstraint 必须使用同一基座坐标系。
            # 未设置时 MoveIt 会将姿态约束的 frame_id 留空，运行时出现
            # "No frame specified" / getFrameTransform() 警告，导致终点约束语义
            # 不可追溯。
            oric.header.frame_id = 'base_link'
            oric.link_name = eef_link or ''
            oric.absolute_x_axis_tolerance = 0.05
            oric.absolute_y_axis_tolerance = 0.05
            oric.absolute_z_axis_tolerance = 0.05
            oric.orientation = pose.orientation
            oric.weight = 1.0
            constraints.orientation_constraints.append(oric)
            request.goal_constraints.append(constraints)
        else:
            raise RuntimeError('plan() 前未设置关节/位姿目标')

        goal = MoveGroup.Goal()
        goal.request = request
        # ⚠ 2026-08-07 安全修复：MoveGroup.Goal.planning_options.plan_only 默认
        # False = "规划即执行"！此前未设置导致 plan() 实际执行轨迹（速度无缩放）
        # → 高速驱动真机 → 轨迹失控触发保护失能。plan() 必须显式 plan_only=True，
        # 执行只走 execute()（/execute_trajectory，明确语义）。
        goal.planning_options.plan_only = True
        if not self._mg_action.wait_for_server(timeout_sec=_SERVICE_TIMEOUT_S):
            raise RuntimeError('/move_group action server 不可用')
        # humble rclpy：send_goal 是阻塞式（等待完整结果，禁止在回调里用）——
        # 必须用 send_goal_async + future.result(timeout)（回调由调用方 executor
        # 的其余线程处理，MTE 服务回调线程 / 测试脚本 spin 线程均可）
        send_future = self._mg_action.send_goal_async(goal)
        if not self._wait_future(send_future, _SERVICE_TIMEOUT_S):
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, _ACTION_TIMEOUT_S):
            return None
        # get_result_async 的 future 返回 GetResult_Response 包装，.result 才是
        # MoveGroup.Result（humble rclpy 行为）
        result = result_future.result().result
        # MoveGroup.Result.error_code.val：SUCCESS=1
        if result.error_code.val != 1:
            return None
        # Result 轨迹字段为 planned_trajectory（RobotTrajectory）；
        # 包装成带 .trajectory 的对象与调用侧契约一致
        return _PlanResult(result.planned_trajectory)

    def execute(self, plan) -> bool:
        if plan is None or plan.trajectory is None:
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan.trajectory
        if not self._exec_action.wait_for_server(timeout_sec=_SERVICE_TIMEOUT_S):
            raise RuntimeError('/execute_trajectory action server 不可用')
        # 同 plan()：humble send_goal 阻塞式，必须用 send_goal_async + 轮询等待
        send_future = self._exec_action.send_goal_async(goal)
        if not self._wait_future(send_future, _SERVICE_TIMEOUT_S):
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, _ACTION_TIMEOUT_S):
            return False
        # 同 plan()：GetResult_Response 包装，.result 才是 ExecuteTrajectory.Result
        result = result_future.result().result
        return bool(result is not None and result.error_code.val == 1)

    def compute_cartesian_path(self, poses, eef_step: float = 0.01,
                               jump_threshold: float = 0.0,
                               avoid_collisions: bool = True):
        """笛卡尔直线规划。返回 (plan(含 .trajectory), fraction)。"""
        if not self._cart_service.wait_for_service(timeout_sec=_SERVICE_TIMEOUT_S):
            raise RuntimeError('/compute_cartesian_path service 不可用')
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = self._group_name
        req.link_name = self._end_effector_link
        if self._start_state is not None:
            req.start_state = self._start_state
        req.waypoints = list(poses)
        # ROS2 Humble moveit_msgs/srv/GetCartesianPath 使用 max_step；
        # eef_step 是其他 MoveIt API 的参数名，Request 消息没有该字段。
        req.max_step = float(eef_step)
        req.jump_threshold = float(jump_threshold)
        req.avoid_collisions = bool(avoid_collisions)
        future = self._cart_service.call_async(req)
        deadline = time.monotonic() + _SERVICE_TIMEOUT_S
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError('compute_cartesian_path 超时')
            time.sleep(0.05)
        resp = future.result()
        if resp is None:
            raise RuntimeError('compute_cartesian_path 返回空响应')
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'compute_cartesian_path 失败：error_code={resp.error_code.val}, '
                f'fraction={float(resp.fraction):.3f}')
        # ROS 2 Humble moveit_msgs/srv/GetCartesianPath 的响应字段是
        # ``solution``，不是 MoveIt 旧包装中常见的 ``trajectory``。
        trajectory = resp.solution
        joint_points = list(trajectory.joint_trajectory.points)
        multi_dof_points = list(trajectory.multi_dof_joint_trajectory.points)
        if not joint_points and not multi_dof_points:
            raise RuntimeError(
                f'compute_cartesian_path 返回空轨迹：fraction={float(resp.fraction):.3f}')
        return (_PlanResult(trajectory), float(resp.fraction))
