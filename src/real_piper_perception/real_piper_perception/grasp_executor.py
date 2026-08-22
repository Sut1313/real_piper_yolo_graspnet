#!/usr/bin/env python3
"""grasp_executor 主节点（Task 8）：服务/键盘触发 + 全流程编排 + 结果落盘/广播。

编排（brief 服务回调步骤 1-5 原样实现）：

1. 读 csv_path 最近文件 → parse_grasp_csv
2. rank_candidates → 逐候选（≤ graspnet_6d_max_candidates）：
   self_consistency 过滤 → camera_pose_to_base + graspnet_to_tcp_rotation
   （变换链由 Prechecker.make_stage_poses 内部完成）→ precheck_stages
   → 首个全过候选选中
3. scan_only=true：返回 {success=true, message=scan 统计}（不落盘不广播）；
   否则 GraspExecutor.execute_pick(stage_poses, width) → 判定结果
4. 结果写 shared_dir/grasp_result.json：
   {timestamp, success, status, stage, actual_width, cmd_width,
    candidate_index, message}
5. 发布 /grasp_result（std_msgs/String，JSON 字符串）

键盘触发：后台 daemon 线程 input() 读 'g' 触发同一流程回调；
无 TTY（stdin EOF / OSError）时静默退出。

scan_only 参数：节点默认 false；launch 文件默认 true（只做预检 dry-run、
臂不动）——切 false 需现场二次确认（真机抓取执行）。

依赖上游模块（接口按 brief 原样调用，不臆造）：
- grasp_candidates.py（Task 4）：parse_grasp_csv / rank_candidates / self_consistency
- grasp_transform.py（Task 3）：load_handeye
- grasp_precheck.py（Task 6）：Prechecker.make_stage_poses / precheck_stages
- grasp_execute.py（Task 7，并行代理）：GraspExecutor.execute_pick
"""

import json
import os
import select
import sys
import threading
import time
import traceback
from datetime import datetime
from glob import glob

import rclpy
from scipy.spatial.transform import Rotation
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionFK
from std_srvs.srv import Trigger

from real_piper_perception.grasp_candidates import (
    parse_grasp_csv,
    rank_candidates,
    build_precheck_results,
    self_consistency,
)
from real_piper_perception.grasp_execute import END_EFFECTOR_LINK, GraspExecutor
from real_piper_perception.grasp_precheck import Prechecker
from real_piper_perception.grasp_transform import load_handeye

# 2026-08-14 用户裁决：预检规划与真机抓取以 zero 为初始位姿，收尾也回 zero
# （取代 2026-08-10 的 ready 语义）。zero = SRDF <group_state name="zero"
# group="arm"> 的关节值（piper.srdf，与 scripts/moveit_go_ready.py 同源）。
# 注：ready/zero 起点对预检结果无差异（2026-08-14 受控 A/B 实测，逐候选 0 不一致）。
ZERO_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class GraspExecutorNode(Node):
    """主节点：grasp_execute 服务触发完整抓取流程，结果落盘 + 广播。"""

    def __init__(self, **kwargs):
        # **kwargs 透传给 rclpy Node（集成测试用 parameter_overrides 覆盖默认参数）
        super().__init__('grasp_executor', **kwargs)

        # ------------------------------------------------------------------
        # 参数声明（brief 参数清单，默认值原样；handeye_yaml 用 camera_data 真实文件）
        # ------------------------------------------------------------------
        self.declare_parameter('velocity_scale', 0.1)
        # scan_only: 节点默认 false；launch 默认 true（仅预检 dry-run，臂不动）。
        # 切 false 需现场二次确认（真机抓取执行）。
        self.declare_parameter('scan_only', False)
        self.declare_parameter('dry_run_max_stage', '')
        self.declare_parameter(
            'handeye_yaml',
            'camera_data/handeye_eye_to_hand.yaml')
        self.declare_parameter(
            'shared_dir',
            'shared_camera_data')
        self.declare_parameter('csv_path', '')  # 空 → <shared_dir>/perception_grasps.csv
        self.declare_parameter('approach_clearance', 0.05)
        self.declare_parameter('lift_z', 0.10)
        self.declare_parameter('open_gripper_lift_height', 0.10)
        # 2026-08-08 用户裁决：grasp 位 z 标定修正（FK 比对相机位置后设定）
        self.declare_parameter('grasp_z_offset', 0.02)

        self.declare_parameter('gripper_margin', 0.005)
        self.declare_parameter('gripper_detect_threshold', 0.01)
        # I2 修复：默认候选数 1 → 50（真实 CSV 37 候选、rank[0] 门控漂移时
        # 默认配置曾导致 tested=0，流程永不执行；设计上限 300，M4 钳制）
        # 2026-08-14 用户裁决：50 → 150（扩大被检候选集，缓解"高分候选
        # 不可达 + 截断点挡住低分可达候选"的耦合）
        self.declare_parameter('graspnet_6d_max_candidates', 150)
        # 2026-08-07 用户裁决：只接受 score >= 阈值的候选（默认 0.45）
        # 2026-08-07 用户裁决：阈值关闭（默认 0 = 不过滤）
        self.declare_parameter('grasp_score_min', 0.0)
        # 2026-08-14 用户裁决：self-consistency 门控可关（实验用；默认开）。
        # 关闭后所有候选直接进入预检（门控与可达性无关，仅过滤帧内错位候选）。
        self.declare_parameter('enable_self_consistency', True)
        # 仿真朝向变体筛选参数；本编排流程（brief 步骤 1-5）未使用，按参数清单保留声明
        self.declare_parameter('max_orientation_variants', 2)
        self.declare_parameter('enable_piper_analytic_ik_planning', False)
        # 2026-08-07 回退：旧映射（水平接近=夹爪朝下抓）下 ssik 有解，
        # 恢复原设计默认 true（analytic-only 门控）
        self.declare_parameter('graspnet_6d_analytic_only_precheck', False)
        self.declare_parameter('piper_analytic_ik_script', self._default_ssik_script_path())
        self.declare_parameter('piper_analytic_ik_max_solutions', 8)
        self.declare_parameter('piper_analytic_ik_max_solution_plans', 6)
        self.declare_parameter('planning_time_precheck', 0.5)
        self.declare_parameter('planning_time_exec', 10.0)

        # ------------------------------------------------------------------
        # 读取参数
        # ------------------------------------------------------------------
        self._velocity_scale = float(self.get_parameter('velocity_scale').value)
        self._scan_only = bool(self.get_parameter('scan_only').value)
        self._dry_run_max_stage = self.get_parameter('dry_run_max_stage').get_parameter_value().string_value
        self._handeye_yaml = str(self.get_parameter('handeye_yaml').value)
        self._shared_dir = str(self.get_parameter('shared_dir').value)
        self._csv_path = str(self.get_parameter('csv_path').value)
        self._approach_clearance = float(self.get_parameter('approach_clearance').value)
        self._lift_z = float(self.get_parameter('lift_z').value)
        self._grasp_z_offset = float(self.get_parameter('grasp_z_offset').value)

        self._open_gripper_lift_height = float(
            self.get_parameter('open_gripper_lift_height').value)
        self._gripper_margin = float(self.get_parameter('gripper_margin').value)
        self._gripper_detect_threshold = float(
            self.get_parameter('gripper_detect_threshold').value)
        # M4 修复：graspnet_6d_max_candidates 钳制到设计上限 300（防误配置大值
        # 长时间空转：每候选 × 3 段 × ssik 子进程 30s 超时）
        self._grasp_score_min = float(self.get_parameter('grasp_score_min').value)
        self._enable_self_consistency = bool(
            self.get_parameter('enable_self_consistency').value)
        self._max_candidates = max(1, min(
            int(self.get_parameter('graspnet_6d_max_candidates').value), 300))
        if int(self.get_parameter('graspnet_6d_max_candidates').value) > 300:
            self.get_logger().warn(
                f'graspnet_6d_max_candidates={self.get_parameter("graspnet_6d_max_candidates").value} '
                f'超设计上限 300，已钳制为 300')
        self._piper_analytic_ik_script = str(self.get_parameter('piper_analytic_ik_script').value)

        # ------------------------------------------------------------------
        # 服务 + 结果发布器
        # ------------------------------------------------------------------
        self._srv = self.create_service(Trigger, 'grasp_execute', self._on_grasp_execute)
        # 夹爪朝向 FK 验证 client（2026-08-07）：独立 Reentrant 组——默认
        # MutuallyExclusive 组下 wait_for_service/call 会超时（与适配器同根源）
        self._fk_client = self.create_client(
            GetPositionFK, '/compute_fk',
            callback_group=ReentrantCallbackGroup())
        self._result_pub = self.create_publisher(String, 'grasp_result', 10)

        # ------------------------------------------------------------------
        # 依赖模块装配（接口签名按 brief 原样）
        # ------------------------------------------------------------------
        self._handeye = load_handeye(self._handeye_yaml)  # 4x4 T_base_camera
        move_group, gripper_move_group = self._create_move_group()

        precheck_params = {
            'enable_piper_analytic_ik_planning': bool(
                self.get_parameter('enable_piper_analytic_ik_planning').value),
            'graspnet_6d_analytic_only_precheck': bool(
                self.get_parameter('graspnet_6d_analytic_only_precheck').value),
            'piper_analytic_ik_max_solutions': int(
                self.get_parameter('piper_analytic_ik_max_solutions').value),
            'piper_analytic_ik_max_solution_plans': int(
                self.get_parameter('piper_analytic_ik_max_solution_plans').value),
            'planning_time_precheck': float(
                self.get_parameter('planning_time_precheck').value),
        }
        self._prechecker = Prechecker(
            node=self, move_group=move_group,
            ssik_script=self._piper_analytic_ik_script, params=precheck_params)

        exec_params = {
            'velocity_scale': self._velocity_scale,
            'gripper_margin': self._gripper_margin,
            'gripper_detect_threshold': self._gripper_detect_threshold,
            'lift_z': self._lift_z,
            'planning_time_exec': float(self.get_parameter('planning_time_exec').value),
        }
        # C2 修复：夹爪闭合走独立 gripper 组（gripper_move_group）
        self._executor = GraspExecutor(
            node=self, move_group=move_group, params=exec_params,
            gripper_move_group=gripper_move_group)

        # 并发触发互斥（服务 + 键盘可同时触发同一流程）
        self._flow_lock = threading.Lock()
        self._keyboard_thread: threading.Thread | None = None

        self.get_logger().info(
            'grasp_executor started:\n'
            f'  scan_only:      {self._scan_only}\n'
            f'  handeye_yaml:   {self._handeye_yaml}\n'
            f'  shared_dir:     {self._shared_dir}\n'
            f'  csv_path:       {self._resolve_csv_path()}\n'
            f'  velocity_scale: {self._velocity_scale}\n'
            f'  max_candidates: {self._max_candidates}')

    # ------------------------------------------------------------------
    # 装配辅助
    # ------------------------------------------------------------------

    @staticmethod
    def _default_ssik_script_path() -> str:
        """默认 piper_analytic_ik_script：包内 scripts/piper_ssik_ik.py 绝对路径。

        由本模块文件位置推导（symlink-install / egg-link 安装均指向源目录）。
        """
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_root, 'scripts', 'piper_ssik_ik.py')

    def _create_move_group(self):
        """创建 MoveIt MoveGroupInterface，返回 (arm_mg, gripper_mg)。

        C2 修复：夹爪闭合必须用独立 gripper 组（piper.srdf：arm 组 = joint1-6、
        gripper 组 = gripper_base→link7 仅 joint7），故 arm 与 gripper 各建一个
        MoveGroupInterface。优先 moveit_py；humble 无 moveit_py/moveit_commander
        （2026-08-07 实测），回退本地 action/service 适配器 moveit_adapter.py
        （Task 9 干跑联调接线：/move_group 规划 + /execute_trajectory 执行）。
        """
        try:
            from moveit_py.core import MoveGroupInterface
        except Exception as exc:
            self.get_logger().info(
                f'moveit_py 不可用（{exc}）：改用 moveit_adapter（move_group '
                f'action/service 后端）')
            try:
                from real_piper_perception.moveit_adapter import MoveGroupInterface
            except Exception as exc2:
                self.get_logger().warn(
                    f'moveit_adapter 导入失败（{exc2}）：move_group=None，'
                    f'预检/执行 fail-closed')
                return None, None
        arm_mg, gripper_mg = None, None
        try:
            # move group 名按 Task 1 调研 kMoveGroupName="arm"；API 细节 Task 9 实测核对
            arm_mg = MoveGroupInterface(
                node=self, group_name='arm', end_effector_link=END_EFFECTOR_LINK)
            self.get_logger().info('MoveIt MoveGroupInterface("arm") 就绪')
        except Exception as exc:
            self.get_logger().warn(f'MoveGroupInterface("arm") 创建失败（{exc}）')
        try:
            # C2：gripper 组（piper.srdf group name="gripper"，仅 joint7）
            gripper_mg = MoveGroupInterface(
                node=self, group_name='gripper', end_effector_link='link7')
            self.get_logger().info('MoveIt MoveGroupInterface("gripper") 就绪')
        except Exception as exc:
            self.get_logger().warn(f'MoveGroupInterface("gripper") 创建失败（{exc}）')
        return arm_mg, gripper_mg

    def _resolve_csv_path(self) -> str:
        """csv_path 参数 → 实际 csv 文件路径（"读最近文件"语义）。

        参数为空 → <shared_dir>/perception_grasps.csv（感知侧每次覆盖写入的
        最新输出）；参数指向目录 → 取目录内最近修改的 *.csv。
        """
        p = (self._csv_path if self._csv_path
             else os.path.join(self._shared_dir, 'perception_grasps.csv'))
        if os.path.isdir(p):
            try:
                files = sorted(glob(os.path.join(p, '*.csv')),
                               key=os.path.getmtime, reverse=True)
            except OSError:
                files = []
            if files:
                return files[0]
            return p  # 目录内无 csv：调用方报文件缺失
        return p

    # ------------------------------------------------------------------
    # 触发入口
    # ------------------------------------------------------------------

    def _on_grasp_execute(self, request, response):
        """grasp_execute 服务回调（std_srvs/Trigger）。

        GraspExecutor.execute_pick 阻塞运行（Task 7 docstring：调用方需在后台
        线程执行）——故流程在独立后台线程执行、本回调 join 等待，保持 brief
        步骤 3 的同步响应语义（响应携带报告）。
        C1 修复：节点用 MultiThreadedExecutor spin，join 等待期间其他线程仍处理
        /joint_states_feedback 订阅回调（反馈不饿死，夹爪判定可达）。
        """
        report = self._run_flow_in_thread(trigger='service')
        response.success = report['success']
        response.message = report['message']
        return response

    def _run_flow_in_thread(self, trigger: str) -> dict:
        """在后台线程执行流程并等待结果（服务回调专用；键盘线程直接调 _run_flow）。"""
        box: dict = {}

        def worker() -> None:
            box['report'] = self._run_flow(trigger=trigger)

        # M8（deferred）：daemon 线程在节点关闭（Ctrl+C）时被强杀，流程中断后
        # move_group.execute 侧轨迹由控制器继续执行完、臂不会立即停——行为未定义。
        # 真机联调（Task 10）前捕获流程中段的 shutdown 信号并记录"执行中断"。
        thread = threading.Thread(target=worker, daemon=True, name=f'grasp_flow_{trigger}')
        thread.start()
        thread.join()
        return box.get('report', {'success': False, 'message': 'grasp flow thread died'})

    def _keyboard_listener(self):
        """后台 daemon 线程：stdin 读 'g' 触发同一流程。

        select 超时轮询而非阻塞 input()：daemon 线程在解释器关闭时不得持有
        stdin 缓冲区锁（否则 _enter_buffered_busy 致命错误，launch 下 stdin
        是常开管道会无限阻塞）。无 TTY / stdin 关闭 → 静默退出（不告警不阻断）。
        """
        if sys.stdin is None:
            return
        while True:
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            except (OSError, ValueError):
                return  # stdin 不可用（无 TTY / 已关闭）
            if not ready:
                continue
            try:
                line = sys.stdin.readline()
            except OSError:
                return
            if line == '':
                return  # EOF（无 TTY 时静默退出）
            if line.strip().lower() == 'g':
                self.get_logger().info("keyboard trigger: 'g' -> grasp flow")
                self._run_flow(trigger='keyboard')

    def start_keyboard_thread(self) -> None:
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener, name='grasp_keyboard', daemon=True)
        self._keyboard_thread.start()

    # ------------------------------------------------------------------
    # 全流程编排（brief 步骤 1-5）
    # ------------------------------------------------------------------

    def _run_flow(self, trigger: str) -> dict:
        """服务/键盘共用流程入口。返回 {'success': bool, 'message': str}。

        互斥：一次只跑一个流程（键盘与服务并发触发时，后者忽略并告警）。
        """
        if not self._flow_lock.acquire(blocking=False):
            msg = f'grasp flow already running; ignore trigger={trigger}'
            self.get_logger().warn(msg)
            return {'success': False, 'message': msg}
        # 感知-执行互锁（2026-08-07）：流程开始即暂停 graspnet 推理
        #（写 pause_inference 标记 → 推理跳过帧、CSV 冻结在执行前干净帧）；
        # 流程结束（无论结果，A 方案）机械臂归零完成后删除标记恢复推理。
        self._set_pause_marker()
        try:
            return self._run_flow_impl(trigger)
        except Exception as exc:
            self.get_logger().error(
                f'grasp flow exception: {exc!r}\n{traceback.format_exc()}')
            return {'success': False, 'message': f'grasp flow exception: {exc!r}'}
        finally:
            # 收尾回 zero（2026-08-14 用户裁决：抓取完成后归零，
            # 取代 2026-08-10 的回 ready；scan_only 不动臂，跳过）；
            # 失败不阻塞恢复
            if not self._scan_only:
                try:
                    if not self._goto_joints(ZERO_JOINTS, '回 zero'):
                        self.get_logger().warn('收尾回 zero 失败（继续恢复推理）')
                except Exception as exc:
                    self.get_logger().warn(f'收尾回 zero 异常（{exc}）')
            self._clear_pause_marker()

    def _set_pause_marker(self) -> None:
        """写暂停标记：graspnet 推理循环检测到即跳过帧（CSV 冻结）。"""
        try:
            with open(os.path.join(self._shared_dir, 'pause_inference'), 'w'):
                pass
            self.get_logger().info('感知暂停：写入 pause_inference（推理冻结）')
        except Exception as exc:
            self.get_logger().warn(f'写 pause_inference 失败（{exc}）')

    def _clear_pause_marker(self) -> None:
        """删除暂停标记：graspnet 恢复推理。"""
        try:
            p = os.path.join(self._shared_dir, 'pause_inference')
            if os.path.exists(p):
                os.remove(p)
            self.get_logger().info('感知恢复：删除 pause_inference（推理继续）')
        except Exception as exc:
            self.get_logger().warn(f'删 pause_inference 失败（{exc}）')
        finally:
            self._flow_lock.release()

    def _write_precheck_results(self, ranked: list, approved_indices: list,
                                selected_index) -> None:
        """预检结果原子写 shared_dir/precheck_results.json（推理侧可视化用）。

        内容 = 相机系位姿（build_precheck_results）；失败仅告警不阻断流程。
        """
        try:
            data = build_precheck_results(ranked, approved_indices, selected_index)
            data['timestamp'] = time.time()
            path = os.path.join(self._shared_dir, 'precheck_results.json')
            tmp = f'{path}.tmp.{os.getpid()}'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, path)
            self.get_logger().info(
                f'预检结果写入 {path}: approved={len(data["approved"])} '
                f'selected={selected_index}')
        except Exception as exc:
            self.get_logger().warn(f'写 precheck_results.json 失败（{exc}）')

    def _run_flow_impl(self, trigger: str) -> dict:
        # ---- 1. 读 csv_path 最近文件 → parse_grasp_csv ----
        csv_path = self._resolve_csv_path()
        if not os.path.isfile(csv_path):
            msg = f'perception CSV not found: {csv_path} (run the perception pipeline first)'
            self.get_logger().error(msg)
            if self._scan_only:
                return {'success': True, 'message': f'scan_only: {msg}'}
            return self._finish_failure('no_candidate', 'precheck', msg)

        cands = parse_grasp_csv(csv_path)
        if not cands:
            msg = f'no candidates parsed from {csv_path}'
            self.get_logger().warn(msg)
            if self._scan_only:
                return {'success': True, 'message': f'scan_only: {msg}'}
            return self._finish_failure('no_candidate', 'precheck', msg)

        # ---- 2. 预检前确保回 zero（2026-08-14 用户裁决）----
        # 预检规划与真机抓取均以 zero 为初始位姿（取代 2026-08-10 的 ready）：
        # 预检的 KDL 起始状态与执行的预位/approach/grasp 都从 zero 出发。
        # scan_only 不动臂，跳过；失败 fail-closed。
        if not self._scan_only:
            if not self._goto_joints(ZERO_JOINTS, '回 zero'):
                msg = '回 zero 失败（fail-closed，不执行预检/抓取）'
                self.get_logger().error(msg)
                return self._finish_failure('home', 'home', msg)

        # ---- 3. 候选检测（2026-08-10 用户裁决：每次执行只取一帧）----
        # 不再多帧累加等待：2026-08-07 曾连续最多 5 帧（每帧 CSV 更新 ~2s），
        # 任一帧出现全过候选立即选中；现每次执行只取最新一帧的候选集，
        # 单帧内无全过候选即失败返回。如需恢复可运行时传 candidate_frames。
        max_frames = int(self.get_parameter('candidate_frames').value) \
            if self.has_parameter('candidate_frames') else 1
        ranked_all = []
        tested = 0
        gate_rejected = 0
        pass_counts = {'approach': 0, 'grasp': 0, 'lift': 0}
        selected = None
        selected_index = -1
        selected_stage_poses = None
        best_down = float('-inf')  # 方向偏好：全过候选 TCP 前向朝下分量最大值

        # 2026-08-07 修复（Task 10 真机验证）：全过候选中选 TCP 前向最朝下的
        # ——首个全过可能是水平候选（GraspNet 方向混合），其 link6 在物体侧方
        # 0.1358m（旧 calib_18 水平语义，TCP_OFFSET_FROM_LINK6_Z），末端不在
        # 物体上方。方向偏好保证选中候选的前向朝下、link6 目标在物体正上方。
        for frame_idx in range(max_frames):
            if frame_idx > 0:
                # 等待下一帧（CSV mtime 变化，最多 ~10s）
                self.get_logger().info(f'第 {frame_idx}/{max_frames} 帧无全过候选，等待下一帧...')
                last_m = os.path.getmtime(csv_path)
                for _ in range(20):
                    time.sleep(0.5)
                    if os.path.exists(csv_path) and os.path.getmtime(csv_path) != last_m:
                        break
                cands = parse_grasp_csv(csv_path)
                if not cands:
                    continue
            ranked = rank_candidates(cands)
            approved_indices = []  # 预检全过候选（推理侧可视化过滤用）
            self.get_logger().info(
                f'--- 候选帧 {frame_idx + 1}/{max_frames}：{len(ranked)} 候选 ---')
            for idx, cand in enumerate(ranked[:self._max_candidates]):
                self.get_logger().info(
                    f'candidate[{idx}] {cand.name}: score={cand.score:.4f} '
                    f'width={cand.width:.4f} xyz_camera={cand.xyz_camera.tolist()}')
                # 变换链（camera_pose_to_base + graspnet_to_tcp_rotation）在
                # Prechecker.make_stage_poses 内部完成（Task 6 接口）
                stage_poses = self._prechecker.make_stage_poses(
                    cand, self._handeye,
                    self._approach_clearance, self._lift_z, self._open_gripper_lift_height,
                    self._grasp_z_offset)
                # 2026-08-07 用户裁决：得分质量阈值（0.45+）
                if float(cand.score) < self._grasp_score_min:
                    gate_rejected += 1
                    self.get_logger().warn(
                        f'candidate[{idx}] {cand.name}: score={cand.score:.3f} '
                        f'< {self._grasp_score_min:.2f}（质量阈值），跳过')
                    continue
                if self._enable_self_consistency and not self_consistency(cand):
                    gate_rejected += 1
                    self.get_logger().warn(
                        f'candidate[{idx}] {cand.name}: self-consistency 过滤（中心横向漂移），'
                        f'跳过继续遍历（gate_rejected={gate_rejected}）')
                    continue
                tested += 1
                all_pass, stage_results = self._prechecker.precheck_stages(
                    stage_poses, scan_only_report=True)
                for name, ok in stage_results.items():
                    pass_counts[name] += int(bool(ok))
                if all_pass:
                    approved_indices.append(idx)
                    # 方向偏好（2026-08-07 第三次）：新映射下 link6 前向 = GraspNet
                    # +Z（法向）在 base、开口（link7 = -Y_link6）朝下分量 = r[1,2]。
                    q = stage_poses['grasp']['quat']  # [w,x,y,z]
                    r = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
                    down = float(r[1, 2])  # 开口朝下分量
                    self.get_logger().info(
                        f'candidate[{idx}] {cand.name}: 预检全过，'
                        f'link6 前向 z={float(r[2, 2]):.2f} '
                        f'开口朝下={down:.2f}'
                        f'{"（当前最佳）" if selected is None or down > best_down else ""}')
                    if selected is None or down > best_down:
                        best_down = down
                        selected = cand
                        selected_index = idx
                        selected_stage_poses = stage_poses
            if selected is not None:
                break  # 帧循环：找到可用候选立即执行

        summary = (f'tested={tested}, gate_rejected={gate_rejected}, '
                   f'approach_pass={pass_counts["approach"]}, '
                   f'grasp_pass={pass_counts["grasp"]}, lift_pass={pass_counts["lift"]}')

        # ---- 3. scan_only：返回报告（不落盘不广播）；否则执行 ----
        if self._scan_only:
            # 预检结果写共享目录（推理侧可视化：只画 approved、选中项红色）
            self._write_precheck_results(ranked, approved_indices, selected_index)
            if selected is None:
                msg = (f'scan_only report: candidates={len(ranked)} {summary}'
                       f' -> 无全过候选（scan_only 模式，臂不动）')
            else:
                msg = (f'scan_only report: candidates={len(ranked)} {summary}'
                       f' -> 候选 [{selected_index}] {selected.name} 全过，'
                       f'将执行抓取（scan_only 模式，臂不动）')
            self.get_logger().info(msg)
            return {'success': True, 'message': msg}

        if selected is None:
            # 无可用候选也回写（approved 空 → 推理侧回退全量显示）
            self._write_precheck_results(ranked, approved_indices, None)
            msg = f'no candidate passed precheck: candidates={len(ranked)} {summary}'
            self.get_logger().warn(msg)
            return self._finish_failure('no_candidate', 'precheck', msg)

        self._write_precheck_results(ranked, approved_indices, selected_index)
        # 2026-08-14：起点/终点均为 zero（预检前已回 zero，见第 2 段；
        # 收尾回 zero 在 finally）。
        width = float(selected.width)
        pick = self._executor.execute_pick(
            selected_stage_poses, width, max_stage=self._dry_run_max_stage or None)

        # 2026-08-07 用户要求：执行后（收尾归零前）FK 验证夹爪实际朝向
        #（采样 mock 终态 = 执行后姿态，参考 URDF joint7 X90° 建模）
        gripper_dir = self._fk_gripper_orientation()
        if gripper_dir is not None:
            self.get_logger().info(
                f'夹爪开口方向（FK/URDF）: [{gripper_dir[0]:+.2f},'
                f'{gripper_dir[1]:+.2f},{gripper_dir[2]:+.2f}] '
                f'朝下分量={-gripper_dir[2]:.2f}')

        # 2026-08-08 用户要求：FK 比对相机感知位置（诊断手眼标定/高度偏差）——
        # 在收尾归零前落盘 link7 位置（复用朝向 FK 的请求/响应）。
        fk_pos = self._fk_link7_position()
        if fk_pos is not None:
            self.get_logger().info(
                f'FK link7 位置（收尾回 zero 前）: ({fk_pos[0]*1000:.1f}, '
                f'{fk_pos[1]*1000:.1f}, {fk_pos[2]*1000:.1f}) mm')
            try:
                import json as _json
                with open(os.path.join(self._shared_dir, 'fk_link7_position.json'), 'w') as f:
                    _json.dump({'pos': [float(v) for v in fk_pos]}, f)
            except OSError as exc:
                self.get_logger().warn(f'FK 位置落盘失败（{exc}）')

        status = pick.get('status', 'failed')
        stage = pick.get('stage', '')
        actual_width = pick.get('actual_width')
        # Task 10 单段验证：dry_stop = 按 max_stage 预期停止（臂停在段末位姿）
        success = (status == 'success') or (status == 'dry_stop')
        # I3 修复：记录实际命令宽度（execute_pick 回传 = width + margin 米制总开度，
        # 与内部判定同一数值；兜底按统一公式计算）
        cmd_width = pick.get('cmd_width')
        if cmd_width is None:
            cmd_width = width + self._gripper_margin
        msg = (f'candidate [{selected_index}] {selected.name}: pick status={status} '
               f'stage={stage!r} actual_width={actual_width} cmd_width={cmd_width:.4f}')
        self.get_logger().info(msg)

        # ---- 4+5. 结果落盘 + 广播 ----
        return self._finish_pick(
            success=success, status=status, stage=stage,
            actual_width=actual_width, cmd_width=cmd_width,
            candidate_index=selected_index, message=msg)

    # ------------------------------------------------------------------
    # 结果落盘 / 广播
    # ------------------------------------------------------------------

    def _finish_pick(self, success: bool, status: str, stage: str,
                     actual_width, cmd_width, candidate_index, message: str) -> dict:
        result = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'success': bool(success),
            'status': status,
            'stage': stage,
            'actual_width': actual_width,
            'cmd_width': cmd_width,
            'candidate_index': candidate_index,
            'message': message,
        }
        self._write_result(result)
        self._publish_result(result)
        return {'success': bool(success), 'message': json.dumps(result, ensure_ascii=False)}

    def _fk_link7_position(self):
        """执行后（回 zero 前）link7 位置（base 系），FK 计算。返回 (3,) 或 None。"""
        try:
            mg = self._executor.move_group
            if mg is None:
                return None
            js = getattr(mg, '_joint_state', None)
            if js is None or not js.name:
                return None
            from moveit_msgs.msg import RobotState
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['link7']
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(js.name)
            req.robot_state.joint_state.position = list(js.position)
            if not self._fk_client.wait_for_service(timeout_sec=2):
                return None
            fut = self._fk_client.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3:
                time.sleep(0.05)
            if not fut.done():
                return None
            resp = fut.result()
            if not resp.pose_stamped:
                return None
            ps = resp.pose_stamped[0].pose.position
            return (float(ps.x), float(ps.y), float(ps.z))
        except Exception:
            return None

    def _fk_gripper_orientation(self):
        """执行后夹爪朝向验证（2026-08-07 用户要求：FK + URDF 建模）。

        采样 mock 关节角 → /compute_fk 求 link7 位姿 → 开口/夹持方向
        = link7 的 Z 轴（base 系）。URDF 建模：joint7 origin rpy=π/2
        （X 旋转 90°）→ link7 Z 轴 = link6 姿态的 -Y 轴。
        返回方向向量或 None（失败）。
        """
        try:
            self.get_logger().info('夹爪朝向 FK：开始')
            mg = self._executor.move_group
            if mg is None:
                self.get_logger().warn('夹爪朝向 FK：move_group 为 None')
                return None
            js = getattr(mg, '_joint_state', None)
            if js is None or not js.name:
                self.get_logger().warn('夹爪朝向 FK：/joint_states 缓存为空')
                return None
            self.get_logger().info(f'夹爪朝向 FK：关节数 {len(js.name)}')
            from moveit_msgs.msg import RobotState
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['link6', 'link7']
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(js.name)
            req.robot_state.joint_state.position = list(js.position)
            if not self._fk_client.wait_for_service(timeout_sec=2):
                self.get_logger().warn('夹爪朝向 FK：/compute_fk 服务不可用')
                return None
            fut = self._fk_client.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 5:
                time.sleep(0.05)
            if not fut.done():
                self.get_logger().warn('夹爪朝向 FK：调用超时')
                return None
            res = fut.result()
            from scipy.spatial.transform import Rotation as _R
            # FK 响应按请求 fk_link_names 顺序（link6, link7）返回
            p6 = res.pose_stamped[0].pose if len(res.pose_stamped) > 0 else None
            p7 = res.pose_stamped[1].pose if len(res.pose_stamped) > 1 else None
            if p6 is not None:
                R6 = _R.from_quat([p6.orientation.x, p6.orientation.y,
                                   p6.orientation.z, p6.orientation.w]).as_matrix()
                self.get_logger().info(
                    f'执行后 link6 前向（FK）: [{R6[0,2]:+.2f},{R6[1,2]:+.2f},'
                    f'{R6[2,2]:+.2f}] 前向 z={R6[2,2]:+.2f}')
            if p7 is not None:
                R7 = _R.from_quat([p7.orientation.x, p7.orientation.y,
                                   p7.orientation.z, p7.orientation.w]).as_matrix()
                return R7[:, 2]  # link7 Z 轴 = 夹持方向（base 系）
            return None
        except Exception as exc:
            self.get_logger().warn(f'夹爪朝向 FK 失败（{exc}）')
            return None

    def _go_home(self) -> bool:
        """兼容包装：回 zero（历史语义，保留给外部调用）。"""
        return self._goto_joints([0.0] * 6, '回 zero')

    def _goto_joints(self, target_joints: list, label: str) -> bool:
        """MoveIt 低速规划执行到指定关节角（2026-08-10 泛化自 _go_home）。

        2026-08-14 用户裁决：预检起点与收尾终点均为 zero（ZERO_JOINTS），
        取代 2026-08-10 的 ready 语义。
        失败 fail-closed（不执行抓取/不进入预检）。

        Args:
            target_joints: 6 个关节角目标（弧度）。
            label: 日志标签（如 '回 zero' / '回 ready'）。
        """
        if self._executor is None or self._executor.move_group is None:
            self.get_logger().warn(f'{label} 占位（move_group=None）：fail-closed')
            return False
        try:
            mg = self._executor.move_group
            # 2026-08-08：当前已在目标位（容差 0.01 rad）则跳过规划执行——
            # 零位移轨迹经 /execute_trajectory 执行失败（第 3 次触发实测：
            # 机械臂已在目标位时收尾回位报"执行失败"）。
            try:
                cur = mg.get_current_joint_values()
                if cur is not None and max(
                        abs(v - t) for v, t in zip(cur, target_joints)) < 0.01:
                    self.get_logger().info(f'{label}：当前已在目标位，跳过执行')
                    return True
            except Exception:
                pass  # 读数失败不阻塞，继续规划执行路径
            mg.set_planning_time(float(
                self.get_parameter('planning_time_exec').value))
            mg.set_max_velocity_scaling_factor(self._velocity_scale)
            mg.set_start_state_to_current_state()
            mg.set_joint_value_target(list(target_joints))
            plan = mg.plan()
            if plan is None:
                self.get_logger().warn(f'{label} 规划失败：fail-closed')
                return False
            ok = mg.execute(plan)
            self.get_logger().info(f'{label} 执行：{"成功" if ok else "失败"}')
            return ok
        except Exception as exc:
            self.get_logger().warn(f'{label} 异常（{exc}）：fail-closed')
            return False

    def _finish_failure(self, status: str, stage: str, message: str) -> dict:
        return self._finish_pick(False, status, stage, None, None, None, message)

    def _write_result(self, result: dict) -> None:
        """grasp_result.json 原子落盘（tmp + os.replace，与感知侧文件协议一致）。"""
        path = os.path.join(self._shared_dir, 'grasp_result.json')
        try:
            os.makedirs(self._shared_dir, exist_ok=True)
            tmp = f'{path}.tmp.{os.getpid()}.{time.time_ns()}'
            try:
                with open(tmp, 'w') as f:
                    json.dump(result, f, ensure_ascii=False, indent=2)
                os.replace(tmp, path)
            finally:
                if os.path.exists(tmp):
                    try:
                        os.unlink(tmp)
                    except OSError:
                        pass
            self.get_logger().info(f'grasp_result.json written: {path}')
        except OSError as exc:
            self.get_logger().error(f'failed to write grasp_result.json: {exc}')

    def _publish_result(self, result: dict) -> None:
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    node.start_keyboard_thread()
    # C1 修复：MultiThreadedExecutor —— 服务回调在 _run_flow_in_thread 内
    # thread.join() 阻塞期间（velocity_scale=0.1 下流程可达数分钟），
    # /joint_states_feedback 订阅回调必须仍在其他线程被处理，否则
    # GraspExecutor._feedback_cb 饿死、_latest_gripper 恒为 None、夹爪判定
    # 恒走"反馈无数据→failed"分支。单线程 executor 下同一流程的两种触发
    # （服务/键盘）判定行为不一致。
    # 注意（C1 根因实测）：MTE 单独不够——反馈订阅还必须放在独立可重入回调组
    # （grasp_execute._setup_feedback_subscription）——rclpy 默认互斥回调组在
    # 服务回调阻塞期间会锁住同组订阅回调。两者缺一不可（集成测试覆盖）。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('grasp_executor stopping (Ctrl+C)')
    except ExternalShutdownException:
        # rclpy 默认安装 SIGINT/SIGTERM C 级处理器 → spin 以该异常退出
        node.get_logger().info('grasp_executor stopping (external shutdown)')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
