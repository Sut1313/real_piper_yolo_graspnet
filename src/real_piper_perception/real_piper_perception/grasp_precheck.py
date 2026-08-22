"""grasp_executor 4 段预检模块（Task 6）：approach→grasp→open-lift 三段 dry-run。

真机 grasp_executor 的候选选中逻辑：对每个 GraspNet 候选先做三段可达性 dry-run
预检（只规划不执行），全过才执行。本模块移植仿真实现：

- ``planFullPoseWithSeeds``（pick_and_place.cpp:183-307）→ ``Prechecker.precheck_pose``：
  ssik 解析种子 → FK 回验（0.015m / 0.1rad）→ move_group.plan()；analytic-only 门控
  （label 含 'visual-center' 且 ``graspnet_6d_analytic_only_precheck``=True 时 ssik
  无解直接 False，禁止 KDL/direct 兜底）。
- ``solvePiperAnalyticIkForTcp``（piper_analytic_ik.cpp）→ ``_run_ssik``：TCP 位姿
  折算 link6（TCP_OFFSET_FROM_LINK6_Z=0.1358m 偏移）→ 子进程调用
  ``piper_ssik_ik.py`` → 按 seed_cost 升序 / residual 升序排序。
- ``chooseReachableGraspCandidate`` 预检流水线（graspnet_6d_grasp_planner.cpp:182-202）
  → ``precheck_stages``：三段 AND，scan 统计（tested/approach_pass/grasp_pass/lift_pass）。
- ``makeObjectPoses``（grasp_frame_transform.cpp:246-271，预检相关部分）
  → ``make_stage_poses``：grasp / approach（grasp + Z approach_clearance）/
  open-lift（grasp + Z open_gripper_lift_height，仿真预检流水线原样）。

MoveIt2 交互部分：当前环境未安装 ``moveit_py``（探测见 Task 6 报告），此处为
接口占位 + 语法检查（docstring 已注明），真实 MoveGroupInterface 接线与联调在
Task 9 干跑完成。
"""

from __future__ import annotations

import math
import subprocess
import sys
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from real_piper_perception.grasp_transform import (
    camera_pose_to_base,
    graspnet_to_tcp_rotation,
)

if TYPE_CHECKING:
    from rclpy.node import Node  # 仅类型标注，不强制（node 传 None 时用 print 日志）

# ---------------------------------------------------------------------------
# 常量（数值从仿真源码原样拷贝）
# ---------------------------------------------------------------------------

# TCP 在 link6 坐标系下的 Z 向偏移（米）（URDF 权威值，2026-08-18 用户裁决：
# docs/real_grasp_error_investigation/06-urdf-tcp-authoritative-plan.md §1）。
# ssik 只解 base_link→link6，TCP 目标必须先折算回 link6 才能求解。
# I1 修复（2026-08-06 真机事实）：真机 URDF 链接清单 = base_link, link1..6,
# gripper_base, link7, link8——**不存在 grasp_tcp 帧，也不存在
# gripper_base_to_grasp_tcp 关节**（grasp_tcp 是仿真独有）。真实几何：
# joint6_to_gripper_base 原点 0 0 0，joint7/joint8 原点 z=0.1358（prismatic
# 滑移）。抓取 TCP 目标沿 TCP 局部 Z 轴定义，TCP→link6 折算 = 右乘
# T(0,0,-0.1358)。joint7 的 0.035m 开度与 rpy=1.5708 只用于 link7/link8 FK
# 几何验证，不混入 TCP 主偏移；link7 铰点 ≠ TCP 指尖中心。
TCP_OFFSET_FROM_LINK6_Z = 0.1358

# FK 回验门限（planFullPoseWithSeeds:254 原样）：位置误差 >1.5cm 或角度误差
# >0.1rad 的解不可信，丢弃（宁可不规划，防止轨迹终点偏离目标位姿）。
FK_MAX_POS_ERR = 0.015   # 米
FK_MAX_ANGLE_ERR = 0.10  # 弧度

# ssik 子进程超时（仿真用 popen 无超时；此处防死锁挂起整条流水线）
SSIK_TIMEOUT_S = 30.0

# 参数键默认值（brief 参数键清单；与仿真 get_parameter_or 默认值一致）
PARAM_DEFAULTS = {
    'graspnet_6d_analytic_only_precheck': True,
    'enable_piper_analytic_ik_planning': True,
    'piper_analytic_ik_max_solutions': 8,
    'piper_analytic_ik_max_solution_plans': 6,
    'planning_time_precheck': 1.0,
}

# 预检三段名（顺序即 AND 流水线顺序）
STAGE_ORDER = ('approach', 'grasp', 'lift')


def _quat_angle_err(q_actual: np.ndarray, q_target: np.ndarray) -> float:
    """两四元数（[w,x,y,z]）间最短角误差（弧度），仿真 dot 公式原样
    （planFullPoseWithSeeds:250-251）：angle_err = 2*acos(|dot|)。"""
    dot = abs(float(q_actual @ q_target))
    if dot >= 1.0:
        return 0.0
    return 2.0 * math.acos(max(-1.0, min(dot, 1.0)))


class Prechecker:
    """三段可达性 dry-run 预检器（approach→grasp→open-lift）。

    Args:
        node: rclpy Node，日志用；None 时退化为 print（无 ROS 单测可用）。
        move_group: MoveIt MoveGroupInterface（moveit_py）；None 为占位，
            MoveIt 交互走 fail-closed + 告警（Task 9 干跑联调接线）。
        ssik_script: piper_ssik_ik.py 绝对/相对路径。
        params: 参数键见 :data:`PARAM_DEFAULTS`（缺省自动补默认值）。
    """

    def __init__(self, node: 'Node' | None, move_group, ssik_script: str, params: dict):
        self.node = node
        self.move_group = move_group
        self.ssik_script = ssik_script
        self.params = dict(PARAM_DEFAULTS)
        if params:
            self.params.update(params)

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
        print(f'[grasp_precheck {level.upper()}] {msg}')

    # ------------------------------------------------------------------
    # 位姿数学
    # ------------------------------------------------------------------

    @staticmethod
    def _pose_to_matrix(pose: dict) -> np.ndarray:
        """geometry_msgs Pose 语义 dict（pos=[x,y,z], quat=[w,x,y,z]）→ 4x4 齐次。"""
        pos = np.asarray(pose['pos'], dtype=np.float64).reshape(3)
        q = np.asarray(pose['quat'], dtype=np.float64).reshape(4)  # [w,x,y,z]
        q = q / np.linalg.norm(q)
        R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()  # scipy 要 [x,y,z,w]
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = pos
        return T

    @staticmethod
    def _tcp_to_link6_matrix(tcp_matrix: np.ndarray) -> np.ndarray:
        """TCP 位姿 → link6 位姿（solvePiperAnalyticIkForTcp 步骤① 原样）：
        T_link6 = T_tcp · T(0,0,-0.1358)（右乘偏移逆变换，纯平移所以等价于
        沿 TCP Z 轴 -0.1358m）。ssik 求解的是 base_link→link6。"""
        offset_inv = np.eye(4, dtype=np.float64)
        offset_inv[2, 3] = -TCP_OFFSET_FROM_LINK6_Z
        return np.asarray(tcp_matrix, dtype=np.float64) @ offset_inv

    # ------------------------------------------------------------------
    # ssik 子进程封装
    # ------------------------------------------------------------------

    def _run_ssik(self, matrix_4x4, seed, max_solutions) -> list[tuple[np.ndarray, float]]:
        """调用 piper_ssik_ik.py 求 link6 解析 IK（solvePiperAnalyticIkForTcp ③④ 移植）。

        Args:
            matrix_4x4: link6 在 base_link 下的 4x4 齐次目标（脚本契约：
                行主序 16 浮点序列化，17 位精度——与 C++ 一致）。
            seed: 当前 6 关节角种子（None 时不传，解析器自行收敛）。
            max_solutions: 最多返回解数（至少 1）。

        Returns:
            [(joints(6,), residual), ...]，已按 seed_cost（最短角度差平方和）升序、
            同价 residual 升序排序（C++ jointDistance 排序移植）。非零退出码
            （2=ssik 导入失败 / 3=求解异常 / 4=矩阵格式错 / 5=种子格式错）告警后
            返回 [] —— 与仿真一致：ssik 不可用按 analytic-only 门控处理，不阻断。
        """
        m = np.asarray(matrix_4x4, dtype=np.float64).reshape(4, 4)

        def fmt(values) -> str:
            return ','.join(f'{v:.17g}' for v in np.asarray(values, dtype=np.float64).flatten())

        # 注意：17g 精度下接近 0 的值是科学计数法（如 -1.23e-05），argparse 会把
        # 以 '-' 开头的非负数模式参数误判为选项（2026-08-07 实测 rc=2）——
        # 用 --arg=<value> 等号形式，等号后内容无条件按值处理。
        cmd = [sys.executable, self.ssik_script, f'--matrix={fmt(m)}',
               '--max-solutions', str(max(1, int(max_solutions)))]
        if seed is not None:
            cmd += [f'--seed={fmt(np.asarray(seed, dtype=np.float64).reshape(6))}']

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=SSIK_TIMEOUT_S)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._log('warn', f'Piper ssik IK helper invocation failed '
                              f'({self.ssik_script}): {exc}')
            return []

        if proc.returncode != 0:
            stderr = proc.stderr.strip()
            self._log('warn', f'Piper ssik IK helper failed rc={proc.returncode}: {stderr}')
            return []

        # 逐行解析 stdout：6 关节角 + residual（注释/空行跳过，同 C++ fscanf 语义）
        solutions: list[tuple[np.ndarray, float]] = []
        for line in proc.stdout.splitlines():
            parts = line.split()
            if len(parts) != 7:
                continue
            try:
                joints = np.array([float(v) for v in parts[:6]], dtype=np.float64)
                residual = float(parts[6])
            except ValueError:
                continue
            solutions.append((joints, residual))

        if seed is not None and solutions:
            s = np.asarray(seed, dtype=np.float64).reshape(6)

            def seed_cost(sol) -> float:
                d = sol[0] - s
                d = (d + np.pi) % (2.0 * np.pi) - np.pi  # 最短角度差，包裹 [-π, π]
                return float(d @ d)

            solutions.sort(key=lambda sol: (seed_cost(sol), sol[1]))
        return solutions

    # ------------------------------------------------------------------
    # MoveIt 交互（占位，Task 9 干跑联调）
    # ------------------------------------------------------------------

    def _current_joints(self) -> np.ndarray | None:
        """当前关节角（种子）。move_group 未接线 / 读取失败返回 None——
        与仿真一致：读取失败则跳过解析 IK 层，走 analytic-only 门控。"""
        if self.move_group is None:
            return None
        # Task 9 干跑联调：moveit_py MoveGroupInterface.get_current_joint_values()
        try:
            joints = self.move_group.get_current_joint_values()
        except Exception as exc:
            self._log('warn', f'get_current_joint_values failed: {exc}')
            return None
        try:
            return np.asarray(joints, dtype=np.float64).reshape(6)
        except (ValueError, TypeError) as exc:
            self._log('warn', f'current joint values malformed: {exc}')
            return None

    def _get_planning_time(self) -> float | None:
        if self.move_group is None:
            return None
        try:
            return float(self.move_group.get_planning_time())
        except Exception:
            return None

    def _set_planning_time(self, t: float | None) -> None:
        if self.move_group is None or t is None:
            return
        try:
            self.move_group.set_planning_time(t)
        except Exception as exc:
            self._log('warn', f'set_planning_time failed: {exc}')

    def _fk_tcp(self, joints: np.ndarray):
        """MoveIt FK：关节解 → TCP 位姿（Task 9 干跑联调接入）。

        接入前（move_group 未接线）返回 None，FK 回验退化为 ssik residual 代理
        （见 :meth:`_fk_verify`）。I1 修复：真机 URDF 无 grasp_tcp 帧，arm 组
        tip 为 link6——Task 9 接 RobotState FK：setJointGroupPositions +
        getGlobalLinkTransform('link6')，再右乘 T(0,0,+0.1358)（与
        _tcp_to_link6_matrix 互逆）折算回 TCP 与 pose 比较，返回
        (pos(3,), quat[w,x,y,z])。
        """
        if self.move_group is None:
            return None
        self._log('warn', 'MoveIt FK 回验未接线（Task 9 干跑联调）：暂以 ssik residual 代理')
        return None

    def _fk_verify(self, joints: np.ndarray, pose: dict, residual: float) -> bool:
        """FK 回验（planFullPoseWithSeeds:241-260 移植）：TCP 位置误差 ≤0.015m
        且姿态角误差 ≤0.1rad。MoveIt 未接线时用 ssik 自带 FK 残差（link6 级，
        ~1e-10）做代理：残差超 1.5cm 视为不可信解。"""
        tcp = self._fk_tcp(joints)
        if tcp is None:
            return float(residual) <= FK_MAX_POS_ERR
        pos, quat = tcp
        pos_err = float(np.linalg.norm(np.asarray(pos, dtype=np.float64)
                                       - np.asarray(pose['pos'], dtype=np.float64)))
        qa = np.asarray(quat, dtype=np.float64)
        qa = qa / np.linalg.norm(qa)
        qt = np.asarray(pose['quat'], dtype=np.float64)
        qt = qt / np.linalg.norm(qt)
        angle_err = _quat_angle_err(qa, qt)
        return pos_err <= FK_MAX_POS_ERR and angle_err <= FK_MAX_ANGLE_ERR

    def _plan_to_joints(self, joints: np.ndarray, label: str) -> bool:
        """MoveIt 关节目标规划（碰撞感知轨迹）：移植 planFullPoseWithSeeds:264-268
        （setStartStateToCurrentState + setJointValueTarget + plan()）。

        占位（move_group=None 或接口未接线）：fail-closed 返回 False 并告警，
        Task 9 干跑联调接入真实 moveit_py MoveGroupInterface。
        """
        if self.move_group is None:
            self._log('warn', f'MoveIt plan() 占位（move_group=None）：{label} 预检规划 '
                              f'fail-closed；Task 9 干跑联调')
            return False
        # Task 9：真实 moveit_py 接入（方法名按 moveit_py snake_case API，
        # 若有出入以 Task 9 实测为准；异常路径 fail-closed 不阻断流水线）
        try:
            self.move_group.set_start_state_to_current_state()
            if not self.move_group.set_joint_value_target(joints.tolist()):
                return False
            plan = self.move_group.plan()
            return bool(plan)
        except Exception as exc:
            self._log('warn', f'MoveIt plan() 未接线或调用失败（{exc}）：{label} '
                              f'fail-closed；Task 9 干跑联调')
            return False

    def _plan_to_pose(self, link6_matrix: np.ndarray, label: str) -> bool:
        """数值 IK 兜底：MoveIt pose target 规划（move_group 自带 IK 求解器）。

        M2 落地（2026-08-07 真机联调裁决）：朝下抓姿态 ssik 解析 IK 无解，
        MoveIt OMPL + KDL 数值 IK 可达（实测 6/6）——analytic 门控关闭时以此兜底。
        目标为 link6 位姿（4x4 基座系），由 move_group 自行求解 IK 与轨迹。
        """
        if self.move_group is None:
            self._log('warn', f'MoveIt pose 兜底占位（move_group=None）：{label} fail-closed')
            return False
        try:
            from geometry_msgs.msg import Pose
            from scipy.spatial.transform import Rotation as _Rotation
            q = _Rotation.from_matrix(link6_matrix[:3, :3]).as_quat()  # [x,y,z,w]
            pose_msg = Pose()
            pose_msg.position.x = float(link6_matrix[0, 3])
            pose_msg.position.y = float(link6_matrix[1, 3])
            pose_msg.position.z = float(link6_matrix[2, 3])
            pose_msg.orientation.w = float(q[3])
            pose_msg.orientation.x = float(q[0])
            pose_msg.orientation.y = float(q[1])
            pose_msg.orientation.z = float(q[2])
            self.move_group.set_start_state_to_current_state()
            self.move_group.set_pose_target(pose_msg, end_effector_link='link6')
            plan = self.move_group.plan()
            if plan is None:
                self._log('warn', f'MoveIt pose 兜底规划失败：{label}')
                return False
            self._log('info', f'MoveIt pose 兜底规划 PASS（{label}）')
            return True
        except Exception as exc:
            self._log('warn', f'MoveIt pose 兜底规划调用失败（{exc}）：{label}')
            return False

    # ------------------------------------------------------------------
    # 核心 API
    # ------------------------------------------------------------------

    def precheck_pose(self, pose, label: str) -> bool:
        """单段 dry-run：ssik 种子 → FK 回验(0.015m/0.1rad) → move_group.plan()。

        移植 planFullPoseWithSeeds（183-307 行）；analytic-only 门控：label 含
        'visual-center' 且 params['graspnet_6d_analytic_only_precheck']=True 时
        ssik 无解直接 False（禁止 KDL/direct 兜底，兜底由候选遍历提供）。

        Args:
            pose: geometry_msgs Pose 语义 dict（pos=[x,y,z], quat=[w,x,y,z]），基座系 TCP。
            label: 预检段标签（含 'visual-center' 触发 analytic-only 门控）。
        """
        previous_planning_time = self._get_planning_time()
        # 预检/快速求解用 1s 上限的短规划时间（只探可达性），结束恢复
        self._set_planning_time(min(float(self.params['planning_time_precheck']), 1.0))
        try:
            if self.params['enable_piper_analytic_ik_planning']:
                current_joints = self._current_joints()
                if current_joints is None:
                    self._log('warn', f'Cannot get current robot state for Piper analytic '
                                      f'IK while planning {label}.')
                else:
                    tcp_matrix = self._pose_to_matrix(pose)
                    link6_matrix = self._tcp_to_link6_matrix(tcp_matrix)
                    solutions = self._run_ssik(
                        link6_matrix, current_joints,
                        self.params['piper_analytic_ik_max_solutions'])
                    max_plans = max(1, int(self.params['piper_analytic_ik_max_solution_plans']))
                    planned_attempts = 0
                    for joints, residual in solutions:
                        if planned_attempts >= max_plans:
                            break
                        planned_attempts += 1
                        if not self._fk_verify(joints, pose, residual):
                            self._log('warn', f'Piper analytic IK rejected by FK for '
                                              f'{label}: residual={residual:.3g}.')
                            continue
                        if self._plan_to_joints(joints, label):
                            self._log('info', f'Piper analytic IK planning PASS for {label} '
                                              f'(solutions={len(solutions)}, '
                                              f'planned_attempt={planned_attempts}, '
                                              f'residual={residual:.3g}).')
                            return True
                    self._log('warn', f'Piper analytic IK planning failed for {label}: '
                                      f'solutions={len(solutions)}, '
                                      f'planned_attempts={planned_attempts}.')

            # analytic-only 门控：预检阶段（visual-center 标签）禁止数值 IK 兜底。
            # M2 落地（2026-08-07 真机联调裁决）：ssik 解析 IK 对"朝下抓"姿态
            # 无解（实测 0/8），但 MoveIt OMPL + KDL 数值 IK 可达（实测 6/6）——
            # 门控开启时 ssik 无解直接 False（纯解析安全模式）；关闭时走
            # _plan_to_pose 数值 IK 兜底（MoveIt 自带 IK 求解器 + 碰撞感知）。
            if 'visual-center' in label and self.params['graspnet_6d_analytic_only_precheck']:
                self._log('warn', f'Skipping numeric IK fallback for {label} because GraspNet '
                                  f'6D candidate precheck is analytic-only.')
                return False

            # 数值 IK 兜底（M2 落地）：ssik 无解时以 link6 位姿目标走 move_group
            # 规划（move_group 内部 KDL/数值 IK 求解 + OMPL 轨迹 + 碰撞检查）。
            tcp_matrix = self._pose_to_matrix(pose)
            link6_matrix = self._tcp_to_link6_matrix(tcp_matrix)
            return self._plan_to_pose(link6_matrix, label)
        finally:
            self._set_planning_time(previous_planning_time)

    def precheck_stages(self, stage_poses: dict, scan_only_report: bool = True) -> tuple[bool, dict]:
        """两段 AND 预检：approach → grasp（2026-08-10 用户裁决：lift 段已关）。

        原为三段（含 open-lift）；ready 起点下 lift 段规划经常失败导致候选
        误拒，现只检 approach + grasp，lift 可达性由执行段验证（place 段不在
        本任务范围）。

        移植 chooseReachableGraspCandidate 预检流水线（graspnet_6d_grasp_planner.cpp:
        182-202）的 AND 语义：任一段失败即停（后续段保持 False）。label 恒含
        'visual-center'（真机 workspace 模式物体名恒为 'target'），触发
        analytic-only 门控。scan_only_report=True 时打印 scan 统计
        （tested/approach_pass/grasp_pass/lift_pass，仿真 scan summary 格式移植）。

        Returns:
            (全过?, {'approach': bool, 'grasp': bool, 'lift': bool})。
        """
        results = {name: False for name in STAGE_ORDER}
        tested = 0
        for name in STAGE_ORDER:
            if name == 'lift':
                # 2026-08-10 用户裁决：预检关掉 lift 段——ready 起点下
                # lift 段规划经常失败导致候选误拒（实测 approach+grasp 过、
                # lift 挂）；lift 可达性由执行段验证。results 保留 lift 键
                # （恒 False）兼容报告/调用方。
                continue
            if name not in stage_poses:
                break  # 缺段：AND 语义下后续段不再尝试
            tested += 1
            label = f'visual-center {name}'
            results[name] = self.precheck_pose(stage_poses[name], label)
            if not results[name]:
                break
        all_pass = bool(results['approach'] and results['grasp'])
        if scan_only_report:
            self._log('info', f'target GraspNet 6D scan summary: tested={tested}, '
                              f'approach_pass={int(results["approach"])}, '
                              f'grasp_pass={int(results["grasp"])}, '
                              f'lift_pass={int(results["lift"])}.')
        return all_pass, results

    def make_stage_poses(self, cand: 'PerceptionTarget', handeye: np.ndarray,
                         approach_clearance: float, lift_z: float,
                         open_gripper_lift_height: float,
                         grasp_z_offset: float = 0.0) -> dict:
        """由候选生成三段位姿（基座系，geometry_msgs Pose 语义：位置+四元数）：

        - grasp: 候选相机系 (xyz_camera, rotation) → camera_pose_to_base →
          graspnet_to_tcp_rotation（变换链见 docs/grasp_executor_transform_notes.md §1）
        - approach: grasp 位置 + Z(基座系向上) approach_clearance；姿态同 grasp
        - lift: grasp 位置 + Z open_gripper_lift_height；姿态同 grasp
          （仿真预检流水线 open_lift_pose = grasp.z + open_gripper_lift_height 原样，
          graspnet_6d_grasp_planner.cpp:196-198）

        Args:
            cand: 上游 PerceptionTarget（xyz_camera/rotation 相机系原值）。
            handeye: load_handeye 返回的 4x4 T_base_camera。
            approach_clearance: 接近间隙（米，基座系 Z 向上）。
            lift_z: 仿真执行路径抬升高度（米）——makeObjectPoses 的绝对抬升目标，
                预检 lift 段按 open_gripper_lift_height 计算（同仿真预检流水线），
                lift_z 在本预检三段中不使用，保留参数以匹配仿真 API
                （Task 8/9 执行路径接入时用）。
            open_gripper_lift_height: 张开夹爪抬升量（米）。

        Returns:
            {'approach': dict(pos=[x,y,z], quat=[w,x,y,z]),
             'grasp': ..., 'lift': ...}（dict 而非 Pose 消息，便于无 ROS 单测）。
        """
        pos_base, rot_base_graspnet = camera_pose_to_base(cand.xyz_camera, cand.rotation, handeye)
        r_tcp = graspnet_to_tcp_rotation(rot_base_graspnet)
        # scipy 返回 [x,y,z,w]，按接口约定重排为 [w,x,y,z]
        q = Rotation.from_matrix(r_tcp).as_quat()
        quat = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]

        # 2026-08-08：grasp_z_offset 真机标定修正——感知候选中心 z 与夹爪闭合
        # 平面有系统性偏差（实测偏高 2cm），grasp 位下移 offset；approach/lift
        # 基于修正后的 grasp 高度（保持相对高度差）。
        grasp_z = float(pos_base[2]) - float(grasp_z_offset)
        grasp = {'pos': [float(pos_base[0]), float(pos_base[1]), grasp_z],
                 'quat': quat}
        approach = {
            'pos': [float(pos_base[0]), float(pos_base[1]),
                    grasp_z + float(approach_clearance)],
            'quat': quat,
        }
        lift = {
            'pos': [float(pos_base[0]), float(pos_base[1]),
                    grasp_z + float(open_gripper_lift_height)],
            'quat': quat,
        }
        return {'approach': approach, 'grasp': grasp, 'lift': lift}
