# 真机 Grasp Executor 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 real_piper_graspnet 上实现单次真机抓取验证：读 perception_grasps.csv → 候选筛选 → 4 段预检（OMPL+ssik）→ MoveIt 执行 → 夹爪反馈判定。

**Architecture:** 新 Python 节点 `grasp_executor`（加入现有 `real_piper_perception` 包），按职责拆为候选/变换/预检/执行/判定五个模块；预检移植仿真 `planFullPoseWithSeeds` 的 ssik 解析种子 + analytic-only 门控；执行走官方真机路径（`start_single_piper` remap + `demo.launch.py` mock 控制器）。

**Tech Stack:** ROS 2 Humble、MoveIt2（move_group_py）、piper_sdk 0.6.2（参考层）、ssik 3.3.1（解析 IK）、Python 3.10、pytest。

**Spec:** `docs/superpowers/specs/2026-08-06-grasp-executor-design.md`

## Global Constraints

- 范围：单次抓取验证；**不含**放置段、pick_state 多物体闭环、自动循环
- 执行链路：MoveIt2 全链路（can_activate.sh → start_single_piper.launch.py（remap /joint_ctrl_single→/joint_states）→ piper_with_gripper_moveit demo.launch.py）；不写 ros2_control 硬件接口
- 预检：`move_group.plan()`（OMPL 管线）+ ssik 解析种子 + FK 回验（pos_err<1.5cm 且 angle_err<0.1rad）+ analytic-only 门控（默认开）；预检规划时间 min(1.0s)
- 笛卡尔（compute_cartesian_path）仅用于执行段直线运动（approach 下探/抬升），不用于预检
- 安全：velocity_scale 默认 0.1；任一执行段失败 → 保持姿态、服务返回失败；scan_only 干跑模式
- 成功判定：`W_actual < W_cmd - detect_threshold` ⇒ 夹住（W_cmd = CSV width + gripper_margin）
- 夹爪单位：piper_sdk GripperCtrl range 单位 0.001mm；moveit joint7 弧度（1rad=1 单位关节角）；换算系数真机标定（gripper_val_mutiple=2）
- CSV 列格式（24 列，列序严格；以真实管线文件核实，原"25 列"为笔误）：`name,camera_x,camera_y,camera_z,count,score,width,height,depth,r00..r22,center_camera_x,center_camera_y,center_camera_z,link6_grasp_z,tcp_forward_z,source_dist_xy`
- 环境：ROS 2 humble + Python 3.10；piper_sdk 0.6.2、ssik 3.3.1 已装（`<python-site-packages>/`）
- 结果落盘：`shared_camera_data/grasp_result.json`（与感知文件协议一致）

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `src/real_piper_perception/real_piper_perception/grasp_transform.py` | handeye 标定加载、相机系→基座系、GraspNet→TCP 旋转映射（仿真常量移植，T1 确认值） |
| `src/real_piper_perception/real_piper_perception/grasp_candidates.py` | CSV 解析（PerceptionTarget dataclass）、候选排序/round-robin/自洽检查/姿态变体 |
| `src/real_piper_perception/scripts/piper_ssik_ik.py` | 从仿真移植的 ssik 子进程工具（--matrix/--seed/--max-solutions） |
| `src/real_piper_perception/real_piper_perception/grasp_precheck.py` | 4 段 dry-run 预检（MoveIt plan + ssik 种子 + FK 回验 + analytic-only 门控） |
| `src/real_piper_perception/real_piper_perception/grasp_execute.py` | 执行序列（预位/approach/grasp/闭合夹爪/抬升，笛卡尔直线段） |
| `src/real_piper_perception/real_piper_perception/grasp_verify.py` | 夹爪反馈成功判定（纯函数） |
| `src/real_piper_perception/real_piper_perception/grasp_executor.py` | 主节点：服务/键盘触发、编排、grasp_result.json、结果广播 |
| `src/real_piper_perception/test/test_grasp_candidates.py` | 候选模块单测 |
| `src/real_piper_perception/test/test_grasp_transform.py` | 变换模块单测 |
| `src/real_piper_perception/test/test_grasp_verify.py` | 判定模块单测 |
| `src/real_piper_perception/test/fixtures/sample_grasps.csv` | CSV 解析测试夹具（真实列格式） |
| `src/real_piper_perception/launch/grasp_execute.launch.py` | 一键启动（含键盘触发） |

---

### Task 1: 变换链调研确认（T1）

**Files:**
- Create: `docs/grasp_executor_transform_notes.md`（调研产出文档）

**Interfaces:**
- Consumes: 仿真代码 `~/piper_graspnet/gazebo_piper_pick_place/`、`real_piper_graspnet/graspnet-baseline/inference_from_camera.py`
- Produces: 三个常量值（`CAMERA_TO_WORLD_ROTATION_R`、`GRASPNET_TO_PIPER_TCP_R`、`MIN_REACHABLE_TCP_GRASP_Z`）+ 变换公式，供 Task 3 使用

- [ ] **Step 1: 读 inference_from_camera.py 的 select_object_grasp_targets（约 680-733 行）**

确认以下问题并记录到调研文档：
1. CSV 中 `r00..r22` 的坐标系：是 GraspNet 原始抓取系旋转，还是已经过 `CAMERA_TO_WORLD_ROTATION_R` 变换的旋转？读代码确定 `grasp_array[idx][4:13]` 的赋值来源
2. `camera_x/camera_y/camera_z` 是相机系位置（确认无变换）
3. `CAMERA_TO_WORLD_ROTATION_R`、`GRASPNET_TO_PIPER_TCP_R` 两个常量的具体数值（从代码常量表拷贝）

- [ ] **Step 2: 读仿真 graspnet_perception_loader.cpp（97 行）**

确认仿真侧 CSV→world 系的变换位置（`candidates.world` 怎么来的），记录其与真机 handeye 变换的对应关系。

- [ ] **Step 3: 读 camera_data/handeye_eye_to_hand.yaml 与 calibration.yaml**

确认标定文件格式（旋转表示：四元数/矩阵；平移单位：米/毫米；eye-to-hand 语义：camera→base 还是 base→camera）。

- [ ] **Step 4: 读仿真 pick_and_place.cpp 的 kMinReachableTcpGraspZ / kExecutionPlanningTime / kMoveGroupName 常量**

记录到调研文档（这些将移植到真机模块）。

- [ ] **Step 5: 写调研文档**

`docs/grasp_executor_transform_notes.md` 必须包含：① 变换公式（含每步的矩阵/四元数）② 三个常量数值表 ③ CSV rotation 语义结论 ④ handeye 解析伪代码。提交时附文档路径。

---

### Task 2: 环境准备与官方启动链冒烟（T2）

**Files:**
- Modify: 无（仅验证环境）

**Interfaces:**
- Consumes: 官方教程 `src/piper_ros/src/piper_moveit/README.md §3.2`
- Produces: 可用的启动环境（后续任务全部依赖）

- [ ] **Step 1: 检查依赖**

```bash
dpkg -l | grep -E "ros-humble-moveit" | head -5   # moveit 二进制包
dpkg -l | grep -E "ros-humble-ros2-control|joint-trajectory-controller" | head -5
python3 -c "import piper_sdk, ssik; print('sdk ok')"   # 已确认可用
source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E "piper_with_gripper_moveit|piper|real_piper_perception"
```
缺失的 moveit 依赖：`sudo apt install ros-humble-moveit* ros-humble-ros2-controllers ros-humble-ros2-control`

- [ ] **Step 2: CAN 激活**

```bash
cd ~/real_piper_graspnet/src/piper_ros && bash can_activate.sh can0 1000000
candump can0 -n 1   # 验证 CAN 帧可读（需机械臂上电）
```

- [ ] **Step 3: 三启动件冒烟（需真机在位）**

终端 1：`ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2`
终端 2：`ros2 launch piper_with_gripper_moveit demo.launch.py`
验证：rviz 出现模型；`ros2 topic echo /joint_states_feedback` 有真机关节数据；RViz 手动 Plan&Execute 一个简单位姿，机械臂以低速动作（**先手动确认官方链路可用，再写代码**）。

- [ ] **Step 4: 记录冒烟结果**

把三启动件的实际命令、Topic 名、关节名写入调研文档附录。

---

### Task 3: grasp_transform.py 变换模块

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/grasp_transform.py`
- Create: `src/real_piper_perception/test/test_grasp_transform.py`
- Create: `src/real_piper_perception/test/fixtures/handeye_fixture.yaml`

**Interfaces:**
- Consumes: Task 1 调研文档（常量值、handeye 格式、变换公式）
- Produces:
  - `load_handeye(path: str) -> np.ndarray` — 返回 4x4 齐次变换 T_base_camera（矩阵）
  - `camera_pose_to_base(xyz: tuple[float,float,float], r: tuple[float,...9], handeye: np.ndarray) -> tuple[np.ndarray, np.ndarray]` — (位置 3-vec, 旋转 3x3) 基座系
  - `graspnet_to_tcp_rotation(r_base: np.ndarray) -> np.ndarray` — 应用 `GRASPNET_TO_PIPER_TCP_R`（右乘）
  - 常量模块级：`CAMERA_TO_WORLD_ROTATION_R`、`GRASPNET_TO_PIPER_TCP_R`（Task 1 确认值）

- [ ] **Step 1: 写失败测试**

```python
# test_grasp_transform.py
import numpy as np
from real_piper_perception.grasp_transform import (
    load_handeye, camera_pose_to_base, graspnet_to_tcp_rotation,
    GRASPNET_TO_PIPER_TCP_R)

def test_load_handeye_returns_4x4():
    m = load_handeye('test/fixtures/handeye_fixture.yaml')
    assert m.shape == (4, 4)
    assert np.allclose(m[3], [0, 0, 0, 1])

def test_camera_to_base_translation():
    # handeye 平移 (0.1, 0.2, 0.3)，单位向基座平移
    m = load_handeye('test/fixtures/handeye_fixture.yaml')
    pos, rot = camera_pose_to_base((0.0, 0.0, 0.0), np.eye(3).flatten(), m)
    assert np.allclose(pos, [0.1, 0.2, 0.3], atol=1e-6)
```
fixture `handeye_fixture.yaml` 按 Task 1 确认的格式编写（含纯平移 T=(0.1,0.2,0.3) 的样例）。

- [ ] **Step 2: 运行确认失败**

```bash
cd ~/real_piper_graspnet && source /opt/ros/humble/setup.bash
python3 -m pytest src/real_piper_perception/test/test_grasp_transform.py -v
```
Expected: FAIL（ImportError: grasp_transform 不存在）

- [ ] **Step 3: 实现 grasp_transform.py**

按 Task 1 调研文档的公式实现（handeye yaml 解析 + 变换组合 `T_base_object = T_base_camera @ T_camera_object` + TCP 旋转映射右乘）。旋转矩阵从 CSV 9 个 r 值按行主序组装。

- [ ] **Step 4: 运行测试通过**

```bash
python3 -m pytest src/real_piper_perception/test/test_grasp_transform.py -v
```
Expected: PASS（3 个测试）

- [ ] **Step 5: 提交**

```bash
git add src/real_piper_perception/ && git commit -m "feat: grasp transform module with handeye loading"
```

---

### Task 4: grasp_candidates.py 候选解析与筛选

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/grasp_candidates.py`
- Create: `src/real_piper_perception/test/test_grasp_candidates.py`
- Create: `src/real_piper_perception/test/fixtures/sample_grasps.csv`（3 行真实格式：2 个近分高、1 个低分，其中 2 个姿态同桶、1 个不同桶）

**Interfaces:**
- Consumes: CSV 24 列格式（Global Constraints）
- Produces:
  - `@dataclass PerceptionTarget`：`name, xyz_camera: np.ndarray(3,), rotation: np.ndarray(3,3), score: float, width: float, center_camera: np.ndarray(3,), link6_grasp_z: float, tcp_forward_z: float, source_dist_xy: float`
  - `parse_grasp_csv(path: str) -> list[PerceptionTarget]`（空文件/坏行跳过并告警）
  - `rank_candidates(cands: list[PerceptionTarget]) -> list[PerceptionTarget]` — 分数降序 + 8 姿态桶 round-robin（桶号 = 四元数符号位编码的整数，直接用 rotation 的 3 个元素符号：`b = (r[0,0]>=0) | (r[0,1]>=0)<<1 | (r[0,2]>=0)<<2`——用矩阵第一行符号近似方向）
  - `self_consistency(c: PerceptionTarget, max_center_dist: float = 0.08) -> bool` — `||center_camera[:2] - xyz_camera[:2]|| <= max_center_dist`（替代 Gazebo 门控）

- [ ] **Step 1: 写失败测试**

```python
# test_grasp_candidates.py
import numpy as np
from real_piper_perception.grasp_candidates import (
    parse_grasp_csv, rank_candidates, self_consistency, PerceptionTarget)

def test_parse_csv_row_count():
    cands = parse_grasp_csv('test/fixtures/sample_grasps.csv')
    assert len(cands) == 3
    c = cands[0]
    assert c.rotation.shape == (3, 3)
    assert c.width > 0

def test_rank_round_robin_diverse():
    cands = parse_grasp_csv('test/fixtures/sample_grasps.csv')
    ranked = rank_candidates(cands)
    # 相邻候选来自不同姿态桶（构造数据保证）
    def bucket(c):
        r = c.rotation
        return (r[0, 0] >= 0) | ((r[0, 1] >= 0) << 1) | ((r[0, 2] >= 0) << 2)
    assert bucket(ranked[0]) != bucket(ranked[1])

def test_self_consistency_gate():
    good = PerceptionTarget(name='t', xyz_camera=np.array([0.0, 0.0, 0.4]),
                            rotation=np.eye(3), score=0.9, width=0.05,
                            center_camera=np.array([0.01, -0.01, 0.4]), link6_grasp_z=0.0,
                            tcp_forward_z=1.0, source_dist_xy=0.0)
    assert self_consistency(good)
    bad = PerceptionTarget(name='t', xyz_camera=np.array([0.0, 0.0, 0.4]),
                           rotation=np.eye(3), score=0.9, width=0.05,
                           center_camera=np.array([0.5, 0.0, 0.4]), link6_grasp_z=0.0,
                           tcp_forward_z=1.0, source_dist_xy=0.0)
    assert not self_consistency(bad)
```
fixture `sample_grasps.csv` 手工构造 3 行（列序严格按 24 列；rotation 用正交矩阵，如单位阵和绕 z 的 90° 旋转，符号位组合覆盖不同桶——注：绕 z 90° 矩阵 [[0,-1,0],[1,0,0],[0,0,1]] 按桶公式实为桶 5，非桶 1）。

- [ ] **Step 2: 运行确认失败** → `python3 -m pytest .../test_grasp_candidates.py -v`（FAIL: no module）

- [ ] **Step 3: 实现 grasp_candidates.py**（按 Interfaces 签名；解析按列序位置解析，不依赖列名）

- [ ] **Step 4: 运行通过**（3 个测试 PASS）

- [ ] **Step 5: 提交**

---

### Task 5: piper_ssik_ik.py 移植

**Files:**
- Create: `src/real_piper_perception/scripts/piper_ssik_ik.py`（从仿真 `~/piper_graspnet/gazebo_piper_pick_place/src/piper_gazebo/scripts/piper_ssik_ik.py` 原样拷贝，路径参数化）

**Interfaces:**
- Consumes: ssik 3.3.1（已装）
- Produces: 子进程 CLI：`--matrix <16 逗号浮点(行主序 4x4)> --seed <6 逗号浮点> --max-solutions <int> [--respect-limits]`；stdout 每行 `q0..q5 residual`；退出码 0/2/3/4/5

- [ ] **Step 1: 拷贝脚本**

```bash
cp ~/piper_graspnet/gazebo_piper_pick_place/src/piper_gazebo/scripts/piper_ssik_ik.py \
   src/real_piper_perception/scripts/piper_ssik_ik.py
```

- [ ] **Step 2: 冒烟验证（无真机依赖，纯数学）**

```bash
cd ~/real_piper_graspnet && source /opt/ros/humble/setup.bash
python3 src/real_piper_perception/scripts/piper_ssik_ik.py \
  --matrix "1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1" --seed "0,0,0,0,0,0" --max-solutions 4
```
Expected: 至少 1 行解输出，residual ~1e-10；且 `echo $?` = 0

- [ ] **Step 3: 验证解析解经 FK 回验**

写 5 行内验证：对每个输出解，构造关节角向量，用 piper URDF（`src/piper_ros/src/piper_description` 的 xacro 转 urdf 或直接用 ssik 的 FK？——用 ssik 输出 residual 即 FK 残差，打印确认 <1e-8。**不引入 URDF 解析依赖**）。

- [ ] **Step 4: 提交**

---

### Task 6: grasp_precheck.py 4 段预检

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/grasp_precheck.py`

**Interfaces:**
- Consumes: Task 3 变换（基座系位姿）、Task 5 ssik 子进程、move_group（`move_group_py.MoveGroupInterface`）
- Produces:
  - `class Prechecker`：`__init__(self, move_group, ssik_script: str, params: dict)`；`precheck_pose(self, pose: Pose, label: str) -> bool`（OMPL plan + ssik 种子 + FK 回验 + analytic-only 门控，移植仿真 planFullPoseWithSeeds 逻辑）；`precheck_stages(self, poses: dict[str, Pose], scan_only_report: bool = True) -> tuple[bool, dict]`（4 段 AND；返回 (全过?, 各段统计 {approach/grasp/lift: pass/fail})）
  - `make_stage_poses(self, candidate: PerceptionTarget, base_pose, params) -> dict[str, Pose]`（approach=grasp 位+Z approach_clearance；grasp=TCP 位；lift=grasp 位+Z open_gripper_lift_height）

- [ ] **Step 1: 移植 planFullPoseWithSeeds 逻辑**（pick_and_place.cpp:183-307）

Python 等价实现，要点：
1. `move_group.set_planning_time(min(planning_time, 1.0))`
2. 子进程调 ssik（`subprocess.run`，timeout 5s）→ 逐解：`set_joint_value_target(joints)` + `move_group.plan()`，FK 回验用 `move_group.get_current_state().compute_fk()`（位置误差<0.015m 且角度误差<0.1rad）
3. analytic-only 门控：`precheck` 阶段 label 含 `'visual-center'` 且 `analytic_only_precheck=True` → ssik 无解直接返回 False
4. 预检后恢复 planning_time

- [ ] **Step 2: scan_only 报告输出**

`precheck_stages` 返回统计 `{tested, approach_pass, grasp_pass, lift_pass}`（移植仿真 scan summary 打印格式），供 Task 9 干跑验证使用。

- [ ] **Step 3: 语法与导入检查**

```bash
python3 -m py_compile src/real_piper_perception/real_piper_perception/grasp_precheck.py
```
（MoveIt 依赖无法在无环境时单测——集成验证推迟到 Task 9 干跑）

- [ ] **Step 4: 提交**

---

### Task 7: grasp_execute.py 执行序列 + grasp_verify.py 判定

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/grasp_execute.py`
- Create: `src/real_piper_perception/real_piper_perception/grasp_verify.py`
- Create: `src/real_piper_perception/test/test_grasp_verify.py`

**Interfaces:**
- Consumes: Task 4 候选、Task 6 预检通过的位姿、move_group、`/joint_states_feedback`（rclpy 订阅）
- Produces:
  - `verify_grasp(actual_width: float, cmd_width: float, detect_threshold: float) -> bool` — `actual_width < cmd_width - detect_threshold`
  - `class GraspExecutor`: `__init__(self, node, move_group, params)`；`execute_pick(self, stage_poses: dict[str, Pose], grasp_width: float) -> dict`（返回 {status: 'success'|'grasp_empty'|'failed', actual_width, stage}）
  - 执行序列：`moveLinear(pose)`（笛卡尔 compute_cartesian_path，approach/抬升段）与 `movePose(pose)`（OMPL plan+execute，预位/grasp 段）两个内部方法，均应用 velocity_scale

- [ ] **Step 1: 写失败测试（判定纯函数）**

```python
# test_grasp_verify.py
from real_piper_perception.grasp_verify import verify_grasp

def test_grasp_success_when_blocked():
    # 命令闭合 60mm，实际只到 25mm → 夹住物体
    assert verify_grasp(actual_width=25.0, cmd_width=60.0, detect_threshold=10.0)

def test_grasp_empty_when_full_close():
    # 实际接近命令值 → 抓空
    assert not verify_grasp(actual_width=58.0, cmd_width=60.0, detect_threshold=10.0)

def test_edge_threshold():
    assert not verify_grasp(actual_width=50.0, cmd_width=60.0, detect_threshold=10.0)
    assert verify_grasp(actual_width=49.9, cmd_width=60.0, detect_threshold=10.0)
```

- [ ] **Step 2: 运行确认失败** → `python3 -m pytest .../test_grasp_verify.py -v`

- [ ] **Step 3: 实现 grasp_verify.py**（3 行纯函数）

- [ ] **Step 4: 实现 grasp_execute.py**

- `moveLinear`：`compute_cartesian_path`（eef_step 0.01, jump_threshold 0.0, avoid_collisions=True），失败回退 OMPL plan；执行前 `set_max_velocity_scaling_factor(velocity_scale)`
- 夹爪闭合：`joint7` 目标 = `grasp_width + gripper_margin`（弧度，与 piper 驱动约定一致——Task 2 冒烟时确认 joint7 命令单位）；`move_group.plan()`+`execute()`
- 判定：夹爪闭合后 sleep 0.5s，读最新 `/joint_states_feedback`（node 订阅缓存）的 gripper 关节值 → `verify_grasp`
- 失败路径：任一段异常 → 返回 failed，不再继续

- [ ] **Step 5: 运行测试通过**（3 个判定测试 PASS）

- [ ] **Step 6: 提交**

---

### Task 8: grasp_executor.py 主节点 + launch

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/grasp_executor.py`
- Create: `src/real_piper_perception/launch/grasp_execute.launch.py`
- Modify: `src/real_piper_perception/setup.py`（entry_points 加 `grasp_executor = real_piper_perception.grasp_executor:main`）

**Interfaces:**
- Consumes: Task 3-7 全部模块、`perception_grasps.csv`（感知侧产物）
- Produces:
  - 服务 `grasp_execute`（`std_srvs/Trigger`）：回调编排 `parse → rank → self_consistency → precheck → execute → verify`
  - 参数：`velocity_scale=0.1, scan_only=false, handeye_yaml=..., approach_clearance=0.05, lift_z=0.10, open_gripper_lift_height=0.10, gripper_margin=0.005, gripper_detect_threshold=0.01, graspnet_6d_max_candidates=1, max_orientation_variants=2, enable_piper_analytic_ik_planning=true, graspnet_6d_analytic_only_precheck=true, piper_analytic_ik_script=..., planning_time_exec=10.0`
  - 结果写 `shared_camera_data/grasp_result.json`：`{timestamp, success, status, stage, actual_width, cmd_width, candidate_index, message}`
  - 发布 `/grasp_result`（std_msgs/String, JSON）
  - 键盘触发：`rclpy` spin 外线程 `input()` 读 'g' 触发

- [ ] **Step 1: 写主节点骨架**（参数声明、服务、键盘线程、模块编排；无 MoveIt 细节——调 Task 6/7 接口）

- [ ] **Step 2: 语法检查 + 无真机启动测试**

```bash
python3 -m py_compile src/real_piper_perception/real_piper_perception/grasp_executor.py
cd ~/real_piper_graspnet && source /opt/ros/humble/setup.bash
colcon build --packages-select real_piper_perception --symlink-install
ros2 run real_piper_perception grasp_executor --ros-args -p scan_only:=true
```
Expected: 节点启动、参数就绪、`ros2 service list | grep grasp_execute` 可见、Ctrl+C 干净退出

- [ ] **Step 3: 写 launch 文件**

`grasp_execute.launch.py`：起 grasp_executor（参数透传，默认 scan_only:=true，注释说明切 false 需二次确认）。

- [ ] **Step 4: 提交**

---

### Task 9: 阶段 1 —— scan_only 干跑验证

**Files:**
- Modify: 无（验证任务）

**Interfaces:**
- Consumes: 感知侧运行产物（perception_grasps.csv）、Task 8 节点
- Produces: 干跑验收报告（写入 `docs/grasp_executor_test_log.md`）

- [ ] **Step 1: 启动感知与 MoveIt（无执行）**

终端 1：`./run_d435i_bridge.sh`
终端 2：`./run_graspnet_d435i.sh gui`（感知产出 CSV，物体放 ROI 内）
终端 3：`ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2`（真机保持不动）
终端 4：`ros2 launch piper_with_gripper_moveit demo.launch.py`

- [ ] **Step 2: 触发干跑**

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger {}
```
Expected: 服务返回 success=true（干跑即预检全过），日志打印候选数、各段通过统计；**机械臂全程不动**

- [ ] **Step 3: 验证报告合理**

对照 Task 1 调研文档：出现不可达候选（朝下×左区）时 scan 统计 approach/grasp/lift pass 数递减，与仿真 08-06 结论一致。记录到测试日志。

- [ ] **Step 4: 异常路径验证**：清空/删除 CSV → 服务返回 failure + 明确 message

---

### Task 10: 阶段 2 —— 低速单段执行验证

**Files:**
- Modify: 无（验证任务）

**Interfaces:**
- Consumes: Task 9 环境、Task 7 执行模块（velocity_scale=0.1）
- Produces: 阶段 2 验收记录（测试日志追加）

- [ ] **Step 1: 手动预位**

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger {}  # 临时改 scan_only=false 且只执行 approach
```
为安全，本阶段在 grasp_executor 加临时参数 `dry_run_max_stage=approach`（默认 -1 全执行，仅本阶段验证用，提交前删除或保留参数化均可——**保留参数化，默认全执行**）。

- [ ] **Step 2: 验证桥接**

Expected: 机械臂以低速从预位规划并移动到 approach 位（物体上方 ~5cm），到位后停止；日志显示笛卡尔直线段成功；无异常报警。

- [ ] **Step 3: 人工检查**：approach 位姿与 rviz 模型一致；机械臂停止后无漂移。

---

### Task 11: 阶段 3 —— 完整抓取验证

**Files:**
- Modify: 无（验收任务）

**Interfaces:**
- Consumes: 全部前置任务
- Produces: 阶段 3 验收记录 + 最终测试日志

- [ ] **Step 1: 完整配置启动**

`grasp_execute.launch.py` 参数 `scan_only:=false`；物体放 ROI 中心。

- [ ] **Step 2: 触发完整抓取**

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger {}
```

- [ ] **Step 3: 验收清单**

1. 序列正确：预位 → approach（笛卡尔直线）→ grasp（OMPL）→ 闭合夹爪 → 判定 → 抬升
2. `grasp_result.json` 字段完整、status 与实际一致（夹住=success / 抓空=grasp_empty）
3. 夹爪判定阈值与实物一致（真机标定 gripper 读数：空载全关 vs 夹持 5cm 物体）
4. 结果落盘 + `/grasp_result` 话题广播
5. 重复 3 次：至少 1 次 success（抓取验证通过标准）

- [ ] **Step 4: 写最终测试日志**（`docs/grasp_executor_test_log.md`：三阶段结果、标定数据、问题记录）

- [ ] **Step 5: 提交**（若代码有本阶段微调）

---

## Self-Review 记录

**Spec 覆盖核对**：
- 5.1 节点/5.2 接口 → Task 8 ✓；5.3 候选处理 → Task 4 ✓；5.4 预检（OMPL+ssik+门控）→ Task 5/6 ✓；5.5 执行序列 → Task 7 ✓；5.6 判定 → Task 7 ✓；5.7 参数 → Task 8 ✓；5.8 变换链 → Task 1/3 ✓
- 六 错误处理与安全（scan_only/velocity/使能/失败保持姿态）→ Task 6/7/8 ✓
- 七 三阶段测试 → Task 9/10/11 ✓
- 八 T1~T10 → 映射到 Task 1-11 ✓（T1=Task1, T2=Task2, T3=Task3, T4=Task4, T5=Task6(预检), T6=Task7, T7=Task7, T8=Task8, T9=Task9, T10=Task10）
- 风险表 → Task 1（CSV rotation）、Task 2（单位标定）、Task 6/9（规划空洞）、Task 3（标定回退）✓

**类型一致性**：`PerceptionTarget` 字段在 Task 4 定义、Task 6/7 消费，签名一致；`verify_grasp` 在 Task 7 定义并测试；`Prechecker.precheck_stages` 返回 dict 键名在各任务一致（approach/grasp/lift）。

**注意事项**：本计划中 MoveIt/真机部分无法纯 TDD（无硬件环境），以"语法检查 + scan_only 集成验证"替代；纯逻辑模块（transform/candidates/verify）严格执行 TDD。非 git 仓库——提交步骤改为记录变更（`docs/` 变更日志），不执行 git 命令。
