# 真机抓取验证：grasp_executor 设计文档与任务方案

日期：2026-08-06
状态：已批准（最终版：MoveIt2 全链路 + piper_sdk 参考层）

## 一、背景与目标

real_piper_graspnet 的**感知侧已完整可用**（D435i 桥接 → docker GraspNet 推理 → 动态 ROI →
双视图可视化，2026-08-06 验证），但**执行侧完全没有代码**。本任务仿照仿真参考实现
（`~/piper_graspnet/gazebo_piper_pick_place` 的 `graspnet_6d_grasp_planner.cpp` +
`pick_and_place.cpp`），编写真机**单次抓取验证**代码：

```
感知 → 读 perception_grasps.csv → 候选筛选 → 4段预检 → MoveIt 执行 → 夹爪反馈判定成功与否
```

不含放置、不含多物体闭环（pick_state）、不含自动循环。

## 二、已确认的需求（用户确认）

1. **范围**：单次抓取验证（服务/按键触发一次，返回成功/失败）
2. **执行**：MoveIt2 规划执行——官方真机路径
3. **成功判定**：夹爪位置反馈（实际开度 < 命令宽度 - 阈值 ⇒ 夹住物体）
4. **代码参考**：piper_ros 官方 MoveIt 控制真机教程 + piper_sdk 0.6.2 demo/API
   （`C_PiperInterface_V2`：`EnablePiper` 等待模式、`GripperCtrl` 单位 0.001mm、
   `GetArmGripperMsgs` 监控）

## 三、关键调研结论

### 3.1 仿真参考实现（graspnet_6d_grasp_planner.cpp）

- 候选处理：视觉中心距 Gazebo 真值距离升序 + 分数降序 → 8 姿态桶 round-robin →
  逐候选做**视觉中心门控**（0.08m，真机无 Gazebo 真值 → 改用候选自洽检查）→
  姿态变体（calib_18，`max_orientation_variants` 参数）
- **4 段 dry-run 预检**（MoveIt `plan()` 不执行，AND 语义）：approach → grasp →
  open-gripper lift → place；本任务单抓取去掉 place 段
- 参数：`graspnet_6d_max_candidates`（默认 1，上限 300）、`scan_only` 干跑模式

### 3.2 真机 MoveIt 控制（piper_ros 官方教程 piper_moveit/README.md §3.2）

```
1. bash can_activate.sh can0 1000000                      # CAN 激活
2. ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
   # 真机驱动；官方 remap: /joint_ctrl_single → /joint_states，
   # 即订阅 /joint_states 的关节位置即驱动真机
3. ros2 launch piper_with_gripper_moveit demo.launch.py
   # move_group + rviz + ros2_control(mock_components/GenericSystem)，
   # 轨迹播放发布到 /joint_states → 真机驱动执行
```

关键结论：**MoveIt 全链路控制真机是官方支持路径**（mock 控制器 + remap 桥接），
无需自写 ros2_control 硬件接口；`piper_humble` 是空包（无硬件接口插件），不要走那条路。

### 3.3 piper_sdk 0.6.2（参考层）

- 已安装：`<python-site-packages>/piper_sdk/`
- `C_PiperInterface_V2("can0").ConnectPort()`；`while not EnablePiper(): sleep(0.01)`
  （参考 `demo/V2/piper_ctrl_enable.py`）
- `GripperCtrl(range, speed, 0x01, 0)`：range 单位 **0.001mm**（0.05m = 50000）
  （参考 `demo/V2/piper_ctrl_gripper.py`）
- `GetArmGripperMsgs()`：夹爪位置反馈（判定用，独立于 MoveIt 链路）
- `GetArmStatus()/GetArmEndPoseMsgs()/isOk()`：状态监控（参考 `detect_arm.py`）
- 用途：状态读取、使能流程、夹爪 API 的单位/模式参考——不直接替代 MoveIt 执行

## 四、架构

```
┌─ 启动三件套（官方教程）────────────────────────────────────────┐
│ can_activate.sh → start_single_piper.launch.py → demo.launch.py│
└──────────────┬─────────────────────────────────────────────────┘
               │ /joint_states（MoveIt 轨迹）→ 真机
               │ /joint_states_feedback（关节+夹爪反馈）
┌──────────────▼─────────────────────────────────────────────────┐
│ grasp_executor 节点（src/real_piper_perception）                │
│                                                                │
│ 触发: grasp_execute 服务(std_srvs/Trigger) / 键盘              │
│   │                                                           │
│   1. 读最新 perception_grasps.csv + last_validation.json      │
│   2. 候选筛选: 分数排序 → 8姿态桶round-robin → 自洽检查        │
│      → 姿态变体(calib_18, 参数限数)                            │
│   3. 4段 dry-run 预检(MoveIt plan): approach→grasp→open-lift  │
│   4. 执行(MoveIt plan+execute, velocity_scale 默认0.1):       │
│      预位→接近→抓取位→闭合夹爪(joint7)→抬升                   │
│   5. 判定: /joint_states_feedback.gripper 实际开度             │
│      (备选: piper_sdk GetArmGripperMsgs 直连)                  │
│   6. 结果: 服务响应 + shared_camera_data/grasp_result.json    │
│                                                                │
│ 坐标变换: handeye_eye_to_hand.yaml(eye-to-hand)                │
│   + CAMERA_TO_WORLD_ROTATION_R + GRASPNET_TO_PIPER_TCP_R       │
│   (从仿真代码移植; 缺标定回退常量)                              │
└────────────────────────────────────────────────────────────────┘
```

## 五、组件设计

### 5.1 grasp_executor.py（新文件，Python 节点）

加入现有包 `real_piper_perception`（与 d435i_graspnet_bridge.py 同包），setup.py 加
entry point。MoveIt2 Python API（`move_group_py` / MoveGroupInterface）与 C++ 同能力。

### 5.2 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `grasp_execute` | 服务 `std_srvs/Trigger` | 触发一次完整抓取；响应 success+message |
| `/grasp_result` | topic（std_msgs/String） | 结果广播（JSON） |
| `shared_camera_data/grasp_result.json` | 文件 | 结果落盘（与感知文件协议一致） |
| 参数 | 参数服务器 | 见 5.7 |
| 依赖 | Python 包 | `ssik`(3.3.1, 已装)、`moveit`、`piper_ros` |

### 5.3 候选处理（移植仿真逻辑）

1. 读 CSV 行（24 列，列序与 C++ 解析严格对应；2026-08-06 实现时以真实管线文件核实为 24 列）→ `PerceptionTarget`（位置/旋转/宽度/分数/物体中心）
2. 分数降序 + 8 姿态桶 round-robin 重排
3. **自洽检查替代 Gazebo 门控**：`object_center` 与 grasp center 的横向距离合理性阈值
4. 姿态变体 `graspNetTcpOrientationVariants`（从仿真移植），`max_orientation_variants` 参数

### 5.4 4 段预检（dry-run）—— OMPL 全管线 + ssik 解析种子

`MoveGroupInterface.plan()` 不执行，AND 语义（approach → grasp → open-lift，无 place 段）。
**预检是完整 MoveIt/OMPL 规划管线（碰撞感知采样规划，非纯笛卡尔）**，并移植仿真参考的
analytic IK 增强（`pick_and_place.cpp:183 planFullPoseWithSeeds`）：

```
1. move_group.plan()：OMPL 采样规划（RRTConnect 等，来自 moveit planning pipeline）
   + 碰撞检测 + 轨迹后处理
2. ssik 解析 IK 种子层（默认开启 enable_piper_analytic_ik_planning）
   ├─ 子进程调 piper_ssik_ik.py（--matrix 目标位姿 / --seed 当前关节角）
   │   → ssik.prebuilt.agilex.piper_ik 解析求解（ssik 3.3.1 已装，独立包，
   │     与 Pinocchio 无关；Pinocchio 未装且不需要）
   ├─ FK 回验门限：pos_err < 1.5cm 且 angle_err < 0.1rad（防脚本残差/URDF 偏差）
   └─ 通过后以"关节解"为规划目标交给 OMPL —— 替换 OMPL 默认的 IK 采样阶段
3. analytic-only 门控（默认开启 graspnet_6d_analytic_only_precheck）
   └─ 预检阶段禁止 KDL/数值 IK 兜底：ssik 失败即真不可达（防假阳性，
      仿真 08-06 已知"朝下×左区空洞"即此现象；真机上防假阳性是安全关键）
4. 预检用短规划时间 min(planning_time, 1.0)；ssik 导入失败(退出码2)按不可达处理
5. 笛卡尔规划（compute_cartesian_path）只用于执行段的直线运动（approach 下探/抬升，
   同仿真 moveLinear），不用于预检
```

### 5.5 执行序列

```
home 预位 → approach → grasp → 闭合夹爪(joint7 目标 = CSV width + gripper_margin)
→ 等待稳定 0.5s → 读夹爪反馈 → 判定 → 抬升 lift_z
```

### 5.6 成功判定

- 命令闭合量 `W_cmd = width + gripper_margin`
- 读反馈开度 `W_actual`（/joint_states_feedback 的 gripper 关节；或 sdk 直连）
- `W_actual < W_cmd - detect_threshold` ⇒ 夹住物体（成功）；`W_actual ≈ W_cmd` ⇒ 抓空

### 5.7 参数清单

```
velocity_scale(0.1)  scan_only(false)  handeye_yaml(camera_data/handeye_eye_to_hand.yaml)
approach_clearance   lift_z            gripper_margin   gripper_detect_threshold
graspnet_6d_max_candidates(默认1)  max_orientation_variants(2)  back_place_x/y(预留,本任务不用)
enable_piper_analytic_ik_planning(true)  graspnet_6d_analytic_only_precheck(true)
piper_analytic_ik_script(piper_ssik_ik.py 路径)  piper_analytic_ik_max_solutions(8)
piper_analytic_ik_max_solution_plans(6)  planning_time(预检1s/执行kExecutionPlanningTime)
```

### 5.8 坐标变换链（实现第一步必须确认的点）

仿真侧变换（inference_from_camera.py 的 select_object_grasp_targets）：
`world_tcp_rotation = CAMERA_TO_WORLD_ROTATION_R @ rotation @ GRASPNET_TO_PIPER_TCP_R`。
**实现时先读该函数确认 CSV 中 r00..r22 的坐标系语义**（GraspNet 原始系还是已变换系），
再移植：相机系 → handeye 标定(eye-to-hand) → 基座系 → TCP。缺标定文件回退仿真常量
（确认标定文件格式：yaml 四元数/旋转矩阵/平移）。

## 六、错误处理与安全

- `scan_only`：只预检输出可达性统计（tested/approach_pass/grasp_pass/lift_pass），臂不动
- `velocity_scale` 默认 0.1（低速安全）
- 使能检查（arm_status / sdk isOk）；异常码记录
- 任一执行段失败 → 服务返回失败、机械臂保持当前姿态、不继续后续段
- 夹爪判定超时保护（等待反馈上限）

## 七、测试计划（三阶段）

| 阶段 | 内容 | 验收 |
|---|---|---|
| 1. 干跑 | `scan_only`：CSV 解析、变换、预检（臂不动） | 可达性报告合理；与仿真 08-06 结论（部分候选不可达）一致 |
| 2. 低速单段 | 只执行 approach（不接近物体） | MoveIt↔真机桥接成功，臂缓慢到位，无异常 |
| 3. 完整抓取 | 放物体 → 触发 → 抓取 → 判定 → 结果落盘 | grasp_result.json 正确；成功/抓空判定符合实际 |

## 八、具体任务方案（任务分解）

| # | 任务 | 验收标准 |
|---|---|---|
| T1 | 精读仿真变换链：select_object_grasp_targets 的 CSV rotation 语义、GRASPNET_TO_PIPER_TCP_R/CAMERA_TO_WORLD_ROTATION_R 常量值、handeye 标定文件格式 | 写出变换链伪代码 + 常量表，文档化 |
| T2 | 环境准备：`ros-humble-moveit*` 依赖检查、can0 激活、三启动件冒烟（rviz 手动动臂） | 官方教程 3.2 在真机跑通 |
| T3 | grasp_executor.py 骨架：节点/参数/服务/CSV 解析/变换模块 | 服务可调用，CSV 解析出候选并打印 |
| T4 | 候选筛选模块（排序/round-robin/自洽检查/姿态变体） | 单元测试：排序与重排逻辑正确 |
| T5 | 4 段预检（MoveIt plan dry-run + scan_only 报告） | 阶段 1 验收 |
| T6 | 执行序列（预位/接近/抓取/闭合夹爪/抬升） | 阶段 2 验收 |
| T7 | 成功判定（夹爪反馈 + 超时保护） | 单元测试阈值逻辑 |
| T8 | 结果上报（服务响应/grasp_result.json） | 结果文件格式正确 |
| T9 | launch 文件（grasp_execute.launch.py）+ 键盘触发 | 一键启动 |
| T10 | 完整真机抓取验证（阶段 3） | 阶段 3 验收 |

依赖关系：T1→T3→T4→T5→T6→T7→T8→T9；T2 与 T1 可并行；T10 依赖全部。

## 九、非目标

- 不做放置段与多物体 pick_state 闭环（后续任务）
- 不做自动循环抓取（触发式）
- 不写 ros2_control 硬件接口插件（官方 mock+remap 路径已够）
- 不修改感知侧与推理侧代码

## 十、风险与缓解

| 风险 | 缓解 |
|---|---|
| CSV rotation 坐标系与仿真假设不一致 | T1 先行确认，文档化后再编码 |
| 夹爪单位换算错误（moveit joint7 ↔ sdk 0.001mm） | 参考官方 gripper demo + 真机读数标定 |
| MoveIt 规划失败率（朝下×左区空洞，仿真已知） | ssik 种子 + 候选遍历 + round-robin + max_candidates 参数调 |
| ssik 不可用 | 已确认 3.3.1 本机可用；导入失败按不可达处理（退出码 2 路径） |
| 标定文件缺失/格式不明 | 回退仿真常量 + 显式日志告警 |
