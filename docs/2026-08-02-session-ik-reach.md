# 会话重点记录 2026-08-02（real_piper_graspnet）

> 主题：真机抓取管道缺口盘点、后台进程清理、IK/规划算法概念澄清、Piper 最大伸展计算。

## 一、真机抓取管道：已完成 vs 缺口

**已完成（感知侧，日志佐证）：**
- D435i 桥接（`run_d435i_bridge.sh`）持续发布 `color/depth/meta/.ready`，20:50 仍在运行
- GraspNet Docker 推理已验证（11:13）：`result.png` / `last_validation.json`（54 候选，best score 0.95）/ `perception_grasps.csv`
- piper_ros 已拷入 `src/` 并编译进 `install/`

**还差（按依赖顺序）：**
1. **相机↔机械臂外参标定**（用户计划用**眼在手外** eye-to-hand）——CSV 位姿在相机系，机械臂需要 base 系
2. **真机手臂启动链路验证**：CAN → `piper_ctrl_single_node` → MoveIt demo；`/joint_states` 通信状态未实测（部署日志期望状态 `communication_status_joint_*` 全 false）
3. **感知→执行闭环节点（完全没有）**：仿真侧有 `graspnet_perception_loader → graspnet_6d_grasp_planner → position_grasp_planner → pick_and_place.cpp` 全套；真机侧无人消费 `perception_grasps.csv`
4. 抓取位姿筛选与可达性预检（仿真有视觉门控 + IK cost 预检）
5. `perception_grasps.csv` 帧约定验证（`link6_grasp_z`/`tcp_forward_z`、TCP→link6 偏移 0.14）
6. 工作空间标定（ROI + 深度窗口）
7. 安全机制：碰撞（octomap depth updater 未配置）、桌面高度、关节限位、急停
8. 抓取-抬升序列 + 夹爪状态完成判定

## 二、操作记录

- **关闭后台进程**（10:48 启动，运行约 10 小时）：`run_d435i_bridge.sh`(20571) / ros2 launch(20593) / `realsense2_camera_node`(20594) / `d435i_graspnet_bridge`(20596)，SIGINT 优雅关闭，无残留
- `logs/d435i_bridge.log` 仍有 **24.8MB** 残留，未删（用户未确认）

## 三、算法概念澄清（已查证源码/官方文档）

- **MoveIt 规划**：默认 **OMPL**（采样规划，未自定义 ompl_planning.yaml）；IK 为 **KDL**（`kdl_kinematics_plugin/KDLKinematicsPlugin`，数值 IK）；另有 Pilz 笛卡尔直线配置（`moveLinear` 用）
- **piper_sdk**：纯执行器，**无规划无 IK**——`piper_ctrl_single_node.py` 只订阅 `pos_cmd`/`joint_ctrl_single`/`enable_flag`，关节角 × factor(57324.840764) 转编码器计数发 CAN
- **ssik**（仿真侧 IK）：解析 IK（Siddhartha Srinivasa，6R/7R，机器精度全解分支），种子无关（已实验证实）
- **Pinocchio（皮诺曹）**：LAAS-CNRS 刚体**动力学计算库**，非规划算法；官方 3.6/3.7 内置 **CLIK**（闭环逆运动学）IK；核心价值是高速 FK/雅可比/逆动力学。本项目未安装未使用
- **动力学**：`τ = M(q)q̈ + C(q,q̇)q̇ + g(q)`；逆动力学（运动→力矩，重力补偿用它）、正动力学（力矩→运动，Gazebo 每步在做）
- **重力补偿**：前馈 `g(q)` + PID 反馈环**两者配合**——纯 PID 也能稳（I 项积掉重力），纯前馈不稳（开环）；商业关节模块通常内置

## 四、眼在手外 × 解析空洞（决策相关）

- 眼在手外 → 抓取姿态族固定朝下 → **高频命中固定空洞区**；但空洞位置固定、可绕开（比眼在手上可控）
- 08-01 已证实：空洞 = "**朝下姿态 × 左区(y≈-0.13~-0.15)**" 限位约束下真不可达（ssik `solutions=0` + KDL `NO_IK_SOLUTION(-31)` 双确认）；竖立姿态 x 到 0.70 全可解
- 缓解三件套（均在遗留清单）：**姿态松弛预检**（绕 X/Y 倾斜采样，绕 Z 无效）、**物体摆位避开左区 / ROI 收紧**、**执行前 canPlanFullPose 预检**；换 IK 求解器无效

## 五、IK 出问题的六层分类（排查顺序）

① 模型层（URDF/TCP 偏移/外参残差）→ ② 几何层（工作空间外/空洞）→ ③ 数值层（局部极小/**DLS 180° 镜像伪解**——必须 FK 权威回验）→ ④ 限位层（解被 joint_limits 裁掉）→ ⑤ 奇异层（近奇异 IK cost 飙升，x=0.58 时 5-10）→ ⑥ 资源层（`kinematics.yaml` `solver_timeout: 0.005` 仅 5ms，需时调大）

## 六、Piper 最大伸展（数值扫描，URDF 链 + 限位，30 万采样）

| 方向 | 最远（指尖→基座原点） |
|---|---|
| 理论极限（竖直向上） | **≈ 0.886 m**（连杆和 0.123+0.285+0.251+0.091+0.136，完全吻合） |
| 水平 ±X/±Y | **≈ 0.76 m**（四方向对称） |
| 向下 -Z | ≈ 0.39 m（向下够得浅） |

- 与实测交叉验证：仿真实测"竖立姿态 x=0.70 可解"，恰在 0.76 内（余量 ~6cm）
- **推荐工作区：离基座 0.30 ~ 0.70 m 水平环带**（<0.3m 进内部空洞区，>0.72m 近奇异）；指尖口径需扣夹爪开口 3~5cm
- 建议真机用 `/compute_fk` 抽查远端位姿确认 URDF 尺寸与真机一致

## 七、下一步建议（优先级）

1. 外参标定（眼在手外）方案落地
2. 真机手臂启动链路验证（/joint_states 闭环）
3. 基于 `perception_grasps.csv` 的闭环节点（消费 + MoveIt 下发 + IK 预检 + 工作区 0.3~0.7m 约束）
