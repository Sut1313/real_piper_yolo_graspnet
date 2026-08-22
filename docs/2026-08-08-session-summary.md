# 2026-08-08 真机联调会话完整记录

> **本文档是下次会话的完整恢复手册**：环境状态、日志索引、时间线、全部数据、代码修改、启动指南、待办。
> 概要版见文末"一页摘要"；详细数据见各章节。

---

## 0. 会话结束时环境状态（下次会话从这里开始）

| 项 | 状态 |
|----|------|
| 后台进程 | **全部已停止**（executor/驱动/MoveIt/相机桥接/graspnet 容器均停） |
| can0 | UP（ERROR-ACTIVE 正常），机械臂**上电中** |
| 相机 | D435i USB3（speed=5000）正常连接 |
| handeye | `camera_data/handeye_eye_to_hand.yaml` = **t_y 修正版**（−0.06516→−0.02916，+0.036）；备份 `.bak_20260808_yfix`（原标定） |
| 代码 | 全部修改在源码（egg-link 安装，无需 rebuild，重启节点即生效） |
| 测试 | 36 passed + 1 skipped（`PYTHONPATH=src/real_piper_perception python3 -m pytest src/real_piper_perception/test/ --ignore=src/real_piper_perception/test/test_sdk_safe_execute.py -q`） |

## 1. 日志文件索引（log/run_2026-08-08/）

| 文件 | 内容 |
|------|------|
| driver.log / driver2.log / driver3.log / driver4.log | 驱动 1~4 次启动（driver3 是碰撞后重启，driver4 是最后关闭前的） |
| moveit.log | move_group/controller_manager（含 FakeSystem 证据） |
| bridge.log | 相机桥接 |
| graspnet.log / graspnet2.log / graspnet3.log | graspnet 容器（gui / headless / gui 重启） |
| executor.log ~ executor12.log | executor 各次启动（含所有执行记录） |
| can_trace.log | candump CAN 记录 |

**系统关键事实（事故调查结论，勿再重复排查）**：
- MoveIt 执行链 = `/execute_trajectory → arm_controller → FakeSystem（模拟硬件）`——"执行成功"与真机无关（moveit.log: `Loading hardware 'FakeSystem'`）
- 真机链 = FakeSystem 状态 → joint_state_broadcaster → `/joint_states` → **驱动 remap 订阅当指令** → CAN → 真机（脆弱隐式链）
- SDK 直控链 = `piper_sdk` 直接 CAN（夹爪 GripperCtrl 直控已用；arm 的 EndPoseCtrl 已验证可靠但欧拉角奇异区危险）

## 2. 完整时间线（2026-08-08）

| 时刻 | 事件 |
|------|------|
| 18:22-18:28 | 启动：can0 激活 → 驱动（piper 包）→ MoveIt → D435i 桥接 → graspnet GUI → executor |
| 18:29 | 发现 matplotlib 无画面：残留 pause_inference 标记 → 删标记恢复；固化启动清理 |
| 18:32 / 18:34 / 18:37 | Task 10 执行 1/2/3 次成功（dry_stop，开口朝下 0.59/0.66/0.83） |
| 18:33:17 | **驱动崩溃**（can0 is loss → rclpy shutdown 二次异常，C1'）→ 重启驱动2 |
| 18:38 | Task 10 第 4 次触发：145 候选 × 8s 预检超时 → 中止 |
| 18:46 | **executor 崩溃**（spin RCLError: context invalid）→ 重启 |
| 18:48-18:54 | Task 10 第 5/6 次：第 5 次 lift_pass=0 失败；第 6 次成功（candidate[18] 开口 0.73） |
| 19:30-19:46 | **SDK 路径探索 + 碰撞事故**（详见 §4）→ 机械臂断电 → 重新上电 → 驱动3 |
| 19:47 | Task 10 第 6 次（重启后）：成功 dry_stop |
| 19:48 | matplotlib 冻结（Tk 非主线程）→ headless 重启容器 |
| 19:5x-20:05 | 全闭判定方案实现（用户裁决）+ 测试 |
| 20:06-21:11 | **Task 11 完整抓取 8 次**（详见 §5）全部 grasp_empty |
| 21:15-21:32 | 感知噪声实验一（30 帧，8-14cm 波动）→ 多帧中值滤波实现 → **验证失败（门控过严）→ 用户裁决回退** |
| 21:35 | 关闭全部后台进程，写总结 |

## 3. Task 10 执行记录（6 次）

| 次数 | 触发 | 候选 | 开口朝下 | 结果 |
|------|------|------|----------|------|
| 1 | 18:32 | [34] | 0.59 | dry_stop ✅ |
| 2 | 18:34 | [34] | 0.66 | dry_stop ✅ |
| 3 | 18:37 | [0] | 0.83 | dry_stop ✅ |
| 4 | 18:38 | — | — | 预检超时中止（145 候选） |
| 5 | 18:48 | — | — | no_candidate（approach_pass=7, lift_pass=0） |
| 6 | 19:47 | [18] | 0.73 | dry_stop ✅ |

启动配置（标准）：`ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2 gripper_forward:=false` + `ros2 launch piper_with_gripper_moveit demo.launch.py` + `run_d435i_bridge.sh` + `TOP_K=50 run_graspnet_d435i.sh [gui|headless]` + `ros2 launch real_piper_perception grasp_execute.launch.py scan_only:=false dry_run_max_stage:=approach graspnet_6d_analytic_only_precheck:=false`

## 4. SDK 路径探索 + 碰撞事故（重要）

### 4.1 SDK 事实（已验证）
- `C_PiperInterface_V2("can0", dh_is_offset=1)`——**dh_is_offset=1 与 URDF 一致**（`scripts/sdk_fk_vs_urdf_fk.py`：位置 <0.1mm、姿态 <0.01°）
- SDK 零位（关节全 0）= 几何 (56, 0, 213)mm + (0, 85°, 0)（=官方 demo 初始位姿）
- `GetFK('feedback')` 返回 6 个 [x,y,z,r,p,y]（mm/度，XYZ 顺序欧拉角）；`EnableFkCal()` 启用
- `EndPoseCtrl(X,Y,Z,RX,RY,RZ)`（0.001mm/0.001° 单位）：**固件内置 IK，可靠**（Z=600 精确到位 0.26mm，13s@5%）
- `MotionCtrl_2(0x01, move_mode, spd_rate, 0x00)`：spd_rate=**速度百分比 0~100**；move_mode 0x00=MOVE P, 0x01=MOVE J, 0x02=MOVE L
- `MotionCtrl_1(0x01,0,0)` 快速急停；`MotionCtrl_1(0x00,0x06,0)` 终止执行
- `GetArmStatus().arm_status.motion_status`：**0x00 语义不可靠**（运动开始时即报 0x00）——等待到位必须用 **FK 位置稳定判据**（连续 N 次采样变化 <1mm）
- SDK FK 欧拉角 = EndPoseCtrl 目标格式（XYZ 顺序）——控制闭环以 SDK FK 语义为准

### 4.2 碰撞事故（19:30，强制断电）
- **候选 [6]**：score=0.649，目标 J6=(524,98,151)mm rpy=(92,85,16)°（RY=85° **欧拉角奇异区**）
- 固件"接受"后执行到 (53,-10,190)mm——**折回底座碰撞**
- 根因链：欧拉角奇异（SDK/固件解释不一致）→ 固件多解分支 → SDK 链路无碰撞保护（回验在运动完成后，太晚）
- **MOVE P 只对"当前位形附近小幅目标"可靠**；|RY|>75° 目标不可用
- 处理：`sdk_go_zero.py`（MoveJ 关节空间回零，无歧义）

### 4.3 M 轴映射数学证明（sdk_mapping_validation.py）
- 恒等式：夹爪轴线（link7 Z）= −Y_link6（0.0002° 验证）
- 无 M：夹爪轴线与接近方向 cos=0.0（错误）；带 M：cos=1.0（正确）——**M 不可省**

### 4.4 SDK 安全闭环设计（暂停待续）
- 设计：`docs/superpowers/specs/2026-08-08-sdk-safe-execute-design.md`（三层：候选奇异过滤 \|RY\|>75° + 连杆级碰撞预检 pinocchio（包围盒 150×150×120mm、连杆端点 5mm 缓冲）+ 执行中实时监控急停（圆柱 xy<0.25m/z<0.40m、突变 30mm/帧））
- 实现计划：`docs/superpowers/plans/2026-08-08-sdk-safe-execute.md`（6 任务 TDD）
- 用户约束（重要）：**MoveIt 工作中不涉及 SDK 修改和使用，除夹爪开合外**

## 5. Task 11 完整抓取验证（8 次全 grasp_empty）

**执行序列**（完整配置）：回 zero → 张开 70mm → **标定段**（实测 full_open/full_closed）→ 预位 → approach（笛卡尔直线）→ grasp（OMPL）→ **闭合（命令恒 0 = 完全闭合）** → 判定 → 抬升（lift_z=0.10）→ 回 zero → 感知恢复

**判定（全闭方案）**：`classify_grasp(actual, full_open, full_closed)`：≈full_open→not_closed；>full_closed+0.002→grasped；≈full_closed→closed_empty（grasp_empty）

### 5.1 八次执行记录

| # | 时间 | 候选 | score | 宽度(mm) | 感知base z(mm) | 结果 |
|---|------|------|-------|----------|---------------|------|
| 1 | 20:06 | [8] | 0.79 | 58.5 | — | grasp_empty（判定盲区已修） |
| 2 | 20:08 | [24] | — | 80.7 | — | failed gripper（超限 N1，后删除该检查） |
| 3 | 20:10 | — | — | — | — | failed home（回 zero 零轨迹，已修） |
| 4 | 20:16 | [32] | 0.1556 | 58.5 | — | failed gripper（超限） |
| 5 | 20:27 | [18] | 0.2974 | 94.5 | — | grasp_empty |
| 6 | 20:51 | [0] | 0.3811 | 97.1 | 286.9 | grasp_empty（FK 比对：x−18,y+10,z−37） |
| 7 | 21:08 | [2] | 0.3266 | 65.3 | 341.4 | grasp_empty（FK：x+25,y+47,z−60） |
| 8 | 21:11 | [32] | 0.1826 | 72.0 | 322.3 | grasp_empty（FK：x−13,y+8,z−25） |

### 5.2 FK vs 感知比对（每次执行后自动落盘 `shared_camera_data/fk_link7_position.json`）

| 候选 | FK link7 抬升位(mm) | grasp 位(减抬升100) | 感知(base) | 偏差(mm) |
|------|---------------------|---------------------|------------|----------|
| [0] | (469.2, 29.3, 349.9) | (469.2, 29.3, 249.9) | (487.1, 19.3, 286.9) | x−17.9, y+10.0, **z−37.0** |
| [10] | (480.5, 1.1, 373.1) | (480.5, 1.1, 273.1) | (483.4, 0.6, 299.3) | x−2.9, y+0.5, **z−26.2** |
| [2] | (489.4, 10.5, 381.5) | (489.4, 10.5, 281.5) | (464.2, −36.5, 341.4) | x+25.2, y+47.0, **z−59.9** |
| [32] | (417.7, 35.7, 397.8) | (417.7, 35.7, 297.8) | (430.8, 27.9, 322.3) | x−13.1, y+7.8, **z−24.5** |

**结论**：x/y 可修正对准（t_y +0.036 验证有效：35.8→5.4mm）；**z 系统性偏深 25-60mm**（grasp_z_offset=0.02 不够 + 执行链偏移）；**但根因是感知噪声**（下节）。

### 5.3 感知噪声实验一（决定性，perception_stability_test.py）
静止物体 30 帧候选中心（base 系）：
```
均值: (519.5, -28.4, 316.2) mm    std: (30.7, 17.7, 26.5) mm
范围: x 136.3mm  y 83.8mm  z 97.7mm
```
**结论：候选位置每帧波动 8-14cm——超手眼/执行误差一个量级，Task 11 抓不到的根因**。候选集还混杂高分误检（score>1 异常、多目标）。

### 5.4 多帧中值滤波尝试（已回退，勿重做）
- 实现：采样 10 帧最高分候选 → 中值 → 候选位置门控（tol=50mm）
- 验证失败：中值基准 (570.7,−8.3,281.2) 被高分误检污染（与均值差 5cm）→ 5 帧全拒（gate_rejected=250）→ **用户裁决回退**
- 教训：最高分候选 ≠ 物体；中值滤波需位置聚类（取最密集簇），未实现

## 6. 代码修改明细（源码在 src/，egg-link 生效无需 rebuild）

| 文件 | 修改 | 状态 |
|------|------|------|
| `real_piper_perception/grasp_verify.py` | `classify_grasp`（全闭方案）+ CLOSE_TOL=0.002/OPEN_TOL=0.003 | 保留 |
| `real_piper_perception/grasp_execute.py` | 闭合命令 0.0；判定段 classify_grasp；标定段（0.5 段：张开→读 full_open→空载闭合→读 full_closed→再张开）；N1 提前检查已加后删（用户裁决删宽度超限检测） | 保留 |
| `real_piper_perception/grasp_executor.py` | `grasp_z_offset` 参数；`_fk_link7_position`（FK 位置落盘 fk_link7_position.json）；回 zero 已零位跳过；多帧中值滤波（已回退） | 保留（滤波除外） |
| `real_piper_perception/grasp_precheck.py` | `make_stage_poses` 加 `grasp_z_offset`（grasp z = 感知 z − offset） | 保留 |
| `launch/grasp_execute.launch.py` | `grasp_z_offset` 默认 0.02 透传 | 保留 |
| `run_graspnet_d435i.sh` | 启动自动 rm pause_inference | 保留 |
| `camera_data/handeye_eye_to_hand.yaml` | t_y +0.036（T_cam2base 同步） | 保留（备份 bak_20260808_yfix） |
| `test/test_grasp_verify.py` | classify 8 用例 | 保留 |
| `test/test_grasp_execute_units.py` | SDK 直控语义 3 用例更新 + 超限测试删除 | 保留 |

**新脚本**：`sdk_fk_vs_urdf_fk.py`、`sdk_endpose_test.py`、`sdk_go_zero.py`、`sdk_mapping_validation.py`、`sdk_candidate_scan.py`、`sdk_full_pipeline_test.py`、`fk_vs_camera_compare.py`、`perception_stability_test.py`

**关键参数现状**：grasp_z_offset=0.02（launch 默认）；grasp_score_min=0.0（阈值关）；velocity_scale=0.1；TOP_K=50；gripper_val_mutiple=2；gripper_forward=false

## 7. 下次恢复启动指南

```bash
# 1. CAN（can0 已 UP 则跳过）
bash src/piper_ros/can_activate.sh can0 1000000
# 2. 驱动（包名 piper！）
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2 gripper_forward:=false
# 3. MoveIt
ros2 launch piper_with_gripper_moveit demo.launch.py
# 4. 相机 + 感知
bash run_d435i_bridge.sh
TOP_K=50 bash run_graspnet_d435i.sh headless   # 或 gui（matplotlib 长跑会卡，headless+result_viewer 更稳）
# 5. executor（Task 11 完整抓取）
ros2 launch real_piper_perception grasp_execute.launch.py scan_only:=false graspnet_6d_analytic_only_precheck:=false
# 6. 触发（异步，勿阻塞等待）
ros2 service call /grasp_execute std_srvs/srv/Trigger "{}"   # 轮询 grasp_result.json 时间戳
```

**执行注意事项**（血泪教训）：
- 服务响应 = 流程结束才返回（候选遍历+执行可能 1-8 分钟）——用异步触发+轮询结果文件
- `pause_inference` 标记存在 = 执行中（画面冻结正常）；消失 = 完成
- 等待夹爪/机械臂到位必须用 FK 稳定判据（motion_status 0x00 不可靠）
- executor/驱动重启后**检查进程存活**（rclpy shutdown 竞态会崩溃）

## 8. 待办（按优先级）

1. **感知质量**（Task 11 卡点根因）：候选位置时域波动 8-14cm、高分误检/多目标、score>1 异常——感知侧排查（ROI/分割/多帧位置聚类）
2. **z 下探偏移**：grasp_z_offset 需感知稳定后重标定（当前被噪声淹没）
3. **预检 IK→FK 回验接线**（昨天遗留："暂以 ssik residual 代理"）
4. **进程崩溃加固**：驱动/executor rclpy shutdown 竞态（C1' 模式）
5. **SDK 安全闭环实现**（设计/计划完成，暂停待续；MoveIt 工作不涉及 SDK 除夹爪）
6. **预检效率**：候选遍历耗时失控（145×8s）——brainstorming 任务挂起（任务 #5-9）
7. **Task 11 验收**：重复 3 次 ≥1 success 未达成（8 次全 grasp_empty）

## 9. 一页摘要

今日：Task 10 回归通过（6 次 4 成功）→ 开放 Task 11（8 次全 grasp_empty）→ 定位根因 = **感知候选位置时域噪声 8-14cm**（实验一 30 帧实测，非手眼/执行误差为主）。期间：碰撞底座事故（SDK 欧拉角奇异，三层防护设计完成待实现）；判定改全闭方案（用户裁决，链路验证成功）；x/y 对准经 FK 比对修正（t_y 验证有效）；多帧中值滤波尝试后回退（误检污染基准）。进程全部停止，can0 UP，handeye 为 t_y 修正版，代码修改在源码（egg-link 生效）。下次启动见 §7。

---

## 10. 问题深度分析（会话末整理，2026-08-08）

### 10.1 今日问题完整清单（修正表述版）

| # | 问题 | 修正后表述 | 状态 |
|---|------|-----------|------|
| 1 | IK 求解易失败 | **目标姿态族几何不可达**（joint2 前倾限位 [0°,180°] + GraspNet 候选大滚转/朝下姿态）——`solutions=0` 是正确拒绝非求解器缺陷；少量 DLS 单 seed 局部解陷阱 | 未解决（需感知侧配合） |
| 2 | 候选预检耗时长 | **无效候选占比高**（门控 110-250 个 + IK 0 解 80%+）：无粗筛前置、5 帧线性放大、串行 | 未解决（brainstorming 挂起） |
| 3 | 抓取精度偏差 | **不是执行/机械精度**（FK 比对显示毫米级）——**感知候选位置噪声 8-14cm** | 未解决（感知侧） |
| 4 | 感知多维质量 | 高分误检/多目标混杂、score>1 异常、宽度预测偏大 2 倍（58-100 vs 实际 40-50mm）、姿态分布差 | 未解决 |
| 5 | z 下探偏移 25-60mm | grasp_z_offset=0.02 方向对量不足 + 执行链额外偏移；被感知噪声淹没无法精标定 | 部分解决 |
| 6 | 进程稳定性 | 驱动崩溃 ×1 + executor 崩溃 ×1（rclpy shutdown 竞态 C1'） | 未加固 |
| 7 | 执行链无闭环 | 预检 IK→FK 回验未接线；MoveIt"执行成功"≠真机（FakeSystem 隐式链）；/joint_states 曾断流 | 未解决 |
| 8 | SDK 碰撞事故 | 欧拉角奇异（\|RY\|>75°）+ 固件多解 + 无保护链；三层防护设计完成 | 设计完成未实现 |
| 9 | matplotlib 卡死 | Tk 非主线程事件循环卡死（长跑 ~3500 帧后）；headless 规避 | 规避未根治 |
| 10 | 互锁残留 | pause_inference 强杀残留（已修+固化启动清理） | ✅ 已解决 |

### 10.2 IK 失败原因四层分析

1. **目标姿态族几何不可达（主因）**：joint2 单向限位只能前倾——大滚转（RX 120-150°）/朝下姿态无逆解；证据：同帧候选有通过有失败（非全灭），昨日 ssik 朝下 0/8
2. **感知候选姿态质量差（放大因素）**：位置波动 8-14cm + 姿态每帧跳变 + score>1 异常混杂
3. **解析 IK（ssik）求解域限制**：解析解对特定姿态族无解
4. **数值 IK（DLS）局部解陷阱**：从零位 seed 出发只收敛零位附近分支；`solve_ik_link6` 无多起点策略

**与机械臂关节位置的关系（两层区分）**：
- IK **解的存在性**（几何可达）——与当前关节位置**无关**（纯位姿空间属性）
- **规划成功**（MoveIt 从当前位形出发）——与关节位置**有关**；今日所有预检均从 zero 位形出发（每次执行前回 zero）——"Unable to sample valid states"可能含位形依赖成分
- 区分实验（未做）：换起始位形预检 / 多起点数值 IK / 失败候选姿态分布统计

### 10.3 预检耗时构成（每候选 3-8s）

```
self-consistency 门控 ~1ms → 变换链 ~1ms → ssik IK ~0.5s（失败也要）
→ MoveIt plan_only ~1-2s → MoveIt pose 兜底 ~1s → FK 回验 ~0.5s
总：50-196 候选 × 3-8s × 5 帧检测 = 50s ~ 20 分钟
```

优化方向（brainstorming 挂起）：A. 秒级粗筛前置（多起点 IK 几何可达判定 + 宽度 ≤0.065m + score 粗筛）B. 单段预检 C. 5 帧去重 D. IK 并行

### 10.4 根因链（问题优先级）

```
感知质量（#4）──→ 候选不可达（#1）──→ 预检耗时长（#2）
      │
      └──→ 抓取落空（#3）
稳定性（#6/#7）与安全（#8）是独立于抓取成功率的基础问题
```

**一句话**：感知候选质量差 → 大部分候选不可达 → 预检耗时长 → 能执行的候选仍受位置噪声影响落空；进程崩溃、闭环缺失、SDK 安全是三条独立待办线。

### 10.5 明日/后续优先行动建议

1. 感知侧排查（候选位置波动/误检/宽度预测/score>1）——Task 11 卡点根因
2. 预检粗筛（多起点 IK 几何可达判定）——同时解决效率与准确度
3. 感知稳定后重标定 grasp_z_offset（z 偏移 25-60mm）
4. 预检 IK→FK 回验接线
5. 进程崩溃加固（rclpy shutdown 竞态）
6. SDK 安全闭环实现（三层防护）
