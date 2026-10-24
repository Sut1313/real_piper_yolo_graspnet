# 2026-10-22 真机抓取测试问题报告

- 测试日期：2026-10-22
- 测试对象：`real_piper_yolo_graspnet`（D435i + YOLOv8s-seg v5 + GraspNet + PiPER）
- 本次背景：机械臂重新安装，朝向与上一次安装相反；相机内参不变，重新完成手眼标定
- 报告结论：**预检通过，执行在夹爪反馈/夹持判定阶段失败；本次没有成功抓取**

---

## 1. 结论摘要

本次测试中：

1. YOLO 和 GraspNet 能持续输出目标与抓取候选；
2. 新安装方向下的手眼结果能够通过 MoveIt 预检；
3. 机械臂实际执行到了抓取位，夹爪执行了张开/闭合命令；
4. 夹爪 SDK 反馈始终为 `0.0`，`full_open` 和 `full_closed` 也均为 `0.0`；
5. 最终夹持判定得到 `not_closed → grasp_empty`，执行失败；
6. 机械臂收尾回到 zero，驱动状态正常，`err_code=0`，没有碰撞或驱动错误。

直接失败点不是预检，也不是运动规划，而是**夹爪反馈无效 + 夹持判定门控**。

---

## 2. 本次使用的配置

### 2.1 相机内参

项目内参与标定目录一致，未在本次重标中修改：

```text
fx = 604.137589
fy = 601.116717
cx = 340.826966
cy = 238.674895
dist = [0.083091, 0.053479, -0.007423, 0.010382, -0.555466]
reprojection_error = 0.42145 px
```

哈希：

```text
84ad2a80b0afe22cb01ed5591f0b06eb7ad5959df177273c423a9e2e18254415
```

### 2.2 手眼标定

本次使用 2026-10-22 16:30 新安装后的结果：

```text
t_cam2base = [0.334103738, -0.285978249, 0.339165487] m
           = [334.1, -286.0, 339.2] mm
```

相机光轴在基座系下约为：

```text
[-0.066361, 0.765422, -0.640098]
```

项目文件：

```text
camera_data/handeye_eye_to_hand.yaml
```

项目文件与标定目录源结果的 `R/t/T` 逐位一致；项目文件额外包含：

```yaml
parent_frame: base_link
child_frame: camera_color_optical_frame
```

项目文件哈希与源文件不同，只是因为增加了上述帧字段和 description，不是数值差异。

### 2.3 感知与规划

```text
相机：RealSense D435I，序列号 327122073765
YOLO：tabletop_yolov8s_seg_v5.onnx
YOLO SHA-256：3b3d52c854000511348795276f9d52590d3d8c9728a2b7805f64cdbf9ee281ca
目标类别：block
MoveIt：piper_with_gripper_moveit demo.launch.py，FakeSystem
执行速度缩放：0.02
grasp_z_offset：0.0
graspnet_6d_max_candidates：30
```

### 2.4 驱动与机械臂状态

驱动启动参数：

```text
can_port=can0
auto_enable=True
gripper_exist=True
gripper_forward=False
```

驱动日志显示：

```text
Enable status: False → True
gripper_val_mutiple is 1
```

SDK 读取到：

```text
enable_status = [True, True, True, True, True, True]
ctrl_mode = CAN_CTRL
arm_status = NORMAL
mode_feedback = MOVE_J
teach_status = DISABLED
motion_status = REACH_TARGET_POS_SUCCESSFULLY
err_code = 0
```

注意：代码中 `GRIPPER_VAL_MULTIPLE = 2`，但本次驱动实际 `gripper_val_mutiple=1`。当前夹爪命令走 SDK 直控，驱动 `gripper_forward=False`，因此这不是本次零反馈的直接原因，但属于配置不一致，需要修正或明确约束。

---

## 3. 关键数据文件

### 3.1 执行结果

| 文件 | 时间 | 内容 |
|---|---|---|
| `shared_camera_data/grasp_result.json` | 16:45 | 最终抓取结果 |
| `shared_camera_data/precheck_results.json` | 16:45 | 执行流程内的预检 approved/selected |
| `shared_camera_data/fk_link7_position.json` | 16:45 | 收尾前 FK link7 位置 |

`grasp_result.json`：

```json
{
  "timestamp": "2026-10-22T16:45:35",
  "success": false,
  "status": "grasp_empty",
  "stage": "",
  "actual_width": 0.0,
  "cmd_width": 0.089388591,
  "candidate_index": 10,
  "message": "candidate [10] block_0: pick status=grasp_empty stage='' actual_width=0.0 cmd_width=0.0894"
}
```

`precheck_results.json` 的执行结果：

```text
approved = 6
selected_index = 10
selected_camera_xyz = [0.095169164, 0.022668511, 0.435000002]
selected_camera_R =
  [[ 0.531377196, -0.790735006, -0.303934962],
   [-0.169523820,  0.252266020, -0.952692747],
   [ 0.830000043,  0.557763398, -0.000000024]]
```

selected 候选在基座系下约：

```text
base ≈ [402.4, 45.8, 49.2] mm
```

### 3.2 感知数据

| 文件 | 最新时间 | 说明 |
|---|---|---|
| `shared_camera_data/detections.json` | 16:59 | 当前帧检测，不是执行瞬间快照 |
| `shared_camera_data/perception_targets.csv` | 16:59 | 当前帧物体中心 |
| `shared_camera_data/perception_grasps.csv` | 16:59 | 当前帧 262 个候选 |
| `shared_camera_data/frame_manifest.json` | 16:59 | 当前帧模型/时间戳 |
| `shared_camera_data/last_validation.json` | 16:59 | 当前帧完整诊断 |

重要：执行发生在 16:45，而上述 `detections/perception_grasps/targets` 已在 16:59 被后续帧覆盖。**不能用 16:59 的物体中心直接评价 16:45 的 selected 候选**；执行瞬间的可靠数据是 `precheck_results.json`（16:45）和 `grasp_execute` 日志。

执行后当前帧状态（仅供确认感知仍正常）：

```text
frame_sequence = 975
block_0 confidence = 0.983
mask_pixels = 7672
model = tabletop_yolov8s_seg_v5.onnx
```

当前 `last_validation.json`：

```text
frame = 815
block_0 points = 7549
per_object_grasps = 262
object_center_camera = [0.125266, 0.044905, 0.426181] m
```

### 3.3 日志

| 文件 | 用途 |
|---|---|
| `/tmp/grasp_execute_real.log` | 完整执行流程、预检、夹爪判定 |
| `/tmp/grasp_execute_real_call.log` | `/grasp_execute` 服务响应 |
| `/tmp/grasp_precheck_new_install.log` | 新安装后的预检 |
| `/tmp/graspnet_yolo.log` | GraspNet 逐帧推理与候选统计 |
| `/tmp/real_piper_yolo_bridge.log` | YOLO 桥接、模型、帧提交 |
| `/tmp/piper_driver.log` | PiPER 驱动、使能、关节命令 |
| `/tmp/piper_moveit_demo.log` | MoveIt/RViz 规划与状态 |

---

## 4. 测试时间线

```text
16:30  新安装方向的手眼标定结果生成
16:37  D435i + YOLO 桥接 + 手眼 TF 启动
16:41  GraspNet 消费者启动，.ready 正常消费，YOLO 开始持续更新
16:42  预检-only：125 候选，18 个 approved，selected=2
16:44  执行节点启动：scan_only=false，velocity_scale=0.02
16:45  执行流程重新预检：217 候选，6 个 approved，selected=10
16:45:35  执行结果 grasp_empty；夹爪反馈 full_open/full_closed=0/0
16:45:58  收尾回 zero 成功
```

说明：16:42 的预检-only 和 16:45 执行流程使用的是不同感知帧，所以 selected 从 2 变成 10；这是候选随帧变化的结果，不是预检失败。

---

## 5. 本次实际结果

### 5.1 预检

执行流程内：

```text
候选数：217
测试数：30
approach_pass：6
grasp_pass：6
approved：6
selected：10
```

`candidate[10]` 预检全过：

```text
link6 前向 z = -0.37
开口朝下 = 0.81
```

说明新安装方向下的手眼、候选姿态和 MoveIt 规划是能跑通的；本次失败不在预检。

### 5.2 执行

执行日志关键序列：

```text
回 zero：当前已在目标位，跳过执行
候选帧 1/1：217 候选
candidate[10] 预检全过
execute_pick: grasp_width=0.0844 m, cmd_width=0.0894 m
夹爪张开：SDK GripperCtrl(70000, 1000, 0x01, 0)   # 总开度 70.0 mm
夹爪标定闭合：SDK GripperCtrl(0, 1000, 0x01, 0)    # 总开度 0.0 mm
夹爪张开：SDK GripperCtrl(70000, 1000, 0x01, 0)    # 总开度 70.0 mm
判定参考标定：full_open=0.0mm full_closed=0.0mm
夹爪闭合：SDK GripperCtrl(0, 1000, 0x01, 0)        # 总开度 0.0 mm
grasp verdict: actual_width=0.0000 m,
  full_open=0.0000, full_closed=0.0000,
  detail=not_closed → grasp_empty
回 zero 执行：成功
```

`FK link7`（收尾前）：

```text
(413.9, 43.9, 41.5) mm
```

---

## 6. 问题分析

### P0：夹爪 SDK 反馈无效，始终为 0

执行器代码路径：

```text
_gripper_set_width()
  └─ 首次命令时创建 C_PiperInterface('can0')
     ├─ GripperCtrl(0, 1000, 0x02, 0)
     └─ GripperCtrl(0, 1000, 0x01, 0)

_read_gripper_feedback_stable()
  └─ GetArmGripperMsgs().gripper_state.grippers_angle / 1e6
```

执行器创建的是旧 `C_PiperInterface`，而不是驱动/标定使用的 `C_PiperInterface_V2`，并且没有显式：

```text
ConnectPort()
使能/反馈初始化
确认反馈时间戳/Hz
```

本次 `grippers_angle` 在“全开”和“全闭”两个状态都返回 `0.0`，因此得到：

```text
full_open = 0.0
full_closed = 0.0
```

这不代表夹爪物理上全开/全闭都是 0，而是反馈没有被读到。

同时，`/joint_states_feedback` 也没有为执行器提供有效数据；驱动日志每秒输出大量 `joint1..joint7` 信息（本次日志约 699k 行），`joint_states_feedback` 的发布存在被高频日志/回调饿死的现象。执行器在 `_piper` 创建后优先使用 SDK，所以不会自动回退到话题反馈。

### P0：标定门控没有识别 0/0

`_read_gripper_feedback_stable()` 的稳定窗口算法是：

```text
连续 >=5 个样本
时间跨度 >=0.35 s
最大最小值差 <=0.8 mm
返回窗口中位数
```

它只判断“数值稳定”，不判断“数值是否有效”。连续的 `0.0` 完全满足稳定条件，因此 `full_open=0.0`、`full_closed=0.0` 被当成有效标定，没有在 `gripper_calib` 阶段失败。

随后 `classify_grasp()`：

```python
if actual_width > full_open - OPEN_TOL:
    return 'grasp_empty', 'not_closed'
```

代入 `actual_width=0`、`full_open=0`：

```text
0 > -0.003  → True
```

于是最终判定 `not_closed`，返回 `grasp_empty`。所以本次是 fail-closed，但触发原因是反馈无效，不是物体真的没夹住。

### P1：候选宽度预测超夹爪行程

执行日志：

```text
candidate[10] width = 0.0844 m
cmd_width = 0.0894 m
```

本次木块约 50 mm，但候选预测宽度 84.4 mm，已经超过 SDK 小夹爪 70 mm 总行程；`cmd_width` 89.4 mm 也超限。当前执行流程使用“完全闭合”策略，闭合命令是 0.0，不直接使用这个宽度，所以没有在 `gripper_width` 处失败；但候选宽度本身说明 `candidate[10]` 的质量不可靠，可能与实际木块位置/姿态偏差较大。

本次选中的候选在基座系约为：

```text
[402.4, 45.8, 49.2] mm
```

执行后当前帧物体中心（16:59，不是执行快照）约为：

```text
[435.7, 29.5, 39.4] mm
```

两者相差约 3–4 cm，和 84.4 mm 的预测宽度一起，说明候选质量存在问题。即使夹爪反馈修好，candidate[10] 也未必能稳定夹住 50 mm 木块。

### P1：驱动配置不一致

驱动日志：

```text
gripper_val_mutiple is 1
```

代码常量：

```text
GRIPPER_VAL_MULTIPLE = 2
```

当前夹爪命令走 SDK 直控，驱动 `gripper_forward=False`，所以该不一致不是本次零反馈的直接原因；但如果以后切回驱动转发夹爪，`joint7` 命令与 `GripperCtrl` 行程会差 2 倍，必须统一。

### P1：驱动反馈发布被饿死

驱动日志约 699k 行，包含高频 `joint1..joint7` INFO 输出；`/joint_states_feedback` 在本轮没有提供稳定反馈。这与历史问题一致：

```text
MoveIt /joint_states 约 200 Hz
驱动 joint_callback 高频日志
joint_states_feedback 发布线程得不到足够调度
```

执行器虽然加了 SDK fallback，但 fallback 自己也返回 0，说明“SDK 直读”这条路同样没有建立有效反馈。

### P1：MoveIt 状态完整性警告

MoveIt 日志在执行收尾阶段出现：

```text
The complete state of the robot is not yet known. Missing joint8
```

`joint8` 是夹爪镜像手指，不在 arm 规划组，通常不应导致抓取失败；但它说明 `/joint_states` 与机器人完整状态的同步不完整，建议作为后续清理项。

### P2：数据快照与可追溯性

- 16:42 预检-only 的 selected 是 2；
- 16:45 执行流程的 selected 是 10；
- `perception_grasps.csv` 在 16:59 被后续帧覆盖，无法回放 16:45 的完整候选集；
- 本次只能依赖 `precheck_results.json`、`grasp_execute` 日志和 `grasp_result.json` 还原执行瞬间。

建议每次执行时把候选 CSV、precheck JSON 和 handeye/K 哈希一起归档到带时间戳的运行目录。

### P2：YOLO/GraspNet 帧间歇性无目标

GraspNet 日志中部分帧出现：

```text
Process error: YOLO frame contains no selected target instance
```

桥接日志也有 `selected=0 (no_matching_detection)` 的阶段。当前最新帧已经能稳定选中 block（置信度约 0.98），但推理帧仍存在间歇性空目标，抓取前应确认执行使用的是“有有效目标”的帧。

---

## 7. 门控链结论

| 门控 | 本次结果 | 说明 |
|---|---|---|
| 候选门控 | PASS | `approved=6`，`selected=10` |
| 预检门控 | PASS | approach/grasp 均通过 |
| 执行到位门控 | PASS（到抓取位） | MoveIt 执行完成，FK link7 有值 |
| 夹爪标定参考门控 | **未正确拦截** | `full_open=0`、`full_closed=0` 被当成有效参考 |
| 最终夹持/闭合门控 | **FAIL** | `actual_width=0`，`not_closed`，`grasp_empty` |

失败点明确是**夹爪反馈与最终夹持判定**，不是预检或运动规划。

---

## 8. 修复建议

### 8.1 先修夹爪反馈（P0）

1. 执行器改用 `C_PiperInterface_V2`，显式 `ConnectPort()`，确认使能后读取 `GetArmGripperMsgs()`；
2. 读取时检查：
   - `grippers_angle` 是否随张开/闭合变化；
   - SDK 反馈时间戳/Hz 是否有效；
   - 是否出现通信异常；
3. 如果 SDK 仍为 0，检查重装后的夹爪反馈线、夹爪供电/使能，而不是先改判定阈值；
4. 在 SDK 无效时，显式回退到 `/joint_states_feedback`，并修复驱动的反馈发布。

### 8.2 增加夹爪标定合理性门控（P0）

在 `gripper_calib` 阶段增加硬检查：

```text
full_open 应接近 70 mm
full_closed 应接近 0–5 mm
full_open - full_closed 应 > 30 mm
```

任一不满足，直接 fail-closed，禁止继续到抓取位；不要让 `0/0` 通过稳定窗口。

### 8.3 恢复候选宽度/质量门控（P1）

- `width + margin` 超过 70 mm 的候选应在预检前拒绝；
- 对 50 mm 木块，预测 84.4 mm 的候选应被标记为低可信；
- 保留中心一致性/自一致性检查，并记录候选与物体中心距离。

### 8.4 统一驱动配置与日志（P1）

- 明确夹爪控制路径：当前 `gripper_forward=False` + SDK 直控；
- 如果使用驱动转发，`gripper_val_mutiple=2` 必须与代码常量一致；
- 降低驱动 `joint1..joint7` INFO 日志频率，避免饿死 `joint_states_feedback`；
- 检查 `joint8` 是否应加入 `/joint_states` 或 MoveIt 状态同步。

### 8.5 执行快照（P1）

每次执行前保存：

```text
perception_grasps.csv
precheck_results.json
grasp_result.json
frame_manifest.json
detections.json
calibration.yaml
handeye_eye_to_hand.yaml
candidate_index / selected xyz / R
```

到独立运行目录，避免被后续帧覆盖。

---

## 9. 最终判断

```text
手眼/预检：本次通过，能用
执行运动：到达抓取位
夹爪反馈：无效，始终 0
夹持判定：not_closed → grasp_empty
抓取结果：失败
驱动/机械臂：err_code=0，收尾回 zero
```

本次不能判定“夹爪没有夹住”，因为反馈链路没有提供有效宽度；在修复 SDK/驱动反馈并增加 `full_open/full_closed` 合理性门控之前，重复执行大概率仍会在同一门控失败。
