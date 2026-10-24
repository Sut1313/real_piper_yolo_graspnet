# 真机测试前修正方案（P0/P1/P2）

> 记录日期：2026-10-21
> 状态：方案已记录，代码尚未修改
> 适用仓库：`/home/liyidong/agrobot_code/real_piper_yolo_graspnet`
> 关联文档：[REAL_ROBOT_TEST_RUNBOOK.md](REAL_ROBOT_TEST_RUNBOOK.md)

## 0. 目标与边界

先让“夹住并保持”在真机上安全跑通，再补完整抓取/放置，最后才做真正的
`ros2_control` 反馈闭环。不要一次改完标定、门控、lift/place 和自动循环，
否则出问题无法区分是标定、代码还是硬件。

当前基线事实：

- GraspNet 真机链在 2026-08-21 做过 3/3 次“夹住并保持”，随后回 zero；
- YOLO 前端 `src/real_piper_yolo_graspnet/` 已编译、单测通过，但没有真机运行记录；
- `camera_data/handeye_eye_to_hand.yaml` 是旧工位结果，必须重做；
- 当前执行段没有 lift/place，成功后夹爪仍夹着物体回 zero。

---

## 1. P0：第一次真机夹持前必须完成

### P0-1 重标定并修正手眼帧

步骤：

1. 用 `piper_single_calibration` 重做内参（640×480）和 eye-to-hand；
2. 复制到 `camera_data/calibration.yaml`、`camera_data/handeye_eye_to_hand.yaml`；
3. 在 handeye yaml 中加入：

```yaml
parent_frame: base_link
child_frame: camera_color_optical_frame
```

4. 修改 `handeye_tf_publisher` 的默认 `child_frame`，或由 yaml 键覆盖；
5. 把 `grasp_transform.load_handeye` 的“文件缺失回退仿真常量”改为 fail-closed：

```python
except Exception as exc:
    raise RuntimeError(f'handeye calibration load failed: {path}') from exc
```

验收：

- 启动日志打印正确的 `R_cam2base/t_cam2base`；
- `tf2_echo base_link camera_color_optical_frame` 与 yaml 一致；
- 独立触点验证误差 < 5 mm。

### P0-2 第一次测试参数

```text
scan_only=true
velocity_scale=0.05
grasp_z_offset=0.0
graspnet_6d_max_candidates=30
```

`grasp_z_offset=0.02` 是旧标定补偿，新标定后先归零。

### P0-3 增加执行到位门控

位置：`grasp_execute.py`，在 `_move_pose(stage_poses['grasp'])` 之后、`_close_gripper` 之前。

要求：

- 读取真实末端位姿：`piper_sdk.GetArmEndPoseMsgs()` 或
  `/joint_states_feedback` + `/compute_fk`；
- 目标折算到同一坐标系：`_tcp_pose_to_link6(stage_poses['grasp'])`；
- 等稳定窗口后再比较，不要用单帧；
- 位置阈值建议 0.010–0.015 m，姿态阈值 0.03–0.05 rad；
- 超限返回 `_fail('pose_gate')`，不闭合夹爪、不重试。

### P0-4 成功后释放

当前 `grasp_executor` 成功后直接回 zero，夹爪仍闭合。下一轮 `execute_pick` 会先张开，
物体会在 zero 位掉落。

处理：

- 在 `grasp_executor` 收尾回 zero 后，按参数 `release_after_pick` 调用一次
  `GraspExecutor.release()`（SDK 张开到 0.07 m）；
- 或者第一次测试人工在 zero 位释放，连续测试前必须补。

### P0-5 增加候选宽度门控

`gripper_limits.fits_small_gripper()` 已存在但未被调用。在 `grasp_executor`
遍历候选时加入：

```python
from real_piper_perception.gripper_limits import fits_small_gripper

if not fits_small_gripper(cand.width, self._gripper_margin):
    self.get_logger().warn(f'{cand.name}: width {cand.width:.3f} 超出夹爪行程，跳过')
    continue
```

验收：宽度 + margin > 70 mm 的候选在预检前被拒绝并记录原因。

### P0-6 YOLO 前端上机检查

- 只跑感知，不开执行器；
- 检查 `yolo_overlay.png`、`instances.npz`、`frame_manifest.json`、
  `perception_grasps.csv`；
- ONNX Runtime 导入失败时设置 `YOLO_ONNXRUNTIME_SITE_PACKAGES`；
- 第一次只用 v2 模型、单目标、`TARGET_CLASS` 与场景一致；
- 标定后 `calibration.yaml` 必须同步，分辨率保持 640×480。

---

## 2. P1：跑通“夹住并保持”后再做

### P1-1 恢复 lift/place

当前 `grasp_execute.py` 在闭合判定后直接返回，注释写明“抬升暂时关闭”。
要做完整抓取放置，需要：

- 闭合判定成功后 `_move_linear(stage_poses['lift'])`；
- 抬升后再读一次夹爪反馈，确认未掉件；
- 增加 `place` 位姿（固定基座点或相对偏移）；
- 执行 `_move_pose(place)`、张开、退开、回 zero；
- 在 `grasp_precheck` 恢复 lift 段预检，并补 place 段。

验收：

- `dry_run_max_stage=lift` 能单独空跑；
- 空夹爪走完整流程不碰撞；
- 低速抓取后抬升 10 cm，夹爪反馈稳定。

### P1-2 补预检 FK 回验

`grasp_precheck._fk_tcp()` 当前是占位，返回 `None`，`_fk_verify` 退化为
ssik residual。需要接 `/compute_fk` 或 MoveIt RobotState，用关节解算 link6
位姿，再与目标比较。

验收：故意给一个差 3 cm 的位姿，预检能判失败。

### P1-3 感知时序

执行器写 `pause_inference` 后 GraspNet 跳过帧，但桥接仍会写新帧；恢复后
第一帧可能拍到机械臂。建议恢复后丢弃一帧，或在桥接里检查 pause 标记。

---

## 3. P2：长期改造

### P2-1 真正的 ros2_control 硬件接口

当前链路：

```text
MoveIt → FakeSystem → /joint_states → Piper 驱动 → CAN
```

长期实现 `SystemInterface`：

- `read()`：从 SDK 读关节/夹爪反馈；
- `write()`：写 `JointCtrl`；
- 替换 `piper.ros2_control.xacro` 的 `mock_components/GenericSystem`；
- 取消 `joint_ctrl_single → /joint_states` 重映射。

这样 `/joint_states` 才是真实反馈，规划状态与物理状态一致。

### P2-2 安全与自动循环

- 急停订阅/心跳；
- `arm_status` 异常停机；
- 工作空间边界；
- 失败恢复；
- 单次触发，不做无人值守自动循环。

---

## 4. 手眼标定后必须同步修正的参数

| 参数/文件 | 当前值 | 重标后处理 |
|---|---|---|
| `camera_data/calibration.yaml` | 旧 K/D | 替换为新内参，保持 640×480 |
| `camera_data/handeye_eye_to_hand.yaml` | 旧 R/t | 替换新值，补 `parent_frame`/`child_frame` |
| `handeye_tf_publisher` 默认帧 | `camera_link` | 改为 `camera_color_optical_frame` |
| `grasp_transform.load_handeye` 回退 | 仿真常量 | 改为抛异常，fail-closed |
| `grasp_z_offset` | 0.02 | 先设 0.0，重新测量 |
| `approach_clearance` | 0.05 | 用新标定复测，必要时调整 |
| `open_gripper_lift_height` / `lift_z` | 0.10 | 恢复 lift 时复测 |
| `TCP_OFFSET_FROM_LINK6_Z` | 0.1358 | URDF 值，核对实际夹爪安装 |
| `GRASPNET_TO_PIPER_TCP_R` | 固定轴映射 | 用新标定做 plan-only A/B 复核 |
| `self_consistency_max_center_dist` | 0.08 | 根据新候选分布调整 |
| `grasp_score_min` | 0.0 | 收集数据后再设阈值 |
| `min_approach_down` | 0.70（仅声明） | 实现硬门控或删除 |
| `gripper_margin` | 0.005 | 物体宽 + margin < 0.07 |
| `graspnet_6d_max_candidates` | 150 | 单目标先用 30 |
| `velocity_scale` | 0.1 | 第一次用 0.05 |
| `place` 位姿 | 无 | 做 lift/place 时实测 |
| `scripts/fk_vs_camera_compare.py` | 读 `/joint_states` | 改读 `/joint_states_feedback` |

---

## 5. 门控问题详解

### 5.1 什么是门控

门控不是“规划成功就继续”，而是在关键动作前后用**独立、可测的信号**决定
继续还是停止。当前系统有四层门：

```text
候选门控 → 预检门控 → 执行到位门控 → 夹持/保持门控
```

任何一层用了错误信号，后面都会误判。

### 5.2 当前四层门控的实际状态

| 层 | 位置 | 当前信号 | 问题 |
|---|---|---|---|
| 候选 | `grasp_executor` | score、self-consistency、方向偏好 | `grasp_score_min=0` 等于关闭；`min_approach_down` 只声明未使用；宽度门未调用 |
| 预检 | `grasp_precheck` | MoveIt plan-only + ssik residual | 只检 approach+grasp；`_fk_tcp()` 是占位；起始状态是 FakeSystem，不是真实反馈 |
| 执行到位 | `grasp_execute` | MoveIt `execute()` 返回值 | 没有实际位姿比较；`execute()` 成功只代表轨迹被接受 |
| 夹持/保持 | `grasp_execute` | `/joint_states_feedback` | 这一层是对的；稳定窗口 0.35 s / 5 样本 / 0.8 mm |

### 5.3 核心问题：把“命令状态”当成“实际状态”

当前 MoveIt 使用 `mock_components/GenericSystem`：

```text
MoveIt → FakeSystem → /joint_states → Piper 驱动 → CAN
```

- `/joint_states` 是 FakeSystem 的命令状态，不是真实反馈；
- 真实反馈在 `/joint_states_feedback`；
- `rsp.launch.py` 又用 `/joint_states` 发布 TF，所以 RViz 显示的也是命令状态；
- `grasp_executor._fk_gripper_orientation` / `_fk_link7_position` 读的也是
  `/joint_states`，诊断日志可能显示“到位”，但物理臂可能还在滞后或被挡住。

结果：规划成功、FakeSystem 到位、`execute()` 返回 True，三者都不等于
机械臂实际到达抓取位。夹爪如果此时闭合，就会在错误位置抓空或碰撞。

### 5.4 具体失效模式

1. **规划成功但物理不到达**：控制器限幅、关节滞后、碰撞、CAN 丢帧；
2. **TCP/link6 坐标系混用**：目标用 TCP，实际读 link6，误差 13.58 cm；
3. **只看位置不看姿态**：位置到了，夹爪朝向偏 10–20°，闭合时夹偏；
4. **单帧反馈**：运动刚结束时反馈还在变化，过早判定；
5. **门限过松**：4 cm 的旧门限对 7 cm 夹爪没有保护意义；
6. **门限过紧**：控制器滞后/背隙导致正常动作被误判失败；
7. **假阳性诊断**：`/joint_states` 的 FK 日志显示到位，掩盖真实偏差；
8. **成功后不释放**：下一轮张开夹爪时物体掉落。

### 5.5 推荐的门控设计

#### 候选门控

- `fits_small_gripper(width, margin)`：宽度 + margin ≤ 0.070；
- `self_consistency`：中心与抓取点横向距离 ≤ 0.08；
- `grasp_score_min`：先 0，收集 20–30 条真实候选后设 0.2–0.3；
- `min_approach_down`：如果要用，计算 TCP +Z 与 base -Z 的对齐度，作为硬门控，
  不是只声明参数；
- 排序：当前 `rank_candidates` 是 score 升序（场景特定），改成参数
  `rank_order=asc|desc`，并在日志里打印。

#### 预检门控

- 起始状态必须是执行时的真实位形；
- approach + grasp + lift（恢复后）+ place 全部 plan-only；
- FK 回验必须真正接上，不能用 residual 代替；
- 预检通过只代表“规划可达”，不代表“物理到位”。

#### 执行到位门控

数据源：

```text
/joint_states_feedback → /compute_fk(link6)
或 piper_sdk.GetArmEndPoseMsgs()
```

不要用 `/joint_states`。

比较：

```python
target_link6 = _tcp_pose_to_link6(stage_poses['grasp'])
actual_link6 = read_actual_link6_pose()
pos_err = ||actual.translation - target.translation||
ang_err = quaternion_angle(actual.rotation, target.rotation)
```

稳定窗口：

- 等 0.2–0.5 s；
- 至少 3–5 个样本；
- 位置抖动 < 2 mm 再判定。

阈值：

```text
approach: pos 0.020 m, ang 0.08 rad
grasp:    pos 0.010–0.015 m, ang 0.03–0.05 rad
```

失败处理：

```python
if pos_err > tol or ang_err > tol:
    return self._fail('pose_gate')
```

不要闭合夹爪，不要盲目重试；退到安全位后重新感知。

#### 夹持/保持门控

现有 `/joint_states_feedback` 稳定窗口是对的，保留：

- 全开/全闭参考每次执行前实测；
- 闭合后等稳定窗口；
- `classify_grasp` 判定 `grasped/closed_empty/not_closed`；
- 抓住后保持复核一次；
- 恢复 lift 后，抬升再读一次，判断是否掉件。

### 5.6 门控的验收测试

1. 正常抓取位：pose gate 通过；
2. 目标故意偏 10 cm：pose gate 失败，夹爪不闭合；
3. 目标姿态故意转 10°：姿态门失败；
4. 用手/挡板阻止机械臂到达（安全前提下）：pose gate 超时失败；
5. 断开 `/joint_states_feedback`：门控 fail-closed，不执行闭合；
6. 正常夹住物体：夹持门 success；空夹：`closed_empty`；
7. 抓住后轻轻拉物体：保持门失败。

只有 1–7 都通过，才认为门控可信。

---

## 6. 实施顺序

```text
1. P0-1 重标定 + 帧名 + load_handeye fail-closed
2. P0-2 参数归零
3. P0-6 只感知 + scan_only
4. P0-3 执行到位门控 + P0-5 宽度门控 + P0-4 成功后释放
5. approach 空跑 → grasp 空跑 → 空中闭合 → 第一次夹持
6. P1 lift/place + 预检 FK + 感知时序
7. P2 真正的 ros2_control 硬件接口 + 安全门控
```

每一步单独提交、单独验证；标定文件和代码改动不要混在同一次变更里。
