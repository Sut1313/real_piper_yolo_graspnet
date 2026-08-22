# 2026-08-21 新 TCP 矩阵真机抓取与日志分析

## 最终结论

本轮已真实下发一次抓取运动，机械臂完成 approach、grasp 和回 zero，CAN/机械臂均未报错。新 TCP 矩阵的姿态语义在执行终点得到验证：FK 的 link6/TCP 前进轴基座 Z 分量为 `-0.96`，与候选的 `approach_down=0.968` 一致，说明末端确实以 GraspNet 接近轴朝下。

但本轮**不能判定为真实抓取成功**。应用在夹爪仍在闭合过程中读取到首条 `24.0 mm` 瞬态反馈并写出 `status=success`；回零后的真实夹爪反馈最终为 `4.3 mm`，与本轮标定的空夹全闭基线 `4.3 mm` 相同。目标中心前后只变化约 `3.6 mm`，也不支持“物体已被抓走”。因此本轮的真实裁决是：

```text
机械臂运动执行：成功
新 TCP 朝向：符合 GraspNet approach 语义
夹爪抓取保持：失败（抓空）
grasp_result.json 的 success：假阳性
```

在修复夹爪稳定反馈判定和最终保持验证之前，不应自动重复真机抓取。

## 已完成修改

### Docker 推理页面

- “预检成功候选”页面改为“机械臂执行最佳候选”。
- 页面只匹配并显示 `precheck_results.json.selected`，不再显示全部 `approved`。
- 宽松阈值命中多个候选时，按位置距离、姿态角和序号选出唯一最近项。
- 匹配失败时显示“当前帧无匹配的执行候选”，不再回退显示全量候选。
- 匹配遍历 NMS 后的全部候选，不受显示 `top_k=50` 限制。
- 页面缓存按 `selected` 位姿而不是只按 JSON 时间戳判定；实际执行仅重写同一候选时，保留已经匹配成功的最佳候选画面，避免短时感知波动把页面覆盖为空。
- 页面同步保存为 `shared_camera_data/result_selected.png`。

scan-only 后页面实测只显示一个抓手，并标注：

```text
机械臂执行最佳候选: rank 16
```

Docker 重启加载最终代码后的首帧再次命中 `rank 16`，日志记录为 `只显示机械臂执行最佳候选 1/64`；最终截图中也仅有该一个抓手。

### TCP 朝向矩阵

执行侧 `grasp_transform.py` 已从旧矩阵切换为：

```python
np.array([
    [ 0.0, 0.0, 1.0],
    [ 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
```

轴语义为：

```text
TCP +Z = GraspNet +X（接近/进深）
TCP +Y = GraspNet +Y（夹爪宽度）
TCP +X = -GraspNet +Z
```

执行侧和 Docker 推理侧矩阵逐元素一致，行列式为 `+1`。

### 执行候选安全门

- 预检规划通过后计算 `approach_down = -R_base_tcp[2,2]`。
- 仅 `approach_down >= 0.70` 的候选进入 `approved` 和执行选择集合。
- 从安全且可规划的候选中选择 `approach_down` 最大的一项。

相关 Python 编译通过，定向回归测试 `59 passed`，ROS 包重新构建成功。

### 当前最终候选选取模式

当前模式是“可执行性与接近方向优先”，不是“GraspNet 最高分优先”：

1. 单帧候选按 GraspNet `score` **升序**排列，最多检查 150 个；
2. `score >= grasp_score_min`，本轮阈值为 `0.0`；
3. 抓取宽度加 5 mm 余量不得超过 Piper 小夹爪最大 70 mm；
4. 物体中心与抓取中心横向距离不超过 80 mm；
5. approach、grasp 两阶段 MoveIt 规划均通过；
6. `approach_down >= 0.70`；
7. 在全部通过候选中选择 `approach_down` 最大者，完全相同时保留升序遍历中先出现者。

本轮候选 rank 16 的 `score=0.049211588`，在 91 个候选中是第 17 个最低分，
低于中位数 `0.088710181`，属于低置信度候选。它被选中不是因为 score 高，
而是因为规划通过且 `approach_down=0.968222` 为通过集合中的最大值。

### Piper SDK 夹爪最大开口

离线测试任务 `scripts/check_piper_gripper_width.py` 不连接 CAN，依据 SDK 得到：

```text
Demo 示例：0.05m × 1000 × 1000 = GripperCtrl(50000)
SDK 位置单位：0.001mm
SDK 默认小夹爪范围：[0.0, 0.07]m
本机小夹爪最大总开口：70mm
最大开口 SDK 指令：GripperCtrl(70000, ...)
本轮 rank 16 所需开口：52.388 + 5.000 = 57.388mm，可容纳
```

真机抓取现已在两层使用该 70 mm 限值：候选预检前跳过超宽候选；执行器在任何
张爪或机械臂运动前再次检查并 fail-closed。最大张开命令也统一使用 70 mm / 70000，
不再依赖代码中的隐式 `0.035×2` 推算。

## 输入与运行配置

```text
冻结 CSV：/tmp/grasp_execute_20260821_frozen.csv
CSV SHA256：9e062ec12b4817aecbdecd15b6bdf36c8b3aaf3db46ec71d4f284987b81c0f78
候选数：91

handeye：camera_data/handeye_eye_to_hand.yaml
handeye SHA256：14dcacc87184f1405c9d6c63ce16cfb9dcd18fb46cd7c93f2e670fb9dad4bd67

TCP offset：0.1358 m
grasp_z_offset：0.02 m
min_approach_down：0.70
真机执行速度比例：0.05
```

执行前状态：

- `can0`：1 Mbps，`ERROR-ACTIVE`，无 bus-off/error-pass/error-warn。
- Piper：`ctrl_mode=1`、`arm_status=0`、`err_code=0`，六轴通讯错误均为 false。
- `arm_controller`、`gripper_controller`、`joint_state_broadcaster` 均为 active。
- MoveIt：`allow_trajectory_execution=True`。
- 真机六轴反馈位于 zero。

## scan-only 预检

```text
candidates=91
tested=91
gate_rejected=0
orientation_rejected=5
approach_pass=18
grasp_pass=16
approved=11
selected=16
```

候选 16 在 scan-only 与实际执行预检中均被选中。

## 实际执行候选

| 字段 | 数值 |
|---|---|
| rank | 16 |
| GraspNet score | 0.049211588 |
| GraspNet width | 0.052387975 m |
| camera XYZ | `[-0.009642, -0.016714, 0.538000] m` |
| command TCP XYZ | `[0.490150, 0.023704, 0.015590] m` |
| command TCP quaternion wxyz | `[-0.106737, 0.723542, -0.678673, 0.067052]` |
| command link6 XYZ | `[0.457299, 0.015088, 0.147075] m` |
| approach_down | `0.968222` |

执行后、回零前的 MoveIt FK：

```text
link6 前进轴 = [+0.26, +0.11, -0.96]
FK link7 XYZ = [0.481034, 0.037562, 0.023254] m
FK link7 - command TCP = [-9.1, +13.9, +7.7] mm
误差范数 = 18.3 mm
```

该 FK 使用 MoveIt `/joint_states`（FakeSystem/控制器命令状态），可用于检查规划终点和姿态，但不是独立的真机反馈 FK。要得到严格的真实末端误差，下一轮必须在 grasp 终点同步保存 `/joint_states_feedback` 并离线 FK。

## 轨迹日志

MoveIt 记录三次轨迹执行均为 `SUCCEEDED`：

1. approach：16:55:46.548 开始，16:55:49.527 完成；
2. grasp：16:55:49.658 开始，16:55:50.537 完成；
3. 回 zero：16:55:51.788 开始，16:56:03.017 完成。

但 approach 没有走预期笛卡尔直线：

```text
Computed Cartesian path ... followed 0.000000%
GetCartesianPath_Response object has no attribute 'trajectory'
回退 OMPL
```

Humble 的 `GetCartesianPath` 响应字段为 `solution`，当前适配器错误读取 `resp.trajectory`。因此本轮 approach 实际是 OMPL 路径，不是直线插入路径。

MoveIt 仍缺少 `DepthImageOctomapUpdater`，所以规划有机器人自身碰撞约束，但没有真实桌面/物体深度碰撞场景。

## 夹爪假阳性根因

日志时序：

```text
full_open = 69.7 mm
full_closed = 4.3 mm
16:55:51.559  闭合命令完成发送
16:55:51.580  第一条新反馈 = 24.0 mm → 立即判定 grasped
回 zero 后最终真实反馈 = 4.3 mm
```

`_read_gripper_feedback_wait()` 当前只等待“时间戳更新”，收到第一条新反馈就返回；它没有等待开度稳定。`24.0 mm` 是夹爪由全开向全闭运动过程中的中间值，不是物体阻挡后的稳定开度。

冻结帧目标中心与执行后重新感知目标中心的差异约为：

```text
[+0.06, +0.61, +3.55] mm，范数 3.60 mm
```

该变化处于深度/分割波动量级，目标仍在原区域。结合最终开度回到空夹全闭基线，确认本轮抓空。

## 下一轮执行前必须修改

1. 修复 `moveit_adapter.compute_cartesian_path()`：读取 `resp.solution`，并检查 `error_code` 与 `fraction`。
2. 夹爪闭合后等待稳定窗口，而不是读取第一条反馈；建议至少连续 4–5 个样本开度变化小于 0.5–1.0 mm，并设置合理超时。
3. 只有稳定开度明显大于 `full_closed` 才能初判夹住。
4. 增加抬升/保持后的第二次开度验证，再写 `grasp_result.json success=true`。
5. 在 grasp 终点保存真实 `/joint_states_feedback`，用真实反馈 FK 取代 FakeSystem FK。
6. 修复完成前保持抓取服务停止，不自动重试真机。

## 固化产物

- `log/grasp_execute_frozen_20260821.csv`
- `log/grasp_scanonly_new_tcp_20260821.log`
- `log/grasp_execute_new_tcp_20260821.log`
- `log/move_group_new_tcp_20260821.log`
- `log/precheck_results_new_tcp_20260821.json`
- `log/grasp_result_new_tcp_20260821.json`
- `log/fk_link7_new_tcp_20260821.json`
- `log/result_selected_new_tcp_20260821.png`
