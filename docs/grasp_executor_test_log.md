# Grasp Executor 真机联调测试日志

> 2026-08-07 真机联调（Task 9 scan_only 干跑 + 链路修复记录）。
> 前置：31/31 单测绿、scan_only 冒烟过（dev log 2026-08-06）。

## 一、Task 9：scan_only 干跑

### 1.1 环境

- CAN：`can_activate.sh can0 1000000`（can0 UP、ERROR-ACTIVE）
- 驱动：`start_single_piper.launch.py gripper_val_mutiple:=2 gripper_forward:=false`
  - `gripper_forward` 为 2026-08-07 新增参数：false 时驱动停止转发夹爪指令
    （夹爪由 grasp_executor piper_sdk 直控，避免 0x159 帧双源打架）
- MoveIt：`piper_with_gripper_moveit demo.launch.py`
- 感知：`run_d435i_bridge.sh` + `run_graspnet_d435i.sh gui`（相机 5000M USB3）
- executor：`grasp_execute.launch.py`（默认 scan_only=true）

### 1.2 干跑结果（2026-08-07 20:0x，物体垫高后）

```
scan_only report: candidates=27 tested=9 gate_rejected=2
approach_pass=2  grasp_pass=1  lift_pass=1
→ 候选 [10] target 全过（三段预检通过，臂不动）
```

**全链路打通**：CSV(27 候选) → self-consistency 门控 → 变换链（新 handeye
`handeye_eye_to_hand.yaml`，相机位置 (0.47,-0.008,0.703)）→ ssik 解析 IK
（部分候选有解）→ MoveIt 规划（`/move_action`，碰撞感知）→ FK 回验（ssik
residual 代理）→ 三段 AND 报告。

### 1.3 异常路径

- 删除 CSV（停感知容器后）→ 服务返回明确 message：
  `perception CSV not found: ... (run the perception pipeline first)`
- 注：scan_only 下 success=True（服务调用成功 + message 解释状态，非静默）；
  计划预期 failure 语义待用户裁决是否调整。

## 二、链路修复记录（2026-08-07 真机联调发现）

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| M6 | ssik 脚本路径不存在 | launch 用 `abspath`（不解析 symlink）→ install 路径解析失败 | 改 `realpath` + setup.py 打包 scripts/ 进 share |
| — | ssik `--matrix: expected one argument` | 17g 精度科学计数法负值（`-1.2e-05`）被 argparse 当选项 | 改 `--matrix=<value>` 等号形式 |
| — | `/move_group action server 不可用` | MoveIt2 humble action 名是 `/move_action`（MoveGroupMoveAction capability） | 适配器改连 `/move_action` |
| — | **plan() 规划即执行（致命）** | `MoveGroup.Goal.planning_options.plan_only` 默认 False | 显式 `plan_only=True`；执行仅走 `/execute_trajectory` |
| — | execute()/plan() 卡死/崩溃 | humble `send_goal` 阻塞式（禁止回调内用）；`Future.result()` 无 timeout 关键字；`get_result_async()` 返回 GetResult_Response 包装；Result 轨迹字段是 `planned_trajectory` | `send_goal_async` + 轮询 `_wait_future` + `.result().result` + `_PlanResult(planned_trajectory)` |
| — | executor 干跑 MoveIt plan 10s 超时（move_group 16ms 已响应） | 适配器 action client 在 node 默认 MutuallyExclusive 组，goal response 回调延迟 | 全部异步组件移入独立 `ReentrantCallbackGroup`（修复后 plan 秒级成功） |
| C1' | 驱动崩溃退出（rclpy shutdown 异常路径） | 驱动主循环异常退出（`isOk()`/CAN 失联）后 shutdown 二次异常；**轨迹失控根因是 plan_only 缺陷** | plan_only 修复后不再有未授权的执行 |
| — | 夹爪"直接开到最大"/张开不可控 | MoveIt gripper 组 → mock 轨迹走不完（mock 停在 0.0153/0.0252） | **SDK 直控**（用户裁决）：grasp_executor 直接 `GripperCtrl(angle,1000,0x01,0)`，0.001mm 单位、总开度×1e6；驱动 `gripper_forward:=false`。实测：张开 69.7mm（2 次可重复）、闭合 4.3mm、闭合→再张开 69.7mm ✅ |

## 三、标定/决策记录

1. **handeye 新标定**（用户更换）：相机位置 (0.47,-0.008,0.703) 垂直朝下
   （前向 ≈ -Z_base）。物体贴地映射 z≈0.03 ✅（旧文件 0.596 映射 z≈-0.07 地
   下，错误）。
2. **物体可达高度**：朝下/水平抓姿态需物体中心离地 ≈0.22-0.30m（link6 目标
   z≈0.25 起 ssik 有解；贴地 0.03 全无解）。
3. **TCP 轴映射回退**（用户裁决 2026-08-07）：恢复 C++ calib_18
   `[[0,-1,0],[0,0,-1],[1,0,0]]`（水平接近语义；此前试改朝下语义
   `[[0,0,1],[0,1,0],[-1,0,0]]` 后 ssik 0 解且位置偏差，已回退）。
4. **朝下门控**：搁置（用户指示"暂且不论"）。

## 四、遗留

- scan_only CSV 缺失 success 语义（见 1.3）
- ssik 对水平姿态部分无解 → 候选遍历兜底（analytic-only 门控默认开）
- mock 轨迹走不完问题（MoveIt gripper 组路径，已被 SDK 直控替代）
- 真机 SDK 读 GetArmEndPoseMsgs 返回 0（V1/V2 均），末端位姿验证改用
  URDF FK + 变换链计算

## 五、Task 10：低速单段 approach 执行验证（2026-08-07 20:5x）

**验收通过**（用户目测）：臂从 zero 低速（0.1）执行 张开夹爪(70mm) → 预位 → approach
（笛卡尔直线）→ **停在物体附近上方 ~5cm、无漂移**；夹爪水平接近（旧 calib_18 映射
语义，用户裁决保留）。

**关键修复（"臂不在物体上方"真正根因）**：
- `moveit_adapter.py` 的 `PositionConstraint` **缺 `header.frame_id='base_link'`** →
  约束区域在错误坐标系解释 → MoveIt 规划终点 z 偏 22cm（实测 0.412 vs 0.187），
  执行层"成功"但目标错误。
- 修复验证：规划终点 FK vs 目标偏差 (0.004, 0.010, 0.002) <1cm ✅；执行后
  FK (0.325,-0.063,0.269) + 用户目测在物体上方附近 ✅。

**方法论**：多层串联系统的"全部成功"可能是"全部自洽"——必须闭环交叉验证
（规划终点 FK vs 目标、执行后 FK vs 目标、目测独立参考）。用户提出的执行闭环比对
（FK 验证实际运动位置）定位了规划层错误（此前怀疑 handeye/映射/候选均为误判）。

**新增功能**：
- `dry_run_max_stage` 参数（Task 10 单段验证：open/pre_pose/approach/grasp/gripper/lift）
- 每次执行前自动回 zero（用户裁决）
- 候选选择方向偏好（全过候选中选 TCP 前向最朝下的；注：旧映射下全候选前向≈水平，
  该偏好当前区分度低，朝下映射时才有意义）

## 六、感知-执行互锁（2026-08-07 21:1x，用户需求）

**需求**：机械臂执行过程中 graspnet 停止推理（避免机械臂被识别进规划目标）；
推理开始前清空旧结果；执行完毕、机械臂归零后恢复推理。

**实现**（A 方案：无论结果流程结束都归零后恢复）：
- `inference_from_camera.py`：主循环检查 `shared_dir/pause_inference` 标记，
  存在 → 跳过帧不写 CSV（CSV 冻结在执行前干净帧）；启动时 cleanup_stale_outputs
  清空旧 CSV（原有机制）
- `grasp_executor.py`：`_run_flow` 开头写标记（推理冻结）→ finally：
  收尾归零（scan_only 跳过）→ 删标记（推理恢复）

**实测**：触发 → 感知冻结（执行期间 109 帧被跳过）→ 执行 → 收尾归零（真机全 0）
→ 标记清除 → 推理恢复。时序 = 用户要求的"执行中停止推理、归零后重新开始"。
