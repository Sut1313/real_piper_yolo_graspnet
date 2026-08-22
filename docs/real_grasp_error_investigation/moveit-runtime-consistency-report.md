# MoveIt 运行时一致性阶段报告

## 当前状态

**部分验证，真机复测 blocked。**

本轮完成了代码静态核对、ROS2 Cartesian service 接口核对、最小代码修复、编译和测试；由于当前环境无法访问 ROS graph，尚未证明真实运行时的约束 warning、关节状态和 Cartesian fraction 已恢复正常。

## 已完成修改

文件：`src/real_piper_perception/real_piper_perception/moveit_adapter.py`

1. 为 `OrientationConstraint` 设置 `header.frame_id = base_link`。
2. 为 `GetCartesianPath.Request` 设置 `header.frame_id = base_link`。
3. 为 Cartesian 请求设置 `link_name = link6`。
4. 将错误字段 `req.eef_step` 改为 ROS2 Humble service 实际字段 `req.max_step`。
5. 不修改 TCP、MoveIt 容差、手眼、相机内参、轴映射和 GraspNet。

源码和安装版 hash 已一致：

    cc963e1e2275f4ef97397ef253274b556bf5ebae38e234fd287e03faec30146f

## 验证结果

| 检查 | 结果 |
|---|---|
| `GetCartesianPath` service 定义 | `max_step` 存在，`eef_step` 不存在 |
| Python 语法检查 | 通过 |
| `colcon build --packages-select real_piper_perception` | 通过 |
| TCP/预检相关测试 | 20 passed |
| 全量测试 | 50 passed，2 failed |

全量测试的两个失败：

1. `test_grasp_executor_mte.py`：ROS 日志目录 `~/.ros/log` 只读，无法初始化 `rclpy`。
2. `test_sdk_safe_execute.py::test_box_clearance`：既有 `assert -0.0 < 0`，与本轮文件无关。

## 尚未完成

- 未能查询真实 ROS nodes、topic publishers 和 runtime topology。
- 未能读取 live `/joint_states`、`/joint_states_feedback`。
- 未能确认 `joint8` 是否仍缺失。
- 未能确认 MoveIt live `MotionPlanRequest` 的 frame 是否正确。
- 未能确认 Cartesian path 的 live `fraction` 和是否仍回退 OMPL。
- 未执行真机运动。

## 2026-08-18 本次实验执行结果

### 离线/单元测试

- 变换、预检、执行单位和候选测试：`29 passed`。
- `GetCartesianPath.Request`：确认存在 `max_step`，不存在 `eef_step`。
- 输入冻结文件 hash：`a80a1996bcf4367b7dbcb2b6d28f7a658279ca031acfe0eb71a922e15ad8ec7d`。

### ROS graph 只读探测

只发现 `/parameter_events` 和 `/rosout`，没有：

- `/joint_states`
- `/joint_states_feedback`
- `/move_group`
- 真实驱动、控制器或 joint state publisher

详见 [moveit_live_probe.json](./log/moveit_live_probe.json)。

### scan_only 实验

`scan_only=true` 节点成功启动并读取冻结 CSV，服务触发后完成 62 个候选遍历：

    candidates=62, tested=41, gate_rejected=21
    approach_pass=0, grasp_pass=0, selected=-1

失败原因不是候选几何结论，而是 `/move_group action server` 不存在；机械臂没有运动。详见 [scan_only_execution_20260818.json](./log/scan_only_execution_20260818.json)。

## 必须在真实 ROS 会话中完成的最小复测

1. 启动与实际拓扑匹配的 `move_group`、robot state 和关节状态发布者。
2. 使用冻结候选 `[44]`，重新执行 `scan_only=true`，确认不再出现 action server/frame warning。
3. 执行仅到 `approach` 的 dry-run，记录 Cartesian `fraction` 和是否 fallback OMPL。
4. 记录同一 RobotState 的 FK `link6/link7`，确认 `joint1..joint8` 状态来源。
5. 以上通过后，才允许进行 3 次 `grasp` 阶段 MoveIt 误差复测。

详细字段要求见 [07-moveit-runtime-consistency-plan.md](./07-moveit-runtime-consistency-plan.md)。夹爪姿态与插入方向修改见 [08-grasp-orientation-planning-modification.md](./08-grasp-orientation-planning-modification.md)，不得与本轮混改。
