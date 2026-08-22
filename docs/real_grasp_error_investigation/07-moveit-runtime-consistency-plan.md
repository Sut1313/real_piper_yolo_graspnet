# MoveIt 运行时一致性与末端误差复测计划

## 1. 任务目的

本轮只调查 MoveIt 规划请求和执行状态链路是否可靠，目标是回答：

1. 发送给 MoveIt 的位置/姿态约束是否带有正确的 `base_link` 坐标系。
2. 真正运行的 Python 代码是否与当前源码一致。
3. `/joint_states`、`/joint_states_feedback` 是否完整、同步且包含 `joint8`。
4. `compute_cartesian_path` 是否因 ROS2 请求字段错误而始终回退 OMPL。
5. 在运行链路修正后，MoveIt 末端偏差是否仍达到 7–13 mm。

本轮不重新判断 TCP 长度。TCP 固定使用 URDF 权威值 `0.1358 m`。

## 2. 当前日志依据

当前已知问题来自以下证据：

- MoveIt 日志多次出现 `No frame specified for position constraint on link 'link6'`。
- 日志出现 `GetCartesianPath_Request object has no attribute 'eef_step'`，随后回退 OMPL。
- MoveIt 日志出现 `Missing joint8`。
- U4 仅有一次 dry-run，且未采集实际指尖中心误差和本次 FK link6。
- 阶段 D 的 `E_plan` 约为 7–13 mm，但尚未在干净的请求帧和完整关节状态下复现。

相关日志：

    docs/real_grasp_error_investigation/log/tcp_dry_run.json
    docs/real_grasp_error_investigation/log/moveit_tolerance_check.json
    log/investigation_20260818_U4_demo.log
    log/investigation_20260818_U4_execute.log

## 3. Claude Code 最小读取白名单

只允许读取以下文件或命令输出：

    docs/real_grasp_error_investigation/tcp-update-report.md
    docs/real_grasp_error_investigation/log/tcp_dry_run.json
    docs/real_grasp_error_investigation/log/moveit_tolerance_check.json
    docs/real_grasp_error_investigation/log/intrinsics_runtime_check.json
    docs/real_grasp_error_investigation/log/intrinsics_sensitivity.json
    log/investigation_20260818_U4_demo.log
    log/investigation_20260818_U4_execute.log
    src/real_piper_perception/real_piper_perception/moveit_adapter.py
    src/real_piper_perception/real_piper_perception/grasp_execute.py
    src/real_piper_perception/real_piper_perception/grasp_executor.py
    src/real_piper_perception/test/test_grasp_execute_units.py
    src/real_piper_perception/test/test_grasp_precheck.py
    /tmp/stageE_frozen.csv
    /tmp/stageD_report.json
    install/real_piper_perception/lib/python3.10/site-packages/real_piper_perception/moveit_adapter.py

允许执行的窄范围检查：

    sha256sum <上述源码和安装版文件>
    ros2 topic info /joint_states
    ros2 topic info /joint_states_feedback
    ros2 service type /compute_cartesian_path
    ros2 node list

禁止扫描或读取：

    build/
    install/ 下除指定安装版 moveit_adapter.py 以外的内容
    src/realsense-ros/
    src/piper_ros/src/piper_sim/
    graspnet-baseline/
    camera 原始图像、GraspNet 模型和无关历史日志
    其他调查文档、__pycache__、*.pyc

## 4. 本轮禁止事项

- 不修改 `TCP_OFFSET_FROM_LINK6_Z`。
- 不修改手眼 YAML、内参、轴映射或 GraspNet。
- 在 V0–V2 完成前不修改 MoveIt 容差。
- 不运行完整抓取，不闭合夹爪，不执行 lift。
- 不同时修改多个误差源。
- 不把 `FK link7` 直接当作真实指尖中心。
- 没有用户明确授权时，不得启动会移动真机的命令。

## 5. V0：只读建立运行时身份基线

不改代码、不启动真机运动。

检查：

1. 源码版和安装版 `moveit_adapter.py` 的 hash 是否一致。
2. 当前 launch 使用的 Python 包路径和模块路径。
3. `/compute_cartesian_path` 的实际 service 类型和字段。
4. `/joint_states`、`/joint_states_feedback` 的消息类型、发布节点和字段名。
5. 当前运行节点是否同时存在真实驱动、mock controller、joint state publisher 等可能互相覆盖状态的来源。

生成：

    docs/real_grasp_error_investigation/log/moveit_runtime_identity.json

必须包含：

    status
    generated_at
    commands
    exit_codes
    source_file
    installed_file
    source_sha256
    installed_sha256
    python_module_path
    ros_nodes
    service_types
    joint_state_publishers
    runtime_topology
    warnings
    missing_data

`runtime_topology` 必须明确写出：真实驱动、MoveIt、mock controller、joint_state_publisher 是否同时运行。不能只写“启动成功”。

## 6. V1：验证 MotionPlanRequest 的坐标系和容差字段

先使用冻结输入 `/tmp/stageE_frozen.csv`、候选 `[44]`，不移动机械臂。

允许修改的代码范围仅为：

- `moveit_adapter.py` 中约束消息的 frame 设置和诊断输出。
- `moveit_adapter.py` 中 `GetCartesianPath` 请求字段的兼容修正。
- 对应单元测试和日志代码。

不得在本阶段修改位置盒、关节容差、姿态容差的数值。

必须验证并记录：

    PositionConstraint.header.frame_id
    OrientationConstraint.header.frame_id
    PositionConstraint.link_name
    OrientationConstraint.link_name
    target position/orientation
    position box dimensions
    joint tolerance
    orientation tolerance
    request group_name
    request start_state joint names
    request goal constraint count

生成：

    docs/real_grasp_error_investigation/log/moveit_request_integrity.json

JSON 必须包含：

    status
    code_version/source_hash
    candidate_index
    csv_path/csv_sha256
    tcp_offset_m
    request_before
    request_after
    frame_id_check
    link_name_check
    tolerance_values
    moveit_warnings
    commands
    exit_codes
    conclusion
    missing_data

通过标准：

- 位置约束 frame 明确为 `base_link`。
- 姿态约束 frame 明确为 `base_link`。
- 约束 link 明确为 `link6`。
- 日志中不再出现 `No frame specified`。
- 运行时请求与源码预期一致。

若仍出现 frame 警告，必须标记 `status=blocked` 或 `status=failed`，不得进入真机复测。

## 7. V1.5：验证笛卡尔路径 API

只做 service/request 级检查和单元测试，不移动机械臂。

重点核对 ROS2 `GetCartesianPath.Request` 的实际字段。当前日志显示代码给请求设置了 `eef_step`，但实际请求对象不包含该字段；应根据 service 定义使用正确字段，通常由 `max_step` 承载步长。

生成：

    docs/real_grasp_error_investigation/log/cartesian_api_check.json

必须包含：

    service_type
    request_fields
    eef_step_input_m
    max_step_sent_m
    jump_threshold_sent
    avoid_collisions_sent
    api_exception_before
    api_exception_after
    fallback_to_ompl_before
    fallback_to_ompl_after
    unit_test_command
    exit_code
    conclusion

通过标准：

- 不再出现 `Request object has no attribute 'eef_step'`。
- 请求中步长字段为非零、可解释的值。
- 若调用 mock service，能得到明确 `fraction` 和返回轨迹结构。
- 任何 OMPL 回退都必须带原因和 `fraction`，不能静默回退。

## 8. V2：关节状态完整性与 FK 同步检查

不移动机械臂时先检查 zero 状态；如需真机采样，必须获得用户确认。

必须同时记录：

    /joint_states 的 name/position/stamp
    /joint_states_feedback 的 name/position/stamp
    joint1..joint8 是否全部存在
    两个话题的发布节点
    FK 使用的 RobotState 来源
    FK 请求时间与关节状态时间差

必须改进诊断落盘，使同一份 RobotState 同时计算并记录：

    FK link6
    FK link7
    FK link7 - FK link6

不能只记录 `FK link7`，也不能把 mock `/joint_states` 与真机反馈混合比较。

生成：

    docs/real_grasp_error_investigation/log/joint_state_completeness.json

必须包含：

    status
    topic_types
    publishers
    names_by_topic
    missing_joints
    duplicate_joints
    timestamps
    timestamp_delta_ms
    fk_source
    fk_link6_mm
    fk_link7_mm
    fk_link7_minus_link6_mm
    commands
    exit_codes
    warnings
    conclusion

通过标准：

- 规划和 FK 使用的关节状态来源明确且唯一。
- `joint1..joint8` 完整，或明确说明当前 arm-only FK 不需要 gripper 状态并证明不会影响目标比较。
- `FK link6` 与 `FK link7` 来自同一时间点和同一 RobotState。

## 9. V3：干净基线复测（用户授权后）

只有 V0、V1、V1.5、V2 全部通过后，才允许执行。

固定条件：

- `/tmp/stageE_frozen.csv`，记录 hash。
- 候选 `[44]`。
- TCP 偏移固定 `0.1358 m`。
- 起点固定 zero。
- `velocity_scale=0.1`。
- `dry_run_max_stage=grasp`。
- 不闭合夹爪，不执行 lift。
- 每组至少 3 次，每次前后回 zero。
- 不自动重试，用户在场，急停可用。

第一组只复测当前容差，不修改容差数值：

    PositionConstraint box = 0.02 m
    JointConstraint tolerance = 0.01 rad
    OrientationConstraint tolerance = 0.05 rad

每次必须记录：

    command_tcp_mm
    command_link6_mm
    planned_end_joints
    executed_feedback_joints
    FK_link6_mm
    FK_link7_mm
    E_plan = FK_link6 - command_link6
    E_link7 = FK_link7 - expected_link7
    stage/result/status
    all warnings/errors

生成：

    docs/real_grasp_error_investigation/log/moveit_clean_baseline.json

未授权或任一前置检查失败时，生成同名 JSON，写明：

    status=blocked
    reason
    completed_prechecks
    required_minimal_input

## 10. V4：容差 A/B（仅在 V3 干净通过后）

V4 是后续可选阶段，不得与 V3 同一批次修改。

保持所有输入、起点、候选和 TCP 不变，只改变 MoveIt 容差：

    A：0.02 m 位置盒、0.01 rad 关节、0.05 rad 姿态
    B：0.005 m 位置盒、0.003 rad 关节、0.02 rad 姿态

每组至少 3 次，分别计算：

    E_plan 每次 XYZ 和范数
    E_plan 平均值、标准差、最大值
    E_link7 每次 XYZ 和范数
    真机反馈关节误差
    规划失败率
    是否发生 OMPL 回退

生成：

    docs/real_grasp_error_investigation/log/moveit_tolerance_ab.json

通过参考：

- B 组 `E_plan` 平均值和最大值显著低于 A 组。
- B 组不出现新的规划失败或安全警告。
- `E_plan <5 mm` 才能认为容差问题得到有效控制。
- 不能仅凭 `FK link7` 改善就宣布真实抓取误差已解决。

## 11. 最终报告要求

生成：

    docs/real_grasp_error_investigation/moveit-runtime-consistency-report.md

报告必须包含：

1. 实际运行源码/安装版 hash 和 Python 模块路径。
2. 运行节点、service、关节状态发布者和 runtime topology。
3. MotionPlanRequest 的 frame、link、目标位姿和容差字段。
4. `No frame specified` 是否消失，若未消失必须说明阻塞点。
5. `GetCartesianPath` 字段错误是否修复，是否仍回退 OMPL。
6. `joint1..joint8` 完整性和 FK 状态来源。
7. `FK link6`、`FK link7`、command link6、command TCP 的分开数据。
8. V3 每次的 E_plan、E_link7、关节反馈和所有 warning/error。
9. V4 A/B 数据（若未执行必须写明原因）。
10. 本轮是否修改代码；逐文件列出函数、行号、hash 和测试结果。
11. 明确声明本轮没有修改 TCP、手眼、内参、轴映射和 GraspNet。
12. 不得把规划终点误差直接写成真实指尖误差。
13. 最终状态只能是：`已验证`、`部分验证`、`未验证`、`未通过` 或 `blocked`。
14. 下一步最多列 2 项，并且每项必须指定输入、命令和输出日志。

## 12. Claude Code 可直接执行的受限提示词

```text
目标：完成 07-moveit-runtime-consistency-plan.md，不要扫描全仓库。

只读取该文档第 3 节白名单。先执行 V0，只读建立运行时身份日志；V0 不通过时停止。
然后执行 V1/V1.5/V2，分别生成：
  docs/real_grasp_error_investigation/log/moveit_runtime_identity.json
  docs/real_grasp_error_investigation/log/moveit_request_integrity.json
  docs/real_grasp_error_investigation/log/cartesian_api_check.json
  docs/real_grasp_error_investigation/log/joint_state_completeness.json

V0–V2 禁止移动机械臂，禁止闭合夹爪，禁止修改 TCP、手眼、内参、轴映射、GraspNet
和 MoveIt 容差。只允许修复约束 frame、GetCartesianPath 请求字段和诊断落盘。
所有命令记录完整命令、退出码、输入 hash、源码/安装版 hash 和 warning/error。

只有 V0–V2 全部通过且用户明确授权，才可执行 V3；V3 只执行候选[44]到 grasp，
velocity=0.1，不闭合夹爪，每组至少 3 次，每次回 zero，生成 moveit_clean_baseline.json。
V4 容差 A/B 不得和 V3 同批次，必须另行确认后执行，生成 moveit_tolerance_ab.json。

最终生成 moveit-runtime-consistency-report.md。报告必须区分 command TCP、command link6、
FK link6、FK link7、实测指尖误差；没有实测指尖数据时只能写“未验证”，不得声称已解决 2–3 cm。
```

## 13. 本轮完成判定

只有同时满足以下条件，才算本任务完成：

- V0–V2 的 JSON 日志齐全。
- 运行时版本和请求字段可追溯。
- frame 警告、`eef_step` 异常和关节状态缺失均有明确通过/失败结论。
- V3 若未授权，必须有规范的 `blocked` 日志，而不是缺文件。
- 最终报告没有把 FK link7 误写为指尖实测误差。
