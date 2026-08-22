# 夹爪姿态与插入方向规划修改任务

## 1. 与当前 MoveIt 任务的边界

本文件只规划“GraspNet 姿态经过 TCP 映射后，夹爪是否能够沿正确方向插入物体空隙”的修改。

本文件不替代 `07-moveit-runtime-consistency-plan.md`。执行顺序必须是：

1. 先完成 07：确认 MoveIt 请求 frame、运行版本、关节状态和 Cartesian API。
2. 再执行本文件：增加姿态/插入方向约束和沿夹爪轴的 approach 位姿。
3. 最后重新做 MoveIt 容差 A/B 和真机抓取验证。

本任务不修改：

- URDF 权威 TCP `0.1358 m`。
- 手眼 YAML、相机内参和深度尺度。
- `GRASPNET_TO_PIPER_TCP_R`，除非轴语义验证日志证明矩阵确实错误。
- GraspNet 模型或网络输出。

## 2. 已知问题

当前候选选择逻辑只在预检全过的候选中偏好“开口朝下”，没有把“夹爪接近方向不能接近水平”作为硬约束。

现有日志中候选 `[44]`：

    link6 前向 z = -0.17
    开口朝下 = 0.67

该候选仍被选择，说明当前方向评分不能保证夹爪能够从上方插入物体空隙。

此外，`make_stage_poses()` 当前将 approach 位姿固定为基座 Z 方向上移，不能保证倾斜抓取时 approach→grasp 与夹爪插入轴共线。

## 3. 最小读取白名单

只允许读取：

    docs/real_grasp_error_investigation/07-moveit-runtime-consistency-plan.md
    docs/real_grasp_error_investigation/log/moveit_runtime_identity.json
    docs/real_grasp_error_investigation/log/moveit_request_integrity.json
    docs/real_grasp_error_investigation/log/joint_state_completeness.json
    src/real_piper_perception/real_piper_perception/grasp_transform.py
    src/real_piper_perception/real_piper_perception/grasp_precheck.py
    src/real_piper_perception/real_piper_perception/grasp_executor.py
    src/real_piper_perception/real_piper_perception/grasp_candidates.py
    src/real_piper_perception/test/test_grasp_precheck.py
    src/real_piper_perception/test/test_grasp_execute_units.py
    src/piper_ros/src/piper_description/urdf/piper_description.urdf
    src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
    /tmp/stageE_frozen.csv

禁止读取模型、相机原始图像、无关历史日志、`build/`、`graspnet-baseline/` 和仿真目录。

## 4. O0：先固定坐标轴语义

不改代码，只用 URDF、SRDF 和 FK 日志确认以下定义：

    R = base_link 下 link6 的旋转矩阵
    approach_axis = R[:, 2]       # link6/TCP +Z，指向末端前方
    opening_axis = -R[:, 1]       # joint7 X90° 后的 link7 滑移方向
    down_axis = [0, 0, -1]

必须用一个已知关节姿态调用 FK 验证 `opening_axis` 的方向，避免把旋转矩阵的行列索引混淆。

当前代码中的 `R[1,2]` 不得直接继续命名为“开口朝下”，除非 O0 日志证明它确实对应物理开口方向。

生成：

    docs/real_grasp_error_investigation/log/grasp_axis_semantics.json

必须包含：

    urdf/srdf source and hash
    joint7 origin/rpy/axis
    known joint state
    FK link6/link7 rotation
    approach_axis
    opening_axis
    measured_or_fk_link7_minus_link6_direction
    row_column_convention
    conclusion

## 5. O1：增加姿态安全门控

在候选进入 MoveIt 预检前计算：

    approach_down = dot(approach_axis, down_axis)
    opening_down = dot(opening_axis, down_axis)
    approach_tilt_deg = acos(clamp(approach_down, -1, 1)) * 180/pi

对于从上方插入的场景，初始建议参数：

    min_approach_down = 0.70
    max_approach_tilt_deg = 45

最终数值必须参数化，不得散落在代码中。若目标空隙方向已知，还应增加：

    opening_alignment = abs(dot(opening_axis, target_gap_axis))

并设置最小对齐阈值。

门控规则：

- 接近方向接近水平：直接拒绝。
- 开口方向不满足目标空隙方向：直接拒绝。
- 旋转矩阵非法或轴向量不为单位向量：直接拒绝。
- 只有方向门控通过后，才进入 IK/MoveIt 预检。

候选排序顺序改为：

    方向安全门控 → IK/MoveIt 可达 → score/宽度/方向余量排序

不得仅用一个“开口朝下分数”覆盖插入方向约束。

生成：

    docs/real_grasp_error_investigation/log/grasp_orientation_gate.json

每个候选必须记录：

    candidate_index
    R_tcp/R_link6
    approach_axis
    opening_axis
    approach_down
    opening_down
    approach_tilt_deg
    target_gap_axis（若有）
    opening_alignment（若有）
    thresholds
    gate_pass
    reject_reason
    selected_index

## 6. O2：沿插入轴生成 approach 位姿

当前固定基座 Z 偏移只适用于垂直向下抓取。确认轴语义后，改为：

    p_approach = p_grasp - approach_axis * approach_clearance

符号必须用 O0 的已知姿态验证：垂直向下抓取时，approach 点应位于 grasp 点上方。

必须记录：

    p_grasp
    p_approach
    approach_vector
    expected_vector
    vector_norm
    axis_alignment_error_deg

生成：

    docs/real_grasp_error_investigation/log/approach_axis_geometry.json

通过标准：

- approach→grasp 位移与插入轴夹角小于 1°。
- 垂直向下姿态仍保持原有基座 Z 上移行为。
- TCP、link6、姿态四元数和 approach clearance 均有记录。

## 7. O3：姿态变体策略（可选）

如果原始 GraspNet 姿态因夹爪方向不合格，不得直接裁剪 Euler 角。

只允许在保持 approach_axis 不变的前提下，绕 approach_axis 生成有限变体：

    0°、+15°、-15°、+30°、-30°

每个变体必须重新执行：

    TCP 位姿生成 → TCP→link6 折算 → IK → MoveIt 预检 → 碰撞检查

只有目标几何近似旋转对称，或 GraspNet 接触区域仍有效时，才允许启用变体。

生成：

    docs/real_grasp_error_investigation/log/grasp_orientation_variants.json

必须记录原始姿态、变体角度、轴向、预检结果、碰撞结果和最终选中原因。

## 8. 单元测试要求

至少增加以下测试：

1. 垂直向下姿态通过方向门控。
2. 接近水平姿态被拒绝。
3. approach 位移与 `approach_axis` 共线。
4. TCP→link6 折算仍使用 `0.1358 m`。
5. 旋转矩阵行列索引不会把 `R[1,2]` 错当作开口向下分量。
6. 方向门控失败时不调用 MoveIt 规划。
7. 姿态变体保持 approach_axis 不变。

测试结果写入：

    docs/real_grasp_error_investigation/log/grasp_orientation_tests.json

## 9. 真机验证要求

只有 O0–O2 和 07 计划的 MoveIt 运行时检查全部通过后，才允许真机验证：

- 用户在场并明确授权。
- 使用冻结 CSV 和固定候选，或记录新候选 hash。
- 速度不超过 `0.1`。
- 先执行到 approach，确认末端方向和插入路径。
- 再执行到 grasp，禁止自动闭合夹爪和自动重试。
- 每次执行前后回 zero。

生成：

    docs/real_grasp_error_investigation/log/grasp_orientation_dry_run.json

必须分别记录 command TCP、command link6、FK link6、FK link7、approach 轴、实际关节反馈和真实指尖误差。

## 10. 最终报告

生成：

    docs/real_grasp_error_investigation/grasp-orientation-planning-report.md

必须说明：

1. 最终采用的 TCP/link6/夹爪开口轴定义。
2. 当前旧逻辑的错误点及对应日志。
3. 姿态门控阈值和拒绝候选统计。
4. approach 位姿是否改为沿插入轴生成。
5. 是否生成姿态变体及其适用依据。
6. MoveIt 预检、规划和 FK 结果。
7. 是否采集真实指尖误差；未采集时只能写“效果未验证”。
8. 不得把 FK link7 位置直接写成真实指尖位置。
9. 不得同时归因于 TCP、相机内参或手眼矩阵。

最终状态只能是：`已验证`、`部分验证`、`效果未验证`、`未通过` 或 `blocked`。

## 11. 本任务完成条件

- 方向轴语义有 URDF/FK 证据。
- 水平或近水平候选可被稳定拒绝。
- approach→grasp 路径与夹爪插入轴一致。
- 方向门控、路径几何和测试日志齐全。
- 未经真实指尖测量和受控抓取，不能声称 2–3 cm 误差已经消除。
