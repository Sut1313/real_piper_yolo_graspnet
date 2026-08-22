# 当前日志后的最小补测工作清单

> 修订说明：用户已裁决“URDF 中的 TCP 长度是真实值”。本文件中原 F4 的现场物理测量/A-B 设计已由 `06-urdf-tcp-authoritative-plan.md` supersede；后续 TCP 工作以 06 为准。

## 1. 当前结论摘要

根据 `measurement-log.md` 和 `root-cause-report.md`，当前已知：

1. 在线链路使用 ROS 实时 `CameraInfo` 写入 `meta.mat`，GraspNet 读取 `meta.mat`；当前没有证据表明运行时直接使用了 `camera_data/calibration.yaml`。
2. 实时内参与 `calibration.yaml` 的主点存在较大差异：

   ```text
   runtime:   fx=604.214, fy=604.115, cx=321.781, cy=253.111
   stored:    fx=604.140, fy=601.120, cx=340.830, cy=238.670
   delta:     cx≈+19.05 px, cy≈-14.44 px
   ```

3. 如果在约 0.6 m 深度使用错误的旧内参反投影同一像素，主点差异大约对应：

   ```text
   ΔX ≈ 19.05 / 604.2 × 600 ≈ 18.9 mm
   ΔY ≈ 14.44 / 604.1 × 600 ≈ 14.3 mm
   ```

   合成量级约 23.7 mm，数值上足以解释 2–3 cm；但这只说明“若旧内参参与运行时反投影或手眼标定，可能产生该量级误差”，不能证明当前在线推理正在使用旧内参。

4. 阶段 B 已证明当前帧 `xyz_camera` 与实时深度/内参反投影基本一致，误差约 ≤2 mm；因此运行时直接的深度尺度/内参重复缩放暂时不支持为主因。
5. 阶段 D 证明机械臂执行和关节反馈误差通常 ≤2–3 mm，无明显正反向回差。
6. 阶段 E 观察到 FK 与 command TCP 存在约 13.7 mm 差异，必须先确认比较基准是否正确：MoveIt 约束的是 `link6`，而日志比较的是 `link7` 与 TCP；不能直接把 `FK link7 - command TCP` 全部归因于 MoveIt 容差。
7. 当前最有价值的未决问题有四个：

   - 手眼标定是否使用了与实时 CameraInfo 不同的内参；
   - `link6 → link7/指尖夹持中心` 的几何比较是否正确；
   - MoveIt 的位置/姿态容差实际造成了多少 `link6` 末端误差。
   - TCP 当前值 `0.14m` 与真实指尖夹持中心距离是否一致。

## 2. 本轮明确不再做的工作

为节省 Claude Code token，本轮不要重复：

- 全仓库扫描。
- `build/`、`install/`、`src/realsense-ros/`、仿真目录扫描。
- T0 多帧稳定性重新采集。
- 深度单位和 CSV 24 列格式的全面复查。
- 没有新证据时修改 `TCP_OFFSET_FROM_LINK6_Z`、手眼 YAML 或轴映射矩阵。
- 再次执行 15 次机械臂到位测试。
- 真实抓取或自动重试。

## 3. 允许读取的最小文件集合

优先只读以下文件和已有日志：

```text
docs/real_grasp_error_investigation/measurement-log.md
docs/real_grasp_error_investigation/root-cause-report.md
camera_data/calibration.yaml
camera_data/handeye_eye_to_hand.yaml
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/handeye_tf_publisher.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_execute.py
src/real_piper_perception/real_piper_perception/moveit_adapter.py
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
shared_camera_data/meta.mat
shared_camera_data/perception_grasps.csv
docs/real_grasp_error_investigation/log/investigation_20260818_stageD.log
docs/real_grasp_error_investigation/log/investigation_20260818_*.log
```

如果某个日志文件不存在，记录“文件不存在”，不要为了寻找它扫描整个 `docs/`。

## 4. 补测任务 F1：内参差异的因果核验

目标：区分“运行时错误使用旧内参”和“手眼标定阶段使用旧内参造成外参偏差”。

### F1.1 运行时链路核对

只确认以下事实：

```text
ROS CameraInfo K
→ d435i_graspnet_bridge._write_meta
→ shared meta.mat intrinsic_matrix
→ inference_from_camera.read_data_from_shared
→ create_point_cloud_from_depth_image
```

输出 `intrinsics_runtime_check.json`，字段必须包括：

```json
{
  "camera_info_K": [],
  "meta_mat_K": [],
  "max_abs_difference": 0.0,
  "factor_depth": 1000.0,
  "calibration_yaml_K": [],
  "runtime_uses_calibration_yaml": "yes/no/unknown"
}
```

通过标准：`camera_info_K` 与 `meta_mat_K` 一致；如果一致，说明当前输入点云没有直接使用 `calibration.yaml`。

### F1.2 内参敏感性计算

不要重新运行 GraspNet。使用同一组像素 `(u,v)` 和深度 `z=0.4/0.6/0.8m`，分别用 runtime K 和 stored K 反投影，输出：

```text
每个深度的 ΔX、ΔY、ΔZ、norm
主点误差单独贡献
焦距误差单独贡献
```

输出 `intrinsics_sensitivity.json`。

解释规则：

- 若旧内参只用于离线标定而运行时使用 runtime K：不能把本项直接归因给在线点云。
- 若手眼标定使用旧 K：必须标记“手眼外参可能吸收内参误差”，进入 F2。
- 若无法知道手眼标定使用的 K：报告为关键未知，不得标记为已排除。

### F1.3 手眼标定来源核验

只读取 `handeye_eye_to_hand.yaml` 的描述、时间、矩阵字段和 `calibration.yaml`，不要扫描历史文档。输出：

```text
handeye 生成时间/描述（若有）
是否明确记录使用的 CameraInfo/K
是否与 runtime K 相同
是否与 calibration.yaml 相同
来源未知时明确写 unknown
```

如果来源未知，下一步优先级是重新使用当前 runtime K 做一次独立手眼验证，而不是直接替换现有 YAML。

## 5. 补测任务 F2：修正 link6/link7/TCP 比较基准

目标：重新计算 E1/E2 的理论几何位置，避免把不同物理点之间的距离当作 MoveIt 误差。

只读取 `grasp_precheck.py`、`grasp_execute.py`、URDF 和 SRDF。

对每个候选计算并分别报告：

```text
p_tcp_command
p_link6_target = TCP→link6 的代码折算结果
p_link7_expected = 根据 URDF joint7 origin 和当前 joint7 值计算
p_link7_fk = /compute_fk 实际结果
E_plan = p_link6_fk - p_link6_target
E_tool = p_link7_fk - p_link7_expected
E_wrong_compare = p_link7_fk - p_tcp_command（仅作为诊断，不作为执行误差）
```

输出 `tcp_link_geometry_check.json`，必须包含每个向量的 XYZ 和范数。

通过标准：先得到 `E_plan` 和 `E_tool`，再讨论 MoveIt 容差或 TCP 偏移；禁止继续使用 `p_link7_fk - p_tcp_command` 作为唯一误差。

## 6. 补测任务 F3：MoveIt 容差的最小归因

目标：确认 `moveit_adapter.py` 中位置/姿态容差对 `link6` 的实际影响。

优先使用已有 stage D 数据离线计算；只有日志不足时，才请求一次无目标、低速、用户在场的重复测试。

读取范围仅限：`moveit_adapter.py`、stage D 日志、`stageD_report.json`（若存在）。

报告：

```text
PositionConstraint 的盒尺寸和实际允许范围
OrientationConstraint 三轴容差
JointConstraint 容差
每次 p_link6_target、p_link6_fk、E_plan
E_plan 的均值、最大值、XYZ 方向
```

输出 `moveit_tolerance_check.json`。

注意：只有 `E_plan` 持续超过目标阈值并且方向与容差模型一致时，才支持收紧 MoveIt 容差。不能仅凭 `link7 - TCP` 的 13.7 mm 直接修改 `moveit_adapter.py`。

## 7. 补测任务 F4：TCP 实测与 A/B 验证

目标：确认 `TCP_OFFSET_FROM_LINK6_Z=0.14m` 是否与真实的“link6 基准面到指尖夹持中心”一致，并估计它对最终误差的贡献。

### F4.1 先做物理测量，不改代码

只读取 `grasp_precheck.py`、`grasp_execute.py`、URDF、SRDF 和 link6/link7/link8 几何定义。

现场测量必须记录：

```text
link6 基准面定义
joint7/joint8 铰点位置
左指尖夹持面位置
右指尖夹持面位置
两指夹持中心位置
测量方向、工具坐标轴、单位
至少 3 次读数和平均值
```

计算：

```text
L_tip_measured = link6 基准面 → 两指夹持中心
delta_tcp = L_tip_measured - 0.140m
```

如果测量基准不明确、指尖夹持中心无法定义或只能目测，状态必须写 `blocked`，不能修改 TCP。

### F4.2 离线 A/B 计算

使用同一个冻结 CSV、同一个候选、同一个手眼矩阵，分别计算：

```text
A: TCP_OFFSET_FROM_LINK6_Z = 0.140m
B: TCP_OFFSET_FROM_LINK6_Z = L_tip_measured
```

不运行 GraspNet，不执行机械臂。输出 A/B 的：

```text
link6_target XYZ
expected link7 XYZ
expected fingertip center XYZ
A/B 位移差 XYZ 和 norm
```

### F4.3 受控真机验证

只有 F4.1 测量完成、F2 几何比较正确且用户明确确认后才允许执行。

测试约束：

1. 使用同一冻结 CSV 和同一候选。
2. 先 A 后 B，或两次测试顺序随机化；每次之间回 zero。
3. `scan_only:=false` 只执行到 `dry_run_max_stage:=grasp`，不闭合夹爪、不抓取物体。
4. velocity 保持 0.1，用户在场，单次执行，不自动重试。
5. 每次记录 command TCP、command link6、FK link6/link7 和实测指尖/目标误差。

通过标准：在相同感知输入和相同 MoveIt 配置下，B 相比 A 的 `E_tool` 或实测指尖误差是否按 `delta_tcp` 方向和量级变化；如果变化不符合，不能把误差归因于 TCP。

输出 `tcp_ab_test.json`，必须包含 A/B 输入、测量值、命令、退出码、FK、实测误差和结论。

## 8. 本轮最终输出

Claude Code 只生成/更新以下文件：

```text
docs/real_grasp_error_investigation/log/intrinsics_runtime_check.json
docs/real_grasp_error_investigation/log/intrinsics_sensitivity.json
docs/real_grasp_error_investigation/log/tcp_link_geometry_check.json
docs/real_grasp_error_investigation/log/moveit_tolerance_check.json
docs/real_grasp_error_investigation/log/tcp_ab_test.json
docs/real_grasp_error_investigation/follow-up-report.md
```

`follow-up-report.md` 必须包含：

1. 当前已确认事实，引用已有 `test_id/frame_id`。
2. 内参差异的数值影响，以及“直接运行时影响”和“手眼标定间接影响”的区分。
3. F1–F4 每项的输入、命令、退出码、输出文件和结论。
4. 修正后的 `E_plan`、`E_tool`、`E_total` 分解。
5. 是否支持收紧 MoveIt 容差，给出证据而非建议性猜测。
6. TCP 实测值、A/B 差异和是否支持修改 `TCP_OFFSET_FROM_LINK6_Z`。
7. 是否必须重新标定手眼，说明缺失的标定来源证据。
8. 下一步最多 1–2 个任务，禁止重新列出全套 T0–T6。

如果无法执行某项，只生成对应 JSON，填入：

```json
{"status":"blocked","reason":"具体原因","required_minimal_input":"下一步只需要的输入"}
```
