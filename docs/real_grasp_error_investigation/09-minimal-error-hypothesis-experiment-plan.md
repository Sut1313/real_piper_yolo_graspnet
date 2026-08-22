# 2–3 cm 真机抓取误差：Claude Code 最小证据链实验计划

## 1. 任务目标

本任务只回答三个问题：

1. 误差是 MoveIt 规划产生的随机终点误差，还是手眼/内参造成的固定系统偏差？
2. 当前运行时是否真的使用了完整的 MoveIt、关节状态和 FK 链路？
3. 夹爪姿态接近水平的问题是否与位置误差分离存在？

当前不修改机械臂规划代码，不修改 TCP、手眼、内参、轴映射、GraspNet 或 MoveIt 容差。

TCP 固定为 URDF 权威值：`0.1358 m`。

## 2. 当前最小假设集

| 编号 | 假设 | 需要什么证据 |
|---|---|---|
| H1 | MoveIt 位置约束造成 7–13 mm 随机终点误差 | 同一目标、同一起点、重复 plan 的 `E_plan` 分布 |
| H2 | 手眼/标定内参不一致造成固定 2–3 cm 偏差 | runtime K、手眼标定 K 来源、至少 3 个绝对真值点 |
| H3 | 夹爪姿态/接近方向不合适导致无法插入空隙 | 选中候选的 TCP/link6 轴向、approach 路径与开口方向 |
| H4 | TCP 是主要根因 | 当前不再验证数值；0.1358 m 固定，4.2 mm 已知变化不足以解释全误差 |
| H5 | 机械执行误差是主要根因 | `/joint_states_feedback` 与规划终点的关节/末端差异 |

## 3. Claude Code 最小读取白名单

只读取：

    docs/real_grasp_error_investigation/07-moveit-runtime-consistency-plan.md
    docs/real_grasp_error_investigation/08-grasp-orientation-planning-modification.md
    docs/real_grasp_error_investigation/tcp-update-report.md
    docs/real_grasp_error_investigation/log/tcp_dry_run.json
    docs/real_grasp_error_investigation/log/moveit_tolerance_check.json
    docs/real_grasp_error_investigation/log/intrinsics_runtime_check.json
    docs/real_grasp_error_investigation/log/intrinsics_sensitivity.json
    docs/real_grasp_error_investigation/log/moveit-runtime-consistency-report.md
    src/real_piper_perception/real_piper_perception/moveit_adapter.py
    src/real_piper_perception/real_piper_perception/grasp_transform.py
    src/real_piper_perception/real_piper_perception/grasp_precheck.py
    src/real_piper_perception/real_piper_perception/grasp_executor.py
    src/real_piper_perception/real_piper_perception/grasp_execute.py
    camera_data/handeye_eye_to_hand.yaml
    camera_data/calibration.yaml
    /tmp/stageE_frozen.csv
    /tmp/stageD_report.json

允许的窄范围命令：

    sha256sum <指定文件>
    rg -n "CameraInfo|intrinsic|meta.mat|calibration.yaml|handeye|R_cam2base|t_cam2base" <指定文件>
    ros2 node list --no-daemon --spin-time 2
    ros2 topic list --no-daemon --spin-time 2
    ros2 topic info --verbose --no-daemon --spin-time 2 /joint_states
    ros2 topic info --verbose --no-daemon --spin-time 2 /joint_states_feedback
    ros2 service list --no-daemon --spin-time 2
    ros2 service type /compute_cartesian_path

禁止：

- 扫描全仓库或递归读取 `build/`、`install/`、仿真目录、GraspNet 模型。
- 读取相机原始图像、点云和无关历史日志。
- 重跑 GraspNet 或重新生成感知 CSV。
- 修改任何项目源码、YAML、launch、URDF、MoveIt 配置。
- 启动真实驱动、控制器或任何可能移动机械臂的命令，除非进入 P4 且用户明确授权。

## 4. 固定实验输入

所有阶段优先使用同一冻结输入：

    CSV: /tmp/stageE_frozen.csv
    候选: [44]
    TCP_OFFSET_FROM_LINK6_Z: 0.1358 m
    grasp_z_offset: 0.02 m
    approach_clearance: 0.05 m
    起点: zero

开始前必须计算并记录 CSV hash。若 hash 变化，立即停止，不得比较新旧结果。

## 5. P0：环境与输入冻结检查

只读，不运动，不调用抓取服务。

记录：

- 源码与安装版 `moveit_adapter.py` hash。
- 当前 Python 模块路径。
- CSV hash、候选编号和 TCP 值。
- ROS nodes、topics、MoveIt actions/services。
- `/joint_states` 与 `/joint_states_feedback` 的发布者和消息类型。

生成：

    docs/real_grasp_error_investigation/log/hypothesis_runtime_identity.json

若没有 `/move_group`、`/joint_states` 或 `compute_cartesian_path`，写 `status=blocked` 并停止 P1–P4，不要把“无服务”当成候选不可达。

## 6. P1：无运动 scan_only 运行链路检查

前提：P0 通过；必须只启动 MoveIt/RSP/JSP 等无运动规划环境，不能启动真实驱动。

执行：

    source ./install/setup.bash
    ros2 launch .../grasp_execute.launch.py \
      scan_only:=true \
      csv_path:=/tmp/stageE_frozen.csv
    ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'

只读取以下结果：

- 是否出现 `No frame specified`。
- 是否出现 `eef_step` 异常。
- MoveIt 预检调用是否真正返回。
- `candidates/tested/gate_rejected/approach_pass/grasp_pass`。
- 候选 `[44]` 的 command TCP、command link6、姿态轴和预检结果。

生成：

    docs/real_grasp_error_investigation/log/hypothesis_scanonly.json

没有 MoveIt 服务时：`status=blocked`，记录服务缺失，不输出方向或可达性结论。

## 7. P2：同一目标重复 plan，区分随机误差与固定偏差

前提：P1 通过；只做 plan-only，不执行轨迹，不启动真实驱动。

对候选 `[44]`、zero 起点重复规划至少 6 次。每次只记录终点和误差，不保存完整规划轨迹：

    command_tcp_mm
    command_link6_mm
    planned_end_joints
    FK_link6_at_plan_end_mm
    E_plan_xyz_mm = FK_link6 - command_link6
    E_plan_norm_mm
    orientation_error_deg
    planning_time_s
    plan_result/error_code
    MoveIt warnings

FK 必须使用规划终点关节，不得使用上一次真机反馈或 mock 状态混合计算。

生成：

    docs/real_grasp_error_investigation/log/hypothesis_moveit_repeat.json

判定：

- `E_plan` 范数约 7–13 mm、方向随机：支持 H1。
- `E_plan` 均值向量稳定且标准差小于 3 mm：不能归因于随机容差，应优先检查 frame/目标定义/手眼变换。
- `E_plan <5 mm`：MoveIt 不是主要误差源，进入 H2/H3。

## 8. P3：内参与手眼来源核对

只读，不重新标定，不修改 YAML。

记录：

1. runtime `CameraInfo.K`。
2. `meta.mat` 中的 K。
3. `calibration.yaml` 中的 K。
4. 运行时是否引用 `calibration.yaml`。
5. 手眼 YAML 是否记录标定时使用的 K、时间戳、标定工具或输入图像。
6. `R_cam2base/t_cam2base` 的 hash 和数值摘要。

已知敏感性结果若输入未变，不重复计算；只引用现有 `intrinsics_sensitivity.json`。

生成：

    docs/real_grasp_error_investigation/log/hypothesis_intrinsics_provenance.json

判定：

- runtime K = meta.mat K 且 handeye 标定明确使用 runtime K：H2 降级。
- handeye 标定使用 stored K 或来源不明：H2 保留为关键未知。
- 不得因为 `calibration.yaml` 未被运行时代码引用，就声称手眼标定已经排除内参问题。

## 9. P4：受控真机 dry-run（必须用户明确授权）

P4 只在 P0–P3 通过后执行；没有授权时写 blocked 日志，不得自行启动。

固定条件：

- 真实驱动和 MoveIt 拓扑必须明确，禁止 driver/mock controller 混用。
- 速度 `0.1`。
- 候选 `[44]`，或新候选必须记录 hash 和完整输入。
- 只执行到 `grasp`，不闭合夹爪、不 lift、不自动重试。
- 每次前后回 zero。
- 至少 3 次。
- 用户在场，急停可用。

每次记录：

    command_tcp_mm
    command_link6_mm
    planned_end_joints
    /joint_states
    /joint_states_feedback
    FK_link6_mm
    FK_link7_mm
    E_plan_xyz/norm
    FK link7 - expected link7
    实测指尖/夹持中心 XYZ（无法测量则 null）
    真实目标中心 XYZ
    E_total_xyz/norm
    stage/status/error/warnings

生成：

    docs/real_grasp_error_investigation/log/hypothesis_dry_run_repeat.json

如果没有真实指尖 XYZ，只能分析规划和 FK，最终状态必须为“效果未验证”。

## 10. 姿态问题只做观测，不在本任务修改

在 P1/P4 中附带记录，不改变姿态：

    R_tcp/R_link6
    approach_axis = R_link6[:,2]
    opening_axis（按 URDF/FK 定义）
    approach_down
    opening_down
    approach_tilt_deg
    p_grasp
    p_approach
    approach_vector
    approach_vector 与 approach_axis 夹角

这部分只用于确认 H3，不得在本任务中增加姿态门控或修改 approach 生成逻辑；姿态修改另按 `08-grasp-orientation-planning-modification.md` 执行。

## 11. 最终报告要求

生成：

    docs/real_grasp_error_investigation/hypothesis-analysis-report.md

报告必须：

1. 用一张表列出每个假设、证据、样本数、状态和置信度。
2. 分开 command TCP、command link6、FK link6、FK link7、实测指尖误差。
3. 给出 `E_plan` 的均值、标准差、最大值和方向分布。
4. 明确固定偏差还是随机偏差，不能只给单次误差。
5. 明确 runtime K、meta.mat K、stored K 及 handeye K 来源。
6. 说明姿态问题是否观察到，但不得把它直接写成位置误差根因。
7. 记录每条命令、退出码、输入 hash 和所有 warning/error。
8. 未执行的阶段写 `blocked` 和最小缺失条件。
9. 不得把 TCP 4.2 mm 变化写成 2–3 cm 根因。
10. 最终状态只能是：`已确认`、`部分确认`、`效果未验证`、`未通过` 或 `blocked`。
11. 最多列出 2 个下一步任务。

## 12. Claude Code 可直接使用的最小提示词

```text
执行 09-minimal-error-hypothesis-experiment-plan.md。

本轮禁止修改任何源码、YAML、launch、URDF、MoveIt 配置、TCP、手眼、内参、轴映射和 GraspNet。
只读取第 3 节白名单；禁止全仓扫描、重跑 GraspNet 和读取原始图像。

先执行 P0。若没有 /move_group、/joint_states 或 compute_cartesian_path，生成
hypothesis_runtime_identity.json，status=blocked，并停止 P1–P4。

P0 通过后执行 scan_only P1，只启动无运动规划环境；固定使用 /tmp/stageE_frozen.csv、候选[44]、
TCP=0.1358。P1 只记录请求 warning、预检统计、command TCP/link6 和姿态轴，不执行轨迹。

P1 通过后执行 P2：同一候选、zero 起点，plan-only 重复 6 次。只输出每次 planned_end_joints、
FK link6、E_plan XYZ/norm、规划结果和 warning，禁止保存完整轨迹。用重复分布判断随机 MoveIt
误差还是固定变换误差。

随后只读执行 P3，核对 runtime K、meta.mat K、calibration.yaml K 和手眼标定 K 来源；没有来源
就写 unknown，不要推测。

P4 真机 dry-run 必须等用户明确授权；只到 grasp、不闭合夹爪、不 lift、不重试，至少 3 次，
必须记录 command/FK/反馈/实测指尖 XYZ。没有实测指尖数据只能写“效果未验证”。

每个阶段生成独立 JSON，所有命令记录退出码、输入 hash、源码 hash 和 warning/error。最后生成
hypothesis-analysis-report.md，严格区分 MoveIt、手眼/内参、姿态和 TCP 结论。
```
