# Claude Code 当前轮次最小提示词

> 已被 `06-urdf-tcp-authoritative-plan.md` 的 URDF 权威 TCP 任务 supersede。TCP 任务请使用 06 文档；本文件只保留给未采用 URDF 权威值的旧补测流程。

将下面内容直接交给 Claude Code。它取代旧的全量 T0–T6 排查提示词。

```text
当前项目已有真机误差测量日志。不要重新执行完整 T0–T6，也不要扫描仓库。

目标：只核验四个问题：
F1. runtime CameraInfo 与 camera_data/calibration.yaml 的差异是否直接或间接造成 2–3cm 误差；
F2. E1/E2 是否错误地把 FK link7 与 command TCP 直接比较，必须按 link6/link7/TCP 几何重算；
F3. MoveIt 容差实际造成多少 link6 目标误差。
F4. TCP_OFFSET_FROM_LINK6_Z=0.14m 是否等于真实指尖夹持中心距离，并设计/记录 A/B 验证。

第一步只读取：
docs/real_grasp_error_investigation/measurement-log.md
docs/real_grasp_error_investigation/root-cause-report.md
docs/real_grasp_error_investigation/04-follow-up-work-list.md
docs/real_grasp_error_investigation/follow-up-report.md

然后只允许读取以下源码/配置：
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

如果最后两类日志路径不存在，只报告文件不存在，不要用 find 搜索其他目录。

禁止读取：build/、install/、src/realsense-ros/、src/piper_ros/src/piper_sim/、
graspnet-baseline/（除非已经明确给出某个文件路径）、__pycache__/、*.pyc、其他 docs 历史日志。

禁止：
- 修改任何源码、URDF、SRDF、YAML 或标定文件；
- 启动真实抓取、自动重试或改变机械臂参数；
- 重新运行 GraspNet、重新采集 T0、多次真机到位测试；
- 仅凭“calibration.yaml 与 runtime K 不同”就判定根因；
- 仅凭“FK link7 与 TCP 差 13.7mm”就判定 MoveIt 根因。
- 没有物理测量和用户确认就修改或执行 TCP A/B 测试。

F1：内参
1. 从已有日志读取 runtime K、meta.mat K、stored K。
2. 检查 runtime K 是否等于 meta.mat K，并精确列出差值。
3. 对同一像素和 z=0.4/0.6/0.8m，用 runtime K/stored K 分别反投影，计算 ΔX/ΔY/ΔZ/norm；分离主点和焦距贡献。
4. 只根据 handeye YAML 描述和允许文件判断 handeye 使用的内参来源；无法证明时写 unknown。

F2：几何
1. 从已有 E1/E2 数据读取 command TCP、command link6 和 FK link7。
2. 根据 URDF/SRDF 和代码，计算 expected link7，不要把 link7 与 TCP 直接当成同一点。
3. 输出：p_tcp_command、p_link6_target、p_link6_fk、p_link7_expected、p_link7_fk、
   E_plan、E_tool、E_wrong_compare；每项都给 XYZ 和 norm，单位 mm。

F3：MoveIt
1. 只读取 moveit_adapter.py 的 PositionConstraint、OrientationConstraint、JointConstraint 定义。
2. 使用已有 stageD 日志/报告重算 p_link6_target 与 p_link6_fk 的误差。
3. 给出 E_plan 的均值、最大值、XYZ 方向，并说明是否足以支持收紧容差。
4. 不修改 moveit_adapter.py。

F4：TCP
1. 只用允许的 URDF、SRDF、grasp_precheck.py、grasp_execute.py 计算当前 TCP 语义。
2. 记录 link6 基准面、铰点、左右指尖和两指夹持中心的测量定义。
3. 如果已有现场测量，检查至少 3 次读数并计算 L_tip_measured 和 delta_tcp；没有测量就写 blocked，不要猜值。
4. 用冻结 CSV 离线比较 A=0.140m 与 B=L_tip_measured 的 link6/link7/指尖预测位姿，不运行 GraspNet。
5. 只有用户明确确认后才允许真机单次 dry_run_max_stage:=grasp A/B；velocity=0.1、回 zero、禁止闭合和自动重试。
6. 记录 A/B 的 command、FK、实测误差和是否按 delta_tcp 方向变化；不修改 TCP 常量。

只生成/更新以下文件：
docs/real_grasp_error_investigation/log/intrinsics_runtime_check.json
docs/real_grasp_error_investigation/log/intrinsics_sensitivity.json
docs/real_grasp_error_investigation/log/tcp_link_geometry_check.json
docs/real_grasp_error_investigation/log/moveit_tolerance_check.json
docs/real_grasp_error_investigation/log/tcp_ab_test.json
docs/real_grasp_error_investigation/follow-up-report.md

每个 JSON 至少包含：status、inputs、commands、exit_codes、observations、conclusion、missing_data。
命令必须记录开始/结束时间、退出码和关键 stderr。没有采集的数据写“未采集 + 原因”。

最终 follow-up-report.md 必须包含：
1. runtime K 与 stored K 的数值差异；
2. 0.4/0.6/0.8m 的理论误差；
3. runtime 直接影响 vs handeye 间接影响；
4. 修正后的 E_plan/E_tool/E_total；
5. TCP 实测值、A/B 差异和是否支持修改 TCP 常量；
6. 是否支持收紧 MoveIt 容差；
7. 是否需要重新验证 handeye；
8. 证据对应的 test_id/frame_id/文件/函数行号；
9. 下一步最多两个任务。

不要输出泛泛的项目总结。
```
