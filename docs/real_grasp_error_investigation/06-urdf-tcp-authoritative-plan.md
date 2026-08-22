# 基于 URDF 权威 TCP 长度的新任务计划

## 1. 裁决与适用范围

用户裁决：URDF 中定义的 TCP 长度是真实值，不再要求现场卷尺或卡尺测量。

当前项目采用的权威值：

    joint6_to_gripper_base origin = [0, 0, 0]
    joint7 origin z = 0.1358 m
    joint8 origin z = 0.1358 m
    TCP_OFFSET_FROM_LINK6_Z = 0.1358 m

来源：

    src/piper_ros/src/piper_description/urdf/piper_description.urdf
      joint6_to_gripper_base: lines 375-383
      joint7 origin: lines 425-440
      joint8 origin: lines 483-499

坐标语义：

- gripper_base 与 link6 重合。
- 抓取 TCP 目标沿 TCP 局部 Z 轴定义。
- TCP 到 link6 折算使用右乘 T(0,0,-0.1358)。
- joint7/joint8 的 rpy=1.5708 和 prismatic axis 只用于 link7/link8 FK 几何验证，不可把 link7 铰点直接等同于 TCP 指尖中心。
- 不再使用 0.14m，也不再等待现场指尖测量来决定该常量。

## 2. 本轮目标

1. 将代码中的 TCP 偏移从 0.14m 统一切换到 URDF 权威值 0.1358m。
2. 修正所有注释、测试断言和诊断日志中的旧值。
3. 用冻结 CSV 做离线前后对比。
4. 用 scan_only 和无目标低速 dry-run 验证，不立即进行真实抓取。
5. 单独记录 TCP 修改带来的位姿变化，避免与 MoveIt 容差修改混淆。

## 3. Claude Code 最小读取白名单

只允许读取：

    docs/real_grasp_error_investigation/measurement-log.md
    docs/real_grasp_error_investigation/follow-up-report.md
    camera_data/handeye_eye_to_hand.yaml
    src/real_piper_perception/real_piper_perception/grasp_precheck.py
    src/real_piper_perception/real_piper_perception/grasp_execute.py
    src/real_piper_perception/real_piper_perception/grasp_executor.py
    src/real_piper_perception/test/test_grasp_execute_units.py
    src/real_piper_perception/test/test_grasp_precheck.py
    src/piper_ros/src/piper_description/urdf/piper_description.urdf
    src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
    shared_camera_data/perception_grasps.csv
    /tmp/stageE_frozen.csv
    /tmp/stageD_report.json

禁止读取：

    build/
    install/
    src/realsense-ros/
    src/piper_ros/src/piper_sim/
    graspnet-baseline/
    其他历史 docs、__pycache__、*.pyc

## 4. U0：建立 URDF 权威值日志

不改代码。读取 joint6_to_gripper_base、joint7、joint8 和 SRDF 的 arm/gripper 定义。

生成：

    docs/real_grasp_error_investigation/log/tcp_urdf_authority.json

必须包含：

    status
    source_files
    tcp_offset_from_link6_z_m = 0.1358
    joint6_to_gripper_base_origin
    joint7_origin/rpy/axis/limit
    joint8_origin/rpy/axis/limit
    arm_tip = link6
    gripper_group_tip = link7
    semantic_decision

## 5. U1：代码和测试统一修改

允许修改运行代码和相关单元测试，但只允许修改 TCP 数值、注释和对应测试；不得同时修改 MoveIt 容差、手眼矩阵或轴映射。

必须检查：

    src/real_piper_perception/real_piper_perception/grasp_precheck.py
    src/real_piper_perception/real_piper_perception/grasp_execute.py
    src/real_piper_perception/real_piper_perception/grasp_executor.py
    src/real_piper_perception/test/test_grasp_execute_units.py
    src/real_piper_perception/test/test_grasp_precheck.py

修改要求：

- TCP_OFFSET_FROM_LINK6_Z = 0.1358。
- TCP 到 link6 的正向和反向折算使用同一常量。
- 删除“0.14 是仿真近似”的过时说明。
- 注释明确 0.1358 来源于 URDF joint7/joint8 origin。
- 不把 joint7 的 0.035m 开度或 rpy=1.5708 混入 TCP 主偏移。
- 测试断言不得继续硬编码 0.14。

生成：

    docs/real_grasp_error_investigation/log/tcp_code_update.json

必须包含修改前后常量、修改文件/行号、旧值命中处理结果、文件 hash/mtime、测试命令、退出码和结果。

## 6. U2：冻结输入离线回归

不启动相机、不运行 GraspNet、不移动机械臂。

使用 /tmp/stageE_frozen.csv 和同一候选 [44]，比较：

    A: TCP_OFFSET = 0.1400m
    B: TCP_OFFSET = 0.1358m

输出：

    p_tcp_command
    p_link6_target_A/B
    approach_A/B
    grasp_A/B
    lift_A/B
    A 到 B 的 XYZ 差值和范数

生成：

    docs/real_grasp_error_investigation/log/tcp_offline_regression.json

必须说明差值理论上为 0.0042m 乘 TCP Z 轴，且不得把 link7 铰点直接当作 TCP。

## 7. U3：单元测试和 scan_only 预检

运行：

    cd ./src/real_piper_perception
    python3 -m pytest test/test_grasp_execute_units.py test/test_grasp_precheck.py -q
    python3 -m pytest test/ -q

然后运行 scan_only 预检：

    source ./install/setup.bash
    ros2 launch ./src/real_piper_perception/launch/grasp_execute.launch.py scan_only:=true csv_path:=/tmp/stageE_frozen.csv

要求不移动机械臂、不修改 MoveIt 容差，记录候选 [44] 的新 link6 目标、预检通过数和失败阶段。

生成：

    docs/real_grasp_error_investigation/log/tcp_precheck_regression.json

## 8. U4：用户确认后的低速 dry-run

仅在 U0-U3 完成且用户明确确认后执行：

- 同一冻结 CSV 和候选 [44]。
- scan_only=false。
- dry_run_max_stage=grasp。
- velocity=0.1。
- 不闭合夹爪。
- 每次执行前后回 zero。
- 只执行一次，不自动重试。
- 用户在现场确认急停可用。

记录 command TCP、command link6、规划终点关节、joint_states、joint_states_feedback、FK link6/link7、TCP 前后差异和实测误差。

生成：

    docs/real_grasp_error_investigation/log/tcp_dry_run.json

未授权时写 status=blocked、reason 和 required_minimal_input，不得自行执行。

## 9. 最终日志和报告要求

生成：

    docs/real_grasp_error_investigation/tcp-update-report.md

报告必须包含：

1. URDF 权威 TCP 长度和精确来源行号。
2. 旧值 0.14m、新值 0.1358m。
3. 修改过的文件、函数和测试。
4. 所有测试命令、退出码和结果。
5. 冻结候选 [44] 的 A/B 离线位姿差异。
6. scan_only 预检前后差异。
7. 真机 dry-run 是否执行；未执行必须说明原因。
8. command、link6、link7、FK 和实测误差分开记录。
9. TCP 修改是否使误差沿 TCP Z 方向按理论值变化。
10. 不得把 MoveIt、手眼、感知误差混入 TCP 结论。
11. 最终状态只能是：已通过、效果未验证、未通过或 blocked。
12. 下一步最多列 2 项。

## 10. 本计划明确不做

- 不修改手眼 YAML。
- 不修改 GRASPNET_TO_PIPER_TCP_R。
- 不修改 moveit_adapter.py 容差。
- 不重新训练或修改 GraspNet。
- 不重新运行全量 T0-T6。
- 不因预检通过就判定真实抓取改善。

