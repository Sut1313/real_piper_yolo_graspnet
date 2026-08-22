# Claude Code 受限读取与执行提示词

以下提示词用于后续 Claude Code 会话。目的是让代理只读取真机抓取相关内容，避免扫描无关目录、编译产物和历史实验代码。

```text
你正在排查 real_piper_graspnet 项目真机抓取的末端位置误差。

已知事实：
- Piper 真机抓取流程可以正常执行，没有明显运行错误。
- 末端相对真实目标通常存在约 20–30 mm 的位置误差。
- 当前目标是定位误差来源，不要先入为主地认为是 TCP_OFFSET、手眼标定或某个单一常量。
- 默认只做只读分析、数据记录和安全 dry-run；未经用户明确确认，不得修改代码、标定文件或执行真实抓取。

第一步：只读取以下文档，不要扫描整个仓库：
1. docs/real_grasp_error_investigation/README.md
2. docs/real_grasp_error_investigation/01-real-grasp-scope-and-architecture.md
3. docs/real_grasp_error_investigation/02-2cm-error-investigation-plan.md

允许读取的源码白名单：
src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
src/real_piper_perception/real_piper_perception/handeye_tf_publisher.py
src/real_piper_perception/real_piper_perception/grasp_candidates.py
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_executor.py
src/real_piper_perception/real_piper_perception/grasp_execute.py
src/real_piper_perception/real_piper_perception/grasp_verify.py
src/real_piper_perception/real_piper_perception/moveit_adapter.py
src/real_piper_perception/real_piper_perception/grasp_ik.py
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
src/real_piper_perception/launch/grasp_execute.launch.py
src/real_piper_perception/setup.py
run_graspnet_d435i.sh
camera_data/calibration.yaml
camera_data/handeye_eye_to_hand.yaml
shared_camera_data/color.png
shared_camera_data/depth.png
shared_camera_data/meta.mat
shared_camera_data/perception_grasps.csv
shared_camera_data/last_validation.json
src/piper_ros/src/piper/piper/piper_ctrl_single_node.py
src/piper_ros/src/piper/piper/piper_ctrl_single_node_new.py
src/piper_ros/src/piper/piper/piper_read_slave_joint.py
src/piper_ros/src/piper/launch/start_single_piper.launch.py
src/piper_ros/src/piper_msgs/msg/PosCmd.msg
src/piper_ros/src/piper_msgs/msg/PiperStatusMsg.msg
src/piper_ros/src/piper_msgs/srv/Enable.srv
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_description/urdf/piper_description.xacro
src/piper_ros/src/piper_description/meshes/link6.STL
src/piper_ros/src/piper_description/meshes/link7.STL
src/piper_ros/src/piper_description/meshes/link8.STL
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/kinematics.yaml
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/joint_limits.yaml
scripts/fk_vs_camera_compare.py
scripts/roi_coordinate_check.py
scripts/perception_stability_test.py
scripts/tf_tree_viewer.py
scripts/moveit_go_ready.py
scripts/sdk_fk_vs_urdf_fk.py
scripts/sdk_mapping_validation.py
scripts/sdk_endpose_test.py

明确禁止读取：
- build/、install/、任何 __pycache__/ 和 *.pyc
- src/realsense-ros/
- graspnet-baseline/graspnetAPI_local/
- graspnet-baseline/demo.py、demo_headless.py、demo_save.py、train.py、test.py
- src/piper_ros/src/piper_sim/
- 与 link6/link7/link8 无关的 mesh 文件
- docs/ 下其他 session-summary、dev-log 和历史计划，除非当前测试明确需要核对一条事实

工作规则：
1. 先给出当前已读取文件列表，再开始分析。
2. 使用 rg 精确搜索函数、参数、话题和文件字段；禁止用 find/递归 cat 扫描整个仓库。
3. 每次只读取完成当前测试所需的最小代码片段；不要为了“了解项目”读取无关文件。
4. 不修改任何文件，除非我明确批准“实施修复”。
5. 不修改 handeye YAML、TCP 偏移常量、轴映射矩阵或 URDF 来“试试看”。
6. 不启动真实抓取；只允许静态分析、单元测试、scan_only dry-run 和用户在场的测量步骤。
7. 如果需要运行 ROS 命令，先说明命令、是否会移动机械臂、是否需要用户确认。
8. 所有误差统一用毫米，记录 XYZ 分量、范数、坐标系和数据来源。
9. 将假设分成：感知、深度/内参、手眼、TCP/夹爪几何、MoveIt/FK、Piper 执行/机械误差。
10. 不要把“抓取成功”当成“位姿准确”的证据。

必须按以下顺序执行：
T0 建立固定场景和多帧基准
T1 检查深度尺度、内参、对齐和 ROI
T2 分离手眼平移误差与旋转误差
T3 核对 TCP、link6、link7、link8 和指尖几何
T4 对比 MoveIt 目标、规划终点和 FK
T5 检查 Piper 反馈、零位、回差和执行误差
T6 根据数据选择唯一或主要根因，再提出最小修改方案

每完成一个任务，输出：
- 读取过的文件
- 执行过的命令
- 原始观测
- XYZ 误差和范数
- 支持/排除的假设
- 下一步最小读取范围

每个测试还必须记录：
- 测试编号、时间、frame_id 和候选索引
- 输入文件路径、mtime、大小、sha256
- 图像尺寸、dtype、深度统计、CameraInfo、factor_depth
- handeye 的 R/t 摘要、det(R) 和矩阵校验结果
- 候选的 camera/base/TCP/link6/link7 位姿
- MoveIt group、eef_link、frame_id、规划目标和规划终点
- `/joint_states`、`/joint_states_feedback` 时间戳及关节值
- `/compute_fk` 的 link6/link7 结果
- 实测目标中心、实测指尖/夹持中心、XYZ 误差和范数
- 命令退出码、关键 stdout/stderr、完整异常 traceback

如果某项没有采集，必须写“未采集 + 原因”，不能使用推测值填充。

最终只生成或更新：
docs/real_grasp_error_investigation/measurement-log.md
docs/real_grasp_error_investigation/root-cause-report.md

最终报告必须明确：
- 根因或尚未确定的原因
- 证据和误差方向
- 最小修复文件
- 需要新增或更新的回归测试
- 修复前后如何用同一场景验证
- 平均、标准差、最大 XYZ 误差和误差范数
- 误差属于哪个坐标系，以及从哪些原始数据计算
- 已排除的假设和排除证据
- 剩余假设的置信度排序
- 如果不能确定，下一次只需要采集的最小数据集合
```
