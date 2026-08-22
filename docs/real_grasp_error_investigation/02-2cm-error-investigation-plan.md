# 真机末端 2–3 cm 目标误差排查计划

## 1. 目标与限制

目标：确定末端误差是固定平移、姿态引起的空间误差、感知目标误差、手眼外参误差、TCP 几何误差，还是机器人执行误差。

限制：

- 不以“流程成功”作为几何正确的证据。
- 不先修改 `TCP_OFFSET_FROM_LINK6_Z`、手眼 YAML 或轴映射矩阵。
- 真机动作必须单次、低速、有人在场，默认先使用 `scan_only:=true`。
- 每个测试必须记录输入帧、候选索引、目标坐标、规划目标、实际 FK、实测目标位置。

## 2. 统一误差定义

对每次测试记录：

```text
p_target_camera       = 感知目标/物体中心，相机系
p_target_base         = 手眼变换后的目标，base_link 系
p_command_tcp         = grasp_executor 生成的 TCP 目标，base_link 系
p_fk_link7            = 执行后 /compute_fk 的 link7 位置，base_link 系
p_measured_object     = 标定物实际中心，base_link 系或测量坐标系
```

至少计算：

```text
E_perception = p_target_camera 与人工目标中心反投影的差
E_transform  = p_target_base - 独立计算的目标 base 坐标
E_execution  = p_fk_link7 - p_command_tcp 或工具中心的几何换算
E_total      = 实际末端/指尖 - 真实目标中心
```

统一使用毫米；每个向量同时记录 XYZ 分量和范数，不能只记录“约 2 cm”。

## 3. 测试任务

### T0：建立可重复基准

读取范围：`grasp_execute.launch.py`、`grasp_executor.py`、`shared_camera_data/*`、当前手眼 YAML。

步骤：

1. 固定相机、工作台、目标物和机械臂初始位姿。
2. 连续采集至少 5 帧，不执行抓取。
3. 保存每帧 `color.png`、`depth.png`、`meta.mat`、`perception_grasps.csv` 和 `last_validation.json` 的副本或校验值。
4. 记录同一候选在不同帧的 `camera_x/y/z`、旋转、score 和 width。

判定：

- 如果感知坐标自身变化超过 5 mm，优先进入 T1。
- 如果感知坐标稳定但实际末端误差稳定，进入 T2/T3。

### T1：感知目标与深度尺度测试

读取范围：`d435i_graspnet_bridge.py`、`graspnet-baseline/inference_from_camera.py`、`camera_data/calibration.yaml`、`meta.mat`、`perception_grasps.csv`。

检查：

1. 确认 `depth.png` 单位为毫米，`factor_depth` 语义没有被重复乘除。
2. 用 CameraInfo 内参独立反投影目标像素/深度。
3. 比较点云物体中心与人工量取的目标中心。
4. 检查深度是否使用 aligned-to-color 图像。
5. 检查 ROI 是否截断目标或引入背景点。

判定：

- 误差随距离按比例增大：怀疑深度尺度或内参。
- 误差在相机坐标中已存在：定位到感知层。
- 相机坐标准确但 base 坐标错误：进入 T2。

### T2：手眼外参平移/旋转分离测试

读取范围：`handeye_eye_to_hand.yaml`、`grasp_transform.py`、`handeye_tf_publisher.py`、`scripts/tf_tree_viewer.py`。

步骤：

1. 用同一个 `p_camera`，分别计算当前代码的 `p_base` 和独立矩阵乘法结果。
2. 检查 `R_cam2base` 的正交性、行列式和单位。
3. 使用至少 3 个相机内已知位置的标定点，比较预测 base 坐标与实测 base 坐标。
4. 检查 TF 中是否存在重复的 `camera_link` 父节点或错误的相机 optical frame。
5. 分别评估平移误差和旋转误差：
   - 旋转误差会随目标离相机距离放大。
   - 固定 XYZ 偏差更像平移外参或工具偏移。

判定：

- 三个点的误差近似同一向量：手眼平移或 TCP 固定偏移。
- 误差随目标位置/距离变化：手眼旋转或坐标系方向错误。
- 静态 TF 正确但模块计算错误：定位到变换实现。

### T3：TCP/夹爪几何和 link6/link7 语义测试

读取范围：`grasp_precheck.py`、`grasp_execute.py`、`grasp_transform.py`、`piper_description.urdf`、`piper.srdf`、`link6.STL`、`link7.STL`、`link8.STL`。

步骤：

1. 明确代码中的目标究竟代表：GraspNet 抓取中心、TCP、link6、link7，还是指尖夹持中心。
2. 从 URDF 计算 `link6 → gripper_base → link7/link8` 的真实几何链。
3. 对比 `grasp_precheck.py` 中 TCP→link6 偏移方向、符号和数值。
4. 让机械臂停在 `dry_run_max_stage:=grasp` 对应位置，现场测量 link6 基准面到指尖夹持中心的距离。
5. 对左右手指分别测量，排除镜像/中心定义错误。

判定：

- 固定沿工具轴方向的 20–30 mm：优先怀疑 TCP 定义或偏移。
- XY 误差与姿态变化相关：怀疑工具轴映射或姿态目标。
- 代码 TCP 与 FK 的 link7 位置不一致：优先修正几何折算，不改手眼。

### T4：MoveIt 目标与实际 FK 测试

读取范围：`grasp_precheck.py`、`moveit_adapter.py`、`grasp_execute.py`、`grasp_executor.py`、`piper.srdf`、`kinematics.yaml`。

步骤：

1. 只做 `scan_only:=true`，保存预检产生的 `stage_poses`。
2. 比较 MoveIt 规划目标使用的末端 link 与 FK 查询的 link。
3. 对同一关节解执行离线/MoveIt FK，确认 pose target 与 FK 一致。
4. 检查 `PositionConstraint.header.frame_id`、末端 link 和姿态四元数顺序。
5. 真机低速执行一次，分别记录：
   - 规划目标
   - MoveIt 规划终点
   - `/joint_states`
   - `/joint_states_feedback`
   - `/compute_fk` 的 link6/link7

判定：

- MoveIt FK 已经偏离目标：规划/坐标接口问题。
- MoveIt FK 正确但实物偏离：URDF、零位、机械回差或工具几何问题。
- `/joint_states` 与实际反馈不一致：驱动或控制器状态映射问题。

### T5：机械臂执行和机械误差测试

读取范围：Piper 驱动三个 Python 文件、`start_single_piper.launch.py`、相关 ROS 控制配置、`scripts/sdk_fk_vs_urdf_fk.py`。

步骤：

1. 在不接触目标的情况下，执行 3 个已知关节目标。
2. 每个目标往返运动 3 次，记录正向和反向到位差。
3. 比较 `/joint_states`、`/joint_states_feedback`、SDK 反馈和 MoveIt FK。
4. 检查是否存在关节零位偏置、单位换算、关节方向或夹爪倍数不一致。
5. 检查机械臂是否因负载、柔顺或回差在抓取方向产生固定偏差。

判定：

- 误差随运动方向改变：回差/机械柔顺。
- 误差只在某个关节姿态出现：关节标定或 URDF 几何。
- 所有关节反馈正确但指尖测量错：工具几何/TCP 定义。

### T6：根因归类与最小修复方案

只有完成 T0–T5 后才能进入本任务。

输出一张根因表：

| 假设 | 证据 | 误差方向/量级 | 置信度 | 建议修改 |
|---|---|---:|---|---|
| 深度尺度 |  |  |  |  |
| ROI/目标中心 |  |  |  |  |
| 手眼平移 |  |  |  |  |
| 手眼旋转 |  |  |  |  |
| TCP 偏移 |  |  |  |  |
| GraspNet→TCP 轴映射 |  |  |  |  |
| MoveIt/FK 末端定义 |  |  |  |  |
| Piper 执行/机械误差 |  |  |  |  |

修复原则：一次只改变一个误差源；每次修改必须先增加/更新回归测试，并用同一基准场景复测。

## 4. 必须交付的结果

```text
docs/real_grasp_error_investigation/measurement-log.md
docs/real_grasp_error_investigation/root-cause-report.md
```

其中应包含：

- 测试时间、硬件状态、相机位置和目标位置。
- 每帧输入和候选索引。
- command / FK / 实测位置。
- XYZ 误差和范数，单位 mm。
- 失败测试及原因。
- 最终根因、最小修复文件和回归测试命令。

## 5. 严格执行协议：一步一步排查

### 阶段 A：静态代码和配置核对

1. 只读取架构文档中列出的真机白名单文件。
2. 用 `rg` 定位以下符号的定义和调用：
   `factor_depth`、`R_cam2base`、`t_cam2base`、`TCP_OFFSET`、
   `GRASPNET_TO_PIPER_TCP_R`、`END_EFFECTOR_LINK`、`link7`、
   `/compute_fk`、`/joint_states_feedback`。
3. 输出每个符号的“来源 → 变换 → 消费者”关系。
4. 检查单位、坐标系、帧名、四元数顺序和末端 link 是否一致。

停止条件：发现单位或坐标系定义矛盾时，先记录矛盾，不得直接改代码；进入阶段 B 验证矛盾是否能解释 20–30 mm 误差。

### 阶段 B：共享数据离线复算

1. 固定一帧 `color.png/depth.png/meta.mat/perception_grasps.csv`，不要覆盖原文件。
2. 记录文件 `mtime`、大小和 sha256。
3. 用 `meta.mat` 的内参和深度因子独立反投影至少一个目标点。
4. 用 `grasp_transform.py` 的手眼矩阵独立计算 `p_base`。
5. 用独立计算结果与程序输出逐项比较。
6. 检查深度值是否发生重复缩放、ROI 是否改变物体中心、CSV 是否存在列错位。

必须输出：

```text
frame_id
depth dtype / 单位 / factor_depth
intrinsic_matrix
候选 camera 坐标
独立反投影 camera 坐标
程序 base 坐标
独立 base 坐标
每一步 XYZ 差值和范数（mm）
```

停止条件：如果相机坐标误差已经超过 10 mm，暂不执行真机运动，先完成感知/深度问题归因。

### 阶段 C：ROS 状态和 scan_only 预检

1. 确认当前没有遗留 `pause_inference` 或旧 CSV。
2. 启动相机和 bridge，但不启动抓取执行。
3. 确认 RGB、aligned depth、CameraInfo 均有数据。
4. 启动 `grasp_execute.launch.py scan_only:=true`。
5. 记录候选数量、排序索引、门控拒绝数、每阶段预检结果。
6. 保存 `precheck_results.json`、`grasp_result.json`（如有）和相关终端日志。

停止条件：如果候选在预检前后坐标不一致、CSV 不是同一帧或 MoveIt 服务不可用，先修复数据链或记录阻塞，不进行真机执行。

### 阶段 D：无目标低速到位测试

1. 移除目标物，确保夹爪周围无障碍物。
2. 选择 `zero`、一个中间姿态和一个抓取姿态附近的关节目标。
3. 每个目标执行“正向到位 → 记录 → 回 zero → 反向到位 → 记录”至少 3 次。
4. 同时记录 `/joint_states`、`/joint_states_feedback`、MoveIt 规划终点和 `/compute_fk`。
5. 比较正向/反向误差，判断回差、零位或控制器映射问题。

停止条件：出现报警、关节反馈跳变、实际位置与 FK 差异超过 10 mm，立即停止运动测试。

### 阶段 E：固定目标单次验证

1. 回到与阶段 B 相同的相机、目标和机械臂初始状态。
2. 使用同一 `frame_id` 和同一候选索引。
3. 先执行 `scan_only:=true`，确认目标位姿和预检结果。
4. 用户明确确认后，才允许 `scan_only:=false` 单次执行。
5. 在 approach、grasp、闭合后分别记录：
   - command TCP
   - FK link6/link7
   - `/joint_states_feedback`
   - 视觉中的目标中心
   - 实测指尖/夹持中心
6. 抓取完成后不要立即覆盖共享目录数据；先复制并计算 hash。

停止条件：任何阶段偏差超过 50 mm、方向异常、碰撞风险或夹爪反馈无数据，立即停止并报告，不自动重试。

### 阶段 F：根因归类和最小修复

1. 将误差分解为感知、坐标变换、工具几何、规划/FK、机械执行五类。
2. 用至少 3 个位置或 3 次重复测量确认“固定偏差”还是“随位置/姿态变化”。
3. 只选择有数据支持的主要假设，不把可能性当结论。
4. 提出一个最小修改方案，一次只修改一个误差源。
5. 为修改增加或更新单元测试，再用相同 `frame_id`/目标场景复测。

## 6. Claude Code 最终必须提供的日志

最终报告必须同时提供原始观测和归因，不能只写“测试通过”或“疑似 TCP 问题”。至少包含以下内容。

### 6.1 环境与输入一致性

```text
测试时间、时区、主机/容器
ROS_DISTRO、Python 版本、代码版本或“非 git 仓库”说明
相机型号/序列号（可获取时）
Piper CAN 端口和启动参数
测试编号和 frame_id
每个输入文件的路径、mtime、大小、sha256
color/depth 的宽高、dtype、深度非零点统计
intrinsic_matrix、factor_depth
handeye YAML 的 sha256、R/t 摘要、det(R) 和正交误差
```

### 6.2 候选和坐标变换

对选中候选和至少两个被拒绝候选报告：

```text
CSV 原始行号、排序后索引、name、score、width
xyz_camera、center_camera（mm）
rotation matrix
score gate / self-consistency 结果
p_base、R_base_graspnet、R_base_tcp
approach/grasp/lift 的位置和四元数
TCP→link6 折算前后位置
```

### 6.3 MoveIt、反馈和 FK

每次 dry-run/真机动作报告：

```text
arm/gripper group
eef_link 和 frame_id
planning_time、velocity_scale
plan/execute 返回值
规划终点关节值
/joint_states 时间戳和 joint1..joint7/gripper
/joint_states_feedback 时间戳和 joint1..joint7/gripper
/compute_fk 的 link6/link7 position + quaternion
服务超时、空反馈、异常 traceback
```

### 6.4 实测误差表

每个测试点至少 3 次测量，使用毫米：

| test_id | frame_id | candidate | command TCP XYZ | FK link7 XYZ | 实测指尖/夹持中心 XYZ | 真实目标中心 XYZ | error XYZ | norm |
|---|---|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |  |  |

报告平均值、标准差、最大值，并说明误差方向是否固定、是否随相机深度/目标位置/运动方向变化。

### 6.5 命令和异常

每条命令必须报告：命令文本、开始/结束时间、退出码、关键 stdout/stderr、是否移动机械臂、是否写文件。密码、令牌和无关环境变量必须脱敏。

### 6.6 最终归因格式

最终 `root-cause-report.md` 必须按以下顺序：

1. 一句话结论：已确定、最可能或尚未确定。
2. 平均/标准差/最大 XYZ 误差和范数。
3. 误差所属坐标系。
4. 证据链：具体日志、文件和函数/行号。
5. 已排除假设及排除证据。
6. 剩余假设的置信度排序。
7. 最小修复文件、变量和预期效果。
8. 修复前必须新增/更新的回归测试。
9. 修复后使用同一场景的复测命令和通过标准。
10. 如果无法确定，下一次只需采集的最小数据集合。

禁止使用“看起来是”“应该是”“规划正常所以没问题”等无数据结论。
