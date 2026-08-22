# Real Piper + D435i + GraspNet 项目架构、数据流与开发者指南

> 文档基线：2026-08-21 当前工作区  
> 项目根目录：`.`  
> 适用对象：接手维护、定位故障、修改感知/规划/执行链路、设计新实验的开发者  
> 重要说明：本文以当前源码和当前配置为准；历史文档只作为实验过程证据。凡当前代码与旧文档冲突，本文会明确指出。

> **回退说明（2026-08-21）**：工作区代码已按用户要求回退到当日 18:23
> 大规模可靠性改造之前的行为。本文中关于 score 降序 Top-K、29 列时序字段、
> K+D 去畸变、支持平面分割、TCP 局部轴插入/退离、收紧 MoveIt 容差和输入
> 快照的段落仅保留为历史分析，不代表回退后的运行代码；当前代码事实以
> `rollback/pre-large-change` 分支及其回退提交为准。

---

## 1. 先读结论：这个项目现在是什么状态

本项目把 Intel RealSense D435i 的 RGB-D 图像，经 ROS 2 桥接到共享目录，在 NVIDIA GPU Docker 中运行 GraspNet，生成 6D 抓取候选；宿主机上的 ROS 2 执行器再将候选从相机坐标系变换到 Piper 基座坐标系，调用 MoveIt 2 做可达性预检和轨迹执行，并通过 Piper SDK 直接控制夹爪。

这不是单一框架的标准流水线，而是三个系统拼接形成的混合架构：

1. ROS 2 Humble 负责相机、TF、服务、MoveIt 和 Piper 驱动；
2. 文件系统负责宿主机与 Docker 间传递帧、候选、预检结果和最终结果；
3. Piper 的六轴运动经“MoveIt FakeSystem 命令状态 → `/joint_states` → Piper ROS 驱动 → CAN”转发，夹爪则由执行器绕过 MoveIt、直接用 `piper_sdk` 写 CAN。

截至本文基线，感知、候选预检、机械臂 approach/grasp/回零动作均实际运行过，但系统还不能被视为可靠的自动抓取产品。2026-08-21 的最新一次真机实验中：

- 机械臂轨迹执行成功；
- 新 TCP 轴映射的接近方向被验证为正确；
- 夹爪最终抓空；
- 旧实现曾把闭合过程中的瞬时 `24.0 mm` 反馈当成夹住；当前实现已改为稳定窗口和保持复核，后续同类实验正确判为 `grasp_empty`；
- 2026-08-21 改造后，Cartesian 响应字段已修复；正式序列为 OMPL 到 approach、100% Cartesian 插入 grasp、抓后沿原路径 Cartesian 退离；
- 候选已改为 score 降序、廉价门控、最多 Top-K 规划并在首个可规划候选处早停；
- MoveIt 没有有效的深度 OctoMap，因此没有真实桌面/障碍物环境碰撞模型。

在修复第 15 节列出的 P0 问题之前，建议只运行 `scan_only:=true`，不要启动无人看守的 `scan_only:=false`，更不要自动循环调用抓取服务。

---

## 2. 阅读范围、版本边界与证据优先级

### 2.1 已分析的内容

本次盘点覆盖：

- 根目录脚本和 README；
- 自研 ROS 2 包 `src/real_piper_perception` 的全部 Python、Launch、测试和配置；
- `graspnet-baseline` 的推理入口、模型、PointNet++、碰撞检测、Docker、训练/测试入口；
- `src/piper_ros` 的驱动、消息、Launch、URDF、SRDF、MoveIt 和 ros2_control 配置；
- `src/realsense-ros` 的 ROS 包结构、相机节点、过滤器、TF、消息和 Launch 路由；
- 根目录 `scripts/` 和 `test/`；
- `camera_data/`、`shared_camera_data/` 当前运行产物；
- `docs/` 中 2026-08-02 至 2026-08-21 的会话、设计、调查和实验报告；
- `log/` 下 551 个文件、约 2.887 GiB 的构建和运行日志，按来源、时间和错误模式进行统计与抽样核验；
- `build/`、`install/` 等生成目录的结构和最后构建结果。

模型权重、PNG、MAT、STL/DAE 等二进制文件按文件类型、元数据、用途和调用关系检查，不应把“阅读所有文件”理解为逐字反编译第三方二进制或模型参数。

### 2.2 仓库并非单一 Git 仓库

项目根目录本身不是 Git 仓库。内含三个独立 Git 工作区：

| 子仓库 | 当前基线 | 工作区状态 |
|---|---|---|
| `src/piper_ros` | `humble`，提交 `017ffefa...`，2025-12-16 | 有 8 个本地修改文件；这些修改是当前真机链路的一部分，不能随意覆盖 |
| `src/realsense-ros` | `ros2-master`，提交 `60c850...`，2026-07-20，包版本 4.58.3 | 干净 |
| `graspnet-baseline/graspnetAPI_local` | `master`，提交 `eb57dd...`，2026-06-29 | 干净 |

自研根目录、`graspnet-baseline` 主体、测试数据和文档没有统一的根级提交历史。因此改动前应另行备份或先建立根级版本管理，不能假定 `git diff` 能保护全部文件。

### 2.3 事实优先级

遇到冲突时按下列顺序判断：

1. 当前源码和当前 Launch 参数；
2. 当前 `camera_data/` 配置与最近构建产物；
3. 2026-08-21 最新实验日志；
4. 旧会话总结和旧设计文档；
5. 注释、docstring 和 README。

原因是本项目迭代很快，若干注释已经落后。例如当前预检实际只检查 `approach + grasp` 两段，部分注释仍写“三段”；当前推理桥接使用 `calibration.yaml` 的 K，8 月 18 日报告却记录当时“只用 runtime CameraInfo K”。

---

## 3. 顶层目录及其生命周期

| 路径 | 类型 | 作用 | 是否应直接修改 |
|---|---|---|---|
| `src/real_piper_perception/` | 自研源代码 | 相机桥、手眼 TF、候选解析、变换、预检、执行、结果发布 | 是，主要开发区 |
| `graspnet-baseline/` | 上游模型 + 本地改造 | GPU 推理、碰撞过滤、共享目录协议、结果渲染、训练测试 | 是，但要区分上游和项目补丁 |
| `src/piper_ros/` | 第三方驱动 + 本地补丁 | Piper CAN 驱动、URDF、MoveIt、仿真 | 谨慎；当前有未提交本地修改 |
| `src/realsense-ros/` | 第三方上游 | D435i ROS 2 驱动、消息、描述 | 通常不改；优先在自研 Launch 配置 |
| `camera_data/` | 权威运行配置 | 相机内参与 eye-to-hand 外参 | 高风险配置；修改必须保留来源和校验记录 |
| `shared_camera_data/` | 运行时 IPC | 最新帧、ROI、候选、预检、结果图和执行结果 | 会被运行进程覆盖，不是稳定档案 |
| `scripts/` | 运维、诊断、实验工具 | 离线验证、MoveIt/SDK 调试、ROI/TF/结果查看 | 按安全等级使用 |
| `test/` | 根级测试与 ROI GUI | 推理辅助模块、TF 工具测试、ROI 编辑器 | 是 |
| `docs/` | 历史与总文档 | 设计、会话、真机误差调查、冻结证据 | 原始实验材料尽量只追加不改写 |
| `log/` | colcon 与运行日志 | 约 2.9 GiB 的历史构建/运行证据 | 不作为源码；可归档压缩，删除前先备份 |
| `build/` | colcon 生成 | 构建中间产物 | 不改；可重建 |
| `install/` | colcon 生成 | ROS 安装空间、入口脚本、资源索引 | 不改；源码改后重建 |
| `.pytest_cache/`、`__pycache__/` | 缓存 | 测试/Python 缓存 | 不改，可清理 |
| `.superpowers/sdd/` | 历史开发账本 | 2026-08-06 任务 brief、实现报告、审查和修复记录 | 只作历史证据；很多结论已被后续代码替代 |
| `.claude/` | 工具目录 | 当前为空 | 无运行作用 |

根级启动入口：

| 文件 | 作用 |
|---|---|
| `build_ws.sh` | source ROS Humble，以标准系统 PATH 执行 `colcon build --symlink-install --packages-up-to realsense2_camera real_piper_perception` |
| `run_d435i_bridge.sh` | source ROS 和本地 install，启动 D435i、共享目录桥和 handeye TF |
| `run_graspnet_d435i.sh` | 启动 GPU Docker 推理，挂载最新宿主机补丁和共享目录 |
| `view_results.sh` | 主机 OpenCV 窗口查看 `result.png` |
| `README.md` | 早期部署摘要；对完整真机执行链路和当前缺陷描述不足，以本文为准 |

---

## 4. 总体架构层次

```text
┌──────────────────────────────── 硬件层 ────────────────────────────────┐
│ Intel D435i RGB-D        Piper 六轴机械臂/小夹爪        USB-CAN/can0  │
└──────────────┬──────────────────────┬───────────────────────────────┘
               │ USB/librealsense     │ CAN 1 Mbps
┌──────────────▼──────────────────────▼───────────────────────────────┐
│ 设备与驱动层                                                         │
│ realsense2_camera_node             piper_ctrl_single_node          │
│ 图像/CameraInfo/相机TF              真反馈/状态/使能/命令转发         │
└──────────────┬──────────────────────▲───────────────────────────────┘
               │ ROS topics           │ /joint_states 被当作关节命令
┌──────────────▼──────────────┐  ┌────┴──────────────────────────────┐
│ 感知桥与坐标层               │  │ MoveIt 规划与执行适配层           │
│ d435i_graspnet_bridge        │  │ move_group + FakeSystem           │
│ handeye_tf_publisher         │  │ MoveGroupInterface                │
└──────────────┬──────────────┘  └────▲──────────────────────────────┘
               │ 原子文件             │ 计划/轨迹
┌──────────────▼───────────────────────┴──────────────────────────────┐
│ shared_camera_data：宿主机 ↔ Docker IPC                             │
│ color/depth/meta/.ready → candidates/precheck/result/ROI/pause      │
└──────────────┬───────────────────────▲──────────────────────────────┘
               │ bind mount            │ CSV/JSON/PNG
┌──────────────▼───────────────────────┴──────────────────────────────┐
│ Docker GPU 感知层                                                    │
│ PointNet++ → GraspNet → collision filter → NMS → CSV/可视化          │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ perception_grasps.csv
┌──────────────────────────────▼─────────────────────────────────────┐
│ 任务编排与安全门层                                                   │
│ grasp_executor：候选门控 → 坐标变换 → 两段预检 → 候选选择 → 结果发布 │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ execute_pick
┌──────────────────────────────▼─────────────────────────────────────┐
│ 动作层                                                              │
│ 六轴：MoveIt/FakeSystem → /joint_states → Piper 驱动 → CAN           │
│ 夹爪：grasp_execute.py → piper_sdk.C_PiperInterface('can0') → CAN    │
└────────────────────────────────────────────────────────────────────┘
```

### 4.1 分层责任

1. **硬件层**：提供彩色/深度帧、关节和夹爪真实反馈。
2. **驱动层**：将硬件数据映射为 ROS 话题，并把 ROS 关节命令转换为 SDK/CAN 指令。
3. **桥接层**：把不同频率的最新 RGB、深度和内参快照写为 Docker 可读文件。
4. **神经网络层**：将 RGB-D 转点云，推理候选并做无模型碰撞过滤。
5. **变换与预检层**：把 GraspNet 候选映射为基座 TCP 位姿，做宽度、质量、方向和 MoveIt 可达性门控。
6. **执行层**：发六轴轨迹、夹爪开闭命令，读取反馈，生成业务结果。
7. **观测层**：日志、JSON、CSV、可视化、FK 和诊断脚本。

边界上最需要注意的是：文件系统 IPC 没有消息队列语义；MoveIt 的“当前关节状态”不是独立真实反馈；夹爪与六轴有两个不同控制通道。

---

## 5. 运行时进程、ROS 图与接口路由

### 5.1 主要进程

| 进程/节点 | 启动来源 | 输入 | 输出 |
|---|---|---|---|
| `realsense2_camera_node`，节点 `/camera/camera` | `d435i_graspnet_pipeline.launch.py` | D435i USB | 彩色、深度、CameraInfo、相机 TF |
| `d435i_graspnet_bridge` | 同上 | 3 个相机话题 | `shared_camera_data/color.png`、`depth.png`、`meta.mat`、`.ready` |
| `handeye_tf_publisher` | 同上 | handeye YAML | `base_link -> camera_link` 静态 TF，另每 5 秒重发 |
| `inference_from_camera.py` | `run_graspnet_d435i.sh` Docker | 共享目录帧和 ROI | 候选 CSV、诊断 JSON、结果图 |
| `move_group`、ros2_control、控制器 | `piper_with_gripper_moveit demo.launch.py` | `/joint_states`、规划请求 | `/move_action`、`/execute_trajectory`、`/compute_cartesian_path`、`/compute_fk` |
| `piper_ctrl_single_node` | `start_single_piper.launch.py` | `/joint_states` 命令、CAN | 真实反馈、状态、末端姿态、CAN 运动 |
| `grasp_executor` | `grasp_execute.launch.py` | 候选 CSV、MoveIt、真实夹爪反馈 | `/grasp_execute`、`/grasp_result`、JSON、六轴和夹爪命令 |

### 5.2 相机话题

自研链路只直接消费以下话题：

| 话题 | 类型 | 消费者 |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image`，BGR8 转换 | `d435i_graspnet_bridge` |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`，通常 uint16 mm | `d435i_graspnet_bridge` |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `d435i_graspnet_bridge` |

Launch 中启用 640×480@30 Hz 的 color/depth 和 `align_depth.enable=true`；关闭红外、IMU 和点云。桥接器以默认 5 Hz 采样缓存，但比较 color/depth ROS 时间戳，超过 `max_color_depth_skew_ms=12` 的帧对直接拒绝，并用 `frame_manifest.json` 记录最终配对时间戳。当前仍是“最新缓存 + 严格时间门”而非 `message_filters` 队列配对。

### 5.3 Piper ROS 接口

`piper_ctrl_single_node.py` 的主要路由：

| 名称 | 方向 | 类型/语义 |
|---|---|---|
| `joint_ctrl_single`，Launch 重映射到 `/joint_states` | 订阅 | 被当作六轴和可选夹爪命令，而不是反馈 |
| `pos_cmd` | 订阅 | `piper_msgs/PosCmd`，末端位姿命令 |
| `enable_flag` | 订阅 | `std_msgs/Bool` |
| `enable_srv` | 服务 | `piper_msgs/srv/Enable` |
| `joint_states_single` | 发布 | SDK 当前关节反馈 |
| `joint_states_feedback` | 发布 | 执行器读取的真实关节/夹爪反馈 |
| `joint_ctrl` | 发布 | SDK 命令回显 |
| `arm_status` | 发布 | `PiperStatusMsg`，控制模式、错误码、各轴状态 |
| `end_pose`、`end_pose_stamped` | 发布 | 固件末端姿态 |

驱动发布循环约 200 Hz。六轴反馈换算使用 `(SDK/1000) * 0.017444`，近似度到弧度；命令换算使用约 `57324.84`。夹爪反馈为 SDK 数值除以 `1e6`，代表米制总开口。

### 5.4 执行器接口

| 接口 | 类型 | 行为 |
|---|---|---|
| `/grasp_execute` | `std_srvs/srv/Trigger` | 触发一次候选加载、预检，或真机抓取 |
| `/grasp_result` | `std_msgs/String` | JSON 字符串；只在部分完成/失败路径发布 |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | 执行后诊断 link7 位置/方向 |
| `/joint_states_feedback` | `sensor_msgs/JointState` | 仅执行动作类用于读取真实夹爪开口 |
| `/move_action` | `moveit_msgs/action/MoveGroup` | 计划请求使用 `plan_only=true` |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | 执行计划轨迹 |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | approach→grasp 插入及抓后退离；要求 fraction=1 |
| `/joint_states` | `sensor_msgs/JointState` | MoveIt 适配器的当前状态缓存；本项目中是 FakeSystem/命令状态 |

服务名和发布名在节点中使用相对名称；当前根命名空间下表现为 `/grasp_execute`、`/grasp_result`。若以后给节点加 namespace，接口会随 namespace 改变。

---

## 6. 数据流：从相机帧到抓取结果

### 6.1 RGB-D 采集与共享目录发布

1. RealSense 节点发布 30 Hz 彩色、对齐深度和 CameraInfo。
2. `d435i_graspnet_bridge.py` 的三个独立回调各自覆盖 `latest_*` 并保存 ROS 时间戳。
3. 5 Hz 定时器确认缓存非空、RGB-D 时间差 ≤12 ms 且不是已发布过的帧对。
4. 彩色图保持 BGR 写 `color.png`。
5. 深度若为浮点或 `depth_is_meters=true`，乘 `factor_depth`；否则按已是毫米处理；清理 NaN/Inf 后截断为 uint16 写 `depth.png`。
6. Launch 强制 `use_calibration_intrinsics=true`；`meta.mat` 同时写入 `camera_data/calibration.yaml` 的 K、D 和 `factor_depth=1000`。
7. `camera_info.txt` 同时写 published K/D 和 runtime K/D，便于审计。
8. 写 `frame_manifest.json`（color/depth/info 时间戳、skew、帧序号）；所有单文件使用临时文件 + `os.replace`，最后原子写 `.ready`。

这里的原子性仍是“单文件原子”，不是目录级事务；`frame_manifest.json` 提供审计 generation，但消费者尚未在读取前后校验 generation。因此极端情况下仍可能跨代读取，后续应改成帧目录或读取前后双检 manifest。

### 6.2 深度反投影与 ROI

Docker 中读取：

- OpenCV 将 `color.png` 从 BGR 转 RGB；
- 读取 uint16 深度；
- `scipy.io.loadmat` 取权威 K、D 和 `factor_depth`；
- `cv2.undistortPoints(pixel,K,D)` 先得到归一化光线 `(x_n,y_n)`，再反投影：

```text
z = depth / factor_depth
x = x_n * z
y = y_n * z
```

有效工作区由深度范围与 ROI 的布尔掩码相交得到。ROI 是归一化 `[x0,y0,x1,y1]`，可来自启动环境变量，也可由 `shared_camera_data/roi.json` 每帧动态读取。实现只屏蔽点，不裁剪图像，所以不需要修改主点，正常情况下不会引入坐标平移。

workspace 模式对 ROI 点云做支撑平面 RANSAC，去平面后只保留最大的二维连通前景；失败时不再退化成整块 ROI。默认采样 20,000 点，使用确定性均匀索引，不再逐帧随机抽样。全场景点云保留给碰撞检测；分割目标点用于网络和候选关联。

### 6.3 GraspNet 推理

当前网络的数据通路：

1. PointNet++ Set Abstraction：20,000 点逐层到 2048、1024、512、256；半径依次约 0.04、0.10、0.20、0.30 m；
2. Feature Propagation 将特征回传到 1024 个 seed；
3. `ApproachNet` 预测 objectness 和每个 seed 的 300 个视角分数；
4. 选中视角后，`CloudCrop` 以 0.05 m 半径、64 点和 4 个深度模板聚合局部特征；
5. `OperationNet` 预测 12 个平面角 × 4 个深度的抓取分数和宽度；
6. `ToleranceNet` 预测容差；
7. `pred_decode()` 根据 objectness、角度、深度、分数和容差生成标准 GraspGroup；
8. `ModelFreeCollisionDetector` 在体素化场景点云上检查手指、掌底和接近区占用；
9. NMS 后按网络 score 降序组织推理输出；
10. `select_object_grasp_targets()` 关联 workspace target，写 CSV/JSON。

`num_view=300` 与当前权重结构绑定，不能在不重新训练/验证权重的情况下随意改。权重位于 `graspnet-baseline/logs/log_rs/checkpoint.tar`，约 12.5 MB。

### 6.4 候选从 Docker 到宿主机

主输出 `perception_grasps.csv` 的前 24 列为基础字段：

```text
name,
camera_x,camera_y,camera_z,
count,score,width,height,depth,
r00,r01,r02,r10,r11,r12,r20,r21,r22,
center_camera_x,center_camera_y,center_camera_z,
link6_grasp_z,tcp_forward_z,source_dist_xy
```

字段分组：

- `camera_*`：GraspNet 抓取平移，相机坐标系，米；
- `r00..r22`：GraspNet 旋转矩阵，行优先；
- `score/width/height/depth`：网络抓取属性；
- `center_camera_*`：工作区目标点云均值；
- `count`：目标点数；
- 24 个基础列之后可追加时序诊断：`stability_count`、`position_jitter_m`、
  `object_radius_xy`、`scene_stability_count`、`scene_center_jitter_m`。

`grasp_candidates.py` 要求至少 24 列，并向后兼容上述可选列；当前实时 CSV 为 29 列。`perception_targets.csv` 只保存物体中心，是旧接口兼容产物，当前执行器默认不用。

### 6.5 候选门控和选择

执行器一次触发的当前流程：

1. 解析最新 CSV；
2. 若非 scan-only，先回六轴 zero；
3. 将候选按 `score` **降序**稳定排序；
4. 最多遍历 150 个；
5. 过滤 `score < grasp_score_min`，默认阈值 0；
6. 检查 `width + 0.005 m <= 0.070 m`；
7. 检查目标场景连续稳定帧数、中心 RMS 抖动和候选时序诊断；
8. self-consistency 使用 `max(30mm, 目标95%横向半径+10mm)`，并封顶 80mm；
9. 为候选生成 approach、grasp、lift 位姿，并计算 `approach_down`（要求 ≥0.70）；
10. 只对通过廉价门控的前 `precheck_top_k=20` 个做 approach/grasp MoveIt 预检；
11. 第一个规划全过者立即早停，因此语义为“最高分且可规划”；
12. 写 `precheck_results.json` 和本次 `executions/<run>/selection.json`；scan-only 到此结束。

当前策略以网络 score 为主序，以安全门和可规划性为硬约束。它不承诺最高分候选一定可用，但承诺在已检查窗口内不会用更低分可规划候选替换更高分可规划候选。每次触发先冻结 RGB-D、K/D、CSV、handeye 和验证日志，便于离线 A/B。

### 6.6 暂停推理握手

进入执行流时，`grasp_executor` 写 `pause_inference`。Docker 看到后跳过推理并删除 `.ready`，从而冻结 CSV 和显示页面，避免机械臂运动时感知候选不断刷新。流程结束在 `finally` 中删除暂停标志。

风险：若进程被强杀，暂停标志可能残留；根级 `run_graspnet_d435i.sh` 启动时会主动删除它。执行器异常路径虽有 `finally`，但 SIGKILL/主机断电仍无法保证清理。

### 6.7 六轴和夹爪执行

当前 `execute_pick()` 的真实序列不是旧设计中的“预位→接近→抓取→抬升”，而是：

```text
最大张开夹爪
→ 读取稳定 full_open
→ 首次/缓存过期时完全闭合并读取稳定 full_closed，再次最大张开
→ approach（OMPL 自由空间运输）
→ grasp（Cartesian，fraction 必须为 1，禁止 OMPL 回退）
→ 完全闭合夹爪
→ 读取稳定窗口并分类；初判抓住再做保持复核
→ 沿原路径 Cartesian 退回 approach
→ 仅在退离成功后由上层回 zero
```

`pre_pose` 和世界 Z lift 仍关闭；抓后安全动作改为沿接近路径反向退回 approach。退离失败或执行中异常时禁止自动大范围回零，保持当前位置等待处理。

---

## 7. 坐标系、手眼、TCP 与单位

### 7.1 主要坐标帧

```text
base_link
├── link1 → link2 → link3 → link4 → link5 → link6
│                                      └── gripper_base
│                                           ├── joint7 → link7
│                                           └── joint8 → link8
└── camera_link
     └── RealSense depth/color/optical frames
```

`handeye_tf_publisher.py` 发布 `base_link` 为 parent、`camera_link` 为 child。YAML 中保存的是 `T_cam2base`，其语义实际是把 camera 坐标中的点变换到 base，即通常写作 `T_base_camera`。命名历史不统一，开发时必须依公式确认，不能只看变量名。

### 7.2 当前 handeye

`camera_data/handeye_eye_to_hand.yaml`：

```text
t_cam2base = [0.4500534950, 0.0139298440, 0.5723518201] m
R_cam2base =
[[ 0.47962911,  0.87052779,  0.11016938],
 [ 0.87469719, -0.48430516,  0.01879708],
 [-0.06971898,  0.08734922, -0.99373506]]
```

当前 `handeye_tf_publisher.load_handeye_tf()` 对矩阵维度、有限值、正交性和行列式做 fail-loud 校验；`grasp_transform.load_handeye()` 却在任何读取异常时退回一个历史仿真常量。这两种错误策略不一致，可能导致 TF 树和执行计算使用不同外参。生产化时应统一为 fail-closed，并记录 YAML 哈希。

### 7.3 位置和姿态变换

设 `R_CB,t_CB` 把 camera 变换到 base，GraspNet 输出 `p_C,R_GC`，则：

```text
p_base = R_CB @ p_camera + t_CB
R_base_tcp = R_CB @ R_grasp_camera @ M
```

2026-08-21 当前语义矩阵：

```text
M = [[ 0, 0,  1],
     [ 0, 1,  0],
     [-1, 0,  0]]
```

轴语义：

```text
TCP +Z = GraspNet +X（接近/进深轴）
TCP +Y = GraspNet +Y（夹爪宽度轴）
TCP +X = -GraspNet +Z
```

`det(M)=+1`。推理侧部分诊断列仍可能基于旧映射；实际执行只信 `grasp_transform.py` 的当前变换。

### 7.4 TCP 到 link6

MoveIt arm 组末端是 `link6`，候选目标代表 TCP。代码和 URDF 使用沿局部 TCP/link6 Z 的 0.1358 m 偏移：

```text
T_base_link6 = T_base_tcp @ Trans(0, 0, -0.1358)
p_link6 = p_tcp - R_tcp @ [0, 0, 0.1358]
```

预检和执行都必须使用同一个右乘局部偏移，不能把 0.1358 当 base Z 直接相减。历史代码/文档曾用 0.14 m，差 4.2 mm；当前以 URDF 关节几何 0.1358 m 为权威，但它仍不等价于现场“指尖实际夹持中心”的物理标定。

### 7.5 高度修正与 stage 位姿

```text
axis       = R_base_tcp[:,2]                            # TCP +Z 接近轴
grasp      = transformed_candidate + axis * grasp_z_offset
approach   = grasp - axis * approach_clearance
lift       = grasp + base_Z * open_gripper_lift_height  # 当前不用
```

三个 stage 保持同一姿态；抓取进深和 approach 退距均使用 TCP 局部接近轴，避免倾斜抓取时把 base-Z 修正投成横向毫米/厘米误差。lift 仍保持 base +Z 语义。

### 7.6 单位表

| 数据 | 单位 |
|---|---|
| ROS/MoveIt 位置 | m |
| ROS/MoveIt 关节 | rad |
| `depth.png` | uint16 mm |
| `meta.factor_depth` | 1000 mm/m |
| GraspNet CSV 坐标/宽度/深度 | m |
| Piper SDK 关节命令 | 0.001 degree 类内部单位，代码用约 57295–57325/rad 换算 |
| Piper SDK `GripperCtrl` position | 0.001 mm，即 `1e-6 m` |
| Piper SDK `EndPoseCtrl` 平移 | 0.001 mm 计数 |
| Piper SDK `EndPoseCtrl` 旋转 | 0.001 degree 计数 |

小夹爪总开口范围在项目中统一为 0–0.070 m，对应 `GripperCtrl(0..70000)`。旧的 `gripper_val_mutiple=2` 是驱动转发 `/joint_states` 的历史半行程语义；当前夹爪由 SDK 直控，执行器发送的是总开口，不能再叠乘 2。

---

## 8. 共享目录文件协议

### 8.1 文件所有权

| 文件 | 写入者 | 读取者 | 原子性/备注 |
|---|---|---|---|
| `color.png` | bridge | Docker | 原子替换，BGR 文件；Docker 读后转 RGB |
| `depth.png` | bridge | Docker | 原子替换，uint16 mm |
| `meta.mat` | bridge | Docker、诊断脚本 | 原子替换；含权威 K、D、factor_depth |
| `meta.json` | bridge | 无 scipy 时的备用/审计 | 原子替换；备用路径同样记录 K、D、factor_depth；正常 scipy 路径只生成 MAT |
| `camera_info.txt` | bridge | 人工审计 | 原子替换；同时记录 published/runtime K/D |
| `.ready` | bridge | Docker | 最后写；Docker消费后删除；latest-wins，不是队列 |
| `roi.json` | `test/roi_editor.py` | Docker、ROI 检查 | 持久配置，推理启动清理时保留 |
| `pause_inference` | executor | Docker | 存在即暂停；推理启动会删除残留 |
| `perception_grasps.csv` | Docker | executor、诊断脚本 | 临时文件 + replace；24 个基础列、当前 29 列，主候选接口 |
| `frame_manifest.json` | bridge | Docker、冻结快照、人工审计 | 同帧彩色/深度/CameraInfo 时间戳、偏差和递增序号 |
| `perception_targets.csv` | Docker | 旧工具 | 兼容接口 |
| `last_validation.json` | Docker | 人工/诊断 | 当前直接写，不是原子替换；可能读到半文件 |
| `precheck_results.json` | executor | Docker 可视化 | 原子替换；相机系 approved/selected 位姿 |
| `result.png` | Docker 显示线程 | viewer/人工 | 原子图像写；完整综合页面 |
| `result_color.png` | Docker | 人工 | 彩色辅助图 |
| `result_selected.png` | Docker | 人工 | 唯一执行最佳候选页面 |
| `grasp_result.json` | executor | 外部业务/人工 | 原子替换；历史旧文件可能是假阳性，必须结合版本/run manifest 使用 |
| `executions/<run_id>/` | executor | 复盘、回归 | 每次触发冻结输入、配置和 SHA-256，并记录 `selection.json` |
| `fk_link7_position.json` | executor | 人工 | 直接写；数据来自 MoveIt/FakeSystem 状态 |
| `output/*` | Docker debug | 人工/清理脚本 | 可选 PCD/NPY；受保存频率和上限控制 |

### 8.2 `.ready` 的并发语义

桥以 5 Hz 写，推理通常约 0.3–1 秒/帧。推理只检查文件是否存在并在处理后删除：

- 多个相机帧可能合并为一个待处理标志；
- bridge 可能在 Docker 删除前覆盖 `.ready`，造成新帧标志被一并删除；
- 丢帧是设计允许的，系统只处理“最新帧”；
- 不适合需要逐帧追溯或时间同步的测量任务。

如果以后需要可靠队列，建议改为带单调 frame ID 的目录/manifest 协议，或者直接用 ROS 2/DDS、共享内存或零拷贝 IPC。最小改造也应给 color/depth/meta 写同一个 generation ID，并让 manifest 最后原子发布。

### 8.3 当前相机内参事实

当前桥发布的是：

```text
calibration K:
fx=604.1376, fy=601.1167, cx=340.8270, cy=238.6749
D=[0.08309, 0.05348, -0.00742, 0.01038, -0.55547]
```

同一 `camera_info.txt` 记录 RealSense runtime K：

```text
fx=604.2141, fy=604.1147, cx=321.7812, cy=253.1111
D=[0,0,0,0,0]
```

主点相差约 `(19.05, -14.44)` 像素。不能因为 runtime `CameraInfo` 报告 D 为零，就直接把整条链切换到 runtime K：本项目的 `calibration.yaml` 来自 640×480 彩色流标定，随后 hand-eye 的 solvePnP 也使用了同一 K、D，因此它们共同定义了当前权威几何链。单独替换 K 会让 hand-eye 外参和像素射线模型不再配套；在当前画面中心附近，runtime K 与权威 K+D 的反投影可相差约 21 mm。

当前实现保留 calibration K、D，并在点云反投影前调用 `cv2.undistortPoints()` 得到归一化射线，再乘深度值；不再用“有畸变图像 + K-only”的近似，也不把 runtime K 当成执行几何。runtime K/D 仍写入 `camera_info.txt`，用途是审计相机输出，而不是覆盖标定配置。剩余必须完成的验证是使用多个已知 base 坐标的实体点，端到端检验 `pixel+depth → camera → base → TCP` 的绝对误差；内部参数一致不等于整条链已经达到毫米级真值精度。

---

## 9. 自研 ROS 包详细路由

### 9.1 包装和 Launch

| 文件 | 责任 |
|---|---|
| `src/real_piper_perception/package.xml` | ament_python 包元数据；当前声明依赖不完整，未列出 moveit_msgs、geometry_msgs、tf2_ros、yaml/scipy 等实际运行依赖 |
| `setup.py` | 安装 Python 包、Launch、SSIK 脚本；注册三个 console entry point |
| `setup.cfg` | ROS 可执行安装路径 |
| `launch/d435i_graspnet_pipeline.launch.py` | RealSense + bridge + handeye TF 的统一入口 |
| `launch/grasp_execute.launch.py` | 执行器参数入口，Launch 安全默认 `scan_only=true` |

入口点：

```text
d435i_graspnet_bridge → real_piper_perception.d435i_graspnet_bridge:main
grasp_executor        → real_piper_perception.grasp_executor:main
handeye_tf_publisher  → real_piper_perception.handeye_tf_publisher:main
```

### 9.2 模块职责

| 模块 | 核心职责 | 关键注意 |
|---|---|---|
| `d435i_graspnet_bridge.py` | ROS 图像转共享目录 | RGB-D 最大偏差 12 ms；输出 calibration K+D 和 frame manifest；单文件原子 |
| `handeye_tf_publisher.py` | 读取/校验外参，发布静态 TF | 校验严格；5 秒重复发送并非标准 StaticTF 必需 |
| `grasp_candidates.py` | 24/29 列 CSV 解析、排序、自洽、预检 JSON | score 稳定降序；selected index 是排序后序号 |
| `grasp_transform.py` | camera→base 与 GraspNet→TCP | 当前语义 M 正确；加载失败退仿真常量有风险 |
| `grasp_precheck.py` | stage 生成、SSIK/MoveIt plan-only 预检 | 当前只检查 approach/grasp；link6 偏移 0.1358 |
| `moveit_adapter.py` | Humble 下替代缺失的 moveit_py | 自建 action/service 客户端；已按 Humble 读取 `solution` 并收紧约束 |
| `grasp_execute.py` | 真实动作序列和夹爪 SDK 控制 | OMPL 到 approach；Cartesian 插入/退离；夹爪直连 can0 并做稳定反馈复核 |
| `grasp_executor.py` | ROS 服务、线程、门控、选择、结果 | MTE 4 线程；高分优先、廉价门控、Top-K 规划、冻结输入；异常结果仍需继续统一 |
| `gripper_limits.py` | 70 mm 开口单位和范围门控 | 纯函数，候选层和动作层双重 fail-closed |
| `grasp_verify.py` | 夹爪分类 | `classify_grasp` 在用；旧 `verify_grasp` 基本遗留 |
| `grasp_ik.py` | Pinocchio DLS 数值 IK | 硬编码 URDF；当前主执行流未使用 |
| `src/real_piper_perception/scripts/piper_ssik_ik.py` | 子进程封装 ssik Piper IK | 默认禁用；历史 58 次求解 0 成功，保留实验路由 |
| `sdk_safe_execute.py` | 禁区几何雏形 | 仅约 45 行；设计文档中的安全执行器未实现 |

### 9.3 `grasp_executor.py` 的线程模型

- ROS 节点使用 `MultiThreadedExecutor(num_threads=4)`；
- Trigger 回调调用 `_run_flow_in_thread()`，再启动 daemon worker；
- 服务回调等待 worker `join()`，但其他线程仍可处理反馈和 MoveIt future；
- 键盘线程支持输入 `g` 触发；非交互/服务化部署应考虑禁用；
- `_running` 防止并行执行两个抓取；
- `pause_inference` 的清理放在外层 `finally`；
- 真机开始前可回 zero；抓后只有 Cartesian 安全退离完成才允许自动回 zero，未知执行故障不再盲目穿越回零。

已知语义问题：

- 节点代码默认 `scan_only=False`，Launch 默认 `true`。不要绕过 Launch 直接 `ros2 run grasp_executor`，否则默认可能进入真机模式；
- `candidate_frames` 已在节点声明；当前默认 1，仅作为候选跟踪诊断，真正硬门控是物体中心连续 3 帧稳定性；
- scan-only 在 CSV 缺失或无候选时仍返回 `success=True`，且不写业务结果文件；
- `_run_flow` 的未捕获异常不一定经过 `_finish_failure()`，外部可能收不到结构化失败结果；
- `fk_link7_position.json` 和 FK 朝向来自 `/joint_states` 的 mock/命令状态，不是 `/joint_states_feedback` 的独立真机 FK。

### 9.4 MoveIt 适配器行为

`MoveGroupInterface` 将典型 moveit_commander API 映射为：

- `set_joint_value_target()`：构造每关节 ±0.005 rad 的 JointConstraint；
- `set_pose_target()`：缓存 link6 Pose；
- `plan()`：发 `/move_action`，`planning_options.plan_only=true`；
- Pose 位置约束是 0.006 m 立方体，即各轴约 ±0.003 m；
- 姿态绝对轴容差为 0.02 rad；
- `execute()`：发 `/execute_trajectory`；
- `compute_cartesian_path()`：调用 Humble 服务。

历史 bug：Humble `GetCartesianPath.Response` 的轨迹字段是 `solution`，旧代码读取 `resp.trajectory`。2026-08-21 日志曾出现：

```text
Computed Cartesian path ... followed 0.000000%
GetCartesianPath_Response object has no attribute 'trajectory'
回退 OMPL
```

当前代码已读取 `resp.solution`，并显式检查 `resp.error_code`、`fraction` 和 trajectory 是否为空。正式插入/退离要求 `fraction=1`，失败即 fail-closed，禁止回退 OMPL。旧宽位置容差在历史实验中造成过 7–13 mm 的规划终点偏差；收紧至 ±3 mm 只是在规划层缩小允许误差，仍不能替代基于真实关节反馈的执行终点验证。

---

## 10. GraspNet 子项目详细路由

### 10.1 运行入口

| 文件 | 作用 |
|---|---|
| `inference_from_camera.py` | 本项目实时推理主入口，约 1400 行 |
| `precheck_match.py` | 当前帧候选与 executor 的 approved/selected 相机位姿匹配 |
| `text_overlay.py` | PIL 中文文字叠加，字体缺失时降级 ASCII |
| `view_saved_result.py` | 查看已保存结果 |
| `demo.py`、`demo_headless.py`、`demo_save.py` | 上游单帧/演示入口，不是当前实时主路径 |
| `train.py`、`test.py` | 上游训练、评测 |
| `command_train.sh`、`command_test.sh`、`command_demo.sh` | 上游命令包装 |

### 10.2 模型路由

| 文件 | 责任 |
|---|---|
| `models/backbone.py` | PointNet++ backbone |
| `models/modules.py` | ApproachNet、CloudCrop、OperationNet、ToleranceNet |
| `models/graspnet.py` | Stage1/Stage2 总网和 `pred_decode` |
| `models/loss.py` | objectness/view/grasp loss |
| `pointnet2/*` | CUDA PointNet++ 查询、分组、采样；宿主修复版在运行时挂载进容器 |
| `knn/*` | KNN CUDA 扩展 |
| `utils/collision_detector.py` | 无模型碰撞检测 |
| `utils/data_utils.py` | 深度反投影、工作区掩码、点云变换 |
| `utils/label_generation.py` | 训练标签匹配和视角标签 |
| `utils/loss_utils.py` | 抓取视角、矩阵、Huber 等基础函数 |
| `dataset/graspnet_dataset.py` | GraspNet 数据集加载、增强、collate |
| `graspnetAPI_local/` | 本地安装的 GraspNet API/GraspGroup 定义 |

### 10.3 推理配置

`inference_from_camera.py` 的参数默认值：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `checkpoint_path` | `logs/log_rs/checkpoint.tar` | 容器内权重 |
| `shared_dir` | `/shared_camera_data` | bind mount |
| `num_point` | 20000 | 网络输入点数 |
| `num_view` | 300 | 视角数，与模型结构/权重对应 |
| `collision_thresh` | 0.01 | 小于等于 0 时关闭模型自由碰撞过滤 |
| `voxel_size` | 0.01 | 碰撞点云体素 |
| `top_k` | 50 | 可视化数量，不等于 CSV 全候选数量 |
| `poll_interval` | 0.1 s | `.ready` 轮询 |
| `min_object_points` | 80 | 物体点数门限 |
| `grasp_object_margin` | 0.08 m | 候选关联目标的外扩 |
| `candidates_per_object` | 300 | 每对象候选上限 |
| `segmentation_mode` | `color` | 代码默认；根脚本强制 `workspace` |
| `min_depth_m/max_depth_m` | 0.15/1.20 | 深度工作区 |
| `roi` | 空 | 静态 ROI；动态 roi.json 优先/补充 |
| `no_display` | false | headless 脚本传 true |

`run_graspnet_d435i.sh` 没有暴露 `voxel_size`、poll、object margin 等全部参数为环境变量；需要时可扩展脚本或直接 Docker 命令。headless 模式仍会使用 Open3D offscreen renderer，仍可能依赖 NVIDIA graphics/EGL，不是纯 CPU 无图形路径。

### 10.4 可视化匹配注意

`precheck_match.py` 的模块注释写姿态阈值 15°，实际常量已放宽为 90°。位置阈值 2 cm，若多个候选命中 selected，则按位置距离、姿态角和序号选唯一最近者。结果页只展示实际 selected 候选，不再展示全部 approved。

当前 90° 很宽，能提高帧间重新推理后的命中率，但也可能将临近不同姿态候选误配。可视化匹配只用于展示，不能反向作为执行安全证明。

### 10.5 Docker 镜像

`Dockerfile.20.04` 基于 `nvcr.io/nvidia/pytorch:22.03-py3`，主要固定：

- NumPy 1.24.4；
- SciPy 1.10.1；
- Open3D 0.17.0；
- 本地编译 pointnet2、knn 和 graspnetAPI；
- 默认 CUDA 架构 `8.6`；
- 容器工作目录 `/workspace`。

根运行脚本会把宿主机的 `inference_from_camera.py`、`precheck_match.py`、`text_overlay.py`、三个 pointnet2 文件和 Noto CJK 字体只读挂载进镜像。因此“镜像标签相同”不代表运行代码相同；排查时必须同时记录镜像 ID 和挂载文件哈希。

`docker-build.sh` 在构建前检查 `pointnet2_utils.py` 是否含 `ret_grouped_xyz` 修复，避免历史上的 tuple 解包崩溃。

---

## 11. Piper 驱动与 MoveIt 路由

### 11.1 包结构

| 包 | 作用 |
|---|---|
| `piper` | Python CAN 驱动和单/双臂 Launch |
| `piper_msgs` | `PiperStatusMsg`、`PosCmd`、`Enable.srv` |
| `piper_description` | URDF/xacro、mesh、显示 Launch |
| `piper_with_gripper_moveit` | 带夹爪 MoveIt SRDF、kinematics、控制器、Launch |
| `piper_no_gripper_moveit` | 无夹爪 MoveIt 配置 |
| `piper_gazebo` | Gazebo 仿真 |
| `piper_mujoco` | MuJoCo 资产/仿真 |
| `piper_humble` | 元/空壳包，不是当前控制实现 |

### 11.2 当前本地补丁

`src/piper_ros` 当前 8 个未提交修改的作用：

- `start_single_piper.launch.py` 增加 `gripper_forward`；
- `piper_ctrl_single_node.py` 在 `gripper_forward=false` 时禁止把 `/joint_states` 的 joint7 转发到夹爪，避免与执行器 SDK 直控双写；
- SRDF 增加 `ready` 关节位姿；
- 增加 `link1-link4` 禁碰对；
- 调整 ros2_controller update rate 为 100 Hz；
- 修正/简化 MoveIt controller 配置；
- 从 demo Launch 移除额外 `joint_state_publisher`；
- setup assistant 时间戳和 joint limit 数值格式变化。

这些修改不能简单用上游 checkout 覆盖。尤其真机启动必须显式传 `gripper_forward:=false`，否则 MoveIt 的 mock joint7 命令与 `grasp_execute.py` 的 SDK 夹爪命令会互相覆盖。

### 11.3 “FakeSystem 驱动真机”的真实含义

`piper_with_gripper_moveit` 的 ros2_control 使用 `mock_components/GenericSystem`。MoveIt 规划和控制器把轨迹写入 FakeSystem，FakeSystem 发布 `/joint_states`；Piper 驱动的 Launch 又把 `joint_ctrl_single` 重映射为 `/joint_states`，于是驱动把这些值当命令转成 CAN。

```text
MoveIt trajectory
→ joint_trajectory_controller
→ GenericSystem command state
→ /joint_states
→ piper_ctrl_single_node.joint_callback
→ MotionCtrl_2 + JointCtrl
→ CAN
```

因此：

- 机械臂确实会动；
- MoveIt 认为的 current state 主要是命令/FakeSystem 状态；
- 真机误差、跟随滞后和失步不会自动反馈回 MoveIt planning scene；
- 真反馈在 `/joint_states_feedback`，但 MoveIt 主链没有消费它；
- FK 诊断如果基于 `/joint_states`，只能验证计划终点，不是独立真机实测。

长期正确方案应实现或接入真实 ros2_control hardware interface，使状态接口来自 Piper 反馈、命令接口写 Piper，同时去掉“订阅 `/joint_states` 作为命令”的反常桥接。

### 11.4 URDF/SRDF

- arm 组：`base_link → link6`，joint1–joint6；
- gripper 组：`gripper_base → link7`，joint7；
- link8 由另一指爪关节形成，不在 arm 规划组；
- zero：六轴全 0；
- ready：`[0, 1.4919, 0, 0, -1.22, 0]`；
- gripper open：joint7 0.035 m 半行程；close：0；
- link6 到手指关节的固定 Z 几何为 0.1358 m。

SRDF 中禁碰对会直接影响自碰撞结果。新增 `link1-link4` 的原因和证据应补成独立回归，避免为了提高规划成功率而掩盖真实碰撞。

### 11.5 驱动关停缺陷

驱动 `main()` 的 `finally` 无条件 `rclpy.shutdown()`；节点内部自动使能超时路径也可能 shutdown。历史日志多次出现：

```text
rclpy.exceptions.ROSInterruptException: rclpy.shutdown() has been called
RuntimeError: Context must be initialized before it can be shutdown
Failed to publish: publisher's context is invalid
```

这通常发生在关停/超时，而不是运动中计算错误，但会让 Launch 报 exit code 1，并可能残留发布线程。应统一 shutdown 所有权，在 finally 中先 `if rclpy.ok()`，并让后台线程观察停止事件后 join。

---

## 12. RealSense 子项目路由

当前为 realsense-ros 4.58.3，要求 librealsense 2.58 级别。项目不直接修改上游，而从自研 Launch 启动可执行 `realsense2_camera_node`。

主要文件路由：

| 文件/目录 | 责任 |
|---|---|
| `realsense2_camera/src/realsense_node_factory.cpp` | 设备发现、创建、断线重连 |
| `base_realsense_node.cpp` | 传感器、帧回调、发布器、滤镜总编排 |
| `ros_sensor.cpp`、`sensor_params.cpp`、`profile_manager.cpp` | 传感器参数、profile、动态配置 |
| `image_publisher.cpp` | Image/CameraInfo 发布 |
| `align_depth_filter.cpp` | 深度对齐颜色 |
| `pointcloud_filter.cpp` | 点云；本项目 Launch 关闭 |
| `tfs.cpp` | 相机静态/动态 TF |
| `actions.cpp` | 触发标定等 action |
| `parameters.cpp`、`dynamic_params.cpp`、`ros_param_backend.cpp` | ROS 参数后端 |
| `launch/rs_launch.py` | 上游单相机通用 Launch |
| `rs_multi_camera_launch*.py` | 多相机/同步 Launch；当前不用 |
| `realsense2_camera_msgs` | Extrinsics、IMUInfo、Metadata、RGBD 及设备/标定服务 |
| `realsense2_description` | D435i xacro 和相机坐标树 |

自研链路只消费图像和 CameraInfo，没有消费 Metadata/Extrinsics/IMU，也没有用 RealSense 点云节点。相机 TF 树通过 handeye 的 `base_link → camera_link` 接到机器人树。

---

## 13. 命令行运行手册

以下命令都从项目根目录执行。每个新终端先运行：

```bash
cd .
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 13.1 构建

自研相机/感知包和其依赖：

```bash
./build_ws.sh
```

只重建自研包：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select real_piper_perception
```

Piper 本地补丁变更后：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to piper piper_with_gripper_moveit
```

构建脚本刻意给 colcon 使用系统 PATH，避免非系统/Homebrew `ld` 污染。构建成功后，新终端必须重新 source `install/setup.bash`。

### 13.2 构建 GraspNet Docker

```bash
cd graspnet-baseline
./docker-build.sh graspnet-baseline:20.04
cd ..
```

需要 NVIDIA Container Toolkit、可用 GPU 和能访问 NGC 基础镜像。默认 CUDA arch 8.6，不同 GPU 应核对。

### 13.3 只启动相机和感知

终端 A：

```bash
./run_d435i_bridge.sh
```

可调桥接发布率：

```bash
PUBLISH_RATE_HZ=5 ./run_d435i_bridge.sh
```

终端 B，GUI：

```bash
MIN_DEPTH_M=0.15 MAX_DEPTH_M=1.20 TOP_K=50 \
  ./run_graspnet_d435i.sh gui
```

终端 B，无主窗口：

```bash
MIN_DEPTH_M=0.15 MAX_DEPTH_M=1.20 TOP_K=50 \
  ./run_graspnet_d435i.sh headless
```

静态 ROI：

```bash
ROI=0.30,0.25,0.80,0.75 ./run_graspnet_d435i.sh headless
```

保存有限调试点云：

```bash
SAVE_DEBUG_OUTPUTS=1 SAVE_DEBUG_EVERY=20 MAX_SAVED_DEBUG_FRAMES=5 \
  ./run_graspnet_d435i.sh headless
```

动态 ROI GUI：

```bash
python3 test/roi_editor.py
```

查看推理结果：

```bash
./view_results.sh
```

### 13.4 离线/无运动 scan-only

scan-only 需要 MoveIt 服务进行规划，但不应发机械臂轨迹或夹爪命令。推荐将 MoveIt 分拆启动，避免不必要的硬件驱动：

终端 C：

```bash
ros2 launch piper_with_gripper_moveit rsp.launch.py
```

终端 D：

```bash
ros2 launch piper_with_gripper_moveit move_group.launch.py use_sim_time:=false
```

如果上述分拆环境没有 `/joint_states`，可使用项目验证过的 demo FakeSystem，但必须确认 Piper 驱动未运行，从而保证不会转发到真机：

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

终端 E：

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=true \
  enable_self_consistency:=true
```

触发：

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

观察：

```bash
ros2 topic echo /grasp_result
```

注意 scan-only 报告主要出现在服务 response 和 executor 日志；当前实现不保证写 `/grasp_result` 或 `grasp_result.json`。

### 13.5 真机基础链路检查

以下命令具备硬件影响。现场必须有急停/断使能手段、清空工作区、确认机械臂初始位姿，并保证同一时刻只有一个进程拥有 CAN 控制权。

激活 CAN：

```bash
bash src/piper_ros/can_activate.sh can0 1000000
ip -details -statistics link show can0
```

启动 Piper 驱动：

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=true \
  gripper_exist:=true \
  gripper_val_mutiple:=2 \
  gripper_forward:=false
```

`gripper_forward:=false` 对当前架构是强制要求。启动后至少检查：

```bash
ros2 topic hz /joint_states_feedback
ros2 topic echo --once /arm_status
ros2 topic echo --once /joint_states_feedback
```

再启动 MoveIt/FakeSystem：

```bash
LC_NUMERIC=en_US.UTF-8 \
  ros2 launch piper_with_gripper_moveit demo.launch.py
```

检查控制器和服务：

```bash
ros2 control list_controllers
ros2 service list | rg 'compute_fk|compute_cartesian_path'
ros2 action list | rg 'move_action|execute_trajectory'
```

### 13.6 真机执行器命令模板

**当前 P0 缺陷未修复，不建议执行本节命令。以下只记录架构入口，不是对当前代码的运行授权。**

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=false \
  velocity_scale:=0.05 \
  enable_self_consistency:=true \
  min_approach_down:=0.70 \
  graspnet_6d_max_candidates:=150
```

单段验证可用 `dry_run_max_stage:=approach` 或 `grasp`，但其名称“dry_run”容易误导：`scan_only=false` 时它会真实执行到指定 stage，只是不继续后续段。

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

不要在服务运行中同时执行 `sdk_go_zero.py`、`sdk_full_pipeline_test.py` 等直连 CAN 脚本。

### 13.7 停止顺序

建议按控制权由高到低停止：

1. 停止/不要再触发 `grasp_executor`；
2. 停止 Docker 推理；
3. 停止 MoveIt、RViz、ros2_control；
4. 停止相机桥；
5. 最后停止 Piper 驱动；
6. 检查残留进程、ROS graph、Docker 容器和 CAN 状态。

```bash
ps -ef | rg 'grasp_executor|inference_from_camera|move_group|piper_ctrl|realsense2_camera'
docker ps
ros2 node list
ip -details -statistics link show can0
```

不要用宽泛 `pkill -9 python`。驱动历史上关停可能报二次 shutdown；关停报错后要确认进程是否真的退出。

---

## 14. 脚本安全分级与功能路由

### 14.1 纯离线，无硬件命令

| 脚本 | 用途 |
|---|---|
| `check_piper_gripper_width.py` | 从已安装 SDK 源码验证 70 mm/70000 单位，不连接 CAN |
| `sdk_fk_vs_urdf_fk.py` | 对比 SDK DH FK 与 Pinocchio URDF FK |
| `sdk_mapping_validation.py` | 历史 M 轴映射离线验证；其中矩阵/说明已旧，不应作为当前权威 |
| `roi_coordinate_check.py` | 验证 ROI 掩码不改变反投影坐标 |
| `tf_tree_viewer.py --ascii/--save` | 解析 URDF、handeye、RealSense xacro，查看坐标树 |
| `result_viewer.py` | 只读结果 PNG |

### 14.2 ROS 只读或 plan-only

| 脚本 | 用途 | 风险 |
|---|---|---|
| `perception_stability_test.py` | 连续读取 CSV，统计 camera→base 稳定性 | 不动机械臂；依赖目标关联假设 |
| `fk_vs_camera_compare.py` | `/compute_fk` 与候选位置比对 | 注释称 `/joint_states` 真机，当前实际多为 mock/命令状态 |
| `tcp_matrix_plan_ab.py` | 冻结 CSV 的 legacy/semantic M plan-only A/B | 不调用 execute；仍要求 MoveIt 状态正确 |

### 14.3 会运动硬件

| 脚本 | 行为 | 使用原则 |
|---|---|---|
| `moveit_go_ready.py` | MoveIt 执行 SRDF ready | 走 FakeSystem→驱动→CAN；必须现场确认 |
| `sdk_go_ready.py` | SDK MoveJ 到 ready，5% | 独占 CAN；不能与驱动/执行器同时运行 |
| `sdk_go_zero.py` | SDK MoveJ 到 zero，5% | 不是“软件急停”；仍会真实运动 |
| `sdk_endpose_test.py` | SDK MOVE P 到硬编码目标 | 高风险实验 |
| `sdk_candidate_scan.py` | 候选/SDK 扫描 | 阅读完整参数和边界后才可运行 |
| `sdk_full_pipeline_test.py` | CSV→变换→EndPoseCtrl→回零 | 直接运动，且历史映射说明可能过时 |

### 14.4 维护脚本

| 脚本 | 说明 |
|---|---|
| `cleanup_output.sh` | 删除 debug output 中除最近 N 帧外的 PCD/NPY；属于删除操作 |
| `scripts/graspnet-docker-camera.sh` | 旧 Gazebo/目录结构脚本；`SCRIPT_DIR` 指向 `scripts`，会把共享目录和 GraspNet 路径解析错，当前不要使用 |

真正的实时推理入口是根目录 `run_graspnet_d435i.sh`。

---

## 15. 当前已知缺陷和技术债

### 15.1 P0：再次真机抓取前仍必须解决

以下三项已在 2026-08-21 代码层修复并有单元测试：Cartesian 正确读取 `solution` 且禁止插入/退离时 OMPL 回退；夹爪闭合使用稳定窗口并保持复核；候选按最高分且可规划优先。它们不等于系统已经具备无人值守真机安全条件，剩余 P0 是：

1. **MoveIt 状态不是真反馈**  
   执行终点必须同步保存 `/joint_states_feedback` 并用真实关节做 FK；中长期替换 FakeSystem 控制桥。

2. **没有真实环境碰撞模型**  
   `sensors_3d.yaml` 指向不存在/未加载的 `DepthImageOctomapUpdater` 和 Kinect 话题。日志持续报加载失败。没有桌面/目标场景时，OMPL 成功不等于真实路径安全。

3. **没有形成可统计的真机成功证据**  
   历史 `grasp_result.json` 的 `success=true` 已被稳定反馈和目标未移动证据推翻；当前实现还没有经过“安全抬升后、保持后二次验证”的足量实体试验，因此不能宣称或保证成功率。

4. **动作前硬件健康门不完整**  
   尚未在每次动作前强制核验 `arm_status`、错误码、控制模式、CAN error-active 和真实当前位置。任何一项未知都应阻止运动。

5. **当前实体与夹爪能力不匹配**  
   在线帧中的胶带环外抓跨度约 87–94 mm，而小夹爪最大开口 70 mm；选择策略无法克服物理尺寸限制。换目标或设计内撑/专用夹具前禁止以该目标验证外夹成功率。

6. **历史成功文件不可信**  
   `shared_camera_data/grasp_result.json` 的 `success=true` 已被稳定反馈和目标未移动证据推翻。下游不能用它训练或统计成功率。

### 15.2 P1：可靠性和正确性

- bridge 当前使用最新帧配对并拒绝时间偏差大于 12 ms；还不是 message_filters 意义上的队列同步；
- 单文件原子但整帧无 generation；
- calibration K+D 与 runtime K 主点差很大；当前已用 calibration K+D 去畸变射线，但仍缺少多真值点的端到端验证；
- `grasp_transform.load_handeye()` 失败时静默退历史仿真外参；
- scan-only 的 `success` 语义是“流程完成”而非“找到候选”；
- 未捕获异常不总能写/发结构化失败；
- `last_validation.json` 非原子写；
- FK JSON 非原子写；
- 候选稳定性硬门当前作用在目标中心，候选自身 `candidate_frames=1` 仍只是诊断；复杂多目标/遮挡场景需要实例级跨帧关联；
- 预检包含 approach/grasp，lift 仍未执行，`lift_pass` 不能解释为完成真实抬升验证；
- MoveIt PoseConstraint 已收紧，但没有真反馈闭环时仍可能存在跟随误差；
- driver 无条件 shutdown 导致关停异常；
- 执行器没有在动作前强制检查 `arm_status`、错误码、控制模式、CAN error-active；
- can0、URDF、shared dir、handeye 路径多处硬编码；
- 夹爪 SDK 与 Piper 驱动是两个 CAN 客户端，虽用 `gripper_forward=false` 避免同一命令源，仍需明确线程/总线所有权。

### 15.3 P2：可维护性

- `package.xml` 实际依赖声明不完整；
- 根目录无统一 Git；
- 缺少统一配置文件和 run manifest；
- 运行代码通过 Docker bind mount 覆盖镜像，不易复现；
- docstring 中“预位、抬升、joint7 半行程”等大量历史描述已过时；
- `grasp_ik.py`、`verify_grasp()`、旧 target CSV 等主路径外代码未明确标为 deprecated；
- `sdk_safe_execute.py` 只有禁区原型，配套 runner 不存在，不能宣称已有三层 SDK 安全执行；
- `precheck_match.py` 注释 15°、实际 90°；
- 推理清理函数启动时直接删除固定输出，缺少 run ID；
- 大日志主要因驱动 200 Hz 高频逐关节 INFO 输出，缺少日志级别和轮转。

---

## 16. 测试现状与安全测试矩阵

### 16.1 当前离线回归结果

在正确 source ROS 环境后，本次运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/real_piper_perception:$PWD/graspnet-baseline:$PWD/scripts:$PYTHONPATH"
/usr/bin/python3 -m pytest -q src/real_piper_perception/test test
```

结果：`114 passed`。

本轮同时修复了两个原有回归项：

- `ForbiddenZone.box_clearance()` 过去把所有盒内点算成 `-0.0`；当前按原始物理盒面返回标准有符号距离，并单独保留 5 mm 禁区缓冲语义；
- hand-eye 工具测试过去写死旧平移 `[0.4438,-0.0972,0.7446]`；当前同步权威 YAML `[0.4501,0.0139,0.5724]`，并交叉校验 `t_cam2base`、`T_cam2base[:3,3]` 和解析结果三者一致。

若未 source ROS，pytest 会在收集时因 `ModuleNotFoundError: rclpy` 失败。这是环境问题，测试命令必须写入 CI 脚本。

### 16.2 测试层级

| 层级 | 是否动硬件 | 典型内容 | 合入要求 |
|---|---|---|---|
| L0 纯函数 | 否 | CSV、变换、开口单位、分类、ROI、匹配 | 每次修改必跑 |
| L1 文件协议 | 否 | 冻结帧、原子写、generation、损坏文件 | 感知/IPC 修改必跑 |
| L2 ROS graph/mock | 否 | Launch、服务、action mock、参数、异常结果 | ROS 修改必跑 |
| L3 plan-only | 否，但依赖 MoveIt | 候选预检、Cartesian fraction、A/B | 规划/坐标修改必跑 |
| L4 台架单段 | 是 | 低速 approach/grasp，不闭夹爪或软物体 | 用户现场确认；记录 run manifest |
| L5 完整抓取 | 是 | 稳定闭合、抬升、保持、放置/回退 | P0 全通过后，单次授权 |

### 16.3 应新增的回归

- mock `GetCartesianPath.Response.solution` 和错误码；
- 空 trajectory、fraction 0/部分/1 的行为；
- 夹爪序列 `69.7→24→4.3 mm` 必须判空；
- 稳定阻挡开口序列必须判抓住，超时/无数据 fail-closed；
- 成功必须经过保持后二次验证；
- calibration K/runtime K 两套反投影对固定像素的差异；
- bridge RGB-D 时间戳/代号一致性；
- handeye 缺失/损坏不得退仿真常量；
- executor 所有异常路径都写一次 terminal result；
- Launch 参数与 node declaration 自动对照；
- score 排序/选择策略用冻结 CSV 做 golden test；
- `/joint_states` 与 `/joint_states_feedback` 明确来源测试；
- driver/gripper 双命令源互斥测试。

---

## 17. 日志清单、错误模式与历史演进

### 17.1 日志统计

`log/` 当前有 551 个普通文件、约 2.887 GiB：

- 50 个 `build_YYYY-MM-DD_HH-MM-SS` 目录；
- 415 个 colcon 组件日志；
- 11 个 driver session 和 4 个 8 月 8 日超大 driver 日志；
- 11 个 MoveIt session；
- 16 个 executor session，另有 scan-only、A/B、验证日志；
- 10 个 GraspNet session；
- 9 个 bridge session；
- 8 月 18 日 C/H/U/Stage 专项调查日志；
- 少量冻结 CSV 和 CAN trace。

最大文件：

| 文件 | 约大小 |
|---|---:|
| `driver_session_20260810_192344.log` | 584 MB |
| `run_2026-08-08/driver3.log` | 524 MB |
| `run_2026-08-08/driver4.log` | 477 MB |
| `run_2026-08-08/driver2.log` | 376 MB |
| `driver_session_20260814_181009.log` | 243 MB |

大部分体积来自 Piper 驱动高频打印 joint1–joint7，而不是独立异常。后续应将周期状态降为 DEBUG/节流，并配置日志轮转。

最近 10 次构建的 `stderr.log` 都为空。2026-08-21 16:49、17:12、17:13 三次 `real_piper_perception` 构建均 return code 0；`log/latest` 和 `latest_build` 指向 `build_2026-08-21_17-13-10`。历史上 2026-08-06 21:32:46 有一次 return code 1，随后构建恢复。

### 17.2 高频错误不等于同一严重度

日志关键词统计中约 43,110 次 `collision` 主要来自规划碰撞检查文本；约 1,390 次 `planning failed` 多为候选预检正常淘汰。真正需要单独处理的模式：

1. **OctoMap 插件加载失败**：`DepthImageOctomapUpdater` 不存在/话题是旧 Kinect；意味着无真实环境模型。
2. **driver shutdown traceback**：context 已 shutdown、publisher context invalid；多在关停或自动使能超时。
3. **MoveIt/RViz 关停 segfault**：8 月 18 日 demo 多次在 callback group 析构时段错误，通常出现在反复启动/关停。
4. **自动使能超时**：驱动打印 `Automatic enable timeout`，必须视为硬件未就绪，不能继续动作。
5. **SSIK 失败**：历史 analytic IK 大量 0 solution，当前默认关闭。
6. **Cartesian 字段错误（已修复）**：8 月 21 日确定造成过 OMPL 回退；当前读 `solution`、检查 error/fraction/trajectory。
7. **Gripper 瞬时反馈假阳性（已修复）**：当前采用稳定窗口和保持复核，历史 `69.7→24→4.3 mm` 序列判为空抓。

### 17.3 演进时间线

| 日期 | 主要变化/结论 |
|---|---|
| 08-02 | 部署 D435i→共享目录→Docker GraspNet，建立基础脚本；当时文档仍含 ROS1/仿真遗留命令 |
| 08-06 | 修 PointNet++ `ret_grouped_xyz`、动态 ROI、结果显示，开始开发执行器 |
| 08-07 | 增加 Humble MoveIt action/service 适配，形成 FakeSystem→driver 真机链，增加 `gripper_forward` |
| 08-08 | 大量 SDK/FK/映射实验；设计“SDK safe execute”，但最终只实现禁区类 |
| 08-10 | 预检默认关闭 analytic IK、改 KDL/MoveIt，预算 0.5 s；动作起点后来由 ready 改 zero |
| 08-14 | 多次抓空后改 score 升序、候选 150，关闭 pre_pose/lift，增加宽度/夹爪实验 |
| 08-16 | ROI 坐标检查、TF 工具、冻结帧调查 |
| 08-18 | 量化真机执行误差、MoveIt 容差、内参/手眼/TCP 假设；确认 FakeSystem 不是独立反馈 |
| 08-20 | 收敛变换和误差调查，准备新 TCP 语义验证 |
| 08-21 | 切换语义 TCP 矩阵；一次真机抓空后修复 Cartesian/夹爪反馈；完成 K+D 射线、平面分割、高分可规划 Top-K 和 scan-only A/B |

历史事实要按日期理解。例如 8 月 18 日“meta K 等于 runtime K”的结论只对当时帧成立；当前权威链使用 calibration K+D，且 inference 以去畸变归一化射线反投影，不能继续引用旧结论为现状。

### 17.4 历史文档导航

| 需求 | 优先阅读 |
|---|---|
| 当前最新状态 | `docs/2026-08-21-session-summary.md` |
| 最新 TCP 真机结果 | `docs/real_grasp_error_investigation/real-grasp-new-tcp-20260821.md` |
| 误差分解 | `root-cause-report.md`、`follow-up-report.md`、`moveit-runtime-consistency-report.md` |
| 原始测量索引 | `measurement-log.md` 和 investigation `log/*.json` |
| 执行器设计演进 | `docs/superpowers/specs/2026-08-06-grasp-executor-design.md` 与对应 plan |
| SDK 安全执行未完成设计 | `2026-08-08-sdk-safe-execute-design.md`、plan 和当前 `sdk_safe_execute.py` 对照阅读 |
| 早期部署 | `2026-08-02-d435i-graspnet-deployment-log.md` |

不要把 `specs/plans` 当已实现清单；它们描述目标设计，必须和当前源码逐项比对。

---

## 18. 面向新开发的修改路由

### 18.1 修改相机/内参/同步

主要文件：

```text
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
camera_data/calibration.yaml
graspnet-baseline/inference_from_camera.py:read_data_from_shared/process_data
graspnet-baseline/utils/data_utils.py
scripts/roi_coordinate_check.py
```

必须验证：时间同步、K/D 与图像是否同一几何、factor_depth、固定像素反投影、冻结帧兼容、文件 generation。

### 18.2 修改 ROI/分割/目标选择

```text
test/roi_editor.py
graspnet-baseline/inference_from_camera.py:
  parse_roi/load_dynamic_roi/process_data/segment_object_points
scripts/roi_coordinate_check.py
```

确保继续使用 mask 而非未经主点补偿的 crop。若引入实例分割，需要给 target ID、时间和候选关联定义稳定协议。

### 18.3 修改网络或候选生成

```text
graspnet-baseline/models/*
graspnet-baseline/dataset/*
graspnet-baseline/utils/collision_detector.py
graspnet-baseline/inference_from_camera.py:
  run_inference/collision_filter/select_object_grasp_targets/write_frame_validation
src/real_piper_perception/real_piper_perception/grasp_candidates.py
```

需要版本化 checkpoint、`num_view`、CSV schema，并保留冻结帧与 golden candidate 排名。

### 18.4 修改坐标/TCP/手眼

唯一执行权威应收敛到：

```text
grasp_transform.py
grasp_precheck.py::_tcp_to_link6_matrix/make_stage_poses
grasp_execute.py::_tcp_pose_to_link6
camera_data/handeye_eye_to_hand.yaml
piper_description.urdf
```

同时更新：变换单测、TF viewer、plan-only A/B、矩阵语义文档。不要只改推理侧诊断常量或脚本里的旧 M。

### 18.5 修改候选门控和排序

```text
grasp_candidates.py:rank_candidates/self_consistency
gripper_limits.py
grasp_executor.py:_run_flow_impl
grasp_precheck.py:precheck_stages
```

建议将硬门与软排序分开：

```text
硬门：有限值、宽度、工作区、碰撞、可规划、approach_down
软排序：网络 score、approach_down、路径长度、最小碰撞余量、关节余量
```

每次策略修改应用同一冻结 CSV 输出完整排名差异，而不是只报告最终一个 index。

### 18.6 修改 MoveIt/真实反馈

```text
moveit_adapter.py
grasp_precheck.py
grasp_execute.py
grasp_executor.py
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/*
src/piper_ros/src/piper/piper/piper_ctrl_single_node.py
```

首选路线是实现真实 ros2_control hardware interface。过渡方案至少应让 `/joint_states` 来自真实反馈，同时将命令改到独立 command topic，避免一个标准状态话题被用作命令。

### 18.7 修改夹爪判定

```text
grasp_execute.py:_feedback_cb/_read_gripper_feedback_wait/execute_pick
grasp_verify.py:classify_grasp
gripper_limits.py
src/piper_ros/.../piper_ctrl_single_node.py
```

需要保存完整时间序列而不是单值，至少包含命令时间、反馈时间戳、开度、速度估计、稳定窗口、full_open/full_closed、保持前后结果。

### 18.8 修改文件协议/API

```text
d435i_graspnet_bridge.py
inference_from_camera.py
grasp_candidates.py
grasp_executor.py
precheck_match.py
result_viewer.py
```

建议新 schema 带：`schema_version`、`run_id`、`frame_id`、源时间戳、文件哈希、K/handeye/checkpoint 哈希、候选排序版本。新旧协议迁移期必须兼容读取或明确 fail-loud。

---

## 19. 建议优化路线图

### 阶段 A：恢复结果可信度

1. [已完成] 修 Cartesian `solution`、错误码、fraction 和空轨迹检查；
2. [已完成] 实现夹爪稳定反馈、保持后二次验证；
3. 所有终态统一写结构化结果；
4. 禁止历史假阳性数据进入评估集；
5. 扩展 L0–L3 回归并建立真机人工复核样本集。

### 阶段 B：恢复状态可信度

1. 将真实 `/joint_states_feedback` 接入 FK 记录；
2. 建 Piper ros2_control hardware interface；
3. 取消 `/joint_states` 命令重映射；
4. 统一 CAN 所有权和夹爪控制接口；
5. 在动作前做 arm/CAN/controller 健康门。

### 阶段 C：恢复几何可信度

1. [进行中] K+D 已进入反投影；仍需用多个实体真值点证明与 RealSense 输出图像的几何对应；
2. [部分完成] 拒绝 RGB-D 偏差 >12 ms；后续改为带队列的严格配对；
3. 用 ≥3 个基座真值点验证 camera→base 绝对位置；
4. 实测 link6→夹持中心 TCP；
5. 将 handeye 和 TCP 版本、哈希写入每次 run manifest。

### 阶段 D：恢复路径安全

1. 配置 D435i 点云/深度 OctoMap 或明确的桌面碰撞体；
2. 移除旧 Kinect sensors 配置；
3. [已完成规划层] Pose/Joint/Orientation tolerance 收紧至 ±3 mm/±0.005 rad/0.02 rad；
4. 预检与真实执行的 stage、碰撞物和控制状态完全一致；
5. [已完成几何层] approach/retreat 沿 TCP 局部轴；后续补足基于真实状态的故障撤退策略；
6. 对 SRDF disable collision 做物理审计。

### 阶段 E：工程化

1. 建立根级 Git/子模块或 manifest；
2. 完善 package 依赖和可移植路径；
3. 统一 YAML 配置、运行 ID、日志轮转；
4. 固化 Docker 镜像 digest，减少运行时源码覆盖；
5. CI 跑纯函数、协议、ROS mock 和 plan-only；
6. 建立有人工授权的硬件测试清单和数据归档模板。

---

## 20. 参数总表

### 20.1 bridge/handeye

| 参数 | 当前 Launch 默认 |
|---|---|
| `shared_dir` | `~/real_piper_graspnet/shared_camera_data` |
| `publish_rate_hz` | 5.0 |
| `factor_depth` | 1000.0 |
| `depth_is_meters` | false |
| `calibration_yaml` | `camera_data/calibration.yaml` |
| `use_calibration_intrinsics` | true，Launch 硬传 |
| `max_color_depth_skew_ms` | 12.0；超限帧不发布 |
| `handeye_yaml` | `camera_data/handeye_eye_to_hand.yaml` |
| `parent_frame/child_frame` | `base_link/camera_link` |

### 20.2 executor

| 参数 | Launch 默认 | 语义 |
|---|---:|---|
| `scan_only` | true | 安全默认；节点裸跑默认却是 false |
| `dry_run_max_stage` | 空 | 真机截断 stage，不等于不运动 |
| `velocity_scale` | 0.1 | MoveIt 速度比例 |
| `approach_clearance` | 0.05 m | 沿 TCP 局部接近轴退到 approach 的距离 |
| `lift_z` | 0.10 m | 当前执行 lift 关闭，基本未用 |
| `open_gripper_lift_height` | 0.10 m | 生成 lift 位姿，当前不预检/执行 |
| `grasp_z_offset` | 0.02 m | 沿 TCP 局部接近轴的抓取进深修正 |
| `gripper_margin` | 0.005 m | 候选宽度余量 |
| `gripper_detect_threshold` | 0.01 m | 旧判定参数；当前 classify 主要用开闭基线容差 |
| `graspnet_6d_max_candidates` | 150 | 每帧最多遍历 |
| `candidate_frames` | 1 | 已声明；候选级跟踪诊断默认不作硬门 |
| `grasp_score_min` | 0.0 | score 硬门 |
| `min_approach_down` | 0.70 | TCP +Z 对 base -Z 的余弦门 |
| `enable_self_consistency` | true | 自适应 XY 门：`max(30mm, object_radius+10mm)`，封顶 80mm |
| `max_orientation_variants` | 2 | 当前主流程影响有限/历史参数 |
| `enable_piper_analytic_ik_planning` | false | 禁用 ssik |
| `graspnet_6d_analytic_only_precheck` | false | 不用 analytic-only 短路 |
| `piper_analytic_ik_max_solutions` | 8 | 历史 ssik |
| `piper_analytic_ik_max_solution_plans` | 6 | 历史 ssik |
| `planning_time_precheck` | 0.5 s | 每次预检规划 |
| `planning_time_exec` | 10.0 s | 正式规划 |

### 20.3 根 Docker 脚本环境变量

```text
IMAGE_NAME, CHECKPOINT_PATH, MIN_DEPTH_M, MAX_DEPTH_M, ROI, TOP_K,
NUM_POINT, NUM_VIEW, COLLISION_THRESH,
SAVE_DEBUG_OUTPUTS, SAVE_DEBUG_EVERY, MAX_SAVED_DEBUG_FRAMES
```

---

## 21. 接手开发者的最短检查清单

第一次接手时，不要直接开真机。按顺序做：

1. 阅读本文、8 月 21 日总结和最新真机报告；
2. 确认根目录不是 Git，备份所有将改文件；
3. `git -C src/piper_ros status --short`，保护 8 个本地补丁；
4. source ROS 和 install，运行离线 pytest，确认已知 2 个失败而无新增失败；
5. 用冻结帧检查 CSV 24 个基础列/当前 29 列、K+D、handeye、M、0.1358 offset；
6. 只启动相机/推理，检查 `camera_info.txt` 中 published/runtime K；
7. 只做 scan-only，记录候选数、门控数和 selected；
8. 修完 P0 后才做 plan-only Cartesian 验证；
9. 硬件实验前重新采样 CAN、arm_status、控制器、真实反馈，不能引用旧会话的“当时正常”；
10. 每次真机只做一个变量的低速单次实验，并冻结输入、参数、代码哈希和结果。

---

## 22. 最终评价

该项目已经具备完整的研究型闭环骨架：真实 RGB-D、GPU 6D 抓取、坐标变换、MoveIt 预检、Piper 真机运动、夹爪反馈、可视化和丰富实验证据都已接通。当前最有价值的资产不是某一次 `success`，而是逐步形成的变换验证、冻结帧、plan-only A/B、真反馈调查和故障日志。

它目前的主要问题集中在“接口边界的真实性”：文件握手不等于帧事务，FakeSystem 状态不等于真机状态，第一条新夹爪反馈不等于稳定反馈，MoveIt 规划成功不等于有真实环境碰撞安全，历史文档结论也不等于当前代码事实。后续优化应优先修复这些边界，再讨论提高网络得分、候选数量或自动重试。只有当输入帧、坐标、状态、轨迹和结果五层都可追溯，系统才适合长期自动化运行。

---

## 23. 项目专有文件完整路由附录

本节补充前文按架构归类后不便逐项出现的项目专有文件。第三方仓库中大量 mesh、许可证、CI 和通用上游测试不逐文件解释；它们分别受 Piper、RealSense 和 GraspNet 上游结构管理。

### 23.1 自研包测试

| 文件 | 覆盖内容 |
|---|---|
| `src/real_piper_perception/test/test_grasp_candidates.py` | 24 列解析、当前排序、self-consistency、precheck JSON |
| `test_grasp_execute_units.py` | 开口/半行程历史接口、动作单元逻辑、边界 |
| `test_grasp_executor_mte.py` | Trigger 阻塞时 MTE + Reentrant 回调仍可接收反馈 |
| `test_grasp_precheck.py` | stage 位姿、TCP→link6、预检分支 |
| `test_grasp_transform.py` | handeye、camera→base、当前/历史 M 与行列式 |
| `test_grasp_verify.py` | 旧判定和当前分类边界 |
| `test_gripper_limits.py` | 0–70 mm、SDK 单位和超宽 fail-closed |
| `test_handeye_tf_publisher.py` | YAML 解析、矩阵校验、frame 名 |
| `test_sdk_safe_execute.py` | ForbiddenZone；当前有 `-0.0` 失败 |
| `fixtures/sample_grasps.csv` | 候选解析冻结 fixture |
| `fixtures/handeye_fixture.yaml` | 不依赖现场 YAML 的手眼 fixture |

### 23.2 根级测试和交互工具

| 文件 | 覆盖内容 |
|---|---|
| `test/test_precheck_match.py` | approved/selected 位置姿态匹配和唯一最近项 |
| `test/test_text_overlay.py` | 中文字体/ASCII 降级、图像尺寸和写入 |
| `test/test_tf_tree_viewer.py` | URDF/handeye/xacro 树解析；当前写死旧 handeye 导致 1 失败 |
| `test/fixtures/two_link_robot.urdf` | TF viewer 最小 URDF fixture |
| `test/fixtures/handeye_rotated.yaml` | 旋转手眼 fixture |
| `test/roi_editor.py` | 交互写 `shared_camera_data/roi.json`，属于运行工具而非 pytest |

### 23.3 标定和当前快照

`camera_data/` 只有两个权威文件：

| 文件 | 当前用途 |
|---|---|
| `calibration.yaml` | bridge 写 `meta.mat` 的 K/D；当前 SHA256 会写入 `camera_info.txt` |
| `handeye_eye_to_hand.yaml` | executor 变换和 handeye TF 的 camera→base 外参 |

`shared_camera_data/` 当前顶层包含 `.ready`、五个输入/配置文件、两个 CSV、三个业务 JSON、三个结果 PNG 和 FK JSON。它们只是 2026-08-21 最后运行遗留快照；`.ready` 仍存在不代表当前有相机或推理进程运行。

### 23.4 GraspNet 补充文件

| 文件/目录 | 说明 |
|---|---|
| `requirements.txt` | 上游 Python 依赖参考；实际镜像以 Dockerfile 固定版本为准 |
| `PLAN.md`、`DOCKER_DEMO_USAGE.md` | 早期计划/演示说明，不是当前真机命令权威 |
| `.Dockerfile.nobuildkit`、`Dockerfile.20.04.nobuildkit` | 无 BuildKit 备用构建 |
| `docker-run-demo.sh` | 上游 demo 容器入口，不使用共享目录实时协议 |
| `doc/teaser.png` | 上游说明图 |
| `graspnetAPI_local/*` | 本地 API 包源码、抓手定义图、安装脚本 |
| `logs/log_rs/checkpoint.tar` | 当前推理权重，12,468,415 字节 |
| `dataset/command_generate_tolerance_label.sh` | 训练标签生成入口 |
| `dataset/generate_tolerance_label.py` | 离线容差标签计算 |

### 23.5 Piper 的备用入口

| 文件 | 说明 |
|---|---|
| `piper_ctrl_single_node_new.py` | 另一套命名/API 的单臂驱动版本；当前 `setup.py` 入口不是它 |
| `piper_read_slave_joint.py` | 从从臂读取/发布关节的双臂辅助节点 |
| `start_single_piper_rviz.launch.py` | 单臂驱动连 RViz 的便利入口 |
| `start_two_piper.launch.py` | 双臂入口，当前项目不用 |
| `piper_description/launch/*` | 有/无夹爪及示教版 URDF/xacro 展示 |
| `piper_no_gripper_moveit/launch/*` | 无夹爪 MoveIt 入口，当前抓取不用 |
| `piper_with_gripper_moveit/launch/setup_assistant.launch.py` | MoveIt Setup Assistant；会改配置，非运行入口 |
| `spawn_controllers.launch.py` | 单独启动 ros2_control controllers |
| `static_virtual_joint_tfs.launch.py` | 虚拟关节静态 TF |
| `warehouse_db.launch.py` | MoveIt warehouse 数据库 |
| `piper_sim/piper_gazebo/*` | 仿真控制器和 Gazebo Launch；当前真机数据流不经过 |

### 23.6 RealSense 的未使用扩展包

| 包/目录 | 当前关系 |
|---|---|
| `realsense2_rgbd_plugin` | RViz RGBD 显示插件，当前流水线不依赖 |
| `realsense2_ros_mqtt_bridge` | MQTT 桥，当前不用 |
| `realsense2_camera_msgs/action/TriggeredCalibration.action` | 触发标定接口，当前不用 |
| `realsense2_camera_msgs/msg/{Extrinsics,IMUInfo,Metadata,RGBD}.msg` | 上游扩展消息，当前 bridge 未订阅 |
| `realsense2_camera_msgs/srv/*` | 设备信息、硬件监控、标定/安全配置服务，当前主路径不用 |
| `res/`、`.github/`、`security.md`、`CONTRIBUTING.md` | 上游资源、CI、安全和贡献文档 |

### 23.7 SDD 账本的正确阅读方式

`.superpowers/sdd/grasp-executor/` 保存 Task 1–8 brief/report、`final-review.md`、两轮修复报告和 `progress.md`。它很好地解释了 MTE、夹爪单位、24 列 CSV、TCP 和预检最初如何形成，但时间停留在 8 月 6 日：其中“score 降序+姿态桶”“三段预检”“pre_pose/lift”“旧 TCP M”等都已被后续迭代替代。

`.superpowers/sdd/2026-08-08-sdk-safe-execute/` 只有 Task 1 禁区几何 brief 和进度账本，印证完整 safe-execute 计划没有继续实现。判断功能是否存在时应查看当前 `sdk_safe_execute.py`，不能按 plan 的任务列表推断。
