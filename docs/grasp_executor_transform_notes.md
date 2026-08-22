# grasp_executor 坐标变换链调研笔记

日期：2026-08-06
任务：Task 1（变换链调研，grasp-executor 计划）
用途：grasp_transform.py 编码的唯一依据（常量数值均从源码原样拷贝，非臆造）

---

## 0. 数据流总览

```
D435i ─→ shared_camera_data/{color,depth,meta} ─→ Docker GraspNet 推理
(inference_from_camera.py) ─→ perception_grasps.csv（相机系 6D 候选，24 列）
──→ grasp_executor（本计划要写的代码）: CSV → 基座系/TCP 系 → MoveIt 执行
```

涉及源码：
- `./graspnet-baseline/inference_from_camera.py`（真机感知侧，写 CSV 的一端）
- `./camera_data/handeye_eye_to_hand.yaml`、`calibration.yaml`（真机标定）
- `<source-workspace>/gazebo_piper_pick_place/src/piper_gazebo/src/graspnet_perception_loader.cpp`（仿真读 CSV 端）
- `<source-workspace>/gazebo_piper_pick_place/src/piper_gazebo/src/grasp_frame_transform.cpp`（仿真变换实现）
- `<source-workspace>/gazebo_piper_pick_place/src/piper_gazebo/include/piper_gazebo/grasp_common.hpp`（仿真常量）
- `<source-workspace>/gazebo_piper_pick_place/src/piper_gazebo/src/graspnet_6d_grasp_planner.cpp`（仿真候选规划器）
- `<source-workspace>/gazebo_piper_pick_place/src/piper_gazebo/src/pick_and_place.cpp`（仿真执行器）

---

## 1. 变换公式链（numpy 记号）

### 1.1 CSV 输入（全部为相机系原值，单位米）

```
p_camera     = np.array([camera_x, camera_y, camera_z])     # 抓取中心（GraspNet translation）
R_grasp_cam  = np.array([r00..r22]).reshape(3, 3)           # GraspNet 抓取系 → 相机系旋转（行主序）
p_center_cam = np.array([center_camera_x/y/z])              # 物体中心（识别/门控用，非抓取中心）
```

CSV 24 列布局（写端 `inference_from_camera.py write_frame_validation`，读端 `graspnet_perception_loader.cpp`）：

| 索引 | 列 | 语义 |
|---|---|---|
| 0 | name | 物体名（真机 workspace 模式恒为 `target`；颜色模式为 red_box/green_cylinder/blue_cylinder） |
| 1-3 | camera_x/y/z | 抓取中心相机系坐标（米），grasp_array[:, 13:16] 原值 |
| 4 | count | 该目标点云点数 |
| 5 | score | GraspNet 置信度 |
| 6-8 | width/height/depth | 抓取尺寸（米） |
| 9-17 | r00..r22 | **原始 GraspNet 旋转矩阵**（行主序，相机系），grasp_array[:, 4:13] 原值 |
| 18-20 | center_camera_x/y/z | 物体中心相机系坐标（米） |
| 21 | link6_grasp_z | 诊断列：世界系抓取 z（= p_world[2]，PIPER_TOOL_Z_OFFSET=0 时即 link6 z） |
| 22 | tcp_forward_z | 诊断列：world_tcp_rotation[2,2]（TCP 前向轴在世界 Z 的分量） |
| 23 | source_dist_xy | 诊断列：抓取中心离物体源先验的 XY 距离（C++ 端忽略） |

### 1.2 位置变换：相机系 → 世界系（仿真）/ 基座系（真机）

```
# 仿真（Python select_object_grasp_targets:725 与 C++ cameraToWorld() 同一公式）：
p_world = CAMERA_TO_WORLD_R @ p_camera + CAMERA_TO_WORLD_T

# 真机（handeye eye-to-hand，camera→base）：
p_base  = R_cam2base @ p_camera + t_cam2base
```

### 1.3 姿态变换：GraspNet 抓取系 → 世界系/基座系 TCP

```
# 仿真 Python（inference_from_camera.py:735，仅用于排序/诊断）：
R_world_tcp = CAMERA_TO_WORLD_ROTATION_R @ R_grasp_cam @ GRASPNET_TO_PIPER_TCP_R

# 仿真 C++（grasp_frame_transform.cpp cameraGraspRotationToWorldTcp，实际执行用）：
R_world_tcp = kCameraToWorldRotationR @ R_grasp_cam @ kGraspNetToPiperTcpR

# 真机（grasp_transform.py 待实现，结构相同、外参换成 handeye）：
R_base_tcp  = R_cam2base @ R_grasp_cam @ GRASPNET_TO_PIPER_TCP_R
```

乘法顺序：相机外参 **左乘**、GraspNet→TCP 轴映射 **右乘**（左右不可调换）。

### 1.4 旋转矩阵 → 四元数

- C++ 参考实现：Shepperd 方法（`grasp_frame_transform.cpp quaternionFromRotation`，对 det=-1 的镜像矩阵也归一化吸收）。
- Python 实现可用 `scipy.spatial.transform.Rotation.from_matrix(R).as_quat()`，注意 scipy 返回 `[x,y,z,w]` 与 ROS 一致。

### 1.5 TCP 高度换算（仿真参考，真机沿用）

```
z_tcp = max(p_world.z + grasp_z_offset + tool_z_offset,
            min_grasp_z, kMinReachableTcpGraspZ)
```
GraspNet 的 z 是抓取中心高度，须换算成 TCP 高度并做下限保护（防规划出低于桌面的点）。

---

## 2. 常量数值表（源码原样拷贝）

### 2.1 感知侧 Python 常量（inference_from_camera.py 136-169 行）

```python
CAMERA_TO_WORLD_R = np.array([
    # 相机系 → 世界系旋转（位置/度量用；中间行 -1.0，det=+1，真旋转）
    [0.0, -0.82806033, 0.56063899],
    [-1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_ROTATION_R = np.array([
    # 姿态变换专用（中间行 +1.0，det=-1 镜像，IK 可解性历史修复点）
    [0.0, -0.82806033, 0.56063899],
    [1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_T = np.array([0.072, 0.021, 0.548], dtype=np.float64)  # 米
GRASPNET_TO_PIPER_TCP_R = np.array([
    # GraspNet 局部系 → Piper TCP 系轴映射（Python 版，注释声称是 C++ 常量镜像）
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)
PIPER_TOOL_Z_OFFSET      = 0.0   # TCP 与 link6 无 Z 偏移
PIPER_MIN_GRASP_Z        = 0.0
PIPER_MIN_LINK6_GRASP_Z  = 0.0
```

### 2.2 仿真 C++ 常量（grasp_common.hpp，原样拷贝）

```cpp
// 标识
kNodeName       = "piper_gazebo_pick_and_place"
kMoveGroupName  = "arm"
kEndEffectorLink = "grasp_tcp"

// 笛卡尔规划与码放
kCartesianStep       = 0.005   // 米
kCartesianMinFraction = 0.9
kLastObjectHeightOffset = 0.04 // 米

// 相机外参（三点反求标定，有效俯仰角约 55.9°）
kCameraToWorldPositionR[3][3] = {{0.0, -0.82806033, 0.56063899},
                                 {-1.0, 0.0, 0.0},
                                 {0.0, -0.56063899, -0.82806033}}
kCameraToWorldRotationR[3][3] = {{0.0, -0.82806033, 0.56063899},
                                 {1.0, 0.0, 0.0},
                                 {0.0, -0.56063899, -0.82806033}}
kCameraToWorldT[3] = {0.072, 0.021, 0.548}   // 米

// 感知与门控
kPerceptionMaxAgeSec       = 20.0  // 秒
kMinPerceptionPoints       = 80
kVisionTargetMaxError      = 0.08  // 米
kVisionCandidateMaxCenterError = 0.08  // 米
kMinReachableTcpGraspZ     = 0.005 // 米
kDefaultMaxGraspDistance   = 0.08  // 米
kMaxAttachCenterDistance   = 0.08  // 米

// 物体几何
kObjectCylinderRadius = 0.024   // 米
kObjectCylinderLength = 0.0675  // 米

// 规划时间
kCandidatePlanningTime = 5.0    // 秒（预检 dry-run）
kExecutionPlanningTime = 10.0   // 秒（正式执行）

// GraspNet 抓取系 → Piper TCP 系映射（calib_18 唯一方案）
kGraspNetToPiperTcpR[3][3] = {{0.0, -1.0, 0.0},
                              {0.0, 0.0, -1.0},
                              {1.0, 0.0, 0.0}}
// 注释语义：X→Z、Y→-X、Z→-Y（实测接近角 127.7° 失败案例后收敛到该方案）
```

### 2.3 真机标定（camera_data/handeye_eye_to_hand.yaml，原样拷贝）

```yaml
R_cam2base:   # 3x3 旋转矩阵（det=+1，已验证正交归一）
  [[0.03525509366404572, -0.9974666236163753, -0.06178521782828538],
   [-0.9991918564790753, -0.03398679266254146, -0.02146000630039423],
   [0.019305758639300414, 0.06249186103695917, -0.997858734985819]]
t_cam2base:   # 平移（米）
  [0.4840150280511777, -0.04460850623283941, 0.5962571391723253]
T_cam2base:   # 冗余的 4x4 齐次矩阵，与 R_cam2base/t_cam2base 完全一致（已校验）
  # 行结构：[r00 r01 r02 tx; r10 r11 r12 ty; r20 r21 r22 tz; 0 0 0 1]
description: 'Eye-to-Hand calibration result: T_cam2base'
```

`calibration.yaml`（相机内参，写 CSV 的推理端另有一套 meta.mat 内参，两者不在此任务范围）：

```yaml
camera_matrix: fx=604.137589082592, fy=601.1167172706504,
               cx=340.8269659342467, cy=238.67489537802712
dist_coeff: [0.08309115999174305, 0.053479165785753936, -0.007422873023980639,
             0.010381721469496044, -0.5554660896928784]
reprojection_error: 0.4214495223523248
```

### 2.4 本任务直接要求的 5 个常量（核对结果）

| 常量 | 数值 | 出处 |
|---|---|---|
| kMinReachableTcpGraspZ | 0.005（米） | grasp_common.hpp:65 |
| kExecutionPlanningTime | 10.0（秒） | grasp_common.hpp:74 |
| kMoveGroupName | "arm" | grasp_common.hpp:20 |
| kEndEffectorLink | "grasp_tcp" | grasp_common.hpp:21 |
| kVisionCandidateMaxCenterError | 0.08（米） | grasp_common.hpp:64 |

---

## 3. CSV rotation 语义结论（Step 1 答案）

**CSV 中 r00..r22 是 GraspNet 原始抓取系旋转矩阵，坐标系为相机系，未经任何变换。**

追踪依据（inference_from_camera.py）：
1. grasp 数组 16 列布局（注释 698-700 行）：`[0]=score, [1..3]=width/height/depth, [4..12]=3x3 旋转(行主序), [13..15]=平移(camera 系)`。该数组直接来自 `gg.grasp_group_array`（GraspNet 网络输出 `pred_decode` → GraspGroup），本身就在相机系。
2. `select_object_grasp_targets`（719-735 行）：`rotation = grasp[4:13].reshape(3,3)` 与 `translation = grasp[13:16]` 都是**原值**；`CAMERA_TO_WORLD_ROTATION_R @ rotation @ GRASPNET_TO_PIPER_TCP_R` 只作用于函数内的局部副本 `world_tcp_rotation`（用于排序与诊断），**不回写** grasp 数组。
3. `write_frame_validation`（854-876 行）：CSV 写入 `rotation = grasp[4:13]`、`translation = grasp[13:16]`，即原值直写。
4. `camera_x/camera_y/camera_z` 同理为相机系原值（无变换）。
5. `world_tcp_rotation` 中的 rotation 来源：`grasp[4:13].reshape(3,3)`（原始相机系 GraspNet 旋转）。

GraspNet 旋转语义（graspnetAPI 约定）：R 的**列**是夹爪局部轴在相机系中的表达——局部 +X = 夹爪进深/接近方向，+Y = 夹爪宽度轴。

**数值验证**（2026-08-06 最后一次真机运行留下的 perception_grasps.csv）：
- 抽样 5 行的 r00..r22 均满足 det=1.000000、正交归一 → 确认为原始旋转矩阵。
- CSV 的 tcp_forward_z 列与 `(CAMERA_TO_WORLD_ROTATION_R @ R @ GRASPNET_TO_PIPER_TCP_R)[2,2]`（Python 矩阵）5/5 行完全吻合 → 证明感知侧诊断列用 Python 矩阵计算，且 CSV 旋转确为原始相机系值。

---

## 4. handeye 解析伪代码（Step 3 答案）

标定文件格式结论：
- 旋转表示：**3x3 旋转矩阵**（R_cam2base，det=+1 正交；另有冗余 4x4 齐次矩阵 T_cam2base，两者一致）。
- 平移单位：**米**（t_cam2base z=0.596m，符合相机架高）。
- eye-to-hand 语义：**给出的是 camera → base 变换**（相机固定于基座系，`p_base = T_cam2base @ p_cam`；description 字段明确 "T_cam2base"）。

```python
import numpy as np
import yaml

def load_handeye(path='camera_data/handeye_eye_to_hand.yaml'):
    with open(path) as f:
        d = yaml.safe_load(f)
    R = np.asarray(d['R_cam2base'], dtype=np.float64)   # (3,3) 旋转矩阵
    t = np.asarray(d['t_cam2base'], dtype=np.float64)   # (3,) 平移，米
    # 可选：T = np.asarray(d['T_cam2base']).reshape(4,4)  # 冗余，与 R,t 一致
    # 校验（可选）：assert abs(np.linalg.det(R) - 1.0) < 1e-6
    return R, t

R_cam2base, t_cam2base = load_handeye('camera_data/handeye_eye_to_hand.yaml')

# 位置：相机系 → 基座系
p_base = R_cam2base @ p_camera + t_cam2base

# 姿态：GraspNet 抓取系 → 基座系 TCP
R_base_tcp = R_cam2base @ R_grasp_cam @ GRASPNET_TO_PIPER_TCP_R

# 缺标定文件回退仿真常量（设计文档确认）：
#   R_cam2base ← CAMERA_TO_WORLD_ROTATION_R（姿态） / CAMERA_TO_WORLD_R（位置）
#   t_cam2base ← CAMERA_TO_WORLD_T
#   并显式日志告警
```

注意：仿真 RotationR 是 det=-1 的镜像矩阵，handeye 的 R_cam2base 是 det=+1 真旋转——两者性质不同，真机以 handeye 为准。

---

## 5. 仿真与真机变换的对应/差异（Step 2 答案）

### 5.1 仿真侧 CSV → 世界系（graspnet_perception_loader.cpp）

- 位置：`target.world = cameraToWorld(target.camera)`，即 `p_world = kCameraToWorldPositionR @ p_camera + kCameraToWorldT`（grasp_frame_transform.cpp:71-81）。
- 姿态：`target.orientation = cameraGraspRotationToWorldTcp(target.camera_rotation)`，即 `kCameraToWorldRotationR @ R_grasp_cam @ kGraspNetToPiperTcpR` → Shepperd 四元数（grasp_frame_transform.cpp:158-176）。
- 语义："world" = Gazebo 世界系 = 机械臂基座所在系（Piper 臂置于 world 原点）。这与真机的"基座系（base_link）"一一对应。
- 姿态变体：当前仅 calib_18 一种（`graspNetTcpOrientationVariants`，grasp_frame_transform.cpp:180-200）。
- 候选有效性：`readValidatedVisionObjectGraspCandidates`（pick_node.hpp:644）要求 has_grasp_pose 且 count ≥ kMinPerceptionPoints(80)。
- 世界系抓取位置 → MoveIt 目标（graspnet_6d_grasp_planner.cpp:146-150）：x/y 用候选世界坐标，z 按 1.5 节公式取 max 保护。

### 5.2 对应关系表

| 环节 | 仿真 | 真机（grasp_executor 待实现） |
|---|---|---|
| 数据源 | shared_camera_data/perception_grasps.csv | 同左（真机 D435i 桥接产出） |
| CSV 旋转 | 相机系原值 r00..r22 | 同左（格式完全一致） |
| 位置变换 | kCameraToWorldPositionR + kCameraToWorldT（三点标定） | R_cam2base + t_cam2base（handeye，eye-to-hand） |
| 姿态变换 | kCameraToWorldRotationR @ R @ kGraspNetToPiperTcpR | R_cam2base @ R @ GRASPNET_TO_PIPER_TCP_R（映射矩阵见疑点 A） |
| 目标系 | Gazebo world | 机械臂基座系（base_link） |
| 门控 | 与 Gazebo 真值比对（kVisionCandidateMaxCenterError=0.08） | 无真值，改自洽检查（设计文档 5.3） |
| z 保护 | max(..., kMinReachableTcpGraspZ=0.005) | 同左 |
| 执行 | MoveIt 4 段预检 + 执行 | MoveIt（MoveGroupInterface Python API） |

### 5.3 差异说明

1. **外参数值完全不同**：仿真常量是"三点反求"标定（0.072, 0.021, 0.548），handeye 是完整 eye-to-hand 标定（0.484, -0.0446, 0.596）。设计文档确认：标定文件存在以 handeye 为准，缺失回退仿真常量并告警。
2. **矩阵性质不同**：仿真 RotationR det=-1（镜像，历史 IK 可解性修复）；handeye det=+1（真旋转）。
3. **映射矩阵不同**（见疑点 A）：仿真执行侧用 C++ kGraspNetToPiperTcpR（calib_18），感知诊断列用 Python GRASPNET_TO_PIPER_TCP_R。
4. **无 Gazebo 真值**：真机候选筛选用 CSV 自带 object_center 自洽检查替代。

---

## 6. 疑点（编码前必须决策）

### 疑点 A（最重要）：Python GRASPNET_TO_PIPER_TCP_R ≠ C++ kGraspNetToPiperTcpR

| | Python（inference_from_camera.py:156） | C++ calib_18（grasp_common.hpp:80） |
|---|---|---|
| 矩阵（行） | [[0,0,1],[0,1,0],[-1,0,0]] | [[0,-1,0],[0,0,-1],[1,0,0]] |
| 列语义 | +X→-Z, +Y→+Y, +Z→+X | +X→+Z, +Y→-X, +Z→-Y |
| det | +1 | +1 |

- Python 注释声称"与 C++ 同一份常量的镜像"，**数值上并不相同**（也不是真正的镜像关系）。
- 甚至 Python 注释本身（"+X→+Z、+Z→-X"）与其矩阵列语义（+X→-Z、+Z→+X）矛盾。
- CSV 的 tcp_forward_z 诊断列按 Python 矩阵计算（实测吻合）；仿真**实际执行**姿态按 C++ calib_18 计算。
- 两矩阵产生的 TCP 姿态相差约 180°（绕某轴），是执行级差异，不是数值误差。
- **建议**：grasp_transform.py 采用仿真执行侧已验证的 calib_18（kGraspNetToPiperTcpR），并保持感知侧不变（tcp_forward_z 仅是诊断列）；在 Task 3 实现时显式注明选择，真机验证阶段实测确认。若采用 Python 矩阵则必须与 C++ 语义一并核对。

### 疑点 B：设计文档称"CSV 25 列"，实际 24 列

- 实测 perception_grasps.csv 表头与数据行均为 **24 列**（index 0-23，末列 source_dist_xy）。
- C++ 解析要求 fields.size() ≥ 23，读取 index 0..22，忽略 source_dist_xy——与 24 列兼容。
- grasp_transform.py 的列索引应以上表为准（r00..r22 = index 9..17）。

### 疑点 C：两套相机外参矩阵不可混用

- CAMERA_TO_WORLD_R（中间行 -1，det=+1，位置变换用）与 CAMERA_TO_WORLD_ROTATION_R（中间行 +1，det=-1，姿态变换用）是互为镜像的**两个矩阵**，注释明确"不可混用"（grasp_common.hpp:35-36）。位置永远用 PositionR，姿态永远用 RotationR（或 handeye 对应矩阵）。

### 疑点 D：真机尚无一次执行验证

- 设计文档明确执行侧"完全没有代码"，真机从未按任一映射矩阵实际执行过抓取。calib_18 是仿真收敛方案，真机适用性（含手性/镜像问题）只能在阶段 2/3 实测确认。
