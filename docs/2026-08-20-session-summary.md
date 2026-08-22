# 2026-08-20 手眼 TF 与 GraspNet→Piper TCP 朝向审计日志

> 日期：2026-08-20（Asia/Shanghai）  
> 本轮性质：只读代码、URDF、历史日志和冻结候选数据审计；未修改运行源码、YAML、URDF、MoveIt 配置或真机参数，未发送机械臂运动命令。  
> 本轮目标：确认 Eye-to-Hand 输出的真实帧语义，重新推导 GraspNet 抓取坐标系到 Piper 实际 TCP/link6 的固定旋转，解释历史“末端偏、接近水平、抓空”现象，并形成后续修改计划。

## 1. 最终结论摘要

当前问题不是 GraspNet 网络训练失败，也不是 Eye-to-Hand 的 AX=XB 求解方向写反；本质是**感知与执行接口中的坐标系契约错误**，包含两个独立根因：

1. **相机 TF 帧语义错误**：手眼标定求得的是 `camera_color_optical_frame → base_link`，当前静态 TF 发布器却把同一数值标成 `camera_link → base_link`。RealSense 驱动随后还会追加 `camera_link → camera_color_frame → camera_color_optical_frame`，使 ROS TF 消费者多乘一次相机内部光学帧变换。
2. **GraspNet→Piper TCP 轴映射错误**：当前执行矩阵把 GraspNet `+X` 接近方向映射到 Piper 手指开合轴 `link7 +Z = -link6 Y`，而不是 Piper 工具前进轴 `link6/TCP +Z`。对典型顶部抓取，GraspNet `+X` 向下，其正交的 `+Z` 接近水平；当前代码又令 `TCP +Z = GraspNet +Z`，因此执行末端天然容易接近水平。

当前抓取中心位置公式与旋转公式彼此分离，所以可能同时出现：

- 项目计算的 TCP 原点 XYZ 基本在目标处；
- link6/腕部因 0.1358 m 逆偏移沿近水平 TCP Z 轴放置，看起来明显偏到物体侧面；
- 夹爪的实际工具前进轴与 GraspNet 预测接近轴相差约 90°；
- 手指开合轴反而与 GraspNet 接近轴重合。

## 2. 审计范围和证据

### 2.1 当前实现

- `graspnet-baseline/utils/loss_utils.py`
  - `batch_towards` 被写入旋转矩阵第一列，证明 GraspNet 局部 `+X` 是 approaching/depth 方向。
- `graspnet-baseline/utils/collision_detector.py`
  - 局部 X 用于 finger length/depth，Y 用于 width，Z 用于 height，进一步确认 GraspNet 抓取器轴语义。
- `graspnet-baseline/inference_from_camera.py`
  - 推理侧已有 `[[0,0,1],[0,1,0],[-1,0,0]]` 固定映射，其注释声明 `GraspNet +X → Piper TCP +Z`。
- `src/real_piper_perception/real_piper_perception/grasp_transform.py`
  - 当前执行侧使用不同矩阵 `[[0,-1,0],[1,0,0],[0,0,1]]`。
- `src/real_piper_perception/real_piper_perception/grasp_precheck.py`
  - 当前 approach 仍固定为 `grasp + base-Z clearance`，未沿 TCP 工具轴生成。
- `src/real_piper_perception/real_piper_perception/grasp_executor.py`
  - 当前方向排序使用 `r[1,2]`，它只是 TCP Z 轴的 base-Y 分量，不是“开口朝下”或“接近朝下”的物理量。
- `src/piper_ros/src/piper_description/urdf/piper_description.urdf`
  - `link6 → gripper_base` 固定变换为单位旋转；joint7/joint8 原点位于 link6 局部 `+Z`；joint7 为 prismatic 且安装 `Rx(+90°)`，所以 `link7 +Z = -link6 Y` 是手指滑移/开合方向。
- `src/real_piper_perception/real_piper_perception/handeye_tf_publisher.py`
  - 当前直接把 `R_cam2base/t_cam2base` 发布成 `base_link → camera_link`。
- `<calibration-repository>/hand_eye_single_eye_to_hand.py`
  - `solvePnP` 使用 RealSense 彩色图像，输出标定板在 OpenCV 彩色相机光学坐标系下的位姿；Eye-to-Hand 求解输出真实语义为 `T_base_color_optical`。

### 2.2 历史工作与实测记录

本轮复核了与姿态、TCP、TF 和真机误差直接相关的记录，包括：

- `docs/grasp_executor_transform_notes.md`
- `.superpowers/sdd/grasp-executor/task-1-report.md`
- `.superpowers/sdd/grasp-executor/task-3-brief.md`
- `.superpowers/sdd/grasp-executor/task-3-report.md`
- `docs/2026-08-07-session-summary.md`
- `docs/2026-08-08-session-summary.md`
- `docs/2026-08-10-session-summary.md`
- `docs/2026-08-14-session-summary.md`
- `docs/real_grasp_error_investigation/hypothesis-analysis-report.md`
- `docs/real_grasp_error_investigation/10-authoritative-calibration-error-validation-report.md`
- `docs/real_grasp_error_investigation/08-grasp-orientation-planning-modification.md`
- `docs/real_grasp_error_investigation/11-calibration-and-safe-grasp-planning.md`
- `docs/real_grasp_error_investigation/log/hypothesis_scanonly.json`
- `docs/real_grasp_error_investigation/log/grasp_axis_semantics.json`
- `docs/real_grasp_error_investigation/log/grasp_orientation_safety.json`
- `docs/real_grasp_error_investigation/log/task11_live_scan_only_20260818.json`
- `docs/real_grasp_error_investigation/log/real_feedback_endpoint_validation.json`
- `shared_camera_data/perception_grasps.csv`
- `shared_camera_data/precheck_results.json`

不相关的 CMake/colcon 编译日志未作为坐标语义证据。

## 3. Eye-to-Hand 关系复核

记：

- `B`：`base_link`
- `G`：机器人 SDK 末端/夹爪刚体帧
- `C`：RealSense 彩色相机 OpenCV 光学帧
- `P`：ChArUco 标定板

Eye-to-Hand 固定关系：

```text
^B T_G · ^G T_P = ^B T_C · ^C T_P
```

标定程序中：

- `solvePnP` 给出 `^C T_P`；
- Piper SDK 给出 `^B T_G`；
- 程序先反演机器人输入，再在 `_solve_ax_xb()` 中构造相对运动，最终求得 `X = ^B T_C`。

对该实现做合成位姿数值验证，恢复误差约为：

```text
translation error = 4.1e-16 m
rotation error    = 2.8e-14 deg
```

当前 `camera_data/handeye_eye_to_hand.yaml` 旋转矩阵检查结果：

```text
det(R)             = 1.0000000000000007
orthogonality error= 1.17e-15
T 与 R/t           = 完全一致
```

因此：

- AX=XB 的求解方向正确；
- SDK 欧拉角 `Rz @ Ry @ Rx` 与 Piper ROS 驱动 `from_euler('xyz')` 一致；
- 当前问题不应归因于“手眼矩阵乘反”；
- 但手眼数值的绝对毫米精度仍缺少已知 base 真值点和原始采样残差，不能仅凭正交性证明。

## 4. 相机 TF 错误根源

### 4.1 标定得到的真实帧

标定程序直接获取 RealSense 彩色流，并将像素送入 OpenCV `solvePnP`。因此 `T_cam2base` 中的 `cam` 是：

```text
camera_color_optical_frame
X：图像向右
Y：图像向下
Z：镜头向前
```

不是采用 REP-103 机器人轴语义的 `camera_link`。

### 4.2 当前错误发布

当前发布器直接执行：

```text
header.frame_id = base_link
child_frame_id  = camera_link
transform       = T_base_color_optical
```

而 RealSense 驱动还会发布：

```text
camera_link
  → camera_color_frame
    → camera_color_optical_frame
```

于是 TF 查询得到的 `base_link → camera_color_optical_frame` 多乘了一次相机内部光学帧变换。

### 4.3 正确关系

设 RealSense 驱动提供：

```text
^camera_link T_color_optical
```

则应该发布：

```text
^base T_camera_link
  = ^base T_color_optical
    · inverse(^camera_link T_color_optical)
```

不能在 RealSense TF 开启时再直接发布 `base_link → camera_color_optical_frame`，否则同一 optical child 会出现两个父节点。

### 4.4 当前影响范围

当前抓取执行器没有通过 tf2 查询相机到 base，而是直接读取 handeye YAML 并对 GraspNet 光学系坐标左乘，所以该 TF 帧名错误暂时不改变当前数值执行链；它会影响：

- RViz 中的相机姿态；
- tf2 查询结果；
- 使用 `camera_link`/`camera_color_optical_frame` 的其他节点；
- 以后若把执行侧改成 tf2 后的抓取位姿。

## 5. GraspNet→Piper TCP 正确旋转的独立推导

### 5.1 GraspNet 抓取器轴

令：

```text
R_base_grasp = [Xg, Yg, Zg]
```

物理语义：

```text
Xg：接近/进深方向
Yg：两指宽度方向
Zg：夹爪高度方向
```

### 5.2 Piper 工具轴

根据 URDF：

```text
TCP/link6 +Z：腕部朝夹爪/物体的工具前进方向
TCP/link6 ±Y：两指开合方向
link7 +Z = -link6 Y：joint7 手指滑移方向
```

因此物理约束必须满足：

```text
TCP +Z = GraspNet +X
TCP +Y = GraspNet +Y       # 宽度轴正负号可按双指对称性选择
TCP +X = -GraspNet +Z      # 保证右手系、det=+1
```

在公式：

```text
R_base_tcp = R_base_grasp @ M
```

中，右乘矩阵 `M` 的三列分别是 TCP X/Y/Z 在 GraspNet 帧中的表达。因此：

```python
M_expected = np.array([
    [ 0.0, 0.0, 1.0],
    [ 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
```

列检查：

```text
col(0) = [0,0,-1] → TCP X = -Zg
col(1) = [0,1, 0] → TCP Y =  Yg
col(2) = [1,0, 0] → TCP Z =  Xg
```

该矩阵恰好与推理侧现有常量一致。

## 6. 当前执行矩阵为何错误

当前执行侧：

```python
M_current = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])
```

右乘后的物理含义：

```text
TCP X =  Yg
TCP Y = -Xg
TCP Z =  Zg
```

又因为：

```text
link7 Z = -TCP Y
```

所以：

```text
link7 Z = Xg
```

即当前代码把 GraspNet 接近轴放到了 joint7 手指开合轴上。

对于顶部抓取，`Xg` 向下，而 `Zg ⟂ Xg`；当前又令 `TCP Z = Zg`，所以 TCP 工具前进轴必然趋向水平。这不是偶然候选或小角度误差，而是固定轴置换的直接结果。

## 7. 历史错误根源追踪

### 7.1 右乘矩阵解释方向错误

2026-08-06 调研已发现推理侧和仿真 C++ 的映射矩阵不同，但旧文档把 `R_tcp = R_grasp @ M` 中的 `M` 当成“主动把 GraspNet 向量变到 TCP”的矩阵，按错误方向解释了行列语义。

正确解释应是：

```text
M 的第 j 列 = TCP 第 j 个轴在 GraspNet 帧中的表达
```

这一误读导致旧文档认为推理侧矩阵注释与数值矛盾；按正确的右乘/换基语义，它的核心关系 `TCP +Z = GraspNet +X` 实际一致。

### 7.2 把手指开合轴误称为接近轴

2026-08-07 的矩阵调整目标是让“夹爪开口朝下”。当时确认了：

```text
link7 Z = -link6 Y
```

但随后把 `link7 Z` 称为“夹爪轴线/从上方接近方向”。实际 joint7 是 prismatic 手指开合关节，`link7 Z` 是开合滑移轴。

`scripts/sdk_mapping_validation.py` 正确验证了 `link7 Z = -link6 Y`，但测试目标随后错误地要求：

```text
link7 Z = GraspNet approaching X
```

因此单元测试和数值验证虽然通过，只证明了代码忠实实现了错误的物理假设。

### 7.3 用 IK 可达性裁决固定坐标关系

历史上 `M_expected` 曾被试用，但因为 ssik 解少/无解以及当时的位置异常被回退。这里混淆了两个问题：

```text
坐标轴关系是否物理正确：由 GraspNet 定义、URDF 和真实工具安装决定
给定姿态是否可达：由候选、机械臂限位、起始状态和 IK/规划器决定
```

固定工具变换不能为了获得 IK 解而修改。正确姿态不可达时，应换候选、拒绝候选、改善 IK/规划或调整相机/物体布置。

### 7.4 测试只检查代数自洽，没有检查物理不变量

旧测试主要检查：

- 矩阵正交；
- `det=+1`；
- 函数输出等于预设常量；
- TCP→link6→TCP 往返一致。

这些测试不能判断轴语义是否正确。错误矩阵同样可以正交、det=1、往返自洽。

必须增加物理不变量：

```text
dot(TCP_forward, GraspNet_approach) ≈ 1
abs(dot(finger_opening, GraspNet_width)) ≈ 1
```

### 7.5 历史日志已经观测到问题，但未闭环

冻结候选 `[44]` 的历史数据：

```text
TCP/link6 +Z = [0.724, 0.669, -0.171]
down component = 0.171
距向下方向约 80.2°，接近水平

opening axis = -TCP Y = [0.247, -0.482, -0.840]
opening down component = 0.840

固定 base-Z approach 与 TCP +Z 夹角约 80.2°
```

2026-08-18 的安全门实验中，新鲜 12 个候选全部被拒：

```text
approach_down range = -0.6424 .. 0.5308
tilt range          = 57.94 .. 129.97 deg
opening_down range  = 0.7649 .. 0.9718
```

这与“开合轴普遍向下、实际 TCP 接近轴普遍接近水平”的错误映射完全一致。

但当时 `grasp_axis_semantics.json` 明确记录：

```text
validation_scope   = no robot FK call
real_fk_confirmation = pending
```

当前 2026-08-20 源码中姿态门已不存在，approach 又恢复为固定 base-Z 偏移，因此该问题在当前版本仍未解决。

## 8. 对当前现象的影响判定

| 现象 | 与本次轴映射错误的关系 | 置信度 |
|---|---|---:|
| TCP/link6 工具前进方向总是偏斜或接近水平 | 固定轴映射的直接结果 | 高 |
| 手指开合方向反而朝下 | 固定轴映射的直接结果 | 高 |
| approach 下插路径与工具前进轴不共线 | 错误映射 + 当前固定 base-Z approach 共同导致 | 高 |
| link6/腕部看起来偏到物体侧面 | TCP→link6 的 0.1358 m 偏移沿近水平轴施加 | 高 |
| 项目计算的 TCP 原点 XYZ 仍在候选抓取中心 | 位置与旋转分别计算，正反偏移使用同一姿态，可能保持自洽 | 高 |
| TCP/真实夹持中心固定偏 2–3 cm | 不能只归因于映射；还包含 MoveIt、内参/畸变、感知稳定性、TCP 原点定义 | 中低 |
| 历史多次 `grasp_empty` | 错误姿态是重要原因，但不是唯一已证实原因 | 中高 |

冻结候选 `[44]` 的命令记录：

```text
command TCP   = [585.4,  82.5, 35.6] mm
command link6 = [487.1,  -8.3, 58.8] mm
```

两者水平相差约 134 mm。这不是 TCP 原点必然偏离目标，而是当前近水平 TCP Z 轴使腕部/link6 被安排在目标侧方，能够解释目视“末端偏且接近水平”。

## 9. 与朝向问题分离的其他误差源

以下问题不能被 TCP 轴映射修复掩盖，必须分批验证：

1. 历史 MoveIt 位姿约束产生 `5–14 mm` 随机终点误差。
2. 旧运行链曾使用与手眼标定不同的相机内参，冻结候选在 0.741 m 处反投影差约 `29.3 mm`；桥接已改为优先 `calibration.yaml`，但畸变是否真正用于反投影仍需核查。
3. 历史 GraspNet 候选跨帧位置波动达到数厘米。
4. `grasp_z_offset=0.02 m` 会主动改变最终抓取高度。
5. `0.1358 m` 是 URDF joint7/joint8 原点距离；它是否等于真实指尖夹持中心 TCP 长度，仍是独立几何定义问题。本次轴向推导不依赖该长度数值。

## 10. 计划修改内容

本节仅记录计划，今日未实施。

### P0：冻结基线与安全边界

- 冻结一帧 CSV、handeye YAML、calibration YAML 和当前源码 hash/mtime。
- 默认只运行离线/scan-only/plan-only。
- 真机先 approach-only，必须再次获得用户明确授权；禁止自动闭合和自动重试。
- 不同时修改轴映射、TCP 长度、手眼矩阵和 MoveIt 容差，保持单变量 A/B。

### P1：修正执行侧固定轴映射

文件：

```text
src/real_piper_perception/real_piper_perception/grasp_transform.py
```

修改前：

```python
GRASPNET_TO_PIPER_TCP_R = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])
```

计划修改为：

```python
GRASPNET_TO_PIPER_TCP_R = np.array([
    [ 0.0, 0.0, 1.0],
    [ 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)
```

保持乘法顺序：

```text
R_base_tcp = R_base_color_optical @ R_color_optical_grasp @ R_grasp_tcp
```

不修改位置公式：

```text
p_base = R_base_color_optical @ p_camera + t_base_color_optical
```

### P2：统一推理侧和执行侧常量

涉及：

```text
graspnet-baseline/inference_from_camera.py
src/real_piper_perception/real_piper_perception/grasp_transform.py
```

计划：

- 删除两侧独立硬编码或建立单一共享配置/生成源；
- 启动时打印矩阵、det 和轴语义；
- 对两侧常量做严格一致性检查；
- 不再允许 CSV 诊断列使用一套矩阵、执行器使用另一套矩阵。

### P3：修正方向排序和安全门

文件：

```text
src/real_piper_perception/real_piper_perception/grasp_executor.py
```

删除当前：

```python
down = float(r[1, 2])
```

正确物理量：

```python
tcp_forward = r_tcp[:, 2]
base_down = np.array([0.0, 0.0, -1.0])
approach_down = float(tcp_forward @ base_down)   # 等价于 -r_tcp[2,2]
```

计划恢复并完善 fail-closed 姿态门：

- TCP 接近轴接近水平或向上时拒绝；
- 阈值参数化，不散落硬编码；
- 记录 `approach_down`、倾角、开合轴和拒绝原因；
- 不再以“开口朝下”替代“工具接近方向安全”。

### P4：沿 TCP 接近轴生成 approach

文件：

```text
src/real_piper_perception/real_piper_perception/grasp_precheck.py
```

修改前：

```text
p_approach = p_grasp + [0, 0, approach_clearance]
```

计划修改为：

```text
p_approach = p_grasp - R_base_tcp[:,2] * approach_clearance
```

要求：

- `approach → grasp` 位移必须与 TCP `+Z` 同向；
- 垂直向下候选退化为原有 base-Z 上移；
- lift 仍可沿 base +Z，接近与抬升语义分开。

### P5：修正 handeye 静态 TF 发布

涉及：

```text
camera_data/handeye_eye_to_hand.yaml
src/real_piper_perception/real_piper_perception/handeye_tf_publisher.py
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
src/real_piper_perception/test/test_handeye_tf_publisher.py
```

计划：

1. 在 YAML 中明确记录：

   ```yaml
   parent_frame: base_link
   calibrated_camera_frame: camera_color_optical_frame
   transform_semantic: T_base_color_optical
   ```

2. 启动后从 RealSense TF 获取 `T_camera_link_color_optical`。
3. 计算：

   ```text
   T_base_camera_link = T_base_color_optical
                        @ inverse(T_camera_link_color_optical)
   ```

4. 只发布 `base_link → camera_link`，保留 RealSense 内部 TF 树。
5. 抓取数值链继续使用 `T_base_color_optical`，不能误换成 `T_base_camera_link`。

### P6：重写轴语义验证脚本和测试

涉及：

```text
scripts/sdk_mapping_validation.py
src/real_piper_perception/test/test_grasp_transform.py
src/real_piper_perception/test/test_grasp_precheck.py
src/real_piper_perception/test/test_grasp_execute_units.py
```

必须增加以下物理断言：

```python
R_base_grasp = R_base_camera @ R_camera_grasp
R_base_tcp = R_base_grasp @ M_expected

assert np.dot(R_base_tcp[:, 2], R_base_grasp[:, 0]) > 1.0 - 1e-6
assert abs(np.dot(-R_base_tcp[:, 1], R_base_grasp[:, 1])) > 1.0 - 1e-6
```

还要验证：

- 水平/向上 TCP 接近候选被拒；
- 垂直向下候选通过；
- approach 路径与 TCP +Z 夹角小于 1°；
- 姿态门拒绝时不调用 MoveIt；
- `T_tcp → T_link6 → T_tcp` 往返保持一致；
- `T_base_camera_link @ T_camera_link_color_optical == T_base_color_optical`；
- 冻结候选 `[44]` 修改前 `dot≈0`，修改后 `dot≈1`。

### P7：规划和真机验证顺序

1. 离线冻结候选 A/B，只改变 `M`。
2. scan-only 记录每个候选的 `Xg/Yg/Zg`、TCP axes、倾角、link6 位置。
3. plan-only 比较正确姿态下的 IK/MoveIt 可达率；无解视为候选不可达，不回改矩阵。
4. 用户授权后进行高空姿态验证，不接近物体。
5. 再进行 approach-only，确认腕部位于物体上方/合理侧上方，路径沿工具轴。
6. 再次授权后执行 grasp 位但不闭合。
7. 最后才允许一次闭合抓取；禁止自动重试。

## 11. 修改后的验收标准

### 11.1 数学/语义

```text
det(R_grasp_tcp) = +1
R^T R = I
dot(TCP +Z, GraspNet +X) ≥ 0.999999
abs(dot(finger opening, GraspNet +Y)) ≥ 0.999999
```

### 11.2 路径

```text
angle(p_grasp - p_approach, TCP +Z) < 1 deg
```

### 11.3 TF

```text
T_base_camera_link · T_camera_link_color_optical
≈ T_base_color_optical
```

### 11.4 运行安全

- 不再执行 TCP 接近轴近水平或向上的候选；
- 正确映射导致 IK 无解时明确报告 `unreachable_orientation`，不得自动切换错误映射；
- 日志同时记录 TCP forward、finger opening、GraspNet approach/width，禁止使用含糊的“夹爪轴线”。

## 12. 尚未确认的事项

1. 真实硬件 TCP 是否与项目定义完全同向于 link6，虽然 URDF 和夹爪几何支持该结论，仍应通过一次安全的高空姿态/FK/目视轴验证闭环。
2. `0.1358 m` 是 joint7/joint8 指根原点，不等同于已测量的指尖夹持中心；TCP 长度问题不能与轴映射同时修改。
3. 手眼标定绝对精度缺少已知 base 真值点验证。
4. calibration 非零畸变是否在当前 GraspNet 反投影中真正使用，需要单独审计。
5. 正确姿态的 Piper 可达率可能降低；这属于候选/规划问题，不构成回退错误轴映射的理由。

## 13. 今日文件变更

今日只新增/更新文档：

```text
新增：docs/2026-08-20-session-summary.md
更新：docs/real_grasp_error_investigation/README.md（增加本日志索引）
```

运行源码、相机标定、手眼标定、URDF、MoveIt 和真机状态均未修改。
