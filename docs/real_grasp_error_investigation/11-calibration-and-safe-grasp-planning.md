# Calibration 内参与安全抓取规划联合修改任务

## 0. 当前状态与执行边界

**状态：已获批准并完成源码实现与离线验证；真机执行仍需单独批准。**

本文件先作为实施计划，现已按用户“开始执行”完成源码实现。没有修改标定 YAML、URDF、手眼文件或原始 RealSense 话题；没有启动 ROS 真机抓取，也没有发送机械臂运动命令。

用户已给出当前方向：

1. `camera_data/calibration.yaml` 的内参在实际抓取中比 RealSense 运行时 `CameraInfo` 更好用、更准确；后续感知链路应使用该文件内容。
2. 当前抓取规划可能让末端接近水平后继续沿下降路径运动，导致关节受力异常甚至 Piper 强行断电；规划必须在执行前阻止这类姿态/路径。

本轮不改变：

- `camera_data/handeye_eye_to_hand.yaml`；
- URDF 权威 TCP `0.1358 m`；
- GraspNet 模型、权重和输出格式；
- 未经用户批准的真机抓取参数。

## 1. 任务 A：统一使用 calibration.yaml 内参

### 1.1 目标

使 GraspNet 点云反投影和候选坐标生成明确使用：

```text
./camera_data/calibration.yaml
```

其中：

- `camera_matrix` 作为 `K`；
- `dist_coeff` 作为 `D`；
- 必须记录文件 SHA256、分辨率和标定重投影误差；
- 不再让 `meta.mat` 无提示地使用运行时 RealSense `CameraInfo.K`。

### 1.2 推荐实现边界

只修改本项目桥接层，不修改 RealSense 驱动源码，也不覆盖原始话题：

```text
/camera/camera/color/camera_info
```

该原始话题仍保留设备运行时信息，便于诊断。推荐修改：

```text
src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
```

设计要求：

1. 增加 `calibration_yaml` 参数，默认指向 `camera_data/calibration.yaml`。
2. 节点启动时读取并校验 `camera_matrix`、`dist_coeff`、分辨率字段和矩阵形状。
3. `meta.mat` 使用 calibration 文件中的 K，而不是订阅到的运行时 K。
4. `camera_info.txt` 同时记录：
   - `source: calibration.yaml`；
   - calibration K/D；
   - 原始 ROS CameraInfo K/D；
   - 使用的分辨率和时间戳。
5. 如需向 ROS 其他节点提供校准内参，只新增独立话题，例如：

   ```text
   /camera/camera/calibrated/color/camera_info
   ```

   不得向原始 `camera_info` 话题追加第二个发布者，避免订阅者随机收到两套内参。
6. 保持 `factor_depth=1000.0` 的语义不变。
7. 加入 `use_calibration_intrinsics` 或等价开关，便于回滚；未经批准不得删除运行时内参诊断信息。

### 1.3 必须先解决的畸变与对齐问题

不能只替换 K 而忽略 D。Claude Code 必须先确认：

1. `color/image_raw` 是否为未去畸变彩色图；
2. `aligned_depth_to_color/image_raw` 的像素坐标由哪套内参/外参生成；
3. `graspnet-baseline/inference_from_camera.py` 和 `utils/data_utils.py` 是否使用 D；
4. calibration 的 K/D 是否与当前 `640x480` 彩色流一致。

最终必须选择并记录一种完整策略：

- 使用 calibration K/D 对图像和深度坐标统一去畸变后反投影；或
- 证明当前对齐图像已经处于可直接使用的投影坐标系，并使用与该坐标系一致的有效 K；或
- 暂停修改并标记 `blocked`，不得仅凭 K 数值更接近实测就上线。

### 1.4 任务 A 最小读取白名单

Claude Code 只允许读取：

```text
docs/real_grasp_error_investigation/11-calibration-and-safe-grasp-planning.md
camera_data/calibration.yaml
camera_data/handeye_eye_to_hand.yaml
src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
graspnet-baseline/inference_from_camera.py
graspnet-baseline/utils/data_utils.py
src/real_piper_perception/test/test_handeye_tf_publisher.py
```

如需新增桥接测试，只读取与该测试直接相关的 setup/package 文件。禁止扫描：

```text
build/
install/
src/realsense-ros/
graspnet-baseline/models/
graspnet-baseline/pointnet2/
所有历史图像、旧 CSV、旧抓取结果和无关日志
```

### 1.5 任务 A 测试顺序

1. 静态单元测试：YAML 读取、矩阵形状、D 长度、缺失文件和非法数值。
2. 离线一致性测试：新建空运行目录，生成一份 `meta.mat`，确认 K 与 YAML 数值一致。
3. ROS 只读测试：启动相机桥接但不启动抓取执行器，检查 `camera_info.txt`、`meta.mat` 和输入文件 hash。
4. 新鲜帧推理测试：只使用当前运行目录内新生成的 color/depth/meta，生成新 CSV。
5. 运行时回归：确认手眼 TF、深度比例和候选 CSV 格式未改变。

任务 A 在以下条件全部满足前不得进入真机：

- `meta.mat` 的 K 与 calibration YAML 一致；
- 来源记录明确；
- D/去畸变/深度对齐策略明确；
- 新鲜数据目录没有历史文件；
- 原始 RealSense CameraInfo 与校准 CameraInfo 未发生话题冲突。

### 1.6 任务 A 必须交付的日志

```text
docs/real_grasp_error_investigation/log/calibration_intrinsics_authority.json
docs/real_grasp_error_investigation/log/calibration_intrinsics_runtime.log
```

JSON 至少包含：

```json
{
  "status": "done|blocked|failed",
  "calibration_yaml": "...",
  "calibration_sha256": "...",
  "K_calibration": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
  "D_calibration": [],
  "K_runtime_camera_info": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
  "K_meta_used": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
  "distortion_policy": "...",
  "depth_alignment_policy": "...",
  "color_topic": "...",
  "depth_topic": "...",
  "camera_info_topic": "...",
  "fresh_run_dir": "...",
  "source_files_sha256": {},
  "conclusion": "..."
}
```

## 2. 任务 B：重新设计末端姿态与下降路径安全策略

### 2.1 目标

禁止出现以下执行序列：

```text
末端接近水平
→ 仍沿基座 Z 方向下降
→ 关节接近极限/发生大力矩或碰撞
→ Piper 关节强行断电
```

安全原则是“预检失败即不运动”，不能依靠真机执行中遇到异常后再停止。

### 2.2 必须先固定的轴语义

只允许基于以下文件确认轴方向：

```text
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_execute.py
src/real_piper_perception/real_piper_perception/grasp_executor.py
src/real_piper_perception/real_piper_perception/grasp_candidates.py
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
```

必须用 URDF + FK 确认：

- TCP/末端接近轴；
- link6 到 TCP 的 `0.1358 m` 折算方向；
- 夹爪开口轴；
- 基座竖直向下轴；
- 旋转矩阵使用行还是列表示轴。

不得继续把未经验证的单个矩阵元素直接命名为“开口朝下”。

### 2.3 推荐安全规划策略

#### B1：执行前姿态安全门控

对每个候选，在 MoveIt/IK 预检之前计算：

```text
approach_axis_base
opening_axis_base
approach_down = dot(approach_axis_base, [0, 0, -1])
approach_tilt_deg = acos(clamp(approach_down, -1, 1))
```

初始阈值只作为参数候选，不得直接视为最终值：

```text
min_approach_down = 0.70
max_approach_tilt_deg = 45 deg
```

门控规则：

- 接近方向接近水平，直接拒绝；
- 需要沿基座 Z 下降但 TCP 接近轴不朝下，直接拒绝；
- 旋转矩阵无效、轴向量不归一化或 TCP/link6 折算失败，直接拒绝；
- 方向门控失败时不得调用 MoveIt 执行接口；
- 所有候选均失败时返回 `no_safe_candidate`，机械臂保持当前位置或安全零位，不自动重试。

#### B2：approach 路径与末端轴一致

不能继续对所有姿态统一使用“基座 Z 上移/下降”的 approach 逻辑。根据已验证的轴语义，计算：

```text
p_approach = p_grasp - approach_axis_base * approach_clearance
```

并验证：

- approach 到 grasp 的位移方向与末端插入轴夹角小于 1°；
- 路径不包含与末端姿态不匹配的强制竖直下降；
- 仍使用 TCP `0.1358 m` 折算；
- approach、grasp、lift 三个位姿分别通过 IK、碰撞和关节限位检查。

#### B3：水平候选的处理方式

默认采取 fail-closed：

1. 优先从 GraspNet 候选中选择安全的上方/斜向接近姿态；
2. 不对原始姿态直接裁剪 Euler 角；
3. 只有在目标几何允许、接触区域不变且 approach 轴保持不变时，才允许生成绕 approach 轴的有限姿态变体；
4. 变体仍需重新执行 TCP 折算、IK、碰撞和三段预检；
5. 没有安全候选时禁止为了“完成一次抓取”强行执行水平下降。

#### B4：运动与故障保护

- 真机速度默认不超过 `0.1`；
- 先 plan-only，再 approach-only，再 grasp；
- 禁止在规划失败后自动降低约束并继续执行；
- 记录关节限位、规划失败、碰撞、CAN/SDK 错误；
- 发生 `can0 loss`、SDK 发送失败或反馈中断时立即停止后续阶段，不得自动重试；
- 每次实验结束后记录是否回到 zero，不能把“MoveIt FakeSystem 成功”当作真实 Piper 反馈成功。

### 2.4 任务 B 最小读取白名单

```text
docs/real_grasp_error_investigation/08-grasp-orientation-planning-modification.md
docs/real_grasp_error_investigation/07-moveit-runtime-consistency-plan.md
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_execute.py
src/real_piper_perception/real_piper_perception/grasp_executor.py
src/real_piper_perception/real_piper_perception/grasp_candidates.py
src/real_piper_perception/real_piper_perception/moveit_adapter.py
src/real_piper_perception/test/test_grasp_precheck.py
src/real_piper_perception/test/test_grasp_execute_units.py
src/real_piper_perception/test/test_grasp_candidates.py
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
```

禁止读取模型、原始图像、历史 CSV、`build/`、`install/`、仿真目录和无关日志。

### 2.5 任务 B 测试顺序

1. 轴语义离线测试：输出 URDF/FK 证据，不运动机械臂。
2. 单元测试：水平姿态拒绝、垂直姿态通过、approach 轴共线、无效姿态拒绝、门控失败不调用 MoveIt。
3. 新鲜 CSV 的 scan-only：统计每个候选的安全角度、拒绝原因和可达性；不启动真机。
4. MoveIt plan-only：确认安全候选路径不包含危险的水平末端下降。
5. 真机 approach-only：必须用户单独批准；禁止自动闭合夹爪。
6. 真机 grasp：必须在 approach-only 日志无异常后再次获得用户批准，单次执行、禁止自动重试。

### 2.6 任务 B 必须交付的日志

```text
docs/real_grasp_error_investigation/log/grasp_axis_semantics.json
docs/real_grasp_error_investigation/log/grasp_orientation_safety.json
docs/real_grasp_error_investigation/log/approach_path_audit.json
docs/real_grasp_error_investigation/log/grasp_orientation_tests.json
docs/real_grasp_error_investigation/log/grasp_safe_plan_only.log
docs/real_grasp_error_investigation/log/grasp_safe_real_execute.log
```

每个候选至少记录：

```json
{
  "candidate_index": 0,
  "score": 0.0,
  "p_camera": [0, 0, 0],
  "p_base": [0, 0, 0],
  "approach_axis_base": [0, 0, -1],
  "opening_axis_base": [0, 0, 0],
  "approach_down": 0.0,
  "approach_tilt_deg": 0.0,
  "approach_clearance_m": 0.05,
  "p_approach": [0, 0, 0],
  "p_grasp": [0, 0, 0],
  "axis_path_error_deg": 0.0,
  "ik_pass": false,
  "moveit_pass": false,
  "collision_pass": false,
  "gate_pass": false,
  "reject_reason": "..."
}
```

真机日志还必须包含：

- 命令 TCP/link6 位姿；
- FK link6/link7 位姿；
- 实际 Piper 关节反馈及时间戳；
- approach/grasp/lift 每阶段开始和结束状态；
- SDK/Can 错误、掉线、急停或断电迹象；
- 是否回 zero；
- `grasp_result.json` 和候选 CSV 的 SHA256。

## 3. Claude Code 执行提示词

执行时只允许使用以下工作顺序：

```text
你只能读取本任务白名单中的文件。先输出文件列表、读取原因和当前 git diff，禁止扫描全仓库。
本任务分为 A（calibration 内参）和 B（姿态安全规划），先做 A，再做 B；不得混合无关修复。
先建立新鲜运行目录，不得读取或复用旧图片、旧 CSV、旧 meta.mat、旧 precheck 或旧 grasp_result。
先做静态分析和单元测试；任何源码修改前先报告拟修改文件、接口、回滚方式和风险。
不得修改 URDF TCP 0.1358 m、手眼 YAML、GraspNet 模型或原始 RealSense camera_info 话题。
水平/近水平候选必须 fail-closed，不能通过降低 MoveIt 约束或关闭预检来强行执行。
真机执行必须等待用户明确批准；未批准时只交付测试日志和差异，不启动真机。
最终报告必须包含本文件要求的全部 JSON、命令、退出码、hash、候选拒绝原因和剩余风险。
```

## 4. 用户批准检查表

用户检查以下内容后再批准执行：

- [ ] 是否同意修改桥接节点，让 `meta.mat` 使用 `calibration.yaml`？
- [ ] 是否同意保留原始 RealSense CameraInfo 话题、不覆盖它？
- [ ] 是否同意采用“近水平候选默认拒绝”的安全策略？
- [ ] 是否同意 approach 改为沿末端接近轴生成？
- [ ] 是否同意先做 plan-only/approach-only，再单独批准完整真机抓取？
- [ ] 是否同意阈值先用 `45°` 作为实验初值，并由日志验证后再调整？

源码实施和离线测试已完成，当前状态为：

```text
IMPLEMENTED_OFFLINE_VALIDATION_COMPLETE
REAL_ROBOT_EXECUTION_PENDING_SEPARATE_APPROVAL
```

## 5. 本轮实施结果

### 5.1 已修改文件

- `src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py`
  - 新增 `calibration_yaml` 和 `use_calibration_intrinsics` 参数；
  - 兼容项目顶层矩阵列表格式和 ROS `{data: ...}` 格式；
  - `meta.mat` 默认使用 calibration K；
  - `camera_info.txt` 同时记录 calibration K/D、运行时 CameraInfo K/D、来源和 SHA256。
- `src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py`
  - 传入项目 calibration 文件，保持原始 CameraInfo 话题不变。
- `grasp_precheck.py`
  - 新增 TCP 接近轴安全门；
  - 默认 `min_approach_down=0.70`、`max_approach_tilt_deg=45.0`；
  - approach 改为 `p_approach = p_grasp - approach_axis_base * clearance`。
- `grasp_executor.py`、`grasp_execute.launch.py`
  - 在 MoveIt/IK 预检前过滤近水平姿态；
  - 全部被姿态门拒绝时返回 `no_safe_candidate`；
  - 将候选姿态安全诊断写入 `precheck_results.json`。
- `test/test_grasp_precheck.py`
  - 增加垂直通过、水平拒绝、路径共线和“门控失败不调用 MoveIt”测试。

### 5.2 验证结论

- 校准桥接离线测试通过：伪造不同的实时 K 后，`meta.mat` 仍等于 `camera_data/calibration.yaml` 的 K；
- 相关回归测试：31 passed；
- 所有修改文件 `py_compile` 通过；
- 尚未验证真实对齐深度流、真实 MoveIt plan-only、真机 approach-only 或完整抓取。

详细日志见 `log/calibration_intrinsics_authority.json`、`log/grasp_orientation_safety.json`、`log/grasp_orientation_tests.json` 和 `log/task11_implementation_20260818.log`。

### 5.3 新鲜真机相机 scan-only 结果

随后使用全新目录执行了真实 D435i → calibration bridge → GraspNet → `grasp_executor scan_only` 链路。相机和推理正常，但当前 12 个候选全部被姿态安全门拒绝：`approach_down` 范围为 `-0.6424..0.5308`，倾角范围为 `57.94..129.97°`。未调用 MoveIt execute、未闭合夹爪、未移动机械臂。

同时发现这些候选的 `-TCP Y` 开口向下分量约为 `0.7649..0.9718`。这表明在放宽阈值前，必须进一步确认 TCP +Z 接近轴与夹爪实际插入轴是否被混用；该结果不能直接证明阈值过严，也不能证明候选可安全执行。

本次日志：`log/task11_live_scan_only_20260818.json`、`log/task11_live_scan_only_20260818.log`。

### 5.4 真机持续抓取请求的阻塞

按“继续执行真机抓取直到成功”启动检查时，确认现有 `piper_with_gripper_moveit demo.launch.py` 的 `ros2_control` 使用 `mock_components/GenericSystem (FakeSystem)`。该链路不能代表真实 Piper 执行，因此已在任何抓取调用前停止；本轮没有发送运动或夹爪指令。必须先提供真实硬件 MoveIt/controller 配置，并验证其反馈来源确实为 `/joint_states_feedback`/Piper 驱动后，才能继续真机抓取。

记录见 `log/task11_real_grasp_attempt_20260818.log`。
