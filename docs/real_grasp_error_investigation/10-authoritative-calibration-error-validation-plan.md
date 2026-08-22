# 当前相机内参权威前提下的误差验证任务

## 1. 任务结论与目标

用户已确认：

- `camera_data/calibration.yaml` 就是当前使用相机的正式内参文件；
- 其中的 `camera_matrix` 和 `dist_coeff` 是本项目相机内参真值来源；
- URDF 中的 TCP 长度 `0.1358 m` 是真实值，后续不得再把 TCP 长度作为可调变量。

根据 `~/piper_single_calibration/hand_eye_single_eye_to_hand.py` 的当前实现，手眼标定的 ChArUco 位姿估计使用 `data/result/calibration.yaml`，而不是从 RealSense SDK 查询内参。因此本任务不再把“手眼标定使用的 K 来源”标记为 unknown。

本任务只验证剩余问题：

1. 当前抓取点云/GraspNet 反投影是否使用了与 `camera_data/calibration.yaml` 一致的内参和畸变处理；
2. `handeye_eye_to_hand.yaml` 的绝对坐标变换是否存在固定平移或旋转误差；
3. MoveIt 规划终点误差与真实机械臂反馈误差分别是多少；
4. 夹爪姿态和 approach 方向问题是否与位置误差独立存在。

本任务默认只读和测量，不修改源码、YAML、URDF、TCP、手眼矩阵、轴映射或 MoveIt 容差。

## 2. 已确认的误差模型

当前不再使用“calibration.yaml 与手眼标定文件不一致”作为假设。新的分层模型如下：

| 编号 | 待验证部分 | 当前状态 | 需要的证据 |
|---|---|---|---|
| C1 | `calibration.yaml` 与运行时点云反投影链路 | 运行时桥接节点从 ROS `CameraInfo` 写入 `meta.mat`，是否与权威 calibration 一致需测量 | 两套 K/D、像素深度、同一点反投影差异 |
| C2 | 手眼外参绝对误差 | 矩阵形式健康，但缺少绝对真值点验证 | 至少 3 个已知 base 坐标点 |
| C3 | MoveIt 规划误差 | 已观察到约 5–14 mm 随机误差 | 固定输入下的 command/FK/反馈重复统计 |
| C4 | Piper 真机执行误差 | 之前测试混用了 FakeSystem，真实末端反馈证据不足 | 真实驱动反馈关节、时间戳和 FK |
| C5 | 夹爪姿态/插入方向 | 候选 `[44]` 已观察到接近水平 | approach 轴、开口轴、路径方向和抓取结果 |
| C6 | TCP | 已确认 `0.1358 m` | 只记录，不做 A/B，不允许修改 |

## 3. Claude Code 最小读取白名单

Claude Code 必须先阅读本文件，只读取以下文件；禁止为了“了解项目”而扫描全仓库：

```text
camera_data/calibration.yaml
camera_data/handeye_eye_to_hand.yaml
shared_camera_data/meta.mat
shared_camera_data/camera_info.txt
shared_camera_data/precheck_results.json
shared_camera_data/grasp_result.json
shared_camera_data/fk_link7_position.json

src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_executor.py
src/real_piper_perception/real_piper_perception/moveit_adapter.py
graspnet-baseline/inference_from_camera.py

docs/real_grasp_error_investigation/log/hypothesis_runtime_identity.json
docs/real_grasp_error_investigation/log/hypothesis_scanonly.json
docs/real_grasp_error_investigation/log/hypothesis_moveit_repeat.json
docs/real_grasp_error_investigation/log/hypothesis_dry_run_repeat.json
docs/real_grasp_error_investigation/log/hypothesis_intrinsics_provenance.json
log/investigation_20260818_H4_execute.log
log/investigation_20260818_H4_driver2.log
```

只有在确认需要读取 URDF 几何时，才允许读取以下两个文件：

```text
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_description/urdf/piper_description.xacro
```

只读取其中包含 `joint6_to_gripper_base`、`joint7`、`joint8`、TCP 或 gripper 末端定义的行，不得读取旧版 URDF、Gazebo URDF 或其他 MoveIt 配置。

明确禁止读取：

- `build/`、`install/`、`log/` 中未列出的文件；
- `graspnet-baseline/models/`、模型权重和推理源码中与 `inference_from_camera.py` 无关的文件；
- `piper_single_calibration/`，其内参来源已经确认，本轮不重复分析；
- 仿真、MuJoCo、Gazebo 和无关测试目录；
- 所有历史图像、点云和历史帧，除非本任务明确指定某一个帧文件。

## 4. 固定输入与安全边界

所有可比较实验固定使用：

```text
TCP_OFFSET_FROM_LINK6_Z = 0.1358 m
CSV = /tmp/stageE_frozen.csv
候选 = [44]
起点 = zero
grasp_z_offset = 0.02 m
approach_clearance = 0.05 m
```

开始前记录：

```bash
sha256sum camera_data/calibration.yaml \
  camera_data/handeye_eye_to_hand.yaml \
  /tmp/stageE_frozen.csv
```

如果内参、手眼文件或冻结 CSV 在实验中发生变化，立即停止比较并在报告中标记 `input_changed`。

安全要求：

- C0、C1、C2：不启动真机驱动，不调用会移动机械臂的服务；
- C3：只允许 plan-only；
- C4：必须获得用户明确授权后才能启动真机；优先使用 `dry_run_max_stage:=grasp`，不闭合夹爪；
- 任何阶段都不得修改 `camera_data/calibration.yaml` 或 `camera_data/handeye_eye_to_hand.yaml`。

## 5. C0：权威文件与运行时身份检查

目标：确认实验输入、内参文件、手眼文件和运行版本没有被混淆。

记录：

1. `calibration.yaml` 的 SHA256、K、D、文件分辨率信息；
2. `handeye_eye_to_hand.yaml` 的 SHA256、R、t、`det(R)`、正交误差；
3. `meta.mat` 中 `intrinsic_matrix` 和 `factor_depth`；
4. `camera_info.txt` 中的 topic、分辨率、K、D；
5. 当前推理读取 `meta.mat` 的代码位置；
6. 当前桥接节点生成 `meta.mat` 的代码位置；
7. 明确声明：本轮把 `camera_data/calibration.yaml` 作为相机内参权威文件。

生成：

```text
docs/real_grasp_error_investigation/log/authoritative_intrinsics_identity.json
```

JSON 至少包含：

```json
{
  "status": "done|blocked",
  "authority": "camera_data/calibration.yaml",
  "calibration_sha256": "...",
  "handeye_sha256": "...",
  "csv_sha256": "...",
  "K_calibration": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
  "D_calibration": [],
  "K_runtime_meta": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
  "D_runtime_camera_info": [],
  "runtime_meta_source": "CameraInfo topic or unknown",
  "runtime_inference_source": "meta.mat or unknown",
  "tcp_offset_m": 0.1358
}
```

## 6. C1：权威 K 与运行时点云链路的一致性

目标：不重新判断哪个文件“更可信”，而是确认当前代码是否把权威 `calibration.yaml` 正确用于目标坐标计算。

只做离线计算和代码追踪，不修改运行时文件。

步骤：

1. 从 `calibration.yaml` 读取 `K_calibration`、`D_calibration`；
2. 从当前 `meta.mat` 读取 `K_runtime`；
3. 从 `camera_info.txt` 读取实时 ROS CameraInfo 的 K/D；
4. 确认 `inference_from_camera.py` 的点云反投影到底使用哪个 K；
5. 对候选 `[44]` 的实际像素和深度，分别用 `K_calibration` 与 `K_runtime` 计算相机坐标；
6. 如果存在畸变参数，明确记录当前反投影是否进行去畸变；
7. 输出每个点的 `p_camera_calibration`、`p_camera_runtime`、差值向量和范数；
8. 不把“两个 K 不相等”直接等同于 2–3 cm 误差，必须同时说明该 K 是否真正进入目标坐标计算。

生成：

```text
docs/real_grasp_error_investigation/log/authoritative_intrinsics_runtime_check.json
```

必须回答：

- 运行时目标点使用的是 calibration K、SDK/CameraInfo K，还是其他来源；
- `meta.mat` 是否只是保存实时 CameraInfo；
- calibration K 与 runtime K 的差异在候选深度处是多少毫米；
- 畸变参数是否被使用；
- 是否存在“手眼使用 calibration K，但抓取点使用另一套 K”的坐标链不一致。

如果 C1 发现运行时不使用权威 K，不得直接改代码；只记录为 `runtime_authority_mismatch=true`，交给后续修复任务。

## 7. C2：手眼外参绝对坐标验证

目标：验证当前 `handeye_eye_to_hand.yaml` 将相机坐标转换到 base 坐标时，是否产生固定平移或旋转误差。

优先使用 ChArUco 板或已知尺寸的固定标定物，至少采集 3 个不共线、深度不同的已知点。每个点必须同时有：

- 相机图像坐标和深度；
- 用 `calibration.yaml` 求得的相机坐标；
- `handeye_eye_to_hand.yaml` 变换后的 base 坐标；
- 人工测量或机械臂/治具给出的 base 真值坐标。

每个点计算：

```text
E_transform = p_base_predicted - p_base_measured
E_norm_mm = ||E_transform|| * 1000
```

至少报告：

- XYZ 误差向量；
- 误差范数；
- 误差随深度的变化；
- 是否存在近似固定方向；
- 是否存在绕某一轴的旋转误差特征。

生成：

```text
docs/real_grasp_error_investigation/log/handeye_absolute_validation.json
```

没有真实测量点时不得伪造结论，必须写：

```json
{
  "status": "blocked",
  "missing": ["known_base_points"],
  "conclusion": "不能用 FK 或 MoveIt 规划终点代替手眼绝对真值"
}
```

## 8. C3：MoveIt 规划误差复核

不重新读取相机文件，不重新运行 GraspNet。直接引用已有：

```text
docs/real_grasp_error_investigation/log/hypothesis_moveit_repeat.json
docs/real_grasp_error_investigation/log/hypothesis_dry_run_repeat.json
```

只核对：

- command TCP 与 command link6 是否使用 `0.1358 m`；
- FK 使用的 link 是否为 `link6`；
- `E_plan = FK_link6 - command_link6`；
- 误差均值、标准差、最大值和方向；
- 该误差是否为随机终点误差，而非固定手眼偏移。

如需复测，只能固定 CSV、候选、起点、姿态和 TCP，单独改变 MoveIt 位置约束；不得同时改变 K、手眼、TCP 或轴映射。

## 9. C4：真实反馈与物理末端误差

此前 H4 使用了 `FakeSystem`，FK 不能作为真实末端实测值。本阶段必须区分：

```text
planned joints
real feedback joints
FK from planned joints
FK from real feedback joints
measured fingertip / grasp center
measured target center
```

若用户授权真机干跑：

1. 使用真实 Piper 驱动和真实关节反馈；
2. 不使用 FakeSystem 作为唯一 `/joint_states` 来源；
3. 固定候选 `[44]`，执行到 `grasp` 后停止；
4. 不闭合夹爪，不执行 lift；
5. 在 grasp 位保持不动，采集至少 3 个连续反馈样本；
6. 记录实际指尖/夹持中心和目标中心，不能只记录 link7；
7. 完成测量后再回 zero。

生成：

```text
docs/real_grasp_error_investigation/log/real_feedback_endpoint_validation.json
```

每次运行必须包含：

```json
{
  "run_id": 1,
  "command_tcp_mm": [0, 0, 0],
  "command_link6_mm": [0, 0, 0],
  "planned_joint_positions": [],
  "feedback_joint_positions": [],
  "fk_link6_feedback_mm": [0, 0, 0],
  "fk_tcp_feedback_mm": [0, 0, 0],
  "measured_fingertip_mm": null,
  "measured_target_mm": null,
  "E_execution_mm": null,
  "E_total_mm": null,
  "feedback_topic": "/joint_states_feedback",
  "feedback_timestamp": null
}
```

如果没有真实反馈或没有物理测量，必须明确标记 `E_total_unavailable`，不得用 mock FK 代替真实误差。

## 10. C5：姿态与位置误差分离

只记录，不修改规划代码。对候选 `[44]` 报告：

- `R_tcp`、`R_link6`；
- TCP/link6 前向轴；
- 夹爪开口轴；
- approach 路径方向；
- approach 轴与路径夹角；
- 开口方向与物体插入方向夹角；
- 是否因为接近水平而无法插入。

该结论必须与以下内容分开：

- 位置误差；
- TCP 长度；
- 手眼平移误差；
- MoveIt 终点误差。

## 11. 最终报告要求

生成：

```text
docs/real_grasp_error_investigation/10-authoritative-calibration-error-validation-report.md
```

报告必须包含：

1. 明确声明 `camera_data/calibration.yaml` 是当前相机内参权威文件；
2. calibration K、runtime meta K、CameraInfo K 的数值与 SHA256；
3. 当前目标坐标实际使用的 K 来源和代码行号；
4. calibration K 与 runtime K 造成的候选点坐标差异；
5. 手眼绝对验证的点数量、真值来源和误差表；
6. MoveIt `E_plan` 统计；
7. 真实反馈与 mock/FakeSystem 的拓扑区别；
8. 真实指尖、目标中心和 `E_total`，没有则写 unavailable；
9. 姿态/approach 方向结论；
10. 逐项判断 C1–C6：confirmed、rejected、blocked 或 unknown；
11. 下一步最多列 2 项，每项必须给出输入文件、命令和输出日志路径。

禁止在最终报告中使用以下无证据表述：

- “SDK 内参一定比 calibration.yaml 正确”；
- “FK link7 就是真实 TCP”；
- “dry_stop success 就等于抓取成功”；
- “MoveIt 规划成功就等于真机到位”；
- “没有物理真值点即可确认手眼误差”。

## 12. 完成条件

本任务只有在以下条件满足时才算完成：

- C0 的权威输入和 hash 齐全；
- C1 明确当前目标坐标使用的内参来源；
- C2 有至少 3 个真实绝对坐标点，或规范记录为 blocked；
- C3 的规划误差与执行误差分开统计；
- C4 明确区分真实反馈、FakeSystem 和 mock FK；
- C5 不与 TCP 或手眼误差混淆；
- 最终报告引用所有 JSON 日志，并且没有伪造缺失的物理测量数据。
