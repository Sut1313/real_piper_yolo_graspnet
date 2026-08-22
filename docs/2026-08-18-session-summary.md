# 2026-08-18 真机误差排查会话总结

> 本文档按 docs/ 惯例汇总 2026-08-18 全天排查工作（含本会话与并行会话/用户裁决）。所有原始数据见 `docs/real_grasp_error_investigation/`（35 个 JSON + 10 份报告）与 `docs/real_grasp_error_investigation/log/`（8 份运行日志）。

## 1. 问题定义

Piper 真机抓取流程可运行，但末端相对真实目标长期存在 **2–3 cm 位置误差**。目标：分层、可重复地定位误差来源，禁止无证据修改代码。

## 2. 今日裁决（用户）

| 时间 | 裁决 |
|---|---|
| 19:28 | `camera_data/calibration.yaml` 是正式内参（后于 21:12 再次确认权威） |
| 19:48 | URDF TCP 长度 `0.1358 m` 为真实值，不得再作为可调变量（F4 现场测量被 supersede） |
| 20:47 | 09 计划授权（H1-H5 最小假设实验） |
| 21:12 | 10 计划授权（权威内参前提下的误差验证）；C4 真机授权 |
| 22:30 | `11-calibration-and-safe-grasp-planning.md` 新计划（姿态安全门已实现入代码） |

## 3. 执行计划与结果（按时间线）

### 3.1 原计划 T0–T6 / 阶段 A–F（18:51–19:16）

**阶段 A 静态核对**：符号溯源（factor_depth / R_cam2base / TCP_OFFSET / GRASPNET_TO_PIPER_TCP_R / END_EFFECTOR_LINK / link7）；handeye R 正交校验（det=1.0000000000，正交误差 1.2e-15）；发现 meta.mat（CameraInfo）与 calibration.yaml 主点差 Δcx≈19px，但 calibration.yaml **无代码引用**；URDF 几何确认（joint6_to_gripper_base 原点 0，joint7/8 原点 z=0.1358）。

**阶段 B 离线复算**（固定帧 2026-08-16 22:31）：xyz_camera 与深度/内参反投影自洽 ≤2mm（排除深度尺度/内参重复缩放）；center_camera 偏深 22.6mm（点云均值语义，已由 grasp_z_offset=0.02 补偿）。

**阶段 C scan_only**（新采集 T0 五帧+推理帧）：候选[14] 全过；T0 发现**帧间候选不稳定**（候选数 46–57 波动，同候选 xyz 范围 38–47mm）。

**阶段 D 真机 15 次到位**（velocity 0.1）：全部成功；真机反馈 vs 规划终点关节差 max **0.0045 rad（≈2–3mm）**、无正反向回差 → **执行层排除**；FK vs 命令预测 7–13mm（MoveIt 位姿约束 ±1cm 盒）。

**阶段 E 候选[44] 单次验证**：dry_stop@grasp；FK link7 vs command TCP = 13.7mm；用户量测指尖 vs 目标 ≈20mm。

**阶段 F 根因报告**：确定存在 MoveIt 位姿容差（±1cm 盒）1cm 级误差；未归属 5–10mm（感知绝对位置/TCP 几何）。

### 3.2 F1–F4 补测（19:28–19:37）

| 任务 | 结论 |
|---|---|
| F1 内参 | runtime K ≡ meta.mat K（差 2.7e-9）；运行时不用 calibration.yaml；stored/runtime 反投影差 23.7mm @0.6m（主点主导）；**手眼标定 K 来源 unknown**（关键未知） |
| F2 几何基准修正 | 原"FK link7 − TCP=13.7mm"**基准错误**（link7 铰点≠TCP，固有差 4.2mm）；修正后 E_tool=13.0mm |
| F3 MoveIt 容差 | E_plan 均值 9.1mm / 最大 13.0mm / 方向随机 → **支持收紧容差** |
| F4 TCP | blocked（无物理测量） |

### 3.3 06 计划 U0–U4：TCP 切换 URDF 权威值（19:48–20:10）

- 代码统一修改 `TCP_OFFSET_FROM_LINK6_Z = 0.1358`（grasp_precheck/grasp_execute/grasp_executor/测试 13 处；未动容差/手眼/轴映射）
- 单测 20/20 通过（1 个预存失败与 TCP 无关）；colcon build 通过
- 离线 A/B：Δ(B−A)=4.2mm 沿 TCP Z 轴，与理论一致
- scan_only 候选[44] 全过（统计不变）
- **U4 真机 dry-run**（用户授权）：E_tool 13.0→7.3mm（n=1，方向一致但样本不足 → **效果未验证**）

### 3.4 07 计划 V0–V3（并行会话 20:21–20:42）

moveit_adapter.py 修复：OrientationConstraint.frame_id=base_link、GetCartesianPath 的 `eef_step`→`max_step`（ROS2 Humble 实际字段）、header/link_name 补全。源码=安装版（hash cc963e1e）。真机复测 blocked（当时无 ROS graph）。

### 3.5 09 计划 P0–P4：最小假设实验（20:47–20:58）

| 假设 | 判定 | 证据 |
|---|---|---|
| H1 MoveIt 随机终点误差 | **已确认** | P2 plan-only 6 次 E_plan 6.7–13.6mm（均值 9.9）；P4 真机 3 次 5.2–13.9mm（均值 10.4）；方向随机 |
| H2 手眼/内参固定偏差 | 关键未知 | 无标定物真值；手眼 K 来源 unknown（后被 10 计划解析） |
| H3 姿态/接近方向 | 观测确认 | 候选[44] 前向与竖直夹角 99.8°（接近水平）；approach 路径与前向轴夹角 80.2°（不共线） |
| H4 TCP 主因 | 排除 | 4.2mm ≪ 2–3cm |
| H5 机械执行 | 排除 | 关节差 ≤0.0045 rad |

另确认：07 修复生效（P1 无 frame/eef_step 警告）。

### 3.6 10 计划 C0–C5：权威内参验证（21:12–21:30）

**核心发现（C1，修复前）**：运行时目标坐标使用 CameraInfo K（meta.mat，D 全 0、无去畸变），**不使用权威 calibration.yaml（含非零畸变 k1=0.083/k3=−0.556）**；候选[44] 深度 0.741m 处两套 K 反投影差 **29.3mm**（[−23.4, +17.7, 0]，随深度线性放大）——手眼标定用权威 K、抓取点用另一套 K 的**坐标链系统性不一致**（`runtime_authority_mismatch=true`），与 2–3cm 固定误差同量级。随后 Task 11 已将桥接节点生成的 `meta.mat` 切换为 calibration.yaml K，并在本日实时链路中验证一致。

| 项 | 判定 |
|---|---|
| C1 内参链路 | **confirmed（修复前不一致；Task 11 后运行时已统一 calibration.yaml K）** |
| C2 手眼绝对验证 | blocked（无标定物） |
| C3 MoveIt 误差 | confirmed（随机 6.7–13.9mm） |
| C4 真实执行 | 真实反馈 FK E_execution 10.4–13.0mm；用户量测"**夹持中心在物体可抓取边缘、指尖超出物体**"（=FK link7 相对物体中心水平偏 [3.7, 9.6]mm）；E_total 未量化 |
| C5 姿态 | confirmed（独立问题） |
| C6 TCP | fixed 0.1358 |

## 4. 误差分层结论（截至今日 22:45）

```text
E_total ≈ 20–30mm 分解：
├─ C1 坐标链不一致（两套 K，29.3mm @0.74m，随深度线性）→ 原始根因已确认；本日已切换运行时 meta.mat 使用 calibration.yaml K，在线发布验证通过
├─ MoveIt 位姿约束 ±1cm 盒 → 随机 5–14mm（H1 已确认）
├─ 执行层 ≤3mm（H5 排除）
├─ TCP 4.2mm 修正（H4 排除为主因）
└─ 姿态问题（C5/08 计划）独立存在——安全门已由用户实现入代码
```

## 5. 关键交付物

- 报告：`root-cause-report.md`、`follow-up-report.md`、`tcp-update-report.md`、`moveit-runtime-consistency-report.md`、`hypothesis-analysis-report.md`、`10-authoritative-calibration-error-validation-report.md`
- 数据：`docs/real_grasp_error_investigation/log/`（35 个 JSON）、`frame_20260816_2231/`、`frame_T0_20260818/`（固定帧+hash）、`/tmp/stageE_frozen.csv`（hash a80a1996…）、`/tmp/stageD_report.json`
- 代码修改（均已 build+测试）：TCP 0.1358（本会话 U1）；moveit_adapter frame/max_step（07 会话）；**姿态安全门 check_orientation_safety（用户，11 计划）**

## 6. 下一步（按 11 计划与各报告）

1. 在恢复 USB 设备后，重新验证 calibration.yaml K → meta.mat → GraspNet → base 坐标的完整链路；当前只证明发布链一致，尚未证明手眼绝对位置准确
2. 姿态安全门落地后复测（11 计划），保持 `min_approach_down=0.70`、`max_approach_tilt_deg=45°`，不得因无候选而关闭门控
3. C2 手眼绝对验证（获取标定点后）
4. 增加或确认候选的 base 高度/工作空间/目标区域过滤，避免将桌面边缘或背景候选送入真机

## 7. 本次“持续真机抓取”执行记录（22:26–22:45）

### 7.1 启动与数据边界

- 新建唯一运行目录：`/tmp/real_grasp_until_success_SUO2Hi`；未复用上一次图片、CSV、`meta.mat` 或结果文件。
- 机械臂启动参数：`can0`、1 Mbps、`auto_enable=true`、`gripper_val_mutiple=2`、`gripper_forward=false`。
- 相机：RealSense D435i，串号 `[REDACTED_CAMERA_SERIAL]`，640×480@30，桥接节点使用 `camera_data/calibration.yaml` 内参生成 `meta.mat`。
- TCP：URDF 权威值 `0.1358 m`。
- 安全门：开启，`min_approach_down=0.70`、`max_approach_tilt_deg=45°`；速度比例 `0.1`。

### 7.2 预检与诊断

- 先后进行了 20 次新鲜候选触发，均未形成完整可执行候选；未产生机械臂动作。
- 以 `0.60/55°` 做了一次只读敏感性扫描：少数候选通过姿态门，但接近规划仍失败。
- 关闭姿态门仅用于只读诊断：15 个候选全部 `approach_pass=0`，证明当前问题不只是姿态门限过严。
- 恢复原安全门后继续真机执行；此前代码虽支持 `candidate_frames`，但 launch 文件未透传，本次实际仍为 `1/1` 帧。已补充 launch 参数映射并完成 `colcon build`，后续可显式使用 5 帧。

### 7.3 唯一一次真实运动

预检选择候选 9 后进入真实执行：

```text
candidate_index=9
candidate_score=0.1123
grasp_width=0.0910m
cmd_width=0.0960m
full_open=0.0697m
full_closed=0.0042m
actual_width=0.0043m
result=grasp_empty
returned_zero=true
```

该候选经手眼变换的位置约为 `(0.150, 0.330, 0.038) m`，执行后 FK 记录的 `link7=(0.151, 0.335,-0.050)m`。夹爪完全闭合且未夹住物体，因此不能视为抓取成功；更可能是背景/非目标候选，不应继续按相同策略盲试。

### 7.4 硬件中断与收尾

- Piper 驱动日志：`can0 is loss`，`piper_single_ctrl` 以退出码 1 结束。
- 随后 `can0` 接口消失；`lsusb` 中 USB-CAN 和 RealSense 同时消失，`/dev/video*` 不存在。
- 已停止 executor、MoveIt、RealSense 和 GraspNet；`pause_inference` 已清除。
- 未能读取内核 `dmesg`（当前用户权限不足），因此 USB 总线复位、供电或连接器问题仍需现场确认。

## 8. 本次结论与恢复条件

1. 本次没有达到“真机抓取成功”；结果是一次 `grasp_empty`，不能与历史 `dry_stop` 混为成功。
2. 当前新证据支持：候选质量/目标筛选和规划可达性仍是主要阻塞；不能只通过放宽姿态门限解决。
3. 在任何下一次运动前，必须先确认 `lsusb` 同时出现 RealSense 与 USB-CAN、`can0` 为 `ERROR-ACTIVE`、驱动反馈正常且机械臂处于 zero。
4. 硬件恢复后应使用新的运行目录和新鲜帧，启用已构建的 `candidate_frames:=5`，保持姿态安全门开启；若再次出现 CAN 丢失，立即停止，不自动重试。

本次详细机器证据见：
`docs/real_grasp_error_investigation/log/task11_real_grasp_until_success_20260818.json`。
