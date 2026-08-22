# 相机内参权威前提下的误差验证报告

> 按 `10-authoritative-calibration-error-validation-plan.md` 执行 C0–C5。未修改任何源码/YAML/URDF/TCP/手眼/内参/轴映射/MoveIt 容差。JSON 位于 `docs/real_grasp_error_investigation/log/`。

## 1. 权威声明与输入

**`camera_data/calibration.yaml` 是本项目相机内参权威文件**（用户裁决）。TCP 长度 `0.1358 m` 为固定值，不再作为可调变量。

| 文件 | SHA256 |
|---|---|
| camera_data/calibration.yaml | 84ad2a80b0afe22cb01ed5591f0b06eb7ad5959df177273c423a9e2e18254415 |
| camera_data/handeye_eye_to_hand.yaml | bb8594191b92b31d5689b49aa996956c6a0de4d1fdedf72d4cb9b3dd1e198da4 |
| /tmp/stageE_frozen.csv | a80a1996bcf4367b7dbcb2b6d28f7a658279ca031acfe0eb71a922e15ad8ec7d（候选[44]，未变） |

## 2. 内参数值与来源

| 项 | 值 |
|---|---|
| calibration K（权威） | fx=604.138, fy=601.117, cx=340.827, cy=238.675 |
| calibration D（权威） | k1=0.0831, k2=0.0535, p1=-0.0074, p2=0.0104, k3=-0.5555（reprojection_error=0.42px） |
| meta.mat K（运行时） | fx=604.214, fy=604.115, cx=321.781, cy=253.111（= ROS CameraInfo，差 2.7e-9） |
| CameraInfo D（运行时） | 全 0（plumb_bob） |
| 差异 | Δcx=+19.05px, Δcy=-14.44px, Δfy=-2.998px；**畸变模型完全不同** |

## 3. 目标坐标实际使用的 K 来源（代码行号）

```text
目标点反投影：inference_from_camera.py read_data_from_shared L235-246
  intrinsic = meta['intrinsic_matrix'] → CameraInfo(fx,fy,cx,cy,factor_depth)
  （GraspNet CameraInfo 无畸变参数接口 → 反投影不做去畸变）
meta.mat 生成：d435i_graspnet_bridge.py _write_meta L115-140（从 ROS CameraInfo msg.k 写入）
手眼标定：~/piper_single_calibration/hand_eye_single_eye_to_hand.py 使用 data/result/calibration.yaml（用户确认）
```

**结论：运行时目标坐标使用 CameraInfo K（meta.mat），不使用权威 calibration.yaml 及其非零畸变。**

## 4. 权威 K 与 runtime K 的候选点坐标差异（C1）

候选[44] 像素 (360, 233)、深度 0.741m：

```text
p_camera_runtime（meta K）     = [46.9, -24.7, 741.0] mm（= CSV xyz_camera，已与深度自洽 ≤2mm）
p_camera_calibration（权威 K） = [23.5, -7.0, 741.0] mm
差值 = [-23.4, +17.7, 0.0] mm，norm = 29.3 mm
```

- 畸变参数（权威 D 非零）在运行时**未被使用**。
- **坐标链不一致确认**：手眼标定建立在 calibration K 语义上，抓取点反投影用另一套 K → `runtime_authority_mismatch = true`。
- 29.3mm（0.74m 处，随深度线性放大）与 2–3cm 误差同量级——**H2/手眼间接路径的关键未知已被本任务解析**：两套 K 的系统性不一致是固定偏差的强候选来源（方向为 calibration−runtime，与 CSV 坐标叠加）。

## 5. C2 手眼绝对验证

**blocked**（`handeye_absolute_validation.json`）：无 ChArUco 板/治具真值点（用户确认）。手眼绝对误差未验证；不用 FK/MoveIt 规划终点代替绝对真值。

## 6. C3 MoveIt 规划误差

引用 `hypothesis_moveit_repeat.json` + `hypothesis_dry_run_repeat.json`（复核 `authoritative_moveit_recheck.json`）：

```text
plan-only（n=6）：E_plan norm 6.7–13.6mm，均值 9.9，XYZ 均值 [-3.5,+1.8,-0.5]，标准差 [5.6,5.3,5.5]
真机执行（n=3）：E_plan norm 5.2–13.9mm，均值 10.4
方向随机（无固定方向）→ 随机终点误差，非固定手眼偏移
```

## 7. C4 真实反馈与 mock 拓扑区别

| 项 | mock/FakeSystem（P4/H4） | 真实反馈（C4，本次） |
|---|---|---|
| /joint_states | FakeSystem 状态回环 = 命令值 | 同左（拓扑不变） |
| /joint_states_feedback | 无 | **真实驱动发布（185Hz）** |
| FK 关节来源 | mock（规划终点） | **真实反馈关节** |
| 用途 | E_plan（规划层） | E_execution（真实执行层） |

C4 结果（grasp 位保持 3 样本，FK 完全一致——臂稳定 0 漂移）：

```text
command TCP = [585.4, 82.5, 35.6]；command link6 = [487.1, -8.3, 58.8]
FK link6（真实反馈）= [489.7, 0.9, 50.0] → E_execution = [2.6, 9.2, -8.8]mm norm 13.0mm
FK link7（真实反馈）= [589.1, 92.1, 33.8] → vs 命令 TCP = [3.7, 9.6, -1.8]mm norm 10.4mm
FK link7 vs 感知物体中心（[585.4,82.5,55.6]）= [3.7, 9.6, -21.8]mm norm 24.1mm（Z -21.8 为 grasp_z_offset 补偿的有意下移；X/Y 水平偏 3.7/9.6mm）
```

**实测（用户量测）**：夹持中心落在物体可抓取边缘上、指尖超出物体（物体实际尺寸 < GraspNet 预测 63.3mm；偏出方向与 FK 水平偏移 [3.7, 9.6]mm 一致）。
**E_total 未量化**（定性描述）→ `E_total_unavailable = true`（10 计划 §9：不得用 mock FK 代替真实误差）。

## 8. C5 姿态/approach 方向（与位置误差分离）

候选[44]（引用 `hypothesis_scanonly.json`）：

```text
R_tcp（base 系）：Z 轴（前向）=[0.724, 0.669, -0.171]，与竖直夹角 99.8°（接近水平）
开口轴（-R[:,1]）=[0.247, -0.482, -0.840]，开口朝下=0.840（可从上插入）
approach 位姿 = grasp + 基座 Z 0.05m → approach 路径（基座 -Z）与夹爪前向轴夹角 80.2°（不共线）
```

结论：**姿态/接近方向问题独立存在**（H3/08 计划），与位置误差、TCP 长度、手眼平移、MoveIt 终点误差分离。本任务未修改。

## 9. C1–C6 判定

| 编号 | 项 | 判定 | 证据 |
|---|---|---|---|
| C1 | calibration K 与运行时反投影链路 | **confirmed（不一致）** | 运行时用 CameraInfo K（无畸变），权威 K 未被使用；候选点差 29.3mm @0.741m；runtime_authority_mismatch=true |
| C2 | 手眼外参绝对误差 | **blocked** | 无真值点（missing=known_base_points） |
| C3 | MoveIt 规划误差 | **confirmed（随机 6.7–13.9mm）** | P2/P4 重复分布 |
| C4 | 真实执行误差 | **部分确认** | 真实反馈 FK E_execution 10.4–13.0mm；实测指尖定性"偏出物体"；E_total 未量化 |
| C5 | 姿态/approach | **confirmed（独立问题）** | 前向接近水平 99.8°、approach 不共线 80.2°（另案 08 处理） |
| C6 | TCP | **confirmed（固定 0.1358）** | 只记录，不可修改 |

## 10. 结论

1. **新确定的最大系统偏差候选**：运行时目标坐标与手眼标定坐标系使用两套不一致的内参（K 主点差 19px + 畸变有无），候选点处差 29.3mm（随深度线性放大）——与 2–3cm 固定误差量级吻合，且能解释"固定方向"特征（此前 E_plan 随机性只能解释 ~10mm）。
2. 真实执行偏差（C4）10.4–13.0mm 与 mock 级（P4）一致 → 执行层本身 ≤3mm（阶段 D）确认，偏差主体为 MoveIt 规划容差。
3. 实测"夹持中心在边缘、指尖超出"= 水平偏移 [3.7, 9.6]mm（MoveIt 容差量级）叠加上可能的坐标系偏差（C1 未修复前）。
4. 姿态问题独立存在（C5）。

## 11. 下一步（最多 2 项）

1. **修复 C1 坐标链不一致（需用户授权修改）**：将运行时点云/目标坐标统一到权威 calibration.yaml（含去畸变）——修改 `d435i_graspnet_bridge.py _write_meta`（写入权威 K/D）与 `inference_from_camera.py` 反投影（应用畸变校正）；或把权威 K 同步为 CameraInfo。输入：calibration.yaml（已冻结 hash）、候选[44] 冻结帧；命令：修改后 `colcon build` + P2 复跑 + C4 复测（E_execution 应消除 ~29mm 系统分量）；输出：`log/c1_fix_validation.json`。
2. **C2 手眼绝对验证（获取真值后）**：放置 ChArUco 板/治具 ≥3 个已知 base 坐标点，离线对比预测 vs 实测（`handeye_absolute_validation.json` 由 blocked 转 done）。输入：标定点坐标表 + 一帧标定板图像；输出：同上 JSON 更新。

> 报告引用 JSON：`authoritative_intrinsics_identity.json`、`authoritative_intrinsics_runtime_check.json`、`handeye_absolute_validation.json`、`authoritative_moveit_recheck.json`、`real_feedback_endpoint_validation.json`、`hypothesis_scanonly.json`（C5）。
