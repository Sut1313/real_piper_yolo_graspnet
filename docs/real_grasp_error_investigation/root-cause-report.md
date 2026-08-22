# 真机抓取误差根因报告

> 本文件基于 `measurement-log.md`（2026-08-18 会话，阶段 A–E 实测数据）。未采集项一律标注"未采集+原因"，不以推测填充。

## 1. 一句话结论

状态：`尚未完全确定`（缺感知绝对位置的独立真值对照），但已确定：**执行层（≤3mm）、深度尺度/内参（≤2mm）、手眼矩阵数值、变换实现均被排除**；**确定存在 1cm 级误差源 = MoveIt 位姿规划容差（PositionConstraint ±1cm 盒 / JointConstraint ±0.01rad）**（实测 FK vs 命令 7–13.7mm）；剩余 ~5–10mm 未归属，候选为感知目标绝对位置（center 偏深 22.6mm 语义问题已部分补偿）或 TCP 指尖几何（0.14 vs 0.1358 + 指尖，未实测）。

## 2. 误差统计

| 数据集/测试 | 样本数 | 平均 X/Y/Z (mm) | 标准差 X/Y/Z (mm) | 最大范数 (mm) | 坐标系 |
|---|---:|---|---|---:|---|
| 阶段D grasp_like：FK link6 vs 命令预测 | 6 | [-3.7, +3.1, +3.2] | [7.3, 3.5, 3.9] | 13.0 | base_link |
| 阶段D grasp_like：FK link7 vs 命令预测 | 6 | [-3.8, +2.1, -0.2] | [6.4, 3.8, 3.0] | 10.6 | base_link |
| 阶段E1：FK link7 vs 命令 TCP | 1 | [-13.7, +0.1, -0.6] | — | 13.7 | base_link |
| 阶段E2：实测指尖 vs 目标物中心 | 1 | 未细分（用户粗测） | — | ≈20 | 现场量测 |
| 阶段D：关节空间（真机 feedback vs 规划终点） | 15 | max 均值 0.0019 rad | — | 0.0045 rad（≈2–3mm 末端） | joint |

误差是否固定方向：阶段 E 的 FK 偏差主要在 base X（-13.7mm）；总误差方向未细分（用户量测未给 XYZ 分量）。
误差是否随目标深度变化：未验证（单目标单距离）。
误差是否随姿态或运动方向变化：阶段 D 正反向无系统性差（无回差）。

## 3. 证据链

| 结论 | 原始日志 test_id/frame_id | 代码文件和函数/行号 | 证据 |
|---|---|---|---|
| 执行层误差 ≤3mm、无回差 | D5（15 次到位，log/investigation_20260818_stageD.log） | moveit_adapter.py plan/execute L178-303；grasp_execute.py execute_pick | 真机 feedback vs 规划终点 max 0.0045 rad；正反无方向性差 |
| MoveIt 位姿容差造成 1cm 级 FK 偏差 | D5 grasp_like 6 次（7–13mm）、E1（13.7mm X 向） | moveit_adapter.py L205-215 PositionConstraint 0.02m 盒、L217-223 姿态容差 0.05rad、L191-199 JointConstraint ±0.01rad | 同一目标重复执行 FK 偏差 7–13mm 随机方向；关节目标（ready/zero）偏差仅 ≤0.0045 rad——位姿目标路径独有 |
| 深度尺度/内参无重复缩放 | B1（固定帧复算） | bridge `_write_depth` L103-113；inference `read_data_from_shared` L235-246、`process_data` L512 | xyz_camera 反投影 vs 深度 ≤2mm；factor_depth=1000 mm→m 单次 |
| 手眼矩阵数值健康 | A1 | grasp_transform.load_handeye L69-105 | det(R)=1.0000000000，正交误差 1.2e-15，T_cam2base 与 R/t 一致 |
| 变换实现与公式一致 | B1/C 复算 | grasp_transform.py L108-134/L137-149 | 独立复算 p_base/R_base_tcp 与代码同构，det 均 1.0 |
| 感知中心偏深（点云均值语义） | B1 | inference process_data（物体中心计算） | center_camera 反投影处深度 621 vs CSV z=643.6（差 22.6mm）；grasp_z_offset=0.02 已补偿 |
| 候选帧间不稳定 | T0（5 推理帧） | perception_grasps.csv（46–57 行/帧） | 同 (name,rotation) 候选仅 1/5 全帧出现，xyz 范围 38–47mm |
| calibration.yaml 内参矛盾无关 | A1 | — | 全仓 rg 无引用，推理只用 meta.mat（实时 CameraInfo） |
| TCP 几何 0.14 vs URDF 铰点 0.1358 | A1/T3 | grasp_precheck.py L54（注释自认 0.14 为近似）；URDF joint7/joint8 origin z=0.1358 | 差 4.2mm；指尖几何未实测 |

## 4. 假设归类

| 假设 | 支持证据 | 排除证据 | 误差方向/量级 | 置信度 |
|---|---|---|---|---:|---|
| 深度尺度 | 无 | 反投影 vs 深度 ≤2mm（B1） | 无 | 已排除 |
| ROI/目标中心 | center 偏深 22.6mm（点云均值语义） | xyz_camera（执行用）与深度一致；ROI 未截断 | Z 向 ~20mm（已用 grasp_z_offset 补偿） | 中（语义问题已补偿，残差未验证） |
| 手眼平移 | 无真值对照 | 矩阵正交精确、t 与 TF 一致 | 未验证 | 未定（需标定点） |
| 手眼旋转 | 无 | 矩阵正交、无重复帧 | 未验证 | 低 |
| TCP 偏移 | 0.14 vs 0.1358 差 4.2mm；指尖几何未知 | 预检/执行两侧折算一致（自洽） | 沿 TCP 前向 ≤5mm（估） | 中低 |
| GraspNet→TCP 轴映射 | 无 | R 矩阵 det=1、复算一致、开口方向符合设计 | 无 | 低 |
| MoveIt/FK 末端定义 | **FK vs 命令 7–13.7mm 实测** | 关节目标路径偏差仅 0.0045 rad | base 系随机 ~1cm | **高（确定存在）** |
| Piper 执行/机械误差 | 无 | 15 次到位 max 0.0045 rad、无回差 | ≤3mm | 已排除 |

## 5. 最小修复方案

只允许选择一个首要修改点：

```text
修改文件：src/real_piper_perception/real_piper_perception/moveit_adapter.py
修改变量/函数：plan() 内 PositionConstraint.constraint_region（_make_box_primitive(0.02,0.02,0.02) L213-214）
            与 JointConstraint.tolerance_above/below（L196-197）、OrientationConstraint 容差（L219-221）
修改前：位置容差盒 ±1cm；关节容差 ±0.01rad；姿态容差 0.05rad
修改后：位置容差盒 ±2.5mm（0.005）；关节容差 ±0.003rad；姿态容差 0.02rad
预期误差变化方向：FK vs 命令 7–13.7mm → ≤3mm（收紧到执行层精度）；E_total 预计从 ~20mm 降至 ~10mm
为什么不同时修改其他层：一次只改一个误差源（02 计划阶段 F 原则）；MoveIt 容差有直接实测证据
（FK 偏差 7–13mm），其余层无真值对照；先消除确定项，再复测评估剩余量归属。
```

## 6. 回归测试

### 修改前必须增加/更新

```text
测试文件：scripts/stage_d_regression.py（固化本次 /tmp/stageD_test.py 的到位测试）
测试内容：零位环境或真机（velocity 0.1）执行 grasp_like 目标 3 次，采集 FK link7 vs 命令预测
预期失败/通过：修改前 FK 偏差 7–13mm（>5mm，暴露 MoveIt 容差）；修改后 <5mm
```

### 修改后复测

```text
命令：同阶段 E1 —— grasp_execute.launch.py scan_only:=false dry_run_max_stage:=grasp csv_path:=<冻结CSV>
固定输入 frame_id/候选：/tmp/stageE_frozen.csv（sha256=a80a1996...）候选[44]（或同场景新帧 scan_only 确认）
真机测试条件：用户在场、velocity 0.1、夹爪不闭合
通过标准：平均误差 < 5mm；最大误差 < 10mm；FK link7 vs 命令 TCP norm < 5mm
```

## 7. 尚未确定时的最小下一步数据

如果根因不能确定，只列出下一次必须采集的数据，不要继续扩大文件读取范围：

1. **标定物真值（最高优先）**：≥3 个相机内已知位置标定点（人工量 base 坐标）→ 验证感知绝对位置与手眼平移残差（E_perception/E_transform 分离）。
2. **TCP 指尖几何实测**：link6 基准面 → 指尖夹持中心距离（左右指），修正 TCP_OFFSET_FROM_LINK6_Z（0.14 vs 实测）。
3. **误差方向细分**：阶段 E 现场量测记录指尖 vs 目标的 XYZ 分量与方向（本次仅"≈20mm"）。
4. 同一目标不同深度（0.4/0.6/0.8m）各一次 grasp 到位，验证误差是否随距离变化（旋转误差判别）。
