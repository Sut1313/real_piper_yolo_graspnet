# 当前日志后的补测报告

> 按 `04-follow-up-work-list.md` 执行 F1–F4。所有输出 JSON 位于 `docs/real_grasp_error_investigation/log/`。本轮未修改任何源码/配置，未运行 GraspNet，未执行真机运动（F4.3 未授权）。

## 1. 已确认事实（引用已有 test_id/frame_id）

| 事实 | 证据（test_id/frame_id） | 来源 |
|---|---|---|
| 运行时链路只用实时 K；meta.mat K ≡ runtime K（max 差 2.7e-9） | A1/C0/B1 | `intrinsics_runtime_check.json` |
| stored K（calibration.yaml）无任何代码引用 | A1（全仓 rg） | 同上 |
| 深度尺度/内参反投影自洽 ≤2mm | B1（固定帧 4 候选） | measurement-log.md |
| 执行层误差 ≤3mm、无回差 | D5（15 次到位） | measurement-log.md |
| E1 原报告"FK link7 - TCP = 13.7mm"基准错误 | E1 | `tcp_link_geometry_check.json` |
| E_plan（FK link6 vs 命令 link6）= 均值 9.1mm / 最大 13.0mm / 方向随机 | D5 grasp_like 6 次 | `moveit_tolerance_check.json` |
| E_total 实测 ≈20mm | E2（用户现场量测，方向未细分） | measurement-log.md |
| 帧间候选不稳定（候选数 46–57 波动，同候选 xyz 范围 38–47mm） | T0（5 推理帧） | measurement-log.md |

## 2. 内参差异分析

### 数值差异（runtime K vs stored K）

```text
runtime: fx=604.214, fy=604.115, cx=321.781, cy=253.111
stored:  fx=604.138, fy=601.117, cx=340.827, cy=238.675
Δfx=-0.077px  Δfy=-2.998px  Δcx=+19.046px  Δcy=-14.436px
```

### 0.4/0.6/0.8m 理论误差（同一像素 (360,233)，反投影 Δ = runtime − stored）

| z | ΔX (mm) | ΔY (mm) | norm (mm) | 主点贡献 | 焦距贡献 |
|---|---|---|---:|---|---|
| 0.4 | +12.6 | -9.5 | 15.8 | ≈100% | ≈0.2% |
| 0.6 | +18.9 | -14.3 | 23.7 | ≈100% | ≈0.3% |
| 0.8 | +25.2 | -19.1 | 31.6 | ≈100% | ≈0.4% |

结论：主点差异主导（Δfy 仅 3px 对应焦距贡献 <0.5%）；误差随深度线性放大，0.6m 处 23.7mm 足以解释 2–3cm。

### 直接运行时影响 vs 手眼标定间接影响

- **直接运行时影响：否**。meta.mat K ≡ runtime K（差 2.7e-9）；inference `read_data_from_shared` 只读 `meta['intrinsic_matrix']`（graspnet-baseline/inference_from_camera.py L235-243）；calibration.yaml 全仓无引用（阶段 A 记录）。阶段 B 已证 xyz_camera 与深度/内参反投影 ≤2mm。
- **间接影响（手眼标定）：未知（关键未知）**。`handeye_eye_to_hand.yaml` 无内参/时间/标定工具记录（description 仅 "Eye-to-Hand calibration result: T_cam2base"）。无法证明手眼标定使用了哪套 K。**若标定用过 stored K，外参会吸收内参误差**——该路径既不能支持也不能排除，标记为关键未知，不得标记为已排除。

## 3. F1–F4 执行记录

| 任务 | 输入 | 命令 | 退出码 | 输出文件 | 结论 |
|---|---|---|---|---|---|
| F1.1 | measurement-log.md、meta.mat、calibration.yaml | python3 内参核验脚本 | 0 | intrinsics_runtime_check.json | runtime K ≡ meta.mat K；runtime 不用 stored K（yes/no/unknown → **no**） |
| F1.2 | 候选[44] 像素 (360,233)、z=0.4/0.6/0.8 | python3 敏感性计算 | 0 | intrinsics_sensitivity.json | 0.6m 处 norm 23.7mm，主点主导，随深度线性放大 |
| F1.3 | handeye YAML | 字段检查 | 0 | intrinsics_runtime_check.json | 无内参/时间记录 → 手眼标定所用 K = **unknown** |
| F2 | E1 数据 + URDF + grasp_precheck.py | python3 几何重算 | 0 | tcp_link_geometry_check.json | E_tool（修正基准后）= FK7 vs 期望7(joint7=0) = **13.0mm**；原 13.7mm 含 4.2mm 几何固有差 |
| F3 | stageD_report.json + moveit_adapter.py | python3 离线重算 | 0 | moveit_tolerance_check.json | E_plan 均值 9.1mm / 最大 13.0mm / 方向随机 → **支持收紧容差** |
| F4.1 | 无现场测量数据 | — | — | tcp_ab_test.json | **blocked**（无 link6 基准面→指尖夹持中心测量） |
| F4.2 | 依赖 F4.1 | — | — | 同上 | **blocked** |
| F4.3 | 依赖 F4.1+F4.2+用户确认 | — | — | 同上 | **blocked** |

注：规划的日志路径 `docs/real_grasp_error_investigation/log/investigation_20260818_*.log` 不存在（该目录本轮创建，此前为空）；阶段 D/E 实际日志在项目根 `log/investigation_20260818_*.log`（本会话生成，路径已知，未做目录扫描）。stageD_report.json 位于 `/tmp/stageD_report.json`（F3 允许"若存在"）。

## 4. 修正后的误差分解（E_plan / E_tool / E_total）

| 量 | 值 | 坐标系 | 说明 |
|---|---|---|---|
| E_plan（FK link6 − 命令 link6） | 均值 norm 9.1mm，max 13.0mm，XYZ=[-3.6,+3.1,+3.2]±[5.7,3.1,3.5] | base_link | F3：6 次 D5 grasp_like；方向随机 = MoveIt ±1cm 盒内随机落点 |
| E_tool（FK link7 − 期望 link7，joint7=0） | 13.0mm | base_link | F2：E1 修正基准（FK 用 mock 关节=规划终点，joint7=0） |
| E_total（实测指尖 vs 目标物中心） | ≈20mm（1 次，方向未细分） | 现场 | E2 用户量测 |
| 原 E_wrong（FK7 − TCP，错误基准） | 13.7mm | base_link | 含 4.2mm 几何固有差（link7 铰点 ≠ TCP 指尖平面） |

分解：E_total ≈ 20mm 中，**E_plan（MoveIt 容差）≈ 9–13mm** 为最大已知项；执行层 ≤3mm（D5）；余 ~5–10mm 未归属（感知绝对位置 或 TCP 指尖几何未知部分；手眼间接路径未排除）。

## 5. 是否支持收紧 MoveIt 容差 —— **支持（有证据）**

- E_plan 量级（9.1 均值 / 13.0 最大）与 PositionConstraint ±1cm 盒（moveit_adapter.py L205-215）一致；
- 方向随机（X 分量 +8.6 / −6.8 / −5.1 / −3.4 / −6.5，无固定符号）符合"容差盒内随机落点"模型，不指向系统性外参/TCP 偏差；
- 对照：同一配置下**关节目标路径** E_plan 仅 <1mm（D5 ready/zero 关节差 ≤0.0045 rad），位姿目标路径独有 9–13mm；
- 收紧方案：位置盒 0.02→0.005m（±2.5mm）、JointConstraint ±0.01→±0.003 rad、姿态容差 0.05→0.02 rad。预期 E_plan 降至 ≤3mm（执行层水平）。此修改仅改 moveit_adapter.py 约束数值，不改变任何变换/标定。

## 6. TCP 实测与 A/B —— **blocked，不修改 TCP 常量**

- F4.1 无物理测量（无 link6 基准面→指尖夹持中心读数）→ F4.2 无 B 值 → F4.3 未授权。`tcp_ab_test.json` 状态 blocked。
- 已知几何：代码 0.140m vs URDF 铰点 0.1358m（差 4.2mm）+ 指尖超出铰点部分（link7/link8.STL，未实测）。即使按最坏估计（指尖 +2cm），TCP 偏移最多解释沿前向轴 ~20mm，但 E_plan 已占 9–13mm 且方向随机——**当前数据不支持把 TCP 作为首要修改点**。
- 若做 F4：先测 L_tip_measured（≥3 次读数），再做离线 A/B（F4.2），最后用户确认真机单次 A/B（F4.3，dry_run_max_stage:=grasp，velocity 0.1）。

## 7. 是否必须重新标定手眼 —— **暂不，但标记为关键未知**

- 矩阵数值健康（det=1.0000000000，正交 1.2e-15，A1）；无证据表明运行时内参错误。
- **唯一风险路径**：手眼标定可能用了 stored K（F1.3 = unknown）。若下一步真值验证（标定物 ≥3 点）显示感知绝对位置系统性偏差且随深度线性放大，才进入手眼重新验证；否则不动手眼 YAML。
- 重新验证的正确姿势：用当前 runtime K 做独立手眼验证（一次性、只读对比），不是直接替换 YAML。

## 8. 下一步（最多两个任务）

1. **收紧 MoveIt 位姿约束并复测**（先于一切）：改 moveit_adapter.py 约束数值（盒 ±1cm→±2.5mm 等）→ 用阶段 D 到位脚本回归（E_plan 通过标准 <5mm）→ 若 E_total 降至 ~10mm 则收敛；若仍 ~20mm 剩余量归属感知/TCP（进入任务 2）。
2. **感知绝对位置真值验证**：放置 ≥3 个已知 base 坐标的标定点（人工量测），对同一帧比对感知反投影 vs 实测（E_perception/E_transform 分离）；同时采集手眼标定来源证据（标定程序/日志中使用的 K）。F4 的 TCP 实测并入本任务后顺序执行。

> 附：`README.md`（19:28 版）已索引本轮文件；`frame_T0_20260818/` 与 `frame_20260816_2231/` 数据未变。
