# TCP 更新报告（URDF 权威值 0.1358m）

> 按 `06-urdf-tcp-authoritative-plan.md` 执行 U0–U4。所有 JSON 位于 `docs/real_grasp_error_investigation/log/`。

## 1. URDF 权威 TCP 长度与来源

```text
TCP_OFFSET_FROM_LINK6_Z = 0.1358 m
来源：piper_description.urdf
  joint6_to_gripper_base（fixed，origin xyz=0 0 0）：L375-383 —— gripper_base 与 link6 重合
  joint7（prismatic，origin xyz=0 0 0.1358）：L425-440
  joint8（prismatic，origin xyz=0 0 0.1358）：L483-499
语义：抓取 TCP 沿 TCP 局部 Z 轴；TCP→link6 折算 = 右乘 T(0,0,-0.1358)；
      joint7 的 0.035m 开度与 rpy=1.5708 只用于 link7/link8 FK 验证，不混入 TCP 主偏移。
```

## 2. 新旧值

| 项 | 旧 | 新 |
|---|---|---|
| TCP_OFFSET_FROM_LINK6_Z | 0.140 m | **0.1358 m** |
| 差 | Δ = 0.0042 m（沿 TCP Z 轴前向） | |

## 3. 修改的文件、函数和测试

| 文件 | 修改点 | 函数/位置 |
|---|---|---|
| `grasp_precheck.py` | 常量 0.14→0.1358；注释更新（删除"仿真近似"） | L54 `TCP_OFFSET_FROM_LINK6_Z`；`_tcp_to_link6_matrix` L139-141（逻辑未变，引用常量） |
| `grasp_execute.py` | 注释更新 | `_tcp_pose_to_link6` docstring、模块注释 |
| `grasp_executor.py` | 注释更新 | L465 |
| `test/test_grasp_execute_units.py` | 断言 0.8-0.14→0.8-0.1358、0.2+0.14→0.2+0.1358 | `test_tcp_pose_to_link6_pure_translation`、`test_tcp_pose_to_link6_rotated` |
| `test/test_grasp_precheck.py` | 无 0.14 命中，未修改 | — |

未修改：MoveIt 容差（moveit_adapter.py）、手眼 YAML、GRASPNET_TO_PIPER_TCP_R（06 §10）。

## 4. 测试命令、退出码和结果

| 命令 | 退出码 | 结果 |
|---|---|---|
| `colcon build --packages-select real_piper_perception` | 0 | 构建成功 |
| `pytest test/test_grasp_execute_units.py test/test_grasp_precheck.py -q` | 0 | **20 passed** |
| `pytest test/ -q` | 1 | 51 passed, 1 failed（`test_sdk_safe_execute.py::test_box_clearance`，`-0.0 < 0` 浮点符号，文件未修改、与 TCP 无共享路径，**预存失败非回归**） |
| scan_only（U3，csv=/tmp/stageE_frozen.csv） | 0 | candidates=62 tested=41 gate_rejected=21 **候选[44] 全过**（approach_pass=3, grasp_pass=3），臂不动 |

## 5. 冻结候选 [44] 离线 A/B 位姿差异（U2）

```text
p_tcp_command = [585.4, 82.5, 35.6] mm（grasp；approach=[585.4,82.5,85.6]、lift=[585.4,82.5,135.6]，TCP 帧级 A/B 相同）
p_link6_target_A（0.140）= [484.090, -11.113, 59.535] mm
p_link6_target_B（0.1358）= [487.129, -8.305, 58.817] mm
Δ(B-A) = [+3.039, +2.808, -0.718] mm，norm = 4.20 mm = 0.0042 m × TCP Z 轴（[0.724, 0.669, -0.171]）✓ 与理论一致
```

## 6. scan_only 预检前后差异（U3）

- 候选集/统计完全相同（tested=41, gate_rejected=21, 候选[44] 全过）——冻结输入相同，TCP 偏移 4.2mm 不改变可达性判定。
- 新 link6 目标较旧值沿 TCP Z 前向 +4.2mm（见 §5）。
- 无失败阶段新增（lift 段历史裁决不检，恒 0）。

## 7. 真机 dry-run（U4，2026-08-18 20:09，用户授权）

**已执行**（06 §8 条件满足）。`scan_only:=false dry_run_max_stage:=grasp csv_path:=/tmp/stageE_frozen.csv`，velocity 0.1，不闭合夹爪，前后回 zero，单次。

```text
结果：success=True, status=dry_stop, stage='grasp', candidate_index=44
FK link7（执行后回 zero 前落盘）= [591.729, 82.419, 31.867] mm
实测指尖 vs 目标物中心：未采集（06 §8 禁止重试，臂已收尾回 zero；E2 旧代码值 ≈20mm 作参考）
```

## 8. command / link6 / link7 / FK / 实测误差（分开记录）

| 量 | 旧代码（E1，0.140） | 新代码（U4，0.1358） | 坐标系 |
|---|---|---|---|
| command TCP | [585.4, 82.5, 35.6] | [585.4, 82.5, 35.6]（不变） | base_link |
| command link6 | [484.090, -11.113, 59.535] | [487.129, -8.305, 58.817] | base_link |
| FK link7 | [569.329, 79.430, 35.749] | [591.729, 82.419, 31.867] | base_link |
| link7 期望（joint7=0，铰点=link6+0.1358·Z） | TCP−0.0042·Z=[582.4,79.7,36.3] | TCP=[585.4,82.5,35.6] | base_link |
| **E_tool（FK7 − 期望7）** | **13.0 mm** | **7.3 mm** | base_link |
| 实测指尖 vs 目标 | ≈20 mm（E2，用户粗测） | 未采集（禁重试） | 现场 |

注意：link7 铰点 ≠ TCP 指尖中心（F2 已证）；FK 用 mock 关节（=MoveIt 规划终点，joint7=0），E_tool 反映"规划终点 vs 命令目标"的 link7 级偏差。

## 9. TCP 修改是否使误差沿 TCP Z 方向按理论值变化

- **方向：是**。E_tool 从 13.0mm → 7.3mm，下降 5.7mm，与 TCP 偏移修正 4.2mm 同向（link7 铰点在 joint7=0 时 = TCP − (offset−0.1358)·Z，偏移减小使期望点沿 +Z 前向移动 4.2mm）。
- **量级：方向一致但样本 n=1**。MoveIt 规划容差（±1cm 盒）使单次 FK 有 ±10mm 级随机性（阶段 D：同一目标 6 次 E_plan 7–13mm），单次 5.7mm 改善**不足以单独归因** TCP（06 §9.10）。
- 实测误差（指尖 vs 目标）未采集，无法验证 E_total 变化。

## 10. 误差分离（TCP vs MoveIt vs 其他）

- TCP 修改只改变 link6 目标（沿 TCP Z ±4.2mm）；MoveIt 容差、手眼、感知均未动。
- E_tool 新值 7.3mm 中：执行层 ≤3mm（阶段 D）+ MoveIt 容差残余（±10mm 随机）+ TCP 修正后残余——三者无法在 n=1 下分离。
- 上一轮结论不变：**MoveIt 位姿约束 ±1cm 盒是已确定的最大单误差源（E_plan 9-13mm，follow-up-report.md §5）**，优先于 TCP 处理。

## 11. 最终状态

**效果未验证**（TCP 修改方向与理论一致、代码/测试/预检全部通过，但真机 n=1 且 MoveIt 容差随机性覆盖 4.2mm 效应；实测指尖未采集）。

## 12. 下一步（最多 2 项）

1. **收紧 MoveIt 位姿约束**（moveit_adapter.py：位置盒 ±1cm→±2.5mm、JointConstraint ±0.01→±0.003 rad）→ 阶段 D 回归（E_plan <5mm）→ 用冻结 CSV 候选[44] 重跑 U4 流程（届时 n≥2 可分离 TCP 与容差效应）。
2. **感知绝对位置真值验证**：≥3 个已知 base 坐标标定点分离 E_perception/E_transform（手眼间接路径 unknown 的关键未知项）。
