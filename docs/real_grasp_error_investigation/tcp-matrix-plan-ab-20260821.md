# 2026-08-21 新旧 TCP 朝向矩阵 plan-only A/B 报告

## 结论

新矩阵（本文称 `semantic`）明确更符合 GraspNet 输出参数的物理语义，应作为执行侧的 TCP 朝向转换矩阵：

```python
np.array([
    [ 0.0, 0.0, 1.0],
    [ 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
```

它满足：

```text
TCP +Z = GraspNet +X（接近/进深方向）
TCP +Y = GraspNet +Y（两指宽度轴，符号按双指对称性处理）
TCP +X = -GraspNet +Z
```

旧矩阵把 TCP `+Z` 映射到 GraspNet `+Z`，与真正的接近方向 GraspNet `+X` 正交；同时把实际手指开合轴映射到 GraspNet 接近轴，而不是宽度轴。

## 安全边界

- Piper CAN：`can0`，1 Mbps，`ERROR-ACTIVE`。
- Piper 驱动已连接，`Enable status: True`。
- MoveIt 参数：`allow_trajectory_execution=False`。
- `/joint_states` 发布者数量：0；Piper 驱动没有收到规划状态作为控制命令。
- A/B 脚本只调用 `MoveGroupInterface.plan()`；该接口显式设置 `planning_options.plan_only=True`。
- 未调用 `execute()`，未发送夹爪命令。
- 实验前后六关节反馈位置一致，未检测到机械臂运动。

## 固定输入

```text
CSV：/tmp/tcp_matrix_ab_20260821_frozen.csv
CSV SHA256：17e3e1fe1f8896db592cc1549cdb10c024705854984131d82865a625e1f99615
候选数：62（规划对照取按 score 排序后的前 20 个）

handeye：camera_data/handeye_eye_to_hand.yaml
handeye SHA256：14dcacc87184f1405c9d6c63ce16cfb9dcd18fb46cd7c93f2e670fb9dad4bd67

TCP offset：0.1358 m
grasp_z_offset：0.02 m
每次规划预算：0.5 s
规划起点：同一份 /joint_states_feedback 真机反馈
```

两组只改变 `R_grasp_tcp`，其余输入、候选、手眼矩阵、TCP 长度、位置偏移、规划起点和 MoveIt 配置完全相同。

## 20 个候选的 plan-only 对比

| 指标 | 旧矩阵 legacy | 新矩阵 semantic |
|---|---:|---:|
| `dot(TCP +Z, GraspNet +X)` 均值 | ≈ 0 | 1.000000 |
| `abs(dot(finger opening, GraspNet +Y))` 均值 | ≈ 0 | 1.000000 |
| TCP 向下分量均值 | 0.166 | 0.883 |
| 接近轴相对竖直向下倾角均值 | 79.92° | 25.02° |
| `approach_down >= 0.70` | 1/20 | 19/20 |
| MoveIt 规划成功 | 3/20（15%） | 5/20（25%） |
| 成功规划的终点位置误差均值 | 9.40 mm | 10.15 mm |
| 成功规划的终点姿态误差均值 | 2.20° | 3.09° |

规划终点误差受当前 MoveIt 位姿容差影响，两组量级相近，不能用于裁决固定轴语义。第一轮中新矩阵的规划成功率由 15% 增至 25%；但规划成功率只表示给定候选的可达性，物理语义裁决仍以轴不变量为准。

## 全部 62 个候选的几何对比

| 指标 | 旧矩阵 legacy | 新矩阵 semantic |
|---|---:|---:|
| 接近轴语义对齐范围 | 约 `[-5.4e-8, 5.3e-8]` | `[0.99999996, 1.00000013]` |
| 开合轴语义对齐范围 | 约 `[0, 3.4e-8]` | `[0.99999981, 1.00000016]` |
| TCP 向下分量均值 | 0.081 | 0.807 |
| `approach_down >= 0.70` | 10/62 | 48/62 |
| TCP 向下分量范围 | `[-0.967, 0.825]` | `[0.118, 0.996]` |

新矩阵对所有候选都精确保持 GraspNet 的 approaching/width 轴语义；旧矩阵对所有候选都将这两对轴映射为正交关系。

## 第二轮独立复测

第二轮从持续运行的 Docker 推理中重新冻结了一帧，并对新的前 20 个候选重复同一套 plan-only 对照。机械臂起止反馈完全一致，仍未调用执行或夹爪接口。

```text
CSV：/tmp/tcp_matrix_ab_20260821_run2_frozen.csv
CSV SHA256：404c88ff5a7747445630c5d7bd0f0169ded6458425293bdc43a23344455b7a01
候选数：71（规划对照取前 20 个）
handeye SHA256：14dcacc87184f1405c9d6c63ce16cfb9dcd18fb46cd7c93f2e670fb9dad4bd67
```

| 指标 | 旧矩阵 legacy | 新矩阵 semantic |
|---|---:|---:|
| `dot(TCP +Z, GraspNet +X)` 均值 | ≈ 0 | 1.000000 |
| `abs(dot(finger opening, GraspNet +Y))` 均值 | ≈ 0 | 1.000000 |
| TCP 向下分量均值（前 20） | 0.167 | 0.867 |
| 接近轴相对竖直向下倾角均值 | 79.96° | 27.40° |
| `approach_down >= 0.70`（前 20） | 1/20 | 18/20 |
| MoveIt 规划成功 | 7/20（35%） | 5/20（25%） |
| 成功规划的终点位置误差均值 | 8.89 mm | 9.44 mm |
| 成功规划的终点姿态误差均值 | 3.13° | 2.94° |

全部 71 个候选中，旧矩阵只有 8/71 满足 `approach_down >= 0.70`，新矩阵为 53/71；向下分量均值分别为 0.067 和 0.814。新矩阵的接近轴和开合轴对齐值在全部候选上仍约等于 1，旧矩阵仍约等于 0。

两轮合计每种矩阵各规划 40 次，规划成功数恰好都是 10/40（25%）。这说明单帧中的 7/20 与 5/20 反转来自候选集合和 OMPL 随机性，不能用来决定固定坐标轴约定。重复实验真正稳定的结果是：新矩阵两轮都严格满足 GraspNet 的 approach/width 语义，并显著增加朝下抓取姿态；旧矩阵两轮都把对应轴旋转成正交关系。因此复测不改变结论，反而排除了“用单次规划成功率选矩阵”的干扰。

## 限制

- MoveIt 启动时缺少 DepthImage Octomap updater，因此本次规划包含机器人自身碰撞和模型约束，但没有真实桌面/物体的深度碰撞场景。
- 每个候选只规划一次，规划成功率包含 OMPL 随机性；这不影响固定轴语义结论。
- 本轮没有修改运行时 `grasp_transform.py` 中的旧矩阵，只新增了可复现的 A/B 工具和实验日志。

## 产物

- A/B 工具：`scripts/tcp_matrix_plan_ab.py`
- 完整逐候选 JSON：`docs/real_grasp_error_investigation/log/tcp_matrix_plan_ab_20260821.json`
- 第二轮完整逐候选 JSON：`docs/real_grasp_error_investigation/log/tcp_matrix_plan_ab_20260821_run2.json`
