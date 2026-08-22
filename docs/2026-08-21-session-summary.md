# 2026-08-21 新 TCP、最佳候选页面、真机抓取与夹爪宽度会话总结

> 本文档是下一次对话的完整交接日志。它记录本次对话的需求演变、昨日计划复核、
> 两轮旧/新 TCP 只规划 A/B、代码修改、一次真机抓取、失败根因、夹爪 70 mm 行程
> 推算与接入、测试结果、运行环境和下一步顺序。
>
> **最高优先级事实：本轮机械臂轨迹执行成功，但物理抓取失败。**
> `grasp_result_new_tcp_20260821.json` 中的 `success=true` 是夹爪闭合瞬态反馈造成的
> 应用层假阳性，不得在下一次对话中当成真实抓取成功。

---

## 0. 下一次对话首先读取的结论

```text
旧 TCP 朝向矩阵：物理轴语义错误，TCP +Z 与 GraspNet approach 近似正交。
新 TCP 朝向矩阵：轴语义正确，TCP +Z = GraspNet +X approach。
Docker 最佳候选页面：已只显示机械臂实际选择的唯一候选。
本轮真机运动：approach、grasp、回 zero 均完成。
本轮真实夹持：失败，夹爪最终回到 4.3mm 空夹全闭基线。
应用 success=true：假阳性，原因是读取了闭合过程首条 24.0mm 瞬态反馈。
夹爪最大总开口：70mm，对应 GripperCtrl(70000, ...)。
70mm 宽度门控：已接入候选层和执行层，已 build/单测，尚未重新真机执行。
当前抓取执行器：已停止，/grasp_execute 服务不存在，不会误触发。
禁止自动重试：先修复笛卡尔响应字段和夹爪稳定反馈判定。
```

详细真机报告：
`docs/real_grasp_error_investigation/real-grasp-new-tcp-20260821.md`。

---

## 1. 本次对话需求演变

用户请求按时间顺序为：

1. 检查昨日规划，需要修改哪些内容，以及最必须修改的内容。
2. 用户指出已更新新的手眼标定文件，要求重新分析昨日日志和计划，并确认最重要的
   是否是 TCP 朝向转换矩阵。
3. 要求激活/连接机械臂、开启 Docker 可视化推理，并对旧 TCP 与新 TCP 做对照实验；
   **只规划、不执行、不抓取**，分析哪一矩阵更符合 GraspNet 参数形式。
4. 要求再次重复一次只规划对照并重新分析。
5. 要求优化 Docker 页面：预检成功候选页面只显示机械臂最终执行的最佳候选；切换
   新 TCP 朝向矩阵；执行一次真机抓取并分析日志。
6. 追问最终候选的选取模式和刚才候选的得分。
7. 要求依据 `piper_sdk` 夹爪控制 Demo 推算最大开合宽度，构建测试任务并将结果加入
   真机抓取。
8. 最后要求将整个对话完整写入日志，供下一次对话恢复。

本轮严格遵守了“先 plan-only，再真机一次；发现假阳性后不自动重试”的安全边界。

---

## 2. 昨日计划复核与最必须修改项

复核 `docs/2026-08-20-session-summary.md` 后确认，昨日最关键问题不是手眼平移或 TCP
长度，而是固定 TCP 朝向换基矩阵的物理语义错误。

### 2.1 旧矩阵

```python
M_legacy = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])
```

右乘 `R_base_tcp = R_base_grasp @ M` 后，矩阵每一列表示 TCP 对应轴在 GraspNet
坐标系中的表达。旧矩阵的核心结果是：

```text
TCP +Z = GraspNet +Z
TCP +Y = -GraspNet +X
```

GraspNet 的 `+X` 才是 approach/depth 方向，因此旧矩阵没有把工具前进轴 `TCP +Z`
对齐到 GraspNet approach。历史上“夹爪开合轴朝下”曾被错误当作“工具接近轴朝下”，
导致错误矩阵在代数自洽测试中长期保留。

### 2.2 新语义矩阵

```python
M_semantic = np.array([
    [ 0.0, 0.0, 1.0],
    [ 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
```

轴语义：

```text
TCP +Z = GraspNet +X（approach/depth）
TCP +Y = GraspNet +Y（width）
TCP +X = -GraspNet +Z
```

该矩阵与 Docker 推理侧原有矩阵逐元素一致，行列式为 `+1`。因此本轮确认：

> 新手眼标定影响 camera→base 的绝对位姿，但固定工具轴的语义错误不会被新手眼外参
> 自动修正；最必须的代码修改确实是 TCP 朝向转换矩阵。

---

## 3. 两轮旧/新 TCP 严格 plan-only A/B

新增/使用脚本：`scripts/tcp_matrix_plan_ab.py`。

安全条件均写入结果文件：

```text
plan_only=true
execute_called=false
gripper_commanded=false
move_group_allow_trajectory_execution=false
```

两组使用同一冻结 CSV、同一手眼、同一真实反馈起始关节、同一 TCP 长度与位置偏置；
仅替换固定旋转矩阵。A/B 脚本按 score 降序取前20个候选，这一点与当前真机执行器
按 score 升序的策略不同；A/B 目的是比较固定轴语义，不是复现最终候选排序。

### 3.1 第一次 A/B（16:31）

结果：`log/tcp_matrix_plan_ab_20260821.json`。

| 指标 | 旧矩阵 | 新矩阵 |
|---|---:|---:|
| 候选数 | 20 | 20 |
| 规划成功 | 3/20 | 5/20 |
| TCP forward·GraspNet approach 均值 | ≈0 | 1.000000 |
| 开合轴与 GraspNet width 绝对对齐均值 | ≈0 | 1.000000 |
| `approach_down` 均值 | 0.166 | 0.883 |
| `approach_down>=0.70` | 1/20 | 19/20 |
| 平均倾角 | 79.9° | 25.0° |

### 3.2 第二次 A/B（16:36）

结果：`log/tcp_matrix_plan_ab_20260821_run2.json`。

| 指标 | 旧矩阵 | 新矩阵 |
|---|---:|---:|
| 候选数 | 20 | 20 |
| 规划成功 | 7/20 | 5/20 |
| TCP forward·GraspNet approach 均值 | ≈0 | 1.000000 |
| 开合轴与 GraspNet width 绝对对齐均值 | ≈0 | 1.000000 |
| `approach_down` 均值 | 0.167 | 0.867 |
| `approach_down>=0.70` | 1/20 | 18/20 |
| 平均倾角 | 80.0° | 27.4° |

第二轮中旧矩阵规划成功数反而更高，进一步说明不能用“IK 是否有解”决定固定坐标轴
关系。轴语义由 GraspNet 定义、工具坐标定义和真实安装决定；不可达时应换候选或拒绝，
不能修改正确的固定换基矩阵来换取更多规划解。

两轮稳定结论：旧矩阵的工具前进轴与 approach 近似正交；新矩阵对齐度约为1.0，
因此选择新矩阵。

---

## 4. 重大代码修改

### 4.1 执行侧切换新 TCP 矩阵

文件：
`src/real_piper_perception/real_piper_perception/grasp_transform.py`

- 将 `GRASPNET_TO_PIPER_TCP_R` 从旧矩阵切换为新语义矩阵。
- 文档化三个轴的物理关系。
- 测试增加矩阵精确值、正交性、行列式和轴语义断言。

推理侧：`graspnet-baseline/inference_from_camera.py` 使用相同矩阵；本轮通过 AST/导入
逐元素核对执行侧与推理侧一致。

### 4.2 新增 TCP 接近方向安全门

文件：

- `src/real_piper_perception/real_piper_perception/grasp_executor.py`
- `src/real_piper_perception/launch/grasp_execute.launch.py`

新增参数：

```text
min_approach_down = 0.70
approach_down = -R_base_tcp[2,2]
```

只有 approach、grasp 规划均通过，且 `approach_down>=0.70` 的候选才进入 approved。
在全部安全可规划候选中选择 `approach_down` 最大者。预检统计新增
`orientation_rejected`。

### 4.3 Docker 页面只显示实际执行候选

修改文件：

- `graspnet-baseline/precheck_match.py`
- `graspnet-baseline/inference_from_camera.py`
- `test/test_precheck_match.py`

行为变化：

1. 页面从“预检成功候选”改名为“机械臂执行最佳候选”。
2. 只读取并显示 `precheck_results.json.selected`，不再画全部 `approved`。
3. 匹配失败时显示无匹配状态，不再回退显示全候选。
4. 在 NMS 后全部候选中搜索 selected，不受主页面 `top_k=50` 限制。
5. 宽松匹配阈值命中多个候选时，按位置距离、姿态角、序号选唯一最近项。
6. 保存页面到 `shared_camera_data/result_selected.png`。
7. 新增按 selected 位姿而非仅 JSON mtime 的缓存：执行过程若只重写同一 selected
   和 timestamp，保留已匹配成功的单候选画面，避免短时推理波动把页面覆盖为空。

最终 Docker 启动首帧日志：

```text
[precheck] loaded: approved=11 selected=16
[precheck] 只显示机械臂执行最佳候选 1/64（rank=16）
```

最终截图中只有一个蓝色抓手，标注“机械臂执行最佳候选: rank 16”。固化截图：
`docs/real_grasp_error_investigation/log/result_selected_new_tcp_20260821.png`。

### 4.4 Piper SDK 小夹爪 70mm 限值

新增文件：

- `src/real_piper_perception/real_piper_perception/gripper_limits.py`
- `scripts/check_piper_gripper_width.py`
- `src/real_piper_perception/test/test_gripper_limits.py`

修改文件：

- `src/real_piper_perception/real_piper_perception/grasp_execute.py`
- `src/real_piper_perception/real_piper_perception/grasp_executor.py`

推算证据：

1. `piper_sdk 0.6.2/demo/V2/piper_ctrl_gripper.py` 用
   `0.05 * 1000 * 1000` 生成 50mm 示例命令，证明 1 SDK unit = 0.001mm。
2. SDK `C_PiperParamManager` 默认 `gripper_range=[0.0,0.07]m`。
3. SDK 参数协议明确：小夹爪70mm，大夹爪100mm。
4. 本机 URDF/SRDF：joint7每指35mm，总开口70mm。
5. 既有真机反馈：全开69.7mm，与70mm理论上限一致。

离线任务输出：

```json
{
  "hardware_commanded": false,
  "demo_example_opening_m": 0.05,
  "demo_example_command": 50000,
  "sdk_default_range_m": [0.0, 0.07],
  "inferred_small_gripper_max_opening_mm": 70.0,
  "inferred_gripper_ctrl_command": 70000
}
```

接入真机抓取的两层保护：

1. 候选层：`grasp_width + gripper_margin > 0.070m` 时跳过该候选，继续遍历。
2. 执行层：即使绕过上层，在任何张爪或机械臂运动前再次检查；超宽、负值、NaN、
   负余量均 `stage=gripper_width` fail-closed，且不触碰 SDK/机械臂。
3. 最大张开统一使用 `GRIPPER_MAX_OPENING_M=0.070`，转换为
   `GripperCtrl(70000,1000,0x01,0)`。

注意：70mm门控是在本轮真机尝试之后加入，已离线测试并构建，但尚未再次启动执行器
做 scan-only 或真机验证。本轮 rank16 所需开口为57.388mm，小于70mm，所以按新门控
仍可通过。

---

## 5. 当前最终候选选取模式

当前执行器不是“最高 GraspNet score 优先”，而是“可执行性与接近方向优先”：

1. 每次只取最新单帧候选。
2. `rank_candidates` 按 score **升序**排列，最多检查150个。
3. `score >= grasp_score_min`；当前默认/本轮为0.0，相当于不设质量下限。
4. 新增宽度门：预测宽度+5mm余量必须在0–70mm内。
5. self-consistency：物体中心与抓取中心横向距离不超过80mm。
6. handeye + 新 TCP 矩阵生成 approach/grasp 位姿。
7. approach、grasp MoveIt 预检均通过。
8. `approach_down>=0.70`。
9. 在全部通过项中选择 `approach_down` 最大者；完全相同时保留升序遍历先出现者。

页面的最近邻匹配只决定显示哪个抓手，不参与执行选择。

### 5.1 本轮 rank16 得分

```text
候选总数：91
selected_index：16（0-based，score升序后的第17个）
score：0.049211588
全体 score 最小：0.011408234
全体 score 中位数：0.088710181
全体 score 最大：0.412049949
预测宽度：0.052387975m = 52.388mm
宽度+5mm余量：57.388mm < 70mm
approach_down：0.968222
```

该候选属于低分候选。它胜出不是因为 score 高，而是因为规划通过并且在 approved
集合中 `approach_down` 最大。若下一轮目标是兼顾 GraspNet 质量，现有“score升序且
阈值0”的策略需要另行设计综合评分，不应把当前 rank16 描述为“最高置信度候选”。

---

## 6. 新 TCP scan-only 预检

冻结输入：

```text
CSV：/tmp/grasp_execute_20260821_frozen.csv
固化副本：docs/real_grasp_error_investigation/log/grasp_execute_frozen_20260821.csv
CSV SHA256：9e062ec12b4817aecbdecd15b6bdf36c8b3aaf3db46ec71d4f284987b81c0f78
候选数：91

handeye：camera_data/handeye_eye_to_hand.yaml
handeye SHA256：14dcacc87184f1405c9d6c63ce16cfb9dcd18fb46cd7c93f2e670fb9dad4bd67
TCP offset：0.1358m
grasp_z_offset：0.02m
min_approach_down：0.70
```

结果：

```text
candidates=91
tested=91
gate_rejected=0
orientation_rejected=5
approach_pass=18
grasp_pass=16
lift_pass=0
approved=11
selected=16
```

该次 scan-only 在新增70mm门控之前完成；由于 rank16 所需开口57.388mm，结论不受
影响。scan-only未移动机械臂、未发送夹爪命令。

---

## 7. 一次真实抓取执行

### 7.1 安全配置

```text
scan_only=false
速度比例：0.05
冻结 CSV：同上，避免推理时域变化
执行次数：1
自动重试：无
```

执行前：

- can0 1Mbps，ERROR-ACTIVE，bus-errors/error-warn/error-pass/bus-off均0。
- Piper `ctrl_mode=1`、`arm_status=0`、`err_code=0`，六轴通信错误均false。
- arm_controller、gripper_controller、joint_state_broadcaster均active。
- MoveIt允许轨迹执行。
- 真机六轴反馈位于zero附近。

### 7.2 实际执行候选

```text
rank：16
name：target
score：0.049211588
GraspNet width：0.052387975m
camera XYZ：[-0.009641690, -0.016714048, 0.537999988]m
command TCP XYZ：[0.490150142, 0.023703814, 0.015590200]m
command TCP quaternion wxyz：[-0.106737458, 0.723542415, -0.678673305, 0.067052481]
command link6 XYZ：[0.457298716, 0.015088024, 0.147074769]m
approach_down：0.968222160
```

### 7.3 轨迹时序

MoveIt三次执行均返回 `SUCCEEDED`：

1. approach：16:55:46.548开始，16:55:49.527完成。
2. grasp：16:55:49.658开始，16:55:50.537完成。
3. 回zero：16:55:51.788开始，16:56:03.017完成。

机械臂确实完成运动并安全回到zero附近，但这不等于抓住物体。

### 7.4 新 TCP 姿态的执行侧验证

回zero前MoveIt FK：

```text
link6/TCP 前进轴 = [+0.26, +0.11, -0.96]
FK link7 XYZ = [0.481034, 0.037562, 0.023254]m
FK link7 - command TCP = [-9.1, +13.9, +7.7]mm
误差范数 = 18.3mm
```

工具前进轴基座Z分量约-0.96，与 `approach_down=0.968`一致，说明新矩阵在规划/
命令终点实现了 GraspNet approach 朝下语义。

限制：该FK读取MoveIt `/joint_states` FakeSystem/控制器命令状态，不是独立真机
`/joint_states_feedback`。下一轮若要严格计算真实末端误差，必须在grasp终点同步保存
真实关节反馈并离线FK。

---

## 8. 真实抓取结果：抓空，应用假阳性

### 8.1 夹爪时序

```text
full_open = 69.7mm
full_closed = 4.3mm
16:55:51.559 闭合命令发送完成
16:55:51.580 第一条“新”反馈 = 24.0mm
应用立即把24.0mm分类为grasped/success
回zero后最终真实反馈 = 4.3mm
```

4.3mm正好等于本轮空夹全闭基线，因此夹爪中没有物体。

### 8.2 目标位置前后对比

冻结帧目标中心：

```text
[0.009822106, 0.000547592, 0.567993453]m（camera）
```

执行后重新感知中心约：

```text
[0.009885460, 0.001156802, 0.571542149]m
```

差值约 `[+0.06,+0.61,+3.55]mm`，范数3.60mm，处于深度/分割波动量级，不支持
物体被抓走或搬动。

### 8.3 假阳性根因

`GraspExecutor._read_gripper_feedback_wait()` 当前只等待反馈时间戳更新，收到第一条
新样本就返回，没有等待夹爪停止和开度稳定。24mm是从69.7mm向4.3mm闭合过程中的
中间值，不是被物体阻挡后的稳定值。

应用固化结果：

```json
{
  "success": true,
  "status": "success",
  "actual_width": 0.024,
  "cmd_width": 0.057387975,
  "candidate_index": 16
}
```

该JSON只保留用于调查，**真实裁决必须覆盖它**：

```text
机械臂运动执行：成功
新 TCP 朝向：符合GraspNet approach语义
夹爪抓取保持：失败（抓空）
应用 success=true：假阳性
```

发现后已停止抓取执行器，未自动重试。

---

## 9. 同时发现但尚未修复的问题

### 9.1 笛卡尔路径响应字段错误

日志：

```text
Computed Cartesian path ... followed 0.000000%
GetCartesianPath_Response object has no attribute 'trajectory'
回退OMPL
```

源码：`src/real_piper_perception/real_piper_perception/moveit_adapter.py` 当前读取
`resp.trajectory`；ROS2 Humble `GetCartesianPath` 响应应读取 `resp.solution`，并检查
`error_code` 与 `fraction`。因此本轮 approach 实际走OMPL，不是预期直线插入。

**本轮只识别问题，没有修改该文件。**

### 9.2 夹爪成功判定缺少稳定窗口

必须改为：

1. 闭合后连续采样至少4–5个反馈。
2. 只有连续样本变化小于0.5–1.0mm才视为稳定。
3. 设置合理总超时，超时fail-closed。
4. 稳定开度必须明显大于full_closed才可初判夹住。
5. 恢复安全抬升/保持后再次读取稳定开度，再写最终success。

**本轮没有修复该逻辑。** 70mm最大开口门控只解决物理行程边界，不解决瞬态假阳性。

### 9.3 缺少真实环境碰撞场景

MoveIt仍缺少 `DepthImageOctomapUpdater`。当前规划有机器人自身碰撞约束，但没有真实
桌面/物体深度碰撞场景。

### 9.4 当前候选分数策略偏低分

当前score升序、阈值0，rank16只有0.0492。历史背景是高分候选常不可达，但现策略把
score只当遍历顺序/最低门，最终完全由approach_down决定。下一步若优化质量，应采用
可解释综合评分（例如规划通过为硬门，score、approach_down、清障距离为软排序），
不能简单宣称当前候选“最佳推理质量”。

### 9.5 宽度门控尚未在线复核

70mm门控已经通过单元测试和colcon build，但新增后未重新启动grasp_executor做
scan-only。下一轮第一步应仅scan-only验证统计和日志，不得直接真机抓取。

---

## 10. 测试、构建及测试过程中的已知现象

最终相关回归：

```text
59 passed in 4.63s
colcon build --packages-select real_piper_perception --symlink-install
Summary: 1 package finished
```

覆盖：

- 唯一selected近邻匹配。
- 新TCP矩阵及轴语义。
- 预检位姿转换。
- 候选解析、排序、自洽。
- 夹爪SDK命令换算。
- 70mm边界、超宽、负值、负余量、NaN。
- 超宽候选在任何硬件动作前fail-closed。
- 抓取执行器反馈与判定相关单测。

过程记录：

- 初次直接运行 `pytest` 命令时系统无该可执行文件；使用 `/usr/bin/python3 -m pytest`
  并设置 `PYTHONPATH` 后正常。
- 一次新增测试用浮点严格相等比较 `0.049999999999999996 == 0.05` 失败，已改为
  `1e-12` 容差，最终通过。
- 曾运行更宽的整个 `src/real_piper_perception/test`，结果58 passed、1 failed；失败是
  未修改的 `test_sdk_safe_execute.py::test_box_clearance`，因为实现返回`-0.0`而测试要求
  `<0`。该问题与本轮TCP、页面和夹爪宽度修改无关，未在本轮处理。
- 工作目录 `.` 不是Git仓库，不能依赖 `git status/diff`
  恢复修改，应使用本文文件清单、hash和逐文件检查。

---

## 11. 本轮文件修改清单

| 文件 | 本轮内容 |
|---|---|
| `scripts/tcp_matrix_plan_ab.py` | 严格plan-only旧/新矩阵A/B，不调用execute和夹爪 |
| `graspnet-baseline/precheck_match.py` | selected唯一最近匹配 |
| `graspnet-baseline/inference_from_camera.py` | 最佳候选单独页面、全候选搜索、selected缓存、保存截图 |
| `src/real_piper_perception/real_piper_perception/grasp_transform.py` | 切换新语义TCP矩阵 |

---

## 12. 后续对话补充：删除宽度门限、连续真机抓取与关机

> 本节记录本文件前述内容之后的完整操作。由于用户后来明确要求“删除宽度门限”，
> 本节中的代码和运行结果是最新状态；若与前文关于70mm候选门控的描述冲突，以本节为准。

### 12.1 用户后续要求

用户随后依次要求：

1. 继续执行此前流程。
2. 删除宽度门限。
3. 重新执行真机抓取，规划成功后直接抓取。
4. 再次重新推理、规划并执行真机抓取。
5. 再次执行新的推理和抓取。
6. 再次执行一遍。
7. 关闭后台进程。
8. 将本次对话全部总结写入日志。

全程每次真机动作均由用户明确请求触发；每一轮只调用一次抓取服务，不做自动重试。

### 12.2 删除70mm宽度门限

修改文件：

- `src/real_piper_perception/real_piper_perception/grasp_execute.py`
- `src/real_piper_perception/test/test_gripper_limits.py`

具体变化：

- 删除 `execute_pick()` 中调用 `fits_small_gripper()` 的执行前 fail-closed 宽度拒绝逻辑。
- 删除对 `fits_small_gripper` 的导入。
- 保留 `GripperCtrl(70000, 1000, 0x01, 0)` 作为全开/标定命令；这不是候选宽度门限。
- 抓取候选的 `grasp_width + margin` 不再因超过70mm被筛掉，最终是否夹住由真实夹爪反馈判定。
- 单元测试从“超70mm必须拒绝”改为验证宽度换算仍正确，并验证执行路径不再在
  `gripper_width` 阶段拒绝超宽候选。

验证结果：

```text
real_piper_perception 相关测试：4 passed
Python 编译检查：通过
./build_ws.sh：成功，3 packages finished
```

### 12.3 真机运行环境

每轮执行前均确认：

```text
can0：UP，ERROR-ACTIVE，bitrate=1000000
Piper 驱动：piper_single_ctrl，can_port=can0，auto_enable=true
MoveIt/RViz：运行，arm_controller、gripper_controller、joint_state_broadcaster active
D435i：相机、d435i_graspnet_bridge、handeye_tf_publisher 运行
Docker GUI 推理：容器 exciting_goldberg，镜像 graspnet-baseline:20.04，运行中
pause_inference：不存在（未冻结推理）
```

执行器统一使用：

```text
scan_only=false
velocity_scale=0.05
enable_self_consistency=true
graspnet_6d_max_candidates=150
graspnet_6d_analytic_only_precheck=false
```

### 12.4 门限删除后的首次真机抓取

该轮使用新的 Docker 帧重新推理和规划，随后直接执行真机抓取。

```text
候选数：94
预检通过：30
selected：39
GraspNet grasp_width：0.0762m
command width：0.0812m（81.2mm，超过70mm但已允许执行）
实际夹爪宽度：0.0273m（27.3mm）
空夹全闭基线：0.0042m（4.2mm）
抓取判定：success
保持判定：success
回zero：成功
服务：success=True
```

本轮日志明确证明删除后的宽度门限生效：81.2mm 指令宽度没有在执行前被拒绝，
机械臂实际完成了接近、抓取和回zero。结果文件随后写入：
`shared_camera_data/grasp_result.json`。

### 12.5 第二次真机抓取

用户要求“重新执行一次，重新推理规划，执行真机抓取”后重新启动执行器并触发一次服务。

```text
新帧候选数：33
预检通过：19
selected：17
score：0.1668
GraspNet grasp_width：0.0845m
command width：0.0895m
实际夹爪宽度：0.0365m（36.5mm）
空夹全闭基线：0.0042m
抓取判定：success
保持判定：success
FK link6 前向 z：-0.65
回zero：成功
服务：success=True
```

服务返回结果：

```json
{
  "timestamp": "2026-08-21T20:11:08",
  "success": true,
  "status": "success",
  "stage": "",
  "actual_width": 0.0365,
  "cmd_width": 0.08953010800000001,
  "candidate_index": 17
}
```

### 12.6 第三次真机抓取

用户再次要求“重新执行新的推理和抓取”后，重新采集/推理/规划并执行一次。

```text
新帧候选数：27
预检通过：9
selected：1
score：0.0550
GraspNet grasp_width：0.0769m
command width：0.0819m
实际夹爪宽度：0.0327m（32.7mm）
空夹全闭基线：0.0042m
抓取判定：success
保持判定：success
FK link6 前向 z：-0.74
回zero：成功
服务：success=True
```

服务与结果文件内容：

```json
{
  "timestamp": "2026-08-21T20:21:12",
  "success": true,
  "status": "success",
  "stage": "",
  "actual_width": 0.0327,
  "cmd_width": 0.081872543,
  "candidate_index": 1,
  "message": "candidate [1] target: pick status=success stage='' actual_width=0.0327 cmd_width=0.0819"
}
```

回zero之后读取到的独立夹爪反馈为约32.4mm，说明夹爪仍保持夹持开度；该轮没有额外
发送释放命令。

### 12.7 最终关闭后台进程

用户要求关闭后台进程后，先核对了进程归属，确认以下均属于本项目：

- `piper_with_gripper_moveit demo.launch.py`、`move_group`、`ros2_control_node`、RViz
- `piper start_single_piper.launch.py`、`piper_single_ctrl`
- `real_piper_perception d435i_graspnet_pipeline.launch.py`
- RealSense 相机、`d435i_graspnet_bridge`、`handeye_tf_publisher`
- Docker 推理容器 `exciting_goldberg`

执行的关停动作：

1. 向三组 ROS launch 进程组发送 `SIGINT`，等待其优雅退出。
2. 执行 `docker stop -t 10 exciting_goldberg`，容器正常停止并因 `--rm` 清理。
3. 执行 `ros2 daemon stop`，关闭 ROS 2 CLI daemon。

最终核验：

```text
项目相关后台进程：无
Docker 运行容器：无
ros2 node list --no-daemon：无实际项目节点
```

`can0` 网络接口仍保留在系统中，但已无 Piper 驱动进程占用；没有额外执行网卡下线。

### 12.8 最新状态与产物

当前机械臂、RViz、相机、MoveIt、Docker 推理和抓取执行器均已关闭。

最新抓取结果：

- 文件：[shared_camera_data/grasp_result.json](../shared_camera_data/grasp_result.json)
- 最新状态：`success=true`
- 最新候选：`candidate_index=1`
- 最新实际夹持宽度：32.7mm

本日志文件：
`docs/2026-08-21-session-summary.md`。
| `src/real_piper_perception/real_piper_perception/grasp_executor.py` | approach_down安全门、方向优选、70mm候选宽度门 |
| `src/real_piper_perception/launch/grasp_execute.launch.py` | 透传min_approach_down |
| `src/real_piper_perception/real_piper_perception/gripper_limits.py` | 新增SDK单位、70mm上限与纯函数门控 |
| `src/real_piper_perception/real_piper_perception/grasp_execute.py` | 70mm统一开爪、执行前二次宽度fail-closed |
| `scripts/check_piper_gripper_width.py` | 新增不连接CAN的离线SDK推算任务 |
| `test/test_precheck_match.py` | selected唯一匹配测试 |
| `src/real_piper_perception/test/test_grasp_transform.py` | 新矩阵精确值与轴语义测试 |
| `src/real_piper_perception/test/test_grasp_precheck.py` | 新矩阵预检测试更新 |
| `src/real_piper_perception/test/test_gripper_limits.py` | 新增Demo换算、70mm和执行前防御测试 |
| `docs/real_grasp_error_investigation/real-grasp-new-tcp-20260821.md` | 真机完整报告及候选/夹爪补充 |
| `docs/2026-08-21-session-summary.md` | 本交接日志 |

未修改但已确认需要下一轮修改：

- `src/real_piper_perception/real_piper_perception/moveit_adapter.py`
- `src/real_piper_perception/real_piper_perception/grasp_execute.py` 中稳定反馈等待逻辑
  （本轮只改了宽度门，未改稳定判定）。

---

## 12. 关键源码 SHA256（日志生成时）

```text
ac65cd955fb4111f84707f1f63e08135db6759ee3ad92978189e8eb3c21aade1  scripts/tcp_matrix_plan_ab.py
5674c67fd853ecb16ba8db343f64ac98d668dba35fc94ef38741fdc81cbaf51d  scripts/check_piper_gripper_width.py
72058fdc3cc4d3db7cfab68ef0b43250759b78b1956c75fb2beeafca2686f4a6  graspnet-baseline/inference_from_camera.py
3def4bed24895d5b23b9bbd1b5845419797d7efdba8c0707f014db970e0e3edf  graspnet-baseline/precheck_match.py
9103c25574bc8f1703ff057e48eeb976c194a1047c9ff1de3b21f080b1dbbe9c  grasp_transform.py
c782295b3bb511d189a410f35b8e8b0bf888cdca500b12aed45f2914f5ca9017  grasp_executor.py
ae746f80f38918cbcfc1c7cfc1172d3807feb9f6a3aff787bfe277b7bd2b81b7  grasp_execute.py
5e19643ef0ba1622b0d20c01c5f697c34a463f7f29af4ecd468802afa7f8ae34  gripper_limits.py
8a1261642ed043e8eab71cc7659dfb33e17e1682c9f1fc245259d58c54f0774a  grasp_execute.launch.py
9d874762606e756324fe92a2b22071f0f3aea1f90b90c0e6632adf950da230bd  test_gripper_limits.py
```

上述短文件名均位于第11节对应路径。

---

## 13. 固化产物与关键 SHA256

| 产物 | SHA256 |
|---|---|
| `log/tcp_matrix_plan_ab_20260821.json` | `34207514fef3f86c40ee7cb1b57c8ed0b0b2596fe061ea044380e94ac9799441` |
| `log/tcp_matrix_plan_ab_20260821_run2.json` | `d374224897cdfb6fb0edf167d655fca67e5d065bc6dc0f4c52cb87e4d2bc1359` |
| `log/grasp_execute_frozen_20260821.csv` | `9e062ec12b4817aecbdecd15b6bdf36c8b3aaf3db46ec71d4f284987b81c0f78` |
| `log/grasp_scanonly_new_tcp_20260821.log` | `8ce54a7b32c8de893cdaa33827e60358873f91a14cdd7cf3e7db802fda45e2c4` |
| `log/grasp_execute_new_tcp_20260821.log` | `574547343f934cdf0774eb53f61d74e1f797684bb7c6cda9ee6b0f1d6c583f1d` |
| `log/move_group_new_tcp_20260821.log` | `f3fb0fc78c7b6d7167f981bd3b0bf60979fc1f649c7a439bb18f30940bd166c1` |
| `log/precheck_results_new_tcp_20260821.json` | `ec7a64a16f62c1e55bb7bd48c5fe5775c2b799a90a564cd06241220ab8c5ac4b` |
| `log/grasp_result_new_tcp_20260821.json` | `439bba7c69b7d1bd1682d7ee973bfeb496ab34e48f8d2b99e7051dfdd9b9e81a` |
| `log/fk_link7_new_tcp_20260821.json` | `fdf39b04e0de3a29decc76073df629671b76cdc714bad08690ca4918663a1e76` |
| `log/result_selected_new_tcp_20260821.png` | `461054b5736dd25ed7753b2cab3dcdcd20023aab4a4e48c6f347c29b66dffe2e` |

产物目录：`docs/real_grasp_error_investigation/log/`。

---

## 14. 会话结束时实时环境状态

采样时间约为2026-08-21 17:17（Asia/Shanghai）。

### 14.1 正在运行

```text
Piper驱动：piper_ctrl_single_node
D435i桥接：d435i_graspnet_bridge
handeye静态TF：handeye_tf_publisher
MoveIt：move_group
Docker/GraspNet GUI推理：inference_from_camera.py
```

### 14.2 已停止

```text
grasp_executor进程：无
/grasp_execute服务：无
```

这意味着页面和感知继续运行，但不会因为服务误触发再次抓取。

### 14.3 机械臂/夹爪反馈

```text
arm_status=0
err_code=0
六轴communication_status均false
joint1=-0.000558208
joint2=+0.002913148
joint3=-0.006750828
joint4=0.0
joint5=0.0
joint6=+0.007413700
gripper=0.0043m
速度均约0
```

机械臂处于近zero位；夹爪为4.3mm空夹全闭状态。

### 14.4 CAN

```text
can0：UP/LOWER_UP
bitrate：1,000,000
state：ERROR-ACTIVE（正常工作态）
restarted=0
bus-errors=0
arbit-lost=0
error-warn=0
error-pass=0
bus-off=0
```

运行进程和PID可能随重启变化，下一次对话必须重新采样，不得只依赖本节PID/状态。

### 14.5 用户要求后的最终关停状态

在本日志首次写完后，用户明确要求“关闭后台进程”。随后按以下顺序停止：

1. Docker GraspNet 推理容器及 `run_graspnet_d435i.sh`；
2. MoveIt、RViz、ros2_control及其launch进程组；
3. D435i相机、桥接、handeye TF及其launch进程组；
4. Piper驱动launch；父进程退出后残留的单一驱动PID使用SIGTERM关闭。

最终复查结果：

```text
项目相关后台进程：0
运行中Docker容器：0
ROS节点：0
grasp_executor：0
/grasp_execute服务：无
```

`can0`仍保持 `UP / ERROR-ACTIVE / 1Mbps`。CAN网络接口不是后台进程，本次没有主动
将其置为down。下一次连接机械臂前仍须重新检查接口状态与错误计数。

---

## 15. 下一次对话建议执行顺序

### P0：只读复核

1. 阅读本文和 `real-grasp-new-tcp-20260821.md`。
2. 重新检查进程、ROS graph、CAN、arm_status、真实关节反馈。
3. 确认 `/grasp_execute` 仍不存在；在修复完成前不要启动 `scan_only=false`。

### P1：修复笛卡尔接口

修改 `moveit_adapter.compute_cartesian_path()`：

```text
resp.trajectory → resp.solution
显式检查 resp.error_code
检查 fraction
失败时明确记录是否回退OMPL
```

增加纯单元测试/接口结构测试，再做plan-only，不先执行真机。

### P2：修复夹爪稳定反馈和最终裁决

1. 新反馈不等于稳定反馈。
2. 增加连续样本稳定窗口、总超时和闭合趋势检查。
3. 初判抓住后必须在安全保持/抬升后再次确认。
4. 只有第二次稳定确认通过才允许写 `success=true`。
5. 测试必须复现本轮序列 `69.7→24.0→4.3mm`，结果应为抓空，不得success。

### P3：新70mm门控scan-only验证

启动执行器时保持：

```text
scan_only:=true
```

用冻结或新鲜CSV验证：

- 日志打印 `gripper_max_opening: 70.0 mm`。
- 超宽候选被跳过但不会中止整帧遍历。
- rank16同类57.388mm候选仍可通过。
- 机械臂不动、夹爪不动。

### P4：候选评分策略评审

明确是否继续“score升序 + approach_down最大”，或改成：

```text
硬门：宽度、自洽、规划、approach_down
软排序：GraspNet score、approach_down、规划余量/路径质量
```

在没有用户裁决前，不要擅自把升序改成降序，也不要把低分rank16称为高置信候选。

### P5：再次真机前的强制条件

必须全部满足：

1. 笛卡尔响应字段修复并plan-only验证。
2. 夹爪稳定反馈测试覆盖本轮假阳性时序。
3. 70mm门控scan-only通过。
4. 保存grasp终点真实 `/joint_states_feedback` 并准备真实FK。
5. 用户明确授权新的一次真机执行。
6. 只执行一次、低速、现场观察、禁止自动重试。

---

## 16. 安全禁区与术语约定

- “轨迹SUCCEEDED”只表示控制链完成，不表示抓取成功。
- “应用success=true”在本轮已被证实不可信，直到稳定反馈逻辑修复。
- `actual_width=24mm`是瞬态，不是最终夹持宽度。
- 真实空夹全闭基线是4.3mm，理论零位与机械实际零位存在残余开度。
- `70mm`是当前小夹爪的最大指尖总开口，不是每指70mm；每指URDF行程35mm。
- `GripperCtrl`位置字段虽名为angle，实际是0.001mm行程单位。
- 新矩阵解决轴语义，不自动解决手眼绝对误差、MoveIt终点误差、候选抖动和碰撞场景。
- 任何只规划任务必须确认 `plan_only=true`、`execute_called=false`、
  `gripper_commanded=false`。
- 在抓取执行器停止状态下保持页面/感知运行是安全的；启动executor前必须确认
  `scan_only`参数。

---

## 17. 最终结论

本次对话完成了三个重大闭环：

1. 通过两轮严格plan-only A/B证明并切换了正确的新TCP朝向矩阵，使工具前进轴与
   GraspNet approach对齐。
2. 将Docker页面改为只显示机械臂实际选中的唯一最佳候选，并解决同一selected仅
   timestamp变化时页面丢失的问题。
3. 完成一次真实运动，诚实识别出“运动成功但抓空”的假阳性；随后依据Piper SDK
   明确70mm小夹爪上限并在候选/执行两层加入fail-closed门控。

系统当前最重要的下一步不是再次抓取，而是先修复：

```text
GetCartesianPath resp.solution字段
夹爪稳定反馈窗口与保持后二次判定
```

在这两项完成前，保持抓取服务停止，不自动重试真机。

---

## 18. 续会记录：Git 备份、回退、回退版真机抓取与最终关停（19:03–19:11）

> **状态覆盖声明（必须先读）**
>
> 本节及后续章节记录同一天稍后的续会操作，代表会话结束时的**最终实际状态**。
> 第 0–17 节描述的大规模抓取可靠性修改已经被完整保存到 Git 备份提交，但当前
> 工作分支已经回退到大规模修改之前的行为。因此，第 4、5、9、11、14、15、17 节中
> 关于“当前代码”“当前运行进程”和“下一步必须先修”的表述只能作为历史背景，不能
> 直接当作当前工作树事实。恢复大改版本前必须先阅读第 19 节的 Git 恢复说明。

### 18.1 续会用户指令顺序

续会阶段用户请求依次为：

1. 关闭宽度门控。
2. 回退先前修改，恢复到大规模修改之前的代码。
3. 明确要求“先使用 Git 备份然后回退”。
4. 在回退版代码上执行一次真机抓取。
5. 关闭全部后台进程。
6. 将整段对话与操作结果补充到日志。

本节记录上述指令的实际执行结果，不对失败的真机抓取做自动重试。

---

## 19. 根仓库 Git 初始化、备份与回退锚点

### 19.1 初始化原因

续会开始前，项目根目录没有 `.git`，只有以下嵌套仓库：

```text
graspnet-baseline/graspnetAPI_local/.git
src/piper_ros/.git
src/realsense-ros/.git
```

为了满足“先备份再回退”，在项目根目录初始化了新的 Git 仓库。新增根
`.gitignore`，排除了构建产物、运行日志、共享相机帧、Python缓存和临时文件：

```text
build/
install/
log/
shared_camera_data/
GraspNet logs/output
Python caches
editor/temp files
```

根仓库使用本地提交身份：

```text
user.name  = Codex Backup
user.email = codex-backup@local
```

### 19.2 大规模修改版本备份

回退前先把当时完整工作树提交为：

```text
commit: e797bfe
message: backup: state before rollback of grasp reliability changes
branch: master
tag: backup-before-large-change-rollback-20260821
```

这个提交是第 0–17 节所述大规模修改的可恢复锚点。即使后续回退，也没有丢失该版本。

### 19.3 回退分支与提交

创建并切换到：

```text
branch: rollback/pre-large-change
```

回退完成后提交为：

```text
commit: e6140b0
message: revert: restore pre-large-scale grasp behavior
tag: rollback-pre-large-change-20260821
```

会话结束时 Git 拓扑：

```text
e6140b0  (HEAD -> rollback/pre-large-change,
          tag: rollback-pre-large-change-20260821)
  |
e797bfe  (master,
          tag: backup-before-large-change-rollback-20260821)
```

恢复大改备份版本可使用：

```bash
git switch master
```

只读检查备份标签可使用：

```bash
git switch --detach backup-before-large-change-rollback-20260821
```

切换版本后必须重新执行 `colcon build` 并重启相机桥接、Docker推理和执行器；仅切换
Git分支不会替换已经载入内存的Python/ROS进程。

### 19.4 保留的嵌套仓库状态

根仓库最终仍显示：

```text
 m src/piper_ros
```

这是 `src/piper_ros` 嵌套仓库原有的脏状态，不是本轮回退产生的。本轮没有清理、重置
或覆盖该嵌套仓库。后续开发者不得用 `git reset --hard` 处理它，除非先单独审计内部
修改并得到用户明确授权。

---

## 20. 实际回退范围与当前代码语义

### 20.1 使用回退前副本恢复的模块

从 `build/real_piper_perception/build/lib/real_piper_perception/` 找到大改前安装/构建副本，
恢复以下源码：

```text
grasp_candidates.py
grasp_executor.py
grasp_precheck.py
sdk_safe_execute.py
```

恢复过程通过 Git blob/index 操作完成，使回退前提交仍可完整恢复，而不是覆盖唯一副本。

### 20.2 手工回退的大规模修改

以下大改行为已从当前分支撤销：

- 推理侧 K+D `cv2.undistortPoints` 点云反投影。
- RGB-D严格时间戳配对、最大时间差门控和 `frame_manifest.json` 新协议。
- 支撑平面RANSAC、最大连通分量和目标时序跟踪。
- 候选时序跟踪、确定性点云采样和29列CSV协议。
- Top-K“最高得分且可规划”早停候选策略。
- 新的稳定帧、候选稳定性、时间一致性参数。
- 新增宽度候选门控及其launch透传。
- 新的笛卡尔撤退、执行序列和更严格MoveIt容差修改。
- 新增时间戳桥接测试、时序列测试、局部轴测试及新执行序列测试。
- 新增文档 `docs/2026-08-21-grasp-success-improvement-implementation.md`。

当前推理重新使用：

```text
K-only create_point_cloud_from_depth_image
ROI workspace全部点作为target
np.random.choice随机采样
24列 perception_grasps.csv
```

当前桥接重新使用“最近color + 最近depth”的旧行为；`meta.mat`只写：

```text
intrinsic_matrix
factor_depth
```

### 20.3 明确保留的大改前既有修复

回退目标是“大规模修改之前”，不是回到项目最早版本。因此以下大改前已经存在的修复
被保留：

- `moveit_adapter.py` 的 ROS2 Humble `resp.solution`、error code与空响应兼容修复。
- 大改前已有的夹爪连续稳定反馈判定与抓住后保持复核。
- 标定文件内参K支持。
- 真机 hand-eye 文件当前值，不恢复旧外参。
- 预检前/收尾回zero的历史行为。
- SDK夹爪70mm执行层物理上限防御；即使上层宽度候选门被关闭，执行层仍会在任何
  机械臂运动前拒绝超出硬件行程的最终候选。

`docs/PROJECT_ARCHITECTURE_AND_DEVELOPER_GUIDE.md` 被保留，但顶部增加回退警告；其中
描述新功能的章节是历史设计资料，不代表当前分支已经启用。

### 20.4 回退后的候选行为

当前分支的 `rank_candidates` 仍按 **score升序**排列。执行器不是“最高分且可规划
立即执行”，而是：

1. 遍历单帧最多150个升序候选；
2. self-consistency通过；
3. approach/grasp预检通过；
4. 收集全部通过项；
5. 选择“开口朝下分量”最大的候选。

因此，规划耗时仍会随候选总数增长，且最终候选可能是低分或中低分候选。这一行为在
第22节真机结果中再次得到验证。

---

## 21. 回退后的构建、测试与运行协议验证

### 21.1 静态检查和构建

执行并通过：

```text
/usr/bin/python3 -m py_compile（推理、ROS Python、launch文件）
colcon build --packages-select real_piper_perception --symlink-install
Summary: 1 package finished
```

### 21.2 测试结果

全量相关测试结果：

```text
105 passed, 2 failed
```

两个失败均为回退前已有问题：

1. `test_sdk_safe_execute.py::test_box_clearance`：旧实现返回 `-0.0`，而测试要求严格
   小于0。
2. `test_tf_tree_viewer.py::test_real_handeye_yaml_parses_to_camera_edge`：测试硬编码旧
   hand-eye平移 `[0.4438,-0.0972,0.7446]`，当前标定文件是
   `[0.4501,0.0139,0.5724]`。

没有发现由本轮回退新增的测试回归。

### 21.3 重启运行环境加载回退代码

代码回退和构建后，旧的相机/Docker进程仍载入大改版本，因此按顺序停止并重新启动：

1. Docker GUI推理；
2. D435i相机流水线；
3. 回退版D435i流水线；
4. 回退版Docker GUI推理。

运行验证：

```text
meta.mat keys = [factor_depth, intrinsic_matrix]
perception_targets.csv = 5列
perception_grasps.csv = 24列
Docker CUDA模型加载成功（epoch 18）
GUI display thread启动
首帧有效点约290k，ROI workspace约76k
碰撞过滤后约70–82个候选/帧
稳定推理约0.08–0.09s，含渲染总循环约0.8s
```

共享目录中旧 `frame_manifest.json` 仍存在，但其mtime停留在旧进程停止前；回退版桥接
没有继续写它。后续诊断不得仅凭文件存在判断新协议仍启用，必须检查mtime和代码版本。

---

## 22. 回退版单次真机抓取（19:09:30–19:10:31）

### 22.1 执行前条件

用户明确要求执行真机抓取。执行前只读确认：

```text
arm_controller: active
gripper_controller: active
joint_state_broadcaster: active
Piper ctrl_mode=1
arm_status=0
err_code=0
六轴communication_status=false
机械臂关节反馈接近zero
相机、桥接、Docker推理持续运行
grasp_executor尚未运行
```

实际启动参数：

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=false \
  velocity_scale:=0.05 \
  enable_self_consistency:=true \
  graspnet_6d_max_candidates:=150 \
  graspnet_6d_analytic_only_precheck:=false
```

然后只调用一次：

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

没有自动重试，也没有同时运行任何直连CAN测试脚本。

### 22.2 预检耗时与候选选择

触发后执行器写入 `pause_inference`，冻结当前CSV，并确认机械臂已经接近zero，无需额外
回零运动。冻结帧包含：

```text
候选总数：87
预检全过：25
最终 selected_index：63（0-based）
selected score：约0.2087
GraspNet预测宽度：0.061564479m（约61.6mm）
宽度+5mm margin：0.066564479m（约66.6mm）
候选预检日志开口朝下分量：约0.63
```

预检从约19:09:30运行到19:10:09，耗时约40秒。耗时长的直接原因是回退版按score
升序遍历本帧全部87个候选，并对失败候选反复做MoveIt兜底规划，而不是在最高分可规划
候选处早停。

本次最终候选所需66.6mm开口没有超过70mm硬件极限，因此执行层物理防御允许继续。

### 22.3 真机执行与夹爪标定

夹爪每次执行前重新做全开/全闭标定：

```text
full_open = 0.0697m（69.7mm）
full_closed = 0.0043m（4.3mm）
```

随后以5%速度执行 approach 和 grasp，并发送完全闭合命令。执行结束前FK诊断：

```text
link6前向（FK） = [+0.39,+0.60,-0.70]
夹爪开口方向（FK/URDF） = [-0.68,+0.70,+0.22]
FK link7位置 = [0.5396,-0.0341,0.0364]m
```

### 22.4 最终结果：抓空

夹爪闭合后的稳定反馈：

```text
actual_width = 0.0043m
full_closed  = 0.0043m
```

二者完全一致，说明夹爪闭合到空夹基线，没有物体阻挡。本次业务结果：

```json
{
  "timestamp": "2026-08-21T19:10:19",
  "success": false,
  "status": "grasp_empty",
  "stage": "",
  "actual_width": 0.0043,
  "cmd_width": 0.066564479,
  "candidate_index": 63
}
```

结果文件：`shared_camera_data/grasp_result.json`（共享目录被Git忽略，是运行产物，不在
提交中）。

这一次没有出现第8节所述24mm瞬态假阳性：保留的稳定窗口逻辑正确等待到4.3mm稳定
闭合，并给出 `grasp_empty`。

### 22.5 收尾状态

抓取流程自动回到zero，MoveIt报告回零成功，删除 `pause_inference` 并恢复推理。复查：

```text
arm_status=0
err_code=0
六轴communication_status=false
机械臂反馈接近zero（最大偏差约0.0105rad）
gripper=0.0043m
pause_inference不存在
```

随后主动停止 `scan_only=false` 的抓取执行器，避免服务或键盘误触发。没有进行第二次
抓取。

### 22.6 本次抓空能证明与不能证明的内容

可以确认：

- Piper/MoveIt控制链、预检、approach、grasp、回zero完整运行。
- 稳定夹爪反馈判定没有把闭合瞬态误报为成功。
- 最终选择候选在执行层宽度范围内。
- 机械臂和驱动收尾无错误码。

不能确认：

- 候选中心是否精确覆盖物体真实可夹持区域。
- 当前hand-eye、深度反投影、TCP偏移的组合误差是否低于抓取容差。
- 无深度OctoMap时规划路径是否适应任意桌面和障碍物场景。
- score升序+朝下优选是否比最高分可规划候选具有更高成功率。

从本次结果看，候选可规划只说明机械臂能到达该位姿，不等于该位姿能抓住物体。

---

## 23. 最终后台关停状态（19:11）

用户要求关闭后台进程后，先精确检查进程组，再按以下顺序发送SIGINT：

1. Docker GraspNet推理进程组；
2. D435i相机、桥接和hand-eye TF流水线；
3. MoveIt、ros2_control和控制器进程组；
4. Piper驱动（在停止序列末尾已自行退出，无需重复发送信号）。

最终只读核对：

```text
inference_from_camera.py：0
Docker运行容器：0
realsense2_camera_node：0
d435i_graspnet_bridge：0
handeye_tf_publisher：0
move_group：0
piper_single_ctrl：0
grasp_executor：0
相关ros2 launch父进程：0
```

`can0`最终仍为：

```text
UP / LOWER_UP
```

CAN网络接口不是后台进程，本轮没有将其置为down。当前没有任何项目进程向机械臂发送
控制命令。

---

## 24. 会话结束时的最终交接结论

### 24.1 当前唯一应视为有效的状态

```text
当前分支：rollback/pre-large-change
当前提交：e6140b0
当前标签：rollback-pre-large-change-20260821
大改备份：e797bfe / backup-before-large-change-rollback-20260821
根工作树额外状态：仅 src/piper_ros 嵌套仓库原有 dirty 状态
项目后台进程：全部关闭
Docker容器：全部关闭
can0：保持UP，但无人控制
最近一次真机结果：grasp_empty（真实抓空）
最近一次候选：index 63，score约0.2087，宽度约61.6mm
机械臂收尾：近zero，arm_status=0，err_code=0
```

### 24.2 后续开发者启动前必须做的检查

1. 先运行 `git branch --show-current` 和 `git log --oneline --decorate -3`，确认究竟使用
   回退分支还是大改备份分支。
2. 审计 `src/piper_ros` 嵌套仓库脏状态，不要擅自清理。
3. 重新构建当前分支，不能假设 `install/` 与源码分支一致。
4. 重新检查 `can0`、控制器、`arm_status`、`err_code`和真实关节反馈。
5. 相机启动后检查 `meta.mat`字段和CSV列数，确认运行协议与源码版本一致。
6. 真机前先以 `scan_only:=true` 验证当前场景候选；不得自动循环触发。
7. 如果继续使用回退版，必须接受其score升序、全量规划和约40秒预检耗时；若要改变
   候选策略，应在新分支中实现并用离线/plan-only A/B验证，不要直接覆盖回退锚点。

### 24.3 关于本日志前后章节的使用方式

- 第0–17节：大规模修改版本的历史、实验与问题分析，对应Git备份 `e797bfe`。
- 第18–24节：备份后的回退操作、回退版真机结果与最终关停状态，对应当前分支
  `e6140b0` 加本日志追加内容。
- 两部分都应保留：前半部分解释为何曾做大改，后半部分解释为何又回退以及当前实际
  运行的是哪套代码。

下一次会话不得把“备份提交中的已实现功能”和“回退分支中的当前功能”混为一谈。

---

## 25. 本次对话最终权威状态（20:09–20:21）

> 本节是本次用户对话结束时的最新交接记录，覆盖并更新前文可能存在的旧运行状态。

### 25.1 本次对话实际完成的工作

用户先要求阅读项目、激活机械臂、打开RViz、启动Docker GUI推理并执行真机抓取；随后
要求继续、删除宽度门限、规划成功后直接抓取，并连续要求重新推理和抓取三次。每次都
重新启动一次 `grasp_executor`、重新触发一次 `/grasp_execute`，没有自动重试。

删除宽度门限的最终源码变化：

- `grasp_execute.py` 不再用 `fits_small_gripper()` 拒绝候选宽度。
- 仍保留70mm全开标定命令 `GripperCtrl(70000, ...)`。
- `test_gripper_limits.py` 改为验证超宽候选不会在 `gripper_width` 阶段被拒绝。
- 4项相关测试通过，Python编译检查通过，`./build_ws.sh`成功（3 packages finished）。

### 25.2 三轮最新真机结果

| 轮次 | 候选总数 | 预检通过 | selected | 指令宽度 | 实际宽度 | 结果 |
|---|---:|---:|---:|---:|---:|---|
| 第1轮（删除门限后） | 94 | 30 | 39 | 81.2mm | 27.3mm | success |
| 第2轮 | 33 | 19 | 17 | 89.5mm | 36.5mm | success |
| 第3轮 | 27 | 9 | 1 | 81.9mm | 32.7mm | success |

三轮均完成抓取和保持判定，随后机械臂回zero成功。最后一轮服务返回 `success=True`，
结果文件最终内容为：

```json
{
  "timestamp": "2026-08-21T20:21:12",
  "success": true,
  "status": "success",
  "stage": "",
  "actual_width": 0.0327,
  "cmd_width": 0.081872543,
  "candidate_index": 1,
  "message": "candidate [1] target: pick status=success stage='' actual_width=0.0327 cmd_width=0.0819"
}
```

最后一轮回zero后的真实夹爪反馈约32.4mm，仍保持夹持开度；没有发送释放命令。

### 25.3 最终关机状态

用户随后要求关闭后台进程。已优雅停止：

- Piper机械臂驱动及启动父进程
- MoveIt、ros2_control、控制器和RViz
- D435i相机、推理桥接和手眼TF
- 单次抓取执行器
- Docker容器 `exciting_goldberg`
- ROS 2 CLI daemon

核验结果：无项目相关实际进程、无运行Docker容器、无实际ROS项目节点。`can0`接口仍
保留UP状态，但没有驱动进程占用，也没有后台程序继续向机械臂发送控制命令。

### 25.4 最新产物与下一次恢复入口

- 运行结果：[shared_camera_data/grasp_result.json](../shared_camera_data/grasp_result.json)
- 会话日志：[docs/2026-08-21-session-summary.md](2026-08-21-session-summary.md)
- 当前运行环境：全部关闭
- 下一次如需抓取，必须重新启动Piper、MoveIt/RViz、D435i桥接和Docker推理，再单次触发
  `grasp_execute`；不要假设当前节点仍在运行。
