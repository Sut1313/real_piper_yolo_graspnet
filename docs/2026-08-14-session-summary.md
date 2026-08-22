# 2026-08-14 真机抓取联调会话完整记录

> **本文档是下次会话的完整恢复手册**：环境状态、分钟级时间线、全部修改（含目的）、
> 实验数据、代码修改明细、日志索引、待办。概要见 §1，时间线见 §2，
> 修改明细见 §3，抓取全记录见 §4，实验数据见 §5，衔接要点见 §9。

---

## 0. 会话结束时环境状态（2026-08-14 18:55）

| 项 | 状态 |
|----|------|
| 后台进程 | **全部关闭**（桥接/推理/驱动/MoveIt/executor/容器全停，无残留） |
| 机械臂 | **零位**（18:49 本轮收尾回 zero 成功，FK link6=(56.5,-0.1,214.3)mm），上电中 |
| 驱动 | 已崩（rclpy 双 shutdown bug，今日第 3 次触发），残留已清 |
| can0 | UP（ERROR-ACTIVE 正常） |
| 相机 | D435i USB3（speed=5000） |
| 代码 | 今日 7 处修改全部在源码（egg-link 生效） |
| 测试 | 48 passed + 1 failed（sdk_safe_execute 既有问题，见 §3.8） |
| 记忆 | 待沉淀（本次会话结论见 §9） |

**本次会话主线**：20 次真机抓取触发全部未命中 → 系统排查：执行-感知自洽性验证（<1cm）→
根因锁定为"感知对物体绝对位置的估计偏差"。过程中按用户裁决完成 7 项配置/代码修改：
升序排序、150 候选截断、门控可关、zero 起点/终点、pre_pose/抬升关闭、handeye TF 桥接模块。

---

## 1. 一页摘要

- **抓取结果**：20 次触发 = 12 次完整执行（**全部 grasp_empty**）+ 2 次 pre_pose 失败（已修复）+ 6 次 no_candidate。
- **核心结论**：9 个有效样本的"感知报点 vs FK link7 落点"对比显示**执行-感知完全自洽**（水平均值 6mm、z 差 -20mm=设计值 grasp_z_offset）——机械臂精确到达感知报的位置，**物体不在感知报的位置**。抓空根因 = 感知对物体绝对位置估计偏差（相机深度/内参/分割或手眼外参），非执行链、非求解器、非排序/门控/起点。
- **关键实验**：ready/zero 起点受控 A/B（2 帧 × 2 起点，逐候选 0 不一致）→ 起点无关；门控关闭后可用候选 5-10 倍增长（1-3 → 8-14/轮）→ 门控在真机上代价大于收益。
- **新增模块**：handeye_tf_publisher（读标定文件 → 发布 base_link←camera_link 静态 TF），MoveIt "unconnected trees" 警告消失。
- **新发现**：预检"开口朝下"度量与 FK 实测方向出现**符号级背离**（预检 -0.69 vs FK +0.97），疑似 graspnet_to_tcp_rotation 变换链口径问题（待办 #2）。

---

## 2. 完整时间线（2026-08-14）

### 2.1 会话前状态
- 系统 08-10 后停机 4 天。**camera_data/handeye_eye_to_hand.yaml 于今日 16:34 被重写**——
  新标定 t=(0.444, -0.097, 0.745) vs 08-08 旧标定 t=(0.467, -0.065, 0.663)，互差 ~9cm（z 差 8.2cm）。
  无伴随会话文档，推测为针对抓空的新一轮标定尝试（未见真机验证记录）。

### 2.2 首次抓取测试（16:39-16:50）

| 时刻 | 事件 |
|------|------|
| 16:39 | 启动桥接 + GUI 推理（MIN_DEPTH 0.20/MAX 1.20） |
| 16:41 | can0 激活、驱动（Enable True）、MoveIt、executor（scan_only:=false） |
| 16:42 | **尝试 1**：候选[47] grasp_empty（预检朝下 -0.07，FK 朝下 0.28） |
| 16:43 | **尝试 2**：候选[32] **pre_pose 失败**（规划 Invalid goal state ×2） |
| 16:45 | 驱动崩溃（rclpy 双 shutdown，当日第 1 次）→ 重启 |
| 16:47 | **尝试 3**：no_candidate（0/19 approach_pass） |
| 16:49 | **尝试 4**：候选[41]（朝下 0.64，方向最佳）**pre_pose 失败** |

### 2.3 修复 pre_pose（16:57-16:58）

- 16:57 用户裁决：**关闭 lift 预位动作（pre_pose 段）**——lift 位姿（grasp+10cm 同姿态）系统性不可达。
- 根因：`grasp_execute.py:523` pre_pose 目标 = `stage_poses['lift']`（08-10 关闭 lift 预检后成为盲区，
  执行第一步才暴露不可达；OMPL "Unable to sample any valid states for goal tree"）。
- 16:58 重启 executor → **尝试 5**：候选[37]（朝下 0.59）grasp_empty。

### 2.4 关闭抓后抬升（17:00-17:02）

- 17:00 **尝试 6**：候选[31] grasp 判定空后，**抓后抬升（stage='lift'）失败**——同 pre_pose 机制
  （grasp+10cm 同姿态不可达，笛卡尔+OMPL 回退双失败）。
- 17:02 用户裁决：**抬升动作暂时关闭（预检+执行都不再含 lift）**。verdict 后直接返回，
  收尾由 executor joint-space 回 ready（实测安全）。
- 17:03 **尝试 7**：候选[1]（朝下 0.61）grasp_empty——lift 全关后执行序列首次完整走通。

### 2.5 no_candidate 期（17:03-17:06）

- **尝试 8/9/10**（17:03/17:05/17:06）：连续 3 轮 no_candidate（approach_pass=0）。
  诊断：候选位置分布正常（dist_xy 0.44-0.52 环带内），是帧姿态族整体不可达（帧内容驱动，
  与起点无关——后续 A/B 实验证实）。

### 2.6 Docker 推理崩溃与恢复（17:05-17:07）

- 17:05 Docker 推理容器退出：日志尾部 **X Error: BadWindow**——matplotlib 窗口被外部销毁，
  GUI 线程崩溃拖垮进程（GUI 模式窗口一关推理即死的特性）。
- 17:05:40 重启 GUI 推理（新容器）。

### 2.7 抓取与 scan_only 混合期（17:08-17:35）

- 17:08 **尝试 11**：候选[34]（朝下 0.36）grasp_empty。FK 朝下=0.77，link7 z=40.6mm。
- 17:11-17:14 **ready/zero 起点 A/B 实验**（详见 §5.1）→ 起点无关。
- 17:31/17:32 **尝试 12/13**：no_candidate ×2。
- 17:34 **尝试 14**：候选[32]（窄开口 5.2cm）grasp_empty。

### 2.8 scan_only 候选普查（17:40-17:47，50 截断）

- 17:40 桥接+headless 推理+MoveIt+scan_only executor。
- **14 轮 scan_only**（§5.2）：1342 候选 → 35 可用（2.6%）。通过者全为低分（≤0.19）、
  位置/方向随帧漂移。拒绝漏斗：排名截断（>50 不被查）→ self-consistency 门控 57% →
  approach 预检 38% → grasp 0.1%。

### 2.9 配置修改轮（17:41-18:50）

| 时刻 | 修改 | 用户裁决 | 验证 |
|------|------|---------|------|
| 17:41 | 切 GUI 推理 | — | 窗口正常 |
| 18:05 | **max_candidates 50→150** | 扩大被检集 | 3 轮 scan：可用 3-8/轮（原 0-5）；候选 [67]/[121] 等 >50 编号解锁 |
| 18:11 | 真机执行 150 配置 | — | 尝试 15/16/17 全空（[35]/[121]/[122]，FK 朝下 0.93/0.97/0.84） |
| 18:25 | **排序改 score 升序**（原降序+8桶轮转） | 低分可达候选优先 | 2 轮 scan：低分候选先被检（编号 7/55）；可用数无显著变化（排序不改可达性） |
| 18:29 | **enable_self_consistency 参数**（现关闭） | 门控实验 | 3 轮 scan：**可用 8-14/轮（5-10 倍增长）**，approach 通过 11-16/轮 |
| 18:33 | 真机执行（门控关） | — | 尝试 18/19 全空（[13]/[40]，score 0.041/0.355） |
| 18:44 | **起点/终点 zero**（原 ready） | 从 zero 规划并回 zero | 尝试 20：全流程日志验证（回 zero ×2 成功），仍空 |

### 2.10 对准分析（18:40-18:50）

- 9 个有效样本感知 vs FK 对比（§5.4）：执行-感知自洽 <1cm → 根因锁定感知绝对位置估计。
- 18:50 驱动第 3 次崩溃（rclpy 双 shutdown）→ 清理。
- 18:55 用户要求关闭后台 → 全停。编写本文档。

---

## 3. 代码修改明细（含目的）

| 文件 | 修改 | 目的 |
|------|------|------|
| `real_piper_perception/grasp_execute.py` | ① pre_pose 段关闭（16:57）：跳过 `_move_pose(stage_poses['lift'])`，保留 max_stage 截断语义 ② 抓后抬升关闭（17:02）：verdict 后直接返回，executor finally 收尾回 zero ③ docstring 同步 | lift 位姿（grasp+10cm 同姿态）系统性不可达，执行中段失败 4 次触发 |
| `real_piper_perception/handeye_tf_publisher.py`（新增） | 读标定文件 → 校验（det≈1、冗余 T 一致性）→ 发布 base_link←camera_link 静态 TF；帧名文件键优先、参数回退；fail-loud | 连通相机 TF 树与机器人 TF 树（此前 move_group 每轮报 unconnected trees） |
| `launch/d435i_graspnet_pipeline.launch.py` | 集成 handeye_tf_publisher + 3 launch 参数（handeye_yaml/parent_frame/child_frame） | TF 桥接随感知栈启动 |
| `setup.py` | 注册 handeye_tf_publisher entry point | 可执行文件 |
| `test/test_handeye_tf_publisher.py`（新增） | 7 个单测：解析/帧名覆盖/缺文件/缺字段/det 错/冗余不一致/四元数往返 | 覆盖 fail-loud 各分支 |
| `real_piper_perception/grasp_candidates.py` | rank_candidates 改 **score 升序**（删除 _bucket_id 与 8 桶轮转） | 低分（可达概率高）候选优先进入被检序列 |
| `real_piper_perception/grasp_executor.py` | ① `graspnet_6d_max_candidates` 50→150 ② 新增 `enable_self_consistency` 参数（默认 true，可关） ③ `READY_JOINTS`→`ZERO_JOINTS`（全 0），预检前回 zero + finally 收尾回 zero，日志文案同步 | ① 缓解"高分不可达+截断挡低分可达"耦合 ② 门控实验开关 ③ 起点/终点 zero（用户裁决） |
| `launch/grasp_execute.launch.py` | 同步 3 处默认值（150 / enable_self_consistency=true / 声明 LaunchConfiguration） | 与节点默认一致 |
| `test/test_grasp_candidates.py` | rank 测试改断言升序（原桶轮转断言删除） | 同步新排序语义 |

## 3.8 测试状态

- **48 passed + 1 failed**。唯一失败 `test_sdk_safe_execute.py::test_box_clearance`：
  `box_clearance([0,0,0.06])` 返回 -0.0，断言 `-0.0 < 0` 恒假——**08-08 既有问题
  （sdk_safe_execute.py 与测试文件均 08-08 未改动），与本日修改无关**。
- handeye_tf_publisher 实测：三步日志（读取→查参→发布）✓；tf2_echo 数据与文件逐元素一致
  （最大误差 3.3e-16）✓；真实候选点双路径变换一致（2.2e-16）✓；**MoveIt unconnected trees 计数=0**（此前常态）✓。

---

## 4. 真机抓取全记录（20 次触发，全部未命中）

| # | 时间 | 配置阶段 | 候选 | score | 预检朝下 | FK朝下 | FK link7 (mm) | 结果 |
|---|------|---------|------|-------|---------|--------|---------------|------|
| 1 | 16:42 | 原配置(ready/pre_pose开) | [47] | 0.466 | -0.07 | 0.28 | (498,-88,115) | 空 |
| 2 | 16:43 | 同左 | [32] | 0.575 | -0.13 | — | (303,2,120) | pre_pose失败 |
| 3 | 16:47 | 同左 | — | — | — | — | — | no_candidate(0/19) |
| 4 | 16:49 | 同左 | [41] | 0.399 | 0.64 | — | — | pre_pose失败 |
| 5 | 16:58 | +pre_pose关 | [37] | 0.347 | 0.59 | 0.44 | (437,-88,178) | 空 |
| 6 | 17:00 | 同左 | [31] | 0.481 | 0.59 | — | (427,-95,90) | 空+抬升失败 |
| 7 | 17:03 | +抬升关 | [1] | 0.385 | 0.61 | 0.64 | (444,-102,103) | 空 |
| 8 | 17:03 | 同左 | — | — | — | — | — | no_candidate(0/4) |
| 9 | 17:05 | 同左 | — | — | — | — | — | no_candidate(0/21) |
| 10 | 17:06 | 同左 | — | — | — | — | — | no_candidate(0/18) |
| 11 | 17:08 | 同左 | [34] | 0.295 | 0.36 | 0.77 | (488,-122,41) | 空 |
| 12 | 17:31 | 同左 | — | — | — | — | — | no_candidate(0/30) |
| 13 | 17:32 | 同左 | — | — | — | — | — | no_candidate(0/15) |
| 14 | 17:34 | 同左 | [32] | 0.362 | 0.50 | — | (440,-83,44) | 空(窄开口5.2cm) |
| 15 | 18:11 | +150 | [35] | 0.044 | 0.14 | 0.93 | (447,-217,76) | 空 |
| 16 | 18:15 | 同左 | [121] | 0.094 | -0.69 | 0.97 | (433,-170,91) | 空 |
| 17 | 18:18 | 同左 | [122] | 0.030 | -0.56 | 0.84 | (469,-71,121) | 空 |
| 18 | 18:33 | +门控关 | [13] | 0.041 | — | 0.86 | (416,-88,14) | 空 |
| 19 | 18:36 | 同左 | [40] | 0.355 | — | 0.34 | (478,-91,97) | 空 |
| 20 | 18:49 | +zero起终点 | [36] | — | — | 0.86 | (416,-88,14) | 空 |

**规律**：score 与结果无关（0.03~0.58 都空）；方向（朝下 0.14~0.97）与结果无关；
宽度（3.5~10cm）与结果无关；位置逐轮漂移（y -0.22~-0.07，z 0.014~0.121）。

---

## 5. 实验与数据分析

### 5.1 ready vs zero 起点 A/B（受控，只规划不执行）

- 协议：冻结 CSV + 同 executor 实例，唯一变量 = 臂物理姿态（ready=[0,1.49,0,0,-1.22,0] / zero=全0）。
- 结果：帧1（80 候选）A vs B 逐候选 **0 处不一致**；帧2（74 候选，顺序对调）**0 处不一致**。
  通过者身份完全相同（帧1=[9]，帧2=[25,39]）。
- **结论：起点姿态对预检结果无影响**（08-10 观察到的"ready 波动大"实为帧内容驱动）。
- 数据文件：`log/executor_ab_scan.log`。

### 5.2 候选可用性普查（50 截断，14 轮）

- 1342 候选 → 700 被检（前 50 × 14 轮）→ 402 self-consistency 拒（57%）→ 263 approach 失败
  （38%）→ 1 grasp 失败 → **35 可用（5%/轮）**。
- 可用候选 score ≤0.19；位置/方向随帧漂移；score 与方向族强耦合（开口朝上族高分、
  开口朝下族低分）。
- 记录文件：`log/usable_candidates_20260814.csv`（35 行，含 base 系坐标与选中标记）。

### 5.3 配置修改效果对比

| 配置 | 被检/轮 | 可用/轮 |
|------|---------|---------|
| 50 截断 | 50 | 0-5（均值 ~2.5） |
| **150 截断** | 150 | 3-8 |
| +门控关 | 150 | **8-14** |
| +升序 | 150 | 1-3（样本小，无显著提升） |

### 5.4 感知 vs FK 对准分析（根因定位，关键）

- 方法：9 个有效样本（抬升关闭后 FK 在抓取位测量）对比"感知报点（handeye 变换到 base）"vs "FK link7 落点"。
- **水平 Δx=-5.7mm（σ5.8）、Δy=-0.2mm（σ6.4）；z 恒 -20mm（= grasp_z_offset 设计值）**。
- 剔除 4 个无效样本：2 个抬升前测量（含 +10cm）、2 个 pre_pose 失败未到达抓取位。
- **结论：执行-感知完全自洽（<1cm）→ 执行链无误差 → 抓空根因 = 感知对物体绝对位置估计偏差**
  （相机深度/内参/分割，或手眼外参本身错——执行数据无法区分，需外部真值基准）。
- 手眼标定相关：新旧标定互差 ~9cm；两代标定下均 18+20 次全空 → 标定未被证实为唯一根因。

### 5.5 预检方向 vs FK 方向系统性背离（新发现）

- 预检"开口朝下"度量 vs FK 实测开口方向：attempt 15（0.14→0.93）、16（-0.69→0.97）、
  17（-0.56→0.84）——**符号级背离**（预检判开口朝上、FK 实测近垂直朝下）。
- 两个度量理论上应一致（预检 r[1,2] = TCP+Y 的 z 分量；FK 朝下 = -(URDF link6 -Y)_z）。
- 08-10 待办第 4 条记录过同类偏差（0.59→0.44），今日数据将其升级为"疑似变换链缺陷"。

---

## 6. 当前架构与配置（2026-08-14 终态）

### 6.1 真机抓取实际用到的层次

```
D435i → realsense2_camera_node → bridge → shared_camera_data/ → Docker 推理
→ perception_grasps.csv → grasp_executor：
   score 升序 → 前 150 → 门控(score 恒过、self-consistency 现关)
   → make_stage_poses(handeye 矩阵+轴映射) → precheck(approach+grasp, KDL+OMPL 0.5s)
   → 选择(开口朝下最大) → execute_pick(回zero→张开→标定→approach线性→grasp OMPL
   → 闭合→判定→回zero)
→ move_group(KDL/OMPL) → FakeSystem → /joint_states → driver → CAN → 臂
```

### 6.2 当前参数（本次会话裁决汇总）

| 参数 | 值 | 来源 |
|------|-----|------|
| 排序 | score 升序 | 今日 |
| graspnet_6d_max_candidates | 150 | 今日 |
| enable_self_consistency | false（实验关） | 今日 |
| grasp_score_min | 0.0（恒过） | 08-07 |
| pre_pose / 抓后抬升 | 关闭 | 今日 |
| 起点/终点 | zero | 今日 |
| planning_time_precheck | 0.5 | 08-10 |
| ssik / analytic 门控 | 关 | 08-10 |
| handeye | 今日 16:34 新标定 | 今日 |

### 6.3 TF 链

```
world → base_link → … → link6 → gripper_base → link7 → link8   （机器人树）
        ↑ handeye_tf_publisher 静态桥接
camera_link → camera_*_frame/optical_frame                       （相机树）
```
两树已连通（MoveIt unconnected trees=0）。executor 内部仍用矩阵（非 TF buffer）。

---

## 7. 已知问题与待办（下次会话优先级排序）

1. **感知绝对位置验证（主线，抓空根因）**：用已知尺寸/位置参照物放已知 base 位置，
   对比感知报点与真值 → 区分"手眼外参错误" vs "相机深度/内参/分割误差"。
   若偏差 ~9cm 且方向与新旧标定差一致 → 手眼问题；若随深度变化 → 相机侧。
2. **预检方向 vs FK 方向符号级背离**：核对 graspnet_to_tcp_rotation 变换链与 FK 度量口径
   （今日数据：-0.69 vs +0.97）。注意：这可能是理解"执行姿态"真实语义的关键。
3. **驱动 rclpy 双 shutdown bug（今日 3 次触发）**：`piper_ctrl_single_node.py` main()
   finally 加 `if rclpy.ok(): rclpy.shutdown()` 防护。修复成本一行，触发频繁，建议下次优先。
4. **当前配置组合的完整验证**：升序+150+门控关+zero 的组合仅在个别轮次测试过；
   可用候选池扩大（8-14/轮）后选择逻辑（开口朝下最大）的表现待更多样本。
5. **运维固化**：收尾按 PID 清子进程（TaskStop 不杀）；pkill 括号技巧；
   GUI 推理窗口关闭即崩溃（长任务用 headless + view_results.sh）。

---

## 8. 日志文件索引（log/）

| 文件 | 内容 |
|------|------|
| `executor_session_20260814_164107.log` | 尝试 1-2 |
| `executor_session_20260814_164712.log` | 尝试 3-11 |
| `executor_session_20260814_170240.log` | 尝试 12-14 |
| `executor_session_20260814_1810xx/1831xx.log` | 尝试 15-20（150/门控关/zero 配置） |
| `executor_scan_only_20260814_174101.log` | 50 截断 14 轮 scan_only（35 可用候选源） |
| `executor_scanonly_asc.log` | 升序 2 轮 |
| `executor_nogate.log` | 门控关 3 轮 |
| `executor_ab_scan.log` | ready/zero A/B 实验 |
| `usable_candidates_20260814.csv` | 51 个可用候选记录（含 base 坐标） |
| `driver_session_20260814_*.log` | 驱动各次启动（含 3 次崩溃日志） |
| `bridge/graspnet/moveit_session_*.log` | 各组件运行日志 |

---

## 9. 下次会话衔接要点

1. **重启栈命令**（按序）：`run_d435i_bridge.sh`（含 handeye TF 节点）→
   `run_graspnet_d435i.sh gui|headless` → 驱动 `ros2 launch piper start_single_piper.launch.py
   gripper_val_mutiple:=2 gripper_forward:=false` → MoveIt demo → executor
   `ros2 launch real_piper_perception grasp_execute.launch.py scan_only:=false`。
   当前代码默认已含今日全部裁决（150/升序/门控开[默认]/zero）——真机实验需显式
   `enable_self_consistency:=false` 才与今日实验配置一致。
2. **机械臂当前在零位**（无需先归零）；驱动需重启（已崩）。
3. **根因主攻方向**：感知绝对位置验证（待办 #1）——这是 20 连空的唯一剩余解释。
4. **口径提醒**：FK link7 ≠ TCP（link6）：link7 = link6 + 135.8mm×前向（URDF joint7
   xyz=(0,0,0.1358) rpy=X90°）；对准分析用"FK link7 vs 感知报点"且须在抬升关闭后的样本。
5. **记忆沉淀建议**：本次会话结论（起点无关、门控代价、对准自洽、变换链背离）应写入
   auto-memory，与 precheck-pure-kdl / sdk-control-forbidden 等并列。
