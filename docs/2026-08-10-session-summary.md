# 2026-08-10 真机联调会话完整记录（详细版）

> **本文档是下次会话的完整恢复手册**：环境状态、分钟级时间线、全部轮次数据、问题诊断过程、
> 代码修改明细、日志索引、待办。概要见 §1，时间线见 §2，数据见 §4-§6。

---

## 0. 会话结束时环境状态（2026-08-10 22:10）

| 项 | 状态 |
|----|------|
| 后台进程 | **全部运行中**：驱动(312348) / MoveIt move_group(312467) / 桥接 / Docker GUI 推理(naughty_elion) / executor(scan_only=false) |
| 机械臂 | **zero 位姿**（FK 56.8, 0.1, 214.3mm，21:53 MoveIt 归零），上电中 |
| can0 | UP（ERROR-ACTIVE 正常），适配器在 USB 地址 **[REDACTED_USB_ADDRESS]**（会话中从 [REDACTED_USB_ADDRESS] 重枚举，接口号 6→8） |
| 相机 | D435i USB3（speed=5000，序列号 [REDACTED_CAMERA_SERIAL]），桥接 + 推理出帧正常（CSV 22:10 新鲜） |
| 代码 | 全部修改在源码（egg-link 生效，重启节点即加载；moveit 配置包需 colcon build） |
| 测试 | 36 passed + 1 skipped |
| 记忆 | 3 条已沉淀：precheck-pure-kdl / sdk-control-forbidden / sdk-vs-driver-contention |
| 推理容器 | graspnet-baseline:20.04（headless 曾用，GUI 模式现用） |

**本次会话与 08-08 的衔接**：08-08 结束时 MoveIt 链判定为 FakeSystem 隐式链、SDK 直控路径已验证、Task 11 抓取 8 次全 grasp_empty、感知噪声实验已做（候选波动 8-14cm）。本次会话主线是把预检配置从"解析 IK + 1.0s + zero 起点 + 三段"演进为"纯 KDL + 0.5s + ready 起点 + 两段"。

---

## 1. 一页摘要

**主线**：预检配置四步演进（纯 KDL → 0.5s 预算 → ready 起点 → 关闭 lift 段），筛选耗时从 ~50-60s 降到 ~20s，真机抓取验证 18 次全 grasp_empty（抓空与求解器无关，根因在感知对准）。过程中解决 4 个环境/代码故障：驱动 rclpy 双 shutdown 崩溃（复现 2 次）、机械臂 CAN 失效（适配器重枚举 + 臂断电）、move_group SIGABRT（joint_limits 整数类型）、SDK 直控失效（driver 链路占线）。沉淀 3 条跨会话记忆。

**核心数据**：ssik 解析 IK 四轮 A/B 共 58 次求解 0 成功；KDL 是唯一有效求解器；纯 KDL 筛选 19.6~31s vs 解析+兜底 49.5~51.8s，通过率相同或更好。

---

## 2. 完整时间线（2026-08-10）

### 2.1 上午-傍晚（会话前）
- 会话开始于 19:19。系统 08-08 后停机两天，机械臂保持上电。

### 2.2 启动与首次执行（19:19-19:26）

| 时刻 | 事件 |
|------|------|
| 19:19 | 预检：D435i USB3（5000M，lsusb -t 确认）、DISPLAY=:1、graspnet-baseline:20.04 镜像存在 |
| 19:20 | 启动桥接（run_d435i_bridge.sh）→ 相机 USB 3.2 识别、RealSense Node Is Up、5Hz 出帧 |
| 19:21 | 启动 Docker GUI 推理（MIN_DEPTH=0.20 MAX_DEPTH=1.20）→ NVIDIA 容器、EGL/OpenGL 初始化 |
| 19:23 | can0 激活（can_activate.sh）→ 启动驱动 → **Enable status True**（第 2 次尝试） |
| 19:24 | 启动 MoveIt demo → "You can start planning now!"（camera TF 警告为常态） |
| 19:25 | 启动 executor（scan_only:=false）+ 布设 Monitor 实时耗时监控 |
| 19:25-19:26 | **Task 11 第 1 次执行**：candidate [29] grasp_empty，~68s |
| 19:26 | 首轮耗时结论：**候选筛选+预检 ~57s 占全流程 79%**，执行段仅 ~18s |

### 2.3 单帧模式改造（19:30-19:33）

| 时刻 | 事件 |
|------|------|
| 19:30 | 用户裁决：每次执行只取一帧，不再累加五帧 → 修改 grasp_executor.py `candidate_frames` 兜底默认 5→1 |
| 19:30-19:31 | scan_only 干跑验证：「候选帧 1/1：87 候选」，0 行"等待下一帧" |
| 19:31 | 重启 executor（scan_only:=false） |
| 19:33 | **Task 11 第 2 次执行**：candidate [42] grasp_empty，~79s（筛选 59s/134 候选） |

### 2.4 ready 位姿测试（19:40-19:55）

| 时刻 | 事件 |
|------|------|
| 19:40 | 用户要求执行 ready 位姿。检查 SRDF：ready 已存在（joint2=1.4919, joint5=-1.22） |
| 19:52 | 写 sdk_go_ready.py（SDK MoveJ）→ **FK 全程不动**（单发 JointCtrl 不执行） |
| 19:53 | 改流式循环发送 → **仍不动**。诊断：FakeSystem 200Hz 发布 /joint_states（全零），driver remap 当指令 → CAN 3kHz 占线，SDK 帧抢不过 |
| 19:55 | 用户裁决：不特别要求时**禁止 SDK 控制臂轨迹**（记忆沉淀 sdk-control-forbidden） |
| 19:56 | 写 moveit_go_ready.py（SRDF 动态解析 + MoveIt 链）→ **执行成功**（3.6s@0.1，FK 169.8,0.1,149.6 到位） |

### 2.5 驱动崩溃与归零（20:25-20:33）

| 时刻 | 事件 |
|------|------|
| 20:25 | **驱动崩溃**（rclpy 双 shutdown，08-08 同款 C1'）——Task 11 完成后 |
| 20:32 | 真机 FK 读数正常（56.1,0,213.3 = 零位）——臂在崩溃后仍正确回零 |
| 20:33 | 用户要求关闭后台进程 → 全停（含补停 docker 容器） |

### 2.6 恢复与故障排查（20:34-20:44）

| 时刻 | 事件 |
|------|------|
| 20:34 | 重启驱动 → **启动即崩**（Enable status 5 次 False → 超时 → 双 shutdown） |
| 20:34 | 重启 MoveIt demo → **move_group SIGABRT**（exit -6）——新故障 |
| 20:35-20:36 | SDK 探臂：EnablePiper 失败、FK 全零、电机无故障标志（默认值）→ 判断臂未响应 |
| 20:37 | 用户指示重新激活 can → can_activate.sh 检测到已激活跳过；强制 down/up 无效 |
| 20:39 | **发现 can0 接口号 6→8**（适配器重枚举，USB 地址 1-7→1-9）→ 重新激活成功 |
| 20:39 | SDK EnablePiper 成功、6 电机 enable=True、FK 恢复（53.3,0.4,189.8，断电后位置微偏） |
| 20:40 | 驱动重启成功（Enable True）；**发现后台命令 cwd 残留坑**（cd 到 src/piper_ros 后相对路径失败）→ 改用绝对路径 |
| 20:42 | MoveIt 重启仍 SIGABRT → 定位：`joint_limits.yaml` 整数类型（`max_velocity: 5`）→ MoveIt 要求 double |
| 20:43 | 修复 joint_limits.yaml（sed 加 .0）+ rebuild → move_group 正常 |
| 20:44 | 移臂到 ready（MoveIt 链） |

### 2.7 A/B 测试（20:45-20:55）

| 时刻 | 事件 |
|------|------|
| 20:45 | 冻结 CSV（189 候选，log/frozen_ab_test.csv） |
| 20:45 | 测试 A（纯 KDL，**误配置**：门控默认 true 未关）→ **0.0s 完成，33 行 Skipping**——门控短路，作废 |
| 20:46 | 测试 B（解析+兜底，同样误配置）→ 5/4/3，30 行 Skipping——ssik 失败后 KDL 没跑，作废 |
| 20:47 | **测试 A2（纯 KDL + 门控关）**：7/7/7 全过，31.0s，KDL 调用 26 次 |
| 20:48 | **测试 B2（解析+兜底 + 门控关）**：7/7/7，51.8s，ssik 30 次全败，KDL 26 次（与 A2 完全一致） |
| 20:51 | **测试 A3（实时帧 + 纯 KDL）**：4/4/3，28.2s |
| 20:52 | **测试 B3（实时帧 + 解析+兜底）**：8/8/7，49.5s，ssik 28 次全败 |
| 20:54 | **结论：ssik 58 次求解 0 成功，KDL 唯一有效** |

### 2.8 预检配置固化（20:57-21:22）

| 时刻 | 事件 |
|------|------|
| 20:57 | 用户裁决：预检只走 KDL + 每次求解 0.5s → 改默认值（节点 + launch 双处） |
| 21:20 | 干跑验证：筛选 19.6s（每候选均值 0.40s），6/6/6 通过不受损 |
| 21:21 | 新默认配置真机抓取：candidate [44] grasp_empty，~52s |
| 21:22 | 发现 **B3 executor 残留进程抢 /grasp_execute 服务**（日志被追加 3 轮假数据）→ 清理 |

### 2.9 ready 起点改造（21:2x-21:3x）

| 时刻 | 事件 |
|------|------|
| 21:2x | 用户裁决：预检规划和真机抓取都从 ready 初始 → `_goto_joints` 泛化 + 三处流程改造 |
| 21:34 | ready 起点第 1 次触发：**no_candidate**（0/19，全部死在 approach） |
| 21:35 | 第 2 次：candidate [13] grasp_empty（自动回 ready 分支验证：臂在 zero → 回 ready 执行成功） |
| 21:37 | 第 3 次：**no_candidate**（1/32，approach+grasp 过、死在 lift）——lift 段成为新瓶颈 |
| 21:38 | 用户裁决：**关掉 lift 段预检** → grasp_precheck.py 跳过 lift |

### 2.10 lift 关闭后的稳定期（21:39-21:50）

| 时刻 | 事件 |
|------|------|
| 21:39 | candidate [37] grasp_empty |
| 21:41 | candidate [29] grasp_empty（预检开口朝下 -0.11，FK 0.30——最"水平"一轮） |
| 21:42 | candidate [5] grasp_empty |
| 21:47 | candidate [13] grasp_empty（自动回 ready 分支第 2 次验证） |
| 21:49 | candidate [38] grasp_empty（FK 方向 [-0.13,-0.60,-0.79]，朝下 0.79 理想姿态仍抓空） |
| 21:53 | 用户要求归零 → MoveIt 链 zero（3.6s，FK 56.8,0.1,214.3） |
| 21:55-22:10 | 数据汇总、用户总结讨论（ready 是否更容易——无受控数据，建议补测）、编写本文档 |

---

## 3. 解决的问题（详细诊断过程）

### 3.1 Task 11 耗时瓶颈定位（19:25 首轮）

**现象**：触发 → 结果 ~68s，其中筛选段 ~57s。
**诊断**：Monitor 按阶段标记实时记录——筛选段（candidate 遍历 + 预检）57s 占 79%；执行段（张开标定 3s + 运动 6.3s + 判定 0s + 抬升 1.9s + 回零 4.5s）仅 ~18s。
**每候选时间拆解**（从日志时间戳）：
- 快速拒（self-consistency / score 门控）：**0.00s**（纯内存，13 个）
- 测试候选（30 个）：均值 **1.66s**，中位 1.57s，区间 1.47~3.29s
- 单次 ssik 失败求解 ≈ **1.0s**；失败后每段叠 MoveIt 兜底 ≈ 0.4-0.6s
- 全过候选反而快（~1.66s：ssik 解出 3 段 0.55s/段，无失败惩罚）
- 候选死在哪个阶段决定耗时：approach 段死 → 1.5s；撑到 lift → 3.3s
**结论**：瓶颈不是候选数量多，而是每个失败候选都要付 ~1.0s 的 ssik 求解（30 个测试候选多数死在第 1 段）。

### 3.2 SDK 直控失效（19:52-19:55）

**现象**：sdk_go_ready.py（JointCtrl）FK 全程不动；单发 + 20ms 流式循环都无效。
**诊断证据**：
- `/joint_states` 被 FakeSystem **200Hz** 持续发布（`ros2 topic hz` 实测 200.05）
- CAN 总线 ~3kHz 帧率（candump 2s ≈ 6079 帧）——driver 在密集发指令
- driver 启动参数：`-r joint_ctrl_single:=/joint_states`（remap 订阅当指令）
**结论**：driver 隐式链持续把 FakeSystem 状态压给真机，SDK 帧抢不过（last-writer-wins 在 Hz 级）。
**用户裁决**：禁止 SDK 控制臂轨迹（除非明确要求）——记忆 sdk-control-forbidden。
**补充**：`sdk_go_zero.py` 在 driver 运行时也从未真正移动过臂（目标=当前位姿，未暴露）。

### 3.3 驱动 rclpy 双 shutdown 崩溃（20:25、20:34）

**现象**：`RuntimeError: Context must be initialized before it can be shutdown`，exit 1。
**代码路径**（piper_ctrl_single_node.py）：
```
publish_thread: auto_enable 循环 5s 超时 → rclpy.shutdown()   ← 第 1 次
main(): finally: rclpy.shutdown()                             ← 第 2 次 → RuntimeError
```
**触发条件**：使能失败（20:34 那次臂未响应，5 次 Enable status:False）或会话异常（20:25 那次）。
**修复建议**（未实施）：main() 的 finally 里 `if rclpy.ok(): rclpy.shutdown()` 或 try/except 包裹。

### 3.4 机械臂 CAN 失效（20:35-20:40）

**现象**：SDK EnablePiper 失败、FK 全零、电机无故障标志、can_activate 重置无效。
**诊断链**：
1. can0 接口 UP、无错误帧 → 总线本身正常
2. 适配器 gs_usb 枚举正常（OpenMoko [REDACTED_USB_ID]）
3. **关键发现：can0 接口号从 6: 变 8:，适配器 USB 地址 1-7→1-9**——适配器被重新枚举
4. SDK 报 "CAN port can0 is not UP"（down 状态）→ 重新激活成功
5. 臂断电重启后 FK 从 (56,0,213) 变 (53.3,0.4,189.8)（z -23mm 下垂偏移，正常现象）
**结论**：适配器重枚举 + 臂断电导致链路失效，重新激活 can0 + 臂上电后恢复。

### 3.5 move_group SIGABRT（20:34、20:42）

**现象**：`process has died [exit code -6]`，rviz 等不到 robot_description_semantic。
**根因定位**：
```
what(): parameter 'robot_description_planning.joint_limits.joint1.max_velocity'
        has invalid type: expected [double] got [integer]
```
**事故链**：20:05 用户用 Setup Assistant 导出/同步配置到 src → 20:26 手动编辑 → joint_limits.yaml 数值变 YAML 整数（`5`/`3`/`1`/`0`）→ 20:31 rebuild（symlink-install）→ move_group 参数加载类型检查失败 → SIGABRT。
**修复**：`sed -i -E 's/max_velocity: ([0-9]+)$/max_velocity: \1.0/...'` + rebuild。
**教训**：MoveIt 参数加载严格区分 int/double；SA 导出或手动编辑后必须验证数值类型。
**附注**：Setup Assistant 会话本身确认导出未改动 config 内容（install/config 文件时间戳未变），只重写了 CMakeLists.txt（无副作用，rebuild 会恢复）。

### 3.6 A/B 测试与门控陷阱（20:45-20:54）

**两次误配置的教训**：
- 测试 A/B 启动时未传 `graspnet_6d_analytic_only_precheck:=false`（默认 true）→ **门控短路**：KDL 路径被 Skipping 拦截，33 个候选瞬间全拒（0.0s，33 行 Skipping）
- **代码事实**：门控检查在 ssik 块之外——即使 analytic 关闭，门控仍会拦截 KDL。所以纯 KDL 必须同时关 analytic 和门控
**正式测试结果**（见 §5 完整数据表）。

### 3.7 预检 INFO 日志不落盘之谜（20:5x）

**现象**：testA/B 日志里 "MoveIt pose 兜底规划 PASS"、"scan summary"（INFO）行为 0，但 WARN 失败行都在。
**推断**：TaskStop 强杀（SIGTERM）时 rclpy 输出缓冲未 flush；WARN 行也非全部保留——实际观察到 Prechecker 走 print 分支时格式为 `[grasp_precheck INFO]`。结论：**PASS/summary 行的缺失不影响成功率结论**（服务响应里的 tested/pass 统计是权威数据）；如需精确日志改用 SIGINT 正常停止。

### 3.8 同名节点服务竞争（21:22 发现）

**现象**：executor_testB3_live_analytic.log 被追加 3 轮 scan_only 报告（845 行、4 轮）。
**根因**：B3 测试的 executor 进程在 TaskStop 后残留（launch 被杀、子进程存活），与后续 executor 同名 `grasp_executor`、同服务 `/grasp_execute`——rclpy 服务发现竞争。
**验证**：21:14/21:21 真机抓取有 execute_pick + 回 zero + FK 记录（真机确实执行），未被污染。
**处理**：pkill 清残留；**后续收尾必须按 PID 验证子进程**。

### 3.9 pkill 自匹配坑（3 次，20:4x-21:2x）

**现象**：`pkill -f 'grasp_executor'` 等命令 exit 144（自杀）。
**根因**：命令自身命令行含目标串（pkill 参数本身或同命令 pgrep 参数）→ 匹配到自己的 shell。
**解法**：`pkill -f '[g]rasp_executor'`（括号正则，自身不匹配）+ **同命令内不得再出现裸词**。

### 3.10 后台命令 cwd 残留坑（20:39）

**现象**：`source install/setup.bash: No such file or directory`，日志文件写到错误位置。
**根因**：后台任务继承前台残留 cwd（cd 到 src/piper_ros 后）。
**解法**：所有后台启动命令用绝对路径（source、log 重定向均绝对化）。

---

## 4. 真机抓取全记录（18 次，全部 grasp_empty）

| # | 时间 | 配置演进 | 候选 | score | 预检朝下 | FK 末端方向 | FK朝下 | 筛选 | 执行段 | 结果 |
|---|------|---------|------|-------|----------|------------|--------|------|--------|------|
| 1 | 19:25 | 原配置(解析+1s+zero+三段) | [29] | — | — | — | — | 57s | 18s | grasp_empty |
| 2 | 19:33 | +单帧 | [42] | — | — | — | — | 59s | 17s | grasp_empty |
| 3 | 20:21 | ready起跑(旧逻辑) | [4] | — | — | — | — | 73s | 16s | grasp_empty |
| 4 | 20:24 | 同左 | [21] | — | — | — | — | 59s | 33s | grasp_empty |
| 5 | 20:57 | 纯KDL(1s) | [13] | 0.6466 | 0.19 | [+0.23,-0.54,-0.81] | 0.81 | 52s | 20s | grasp_empty |
| 6 | 21:14 | 同左 | [4] | 1.0205 | 0.40 | [-0.11,-0.30,-0.95] | 0.95 | 39s | 21s | grasp_empty |
| 7 | 21:21 | +0.5s预算 | [44] | — | — | — | — | 32s | 20s | grasp_empty |
| 8 | 21:35 | ready起点(新逻辑) | [13] | — | — | — | — | 12s | 30s | grasp_empty |
| 9 | 21:39 | +lift关闭 | [37] | — | — | — | — | 19s | 28s | grasp_empty |
| 10 | 21:41 | 同左 | [29] | 0.2091 | **-0.11** | [+0.44,-0.85,-0.30] | **0.30** | 11s | 17s | grasp_empty |
| 11 | 21:42 | 同左 | [5] | 0.3979 | 0.27 | [+0.21,-0.55,-0.81] | 0.81 | 10s | 19s | grasp_empty |
| 12 | 21:47 | 同左 | [13] | 0.5062 | 0.38 | [+0.36,-0.52,-0.78] | 0.78 | 14s | 19s | grasp_empty |
| 13 | 21:49 | 同左 | [38] | 0.5984 | 0.40 | [-0.13,-0.60,-0.79] | 0.79 | 11s | 18s | grasp_empty |

> 注：#1-4 为旧配置对照轮；#5-6 纯 KDL 1.0s；#7 起 0.5s 预算；#8 起 ready 起点（#8 是自动回 ready 分支）；#9 起 lift 关闭。
> #13（21:49）score 0.5984、FK 朝下 0.79 为理想抓取姿态仍 grasp_empty——进一步确认与求解/姿态无关。
> 另有 no_candidate 轮：#21:34（0/19）、#21:37（1/32 死在 lift）——lift 关闭后不再出现。

---

## 5. A/B 测试完整数据（scan_only 干跑，不动臂）

### 5.1 受控测试（冻结 CSV：log/frozen_ab_test.csv，189 候选，ready 姿态，门控关闭）

| 指标 | A2 纯 KDL | B2 解析+兜底 |
|------|-----------|--------------|
| 测试候选 | 33 | 33（gate_rejected=17 相同） |
| approach/grasp/lift | **7/7/7** | **7/7/7**（完全相同） |
| 全过候选 | [5] | [5] |
| 筛选耗时 | **31.0s** | **51.8s（+20.8s）** |
| ssik 调用 | 0 | **30 次全失败（0 成功）** |
| KDL 兜底调用 | 26 | 26（完全一致） |

### 5.2 实时帧测试（ready 姿态，帧不同）

| 指标 | A3 纯 KDL | B3 解析+兜底 |
|------|-----------|--------------|
| 实时候选 | 165 | 214 |
| 测试候选 | 28 | 32 |
| approach/grasp/lift | 4/4/3 | 8/8/7 |
| 筛选耗时 | 28.2s | 49.5s |
| ssik 调用 | 0 | **28 次全失败（0 成功）** |

### 5.3 误配置对照（门控默认开，作废轮）

| 指标 | A 纯KDL(门控开) | B 解析(门控开) |
|------|----------------|----------------|
| 筛选耗时 | 0.0s | 20.5s |
| Skipping 行 | 33 | 30 |
| 结论 | 全部被门控短路，KDL 从未被调用 | ssik 失败后 KDL 被门控拦截 |

### 5.4 四轮合计结论

- **ssik 解析 IK：58 次求解，0 成功**——朝下抓位姿的解析解越关节限位被过滤（--respect-limits 默认开）
- **KDL 是唯一有效求解器**（所有全过候选均来自 KDL；两轮同帧对照 KDL 调用次数完全一致）
- 纯 KDL 省 ~40% 筛选时间且结果不变（甚至更好）
- 注意：ssik 并非永远无用（20:21/20:24 轮有成功记录），关闭决策基于本轮 4 轮数据的位姿族特性

### 5.5 每候选耗时分布（新默认配置干跑，207 候选）

- 筛选总耗时 19.6s；每候选均值 0.40s、中位 0.55s、max 0.56s（**0.5s 预算截断生效**）
- 成功段 ~0.22s（KDL IK 5ms 超时内解出 + OMPL 秒回）；失败段 = 预算上限
- 通过率 6/6/6——预算减半未损通过率

---

## 6. 预检流程层次（当前代码结构）

```
层次0 输入: CSV → parse_grasp_csv → rank_candidates(分数降序+8姿态桶轮转) → 前50
层次1 门控: score阈值(默认0) / self_consistency(中心横向≤8cm)   [~0s, 纯内存]
层次2 位姿: make_stage_poses → approach/grasp/lift(grasp.z-0.02/+0.05/+0.10)
层次3 三段: precheck_stages → approach→grasp(2026-08-10: lift已关) AND短路
层次4 求解: precheck_pose
            [analytic关] → [门控关] → _plan_to_pose(KDL: move_group pose目标,
            内部 KDLKinematicsPlugin IK 5ms + OMPL RRTConnect 0.5s预算)
层次5 选择: 全过候选中选"开口朝下"r[1,2]最大者 → 选中即break(单帧)
层次6 执行: 预检前回ready → execute_pick(预位→approach→grasp→闭合→判定→抬升)
            → FK朝向验证 → FK link7落盘 → 收尾回ready
```

关键参数（当前默认）：`enable_piper_analytic_ik_planning=false`、`graspnet_6d_analytic_only_precheck=false`（必须关，否则短路）、`planning_time_precheck=0.5`、`candidate_frames=1`、`READY_JOINTS=[0,1.4919,0,0,-1.22,0]`。

---

## 7. 代码修改明细

| 文件 | 修改内容 |
|------|----------|
| `src/real_piper_perception/real_piper_perception/grasp_executor.py` | ① declare_parameter 默认：`enable_piper_analytic_ik_planning=False`、`graspnet_6d_analytic_only_precheck=False`、`planning_time_precheck=0.5` ② 新增模块常量 `READY_JOINTS` ③ `_go_home()` 泛化为 `_goto_joints(target_joints, label)`（跳过判据=距目标<0.01rad，保留 _go_home 兼容包装）④ 流程新增第 2 段"预检前确保回 ready"（scan_only 跳过，失败 fail-closed）⑤ 删除预执行回 zero ⑥ finally 收尾回 zero→回 ready ⑦ FK 日志"回 zero 前"→"收尾回 ready 前" ⑧ `candidate_frames` 兜底默认 5→1（20:30 单帧裁决） |
| `src/real_piper_perception/real_piper_perception/grasp_precheck.py` | `precheck_stages` 跳过 lift 段（results 保留 lift 键恒 False，all_pass 改两段 AND），docstring 更新为"两段 AND 预检" |
| `src/real_piper_perception/launch/grasp_execute.launch.py` | `enable_piper_analytic_ik_planning` 默认 'true'→'false'、`graspnet_6d_analytic_only_precheck` 默认 'true'→'false'（注释说明门控在 ssik 块外必须关）、`planning_time_precheck` 默认 '1.0'→'0.5' |
| `src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/joint_limits.yaml` | 全部数值整数→double（`max_velocity: 5`→`5.0` 等 7 关节 × 2 字段），修复 move_group SIGABRT |
| `scripts/moveit_go_ready.py`（新增） | SRDF 动态解析命名位姿 → moveit_adapter 规划+执行；参数化：`python3 scripts/moveit_go_ready.py <zero\|ready>`；已修 spin 线程收尾（executor.shutdown + rclpy.shutdown） |
| `scripts/sdk_go_ready.py`（新增） | SDK MoveJ 直控 ready（流式 JointCtrl + FK 稳定判据）；验证失败（driver 运行时无效），备用保留 |
| 测试 | 36 passed + 1 skipped（未新增用例；行为变化已被现有用例覆盖——lift 键保留兼容） |

---

## 8. 日志文件索引（log/）

| 文件/前缀 | 内容 |
|-----------|------|
| `driver_session_20260810_*` | 驱动各次启动（192344 正常；203418 崩溃；204053 恢复后正常） |
| `moveit_session_20260810_*` | MoveIt demo（203422 含 SIGABRT；204216 修复后正常） |
| `bridge_session_*` | 相机桥接 |
| `graspnet_session_*` | Docker 推理（gui/headless） |
| `executor_session_20260810_192454.log` | #1-2 轮（19:25-19:35） |
| `executor_session_20260810_1931xx.log` | #3-4 轮（20:21-20:25） |
| `executor_testA_kdl.log` / `executor_testB_analytic.log` | 误配置轮（门控短路，作废） |
| `executor_testA2_kdl.log` / `executor_testB2_analytic.log` | 冻结 CSV 正式 A/B |
| `executor_testA3_live_kdl.log` / `executor_testB3_live_analytic.log` | 实时帧 A/B（**B3 被残留进程追加 3 轮，只取第 1 轮 20:52**） |
| `executor_kdl_real_20260810_205645.log` | #5-6 轮（20:57-21:15） |
| `executor_verify_newdefaults.log` | 新默认配置干跑（207 候选） |
| `executor_verify_singleframe.log` | 单帧验证干跑 |
| `executor_session_20260810_212040.log` | #7 轮（21:21） |
| `executor_session_20260810_2133xx.log` 及后续 | #8-13 轮（21:34-21:50，ready 起点 + lift 关闭） |
| `frozen_ab_test.csv` | A/B 冻结候选帧（189 候选，log/ 下） |

---

## 9. 后续计划 / 待办

1. **抓空根因排查（主线，第 18 次 grasp_empty 后优先级最高）**：
   - 感知噪声：候选位置每帧波动 8-14cm（08-08 已定，多帧中值滤波曾试失败回退）
   - z 系统性偏深 25-60mm：`grasp_z_offset=0.02` 可能不够（可测：逐步加大 offset 观察 FK link7 与感知位置比对——executor 已落盘 fk_link7_position.json 支持）
   - 方向偏好：全过候选朝下分量低时选中"水平接近"候选（21:41 轮 [29] 预检 -0.11 / FK 0.30）；可加"朝下分量下限门控"
   - 预检方向分量 vs FK 实测方向系统性偏差（口径不同，值得核对 graspnet_to_tcp_rotation 变换链）
2. **起点受控对比（建议，1 次测试钉死）**：同冻结 CSV + 0.5s 预算，scan_only 分别在臂物理 ready/zero 干跑，对比 approach_pass 与耗时——回答"ready 是否更容易"（当前无受控数据；ready 起点通过率波动大 0/19、1/32 均出现过，zero 起点历史更稳）
3. **驱动双 shutdown bug 修复**：`piper_ctrl_single_node.py:438` main() finally 的 `rclpy.shutdown()` 加 `rclpy.ok()` 防护（复现 2 次，enable 失败路径必现）
4. **预检 INFO 日志丢失**：PASS/summary 行在强杀时不落盘；如需精确日志用 SIGINT 正常停止或改文件日志
5. **通过率稳定性**：ready 起点 + 0.5s 预算波动大，可考虑动态预算（成功段 0.22s 远低于预算）
6. **运维固化**：收尾必须按 PID 验证子进程（TaskStop 不杀子进程）；pkill 用括号技巧；后台命令全绝对路径

---

## 10. 记忆沉淀（跨会话生效）

- `precheck-pure-kdl`：纯 KDL + 0.5s 预算 + ready 起点 + lift 关闭的完整演进、原因与代码默认值
- `sdk-control-forbidden`：用户规则——不特别要求时禁止 SDK 控制臂轨迹（夹爪 GripperCtrl 除外）
- `sdk-vs-driver-contention`：driver+MoveIt 运行时 SDK 直控不生效的技术机制（200Hz /joint_states + 3kHz CAN）
