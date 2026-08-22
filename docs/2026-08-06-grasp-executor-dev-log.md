# 2026-08-06 真机 Grasp Executor 开发日志

## 结论摘要

仿照仿真 piper_graspnet 的规划抓取，完成了真机**单次抓取验证**功能的全部代码开发
（感知 → CSV → 候选筛选 → 4 段预检 → MoveIt 执行 → 夹爪反馈判定），经 superpowers
流程（brainstorming → spec → writing-plans → 子代理并行执行 → 最终审查 → 2 轮修复）
交付，**31/31 测试全绿**。真机联调（三阶段验证）按用户指示保留到下次会话。

设计文档：`docs/superpowers/specs/2026-08-06-grasp-executor-design.md`
实现计划：`docs/superpowers/plans/2026-08-06-grasp-executor.md`
变换链调研：`docs/grasp_executor_transform_notes.md`
进度账本：`.superpowers/sdd/grasp-executor/progress.md`

## 一、交付文件

```
src/real_piper_perception/real_piper_perception/
├── grasp_transform.py    变换模块：handeye 加载(真实标定 camera_data/)、相机→基座、
│                         calib_18 TCP 映射（C++ kGraspNetToPiperTcpR，仿真验证值）
├── grasp_candidates.py   CSV 解析(24 列)/分数排序/8 姿态桶 round-robin/自洽检查
├── grasp_precheck.py     4 段预检：OMPL plan + ssik 解析种子 + FK 回验(0.015m/0.1rad)
│                         + analytic-only 门控 + scan 统计
├── grasp_execute.py      执行序列：预位/approach(笛卡尔直线)/grasp/闭合夹爪/抬升
│                         + 夹爪反馈等待(带超时) + 钳制 fail-closed
├── grasp_verify.py       夹爪判定纯函数（总开度域直比）
├── grasp_executor.py     主节点：grasp_execute 服务/键盘'g'触发/MultiThreadedExecutor
│                         + ReentrantCallbackGroup/编排/grasp_result.json 原子落盘
├── scripts/piper_ssik_ik.py   ssik 解析 IK 子进程（从仿真原样拷贝）
launch/grasp_execute.launch.py  一键启动（默认 scan_only:=true）
test/                     5 个测试文件 + fixtures（31 用例全绿，双目录可跑）
```

## 二、关键调研结论（变换链）

- **CSV 语义**：r00..r22 是 GraspNet 原始抓取系旋转（相机系原值）；坐标为相机系米制；真实管线 CSV 为 **24 列**（计划文档"25 列"为笔误，已修正）
- **变换链**：`R_base_tcp = R_cam2base @ R_grasp_cam @ GRASPNET_TO_PIPER_TCP_R`（外参左乘、轴映射右乘）
- **calib_18 裁决**：Python 侧 GRASPNET_TO_PIPER_TCP_R 与 C++ kGraspNetToPiperTcpR 数值不同（相差 ~180°）——采用**仿真闭环验证过的 C++ calib_18**（[[0,-1,0],[0,0,-1],[1,0,0]]）
- **handeye**：eye-to-hand、3x3 旋转矩阵 + 米制平移，camera→base（真实文件 `camera_data/handeye_eye_to_hand.yaml` 解析验证 det=1.0、T=[0.484,-0.0446,0.5963]）
- **ssik**：独立预编译解析 IK 包（与 Pinocchio 无关，ssik 3.3.1 已装）；烘焙限位导致部分位姿无解（=不可达，analytic-only 门控预期行为）
- **真机 MoveIt 链路**（官方教程）：can_activate.sh → start_single_piper.launch.py（remap /joint_ctrl_single→/joint_states）→ demo.launch.py（mock 控制器轨迹→/joint_states→真机）；`piper_humble` 是空包不可用；本机无 moveit_py（MoveIt 交互为接口占位，Task 9 联调时定调用方式）

## 三、最终审查发现与修复（2 轮）

| # | 级别 | 问题 | 修复 |
|---|---|---|---|
| C1 | Critical | 服务回调 join 阻塞单线程 executor → 反馈回调饿死，服务触发判定恒失败 | MultiThreadedExecutor + ReentrantCallbackGroup 双修复（第二根因 MutuallyExclusiveCallbackGroup）+ MTE 集成测试 |
| C2 | Critical | joint7 真机为 **prismatic 米制半行程**（限位[0,0.035]、总开度=2×joint7、独立 gripper 组）；代码"弧度 1:1"假设被推翻 | 核实 URDF/SRDF/驱动/sdk 后重建：`joint7_cmd=(width+margin)/2`、命令反馈同米制总开度域直比、注释更正（"弧度"为驱动历史遗留命名） |
| I1 | Important | 'grasp_tcp' 端点在真机 URDF 不存在 | 端点改 link6；0.14m 偏移注释更正（link6→指尖≈0.136m 仿真近似） |
| I2 | Important | max_candidates=1 + 门控交互 → 真实 37 候选 tested=0 | 门控失败 continue + 默认 50 + gate_rejected 报告（实测 tested=11/gate_rejected=26） |
| I3 | Important | grasp_result.json 记录原始 width 而非实发命令 | cmd_width 全路径回传统一 |
| N1 | Important(修复引入) | joint7 钳制与判定未联动 → 宽物体误报 success 掉件风险 | **fail-closed**：越限直接失败停在 grasp 位不抬升（结构性消除）+ 4 例测试 |

验证正确：变换链数学、FK 门限、预检语义、判定边界、velocity_scale 安全、ssik 契约。

## 四、测试与验证状态

- **31/31 单测**（candidates 3 + transform 3 + precheck 7 + verify 3 + execute_units 10 + MTE 集成 1 + N1 4），仓库根与包目录内均全过
- **scan_only 干跑冒烟**：37 真实候选 → tested=11, gate_rejected=26（臂不动）；服务可见、键盘'g'触发、grasp_result.json 原子落盘、Ctrl+C 干净退出
- **真实标定文件**解析 OK；ssik 冒烟（可达/不可达/错误格式）OK；夹爪换算往返验证 OK
- 未实测：MoveIt 规划/执行（无 moveit_py 占位）、真机动作、夹爪反馈真值（弧度/米制已按源码核实）

## 五、真机环境准备（已完成部分）

- moveit 依赖 26 包 ✅、ros2_control/controllers/gripper-controllers ✅
- CAN：`can_activate.sh can0 1000000` 激活成功（can0 UP、1Mbps、ERROR-ACTIVE）
- 相机 USB 3.x（5000）✅；机械臂已插电

## 六、待办（下次会话：真机联调，需机械臂在位）

1. **Task 2 冒烟**：三启动件（start_single_piper → demo.launch.py → rviz 手动动臂确认官方链路）；**核实 N2**：GRIPPER_VAL_MULTIPLE=2 硬编码 vs launch 默认 gripper_val_mutiple=1 的实际参数
2. **Task 9 干跑**：MoveIt 接线（确定调用方式：move_group action client / moveit_py 补装）→ scan_only 全链路（含真实标定、预检可达性报告）；核实 TCP 偏移 0.14m 与 URDF；使能检查（M10）在首动前实现
3. **Task 10 低速单段**：只 approach（velocity_scale=0.1），验证 MoveIt↔真机桥接
4. **Task 11 完整抓取**：夹爪判定真值标定（含 margin 与宽物体阈值）、抓取验证 3 次
5. 遗留 minor：M2 参数说明、M5 无 CSV message、M6 ssik 路径 symlink 依赖、M7 预位/抬升参数说明、M8 中断处理、M10 使能检查、N3 测试 flaky

## 七、关键接口备忘（下次会话直接可用）

- 触发：`ros2 service call /grasp_execute std_srvs/srv/Trigger {}` 或运行终端按 `g`
- 参数：`scan_only`（默认 launch true）、`velocity_scale=0.1`、`handeye_yaml=camera_data/handeye_eye_to_hand.yaml`
- 结果：`shared_camera_data/grasp_result.json` + `/grasp_result` 话题
- 夹爪：joint7 半行程米制（0=闭、0.035=单指全开）；命令/反馈同米制总开度域；宽物体越限 fail-closed
