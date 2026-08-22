# 2026-08-07 真机联调会话总结

> 继续 2026-08-06 grasp-executor 计划（Task 9/10/11 真机联调）。
> 本日志记录：遇到的问题、解决方法、代码修改、后续计划。

## 一、会话主线

1. 相机 USB3 检查（换原装线修复 480M→5000M）
2. Task 9 scan_only 干跑 + MoveIt 接线（大量修复）
3. 轨迹失控事故排查（plan_only 缺陷）
4. 夹爪 SDK 直控改造（绕开 MoveIt gripper 组）
5. 轴映射多轮切换（最终：新映射 M，夹爪开口朝下）
6. TF 朝向关系梳理（joint2 语义、开口方向 = -Z_base）
7. Task 10 执行验证（5 帧检测 + 开口朝下 0.84→0.92）

## 二、遇到的问题与解决

### A. 相机 USB（独立于联调）
| 问题 | 根因 | 解决 |
|------|------|------|
| D435i 一直 480M（USB 2.0） | 非原装线（4 芯无 SS 差分对） | 换原装线 → 5000M ✅ |
| 鉴别方法 | 换 N 口均 high-speed + 机器日志 super-speed=0 | 判定线的问题（已记入记忆） |

### B. MoveIt 接线（moveit_adapter.py 手写适配器）
| 问题 | 根因 | 解决 |
|------|------|------|
| `/move_group action server 不可用` | MoveIt2 humble action 名是 `/move_action` | 改连 `/move_action` |
| **plan() 规划即执行（致命，致失控）** | `planning_options.plan_only` 默认 False | 显式 plan_only=True |
| plan/execute 卡死崩溃 | humble `send_goal` 阻塞式；`Future.result()` 无 timeout；`get_result_async` 返回 GetResult_Response 包装；Result 字段是 `planned_trajectory` | send_goal_async + 轮询 `_wait_future` + `.result().result` + `_PlanResult` |
| executor 干跑 plan 10s 超时（move_group 16ms 已响应） | action client 在默认 MutuallyExclusive 组 | 全部异步组件移入 ReentrantCallbackGroup |
| **规划终点 z 偏 22cm（"臂不在物体上方"真正根因）** | `PositionConstraint.header.frame_id` 缺失 | 补 `frame_id='base_link'`（规划终点偏差 <1cm） |
| FK 响应取值失败 | 按 frame_id 匹配失败 | 按请求顺序取值（pose_stamped[0]/[1]） |

### C. ssik 解析 IK
| 问题 | 根因 | 解决 |
|------|------|------|
| ssik 脚本路径不存在 | launch 用 abspath 不解析 symlink | 改 realpath + setup.py 打包 scripts |
| `--matrix: expected one argument` | 17g 精度科学计数法负值被 argparse 当选项 | 改 `--matrix=<value>` 等号形式 |
| 朝下姿态全部无解（0/8） | Piper joint2 限位 [0°,180°]（不能前倾，真机 0x472 查询确认） | 数值 IK 兜底（_plan_to_pose + Pinocchio） |

### D. 夹爪控制
| 问题 | 根因 | 解决 |
|------|------|------|
| 夹爪"直接开到最大"、mock 轨迹走不完 | MoveIt gripper 组 → mock 链路问题 | **SDK 直控**（用户裁决）：GraspExecutor 直接 `GripperCtrl(angle,1000,0x01,0)`，0.001mm 单位、总开度×1e6 |
| 驱动与 executor 夹爪指令打架 | 双源发 0x159 | 驱动加 `gripper_forward` 参数（false 时不转发夹爪） |
| SDK 直控验证 | — | 张开 69.7mm（×2 可重复）、闭合 4.3mm、闭→开 69.7mm ✅ |

### E. 感知-执行互锁（用户需求）
- **需求**：执行中 graspnet 停止推理（避免机械臂被识别进目标）；执行完毕、机械臂归零后恢复推理
- **实现**：executor 触发写 `pause_inference` 标记 → graspnet 每帧检查跳过；流程 finally：收尾归零 → 删标记
- **实测**：执行期间 109 帧被跳过、归零后恢复 ✅

### F. 轨迹失控事故（19:30，严重）
- **现象**：机械臂高速运动触发保护失能
- **根因链**：适配器 plan() 未设 plan_only → "规划即执行" → 外部脚本（moveit_pose_test 孤儿进程）plan 即执行高速轨迹 → 驱动无差值保护转发 → 失控
- **处理**：杀孤儿进程（go_zero/moveit_pose_test 残留）、修 plan_only、修驱动崩溃路径认知
- **教训**：多层系统"全部成功"可能只是"全部自洽"——必须闭环交叉验证（规划终点 FK vs 目标、执行后 FK vs 目标、目测独立参考）

### G. 手眼标定与位置
- 更换新 handeye 文件（相机位置 (0.47,-0.008,0.703) 垂直朝下）
- 物体可达高度标定：需垫高（物体中心离地 22~25cm）
- 双向验证：感知→handeye→base vs FK→逆 handeye→cam，**误差仅 4mm** ✅

### H. TF 朝向关键认知（用户纠正与验证）
- **joint2 正方向 = 前倾**（朝 +X，link6 向前下方）——URDF/真机查询 [0°,180°] 单向
- **夹爪开口方向 = link7 Z 轴 = -Y_link6 = GraspNet +X（base）≈ -Z_base（竖直向下）**——即"从上方抓"
- GraspNet +X（接近方向）视觉上 = 相机 +Z（用户确认）
- Piper 可达的"开口朝下"姿态：link6 前向斜上（z∈[0.2,0.5]）+ Y 竖直（KDL 实测）；前向水平/朝下不可达

### I. 轴映射演进（多轮）
1. calib_18（水平接近）→ 用户反馈"末端没朝下"
2. 朝下映射（link6 前向朝下）→ ssik 全无解 + 位置偏差 → 回退
3. **最终新映射 M=[[0,-1,0],[1,0,0],[0,0,1]]**：link6 前向=GraspNet 法向（斜上/水平）、TCP+Y=-GraspNet+X（Y 竖直→开口朝下）——**执行后夹爪开口朝下 0.84~0.92** ✅

### J. 候选与时域问题
- GraspNet 方向/得分时域不稳定（每帧候选方向分布波动大；得分 0.02~0.46 波动）
- **5 帧检测**（executor 内最多 5 帧 CSV 逐帧预检，任一帧全过即执行）——解决"碰运气"
- 得分阈值（0.45→0.33→**关闭**，用户裁决）——launch 不透传导致参数失效（已补透传 + 默认 0.0）
- score>1 异常候选（感知输出异常，记录待查）
- TOP_K 200→50（候选 118→~50，控制预检耗时）

## 三、代码修改清单

| 文件 | 修改 |
|------|------|
| `src/real_piper_perception/.../moveit_adapter.py` | 全新适配器（/move_action、plan_only、异步修复、Reentrant 组、frame_id） |
| `.../grasp_transform.py` | GRASPNET_TO_PIPER_TCP_R 多轮切换（最终 M） |
| `.../grasp_precheck.py` | ssik 等号参数、数值 IK 兜底（_plan_to_pose） |
| `.../grasp_execute.py` | SDK 夹爪直控（_gripper_set_width/_open_gripper）、max_stage 段截断 |
| `.../grasp_executor.py` | 回 zero 段、感知互锁、5 帧检测、方向偏好（开口）、score 阈值、FK 朝向验证、候选预筛 |
| `.../grasp_ik.py` | **新增**：Pinocchio DLS 数值 IK |
| `launch/grasp_execute.launch.py` | dry_run_max_stage、grasp_score_min 透传、analytic_only 默认 |
| `src/piper_ros/.../piper_ctrl_single_node.py` | gripper_forward 参数 |
| `.../start_single_piper.launch.py` | gripper_forward 透传 |
| `setup.py` | scripts/ 打包（M6） |
| `graspnet-baseline/inference_from_camera.py` | pause_inference 检查 |
| `docs/grasp_executor_test_log.md` | Task 9/10 测试日志 |

## 四、当前状态（会话结束时）

- 环境：全部进程已停止（用户指示"停止所有进程"）；can0 UP
- 配置（下次启动）：
  - 驱动 `gripper_val_mutiple:=2 gripper_forward:=false`
  - graspnet `TOP_K=50`
  - executor `scan_only:=false dry_run_max_stage:=approach graspnet_6d_analytic_only_precheck:=false`（阈值默认关）
  - 5 帧检测 + 方向偏好（开口朝下）+ FK 朝向报告
- 执行验证：Task 10 三次成功，开口朝下 0.84→0.89→0.92

## 五、后续计划

1. **启动修复验证**：launch grasp_score_min 透传（'0.0'）修复后按标准流程启动 → 触发 Task 10 确认无回归
2. **Task 10 收尾**：验证 5 帧检测 + 候选 50 下预检耗时可控；候选 score>1 异常排查
3. **Task 11 完整抓取**（用户此前限定不执行，后续可再开放）：grasp 下探 + 闭合夹爪 + 判定 + 抬升 ×3
4. **感知质量**：GraspNet 方向/得分时域稳定（方向过滤/多帧累积）
5. **已知遗留**：
   - 感知得分波动（0.02~0.46）与 score>1 异常
   - 夹爪判定真值标定（Task 11 时）
   - 真机 SDK GetArmEndPoseMsgs 返回 0（末端位姿用 FK/变换链替代）
   - graspnet 容器长跑段错误（OpenCV 显示，~2600 帧崩溃——需定期重启或 headless）
