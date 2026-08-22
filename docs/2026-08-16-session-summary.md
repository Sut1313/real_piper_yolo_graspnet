# 2026-08-16 会话总结（TF 可视化 / ROI 检验 / 零位预检 / 预检可视化 / TCP 计划）

> 本文档供下一次对话使用：记录本日所有工作、目的、关键数据、环境经验与未完成事项。
> 会话结束时**全部后台进程已关闭**（相机链路、零位环境、推理容器、全部窗口）。

---

## 一、会话目的链（为什么做这些）

1. 用户要求用 `- | ->` 图总结项目架构与数据流（认知基线）
2. 编写 TF 链可视化测试程序 → 迭代为离线 3D 坐标架工具（死数据：URDF + 手眼标定）
3. 验证一个工程疑点：ROI 截取是否导致 GraspNet 坐标偏移（实证检验）
4. 无机械臂硬件环境下，按 zero 位姿对实时感知候选做抓取预检（跑通现有程序）
5. 把预检结果可视化到推理窗口（原页面全量 + 新页面预检成功候选）
6. 为 8.17 的 TCP 偏移修正写实施计划（只写不实施）

---

## 二、完成工作详情

### 1. TF 树离线可视化工具 `scripts/tf_tree_viewer.py`（428 行，测试 33 项全绿）

**目的**：不依赖 ROS/硬件，用静态数据画出完整 TF 关系（坐标架朝向 + 位置 + 位移链）。

数据源（全部死数据）：
- `src/piper_ros/src/piper_description/urdf/piper_description.urdf`（SolidWorks 多行属性格式）
- `camera_data/handeye_eye_to_hand.yaml`（T_cam2base）
- 相机内链：`src/realsense-ros/realsense2_description/urdf/test_d435i_camera.urdf.xacro`（xacro 编译，需 `AMENT_PREFIX_PATH=install/realsense2_description`，且 `use_nominal_extrinsics:=true` 才有流关节）

功能演进（用户逐步要求）：
- 3D 坐标架视图：每帧黑点（位置）+ XYZ 三轴（朝向）+ 帧名；父子灰线（位移）
- 删除每帧 T/RPY 文字标注（拥挤）→ 只留帧名
- 相机只显示 `camera_link`（内链标称帧隐藏）
- 位置重合检测 `find_coincident_frames`：link1/link2、link4/link5、link6/gripper_base、link7/link8 原点重合（URDF 零位移关节/共享铰点），标签合并显示
- `visible_links`：keep 集合内帧沿树路径最短连线（跳过隐藏中间帧）
- **最终默认显示 = 真机抓取 TF 链 10 帧**：`GRASP_CHAIN_FRAMES = base_link, link1-6, gripper_base, link7, camera_link`（依据 piper.srdf：arm 组 base_link→link6 仅 joint1-6，gripper 组 gripper_base→link7 仅 joint7；link8 是 joint7 镜像不在规划组）
- 链路查询 `--from A --to B`：任意两帧（最近公共祖先 + 反向边取逆），打印每步 + 累计 T/R

纯函数（可单测）：load_urdf_edges / load_handeye_edge / compile_camera_xacro / compose_tree / find_chain / chain_transform / compute_frame_poses / find_coincident_frames / visible_links / render_ascii。

测试：`test/test_tf_tree_viewer.py` 33 项。

### 2. ROI 坐标偏移检验 `scripts/roi_coordinate_check.py`

**目的**：实证"ROI 截取是否导致坐标偏移"（用户疑点）。

**结论（数值实证，3 帧稳定）**：**无偏移**。
- 机制：ROI 是**掩码**（`inference_from_camera.py:482-489` `roi_mask &= workspace_mask`），图像不裁剪、内参不修改、点坐标原样
- A 全图 vs B ROI 掩码：同像素坐标 `max|Δ| = 2.96e-08 m`（float32 舍入噪声，物理=0）
- 对照 C（若真裁剪+内参不补偿）：偏移 0.208/0.130 m（20cm 级！），与理论 `(x0/fx)·z` 吻合——这是未来改造的防坑红线
- 中心点差 0.029m = 点集合统计差异，非坐标偏移

### 3. 相机可视化推理运行（真机 D435i）

链路：`run_d435i_bridge.sh`（realsense2_camera_node + d435i_graspnet_bridge + handeye_tf_publisher）→ 共享目录握手 → `run_graspnet_d435i.sh gui`（docker 推理 + matplotlib 可视化）。

实测：640×480@30 双流，ROI x=[0.30,0.80] y=[0.25,0.75]（动态 roi.json），推理 0.09s/帧、avg 0.32s，碰撞过滤后 ~39-73 候选，CSV 持续更新。

### 4. 零位机械臂预检（无硬件环境，跑通）

**目的**：当前无机械臂硬件，按 zero 位姿对实时感知候选做抓取可达性预检。

环境搭建（全部无驱动无 CAN）：
```bash
ros2 launch piper_with_gripper_moveit rsp.launch.py          # robot_state_publisher
ros2 run joint_state_publisher joint_state_publisher         # /joint_states 全 0（zero 位姿）
ros2 launch piper_with_gripper_moveit move_group.launch.py use_sim_time:=false
ros2 launch real_piper_perception grasp_execute.launch.py scan_only:=true
# 触发：ros2 service call /grasp_execute std_srvs/srv/Trigger "{}"
```

验证结果：
- 零位 TF 与离线计算一致（link7 @ [0.191, 0, 0.225]）
- 预检报告示例：`candidates=69 tested=60, gate_rejected=9, approach_pass=3, grasp_pass=2 → 候选 [39] 全过`
- **坑：move_group 长时间承受大量失败规划后 OMPL 状态异常（"Invalid goal state"、空帧名警告）→ 预检全败 → 重启 move_group + executor 即恢复**。下次遇到预检 0 通过先重启 move_group。
- 已知无害告警：octomap 深度传感器插件加载失败（项目不用此功能）
- 服务名注意：executor 的 service 注册名是 `/grasp_execute`（无节点名前缀）

### 5. 预检结果可视化（双窗口 + 中文修复）

**目的**：让推理可视化窗口显示"机械臂能用的候选"。

数据流：executor 预检 → `shared_dir/precheck_results.json`（相机系位姿：approved 列表 + selected，`build_precheck_results` 原子写）→ 推理侧 `load_precheck_results`（mtime 缓存）→ `precheck_match.match_precheck` 位姿匹配（位置 2cm + 角度 90° 宽松——冻结帧与新帧候选方向会不同）→ 渲染。

**两个窗口**（display_loop 双 fig）：
- 原窗口 "GraspNet - Perception Results"：**全部** graspnet 候选（用户要求恢复全量，回滚了最初的过滤方案）
- 新窗口 "GraspNet - 预检成功候选"：只显示 approved 匹配候选（**graspnet 默认线框色**，红色方案已被用户删除），mtime 缓存渲染（预检低频，避免每帧二次 Open3D 渲染）

**中文乱码修复**：`cv2.putText` 仅 ASCII（中文 → ????）。新增 `graspnet-baseline/text_overlay.py`（PIL ImageDraw + Noto Serif CJK），字体从宿主机挂载：`/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc → /workspace/fonts/`（docker 镜像无中文字体！）。字体缺失回退 ASCII。容器内实测中文渲染成功。

**踩坑记录**：
- GraspGroup.__getitem__ 不支持 tuple 索引（`gg[:, 13:16]` 报错）→ 用 `.translations` / `.grasp_group_array` 提取
- `--frames`/keep 语义迭代：`visible_links` 方案

测试：`test/test_precheck_match.py` 6 项 + `test/test_text_overlay.py` 4 项。

### 6. 8.17 计划：TCP 偏移修正（只写不实施）

文件：`docs/superpowers/plans/2026-08-17-tcp-offset-urdf-fix.md`

**调研结论（关键事实）**：
- 当前代码 `TCP_OFFSET_FROM_LINK6_Z = 0.14`（grasp_precheck.py:54，仿真近似，来源 piper_analytic_ik.cpp:30）
- **URDF 几何矛盾**：joint7/8 铰点 link6 系 Z = **0.1358m**（代码验证 R6ᵀ(pos7-pos6)=[0,0,0.1358]）；link7.STL 变换到 link6 系实测指尖平面 Z = **0.0593~0.0658m**（手指沿 -Z 延伸 76.5mm）；**0.14m 在夹爪结构本体之外**
- precheck.py:51 注释"指尖平面 ≈ 0.136m"本身写错（0.136 是铰点）
- 候选值：A=0.1358（铰点）/ B≈0.06（指尖夹取面）/ C=0.14（弃用）——Task 0 用真机尺量裁决

计划结构：Task 0 裁决（实测）→ Task 1 改常量+注释+测试（TDD）→ Task 2 全量回归+干跑验证。

---

## 三、关键事实速查（下次对话直接用）

| 事实 | 值 | 出处 |
|---|---|---|
| 抓取 TF 链帧集 | base_link, link1-6, gripper_base, link7, camera_link | GRASP_CHAIN_FRAMES（tf_tree_viewer.py） |
| 零位 link7 位置 | [0.191, 0, 0.225] m | tf2_echo 实测 = 离线计算 |
| 夹爪铰点高度 | link6 系 Z = 0.1358 m | joint7/8 origin |
| 指尖平面 | link6 系 Z ≈ 0.06 m（0.0593~0.0658） | link7.STL 实测 |
| 手指行程 | 每指 0.035m，总开度 0.07m | joint7/8 limit |
| 手眼外参 | T_cam2base t=[0.444, -0.097, 0.745] | handeye_eye_to_hand.yaml |
| ROI 掩码 | 无坐标偏移（max\|Δ\|=2.96e-08m） | roi_coordinate_check.py |
| 推理性能 | 推理 0.09s/帧，avg 0.32s，碰撞过滤 ~39-73 | /tmp/inference.log |
| executor 服务 | /grasp_execute（std_srvs/Trigger） | grasp_executor.py:153 |

## 四、环境/进程管理经验

- **pkill -f 自匹配坑**：`pkill -f "xxx"` 会杀掉包含该字符串的 bash 自身（exit 144）→ 用 `kill $(pgrep -f "xxx" | grep -vw $$)` 或 `kill <PID>`（先 pgrep 列出）
- 幽灵 X 窗口：进程死了窗口还在 → `DISPLAY=:1 xkill -id <win_id>`（xwininfo -root -tree 查）
- matplotlib 窗口在 DISPLAY 存在时阻塞前台——后台运行必须 `setsid nohup ... &`
- **move_group 无硬件可独立运行**（use_sim_time:=false），但承受大量失败规划后状态异常需重启
- 零位关节状态：joint_state_publisher 默认全 0 = zero 位姿

## 五、本日新增/修改文件清单

| 文件 | 职责 |
|---|---|
| `scripts/tf_tree_viewer.py` | 离线 TF 树 3D/ASCII/链路查询 |
| `test/test_tf_tree_viewer.py` + fixtures | 33 项测试 |
| `scripts/roi_coordinate_check.py` | ROI 偏移检验 |
| `graspnet-baseline/precheck_match.py` | 预检位姿匹配（纯 numpy） |
| `graspnet-baseline/text_overlay.py` | 中文叠加（PIL + Noto CJK） |
| `test/test_precheck_match.py` / `test/test_text_overlay.py` | 匹配/中文测试 |
| `grasp_candidates.py` + test | build_precheck_results（-1 防御） |
| `grasp_executor.py` | 预检结果写 precheck_results.json |
| `inference_from_camera.py` | 双窗口 + 预检过滤（highlight 模式）+ 中文标注 |
| `run_graspnet_d435i.sh` | 挂载 precheck_match/text_overlay/中文字体 |
| `docs/superpowers/plans/2026-08-17-tcp-offset-urdf-fix.md` | 8.17 TCP 偏移修正计划（未实施） |

## 六、下次对话须知

1. **8.17 执行计划**：从 Task 0 开始——真机尺量指尖夹取平面，裁决 NEW_TCP_OFFSET（0.1358 vs ≈0.06），再改 `grasp_precheck.py:54` + `test_grasp_execute_units.py:68,80` 断言 + 三处注释
2. **零位预检环境可复用**：四步启动命令见 §4；预检前若 move_group 状态异常先重启
3. 预检可视化链路已通：触发预检 → precheck_results.json → 新窗口显示 approved（默认色）
4. 未完成/待定：ROI 显示"不正常"的具体现象未定位（像素验证框绘制正确，疑似暂停冻结或框物不重合——下次用户描述现象再查）
