# 真机抓取误差原始测量日志

> 本文件是 Claude Code 的原始数据记录模板。只填写观测结果，不在这里写未经验证的根因结论。所有长度单位为 mm，角度单位需明确注明。

## 测试环境

| 项目 | 值 |
|---|---|
| 测试日期/时区 | 2026-08-18 18:51 起，CST (UTC+8) |
| 主机/容器 | 宿主机（非 git 仓库，无版本号）；GraspNet 推理在 docker `graspnet-baseline:20.04`（headless） |
| ROS_DISTRO/Python | humble / Python 3.10.12（宿主机） |
| 代码版本或非 git 说明 | 非 git 仓库；源文件 mtime 见下方；handeye YAML 修改于 2026-08-18 18:37 |
| 相机型号/序列号 | Intel RealSense D435i，USB3.0（/sys speed=5000），Bus 002 Dev 003 (8086:0b3a) |
| Piper CAN 端口 | can0（状态：DOWN，未激活——阶段 D/E 阻塞项）；USB-CAN 适配器 OpenMoko GS CAN ([REDACTED_USB_ID]) 已连接 |
| 目标物/治具 | 现场场景（深度中值 ~778mm；目标物体在 640x480 画面中），未放置标定治具 |
| 相机和目标是否移动 | 未移动（固定安装，eye-to-hand） |

## 输入文件一致性

### 固定离线帧（阶段 B，2026-08-16 22:31 采集，未覆盖）

| test_id | frame_id | 文件 | mtime | size | sha256 |
|---|---|---|---|---|---:|---|
| B1 | 20260816_2231 | shared_camera_data/color.png | 1786890672 | 292499 | 3220c4732e2ea1708dcb75e02aeba59e0a3c70ee2711e49383c38119c8c2a3bd |
| B1 | 20260816_2231 | shared_camera_data/depth.png | 1786890672 | 85255 | 0235e791dd66987308e1eb54b187513744f2042210089336b495ffa69755bc13 |
| B1 | 20260816_2231 | shared_camera_data/meta.mat | 1786890672 | 352 | e7eb9263682940df16fde8a69c7df54cfc522eae192920a19c03af84f8b25e10 |
| B1 | 20260816_2231 | shared_camera_data/perception_grasps.csv | 1786890669 | 23135 | b30ad320e2506b2c9e260ecdaac3ec681fc5ef4a315354d280350a766c5c5ae8 |
| B1 | 20260816_2231 | shared_camera_data/last_validation.json | 1786890669 | 84330 | 5b209f86a294da6ae89c2df0de5947c4b10a5200fdf599d188bf57673ec13393 |
| B1 | 20260816_2231 | shared_camera_data/roi.json | 1786017383 | 107 | 9caec2bb918d8e97（x=[0.3,0.8] y=[0.25,0.75] 归一化） |

副本：`docs/real_grasp_error_investigation/frame_20260816_2231/`

### 阶段 C 新采集帧（T0，2026-08-18 18:55 起，相机直采 5 帧 + 推理帧 5 帧）

副本：`docs/real_grasp_error_investigation/frame_T0_20260818/frame_1..10/`

| frame | 深度非零 | 深度 median | color sha256 | depth sha256 | meta sha256 |
|---|---:|---:|---|---|---|
| T0-1 | 303633 | 778 | 057b1ea0444e | 1a86ee5f0b46 | 774aa6019b05 |
| T0-2 | 303351 | 778 | 673cb11555a6 | 1d116664708d | f4cda8860598 |
| T0-3 | 303264 | 778 | f25813874fd7 | 0331c03c5a97 | a15f537c8b42 |
| T0-4 | 303451 | 779 | eae6e1b0f120 | 18fe044aaef9 | 1cb9e477d3cd |
| T0-5 | 303393 | 778 | 333fd6907937 | f668b61fa8a0 | ba4f8ce2f633 |
| T0-6（推理帧） | 57 行候选 | | db0da139fb（CSV） | | |
| T0-7 | 53 行 | | 7804aabc06 | | |
| T0-8 | 46 行 | | dfd00d05a6 | | |
| T0-9 | 47 行 | | 453fc6e3e2 | | |
| T0-10 | 48 行 | | 3bc93a28df | | |

每帧补充（8月16 固定帧 / 2026-08-18 新帧均相同，CameraInfo 稳定）：

```text
color: width=640, height=480, dtype=uint8(bgr)
depth: width=640, height=480, dtype=uint16, nonzero≈303k/304222, min=163, max=696, median=648(旧帧)/778(新帧)
intrinsic_matrix=[[604.21411133,0,321.78115845],[0,604.11468506,253.11112976],[0,0,1]]
factor_depth=1000.0
handeye R/t 摘要（handeye_eye_to_hand.yaml sha256=bb8594191b92b31d5689b49aa996956c6a0de4d1fdedf72d4cb9b3dd1e198da4）
  t_cam2base=[471.10, 1.71, 785.19] mm
  R 前向 = [0.992,-0.087,0.089 / -0.094,-0.992,0.082 / 0.081,-0.090,-0.993]
det(R)=1.0000000000, orthogonality_error=1.17e-15, T_cam2base 冗余矩阵与 R/t 一致
```

注：`camera_data/calibration.yaml`（cx=340.83, cy=238.67, fx=604.14, fy=601.12）与实时 CameraInfo（cx=321.78, cy=253.11）**不一致（Δcx≈19px）**，但 rg 全仓检索（src/scripts/test/graspnet-baseline）确认 calibration.yaml **无任何代码引用**——当前链路（bridge 写 meta.mat ← 实时 CameraInfo；推理 read_data_from_shared 读 meta.mat）只用实时内参，calibration.yaml 已排除。

## 候选和变换记录

### 阶段 B：固定帧（8月16）复算

| 行 | score | width | xyz_camera XYZ | center_camera XYZ | 反投影深度(期望) |
|---|---|---:|---:|---|---|
| 2 | 0.6226 | 94.3mm | [28.5, 12.0, 610.0] | [31.8, -14.6, 643.6] | xyz像素(350,265)深度=610(610)✓；center像素(352,239)深度=621(644) |
| 3 | 0.4299 | 88.9mm | [29.5, 2.9, 611.0] | 同上 | 像素(351,256)深度=612(611)✓ |
| 4 | 0.4238 | 99.9mm | [10.1, 14.7, 598.0] | 同上 | 像素(332,268)深度=600(598)✓ |
| 82 | 0.0125 | 97.1mm | [104.6, -115.1, 637.0] | 同上 | 像素(421,144)深度=638(637)✓ |

- 深度→点云独立反投影（center 像素 5x5 邻域，25 点有效）：邻域中心 [31.04, -14.51, 620.7] vs CSV center [31.78, -14.58, 643.6]，**差值 [-0.7, +0.1, -22.9]mm 范数 22.9mm（仅 Z）**
- 判定：**深度尺度与内参自洽（≤2mm）**；`xyz_camera`（执行用）与深度一致；`center_camera` 为点云均值语义（比表面深 22.6mm，即"感知中心偏高 2cm"现象来源，与 `grasp_z_offset=0.02` 补偿量吻合）。CSV 24 列与 `parse_grasp_csv` 对齐无误。
- 手眼独立计算 p_base（固定帧行 1）：xyz_camera=[28.5,12.0,610.0] → p_base=[552.3, 37.1, 180.9]mm；center → [560.9, 66.0, 150.2]mm。

### 阶段 C：scan_only 预检（2026-08-18 18:59，触发时帧 64 候选）

- 结果：`candidates=64 tested=19, gate_rejected=45, approach_pass=5, grasp_pass=5, lift_pass=0 -> 候选 [14] 全过（scan_only 臂不动）`，precheck_results.json approved=5（idx 14/28/35/36/48）selected=14
- gate_rejected=45：self_consistency（center 横向距 ≤0.08m）过滤（观测项）

选中候选 [14]（precheck_results.json approved[0]，score=0.0639, width=0.0695m）：

```text
xyz_camera=[0.072625183, -0.009949016, 0.741000]m  反投影像素(381,245)深度=743mm(期望741)✓
rotation_camera=[0.0228,-0.8852,0.4647 / 0.4925,-0.3945,-0.7757 / 0.8700,0.2465,0.4270]
p_base=独立复算=[609.65, 65.50, 56.36]mm（R_cam2base@p_cam+t_cam2base）
R_base_graspnet det=1.0000；R_base_tcp（右乘 GRASPNET_TO_PIPER_TCP_R）det=1.0000
grasp 位姿（TCP，z 下移 grasp_z_offset=0.02）: pos=[0.6097, 0.0655, 0.0364]m quat(wxyz)=[0.2648, 0.1370, 0.7998, 0.5211]
approach 位姿: pos=[0.6097, 0.0655, 0.0864]m 同姿态
link6 目标（TCP - 0.14Z）: pos=[0.6097, 0.0655, -0.1036]m  ← z 为负（低于 base_link 平面）观测项
link6 前向 z=-0.317，开口朝下分量=0.761（方向偏好选中最朝下者）
```

## MoveIt、反馈和 FK

### 阶段 C：零位预检环境（无真机，rsp+jsp+move_group）

| test_id | group | eef_link | frame_id | plan | execute | planning_time | velocity_scale | 规划终点关节 |
|---|---|---|---|---|---|---|---|---|
| C1 scan_only | arm | link6 | base_link | 5/19 候选 approach+grasp 全过 | 未执行(scan_only) | 0.5s/段(预检) | 0.1 | 未落盘（只规划） |

```text
/joint_states（jsp 发布，零位）：joint1..joint8 全 0 —— 与 ZERO_JOINTS 一致
/joint_states_feedback：无（真机驱动未启动）
/compute_fk link6/link7：服务存在（move_group 提供），真机姿态下未调用
服务/action：/move_action、/execute_trajectory、/compute_fk 均可用
TF：base_link←camera_link 静态 TF 已发布（Translation=[0.471,0.002,0.785]，与 handeye t 一致）
异常/超时/traceback：无
```

## 实测误差

### 阶段 D：无目标低速到位（2026-08-18 19:40–20:10，真机，velocity 0.1，MoveIt 链）

目标定义：zero=[0,0,0,0,0,0]；ready=[0,1.4919,0,0,-1.22,0]；grasp_like=TCP 位姿（pos=[0.55,0.10,0.18]m，quat wxyz=[0.2648,0.1370,0.7998,0.5211]——选中候选[14] 姿态，安全高度）。15 次运动全部成功（exec=True），详见 log/investigation_20260818_stageD.log 与 /tmp/stageD_report.json。

| 运动 | 次数 | 真机 vs 规划终点 max 关节差 (rad) | FK link7 (mm) |
|---|---:|---|---|
| zero 反向 | 3 | 0.0016 / 0.0013 / 0.0011 | [192,±1,225] |
| ready 正向 | 3 | 0.0030 / 0.0029 / 0.0018 | [305,±2,124] |
| ready 反向 | 3 | 0.0012 / 0.0015 / 0.0045 | [303,±2,124] |
| grasp_like 正向 | 3 | 0.0015 / 0.0008 / 0.0012 | [540,95-104,177-186] |
| grasp_like 反向 | 3 | 0.0013 / 0.0017 / 0.0009 | [540-547,93-99,179-186] |

grasp_like 命令预测（TCP=[550,100,180]mm，姿态 Z 轴=[0.566,0.761,-0.317]，link6=TCP-0.14·Z，link7 铰点=TCP-0.0042·Z）：

| 方向 | FK link6 误差 XYZ (mm) | norm | FK link7 误差 XYZ (mm) | norm |
|---|---|---|---|---|
| 正1 | [-6.8,-0.1,1.1] | 6.9 | [-7.9,-1.7,-4.2] | 9.1 |
| 反1 | [-8.7,-0.6,1.1] | 8.8 | [-6.8,-3.5,-2.0] | 7.9 |
| 正2 | [8.6,7.7,5.9] | 13.0 | [7.8,6.0,0.7] | 9.9 |
| 反2 | [-5.1,3.6,2.7] | 6.8 | [-0.6,-1.1,-0.1] | 1.3 |
| 正3 | [-6.5,6.3,-1.1] | 9.1 | [-7.5,7.5,0.0] | 10.6 |
| 反3 | [-3.4,1.6,9.5] | 10.2 | [-7.7,2.6,4.7] | 9.4 |

统计：FK vs 命令预测 norm 平均 8.6mm，最大 13.0mm；关节空间误差 max 0.0045 rad（≈0.26°）。
判定：**无系统性正反向回差**；FK 偏差主因 = MoveIt 位姿约束（PositionConstraint ±1cm 盒 + JointConstraint ±0.01rad，moveit_adapter.py L191-225）——"命令 vs 实际 FK"的 1cm 级偏差由规划容差产生，执行层自身 ≤2-3mm。

### 阶段 E（固定目标单次验证，2026-08-18 19:13，候选[44]，csv=/tmp/stageE_frozen.csv sha256=a80a1996bcf4367b7dbcb2b6d28f7a658279ca031acfe0eb71a922e15ad8ec7d）

scan_only 确认：candidates=62 tested=41 gate_rejected=21 approach_pass=3 grasp_pass=3 → 候选[44]（score=0.1286, width=63.3mm）。
执行：dry_run_max_stage:=grasp（approach→grasp 到位即停，不闭合夹爪），status=dry_stop，velocity 0.1。

| test_id | frame_id | candidate | command TCP XYZ (mm) | FK link7 XYZ (mm) | 实测指尖/夹持中心 | 真实目标中心 | error XYZ | norm |
|---|---|---:|---|---|---|---|---|---:|
| E1 | stageE_frozen.csv | [44] | [585.4, 82.5, 35.6] | [569.3, 79.4, 35.7] | — | 感知中心 p_base=[585.4, 82.5, 55.6]（未补偿） | FK-link7 vs TCP 预测: [-13.7, +0.1, -0.6] | 13.7 |
| E2 | 同帧 | [44] | 同上 | 同上 | 用户现场量测：指尖/夹爪 vs 目标物中心偏差 ≈ **20 mm**（方向未细分） | — | — | ≈20 |

补充记录（E1 执行期间）：
```text
approach 段：笛卡尔直线（velocity 0.1）；grasp 段：OMPL plan+execute；dry_stop 后 executor 收尾回 zero
fk_link7_position.json（执行后回 zero 前落盘）: {"pos": [0.5693291860491531, 0.07943023241895489, 0.035748847395873806]}
grasp_result.json: {"success": true, "status": "dry_stop", "stage": "grasp", "candidate_index": 44}
再次执行停在 grasp 位供现场量测（stageE_hold.py）：link6 目标=[0.4841, -0.0111, 0.0595]m，执行成功，实测约 20mm
```

误差分解（E_total≈20mm）：MoveIt 规划容差贡献 FK-vs-命令 13.7mm（X 向），执行层 ≤3mm（阶段 D），剩余 ~5-10mm 未归属（感知绝对位置 或 TCP 指尖几何 或 量测精度）。

## 命令日志

| test_id | 开始/结束时间 | 命令 | 退出码 | 是否移动机械臂 | 是否写文件 | 关键输出/错误 |
|---|---|---|---:|---|---|---|
| A0 | 18:51 | 环境探测（lsusb/ps/docker/date） | 0 | 否 | 否 | D435i Bus002 speed=5000；can0 DOWN；docker server 29.1.3 |
| A1 | 18:52 | sha256sum + cat handeye/calib yaml | 0 | 否 | 否 | handeye det=1.0 正交；calib 与 CameraInfo 内参矛盾（未使用） |
| B1 | 18:53 | python3 离线复算（/tmp/stageB_recompute.py） | 0(ROI段脚本小错) | 否 | 否 | xyz 与深度自洽；center Z 差 22.9mm；见上文 |
| C0 | 18:55 | run_d435i_bridge.sh（realsense+bridge+handeye） | 运行中 | 否 | 覆盖 shared_camera_data（旧帧已备份） | color 15.5Hz depth 14.5Hz CameraInfo 16.5Hz；K 与旧帧一致 |
| C1 | 18:56 | run_graspnet_d435i.sh headless（推理 docker） | 运行中 | 否 | 每帧覆盖 CSV | 40-56 grasp/帧；avg inf 0.37s |
| C2 | 18:56 | T0 五帧采集+推理帧五帧采集 | 0 | 否 | 写 frame_T0_20260818/ | 帧间候选不稳定（见 T0 结论） |
| C3 | 18:58 | rsp + joint_state_publisher + move_group.launch.py | 运行中 | 否 | 日志 log/investigation_20260818_*.log | /compute_fk /move_action /execute_trajectory 就绪 |
| C4 | 19:00 | grasp_execute.launch.py scan_only:=true + /grasp_execute | 0 | **否（scan_only）** | precheck_results.json | 候选[14]全过；tested=19 gate_rejected=45 |
| D0 | 19:00 | sudo ip link set can0 up type can bitrate 1000000 | 0 | 否 | 否 | can0 UP ERROR-ACTIVE |
| D1 | 19:03 | start_single_piper.launch.py（driver） | 0 | 使能机械臂 | 否 | Enable=True；feedback 185Hz |
| D2 | 19:05 | sdk_go_zero.py（driver 运行时） | 0 | 无效（SDK 被 driver 覆盖） | 否 | 机械臂未动，FK 误判 done |
| D3 | 19:10 | 停 driver → sdk_go_zero.py | 0 | **是（低速回零）** | 否 | FK rpy [158.88,77.34,158.94]→[0,85,0] 回零成功 |
| D4 | 19:20 | 重启 driver + demo.launch.py（ros2_control+mock+moves） | 0 | 否 | 否 | 三控制器 activated；/joint_states mock 全0 |
| D5 | 19:40–20:10 | /tmp/stageD_test.py（15 次到位，velocity 0.1） | 0 | **是（低速）** | /tmp/stageD_report.json | 全部成功；关节差 max 0.0045rad；FK 偏差 7-13mm（MoveIt ±1cm 容差） |
| E1 | 19:13 | grasp_execute.launch.py scan_only:=false dry_run_max_stage:=grasp csv_path:=/tmp/stageE_frozen.csv | 0 | **是（低速 approach→grasp）** | grasp_result.json, fk_link7_position.json, log | dry_stop@'grasp'；FK link7=[569.3,79.4,35.7]mm vs 命令 TCP 偏差 13.7mm（X 向，MoveIt ±1cm 容差） |
| E2 | 19:15 | stageE_hold.py（停在 grasp 位） | 0 | **是（低速）** | log | 用户现场量测：指尖 vs 目标物中心 ≈20mm |

## 阶段结论

### T0（建立可重复基准）

- 原始观测：5 帧相机直采 + 5 帧推理帧。**帧间候选不稳定**：候选数 46–57 波动；按 (name,rotation) 匹配的前 5 候选仅 1 个在 5 帧全出现，其 xyz_camera 范围 [46.9, 46.6, 38.0]mm。
- 是否继续：按 T0 判定标准（感知坐标自身变化 >5mm → 优先进入 T1），**感知层帧间稳定性未达标**，应优先排查感知/深度层；同时继续 T2 以降各层（分层验证不受影响，因 xyz 与深度自洽）。
- 缺失数据：多帧同候选坐标变化需要更多样本（≥5 帧已取，但候选匹配率低）；无人工标定物可作真值。

### T1（感知目标与深度尺度）

- 原始观测：factor_depth 语义正确（bridge 存 uint16 mm；推理 depth/factor_depth→m，无重复缩放，源码行：bridge `_write_depth` L103-113、inference `read_data_from_shared` L235-246 与 `process_data` L512）；depth 为 aligned-to-color（话题 `aligned_depth_to_color`）；xyz_camera 反投影与深度 ≤2mm 一致；center_camera 偏深 22.6mm（点云均值语义）；ROI（x=[0.3,0.8] y=[0.25,0.75]）包含目标，未截断；内参仅实时 CameraInfo（calibration.yaml 无引用）。
- 是否继续：继续 T2+。深度尺度/内参假设**未支持**（无证据），center 语义问题有 `grasp_z_offset=0.02` 已在执行侧补偿（补偿量 20mm vs 22.6mm 偏差近似）。
- 缺失数据：无人工量取的目标中心可对比（感知绝对位置真值未采集——需治具）。

### T2（手眼平移/旋转分离）

- 原始观测：R_cam2base 精确正交（det=1.0000000000，正交误差 1.2e-15）；t=[471.1,1.7,785.2]mm；TF 静态发布与 yaml 一致；`camera_pose_to_base` 公式 p=R@x+t、R_base=R@R_grasp 与独立复算一致（det 均 1.0）；无重复 camera_link 父节点（TF 树单边）。
- 是否继续：继续。手眼矩阵自身健康；需 3 标定点真值才能分离平移/旋转误差（缺标定物）。
- 缺失数据：≥3 个相机内已知位置的标定点与实测 base 坐标。

### T3（TCP/夹爪几何和 link6/link7 语义）

- 原始观测：URDF `joint6_to_gripper_base` 原点 0 0 0（gripper_base≡link6）；`joint7/joint8` 滑移原点 z=+0.1358m、rpy X 旋转 90°；代码 `TCP_OFFSET_FROM_LINK6_Z=0.14`（grasp_precheck.py L54，注释自认 0.14 是 0.136 的仿真近似）；`_tcp_to_link6_matrix` 右乘 T(0,0,-0.14)；MoveIt 位姿目标端点为 link6（arm 组 tip），规划/执行先 TCP→link6 折算。0.14 与铰点 0.1358 差 4.2mm（需现场量测确认指尖平面实际位置）。
- 是否继续：继续 T4+。几何折算自洽（0.14 双侧一致：预检与执行都用同一折算）；**指尖夹持中心到 link6 基准面的实测距离未采集（需真机量测）**。
- 缺失数据：link6 基准面到指尖夹持中心实测距离；左右手指对称性量测。

### T4（MoveIt 目标与实际 FK）

- 原始观测：预检/执行目标端点为 link6（pose target frame_id=base_link，PositionConstraint ±1cm 盒、Orientation ±0.05rad）；scan_only 预检 5/19 全过（KDL 数值 IK，analytic 已关）；选中候选 link6 目标 z=-0.1036m（低于 base 平面，物理可行性存疑——零位环境无桌面碰撞体）；`/compute_fk`、`/joint_states` 可用。
- 是否继续：继续 T5（真机 FK 对照未采集）。**MoveIt 约束/接口与代码一致**；"link6 目标 z 为负"是否被碰撞场景拦截需真机验证（当前零位环境无桌面模型）。
- 缺失数据：真机 /joint_states、/joint_states_feedback、/compute_fk 与规划终点对照。

### T5（机械臂执行和机械误差）

- 原始观测（2026-08-18 19:40–20:10 完成）：真机经 MoveIt 链（velocity 0.1）执行 15 次到位（zero×3 + ready 正反×3 + grasp_like 正反×3），全部成功。真机反馈 vs 规划终点关节差 max=0.0045 rad（0.26°，ready 反向第3次离群），多数 ≤0.003 rad；无系统性正反向回差。FK（真机反馈关节）vs 命令预测 link7 norm 1.3–10.6mm（平均 8.6mm）——主因 MoveIt 位姿约束 ±1cm 盒（moveit_adapter.py L205-215 PositionConstraint 0.02m 盒 / L217-223 姿态容差 0.05rad）。
- 是否继续：**执行层（驱动→CAN→机械）误差 ≤2-3mm，排除"机械执行/回差/零位"作为 2-3cm 误差来源**（无目标状态）。剩余误差在感知/变换/TCP 几何层。
- 缺失数据：抓取执行状态（有负载）下到位精度（阶段 E）。

### T6（根因归类）

- 原始观测：见 root-cause-report.md。**已排除**：深度尺度/内参（≤2mm）、手眼矩阵数值（正交 1e-15）、变换实现（复算一致）、执行层（≤3mm 无回差）、calibration.yaml（无引用）。**确定存在**：MoveIt 位姿约束 ±1cm 盒 → FK vs 命令 7–13.7mm（E1 X 向 -13.7mm）。**未归属**：~5–10mm（感知绝对位置 或 TCP 指尖几何），E_total 实测 ≈20mm。
- 最高置信修复：收紧 MoveIt 位姿约束（moveit_adapter.py PositionConstraint 盒 0.02→0.005m 等），回归测试 = 阶段 D 到位测试（FK vs 命令 <5mm 通过）。
- 缺失数据：标定物真值（感知绝对位置验证）、link6→指尖夹持中心实测、误差方向 XYZ 分量。

### 驱动链路事实（T5 期间记录）

- 真机执行链：move_group(/move_action plan_only) → /execute_trajectory → arm_controller(FakeSystem mock, ros2_control) → /joint_states（mock 状态=命令） → 驱动订阅(remap joint_ctrl_single→/joint_states) → piper_sdk JointCtrl → CAN（参考 moveit_go_ready.py 注释与 piper_ctrl_single_node.py L46-56、L79-80）。
- driver 200Hz 周期下发 ArmCtrl/GripperCtrl → **SDK 直控在 driver 运行时被覆盖**（实测：driver 运行时 sdk_go_zero.py 无效，机械臂不动；停 driver 后 SDK 回零成功）。

## 待办（阶段 D/E 前置）

1. 用户启用 can0 并确认 Piper 驱动可启动（机械臂会动作，需在场）。
2. 阶段 D：无目标低速到位测试（zero/中间/抓取附近 3 目标 × 正反向 3 次）。
3. 阶段 E：固定目标单次验证（同 frame/候选，scan_only 确认后执行）。
4. 阶段 F：填根因表，生成 root-cause-report.md。
