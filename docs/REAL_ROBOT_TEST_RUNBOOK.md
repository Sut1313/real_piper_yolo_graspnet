# real_piper_yolo_graspnet 真机测试操作流程

> 适用仓库：`/home/liyidong/agrobot_code/real_piper_yolo_graspnet`
> 标定工具：`/home/liyidong/agrobot_code/piper_single_calibration`
> 目标：先用 `piper_single_calibration` 重做内外参和手眼，再按“只感知 → 只预检 →
> 分段空跑 → 首次实抓”的顺序验证 YOLO 前端 + GraspNet + Piper 真机执行链。

## 0. 总览与安全红线

阶段顺序不能跳：

```text
准备 → 内参标定 → 手眼标定 → 标定验证 → 构建/启动 → 只感知
     → scan_only 预检 → approach 空跑 → grasp 空跑 → 空中闭合
     → 首次实抓 → 重复试验 → 收尾记录
```

安全红线：

1. 手始终放在急停上；第一次实抓不要无人值守。
2. 每轮只触发一次 `/grasp_execute`，不要写自动循环。
3. 首次实抓 `velocity_scale:=0.05`，稳定后再升到 0.1。
4. 标定期间不要同时运行 ROS 的 Piper 驱动和 RealSense 节点；它们会和标定脚本抢
   `can0` 和相机。
5. 同一时间只能有一个夹爪命令源：驱动必须 `gripper_forward:=false`，夹爪由
   `grasp_execute.py` 通过 SDK 直控。
6. 机械臂上电前确认工作空间清空，零位附近没有线缆、标定板支架和人员。

---

## 1. 准备

### 1.1 硬件检查

- D435i 用 USB3 线连接，`lsusb` 能看到 Intel RealSense；
- USB-CAN 适配器接好，CAN_H/CAN_L 正确，总线两端 120 Ω 终端电阻；
- 机械臂电源、急停回路正常；
- ChArUco 标定板按 100% 比例打印，刚性固定在末端（眼在手外）；
- 尺子/卷尺，用于核对相机安装位置。

### 1.2 软件与构建

```bash
cd /home/liyidong/agrobot_code/real_piper_yolo_graspnet

# 1) 构建感知和 YOLO 包
./build_ws.sh

# 2) 构建 Piper 驱动和 MoveIt（build_ws.sh 不包含它们）
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to piper piper_with_gripper_moveit
source install/setup.bash
```

确认：

```bash
ros2 pkg list | grep -E "^(piper|piper_msgs|piper_description|piper_with_gripper_moveit|real_piper_perception|real_piper_yolo_graspnet)$"
python3 -c "import piper_sdk; print(piper_sdk.__version__)"
```

`piper_sdk` 缺失时：

```bash
pip3 install piper_sdk
```

CAN 工具缺失时：

```bash
sudo apt update
sudo apt install -y can-utils ethtool
```

YOLO 运行库在系统 Python 里可能没有，节点默认回退到：

```text
/home/liyidong/agrobot_code/tabletop_yolov8seg/.venv/lib/python3.10/site-packages
```

如果导入失败，启动时显式指定：

```bash
export YOLO_ONNXRUNTIME_SITE_PACKAGES=/home/liyidong/agrobot_code/tabletop_yolov8seg/.venv/lib/python3.10/site-packages
```

### 1.3 备份旧标定

```bash
cd /home/liyidong/agrobot_code/real_piper_yolo_graspnet/camera_data
cp calibration.yaml calibration.yaml.bak-$(date +%Y%m%d)
cp handeye_eye_to_hand.yaml handeye_eye_to_hand.yaml.bak-$(date +%Y%m%d)
```

### 1.4 启动前检查

```bash
# 没有其他进程占用 CAN
pgrep -af "piper_single_ctrl|grasp_executor|realsense2_camera" || true

# CAN 接口
ip -br link show type can

# 相机
lsusb | grep -i "8086"
```

---

## 2. 手眼标定（piper_single_calibration）

### 2.1 标定环境

标定脚本直接用 `piper_sdk` 和 `pyrealsense2`，不要和 ROS 驱动同时跑。

```bash
cd /home/liyidong/agrobot_code/piper_single_calibration
python3 -m venv venv
venv/bin/pip install opencv-python pyrealsense2 numpy pyyaml matplotlib scipy piper_sdk
```

说明：工具已适配 OpenCV 5。它用 numpy 自己解 `AX=XB`，不依赖已被移除的
`cv2.calibrateHandEye`，不要按旧教程降 OpenCV 版本。

### 2.2 打印标定板

使用仓库内的：

```text
/home/liyidong/agrobot_code/piper_single_calibration/
calib.io_charuco_200x150_9x12_13_9_DICT_5X5.pdf
```

- 必须 100% 比例打印，不能“适合页面”；
- 打印后量一下：方格 13 mm、标定码 9 mm；
- 板子固定在末端时要刚性，不能有软连接或晃动；
- 板面朝向相机，避免反光和阴影。

### 2.3 相机内参

先拉起 CAN：

```bash
cd /home/liyidong/agrobot_code/real_piper_yolo_graspnet/src/piper_ros
bash can_activate.sh can0 1000000
ip -details link show can0
```

再采集内参：

```bash
cd /home/liyidong/agrobot_code/piper_single_calibration
venv/bin/python intrinsics/main_capture.py
```

要点：

- 分辨率保持 **640×480**，和 `d435i_yolo_graspnet_pipeline.launch.py` 一致；
- 画面显示 `GOOD` 时按空格，采 **10–20 张**；
- 覆盖画面中心、四角、不同距离和倾角；
- 按 `Q` 退出后自动标定；
- 输出：`data/result/calibration.yaml`；
- 检查重投影误差，建议 < 1 px；偏大就补采四角样本。

如果已经有一批图片，可以只标定：

```bash
venv/bin/python intrinsics/main_capture.py --calibrate-only
```

### 2.4 手眼标定（eye-to-hand）

相机固定，标定板装在末端：

```bash
cd /home/liyidong/agrobot_code/piper_single_calibration
venv/bin/python hand_eye_single_eye_to_hand.py
```

操作：

1. 程序自动连接 RealSense 和机械臂，并 `EnablePiper()`；
2. 板子离相机约 **30 cm**（开发日志实测 30 cm 检出率 100%，1 m 时几乎检不到）；
3. 每摆好一个姿态，画面显示 `POSE OK` 时按 `S` 采样；
4. 采 **15–20 组**，姿态要有明显旋转差异，不能只平移或绕同一轴转；
5. 避开实测有问题的 RZ≈93° 附近；
6. 按 `C` 解算并保存；
7. 输出：

```text
data/result/handeye_eye_to_hand.yaml
```

文件内容应包含：

```yaml
description: Eye-to-Hand calibration result: T_cam2base
R_cam2base: 3x3
t_cam2base: 3
T_cam2base: 4x4
```

### 2.5 标定验证

先看可视化：

```bash
venv/bin/python test_calib.py
```

再检查数值：

- `R_cam2base` 的 `det` 必须接近 +1；
- `t_cam2base` 单位是米，量级应和相机实际安装位置一致；
- 如果相机没动过，新结果应和旧值 `(0.438, -0.022, 0.575)` 接近；
- 相机坐标系的 z 轴在基座系下应大致朝下（对着桌面）；
- `T_cam2base` 与 `R/t` 必须一致，`handeye_tf_publisher` 会做这项校验。

工具当前只解相邻位姿对，且**不输出残差统计**。不要只看它打印的矩阵，建议做一次
独立“触点验证”：

1. 在桌面上放一个小标记，位置在机械臂可达范围内；
2. 用 YOLO/GraspNet 或直接读点云，得到标记在相机系的位置；
3. 用新 `handeye_eye_to_hand.yaml` 转到基座系；
4. 用 MoveIt 或 SDK 把 TCP 慢慢移到该点上方 5 cm，张开夹爪；
5. 目测/尺量 TCP 与标记的偏差，目标 < 5 mm；偏大就重新采样标定。

注意：`scripts/fk_vs_camera_compare.py` 读的是 `/joint_states`（FakeSystem 命令状态），
不是独立真机反馈；严格验证请用 `/joint_states_feedback` 或 SDK 的 `GetArmEndPoseMsgs()`。

### 2.6 集成到项目

```bash
cd /home/liyidong/agrobot_code/piper_single_calibration

cp data/result/calibration.yaml \
   ../real_piper_yolo_graspnet/camera_data/calibration.yaml
cp data/result/handeye_eye_to_hand.yaml \
   ../real_piper_yolo_graspnet/camera_data/handeye_eye_to_hand.yaml
```

启动执行器后必须检查日志：不能出现 `handeye 标定文件加载失败`。`grasp_transform.load_handeye`
在文件缺失时会回退到仿真常量 `(0.072, 0.021, 0.548)`，这是静默降级，结果不可用；
`handeye_tf_publisher` 则是 fail-loud，不发布错误 TF。

帧名必须改对：

- `piper_single_calibration` 输出的是 **彩色光学系** `camera_color_optical_frame → base_link`；
- `grasp_transform.py` / `grasp_precheck.py` 直接把它作用在 CSV 相机系候选上，执行链不用改；
- `handeye_tf_publisher` 默认 `child_frame:=camera_link` 是错的，会差一个光学系到
  `camera_link` 的 90° 旋转。

在 `handeye_eye_to_hand.yaml` 里加：

```yaml
parent_frame: base_link
child_frame: camera_color_optical_frame
```

或者启动时覆盖（文件里没有 `child_frame` 键时才会用这个参数）：

```bash
ros2 launch real_piper_yolo_graspnet d435i_yolo_graspnet_pipeline.launch.py \
  handeye_yaml:=$(pwd)/camera_data/handeye_eye_to_hand.yaml \
  child_frame:=camera_color_optical_frame
```

`run_d435i_yolo_bridge.sh` 没有暴露 `child_frame`，所以最稳妥的是直接在 yaml 里写
`parent_frame` / `child_frame`，让 `handeye_tf_publisher` 从文件读取。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### 2.7 标定常见问题

| 现象 | 优先检查 |
|---|---|
| 板子检测不到 | 打印比例、板距（约 30 cm）、光照/反光、DICT_5X5 |
| `POSE OK` 但结果很离谱 | 姿态多样性不足、相邻样本旋转太小、板子松动 |
| 平移量级不对 | 单位（米/毫米）、`T_cam2base` 方向、是否用错坐标系 |
| `det != +1` | 旋转矩阵损坏或文件被手改 |
| 抓取位置稳定偏 2 cm | 先用 `grasp_z_offset:=0.0`，再重新测，不要叠加旧补偿 |

---

## 3. 启动 ROS 栈

以下终端都先执行：

```bash
cd /home/liyidong/agrobot_code/real_piper_yolo_graspnet
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 3.1 终端 A：Piper 驱动

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 auto_enable:=true \
  gripper_exist:=true gripper_forward:=false
```

检查：

```bash
ros2 topic hz /joint_states_feedback
ros2 topic echo /arm_status --once
```

要求：

- `/joint_states_feedback` 持续有数据；
- `arm_status` 的 `err_code` 为 0；
- 机械臂已使能。

说明：`gripper_forward:=false` 必须保留，否则 MoveIt 的 joint7 命令会和
`grasp_execute.py` 的 SDK 夹爪命令互相覆盖。当前夹爪走 SDK 直控，
`gripper_val_mutiple` 不影响这条路径；只有改用驱动转发时才需要设为 2。

### 3.2 终端 B：MoveIt

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py use_rviz:=true
```

为什么用 `demo.launch.py`：它会启动 FakeSystem 的 `ros2_control_node` 和控制器，
从而发布 `/joint_states`；单独的 `move_group.launch.py` 不会起控制器，可能导致
`/joint_states` 没有发布者。

检查：

```bash
ros2 service list | grep -E "move_action|compute_ik|compute_fk|compute_cartesian_path"
ros2 topic hz /joint_states
```

注意：

- `/joint_states` 是 FakeSystem 的命令状态，不是真实反馈；
- 真实反馈在 `/joint_states_feedback`；
- MoveIt 规划成功不等于机械臂实际到位。

**只做预检时**：不要启动终端 A 的 Piper 驱动，避免 FakeSystem 的状态被驱动转发成
真实命令。

### 3.3 终端 C：D435i + YOLO + 手眼 TF

```bash
export TARGET_CLASS=cup
export TABLETOP_YOLO_MODEL=/home/liyidong/agrobot_code/tabletop_yolov8seg/models/tabletop_yolov8s_seg_v2.onnx
./run_d435i_yolo_bridge.sh
```

如果 ONNX Runtime 导入失败：

```bash
export YOLO_ONNXRUNTIME_SITE_PACKAGES=/home/liyidong/agrobot_code/tabletop_yolov8seg/.venv/lib/python3.10/site-packages
./run_d435i_yolo_bridge.sh
```

检查：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ls -lh shared_camera_data/
```

首次先用 v2 模型；v5 虽然更新，但先不要同时换模型和标定。确认类别顺序与
`TABLETOP_CLASS_NAMES` 一致后再做 A/B。

第一次测试只放**一个**目标，且物体宽度 + 5 mm 要小于 70 mm。`fail_on_multiple: true`
时多个同类目标会拒绝出帧。

### 3.4 终端 D：GraspNet Docker

```bash
./run_graspnet_yolo.sh headless
```

检查：

```bash
tail -5 shared_camera_data/perception_grasps.csv
```

`perception_grasps.csv` 应在每次新帧后更新；如果没有，先检查 `.ready` 是否被消费、
Docker 是否正常退出、`frame_manifest.json` 的 `source` 是否为
`d435i_yolo_graspnet_bridge`。

### 3.5 终端 E：执行器

预检模式（默认，不动臂）：

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=true \
  handeye_yaml:=$(pwd)/camera_data/handeye_eye_to_hand.yaml \
  shared_dir:=$(pwd)/shared_camera_data \
  graspnet_6d_max_candidates:=30
```

触发一次：

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

检查：

```bash
cat shared_camera_data/precheck_results.json
cat shared_camera_data/grasp_result.json
```

`scan_only` 返回成功只表示流程完成，不表示一定找到了可执行候选。

---

## 4. 只感知验证（不开执行器）

目标：确认 YOLO 掩膜、点云和 GraspNet 候选在相机系里是对的。

1. 启动终端 C（YOLO）和终端 D（Docker）；
2. 打开 `shared_camera_data/yolo_overlay.png` 或 RViz 看掩膜；
3. 检查 `instances.npz`、`detections.json`、`frame_manifest.json`；
4. 用 `view_results.sh` 或 `scripts/result_viewer.py` 看候选；
5. 用 `scripts/perception_stability_test.py --frames 10 --interval 1` 看同一物体的
   候选位置波动；标准差应在毫米级，不能跳几十厘米；
6. 确认候选的 `camera_z` 在 0.15–1.20 m 内，且落在物体上。

---

## 5. 预检 scan_only

目标：确认手眼变换、候选筛选和 MoveIt 三段预检能跑通，臂不动。

1. 只启动终端 B（MoveIt，FakeSystem），**不启动** Piper 驱动；
2. 启动终端 C、D；
3. 启动终端 E，`scan_only:=true`；
4. 触发 `/grasp_execute`；
5. 看 `precheck_results.json` 的 `approved` 和 `selected`；
6. 在 RViz 里确认候选位姿落在物体上方、夹爪朝向合理。

如果 `approved=0`：

- 先看 `perception_grasps.csv` 是否有候选；
- 再检查手眼矩阵和 `base_link` 坐标；
- 降低 `graspnet_6d_max_candidates` 不是解决办法，先确认至少一个候选在 RViz 里
  位置正确；
- 检查 `grasp_z_offset` 是否仍是旧的 0.02。

---

## 6. 分段空跑

目标：让机械臂动，但不抓物体。每一步都保持夹爪张开，手放急停。

> 当前版本（`grasp_execute.py`）在执行段**没有 lift/place**：闭合、判定、保持复核后
> 直接由 executor 收尾回 zero。所以本流程验证的是“夹住并保持”，不是完整搬运。

### 6.1 approach 空跑

终端 A 启动 Piper 驱动；终端 B 启动 MoveIt；终端 C/D 保持运行。

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=false \
  dry_run_max_stage:=approach \
  velocity_scale:=0.05 \
  grasp_z_offset:=0.0 \
  graspnet_6d_max_candidates:=30 \
  handeye_yaml:=$(pwd)/camera_data/handeye_eye_to_hand.yaml \
  shared_dir:=$(pwd)/shared_camera_data
```

触发：

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

检查：机械臂到达物体上方，夹爪张开，没有碰撞；日志里 `status=dry_stop`。

注意：执行器在运动前会做一次全开/全闭参考标定，夹爪会先完全闭合再张开；这一步也会
发生在上面的 approach 空跑里，手指不要放在夹爪行程内。

### 6.2 grasp 空跑

把 `dry_run_max_stage:=grasp` 再跑一次。机械臂到抓取位，夹爪仍张开。检查 TCP 是否
在物体中心上方，偏差目标 < 2 cm。

### 6.3 空中闭合

把 `dry_run_max_stage:=gripper` 跑一次，不放假物体。观察：

- 夹爪闭合；
- `grasp_result.json` 或日志里应判定为空载合拢（`closed_empty` / `grasp_empty`），
  不是 `success`；
- 随后张开夹爪、回零。

如果判定成 `success`，说明夹爪反馈或判定阈值有问题，先不要抓真物体。

### 6.4 关于 lift

当前 `grasp_execute.py` 的抬升段是关闭的，`dry_run_max_stage:=lift` 不会产生抬升动作，
和完整流程等价。`grasp_precheck.py` 也只预检 approach + grasp。要验证抬升/放置，必须
先在代码里重新启用 lift，并单独做 plan-only 和空跑；本 runbook 的首次实抓不包含这一段。

---

## 7. 首次实抓

条件：

- 标定验证通过；
- approach/grasp 空跑通过；
- 空中闭合判定为 `grasp_empty`；
- 工作空间清空，急停在手；
- 只放一个目标，物理宽度 + 5 mm < 70 mm。

启动终端 A/B/C/D，然后：

```bash
ros2 launch real_piper_perception grasp_execute.launch.py \
  scan_only:=false \
  velocity_scale:=0.05 \
  grasp_z_offset:=0.0 \
  graspnet_6d_max_candidates:=30 \
  handeye_yaml:=$(pwd)/camera_data/handeye_eye_to_hand.yaml \
  shared_dir:=$(pwd)/shared_camera_data
```

单次触发：

```bash
ros2 service call /grasp_execute std_srvs/srv/Trigger '{}'
```

当前版本应依次执行：

```text
回 zero → 读候选 → 预检 → 张开 → approach → grasp
→ 完全闭合 → 稳定窗口判定 → 保持复核 → 回 zero
```

注意：当前执行段没有 lift/place；这次真机测试验证的是“夹住并保持”，不是完整搬运。

检查：

```bash
cat shared_camera_data/grasp_result.json
```

关注字段：

- `success`
- `status`
- `stage`
- `candidate_index`
- `cmd_width`
- `actual_width`

成功标准：

- `success=true`；
- 日志里 `detail=grasped`，`actual_width` 明显大于 `full_closed`、小于 `full_open`；
- 保持复核也通过（日志里第二次 verdict 仍是 `success`）；
- 机械臂回 zero，`arm_status.err_code=0`。

说明：闭合命令是**完全闭合**（`_close_gripper(0.0)`），`cmd_width` 只是候选估算值；
判定用的是 `actual_width` 与现场实测的 `full_open/full_closed`。所以不要用
`actual_width < cmd_width` 作为唯一标准。

失败时先看 `stage`，不要连续重试：

| stage / status | 含义 | 先查 |
|---|---|---|
| `precheck` | 无候选全过 | 手眼、候选位置、预检预算 |
| `approach` | 接近失败 | 碰撞、可达性、TCP 偏移 |
| `grasp` | 抓取位失败 | 物体高度、`grasp_z_offset` |
| `gripper` | 夹爪闭合/反馈失败 | 宽度 > 70 mm、CAN、SDK、反馈话题 |
| `gripper_calib` | 全开/全闭参考未稳定 | `/joint_states_feedback`、夹爪初始化 |
| `gripper_hold` | 保持复核失败 | 掉件、夹持力、物体太滑 |
| `status=grasp_empty` | 抓空 | 宽度、抓取深度、物体是否被碰走 |

稳定后可以逐步：

1. `velocity_scale` 0.05 → 0.1；
2. 换第二个物体；
3. 再做 v2/v5 模型 A/B；
4. 最后才考虑连续多轮（仍然每次单独触发，不要自动循环）。

---

## 8. 收尾

1. 先停执行器；
2. 停 Docker；
3. 停 YOLO/RealSense；
4. 停 MoveIt；
5. 停 Piper 驱动；
6. 可选：`sudo ip link set can0 down`；
7. 保存 `grasp_result.json`、`precheck_results.json`、日志和标定文件；
8. 记录本轮参数和结果，见第 10 节模板。

不要在不确认的情况下提交标定文件或代码；标定文件涉及工位，提交前先确认。

---

## 9. 故障排查

| 现象 | 排查顺序 |
|---|---|
| `No module named piper_sdk` | `pip3 install piper_sdk`，确认用的是 ROS 的 Python |
| `can0` 不存在 | `lsusb` 看 CAN 适配器、`can_activate.sh can0 1000000` |
| `Could not find package piper` | 补跑 `colcon build --packages-up-to piper piper_with_gripper_moveit` |
| `/joint_states_feedback` 无数据 | 驱动未启动、CAN 未使能、`arm_status` 报错 |
| MoveIt 没有 `/joint_states` | 用 `demo.launch.py`，不要只用 `move_group.launch.py` |
| YOLO 无检测 | `TARGET_CLASS`、置信度、深度范围、模型路径、ONNX Runtime |
| `instances.npz` 不更新 | `.ready` 未消费、Docker 未运行、YOLO 桥接报错 |
| `no candidate passed precheck` | 手眼、候选是否在物体上、`grasp_z_offset`、规划预算 |
| `status=grasp_empty` | 物体太宽/太矮、抓取深度、夹爪闭合位置 |
| `detail=not_closed` | 夹爪 SDK 路径、CAN、`gripper_forward` 是否 false |
| `detail=closed_empty` | 夹爪合拢但没夹到：宽度/深度/物体被碰走 |
| 抓取位置稳定偏 2 cm | 先设 `grasp_z_offset:=0.0`，再重测手眼 |
| TF 里相机姿态不对 | `child_frame` 是否误用 `camera_link`，应为 `camera_color_optical_frame` |
| Docker 看不到 GPU | `nvidia-smi`、NVIDIA Container Toolkit |

---

## 10. 记录模板

每轮测试至少记录：

```text
日期/时间：
相机序列号：
标定板打印比例：100%（方格 13 mm / 码 9 mm）
内参图片数 / 重投影误差：
手眼采样数：
R_cam2base / t_cam2base：
独立触点验证误差：
模型：v2 / v5
target_class：
物体与实测宽度：
velocity_scale / grasp_z_offset / max_candidates：
candidate_index / score / width：
cmd_width / actual_width：
status / stage / success：
失败现象与处理：
```

建议把每轮 `grasp_result.json` 复制到 `docs/test_runs/` 下按日期命名，便于回溯。
