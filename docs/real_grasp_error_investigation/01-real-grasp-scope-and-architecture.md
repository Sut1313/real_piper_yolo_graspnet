# 真机抓取相关文件清单与架构

## 1. 文件读取边界

后续真机误差排查只允许优先读取下列文件和目录。除非测试结果明确要求，不要扩大范围。

### A. 真机抓取主链路：必须读取

```text
src/real_piper_perception/real_piper_perception/d435i_graspnet_bridge.py
src/real_piper_perception/real_piper_perception/handeye_tf_publisher.py
src/real_piper_perception/real_piper_perception/grasp_candidates.py
src/real_piper_perception/real_piper_perception/grasp_transform.py
src/real_piper_perception/real_piper_perception/grasp_precheck.py
src/real_piper_perception/real_piper_perception/grasp_executor.py
src/real_piper_perception/real_piper_perception/grasp_execute.py
src/real_piper_perception/real_piper_perception/grasp_verify.py
src/real_piper_perception/real_piper_perception/moveit_adapter.py
src/real_piper_perception/real_piper_perception/grasp_ik.py
```

### B. 真机启动与参数：必须读取

```text
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
src/real_piper_perception/launch/grasp_execute.launch.py
src/real_piper_perception/setup.py
run_graspnet_d435i.sh
```

### C. 标定与共享数据：按测试需要读取

```text
camera_data/calibration.yaml
camera_data/handeye_eye_to_hand.yaml
shared_camera_data/color.png
shared_camera_data/depth.png
shared_camera_data/meta.mat
shared_camera_data/perception_grasps.csv
shared_camera_data/last_validation.json
shared_camera_data/roi.json
```

### D. Piper 真机驱动：按执行层测试需要读取

```text
src/piper_ros/src/piper/piper/piper_ctrl_single_node.py
src/piper_ros/src/piper/piper/piper_ctrl_single_node_new.py
src/piper_ros/src/piper/piper/piper_read_slave_joint.py
src/piper_ros/src/piper/launch/start_single_piper.launch.py
src/piper_ros/src/piper_msgs/msg/PosCmd.msg
src/piper_ros/src/piper_msgs/msg/PiperStatusMsg.msg
src/piper_ros/src/piper_msgs/srv/Enable.srv
```

### E. 机器人几何与 MoveIt：只读必要部分

```text
src/piper_ros/src/piper_description/urdf/piper_description.urdf
src/piper_ros/src/piper_description/urdf/piper_description.xacro
src/piper_ros/src/piper_description/meshes/link6.STL
src/piper_ros/src/piper_description/meshes/link7.STL
src/piper_ros/src/piper_description/meshes/link8.STL
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.srdf
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/kinematics.yaml
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/joint_limits.yaml
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/piper.ros2_control.xacro
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/ros2_controllers.yaml
```

### F. 诊断脚本：按测试任务读取

```text
scripts/fk_vs_camera_compare.py
scripts/roi_coordinate_check.py
scripts/perception_stability_test.py
scripts/tf_tree_viewer.py
scripts/moveit_go_ready.py
scripts/sdk_fk_vs_urdf_fk.py
scripts/sdk_mapping_validation.py
scripts/sdk_endpose_test.py
```

## 2. 明确排除的文件和目录

以下内容默认禁止读取，除非测试计划中的某一项明确点名：

```text
build/
install/
src/realsense-ros/
graspnet-baseline/graspnetAPI_local/
graspnet-baseline/demo.py
graspnet-baseline/demo_headless.py
graspnet-baseline/demo_save.py
graspnet-baseline/train.py
graspnet-baseline/test.py
src/piper_ros/src/piper_sim/
src/piper_ros/src/piper_description/meshes/（link6/link7/link8 以外）
docs/ 历史 session-summary 和 dev-log（除非用于核对已知事实）
所有 __pycache__/、*.pyc、编译产物和安装产物
```

## 3. 真机主架构

```text
D435i ROS 话题
  ├─ color image
  ├─ aligned depth image
  └─ CameraInfo
        ↓
d435i_graspnet_bridge
        ↓  color.png / depth.png / meta.mat / .ready
GraspNet Docker 推理
        ↓  perception_grasps.csv
grasp_executor
  ├─ parse_grasp_csv / rank_candidates
  ├─ handeye + GraspNet→Piper TCP 变换
  ├─ Prechecker：IK/FK/MoveIt 预检
  └─ GraspExecutor：approach → grasp → gripper
        ↓
MoveIt action/service
        ↓
Piper 驱动 → CAN → 机械臂
        ↓
/joint_states、/joint_states_feedback、FK
```

## 4. 误差可能出现的层次

| 层次 | 主要变量 | 首要代码位置 |
|---|---|---|
| 图像/深度 | 深度尺度、对齐、有效像素、ROI | `d435i_graspnet_bridge.py`, `inference_from_camera.py` |
| 感知目标 | 抓取中心、物体中心、候选平移 | `grasp_candidates.py`, `perception_grasps.csv` |
| 相机坐标 | CameraInfo、深度反投影 | `meta.mat`, `calibration.yaml` |
| 手眼外参 | `R_cam2base`, `t_cam2base` | `handeye_eye_to_hand.yaml`, `grasp_transform.py` |
| 姿态/工具几何 | GraspNet→TCP 轴映射、TCP→link6 偏移 | `grasp_transform.py`, `grasp_precheck.py` |
| 规划执行 | MoveIt 目标 link、起始状态、轨迹 | `grasp_precheck.py`, `moveit_adapter.py`, `grasp_execute.py` |
| 机械臂实际状态 | 关节零位、回差、柔顺、末端负载 | Piper 驱动、`/joint_states_feedback` |

