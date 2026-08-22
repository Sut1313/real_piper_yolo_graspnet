# D435i + GraspNet Real Piper Deployment Log

Date: 2026-08-02  
Workspace: `.`  
Source reference workspace: `<source-workspace>`

## Goal

Create a real-machine workspace for Piper + D435i + GraspNet perception:

- Start Intel RealSense D435i in ROS 2 Humble.
- Bridge aligned RGB-D topics into the GraspNet shared-folder format.
- Reuse the existing Docker GraspNet inference image and checkpoint.
- Generate grasp points and 6D grasp poses from live D435i frames.
- Display rendered grasp results through `result.png` and optional viewer.

## Workspace Layout

Important paths:

- `src/realsense-ros`: Intel RealSense ROS 2 driver, branch `ros2-master`.
- `src/real_piper_perception`: local ROS 2 package for D435i to shared-folder bridge.
- `graspnet-baseline`: copied GraspNet Docker inference code.
- `shared_camera_data`: shared folder between ROS 2 host and Docker inference.
- `run_d435i_bridge.sh`: starts D435i ROS driver and bridge.
- `run_graspnet_d435i.sh`: starts Docker GraspNet inference.
- `view_results.sh`: opens host-side result viewer.
- `scripts/cleanup_output.sh`: bounded cleanup of debug frame dumps.

Note: an intermediate template directory was first created at
`<source-workspace>/real_piper_graspnet` while filesystem permissions were
restricted. After syncing its contents to `.`, that
temporary directory was deleted. The active workspace is only
`.`.

## Dependency Setup

Downloaded RealSense ROS 2 source:

```bash
cd ~/real_piper_graspnet
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master src/realsense-ros
```

Installed missing ROS/system packages:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-diagnostic-updater ros-humble-librealsense2
```

The system has a Homebrew linker earlier in `PATH`:

```text
<local-toolchain>/bin/ld
```

That caused `realsense2_camera` linking to fail with missing `libspdlog.so.1`.
The build script now forces the system toolchain path before `colcon build`.

Build command:

```bash
cd ~/real_piper_graspnet
./build_ws.sh
```

Successful build included:

- `realsense2_camera_msgs`
- `realsense2_camera`
- `real_piper_perception`

During build and runtime the connected device was detected:

```text
Intel RealSense D435I
Serial: [REDACTED_CAMERA_SERIAL]
Firmware: 5.17.0.10
USB type: 3.2
```

## ROS Bridge Implementation

Added package:

```text
src/real_piper_perception
```

Main node:

```text
real_piper_perception/d435i_graspnet_bridge.py
```

Subscribed topics:

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

Shared-folder outputs:

- `color.png`: BGR PNG for OpenCV compatibility.
- `depth.png`: uint16 millimeter depth PNG.
- `meta.mat`: GraspNet-compatible `intrinsic_matrix` and `factor_depth`.
- `camera_info.txt`: human-readable debug metadata.
- `.ready`: frame-complete marker for Docker inference.

Frame files are written atomically through temporary files and `os.replace()`.

Launch file:

```text
src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py
```

Important launch details:

- IMU is disabled because this machine had permission errors opening D435i IIO
  motion scan elements. RGB-D grasping does not need gyro/accel.
- RealSense profile parameters use the new names:
  - `rgb_camera.color_profile`: `640,480,30`
  - `depth_module.depth_profile`: `640,480,30`
- Depth alignment is enabled with `align_depth.enable`.

Runtime command:

```bash
cd ~/real_piper_graspnet
./run_d435i_bridge.sh
```

The bridge was validated to publish live files under `shared_camera_data`.

## Docker Inference Changes

Copied the existing `graspnet-baseline` and adapted:

```text
graspnet-baseline/inference_from_camera.py
```

Key changes:

- Added `--segmentation_mode workspace` for real D435i input.
- Added `--workspace_name target`.
- Added `--min_depth_m`, `--max_depth_m`, and `--roi`.
- Workspace mode samples valid depth points in the selected depth range and ROI,
  instead of relying on Gazebo red/green/blue color segmentation.
- The output target name is `target`, written to JSON and CSV.
- Added `factor_depth` propagation from `meta.mat`.
- Added headless rendering to `result.png`, so result display does not require
  OpenCV GUI inside the Docker container.
- Wrapped OpenCV window cleanup to tolerate the container's headless OpenCV.
- Added bounded debug output behavior.

Docker run script:

```text
run_graspnet_d435i.sh
```

Default run:

```bash
cd ~/real_piper_graspnet
MIN_DEPTH_M=0.20 MAX_DEPTH_M=1.20 ./run_graspnet_d435i.sh headless
```

Tighter workspace example:

```bash
ROI=0.25,0.20,0.75,0.85 MIN_DEPTH_M=0.25 MAX_DEPTH_M=0.90 ./run_graspnet_d435i.sh headless
```

GUI mode is still available:

```bash
./run_graspnet_d435i.sh gui
```

For long-running use, prefer `headless` plus host viewer:

```bash
./view_results.sh
```

## Output Files

Live outputs:

- `shared_camera_data/result.png`: rendered point cloud and grasp poses.
- `shared_camera_data/last_validation.json`: diagnostics and grasp poses.
- `shared_camera_data/perception_grasps.csv`: CSV grasp candidates.
- `shared_camera_data/perception_targets.csv`: legacy target-center CSV.

CSV columns:

```text
name,camera_x,camera_y,camera_z,count,score,width,height,depth,
r00,r01,r02,r10,r11,r12,r20,r21,r22,
center_camera_x,center_camera_y,center_camera_z,
link6_grasp_z,tcp_forward_z,source_dist_xy
```

Example validation observed:

```text
target_grasps: 54
best_score: 0.9502255320549011
translation: [-0.048391930758953094, -0.048398472368717194, 0.2540000081062317]
```

## Debug Output Retention

Initial implementation saved every frame:

- `grasps_XXXX.npy`
- `scene_XXXX.pcd`
- `objects_XXXX.pcd`

This quickly accumulated hundreds of MB because point clouds are large. Current
default is to not save debug frame dumps. Enable bounded debug dumps only when
needed:

```bash
SAVE_DEBUG_OUTPUTS=1 SAVE_DEBUG_EVERY=20 MAX_SAVED_DEBUG_FRAMES=5 ./run_graspnet_d435i.sh headless
```

Manual cleanup:

```bash
cd ~/real_piper_graspnet
KEEP_FRAMES=5 ./scripts/cleanup_output.sh
```

## White / Empty Render Explanation

`result.png` is an Open3D rendering of valid depth point cloud plus grasp poses,
not a raw RGB image overlay. When the D435i points at a tabletop, enough depth
pixels are inside `MIN_DEPTH_M..MAX_DEPTH_M`, so the rendered scene looks like
the color camera with grasps.

When the D435i points far away:

- Most depth pixels are beyond `MAX_DEPTH_M`.
- Some distant or low-texture areas have invalid depth.
- The inference script filters invalid/out-of-range depth.
- The Open3D scene becomes nearly empty, so the render appears white or light
gray with only sparse gray points.

This is expected. For actual grasping, keep the workspace constrained to the
reachable tabletop region with ROI and depth limits. To inspect raw camera
exposure, open:

```text
shared_camera_data/color.png
```

## Piper MoveIt Real-Arm Bringup

Official references checked:

- AgileX `piper_ros` README: CAN activation, Piper control node startup, enable
  service/topic, and `/joint_states` as the real-arm joint command input.
- AgileX `piper_ros/src/piper_moveit/README.md`: real-arm MoveIt flow. The
  official ROS 1 sequence is to start the Piper control node first, wait for
  successful enable, then start `piper_with_gripper_moveit demo.launch`.

ROS 1 official sequence:

```bash
bash can_activate.sh can0 1000000
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
roslaunch piper_with_gripper_moveit demo.launch
```

ROS 2 Humble equivalent used in this workspace:

```bash
cd .
source /opt/ros/humble/setup.bash
source install/setup.bash
```

CAN setup:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

Expected CAN state:

- Interface: `can0`
- Bitrate: `1000000`
- State: `ERROR-ACTIVE`
- `bus-errors`, `error-warn`, `error-pass`, and `bus-off`: `0`

Start the real Piper control node first:

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=true \
  gripper_exist:=true \
  gripper_val_mutiple:=2 \
  log_level:=warn
```

Then start MoveIt:

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_with_gripper_moveit demo.launch.py
```

Observed successful startup messages:

```text
Configured and activated arm_controller
Configured and activated gripper_controller
You can start planning now!
Ready to take commands for planning group arm.
```

Runtime verification commands:

```bash
ros2 node list | sort
ros2 topic list -t | sort
ros2 topic echo /arm_status --once
ros2 action list -t | sort
ros2 topic info /joint_states --verbose
ros2 topic info /arm_controller/joint_trajectory --verbose
```

Expected nodes include:

- `/piper_ctrl_single_node`
- `/move_group`
- `/rviz`
- `/controller_manager`
- `/arm_controller`
- `/gripper_controller`
- `/joint_state_broadcaster`

Expected arm status:

```text
ctrl_mode: 1
arm_status: 0
err_code: 0
communication_status_joint_1: false
communication_status_joint_2: false
communication_status_joint_3: false
communication_status_joint_4: false
communication_status_joint_5: false
communication_status_joint_6: false
```

Important ROS 2 graph detail:

- Piper real-arm driver subscribes to `/joint_states`.
- `joint_state_broadcaster` publishes `/joint_states`.
- MoveIt sends execution to `arm_controller` /
  `/arm_controller/follow_joint_trajectory`.
- `arm_controller` belongs to the MoveIt ros2_control launch and the URDF
  currently reports hardware `FakeSystem`.

This means the current ROS 2 branch follows the official launch order, but the
real-arm motion path depends on the package's `/joint_states` remap behavior.
If RViz reports trajectory execution succeeded while the physical arm does not
move, inspect the ROS graph first:

```bash
ros2 node info /piper_ctrl_single_node
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

Safety notes:

- Keep the Piper base fixed and the workspace clear before pressing
  `Plan & Execute` in RViz.
- Do not publish test targets from the terminal unless intentionally performing
  a controlled motion test.
- `ros2 launch ... demo.launch.py` may log octomap depth updater errors when no
  compatible depth updater plugin is configured. This does not prevent basic
  joint-space planning and RViz operation.
- On Ctrl-C shutdown, MoveIt/RViz may print class-loader warnings or segfault
  during teardown. This was observed during shutdown after nodes had already
  stopped their controllers.

If the raw color image itself is washed out, reduce ambient light/reflections or
set manual RealSense color exposure in the launch file.

## Verified Commands

Build:

```bash
cd ~/real_piper_graspnet
./build_ws.sh
```

Start D435i bridge:

```bash
cd ~/real_piper_graspnet
./run_d435i_bridge.sh
```

Start inference:

```bash
cd ~/real_piper_graspnet
MIN_DEPTH_M=0.20 MAX_DEPTH_M=1.20 ./run_graspnet_d435i.sh headless
```

View latest result:

```bash
cd ~/real_piper_graspnet
./view_results.sh
```

Check depth coverage:

```bash
cd ~/real_piper_graspnet
python3 - <<'PY'
import cv2, numpy as np
d = cv2.imread('shared_camera_data/depth.png', cv2.IMREAD_UNCHANGED)
print('shape:', d.shape, d.dtype)
print('valid:', np.count_nonzero(d))
print('0.2-1.2m:', np.count_nonzero((d >= 200) & (d <= 1200)))
print('max:', d.max())
PY
```

## Current Runtime State At End Of Work

The D435i bridge was running in the background after the final restart and
using the corrected RealSense profiles:

```text
Depth: 640x480x30
Color: 640x480x30
```

No Docker inference process was intentionally left running by this log.

## Remaining Robot-Side Work

This log validates the perception side: D435i frames to GraspNet grasp
candidates and rendered results. Before commanding real Piper motion:

- Calibrate camera-to-robot extrinsics.
- Define reachable ROI and depth window for the physical workspace.
- Verify `perception_grasps.csv` frame convention against the robot controller.
- Add safety checks for collision, table height, joint limits, and emergency stop.
