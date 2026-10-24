# real_piper_yolo_graspnet

真机 D435i 的 YOLOv8-Seg 感知入口。该包只替换原项目的目标分割前端，
不包含 Gazebo、仿真执行器或机械臂控制代码。

## 数据流

```text
aligned RGB-D -> YOLO ONNX -> selected instance mask
  -> color.png / depth.png / meta.mat / instances.npz
  -> frame_manifest.json -> .ready
  -> GraspNet --segmentation_mode yolo
  -> 原 24 列 perception_grasps.csv
  -> 原 real_piper_perception 真机执行链
```

生产者仅在 `.ready` 不存在时提交新帧，且总是最后写入 `.ready`。GraspNet
消费完成后删除该标志，因此 RGB、深度和实例掩膜不会跨帧混合。

## 启动

从仓库根目录构建后：

```bash
./run_d435i_yolo_bridge.sh
./run_graspnet_yolo.sh headless
```

可用环境变量覆盖：

```bash
TARGET_CLASS=bottle \
TABLETOP_YOLO_MODEL=/absolute/path/to/tabletop_yolov8s_seg_v2.onnx \
./run_d435i_yolo_bridge.sh
```

ROS 的系统 Python 没有安装 ONNX Runtime 时，节点默认复用相邻 YOLO 工程
虚拟环境中的运行库，也可显式指定：

```bash
YOLO_ONNXRUNTIME_SITE_PACKAGES=/absolute/path/to/site-packages \
./run_d435i_yolo_bridge.sh
```

首轮联调继续使用原执行器的 `scan_only:=true`，确认候选转换和预检后再执行真机动作。
