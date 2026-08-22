# 2026-08-06 master 镜像构建与仿真闭环验证

## 一、背景

`graspnet-baseline:20.04` 镜像(8月2日构建)内 `pointnet2_utils.py` 是旧版
(7月21日,442 行),`QueryAndGroup.forward` / `GroupAll.forward` 缺
`ret_grouped_xyz` 返回分支,与 `pointnet2_modules.py` 的 2 元组解包不匹配,
导致推理必崩:

```text
ValueError: not enough values to unpack (expected 2, got 1)
  at pointnet2_modules.py:210  grouped_features, grouped_xyz = self.grouper(...)
```

主机修复版(7月25日,554 行,md5 `056bb538...`)含完整分支,但镜像未重建。
详见 `docs/2026-08-06-d435i-graspnet-fix-log.md`。

## 二、本次工作

### 1. 三个工作区代码统一为修复版

| 工作区 | 处理 |
|---|---|
| `~/piper_graspnet` | 原本就是修复版(7月25日) |
| `~/real_piper_graspnet` | 原本就是修复版 |
| `~/master/piper_graspnet` | rsync 同步修复版(49 个差异文件覆盖,仅剩 .git 元数据差异) |

### 2. docker-build.sh 增加构建前校验

三个工作区的 `graspnet-baseline/docker-build.sh` 均加入校验块:构建前检查
`pointnet2/pointnet2_utils.py` 是否含 `if self.ret_grouped_xyz:` 分支(修复版
特征,3 处;旧版 0 处)。误用旧版目录构建会直接报错退出,防止再产出坏镜像。

### 3. 构建镜像

```bash
cd ~/master/piper_graspnet/graspnet-baseline
./docker-build.sh graspnet-baseline:20.04-master
```

- 构建方式:legacy builder(本机无 buildx),脚本自动处理
- 耗时:约 30 分钟(apt 依赖 + pip 安装 20+ 包 + pointnet2/knn CUDA 编译)
- 产物:`graspnet-baseline:20.04-master`(25.5GB)

镜像内容(Dockerfile.20.04):
1. 基础镜像 `nvcr.io/nvidia/pytorch:22.03-py3`
2. 系统包:build-essential / git / libegl1 / libgl1 / xvfb / xauth 等
3. Python 依赖:torch 全家 + numpy==1.24.4 / scipy==1.10.1 / open3d==0.17.0
   / opencv-python / matplotlib / grasp_nms / autolab_core 等
4. CUDA 扩展:`pip install ./pointnet2`(FPS/BallQuery/Grouping 算子)、
   `./knn`、`./graspnetAPI_local`
5. 项目代码:models / utils / dataset / doc / logs(checkpoint.tar)/
   inference_from_camera.py 等

### 4. 镜像验证(全部通过)

```text
pointnet2_utils.py md5        = 056bb538...  (修复版 554 行)  ✅
ret_grouped_xyz 分支          = 3 处                           ✅
GraspNet + graspnetAPI 导入   = OK                             ✅
CUDA 可用                     = RTX 3060                       ✅
checkpoint.tar (12.5M)        = 存在                           ✅
```

**关键验证:新镜像无需任何挂载即可运行推理**(修复代码已烧进镜像,
`run_graspnet_d435i.sh` 中的 pointnet2 挂载变为冗余但无害)。

## 三、仿真闭环验证结果

### 环境

- 仿真工作区:`~/piper_graspnet/gazebo_piper_pick_place`(ROS2 Humble)
- 推理镜像:`graspnet-baseline:20.04-master`(无挂载)
- GPU:RTX 3060 12GB

### 启动序列

```bash
# 终端 1:仿真 + MoveIt + 抓取主节点
cd ~/piper_graspnet/gazebo_piper_pick_place
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch piper_gazebo piper_pick_and_place.launch.py

# 终端 2:GraspNet 推理(新镜像)
cd ~/piper_graspnet
IMAGE_NAME=graspnet-baseline:20.04-master ./graspnet-docker-camera.sh gui
```

### 闭环数据流(验证通过)

```text
Gazebo 仿真相机 → camera_bridge_node → shared_camera_data/
  (color.png / depth.png / meta.mat / .ready / pick_state.json)
→ Docker 推理(20.04-master) → perception_grasps.csv + result.png
→ pick_and_place C++ 节点读 CSV → MoveIt 规划执行 → 抓取 → 更新 pick_state
→ 推理端只识别 pending 物体(闭环)
```

### 验证结果

```text
✅ 相机桥接出帧           : 19:06 起持续发布
✅ 推理加载 checkpoint     : epoch 18
✅ 推理解码抓取            : 891 个解码 / 碰撞过滤后 529 个(单帧)
✅ perception_grasps.csv   : 172KB,含完整 6D 候选
✅ result.png              : 692KB 渲染输出
✅ pick_state.json         : pending=[green_cylinder, red_box, blue_cylinder]
✅ MoveIt 执行轨迹          : Execution completed: SUCCEEDED(预位移动)
✅ 候选遍历                 : red_box 178 个 GraspNet 6D 候选逐候选 IK 预检
```

## 四、遗留注意事项

1. **仿真侧脚本 TTY 问题**:`graspnet-docker-camera.sh` 硬编码 `-it`,
   后台/无 TTY 运行会报 `the input device is not a TTY`。后台运行时需
   直接用 `docker run`(参考 real 侧脚本的 `DOCKER_TTY_ARGS` 条件判断)。
2. **IK 可达性**:red_box 部分候选 analytic IK 无解(会话记录已知的
   "朝下×左区空洞"),候选遍历会自动跳过并尝试下一个。
3. **GUI 窗口被关会崩**:matplotlib 窗口关闭导致推理进程退出,长时间运行
   建议 headless + result_viewer.py。

## 五、清理(上传 GitHub 前)

已执行:删除三个工作区的 `build/` `install/` `log/` `logs/`
`shared_camera_data/`(运行帧/调试输出)、`master/docker/` 14G 镜像 tar 备份、
`graspnet-baseline/outputs/`、嵌套 `.git`、`__pycache__`。

保留:源代码、`camera_data/` 标定文件、`checkpoint.tar` 权重、
docker 镜像(本地)、演示素材(`video/`、`resources/` gif)。

## 六、当前镜像清单

| 镜像 | 说明 |
|---|---|
| `graspnet-baseline:20.04` | 7月30日构建的修复版(验证时使用) |
| `graspnet-baseline:20.04-good` | 同一镜像备份 tag |
| `graspnet-baseline:20.04-master` | 本次从 master 重新构建(自带修复代码) |
| `nvcr.io/nvidia/pytorch:22.03-py3` | 基础镜像 |
