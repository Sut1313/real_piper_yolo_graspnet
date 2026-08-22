# 2026-08-06 D435i GraspNet 推理故障排障日志

## 结论摘要

启动 D435i → GraspNet Docker 推理管线时遇到**两个独立根因**,均已修复并验证:

1. **相机插在 USB 2.0 口** → 深度流无帧、相机复位(物理连接问题)
2. **镜像内 `pointnet2_utils.py` 有缺陷**(缺 `ret_grouped_xyz` 返回分支)→ 推理必崩(代码版本问题)

修复后推理正常:每帧 10~28 个候选抓取,`result.png` / CSV 实时更新,matplotlib 可视化窗口正常显示。

---

## 现象

- `run_d435i_bridge.sh`:RealSense 节点启动成功,但持续报
  `XXX Hardware Notification: Frames didn't arrived within 5 seconds`(每 5 秒一次)
- `shared_camera_data/` 文件停留在 08-02,无新帧
- 启动 Docker 推理后:`ValueError: not enough values to unpack (expected 2, got 1)`
  (graspnet.py → backbone.py → pointnet2_modules.py:210 grouper 调用)

---

## 根因一:D435i 接在 USB 2.0 口

### 证据链

| 检查项 | 结果 |
|---|---|
| `lsusb -t` | 相机在 **Bus 01(480M,USB 2.0)**,Bus 02(USB 3.0)无设备 |
| `/sys/bus/usb/devices/1-7/speed` | `480`(协商为 USB 2.0) |
| v4l2 直接采彩色(`ffmpeg /dev/video4`) | ✅ 3 帧正常 |
| v4l2 直接采深度(`ffmpeg /dev/video0`) | ❌ 无帧,相机随即 **USB disconnect + 重枚举** |
| 内核日志 | `uvcvideo 1-7:1.2: Failed to set UVC probe control : -32` |
| 08-02 部署日志 | 当时记录 **USB type: 3.2** |

### 原因

D435i 在 USB 2.0(480Mbps)下,640×480@30 深度(Z16)+ 彩色(RGB8)合计约
46 MB/s ≈ 368 Mbps,超过 USB 2.0 实际可用带宽。深度流被挤死 → 相机固件
处理失败直接断开复位。08-02 验证时相机在 USB 3.2,本次启动前被插到了
USB 2.0 口(或使用了 USB 2.0 线)。

### 修复

换插 **USB 3.x 蓝色口 + USB 3.0 线**(9 触点)。验证:
`/sys/bus/usb/devices/2-3/speed` = `5000`。恢复后桥接节点每 ~2s 发布一帧。

---

## 根因二:镜像内 pointnet2_utils.py 缺返回分支

### 证据链

- 容器内 `/workspace/pointnet2/pointnet2_modules.py:210`:
  `grouped_features, grouped_xyz = self.grouper(xyz, new_xyz, features)`(期望 2 元组)
- 容器内 `/workspace/pointnet2/pointnet2_utils.py`(442 行,7月21日版):
  `QueryAndGroup.forward` / `GroupAll.forward` **只返回 `new_features`** ——
  `__init__` 存了 `self.ret_grouped_xyz` 但 forward 忘记实现返回分支
- 主机 `graspnet-baseline/pointnet2/pointnet2_utils.py`(554 行,7月25日版,
  与 `~/piper_graspnet` 同 md5 `056bb538`):有完整 `ret_grouped_xyz` 分支 → 修复版
- 镜像(8月2日构建,25.5GB)内是旧版,`~/master/piper_graspnet` 也是旧版(md5 `a143fbe2`)

### 原因

镜像构建后代码在主机侧继续演进修复,但 25GB 镜像未重建,镜像内仍是
7月21日有缺陷的版本。每次推理必然在第一个 SA 层崩溃。

### 修复

修改 `run_graspnet_d435i.sh`,把主机修复版 3 个文件**只读挂载**进容器
(与既有 `inference_from_camera.py` 挂载同模式,无需重建 25GB 镜像):

```bash
-v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_utils.py:/workspace/pointnet2/pointnet2_utils.py:ro" \
-v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pointnet2_modules.py:/workspace/pointnet2/pointnet2_modules.py:ro" \
-v "${SCRIPT_DIR}/graspnet-baseline/pointnet2/pytorch_utils.py:/workspace/pointnet2/pytorch_utils.py:ro" \
```

gui 和 headless 两个分支均已加。

---

## 验证结果(修复后, 2026-08-06 18:28~18:35)

```text
-> loaded checkpoint (epoch: 18)
--- Frame 1 ---
  valid points: 292839
  workspace points: 290252 depth=(0.20,1.20)m roi=full
  decoded grasps: 28
  after collision filter: 16 (0.01s)
  showing top 16 grasps
  ├─ inference: 3.15s
  ─> 16 grasp(s) found
```

- `result.png`、`perception_grasps.csv`、`last_validation.json` 实时更新
- matplotlib 窗口 `GraspNet - Perception Results`(1280×754)实时渲染,
  两次截屏 md5 不同(内容在刷新)
- 持续运行至 Frame 40+,单帧推理 ~0.12s(缓存后),每帧 10~28 抓取

---

## 遗留注意点

1. **GUI 模式窗口被关会崩**:Frame 39 时窗口被关闭 → `BadWindow` →
   `Render error: std::bad_alloc` → 容器退出。长时间运行建议用
   `headless` + `./view_results.sh`,或不要关闭可视化窗口。
2. 若镜像被重新构建,需把 pointnet2 修复合并进构建上下文,或保持本脚本挂载。
3. 相机换口后记得用 `cat /sys/bus/usb/devices/*/speed` 确认是 `5000`。
