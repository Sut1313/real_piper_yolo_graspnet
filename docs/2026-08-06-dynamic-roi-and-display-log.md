# 2026-08-06 动态 ROI + 可视化显示开发排障日志

## 结论摘要

本会话完成了四件事：**真机全链路首次打通**、**动态 ROI 编辑器**、**双视图可视化显示**、
**3D 取景修正**，并修复了 5 个实际问题（其中 2 个是本会话新引入的 bug）。
设计文档：`docs/superpowers/specs/2026-08-06-roi-editor-design.md`。

涉及文件：
- `test/roi_editor.py`（新增，动态 ROI 编辑器）
- `graspnet-baseline/inference_from_camera.py`（动态 ROI 读取、双视图显示、取景修正、通道修复、启动清理）
- `src/real_piper_perception/launch/d435i_graspnet_pipeline.launch.py`（短暂改 480×480 后还原）
- `.bashrc`（移除 brew shellenv 行）

---

## 一、真机全链路首次打通（桥接 + Docker 推理 + 可视化）

### 问题 1：ROS2 工作区从未构建

**现象**：`run_d435i_bridge.sh` 报
`Package 'real_piper_perception' not found, searching: ['/opt/ros/humble']`。

**根因**：`src/` 下有 `real_piper_perception`、`realsense-ros` 等包，但整个工作区
从未 `colcon build`，无 `install/` 目录。

**修复**：`colcon build --symlink-install`。期间发现第 2 个问题（见下），
重试后成功；`real_piper_perception` 是纯 Python 包（0.8s 构建）。

### 问题 2：linuxbrew 的 ld 破坏 ROS2 链接

**现象**：`realsense2_camera` 链接失败：
`ld: warning: libspdlog.so.1, needed by librcl_logging_spdlog.so, not found`
+ 大量 `undefined reference to spdlog::...`。

**根因**（证据链）：`which ld` → `<local-toolchain>/Cellar/binutils/2.47/bin/ld`
（linuxbrew 在 PATH 首位，fishros 安装）；其 ld 不搜索 `/usr/lib/x86_64-linux-gnu`；
系统 `libspdlog.so.1`（1.9.2）齐全，用 `/usr/bin/ld` 即正常。

**修复**（应 zzu 要求删除 linuxbrew，2026-08-06）：
- `sudo rm -rf <local-toolchain>`（889M）+ `~/.homebrew`（信任缓存）+ `sudo rmdir <local-homebrew>`
- `.bashrc` 删除 `eval "$(<local-toolchain>/bin/brew shellenv bash)"` 一行
- 验证：全盘 grep 无残留引用；注意 **Claude 会话内 shell 快照可能残留旧 PATH**（会话启动时缓存），
  构建命令需 `export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin` 兜底

### 问题 3：相机启动瞬时 USB 协议错误

**现象**：桥接启动时 `set_xu xioctl(UVCIOC_CTRL_QUERY) failed ... Protocol error` +
`device has been disconnected`，随后自动重枚举成功（Device USB type: 3.2）。

**结论**：一次性抖动，重连后无再断连，未处理。启动前检查
`cat /sys/bus/usb/devices/2-3/speed` = 5000（USB 3.x，见 08-02 日志的根因一）。

### 验证

桥接每 ~2s 发布一帧（`Published D435i frame`），推理每帧解码 ~58 抓取 →
碰撞过滤 ~35；`result.png` 实时更新；matplotlib 窗口 "GraspNet - Perception Results" 显示。

---

## 二、分辨率 480×480 尝试（未实施，硬件不支持）

**请求**：改为 480×480。改 launch 两处 profile 后相机拒绝：
`Given value, 480,480,30 is invalid`（depth 回退 848×480、color 回退 1280×720）。

**证据链**：`rs-enumerate-devices` 枚举全部支持模式——Depth 和 Color **均无 480×480**。
原因：传感器模式表由固件预设（ISP 裁剪/缩放档位），无 1:1 方形档；480×480 需非对称裁剪，
固件不暴露。**GraspNet 本身不限制分辨率**（输入是点云，固定 num_point=20000）。

**处理**：用户选择保持 640×480，launch 文件还原（当前文件即最终状态）。

---

## 三、动态 ROI 编辑器（test/roi_editor.py，方案 A）

### 设计（已批准，见 spec 文档）

- 编辑器（宿主机 Tkinter）：4 个归一化输入框（回车生效）+ 实时画面（读 color.png，
  2s 刷新）+ ROI 绿框叠加；校验（0~1 且 x_min<x_max, y_min<y_max，非法红框拦截）；
  原子写 `shared_camera_data/roi.json`（tmp + rename）
- 推理端：`load_dynamic_roi()` 每帧读 roi.json，无文件/非法回退静态 `--roi`（行为不变）

### 问题 4：mtime 缓存导致 ROI 更新静默丢失（测试抓到的真 bug）

**现象**：单元测试"同秒连续写两次"失败——第二次写入不被识别。

**根因**（实测证据）：两次写入得到**完全相同**的 `st_mtime` 甚至 `st_mtime_ns`
（`1786016785403913290` 两次一致）。本机内核用 coarse 时钟记文件时间戳，
同一 tick 内的写入时间戳相同 → 任何 mtime 缓存都不可靠。

**修复**：**放弃 mtime 缓存，每帧直读**（roi.json 仅 ~400B、帧间隔 2s，开销可忽略）。
日志只在值变化时打印（`_dynamic_roi_cache = {'warned': False, 'last': None}`）。

**验证**：单元测试 7 项全过（无文件/合法/同刻改写/顺序非法/非 JSON/删除/恢复）；
实机验证：写 ROI 后下一帧生效，点云 299,736 → 76,722（≈25.6%，与 0.5×0.5 矩形面积比精确吻合）。

---

## 四、matplotlib 显示 ROI 未生效 → 双视图改造

**现象**：编辑器改 ROI 后，推理日志 `roi=... (dynamic)` 已生效（点云数变化），
但 matplotlib 窗口看不出任何变化。

**根因**：窗口只显示 3D 渲染图（点云透视投影），ROI 是 2D 像素掩码，
在该视图上没有对应位置可画——**取景问题而非推理问题**。

**改动**：`display_loop` 改双视图（左：相机彩色图 + ROI 绿框；右：3D 渲染+抓取）；
`process_data` 返回实际应用的 `roi`/`roi_source`；新增 `draw_roi_overlay()`；
新增 `latest_color_image` 全局；存 `result_color.png`（带 ROI 框的 2D 视图）。

### 问题 5：latest_color_image 漏写 global（本会话引入）

**现象**：result.png 正常更新但 result_color.png 从不生成，0 显示错误。

**根因**：主循环 `global` 声明漏了 `latest_color_image` → 赋值成为函数局部变量，
模块级全局从未更新。

**修复**：`global running, frame_count, latest_result_image, latest_color_image, inference_times`。

**验证**：ROI 绿框逐像素验证——水平边 y=120/360（0.25/0.75）、垂直边 x=192/512（0.30/0.80），
与 roi.json 完全一致；改 ROI 后框实时移动。

---

## 五、3D 取景修正（斜视 + 物体出画）

**现象**：物体放画面中央，3D 视图里却挤在底部、甚至只看到一小条；整体"斜"。

**根因**（`render_result` 旧取景逻辑）：
1. 视野中心 = **全场景**包围盒（含 1.2m 远的背景），物体（近场 Z≈0.3m）被投影到画面下缘
2. `offset = max(场景边长×0.5, 0.35)` 在 60° FOV 下距离不足 → 视锥裁剪
3. 固定 45° 后上方俯视 → 桌面透视成平行四边形（"斜"的观感）

**修复**（2026-08-06）：
1. 视野中心 = **物体点云包围盒**（点数 ≥50 且非退化，否则回退全场景）
2. 视线距离按 FOV 计算：`half = max(物体最大边长×0.5×1.3, 0.30)`，
   垂直/水平 FOV 分别约束取较大值 → 物体完整入画
3. 俯角从 45° 降到 **25%** 抬高（`elev=0.25`）

**验证**（容器内用真实点云 objects_0073.pcd 渲染对比）：

| 取景 | 物体像素位置 | 中心 | 出画 |
|---|---|---|---|
| 旧 | x[748,1182] y[711,719]（8px 高残影） | (965,715) | **是** |
| 新 | x[449,828] y[218,475] | (638,346)≈(640,360) | 否，占比 10% |

---

## 六、RGB 通道双转换 bug（本会话引入）

**现象**：matplotlib 左侧 ROI 画面整体偏蓝（红蓝互换）。

**根因**（代码证据）：
- `read_data_from_shared:223` 已做 **BGR→RGB 一次转换**（内部管线约定：
  `color_vis` 是 RGB，点云颜色即用 RGB → 3D 视图颜色一直正确）
- 本会话加的显示代码又做了一次 `cvtColor(BGR2RGB)` → **双转换**，红蓝互换
- 脚本注释第 220-222 行明确记载过这个历史 bug 模式（"若 bridge 侧也转了，这里就会转两次"）

**修复**：
- `display_loop`：左侧面板直接 `img_color.set_data(color)`（RGB 不再转）；
  存 `result_color.png` 时 `cvtColor(RGB2BGR)`（文件与桥接写盘的 color.png 同为 BGR 布局）
- `render_result` 两个回退路径（No grasps / Render error）同样补 RGB→BGR
- `draw_roi_overlay` 文档注明绿框 (0,255,0) 在 RGB/BGR 下对称不受影响

**验证**：result_color.png 与 color.png 逐像素 98.2% 一致（差异仅 ROI 框+文字+帧间运动），
时间戳相差 3ms；当前场景全灰无法做饱和色判别，以代码路径单次转换确定。

---

## 七、启动清理（新增 cleanup_stale_outputs）

**分析结论**（启动前原先不删除任何旧内容）：
- 固定文件名（color/depth/result/CSV/validation）每帧覆盖 → 运行中不累积
- `output/` 调试帧由 `cleanup_debug_outputs` 保留最近 N 帧 → 有界
  （但跨运行语义有瑕疵：新运行的 0001 帧可能被旧运行的 0043 帧"挤掉"）
- 跨运行残留（result*.png、CSV、调试 PCD）会一直留在磁盘——不删但不增长

**改动**：`cleanup_stale_outputs(shared_dir)` 在 `main()` 启动时调用：
清理 result.png / result_color.png / last_validation.json / perception_targets.csv /
perception_grasps.csv / .ready / output/ 全部文件；**保留 roi.json**（编辑器持久配置）。
已验证：重启时打印 `[cleanup] 启动时清理旧输出: ...`（首测清掉 14 个文件）。

---

## 当前状态

- 管线三进程（桥接/推理/编辑器）均已按用户要求关闭，无残留
- 设计文档：`docs/superpowers/specs/2026-08-06-roi-editor-design.md`
- 待办（用户未要求实施）：3D 取景其余优化（如俯角参数化、物体太小回退场景时的视野）
