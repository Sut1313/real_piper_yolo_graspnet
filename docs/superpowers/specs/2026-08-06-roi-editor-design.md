# 动态 ROI 编辑器（test/roi_editor.py）设计文档

日期：2026-08-06
状态：已批准（方案 A）

## 背景

当前 graspnet 推理的 ROI 是静态 CLI 参数 `--roi x_min,y_min,x_max,y_max`（归一化 0~1），在
`run_graspnet_d435i.sh` 中通过环境变量 `ROI` 传入，启动后不可更改。现场调试时需要频繁重启
推理容器来调整工作区范围，效率低。

目标：新建 `test/` 文件夹下的测试程序，运行时通过 4 个输入框实时调整 ROI 矩形，
下一帧推理即生效，无需重启。

## 已确认的需求（用户确认）

1. ROI 为完整矩形：四条边 `x_min, y_min, x_max, y_max`（归一化 0~1，与现有 `--roi` 格式一致）
2. 交互方式：仅输入框（无鼠标拖动），回车/按钮生效
3. 实时画面预览：窗口内显示相机画面（读共享目录 color.png），ROI 矩形亮色叠加，边改边看

## 关键技术结论

- **GraspNet 不固定图像尺寸**：网络输入是点云（固定采样 `num_point=20000`），与图像分辨率无关
- 现有 ROI 实现是**像素掩码**（`inference_from_camera.py:407-418`），不裁剪图像、不改内参，
  因此动态化 ROI 无需处理内参/主点偏移问题

## 架构

```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│  test/roi_editor.py (宿主)   │        │  docker graspnet 推理容器      │
│  Tkinter 窗口:               │        │  inference_from_camera.py     │
│  ├ 4 输入框 + 生效按钮        │        │  process_frame():             │
│  ├ 实时画面(读 color.png)    │ 写     │  ├ 读 roi.json (mtime 缓存)    │
│  └ ROI 矩形叠加              ├───────►│  └ 应用到 workspace_mask      │
└─────────────┬───────────────┘        └──────────────────────────────┘
              │ 读 color.png
              ▼
     shared_camera_data/ (桥接写入)
```

## 组件

### 1. `test/roi_editor.py`（新文件，宿主机，Tkinter）

- **输入区**：4 个 Entry（x_min, y_min, x_max, y_max）+ "生效"按钮（回车等价点击）
- **预览区**：读 `shared_camera_data/color.png`（~2s 刷新一次，与桥接发布频率一致），
  按当前输入框值绘制 ROI 矩形（亮绿色，线宽 2），显示当前帧时间戳
- **校验**：输入必须为 0~1 的浮点数且 `x_min < x_max`、`y_min < y_max`；
  非法时输入框红底提示、不写入文件
- **写入**：生效时原子写 `shared_camera_data/roi.json`（tmp + rename）：
  ```json
  {"x_min": 0.10, "y_min": 0.20, "x_max": 0.85, "y_max": 0.90}
  ```
- 程序启动时若 roi.json 已存在，输入框初始化为其值

### 2. `inference_from_camera.py` 改动（挂载进容器，宿主机源码）

- 新增 `load_dynamic_roi(shared_dir)`：每帧直接读 `roi.json`（~400B，帧间隔 2s，
  读盘开销可忽略）——**不做 mtime 缓存**：实测本机内核 coarse 时钟时间戳下，
  同刻连续写入会得到相同 `st_mtime_ns`，缓存会导致 ROI 更新静默丢失
  （2026-08-06 单元测试实测发现）；解析失败返回 None 并告警一次
- `process_frame()` 中：先尝试动态 ROI，无则回退 `parse_roi(cfgs.roi)`（行为与现在完全一致）
- 日志打印 `roi=` 时同时显示来源（dynamic/file 或 static/cli）

## 数据流

```
用户改输入框 → 回车 → 校验 → 原子写 roi.json
→ 推理下一帧读文件（mtime 缓存命中则跳过）→ 生成掩码叠加 workspace_mask
→ 采样 num_point 点云 → GraspNet → 抓取输出
```

## 错误处理

| 场景 | 行为 |
|---|---|
| roi.json 不存在 | 回退 `--roi` 静态参数（现状） |
| roi.json 格式非法/值越界 | 跳过动态 ROI 用全幅，日志告警，不崩溃 |
| 输入框非法（非数字/越界/顺序错） | 红框提示，不写文件 |
| 写入中断 | 原子写（tmp+rename）保证不会出现半截文件 |

## 测试

1. 仿真验证：`run_graspnet_d435i.sh` + roi_editor 同时运行，改输入框回车后，
   观察推理日志 `roi=` 下一帧起变化（无需重启容器）
2. 边界测试：输入 `0.5,0.5,0.3,0.8`（x_min>x_max）应被拦截
3. 回退测试：删除 roi.json，推理恢复全幅/`--roi` 静态行为

## 非目标

- 不做鼠标拖拽边界
- 不做深度边界（min/max depth）动态化——已有 `MIN_DEPTH_M`/`MAX_DEPTH_M` 环境变量，如需动态化另行设计
- 不改桥接（bridge 只负责采帧写共享目录，与 ROI 无关）
- 不做真裁剪（保持掩码方案，不改内参）
