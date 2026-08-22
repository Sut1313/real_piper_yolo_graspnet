# TCP 偏移距离 URDF 修正 实施计划（2026-08-17）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将抓取链路中当前硬编码的 TCP 偏移距离 `TCP_OFFSET_FROM_LINK6_Z = 0.14`（仿真近似值）修正为基于 URDF 几何的精确值，使预检/执行的 TCP→link6 折算与真实夹爪几何一致。

**Architecture:** 单点修改——`grasp_precheck.py:54` 的常量 `TCP_OFFSET_FROM_LINK6_Z`（所有折算/验证路径都经此常量：`grasp_execute.py` 复用 `Prechecker._tcp_to_link6_matrix`，改一处即全链路生效）。前置 Task 0 先裁决"TCP 偏移的物理定义"（候选值见下），再改值、改测试、真机干跑验证。

**Tech Stack:** Python 3 / ROS2 humble / pytest 6.2 / MoveIt（ssik 子进程仅解 base_link→link6，不涉及偏移）

**Spec:** 无正式 spec 文档；依据用户指令（2026-08-16："根据 urdf TCP 在 z 方向的长度，修改当前使用 TCP 偏移距离"）+ 代码注释 `grasp_precheck.py:46-54`（自述 0.14 为"该值的仿真近似，Task 9 干跑联调"）

## Global Constraints

- 只改偏移**数值与注释**，不动 TCP 轴映射（`GRASPNET_TO_PIPER_TCP_R`，grasp_transform.py:39）与链结构
- 不修改 `inference_from_camera.py` 的 `PIPER_TOOL_Z_OFFSET=0`（推理侧仅诊断字段，约定独立，除非 Task 0 裁决要求统一）
- 全部数值改动遵循 TDD：先改测试断言（RED）→ 改常量（GREEN）
- 真机实抓验证需用户在场，节点 `scan_only:=true` 默认安全模式
- 任务执行前先完成 Task 0 的裁决并记录到本计划（结果追加至"Task 0 结论"小节）

---

## 背景事实（2026-08-16 已核实，全部可复现）

**当前代码**：
- `grasp_precheck.py:54`：`TCP_OFFSET_FROM_LINK6_Z = 0.14`
- 折算路径：`grasp_precheck.py:140`（`offset_inv[2,3] = -TCP_OFFSET_FROM_LINK6_Z`，TCP→link6，右乘 T(0,0,-0.14)）；`grasp_precheck.py:254`（FK 验证反向 +0.14）；`grasp_execute.py:299-303` `_tcp_pose_to_link6` 复用 `Prechecker._tcp_to_link6_matrix`
- 来源：`piper_analytic_ik.cpp:30` `kTcposeOffsetFromLink6Z` 原值；`grasp_executor.py:444` 注释提示"0.14m（旧 calib_18 水平语义）"
- 测试硬编码：`test/test_grasp_execute_units.py:68`（`0.8 - 0.14`）、`:80`（`0.2 + 0.14`）

**URDF 几何（piper_description.urdf + STL 实测）**：
- `joint7/joint8` origin：`xyz=[0, 0, 0.1358]`——铰点平面在 link6 系 Z = **0.1358m**（joint6_to_gripper_base origin 0，gripper_base 系 = link6 系；已用代码验证：`R6.T @ (pos7-pos6) = [0, 0, 0.1358]`）
- 手指滑移轴 = link6 系 Z（joint axis `[0,0,1]`/`[0,0,-1]`），每指行程 0.035m，总开度 0.07m
- `link7.STL` 包围盒（link7 系）X ±28mm / Y 0~-76.5mm / Z 0~26.5mm；变换到 link6 系实测：**Z 覆盖 0.0593 ~ 0.1358m，指尖端面（远端 26.5mm 厚面）在 link6 系 Z = 0.0593 ~ 0.0658m**
- 即：**手指从铰点（Z=0.1358）沿 -Z 向下延伸 76.5mm，指尖夹取平面 ≈ Z=0.06m**；当前偏移 0.14m 位于夹爪结构本体之外（> 铰点 0.1358），与 URDF 几何矛盾

**候选偏移值**（Task 0 裁决对象）：

| 候选 | 值 | 物理含义 | 依据 |
|---|---|---|---|
| A | **0.1358** | TCP = 铰点平面（指根） | URDF joint7 origin，精确 |
| B | **≈0.060**（0.0593~0.0658，取指尖夹取面中心） | TCP = 指尖夹取平面 | STL 实测，需测 26.5mm 厚面的夹取面到底面/中面 |
| C | 0.14 | 现状（仿真近似/旧 calib_18） | 与 URDF 矛盾，拟弃用 |

> 注：precheck.py:51-52 注释自述"指尖平面 ≈ link6 + 0.136m（+ 指尖几何）"与 STL 实测（0.06m）矛盾——0.136 是铰点而非指尖平面，注释本身即错误源头之一，Task 1 一并修正。

---

### Task 0: 裁决 TCP 偏移精确值（调研 + 真机实测，不写代码）

**Files:**
- 只读：`src/piper_ros/src/piper_description/urdf/piper_description.urdf`、`meshes/link7.STL`、`link8.STL`
- 产出：本计划"Task 0 结论"小节（新值 + 理由 + 实测记录）

**Interfaces:**
- Produces: `NEW_TCP_OFFSET`（float，米）——所有后续任务使用该值替换 0.14

- [ ] **Step 1: 复核 STL 夹取面**（link7/link8 在 link6 系 Z 覆盖，验证 0.0593~0.0658）

```bash
python3 - <<'EOF'
import sys; sys.path.insert(0, 'scripts')
import numpy as np, struct
from pathlib import Path
import tf_tree_viewer as tv
edges = tv.load_urdf_edges(tv.DEFAULT_ROBOT_URDF)
edges.append(tv.load_handeye_edge(tv.DEFAULT_HANDEYE))
edges += tv.compile_camera_xacro(tv.DEFAULT_CAMERA_XACRO)
poses = tv.compute_frame_poses(edges, 'base_link')
def stl_verts(p):
    d = Path(p).read_bytes(); n = struct.unpack('<I', d[80:84])[0]; o = 84; out = []
    for _ in range(n):
        out.append(struct.unpack('<9f', d[o+12:o+48])); o += 50
    return np.array(out).reshape(-1,3)
pos6, R6 = poses['link6']; pos7, R7 = poses['link7']
for m in ('link7', 'link8'):
    v = stl_verts(f'./src/piper_ros/src/piper_description/meshes/{m}.STL')
    v6 = (v @ R7.T + pos7 - pos6) @ R6
    print(m, 'link6系Z覆盖:', round(v6[:,2].min(),4), '~', round(v6[:,2].max(),4))
EOF
```

- [ ] **Step 2: 真机实测指尖夹取平面**（需用户在场）
  1. `moveit_go_ready.py` 回位姿 → `ros2 run` 或 launch `grasp_execute.launch.py scan_only:=true`
  2. 在 grasp 位（`dry_run_max_stage=grasp` 截断）用**卷尺/卡尺**量 link6 法兰面到两指夹取中心面的距离，记为 `L_measured`（mm）
  3. 与候选 A（135.8mm）/ B（≈60mm）对比，判定 TCP 定义：夹取中心（两指之间）→ 候选 B；指根/铰点 → 候选 A
- [ ] **Step 3: 记录裁决**：在下方"Task 0 结论"写入 `NEW_TCP_OFFSET` 数值、依据（实测照片/数值）、与 0.14 的差异量（mm）
- [ ] **Step 4: 提交裁决记录**

```bash
git add docs/superpowers/plans/2026-08-17-tcp-offset-urdf-fix.md
git commit -m "docs: TCP offset decision (0.14 -> <NEW_TCP_OFFSET>)"
```

**Task 0 结论**（执行时填写）：
- NEW_TCP_OFFSET = ___ m
- 依据：___
- 与 0.14 差异：___ mm

---

### Task 1: 更新常量与注释（grasp_precheck.py）

**Files:**
- Modify: `src/real_piper_perception/real_piper_perception/grasp_precheck.py:46-54, 254`
- Test: `src/real_piper_perception/test/test_grasp_execute_units.py:60-82`

**Interfaces:**
- Consumes: `NEW_TCP_OFFSET`（Task 0 结论）
- Produces: `TCP_OFFSET_FROM_LINK6_Z = NEW_TCP_OFFSET`（签名不变，下游自动生效）

- [ ] **Step 1: 改测试断言（RED）**——`test_grasp_execute_units.py` 两处硬编码 0.14 改为 NEW_TCP_OFFSET（注释同步改）

```python
# :68 单位姿态分支
assert np.allclose(out['pos'], [0.2, 0.2, 0.8 - NEW_TCP_OFFSET], atol=1e-9)
# :80 绕 X 90° 分支
assert np.allclose(out['pos'], [0.2, 0.2 + NEW_TCP_OFFSET, 0.8], atol=1e-9)
```

- [ ] **Step 2: 验证 RED**

```bash
cd src/real_piper_perception && python3 -m pytest test/test_grasp_execute_units.py -v
# 预期：两个数值断言 FAIL（0.14 与新值不一致）
```

- [ ] **Step 3: 改常量（GREEN）**

```python
# grasp_precheck.py:54
TCP_OFFSET_FROM_LINK6_Z = NEW_TCP_OFFSET  # 2026-08-17 按 URDF 修正（见 docs/superpowers/plans/2026-08-17-tcp-offset-urdf-fix.md）
```

- [ ] **Step 4: 修正注释**（三处错误/过时描述）
  - `grasp_precheck.py:51-52`："指尖平面 ≈ link6 + 0.136m（+ 指尖几何）"改为实测结论（如 "指尖夹取面 link6 系 Z ≈ 0.060m（STL 实测），铰点 0.1358m；偏移取 NEW_TCP_OFFSET"）
  - `grasp_precheck.py:11`（docstring "0.14m 偏移"）与 `:137-138`（"沿 TCP Z 轴 -0.14m"）同步为新值
  - `grasp_execute.py:122, 301-302` 注释（"0.14m 为 link6→指尖平面 ≈0.136m 的仿真近似值"）同步
- [ ] **Step 5: 验证 GREEN + 全量测试**

```bash
cd src/real_piper_perception && python3 -m pytest test/ -q
cd . && python3 -m pytest test/test_tf_tree_viewer.py -q
# 预期：全部通过；检查 test_grasp_precheck.py 无 0.14 数值断言残留
```

- [ ] **Step 6: 提交**

```bash
git add src/real_piper_perception/real_piper_perception/grasp_precheck.py src/real_piper_perception/real_piper_perception/grasp_execute.py src/real_piper_perception/test/test_grasp_execute_units.py
git commit -m "fix: TCP offset from URDF (0.14 -> NEW_TCP_OFFSET)"
```

---

### Task 2: 全链路回归 + 干跑预检验证

**Files:**
- 只读：`launch/grasp_execute.launch.py`、`scripts/moveit_go_ready.py`

**Interfaces:**
- Consumes: `TCP_OFFSET_FROM_LINK6_Z` 新值（Task 1）

- [ ] **Step 1: 单元回归**（含 ssik 路径）

```bash
cd .
python3 -m pytest test/ -q                      # 顶层工具测试
cd src/real_piper_perception && python3 -m pytest test/ -q   # 感知包测试
# 预期：全部通过
```

- [ ] **Step 2: 干跑预检（scan_only，臂不动）**——需用户在场

```bash
source install/setup.bash
ros2 launch src/real_piper_perception/launch/grasp_execute.launch.py \
  scan_only:=true planning_time_precheck:=0.5
# 预期：scan_only 报告无"全过候选"异常；若候选通过数相比 0.14 版本变化，
# 记录差异（偏移变小/变大对可达性的影响方向）
```

- [ ] **Step 3: 真机实抓单次验证**——需用户在场二次确认

```bash
# 在 Step 2 基础上切换 scan_only:=false，单次抓取
# 观察：approach 到位后 link7/指尖相对物体的实际间隙（目视/夹爪反馈 verify_grasp）
# 判定：与 0.14 版相比抓取成功率/间隙是否改善；记录结果
```

- [ ] **Step 4: 更新文档**：`docs/2026-08-17-session-summary.md` 记录新偏移值、实测数据、抓取结果
- [ ] **Step 5: 提交**

```bash
git add docs/
git commit -m "docs: TCP offset validation results (2026-08-17)"
```

---

## Self-Review（计划自查）

1. **Spec 覆盖**：用户指令"根据 URDF TCP 在 Z 方向长度修改偏移"→ Task 0（确定精确值）+ Task 1（改值）+ Task 2（验证）全覆盖；"只写计划不实施"→ 本计划不包含任何实现动作（Task 0 的只读计算与实测是调研，不落代码）
2. **占位符扫描**：唯一占位符 `NEW_TCP_OFFSET` 是 Task 0 的交付物（裁决产出），Task 1/2 依赖它——属计划内显式依赖，非 TBD
3. **类型/签名一致性**：`TCP_OFFSET_FROM_LINK6_Z` 常量名与下游使用（precheck.py:140 / execute.py 复用）保持不变；测试断言数值随常量联动
4. **已知风险**：若 Task 0 裁决为候选 B（≈0.06），偏移变化 80mm 量级，预检可达性结果可能显著变化（方向：TCP 更贴近指尖，approach 间隙语义改变）——Task 2 Step 2 专门记录该影响
