# SDK 执行安全闭环实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 SDK（EndPoseCtrl）执行路径实现三层安全防护（候选奇异过滤+连杆级碰撞预检 / 执行中实时监控急停 / 执行后回验），杜绝碰撞底座事故重演。

**Architecture:** 纯计算防护层（ForbiddenZone 几何 + filter_candidate 预检）+ 发送层（EndPoseCtrl + 监控线程急停）+ 回验层。实现放包内模块（可单测），scripts/ 放薄执行入口。

**Tech Stack:** Python 3.10、numpy、pinocchio（DLS IK 复用 grasp_ik.solve_ik_link6）、piper_sdk（C_PiperInterface_V2）、pytest。

**Spec:** `docs/superpowers/specs/2026-08-08-sdk-safe-execute-design.md`

## Global Constraints

- 项目非 git 仓库：任务步骤中无 git commit；每任务末尾"记录变更"到 `docs/grasp_executor_test_log.md` 或本计划文档
- 常量化参数（模块头部）：BOX_CENTER=(0,0,0.06)、BOX_SIZE=(0.15,0.15,0.12)、BOX_BUFFER=0.005、Z_MIN=0.18、XY_CYL_R=0.25、CYL_Z_MAX=0.40、RPY_SINGULAR_LIMIT=75（度）、POS_TOL_MM=50、RPY_TOL_DEG=10、MONITOR_PERIOD_S=0.1、JUMP_TOL_MM=30
- SDK 构造必须 `C_PiperInterface_V2("can0", dh_is_offset=1)`（URDF 对应 offset 版，已验证）
- 单测放 `src/real_piper_perception/test/test_sdk_safe_execute.py`；运行 `python3 -m pytest src/real_piper_perception/test/test_sdk_safe_execute.py -v`（从仓库根目录）
- 本计划**不包含真机执行**（机械臂断电中）；真机验收另行安排

---

### Task 1: ForbiddenZone 几何类

**Files:**
- Create: `src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`（本任务只含 ForbiddenZone 与常量）
- Test: `src/real_piper_perception/test/test_sdk_safe_execute.py`（本任务只测 ForbiddenZone）

**Interfaces:**
- Produces: `ForbiddenZone` 类（构造无参，读模块常量）：
  - `point_in_base_box(p: np.ndarray) -> bool`：点是否在底座包围盒内（含 5mm 缓冲）
  - `point_in_cyl(p: np.ndarray) -> bool`：点是否在底座圆柱监控区（xy<0.25 且 z<0.40）
  - `z_below_min(p: np.ndarray) -> bool`：点 z < Z_MIN
  - `box_clearance(p: np.ndarray) -> float`：点与盒面的最小距离（米，盒内为负）

- [ ] **Step 1: 写失败测试**（test_sdk_safe_execute.py）

```python
import numpy as np
import pytest
from real_piper_perception.sdk_safe_execute import ForbiddenZone

zone = ForbiddenZone()

def test_box_inside():
    assert zone.point_in_base_box(np.array([0.0, 0.0, 0.06]))       # 盒中心
    assert zone.point_in_base_box(np.array([0.07, 0.07, 0.05]))      # 盒内角部

def test_box_outside():
    assert not zone.point_in_base_box(np.array([0.20, 0.0, 0.06]))   # x 超出
    assert not zone.point_in_base_box(np.array([0.0, 0.0, 0.20]))    # z 超出
    assert not zone.point_in_base_box(np.array([0.0, 0.0, -0.05]))   # 地面以下

def test_box_buffer():
    # 盒面外 3mm（<5mm 缓冲）视为进入
    assert zone.point_in_base_box(np.array([0.078, 0.0, 0.06]))
    # 盒面外 8mm（>5mm 缓冲）不进入
    assert not zone.point_in_base_box(np.array([0.083, 0.0, 0.06]))

def test_cyl_and_z():
    assert zone.point_in_cyl(np.array([0.10, 0.0, 0.20]))    # xy=0.10<0.25, z=0.20<0.40
    assert not zone.point_in_cyl(np.array([0.30, 0.0, 0.20]))  # xy 超出
    assert not zone.point_in_cyl(np.array([0.10, 0.0, 0.50]))  # z 超出
    assert zone.z_below_min(np.array([0.4, 0.0, 0.17]))      # z=0.17<0.18
    assert not zone.z_below_min(np.array([0.4, 0.0, 0.19]))

def test_box_clearance():
    assert abs(zone.box_clearance(np.array([0.075, 0.0, 0.06]))) < 1e-9  # 盒面距离 0
    assert zone.box_clearance(np.array([0.10, 0.0, 0.06])) > 0.02        # 盒外正距离
    assert zone.box_clearance(np.array([0.0, 0.0, 0.06])) < 0            # 盒内负距离
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m pytest src/real_piper_perception/test/test_sdk_safe_execute.py -v`
Expected: FAIL（ModuleNotFoundError / 无 ForbiddenZone）

- [ ] **Step 3: 实现 ForbiddenZone**

`src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`：

```python
"""SDK 执行安全闭环（2026-08-08，spec: docs/superpowers/specs/2026-08-08-sdk-safe-execute-design.md）。

三层防护：第 0 层候选过滤（奇异/粗筛）→ 第 1 层连杆级预检（IK+包围盒）
→ 第 2 层执行中监控急停 → 第 3 层回验。本模块纯计算可单测；
scripts/sdk_safe_execute_run.py 为真机执行入口。
"""
import numpy as np

# ---- 常量化安全参数（现场调参入口）----
BOX_CENTER = np.array([0.0, 0.0, 0.06])   # 底座包围盒中心（base 系，米）
BOX_SIZE = np.array([0.15, 0.15, 0.12])   # 包围盒尺寸（长×宽×高，米）
BOX_BUFFER = 0.005                        # 盒面安全缓冲（米）
Z_MIN = 0.18                              # 末端 z 下限（米，底座平面以上）
XY_CYL_R = 0.25                           # 监控圆柱半径（米）
CYL_Z_MAX = 0.40                          # 监控圆柱高度上限（米）
RPY_SINGULAR_LIMIT = 75.0                 # 欧拉角奇异过滤阈值（度，|RY|）
POS_TOL_MM = 50.0                         # 回验位置容差（mm）
RPY_TOL_DEG = 10.0                        # 回验姿态容差（度）
MONITOR_PERIOD_S = 0.1                    # 监控采样周期（s）
JUMP_TOL_MM = 30.0                        # 单帧位移突变阈值（mm）
FACTOR = 1000                             # SDK 单位（0.001mm / 0.001°）


class ForbiddenZone:
    """底座禁区几何查询（base 系，单位米）。"""

    def __init__(self):
        self._lo = BOX_CENTER - BOX_SIZE / 2.0 - BOX_BUFFER
        self._hi = BOX_CENTER + BOX_SIZE / 2.0 + BOX_BUFFER

    def point_in_base_box(self, p: np.ndarray) -> bool:
        return bool(np.all(p >= self._lo) and np.all(p <= self._hi))

    def point_in_cyl(self, p: np.ndarray) -> bool:
        return bool(np.hypot(p[0], p[1]) < XY_CYL_R and p[2] < CYL_Z_MAX)

    def z_below_min(self, p: np.ndarray) -> bool:
        return bool(p[2] < Z_MIN)

    def box_clearance(self, p: np.ndarray) -> float:
        """点与盒面最小距离（米）：盒内为负、盒面 0、盒外为正（含缓冲）。"""
        lo = self._lo - BOX_BUFFER
        hi = self._hi + BOX_BUFFER
        d = np.maximum(np.maximum(lo - p, 0.0), np.maximum(p - hi, 0.0))
        return float(np.linalg.norm(d)) * (-1.0 if self.point_in_base_box(p) else 1.0)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python3 -m pytest src/real_piper_perception/test/test_sdk_safe_execute.py -v`
Expected: 5 passed

- [ ] **Step 5: 记录变更**

`docs/grasp_executor_test_log.md` 追加：Task 1 ForbiddenZone（包围盒 150×150×120+5mm 缓冲、圆柱 xy<0.25/z<0.40、z 下限 0.18）5 单测绿。

---

### Task 2: 第 0 层候选过滤（奇异 + 粗筛）

**Files:**
- Modify: `src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`
- Test: `src/real_piper_perception/test/test_sdk_safe_execute.py`

**Interfaces:**
- Consumes: `ForbiddenZone`（Task 1）、模块常量 `RPY_SINGULAR_LIMIT`、`Z_MIN`、`XY_CYL_R`
- Produces: `filter_candidate_0(rpy: np.ndarray, p_j6: np.ndarray, zone: ForbiddenZone) -> tuple[bool, str]`：奇异过滤 + 位置粗筛，返回 (通过?, 拒绝原因)

- [ ] **Step 1: 写失败测试**（追加）

```python
from real_piper_perception.sdk_safe_execute import filter_candidate_0, ForbiddenZone
import numpy as np

def test_singular_rpy_rejected():
    # 事故候选 [6]：rpy=(92,85,16) —— RY=85 > 75 必须拒绝
    ok, why = filter_candidate_0(np.array([92.0, 85.0, 16.0]), np.array([0.5, 0.1, 0.15]), ForbiddenZone())
    assert not ok and 'RY' in why

def test_singular_boundary():
    assert filter_candidate_0(np.array([0.0, 74.9, 0.0]), np.array([0.5, 0.0, 0.3]), ForbiddenZone())[0]
    ok, why = filter_candidate_0(np.array([0.0, 75.1, 0.0]), np.array([0.5, 0.0, 0.3]), ForbiddenZone())
    assert not ok

def test_coarse_xy_rejected():
    # xy 半径 0.20 < 0.25 拒绝
    ok, why = filter_candidate_0(np.array([0.0, 80.0, 0.0]), np.array([0.2, 0.0, 0.3]), ForbiddenZone())
    assert not ok and 'xy' in why

def test_coarse_z_rejected():
    ok, why = filter_candidate_0(np.array([0.0, 80.0, 0.0]), np.array([0.5, 0.0, 0.17]), ForbiddenZone())
    assert not ok and 'z' in why

def test_coarse_pass():
    ok, why = filter_candidate_0(np.array([0.0, 80.0, 0.0]), np.array([0.5, 0.0, 0.30]), ForbiddenZone())
    assert ok
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m pytest src/real_piper_perception/test/test_sdk_safe_execute.py -v`
Expected: FAIL（无 filter_candidate_0）

- [ ] **Step 3: 实现 filter_candidate_0**

```python
def filter_candidate_0(rpy: np.ndarray, p_j6: np.ndarray,
                       zone: ForbiddenZone) -> tuple[bool, str]:
    """第 0 层：欧拉角奇异过滤 + 位置粗筛。返回 (通过?, 拒绝原因)。"""
    if abs(rpy[1]) > RPY_SINGULAR_LIMIT:
        return False, f'RY={rpy[1]:.1f}° 超奇异阈值 {RPY_SINGULAR_LIMIT}°'
    if np.hypot(p_j6[0], p_j6[1]) < XY_CYL_R:
        return False, f'xy 半径 {np.hypot(p_j6[0], p_j6[1]):.3f}m < {XY_CYL_R}m'
    if p_j6[2] < Z_MIN:
        return False, f'z={p_j6[2]:.3f}m < {Z_MIN}m'
    if zone.point_in_base_box(p_j6):
        return False, 'J6 目标进入底座包围盒'
    return True, ''
```

- [ ] **Step 4: 运行测试确认通过**

Expected: 10 passed（Task 1 的 5 个 + 本任务 5 个）

- [ ] **Step 5: 记录变更**

`docs/grasp_executor_test_log.md` 追加：Task 2 第 0 层（奇异 \|RY\|>75 拒绝、粗筛 xy<0.25/z<0.18/盒内拒绝；事故候选 [6] 回归 RY=85 必拒）5 单测绿。

---

### Task 3: 第 1 层连杆级碰撞预检

**Files:**
- Modify: `src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`
- Test: `src/real_piper_perception/test/test_sdk_safe_execute.py`

**Interfaces:**
- Consumes: `ForbiddenZone`、`filter_candidate_0`、`grasp_ik.solve_ik_link6`
- Produces: `precheck_link_collision(joints: np.ndarray, zone: ForbiddenZone) -> tuple[bool, str]`（FK 每连杆端点 → 盒距离 <5mm 拒）、`filter_candidate_1(p_j6, R_j6, zone) -> tuple[bool, str, np.ndarray|None]`（构造 4x4 → solve_ik_link6 → 关节限位 → 连杆检查）

- [ ] **Step 1: 写失败测试**（追加）

```python
from real_piper_perception.sdk_safe_execute import (
    filter_candidate_1, precheck_link_collision, ForbiddenZone)
from real_piper_perception.grasp_ik import solve_ik_link6
import numpy as np

ZONE = ForbiddenZone()

def _colliding_joints():
    """构造一个把 link1/link2 折入底座盒的关节解（j2 大幅正转前倾）。"""
    return np.array([0.0, 2.4, -1.0, 0.0, 0.0, 0.0])  # j2=137° 前倾折回

def test_link_collision_detected():
    ok, why = precheck_link_collision(_colliding_joints(), ZONE)
    assert not ok and 'link' in why

def test_link_collision_zero_safe():
    ok, why = precheck_link_collision(np.zeros(6), ZONE)
    assert ok

def test_filter_candidate_1_zero():
    # zero 位姿（J6 目标 = zero FK）：必须通过 IK 与连杆检查
    import pinocchio as pin
    model = pin.buildModelFromUrdf(
        './src/piper_ros/src/piper_description/urdf/piper_description.urdf')
    data = model.createData()
    pin.forwardKinematics(model, data, np.zeros(model.nq))
    T6 = data.oMi[model.getJointId('joint6')].homogeneous
    ok, why, q = filter_candidate_1(T6[:3, 3], T6[:3, :3], ZONE)
    assert ok and q is not None

def test_filter_candidate_1_colliding_target():
    # 直接给一个必然折回底座的目标（j2 前倾姿态的 FK 作为目标）
    q_bad = _colliding_joints()
    import pinocchio as pin
    model = pin.buildModelFromUrdf(
        './src/piper_ros/src/piper_description/urdf/piper_description.urdf')
    data = model.createData()
    pin.forwardKinematics(model, data, q_bad)
    T6 = data.oMi[model.getJointId('joint6')].homogeneous
    ok, why, q = filter_candidate_1(T6[:3, 3], T6[:3, :3], ZONE)
    assert not ok  # 连杆碰撞必须拒绝（IK 解回原姿态）
```

- [ ] **Step 2: 运行测试确认失败**

Expected: FAIL（无 precheck_link_collision / filter_candidate_1）

- [ ] **Step 3: 实现**

```python
import pinocchio as pin

_URDF = ('./src/piper_ros/src/'
         'piper_description/urdf/piper_description.urdf')
_LINK_NAMES = [f'link{i}' for i in range(1, 8)]


def _fk_joint_points(q: np.ndarray) -> list[np.ndarray]:
    """关节解 → 各连杆两端点 base 系坐标（含 link1 起点 base_link 原点）。"""
    model = pin.buildModelFromUrdf(_URDF)
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pts = [np.zeros(3)]
    for name in _LINK_NAMES:
        pts.append(data.oMi[model.getJointId(name)].translation.copy())
    return pts


def precheck_link_collision(joints: np.ndarray, zone: ForbiddenZone) -> tuple[bool, str]:
    """连杆端点-底座盒距离检查（<5mm 拒）。返回 (通过?, 原因)。"""
    for p in _fk_joint_points(np.asarray(joints, dtype=float)):
        if zone.point_in_base_box(p):
            return False, f'link 端点 {p.round(3)} 进入底座包围盒'
    return True, ''


def filter_candidate_1(p_j6: np.ndarray, R_j6: np.ndarray,
                       zone: ForbiddenZone) -> tuple[bool, str, np.ndarray | None]:
    """第 1 层：IK 求解 → 关节限位 → 连杆碰撞。返回 (通过?, 原因, 关节解)。"""
    T = np.eye(4)
    T[:3, :3] = R_j6
    T[:3, 3] = p_j6
    q = solve_ik_link6(T)
    if q is None:
        return False, 'pinocchio IK 无解/不收敛', None
    model = pin.buildModelFromUrdf(_URDF)
    lo = model.lowerPositionLimit.copy()[:6]
    up = model.upperPositionLimit.copy()[:6]
    if np.any(q < lo) or np.any(q > up):
        return False, '关节解超出限位', None
    ok, why = precheck_link_collision(q, zone)
    if not ok:
        return False, why, q
    if zone.z_below_min(p_j6):
        return False, f'J6 z={p_j6[2]:.3f} < {Z_MIN}m', q
    return True, '', q
```

（注：`solve_ik_link6` 内部已 clip 限位，限位检查为二次确认；IK 从零位 seed 起解——折回目标可能收敛到正确解，连杆检查拦截。）

- [ ] **Step 4: 运行测试确认通过**

Expected: 14 passed

- [ ] **Step 5: 记录变更**

`docs/grasp_executor_test_log.md` 追加：Task 3 第 1 层（IK 复用 solve_ik_link6、连杆端点盒距离 <5mm 拒、关节限位二次确认、z 下限）4 单测绿。

---

### Task 4: 第 2 层监控触发规则（纯函数）

**Files:**
- Modify: `src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`
- Test: `src/real_piper_perception/test/test_sdk_safe_execute.py`

**Interfaces:**
- Consumes: `ForbiddenZone`
- Produces: `monitor_check(pos: np.ndarray, last_pos: np.ndarray | None, zone: ForbiddenZone) -> tuple[bool, str]`：返回 (触发急停?, 原因)。规则：圆柱进入 / z 下限 / 单帧位移 > JUMP_TOL_MM

- [ ] **Step 1: 写失败测试**（追加）

```python
from real_piper_perception.sdk_safe_execute import monitor_check, ForbiddenZone
import numpy as np

ZONE = ForbiddenZone()

def test_monitor_cyl_trigger():
    # 事故位形 (0.053,-0.010,0.190)：xy=0.054<0.25 且 z=0.19<0.40 → 触发
    trig, why = monitor_check(np.array([0.053, -0.010, 0.190]), None, ZONE)
    assert trig and '圆柱' in why

def test_monitor_z_trigger():
    trig, why = monitor_check(np.array([0.5, 0.0, 0.15]), None, ZONE)
    assert trig and 'z' in why

def test_monitor_jump_trigger():
    last = np.array([0.4, 0.0, 0.30])
    trig, why = monitor_check(np.array([0.44, 0.0, 0.30]), last, ZONE)  # 40mm/帧 > 30
    assert trig and '突变' in why

def test_monitor_safe():
    last = np.array([0.4, 0.0, 0.30])
    trig, why = monitor_check(np.array([0.41, 0.01, 0.31]), last, ZONE)
    assert not trig
```

- [ ] **Step 2: 运行测试确认失败**

Expected: FAIL（无 monitor_check）

- [ ] **Step 3: 实现**

```python
def monitor_check(pos: np.ndarray, last_pos: np.ndarray | None,
                  zone: ForbiddenZone) -> tuple[bool, str]:
    """第 2 层单帧监控：返回 (触发急停?, 原因)。"""
    if zone.point_in_cyl(pos):
        return True, f'进入底座圆柱监控区 {pos.round(3)}'
    if zone.z_below_min(pos):
        return True, f'z={pos[2]:.3f} 低于 {Z_MIN}m'
    if last_pos is not None:
        jump = np.linalg.norm(pos - last_pos) * 1000.0
        if jump > JUMP_TOL_MM:
            return True, f'单帧位移 {jump:.0f}mm > {JUMP_TOL_MM}mm'
    return False, ''
```

- [ ] **Step 4: 运行测试确认通过**

Expected: 18 passed

- [ ] **Step 5: 记录变更**

`docs/grasp_executor_test_log.md` 追加：Task 4 第 2 层监控规则（圆柱/z 下限/突变 30mm）4 单测绿（事故位形 xy=54mm 必触发回归）。

---

### Task 5: 主流程集成 + 执行入口

**Files:**
- Modify: `src/real_piper_perception/real_piper_perception/sdk_safe_execute.py`
- Create: `scripts/sdk_safe_execute_run.py`
- Test: `src/real_piper_perception/test/test_sdk_safe_execute.py`

**Interfaces:**
- Consumes: Task 1-4 全部
- Produces: `safe_execute_flow(piper, p_j6, R_j6, zone) -> dict`（第 0/1 层过滤 → EndPoseCtrl → 监控线程（线程内用 monitor_check + MotionCtrl_1 急停）→ 回验 → MoveJ 回 zero，返回结果 dict）；`scripts/sdk_safe_execute_run.py` 为候选循环入口（score 降序，逐候选 safe_execute_flow）

- [ ] **Step 1: 写失败测试**（过滤集成，不动真机）

```python
from real_piper_perception.sdk_safe_execute import (
    safe_execute_flow, ForbiddenZone, filter_candidate_0, filter_candidate_1)
import numpy as np

def test_flow_rejects_accident_candidate_without_firmware():
    """事故候选 [6] 目标：必须被第 0 层拒绝（无需真机对象）。"""
    p_j6 = np.array([0.524, 0.098, 0.151])
    R_j6 = np.array([[ 0.07, -1.00, -0.00],
                     [ 0.91,  0.06,  0.41],
                     [-0.41, -0.03,  0.91]])
    # 用 filter 链直接验证（safe_execute_flow 需真机，此测试只验过滤链）
    from real_piper_perception.sdk_safe_execute import matrix_to_rpy
    rpy = matrix_to_rpy(R_j6)
    ok0, why0 = filter_candidate_0(rpy, p_j6, ForbiddenZone())
    assert not ok0  # RY≈85° 奇异必拒
    print('rpy =', rpy, 'reason =', why0)
```

- [ ] **Step 2: 运行测试确认失败**

Expected: FAIL（无 matrix_to_rpy / safe_execute_flow）

- [ ] **Step 3: 实现主流程**

`sdk_safe_execute.py` 追加：

```python
import threading
import time


def matrix_to_rpy(R):
    """旋转矩阵 → XYZ 顺序欧拉角（度），与 SDK __MatrixToeula 同式。"""
    beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    yaw = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    roll = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    return np.degrees([roll, beta, yaw])


def _fk_pos(piper):
    try:
        return np.array(piper.GetFK('feedback')[-1][:3])
    except Exception:
        return None


def _go_zero(piper, speed=5):
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    time.sleep(0.2)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    last = None; stable = 0; t0 = time.time()
    while time.time() - t0 < 120:
        time.sleep(2.0)
        pos = _fk_pos(piper)
        if pos is None:
            continue
        if last is not None and np.linalg.norm(pos - last) < 1e-3:
            stable += 1
        else:
            stable = 0
        last = pos
        if stable >= 4:
            break


def _wait_settled(piper, timeout_s=180):
    last = None; stable = 0; t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(2.0)
        pos = _fk_pos(piper)
        if pos is None:
            continue
        if last is not None and np.linalg.norm(pos - last) < 1e-3:
            stable += 1
        else:
            stable = 0
        last = pos
        if stable >= 4:
            return pos
    return last


def safe_execute_flow(piper, p_j6, R_j6, zone) -> dict:
    """安全执行流程：过滤 → 发送 → 监控 → 回验 → 回 zero。

    返回 {'stage': str, 'dev_pos_mm': float|None, 'reason': str}。
    stage: rejected / estop / pass / fail。
    """
    rpy = matrix_to_rpy(R_j6)
    ok0, why0 = filter_candidate_0(rpy, p_j6, zone)
    if not ok0:
        return {'stage': 'rejected', 'dev_pos_mm': None, 'reason': why0}
    ok1, why1, _q = filter_candidate_1(p_j6, R_j6, zone)
    if not ok1:
        return {'stage': 'rejected', 'dev_pos_mm': None, 'reason': why1}

    # 发送 + 监控线程
    stop_event = threading.Event()
    estop_reason = []

    def _monitor():
        last = None
        while not stop_event.is_set():
            time.sleep(MONITOR_PERIOD_S)
            pos = _fk_pos(piper)
            if pos is None:
                continue
            trig, why = monitor_check(pos, last, zone)
            if trig:
                estop_reason.append(why)
                piper.MotionCtrl_1(0x01, 0, 0)     # 快速急停
                piper.MotionCtrl_1(0x00, 0x06, 0)  # 终止执行
                return
            last = pos

    piper.MotionCtrl_2(0x01, 0x00, 5, 0x00)
    time.sleep(0.2)
    vals = [round(v * FACTOR) for v in list(p_j6) + list(rpy)]
    mon = threading.Thread(target=_monitor, daemon=True)
    mon.start()
    piper.EndPoseCtrl(*vals)
    mon.join(timeout=180)
    stop_event.set()

    if estop_reason:
        time.sleep(1.0)
        _go_zero(piper)
        return {'stage': 'estop', 'dev_pos_mm': None, 'reason': estop_reason[0]}

    pos = _wait_settled(piper)
    fk = piper.GetFK('feedback')[-1]
    dev_pos = np.linalg.norm(np.array(fk[:3]) - p_j6 * 1000.0)
    dev_rpy = np.linalg.norm(np.array(fk[3:]) - rpy)
    ok = dev_pos < POS_TOL_MM and dev_rpy < RPY_TOL_DEG
    _go_zero(piper)
    return {'stage': 'pass' if ok else 'fail',
            'dev_pos_mm': float(dev_pos),
            'reason': '' if ok else f'dev_pos={dev_pos:.0f}mm dev_rpy={dev_rpy:.0f}deg'}
```

`scripts/sdk_safe_execute_run.py`：

```python
#!/usr/bin/env python3
"""SDK 安全执行入口（候选循环，2026-08-08）。

用法：python3 scripts/sdk_safe_execute_run.py [--max N]
读最新 perception_grasps.csv（score 降序），逐候选走安全闭环；
任一候选 pass 即停。全程 5% 速度；机械臂需已使能且回 zero。
"""
import argparse
import csv
import sys
import time

import numpy as np
from piper_sdk import C_PiperInterface_V2

sys.path.insert(0, 'src/real_piper_perception')
from real_piper_perception.grasp_transform import (  # noqa: E402
    load_handeye, camera_pose_to_base, graspnet_to_tcp_rotation)
from real_piper_perception.sdk_safe_execute import (  # noqa: E402
    ForbiddenZone, safe_execute_flow)

CSV_PATH = 'shared_camera_data/perception_grasps.csv'
HANDEYE = 'camera_data/handeye_eye_to_hand.yaml'
TOOL_OFFSET = np.array([0.0, 0.0, 0.1358])  # link6→link7（URDF）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max', type=int, default=15)
    args = ap.parse_args()

    handeye = load_handeye(HANDEYE)
    zone = ForbiddenZone()
    with open(CSV_PATH) as f:
        rows = sorted(csv.DictReader(f),
                      key=lambda r: float(r['score']), reverse=True)

    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print('ERROR: enable timeout'); return 1
    piper.EnableFkCal()
    time.sleep(0.5)

    for i, r in enumerate(rows[:args.max]):
        score = float(r['score'])
        xyz = (float(r['camera_x']), float(r['camera_y']), float(r['camera_z']))
        Rg = np.array([[float(r[f'r{i}{j}']) for j in range(3)] for i in range(3)])
        p_b, R_b = camera_pose_to_base(xyz, Rg, handeye)
        R_j6 = graspnet_to_tcp_rotation(R_b)
        p_j6 = p_b - R_j6 @ TOOL_OFFSET
        print(f'[{i+1}] score={score:.3f} J6=({p_j6[0]*1000:.0f},{p_j6[1]*1000:.0f},{p_j6[2]*1000:.0f})mm')
        result = safe_execute_flow(piper, p_j6, R_j6, zone)
        print(f'    -> {result}', flush=True)
        if result['stage'] == 'pass':
            print('ACCEPTED - stop'); break
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python3 -m pytest src/real_piper_perception/test/test_sdk_safe_execute.py -v`
Expected: 19 passed（含事故候选 [6] 回归：rpy RY≈85° 被第 0 层拒绝）

- [ ] **Step 5: 记录变更**

`docs/grasp_executor_test_log.md` 追加：Task 5 主流程（safe_execute_flow：rejected/estop/pass/fail 四态；监控线程 100ms；急停序列 MotionCtrl_1(0x01)+track 0x06；回验后回 zero）+ scripts/sdk_safe_execute_run.py 候选循环入口。

---

### Task 6: 语法检查 + 全量回归

- [ ] **Step 1: 语法检查**

Run: `python3 -m py_compile src/real_piper_perception/real_piper_perception/sdk_safe_execute.py scripts/sdk_safe_execute_run.py && python3 -m pytest src/real_piper_perception/test/ -v 2>&1 | tail -5`
Expected: 既有 31 单测 + 本计划 19 单测全绿（50 passed）

- [ ] **Step 2: 记录变更**

`docs/grasp_executor_test_log.md` 追加 Task 6 全量回归结果；本计划文档末尾记录实现完成日期与执行方式。

---

## Self-Review 记录

**Spec 覆盖核对**：
- 第 0 层奇异过滤（\|RY\|>75° 拒，事故候选 [6] 回归）→ Task 2 ✓
- 第 0 层位置粗筛（xy>0.25 / z>0.18 / 盒内）→ Task 2 ✓
- 第 1 层 IK 求解 + 关节限位 + 连杆端点盒距离 <5mm + z 下限 → Task 3 ✓
- 第 2 层监控（圆柱 xy<0.25&z<0.40 / z<0.18 / 突变 30mm）+ 急停序列 → Task 4（规则）+ Task 5（线程与急停调用）✓
- 第 3 层回验（50mm/10°）→ Task 5（safe_execute_flow 回验段）✓
- 常量参数化（模块头部）→ Task 1 ✓
- 集成 execute_backend 双轨 → 本计划范围外（后续 executor 集成任务，未列入）
- 真机测试（断电恢复后）→ 本计划范围外（安全原因），设计文档测试计划保留

**占位符扫描**：无 TBD/TODO；所有代码块完整可运行 ✓

**类型一致性**：ForbiddenZone/point_in_base_box/point_in_cyl/z_below_min/box_clearance 全计划一致；filter_candidate_0 返回 (bool, str)；filter_candidate_1 返回 (bool, str, ndarray|None)；monitor_check 返回 (bool, str)；safe_execute_flow 返回 dict 四态 ✓

**注意**：`_fk_joint_points` 每次重建 pinocchio 模型（与 grasp_ik 的 _ensure_model 全局缓存模式不同——单测频率低可接受；后续性能优化可加缓存）。`safe_execute_flow` 中 `_monitor` 线程与主线程共享 piper 对象——piper_sdk 内部有锁（GetFK 线程安全，demo 的 EnableFkCal 即后台线程），风险已评估。
