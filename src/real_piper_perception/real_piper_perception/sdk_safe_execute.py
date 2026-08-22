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
        lo = self._lo + BOX_BUFFER  # 原盒面 = 缓冲盒去掉 BOX_BUFFER（self._lo 已含缓冲）
        hi = self._hi - BOX_BUFFER
        d = np.maximum(np.maximum(lo - p, 0.0), np.maximum(p - hi, 0.0))
        return float(np.linalg.norm(d)) * (-1.0 if self.point_in_base_box(p) else 1.0)
