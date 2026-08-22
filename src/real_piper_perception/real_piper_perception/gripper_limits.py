"""Piper 小夹爪的 SDK 行程单位、最大开口与纯函数门控。

权威链路（piper_sdk 0.6.2）：

* ``demo/V2/piper_ctrl_gripper.py`` 用
  ``0.05 * 1000 * 1000`` 生成 50 mm 的 ``GripperCtrl`` 指令，证明位置字段
  单位是 0.001 mm，即 1 SDK unit = 1e-6 m；
* ``piper_param_manager.py`` 默认 ``gripper_range=[0.0, 0.07]``；
* SDK 的夹爪参数协议注明小夹爪最大控制行程 70 mm、大夹爪 100 mm；
* 当前机器人 URDF/SRDF 是 35 mm/指的小夹爪，总开口为 70 mm。

因此本工程使用的小夹爪最大指尖总开口为 0.070 m，对应 SDK 指令 70000。
该模块不连接 CAN，可安全用于单元测试和离线核验。
"""

from __future__ import annotations

import math


PIPER_SDK_GRIPPER_UNIT_M = 1e-6
PIPER_SMALL_GRIPPER_MAX_OPENING_M = 0.070
PIPER_SMALL_GRIPPER_MAX_COMMAND = 70_000


def opening_m_to_sdk_command(opening_m: float) -> int:
    """米制指尖总开口转换为 ``GripperCtrl`` 的整数位置字段。"""
    opening = float(opening_m)
    if not math.isfinite(opening) or opening < 0.0:
        raise ValueError(f'opening_m must be finite and >= 0, got {opening_m!r}')
    return int(round(opening / PIPER_SDK_GRIPPER_UNIT_M))


def sdk_command_to_opening_m(command: int) -> float:
    """``GripperCtrl`` 位置字段转换为米制指尖总开口。"""
    value = int(command)
    if value < 0:
        raise ValueError(f'command must be >= 0, got {command!r}')
    return float(value) * PIPER_SDK_GRIPPER_UNIT_M


def required_opening_m(grasp_width_m: float, margin_m: float) -> float:
    """GraspNet 预测宽度加安全余量，得到抓取前所需总开口。"""
    width = float(grasp_width_m)
    margin = float(margin_m)
    if not math.isfinite(width) or not math.isfinite(margin):
        raise ValueError('grasp width and margin must be finite')
    return width + margin


def fits_small_gripper(grasp_width_m: float, margin_m: float) -> bool:
    """候选所需开口是否落在当前 70 mm 小夹爪的物理范围内。"""
    try:
        width = float(grasp_width_m)
        margin = float(margin_m)
        opening = required_opening_m(grasp_width_m, margin_m)
    except (TypeError, ValueError):
        return False
    return 0.0 <= width and 0.0 <= margin and 0.0 <= opening <= \
        PIPER_SMALL_GRIPPER_MAX_OPENING_M
