#!/usr/bin/env python3
"""离线核验 Piper SDK Demo 的单位与当前小夹爪最大开口；不连接 CAN。"""

from __future__ import annotations

import inspect
import json
from pathlib import Path
import re

import piper_sdk
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager

from real_piper_perception.gripper_limits import (
    PIPER_SMALL_GRIPPER_MAX_COMMAND,
    PIPER_SMALL_GRIPPER_MAX_OPENING_M,
    opening_m_to_sdk_command,
)


def build_report() -> dict:
    sdk_root = Path(inspect.getfile(piper_sdk)).resolve().parent
    demo_path = sdk_root / 'demo' / 'V2' / 'piper_ctrl_gripper.py'
    demo_text = demo_path.read_text(encoding='utf-8')
    match = re.search(
        r'range\s*=\s*([0-9.]+)\s*\*\s*1000\s*\*\s*1000', demo_text)
    if match is None:
        raise RuntimeError(f'cannot find meter-to-SDK conversion in {demo_path}')

    demo_opening_m = float(match.group(1))
    sdk_min_m, sdk_max_m = C_PiperParamManager().GetGripperRangeParam()
    inferred_command = opening_m_to_sdk_command(float(sdk_max_m))
    if abs(float(sdk_max_m) - PIPER_SMALL_GRIPPER_MAX_OPENING_M) > 1e-12:
        raise RuntimeError(
            f'project max {PIPER_SMALL_GRIPPER_MAX_OPENING_M} m differs from '
            f'piper_sdk default {sdk_max_m} m')
    if inferred_command != PIPER_SMALL_GRIPPER_MAX_COMMAND:
        raise RuntimeError(
            f'project command {PIPER_SMALL_GRIPPER_MAX_COMMAND} differs from '
            f'inferred SDK command {inferred_command}')

    return {
        'hardware_commanded': False,
        'sdk_root': str(sdk_root),
        'demo_path': str(demo_path),
        'demo_example_opening_m': demo_opening_m,
        'demo_example_command': opening_m_to_sdk_command(demo_opening_m),
        'sdk_default_range_m': [float(sdk_min_m), float(sdk_max_m)],
        'inferred_small_gripper_max_opening_m': float(sdk_max_m),
        'inferred_small_gripper_max_opening_mm': float(sdk_max_m) * 1000.0,
        'inferred_gripper_ctrl_command': inferred_command,
    }


if __name__ == '__main__':
    print(json.dumps(build_report(), ensure_ascii=False, indent=2))
