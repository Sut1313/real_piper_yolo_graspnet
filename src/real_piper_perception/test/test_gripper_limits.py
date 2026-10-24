"""Piper SDK Demo 单位与夹爪开度换算测试。"""

from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager

from real_piper_perception.grasp_execute import GraspExecutor
from real_piper_perception.gripper_limits import (
    PIPER_SMALL_GRIPPER_MAX_COMMAND,
    PIPER_SMALL_GRIPPER_MAX_OPENING_M,
    fits_small_gripper,
    opening_m_to_sdk_command,
    required_opening_m,
    sdk_command_to_opening_m,
)


def test_sdk_demo_50mm_conversion_formula():
    # piper_sdk demo: range = 0.05 * 1000 * 1000
    assert opening_m_to_sdk_command(0.05) == 50_000
    assert abs(sdk_command_to_opening_m(50_000) - 0.05) < 1e-12


def test_sdk_default_small_gripper_max_is_70mm():
    assert C_PiperParamManager().GetGripperRangeParam() == (0.0, 0.07)
    assert PIPER_SMALL_GRIPPER_MAX_OPENING_M == 0.070
    assert PIPER_SMALL_GRIPPER_MAX_COMMAND == 70_000
    assert opening_m_to_sdk_command(PIPER_SMALL_GRIPPER_MAX_OPENING_M) == 70_000


def test_required_opening_and_small_gripper_fit_gate():
    assert abs(required_opening_m(0.100, 0.005) - 0.105) < 1e-12
    assert fits_small_gripper(0.065, 0.005)
    assert not fits_small_gripper(0.066, 0.005)


class _NoHardware:
    def __init__(self):
        self.calls = []

    def GripperCtrl(self, *args):
        self.calls.append(args)


def test_execute_pick_rejects_overwidth_before_hardware_command():
    executor = GraspExecutor(node=None, move_group=None, params={})
    fake = _NoHardware()
    executor._piper = fake
    poses = {
        'approach': {'pos': [0, 0, 0], 'quat': [1, 0, 0, 0]},
        'grasp': {'pos': [0, 0, 0], 'quat': [1, 0, 0, 0]},
        'lift': {'pos': [0, 0, 0], 'quat': [1, 0, 0, 0]},
    }
    result = executor.execute_pick(poses, grasp_width=0.066)
    assert result['stage'] == 'gripper_width'
    assert abs(result['cmd_width'] - 0.071) < 1e-12
    assert fake.calls == []
