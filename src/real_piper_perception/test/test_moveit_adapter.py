"""MoveIt Humble GetCartesianPath 响应契约回归。"""

from types import SimpleNamespace

import pytest
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from real_piper_perception.moveit_adapter import MoveGroupInterface


class _DoneFuture:
    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def result(self):
        return self._response


class _CartesianService:
    def __init__(self, response):
        self.response = response
        self.request = None

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.request = request
        return _DoneFuture(self.response)


def _interface(response):
    interface = MoveGroupInterface.__new__(MoveGroupInterface)
    interface._group_name = 'arm'
    interface._end_effector_link = 'link6'
    interface._start_state = None
    interface._cart_service = _CartesianService(response)
    return interface


def _response(error=MoveItErrorCodes.SUCCESS, fraction=1.0, with_point=True):
    solution = RobotTrajectory()
    if with_point:
        solution.joint_trajectory.points.append(JointTrajectoryPoint())
    return SimpleNamespace(
        solution=solution,
        fraction=fraction,
        error_code=SimpleNamespace(val=error),
    )


def test_cartesian_path_uses_humble_solution_field():
    interface = _interface(_response())
    plan, fraction = interface.compute_cartesian_path([])
    assert plan.trajectory is interface._cart_service.response.solution
    assert fraction == 1.0
    assert interface._cart_service.request.max_step == 0.01


def test_cartesian_path_rejects_error_code():
    interface = _interface(_response(error=MoveItErrorCodes.PLANNING_FAILED))
    with pytest.raises(RuntimeError, match='error_code=-1'):
        interface.compute_cartesian_path([])


def test_cartesian_path_rejects_empty_trajectory():
    interface = _interface(_response(with_point=False))
    with pytest.raises(RuntimeError, match='空轨迹'):
        interface.compute_cartesian_path([])
