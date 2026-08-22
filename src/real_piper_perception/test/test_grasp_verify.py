"""Task 7 夹爪成功判定纯函数 TDD 单测（不依赖 MoveIt / ROS 节点）。

语义：实际开度 < 命令宽度 - 阈值 ⇒ 夹住物体(True)；否则抓空(False)。
"""

from real_piper_perception.grasp_verify import verify_grasp


def test_grasp_success_when_blocked():
    # 命令闭合 60，实际只到 25 → 夹住物体
    assert verify_grasp(actual_width=25.0, cmd_width=60.0, detect_threshold=10.0)


def test_grasp_empty_when_full_close():
    # 实际接近命令值 → 抓空
    assert not verify_grasp(actual_width=58.0, cmd_width=60.0, detect_threshold=10.0)


def test_edge_threshold():
    # 边界：恰好等于 cmd - threshold 视为未夹住（严格小于）
    assert not verify_grasp(actual_width=50.0, cmd_width=60.0, detect_threshold=10.0)
    assert verify_grasp(actual_width=49.9, cmd_width=60.0, detect_threshold=10.0)


# ---- classify_grasp：全闭方案判定（2026-08-08 用户裁决）----
# 闭合命令恒为完全闭合；判定基于全开/全闭实测参考（不依赖候选宽度）：
#   ≈ full_open                → not_closed（夹爪没动）
#   > full_closed + CLOSE_TOL  → grasped（物体挡在行程内）
#   ≈ full_closed              → closed_empty（空载合拢）

from real_piper_perception.grasp_verify import classify_grasp

_FULL_OPEN = 0.0697   # 2026-08-07 标定（每次执行前实测）
_FULL_CLOSED = 0.0043


def test_classify_grasped():
    # 夹住 5cm 物体：实际 0.05 远超全闭 0.0043
    assert classify_grasp(0.050, _FULL_OPEN, _FULL_CLOSED) == ('success', 'grasped')


def test_classify_not_closed():
    # 夹爪没动：反馈仍接近全开（差 < OPEN_TOL=3mm）
    assert classify_grasp(0.0685, _FULL_OPEN, _FULL_CLOSED) == ('grasp_empty', 'not_closed')


def test_classify_closed_empty():
    # 空载合拢：反馈 ≈ 全闭
    assert classify_grasp(0.0045, _FULL_OPEN, _FULL_CLOSED) == ('grasp_empty', 'closed_empty')


def test_classify_thin_object_grasped():
    # 夹住 8mm 物体（0.008 > 全闭容差边界 0.0063）：不误判 closed_empty
    assert classify_grasp(0.008, _FULL_OPEN, _FULL_CLOSED) == ('success', 'grasped')


def test_classify_full_open_after_close_cmd():
    # 真机场景：闭合命令发出后反馈仍 ≈ 全开（夹爪未动/指令未生效）
    assert classify_grasp(0.0690, _FULL_OPEN, _FULL_CLOSED) == ('grasp_empty', 'not_closed')


def test_classify_boundary_strict_greater():
    # 严格大于：actual = full_closed + CLOSE_TOL 恰等 → closed_empty；+0.1mm → grasped
    assert classify_grasp(0.0063, _FULL_OPEN, _FULL_CLOSED) == ('grasp_empty', 'closed_empty')
    assert classify_grasp(0.0064, _FULL_OPEN, _FULL_CLOSED) == ('success', 'grasped')


def test_classify_legacy_signature_compat():
    # 旧签名（带 cmd_width/threshold）仍可调用（参数被忽略，判定与命令宽度无关）
    assert classify_grasp(0.050, _FULL_OPEN, _FULL_CLOSED, 0.01) == ('success', 'grasped')
