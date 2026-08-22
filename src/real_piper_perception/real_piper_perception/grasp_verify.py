"""夹爪成功判定纯函数（Task 7）。

语义：夹爪按命令闭合量 cmd_width 闭合后，读反馈实际开度 actual_width；
若实际开度 < 命令闭合量 - 检测阈值 ⇒ 物体挡在夹爪行程内（夹住物体）⇒ True；
否则夹爪到达了命令闭合量（抓空）⇒ False。

单位说明（M9 修复，按 C2 修复后调用点事实）：actual_width / cmd_width /
detect_threshold 三者同单位，调用点当前为**米制指尖总开度**：
- cmd_width = grasp_width(米) + gripper_margin(米)——命令总开度；
- actual_width 来自 /joint_states_feedback 的 gripper 关节值——驱动把 sdk
  grippers_angle（"stroke of the gripper (in 0.001 mm)"，piper_sdk 文档）除以
  1e6 发布，即米制总开度（字段名 "angle" 是历史遗留，实为行程长度）。
命令与反馈同为米制总开度，判定无需单位换算（"弧度 1:1"断言与 URDF/驱动
事实矛盾，已删除；joint7 半行程↔总开度的 ×2 映射由 grasp_execute 的命令
换算与驱动 gripper_val_mutiple=2 处理，本函数不涉及）。
"""


def verify_grasp(actual_width: float, cmd_width: float, detect_threshold: float) -> bool:
    """实际开度 < 命令宽度 - 阈值 ⇒ 夹住物体(True)；否则抓空(False)。

    严格小于判定：实际开度恰好等于命令宽度 - 阈值时视为未夹住
    （抓空），阈值边界上保守失败。
    """
    return actual_width < cmd_width - detect_threshold


# ---- 全开/全闭参考增强判定（2026-08-08 用户裁决）----
# 仅比对命令宽度的判定对"夹爪未闭合/异常"场景信息不足；增加全开/全闭
# 参考（每次执行前实测）扩大判定覆盖：
#   not_closed   ≈ 全开 → 夹爪指令未生效/机械故障（判 grasp_empty）
#   closed_empty ≈ 全闭 → 合拢但没夹到（物体太小/掉落，判 grasp_empty）
#   grasped      实际 < 命令 - 阈值 → 夹住物体（success）
#   empty        到达命令宽度 → 抓空

CLOSE_TOL = 0.002  # 全闭判定容差（米）：认定"合拢但没夹到"的边界（物体厚 > 此值不误判）
OPEN_TOL = 0.003   # 全开判定容差（米）：认定"夹爪未闭合"的边界（2026-08-08 从
                   # 0.010 收紧：0.01 会把 6.6mm 的真实闭合动作误标为 not_closed；
                   # 真机实测闭合 70→63.4mm 后反馈差 6.3mm > 3mm 可正确识别）


def classify_grasp(actual_width: float, full_open: float,
                   full_closed: float,
                   detect_threshold: float | None = None) -> tuple[str, str]:
    """全闭方案判定（2026-08-08 用户裁决：闭合命令恒为完全闭合）。

    判定只依赖全开/全闭实测参考，不依赖候选宽度（消除宽度预测误差影响）：
      ≈ full_open                  → not_closed（夹爪没动/指令未生效）
      > full_closed + CLOSE_TOL    → grasped（物体挡在行程内）
      ≈ full_closed                → closed_empty（空载合拢，物体不在行程内）

    Args:
        actual_width: 反馈实际开度（米制总开度）。
        full_open: 每次执行前实测的全开参考（米制总开度）。
        full_closed: 每次执行前实测的空载全闭参考（米制总开度）。
        detect_threshold: 保留参数（兼容旧签名，不再使用——判定与命令宽度无关）。

    Returns:
        (status, detail)：status ∈ {'success', 'grasp_empty'}；
        detail ∈ {'grasped', 'not_closed', 'closed_empty'}。
    """
    if actual_width > full_open - OPEN_TOL:
        return 'grasp_empty', 'not_closed'      # 夹爪未闭合（≈全开）
    if actual_width > full_closed + CLOSE_TOL:
        return 'success', 'grasped'             # 物体挡在行程内（开度 > 全闭）
    return 'grasp_empty', 'closed_empty'        # 空载合拢（≈全闭）
