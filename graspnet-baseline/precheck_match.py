"""预检结果匹配模块（推理侧可视化过滤用，纯 numpy，无 torch/open3d 依赖）。

grasp_executor 预检后把通过的候选（相机系位姿）写 shared_dir/precheck_results.json；
本模块把当前帧 GraspGroup 的位姿与 approved/selected 做几何匹配（位置+角度阈值），
供 render_result 精确定位机械臂最终选中的执行候选。

坐标系：均为相机系（GraspNet 输出系 = CSV camera_x/y/z 系），可直接比较。
匹配阈值：位置 2cm、角度 15°（同一冻结帧场景下同候选位姿应几乎一致，阈值
宽松以容忍推理/CSV 浮点差异；场景变化时匹配失败则执行候选页面不显示抓手）。
"""

import numpy as np
from scipy.spatial.transform import Rotation

POS_THRESH_M = 0.02
# 角度为宽松方向检查：同一抓取位置在冻结帧与新帧间的候选方向可能不同
#（GraspNet 解码方向多样性），位置 2cm 级精确已是强判据。
ANG_THRESH_DEG = 90.0


def _angle_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """两旋转矩阵的夹角（度），用相对旋转的旋转角。"""
    rel = R1.T @ R2
    # 旋转角 = arccos((tr(R)-1)/2)，钳制数值稳定性
    cos_a = float(np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_a)))


def match_precheck(xyz: np.ndarray, R: np.ndarray,
                   approved: list, selected,
                   pos_thresh: float = POS_THRESH_M,
                   ang_thresh: float = ANG_THRESH_DEG):
    """当前帧候选位姿与预检结果匹配。

    Args:
        xyz: (N, 3) 相机系位置（GraspGroup 平移）
        R: (N, 3, 3) 相机系旋转（GraspGroup 旋转）
        approved: [{'xyz': [..], 'R': [[..]]}, ...]（可空）
        selected: {'xyz': .., 'R': ..} 或 None
        pos_thresh / ang_thresh: 匹配阈值（米 / 度）

    Returns:
        (approved_mask, selected_idx)：approved_mask 是 (N,) bool——候选与任一
        approved 位姿匹配即 True；selected_idx 是匹配 selected 的候选下标
        （无则 -1）。selected 匹配优先于 approved 判定（互不冲突）。
    """
    n = len(xyz)
    mask = np.zeros(n, dtype=bool)
    sel_idx = -1
    selected_matches = []

    def _matches(p, ref) -> bool:
        if ref is None:
            return False
        if float(np.linalg.norm(p[0] - np.asarray(ref['xyz']))) > pos_thresh:
            return False
        return _angle_deg(p[1], np.asarray(ref['R'])) <= ang_thresh

    for i in range(n):
        pose = (xyz[i], R[i])
        if any(_matches(pose, ref) for ref in approved):
            mask[i] = True
        if selected is not None and _matches(pose, selected):
            # 宽松阈值内可能有多个近邻；页面必须唯一显示实际执行项，
            # 因此按位置距离、姿态角、当前序号确定唯一最近匹配。
            pos_dist = float(np.linalg.norm(
                pose[0] - np.asarray(selected['xyz'], dtype=np.float64)))
            angle = _angle_deg(pose[1], np.asarray(selected['R'], dtype=np.float64))
            selected_matches.append((pos_dist, angle, i))
    if selected_matches:
        sel_idx = min(selected_matches)[2]
        mask[sel_idx] = True
    return mask, sel_idx
