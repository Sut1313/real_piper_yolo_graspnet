"""grasp_executor 坐标变换模块（Task 3）。

唯一依据：docs/grasp_executor_transform_notes.md（公式链 §1、常量表 §2、handeye 格式 §4）
与 .superpowers/sdd/grasp-executor/task-1-report.md（疑点清单）。

变换链（真机，eye-to-hand 标定）：
    p_base     = R_cam2base @ p_camera + t_cam2base                 （位置，§1.2）
    R_base_tcp = R_cam2base @ R_grasp_cam @ GRASPNET_TO_PIPER_TCP_R （姿态，§1.3）

两步分离：camera_pose_to_base 只做相机系→基座系（输出仍为 GraspNet 系旋转），
TCP 轴映射由 graspnet_to_tcp_rotation 单独完成（乘法顺序：外参左乘、轴映射右乘）。

2026-08-21 真机 plan-only A/B 复测裁决：GraspNet 局部 +X 是接近/进深轴，
Piper TCP 局部 +Z 是工具前进轴，因此 TCP +Z 必须跟随 GraspNet +X；TCP +Y
保持 GraspNet 的两指宽度轴。执行侧与 Docker 推理侧使用同一矩阵。
"""

import logging

import numpy as np
import yaml

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 模块级常量（数值从调研文档 §2 常量表原样拷贝）
# ---------------------------------------------------------------------------

# GraspNet 抓取系 → Piper TCP 系轴映射（2026-08-21 A/B 裁决）。
# 列语义：
#   TCP +Z（工具前进） = GraspNet +X（接近/进深）
#   TCP +Y（开合/宽度）= GraspNet +Y（两指宽度）
#   TCP +X             = -GraspNet +Z
# det=+1。
GRASPNET_TO_PIPER_TCP_R: np.ndarray = np.array([
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)

# handeye 缺失时的回退仿真常量（调研文档 §2.1，仿真 grasp_common.hpp 同值）：
#   CAMERA_TO_WORLD_R          —— 位置变换用（中间行 -1.0，det=+1，真旋转）
#   CAMERA_TO_WORLD_ROTATION_R —— 姿态变换专用（中间行 +1.0，det=-1 镜像，仅仿真执行侧用法）
#   CAMERA_TO_WORLD_T          —— 平移（米）
# 注意（疑点 C）：两矩阵互为镜像、不可混用；位置恒用 PositionR，姿态恒用 RotationR。
CAMERA_TO_WORLD_R: np.ndarray = np.array([
    [0.0, -0.82806033, 0.56063899],
    [-1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_ROTATION_R: np.ndarray = np.array([
    [0.0, -0.82806033, 0.56063899],
    [1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_T: np.ndarray = np.array([0.072, 0.021, 0.548], dtype=np.float64)

# 回退矩阵选取说明：load_handeye 契约返回单个刚体变换 T_base_camera（camera→base）。
# 仿真侧位置/姿态本用两个不同矩阵，回退时只能取其一——此处取 CAMERA_TO_WORLD_R
# （PositionR，det=+1 真旋转）：它是两矩阵中唯一能构成刚体变换（4x4 齐次）的一个，
# 可同时安全用于位置与旋转两条路径；RotationR（det=-1 镜像）不是合法刚体变换，
# 用于位置路径会产生镜像错误。回退是降级模式，一律打印告警。


def load_handeye(path: str) -> np.ndarray:
    """解析 eye-to-hand 标定 yaml，返回 4x4 齐次变换 T_base_camera（camera→base）。

    格式按调研文档 §4：yaml 含 3x3 旋转矩阵 ``R_cam2base``（det=+1 正交）
    与米制平移 ``t_cam2base``（另有冗余 4x4 ``T_cam2base`` 与 R/t 一致，本函数不依赖）。
    语义：eye-to-hand，给出 camera→base（``p_base = T_cam2base @ p_cam``）。

    文件缺失 / 解析失败 / 字段不完整：按调研文档 §4 回退策略，回退仿真常量
    （位置矩阵 CAMERA_TO_WORLD_R + CAMERA_TO_WORLD_T，理由见模块注释）并打印告警。

    Args:
        path: handeye yaml 路径。

    Returns:
        (4, 4) float64 齐次变换矩阵，末行为 [0, 0, 0, 1]。
    """
    try:
        with open(path) as f:
            d = yaml.safe_load(f)
        R = np.asarray(d['R_cam2base'], dtype=np.float64).reshape(3, 3)
        t = np.asarray(d['t_cam2base'], dtype=np.float64).reshape(3)
    except Exception as exc:
        logger.warning(
            'handeye 标定文件加载失败（%s），回退仿真常量 CAMERA_TO_WORLD_R/T（0.072, 0.021, 0.548）'
            '—— 降级模式，结果不可靠：%s', path, exc)
        R = CAMERA_TO_WORLD_R
        t = CAMERA_TO_WORLD_T

    # 校验（调研文档 §4 可选校验）：det 应接近 +1（真旋转）
    det = np.linalg.det(R)
    if not np.isclose(det, 1.0, atol=1e-6):
        logger.warning('handeye 标定旋转矩阵 det=%.8f，非正交归一真旋转（期望 +1），请检查标定文件', det)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def camera_pose_to_base(xyz: tuple[float, float, float],
                        r: np.ndarray | tuple[float, ...],
                        handeye: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """相机系 (位置, 旋转) → 基座系。

    公式按调研文档 §1.2/§1.3（T_base_camera 左乘）：
        位置：p_base = R_cam2base @ p_camera + t_cam2base
        旋转：R_base = R_cam2base @ R_grasp_cam

    注意：旋转输出只到基座系（仍是 GraspNet 系），TCP 轴映射由
    graspnet_to_tcp_rotation 单独完成——两步分离，与调研文档公式一致。

    Args:
        xyz: 相机系抓取中心坐标 (x, y, z)，单位米。
        r:   相机系 GraspNet 旋转，3x3 矩阵或 9 元素（行主序）序列。
        handeye: load_handeye 返回的 4x4 T_base_camera。

    Returns:
        (pos, rot)：基座系位置 (3,) 与基座系旋转 (3, 3)。
    """
    R_cam2base = handeye[:3, :3]
    t_cam2base = handeye[:3, 3]
    p_camera = np.asarray(xyz, dtype=np.float64).reshape(3)
    R_grasp_cam = np.asarray(r, dtype=np.float64).reshape(3, 3)
    pos = R_cam2base @ p_camera + t_cam2base
    rot = R_cam2base @ R_grasp_cam
    return pos, rot


def graspnet_to_tcp_rotation(r_base: np.ndarray) -> np.ndarray:
    """应用 TCP 轴映射：R_tcp = r_base @ GRASPNET_TO_PIPER_TCP_R（右乘，§1.3）。

    基座系 GraspNet 旋转 → 基座系 TCP 旋转。矩阵采用 2026-08-21 两轮
    plan-only A/B 验证后的 approach/width 语义，并与 Docker 推理侧一致。

    Args:
        r_base: 基座系 GraspNet 旋转 (3, 3)。

    Returns:
        (3, 3) 基座系 TCP 旋转。
    """
    return np.asarray(r_base, dtype=np.float64) @ GRASPNET_TO_PIPER_TCP_R
