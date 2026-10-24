#!/usr/bin/env python3
"""
inference_from_camera.py — GraspNet 相机连续推理脚本

从共享目录读取相机数据，运行 GraspNet 推理，实时显示抓取结果。

用法（在 Docker 容器内）：
  python inference_from_camera.py \
      --checkpoint_path /workspace/logs/log_rs/checkpoint.tar \
      --shared_dir /shared_camera_data

流程：
  1. 加载 GraspNet 模型
  2. 监听共享目录中的 .ready 标志
  3. 读取 color.png, depth.png, meta.mat
  4. 运行推理（点云采样 → 网络前传 → 抓取解码）
  5. 使用 Open3D 渲染标注抓取姿态的场景
  6. 显示在屏幕上（OpenCV imshow 窗口）
  7. 等待下一帧
"""

import os
import sys
import time
import argparse
import threading
import json
import glob

import numpy as np
import cv2
import open3d as o3d

import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from precheck_match import match_precheck  # 预检结果匹配（宿主机/容器同目录挂载）
from text_overlay import put_text_cn  # 中文叠加（cv2.putText 仅 ASCII）

# ── 参数 ──────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', type=str, default='logs/log_rs/checkpoint.tar',
                    help='Model checkpoint path')
parser.add_argument('--shared_dir', type=str, default='/shared_camera_data',
                    help='Shared camera data directory')
parser.add_argument('--num_point', type=int, default=20000,
                    help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300,
                    help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01,
                    help='Collision Threshold [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01,
                    help='Voxel Size for collision detection [default: 0.01]')
parser.add_argument('--top_k', type=int, default=50,
                    help='Number of top grasps to show [default: 50]')
parser.add_argument('--poll_interval', type=float, default=0.1,
                    help='Poll interval in seconds [default: 0.1]')
parser.add_argument('--max_frames', type=int, default=0,
                    help='Stop after N consumed frames; 0 keeps running')
parser.add_argument('--display_width', type=int, default=1280,
                    help='Display window width [default: 1280]')
parser.add_argument('--display_height', type=int, default=720,
                    help='Display window height [default: 720]')
parser.add_argument('--no_display', action='store_true',
                    help='Run without display (save results only)')
parser.add_argument('--min_object_points', type=int, default=80,
                    help='Minimum points required per colored object')
parser.add_argument('--grasp_object_margin', type=float, default=0.08,
                    help='Allowed margin around object point cloud for grasp validation [default: 0.08]')
parser.add_argument('--candidates_per_object', type=int, default=300,
                    help='Number of complete GraspNet 6D candidates to export per object')
parser.add_argument('--segmentation_mode', choices=['color', 'workspace', 'yolo'], default='color',
                    help=('color=colored objects, workspace=depth workspace, '
                          'yolo=instances.npz masks from real_piper_yolo_graspnet'))
parser.add_argument('--workspace_name', type=str, default='target',
                    help='Object name used when segmentation_mode=workspace')
parser.add_argument('--min_depth_m', type=float, default=0.15,
                    help='Minimum usable depth in meters for workspace mode')
parser.add_argument('--max_depth_m', type=float, default=1.20,
                    help='Maximum usable depth in meters for workspace mode')
parser.add_argument('--roi', type=str, default='',
                    help='Optional workspace ROI as x_min,y_min,x_max,y_max normalized to [0,1]')
parser.add_argument('--save_debug_outputs', action='store_true',
                    help='Save per-frame grasps/PCD files under shared_dir/output')
parser.add_argument('--save_debug_every', type=int, default=10,
                    help='Save one debug output frame every N frames')
parser.add_argument('--max_saved_debug_frames', type=int, default=5,
                    help='Keep only the newest N saved debug frame indices')
cfgs = parser.parse_args()

# ── 全局变量 ──────────────────────────────────────────────────────
running = True
frame_count = 0
inference_times = []
latest_result_image = None
latest_color_image = None
latest_precheck_image = None
display_lock = threading.Lock()

OBJECT_COLOR_RANGES = {
    # 各抓取物体的颜色分割阈值（RGB 归一化到 [0,1]）。
    # 平面（灰色平台）颜色落在所有区间之外，因此会被当作背景过滤掉；
    # 这是"平面过滤"能生效的根本原因（见 process_data / segment_object_points）。
    'red_box': {
        'lower': np.array([0.55, 0.00, 0.00], dtype=np.float32),
        'upper': np.array([1.01, 0.35, 0.35], dtype=np.float32),
    },
    'green_cylinder': {
        'lower': np.array([0.00, 0.45, 0.00], dtype=np.float32),
        'upper': np.array([0.35, 1.01, 0.35], dtype=np.float32),
    },
    'blue_cylinder': {
        'lower': np.array([0.00, 0.00, 0.45], dtype=np.float32),
        'upper': np.array([0.35, 0.35, 1.01], dtype=np.float32),
    },
}

if cfgs.segmentation_mode == 'workspace' and cfgs.workspace_name not in OBJECT_COLOR_RANGES:
    OBJECT_COLOR_RANGES[cfgs.workspace_name] = {
        'lower': np.array([0.0, 0.0, 0.0], dtype=np.float32),
        'upper': np.array([1.01, 1.01, 1.01], dtype=np.float32),
    }

SOURCE_OBJECT_PRIORS_WORLD = {
    # 各物体的世界系源位置先验（物体在 Gazebo world 中的初始摆放位置）。
    # 用于过滤"落点距离物体太远"的候选抓取，降低误抓率；
    # 注意 z=0.124 是物体几何中心高度，抓取点应在物体附近。
    'red_box': np.array([0.45, -0.16, 0.124], dtype=np.float64),
    'green_cylinder': np.array([0.45, 0.02, 0.124], dtype=np.float64),
    'blue_cylinder': np.array([0.45, 0.17, 0.124], dtype=np.float64),
}
SOURCE_PRIOR_MAX_XY_DIST = 0.35  # 放宽源区域过滤，容忍机械臂碰撞导致的物体位移

CAMERA_TO_WORLD_R = np.array([
    # 相机系 → 世界系旋转矩阵（标定得到，55.9° 有效俯仰角）。
    # 注意本矩阵第二行符号与 CAMERA_TO_WORLD_ROTATION_R 不同——
    # 这个版本用于"平移/度量"用途（如距离计算）。
    [0.0, -0.82806033, 0.56063899],
    [-1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
# 姿态转换使用独立的旋转矩阵（保持原始 +1.0 符号，使 IK 可解）
CAMERA_TO_WORLD_ROTATION_R = np.array([
    # 专用于旋转变换的版本：第二行 +1.0（右手系语义），
    # 使转换后的 TCP 姿态能被 Piper IK 求到解（历史标定修复点）。
    [0.0, -0.82806033, 0.56063899],
    [1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_T = np.array([0.072, 0.021, 0.548], dtype=np.float64)
# GraspNet local +X is the gripper depth/approach direction. Piper's TCP uses
# +Z as the tool-forward direction for link6 offset compensation, so +Z must
# follow GraspNet +X; +Y remains the gripper width axis.
GRASPNET_TO_PIPER_TCP_R = np.array([
    # GraspNet 局部系 → Piper TCP 系的轴映射：
    #   GraspNet +X（夹爪进深/接近方向）→ Piper +Z（工具前进方向）
    #   GraspNet +Y（夹爪宽度轴）       → Piper +Y
    #   GraspNet +Z                     → Piper -X
    # 该映射是 C++ 主节点 grasp_frame_transform 的同一份常量的 Python 镜像，
    # 修改时必须两边同步（历史坑：不一致会导致抓取姿态整体旋转错误）。
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)
PIPER_TOOL_Z_OFFSET = 0.0
PIPER_MIN_GRASP_Z = 0.0
PIPER_MIN_LINK6_GRASP_Z = 0.0


# ===================================================================
#  模型加载
# ===================================================================
def get_net():
    """初始化并加载 GraspNet 模型"""
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}", flush=True)

    net = GraspNet(
        input_feature_dim=0,
        num_view=cfgs.num_view,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    net.to(device)

    checkpoint = torch.load(cfgs.checkpoint_path, map_location=device)
    net.load_state_dict(checkpoint['model_state_dict'])
    epoch = checkpoint['epoch']
    print(f"-> loaded checkpoint (epoch: {epoch})", flush=True)
    net.eval()
    return net


# ===================================================================
#  数据读取与处理
# ===================================================================
def read_data_from_shared(shared_dir):
    """从共享目录读取相机数据（color.png / depth.png / meta.mat）。

    调用时机：调用方已确认 .ready 存在（整帧已由 bridge 原子写完），
    因此这里的读取不会遇到半帧；返回 dict 或 (None, 错误信息)。
    """
    color_path = os.path.join(shared_dir, 'color.png')
    depth_path = os.path.join(shared_dir, 'depth.png')
    meta_path = os.path.join(shared_dir, 'meta.mat')

    if not all(os.path.exists(p) for p in [color_path, depth_path, meta_path]):
        return None, "Missing data files"

    try:
        color = cv2.imread(color_path, cv2.IMREAD_COLOR)
        if color is None:
            return None, "Failed to read color.png"
        # bridge 侧约定直接存 BGR（见 camera_bridge_node 的注释），
        # 所以这里必须做 BGR→RGB 转换一次且仅一次——
        # 若 bridge 侧也转了，这里就会转两次，红蓝互换（历史 bug）。
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        color_float = color.astype(np.float32) / 255.0

        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            return None, "Failed to read depth.png"

        import scipy.io as scio
        meta = scio.loadmat(meta_path)
        intrinsic = meta['intrinsic_matrix']
        factor_depth = float(meta['factor_depth'][0][0])

        h, w = depth.shape
        camera = CameraInfo(
            float(w), float(h),
            float(intrinsic[0][0]), float(intrinsic[1][1]),
            float(intrinsic[0][2]), float(intrinsic[1][2]),
            factor_depth,
        )

        result = {
            'color': color_float,
            'depth': depth.astype(np.float32),
            'camera': camera,
            'color_vis': color,
        }
        if cfgs.segmentation_mode == 'yolo':
            instances_path = os.path.join(shared_dir, 'instances.npz')
            manifest_path = os.path.join(shared_dir, 'frame_manifest.json')
            ready_path = os.path.join(shared_dir, '.ready')
            if not all(os.path.exists(path) for path in (
                    instances_path, manifest_path, ready_path)):
                return None, "Missing YOLO instances/manifest/ready files"
            with open(manifest_path, 'r') as stream:
                manifest = json.load(stream)
            with open(ready_path, 'r') as stream:
                ready_sequence = stream.read().strip()
            manifest_sequence = str(manifest.get('frame_sequence', ''))
            if ready_sequence != manifest_sequence:
                return None, (f"YOLO frame sequence mismatch: ready={ready_sequence!r} "
                              f"manifest={manifest_sequence!r}")
            if tuple(manifest.get('image_shape', ())) != (h, w):
                return None, (f"YOLO manifest image shape {manifest.get('image_shape')} "
                              f"does not match depth {(h, w)}")
            with np.load(instances_path, allow_pickle=False) as archive:
                required = {'masks', 'class_ids', 'confidences', 'boxes_xyxy', 'names'}
                missing = required.difference(archive.files)
                if missing:
                    return None, f"YOLO instances missing keys: {sorted(missing)}"
                instances = {name: archive[name] for name in required}
            masks = np.asarray(instances['masks'])
            instance_count = len(masks)
            if masks.ndim != 3 or tuple(masks.shape[1:]) != (h, w):
                return None, (f"YOLO masks shape {masks.shape} does not match "
                              f"image {(h, w)}")
            if any(len(np.asarray(instances[key])) != instance_count for key in (
                    'class_ids', 'confidences', 'boxes_xyxy', 'names')):
                return None, "YOLO instance arrays have inconsistent lengths"
            if np.asarray(instances['boxes_xyxy']).shape != (instance_count, 4):
                return None, "YOLO boxes_xyxy must have shape Nx4"
            if int(manifest.get('detection_count', -1)) != instance_count:
                return None, (f"YOLO manifest detection_count="
                              f"{manifest.get('detection_count')} but archive has "
                              f"{instance_count}")
            result['instances'] = instances
            result['frame_manifest'] = manifest
        return result, None
    except Exception as e:
        return None, str(e)


def read_pick_state(shared_dir):
    """读取 pick_state.json（C++ 侧写入），返回 active 物体名列表。

    三态返回：
      None  — 文件缺失/损坏 → 回退全量识别（感知-only 流水线行为不变）
      []    — 无 pending 物体（全部已放置）→ 调用方跳过整帧
      list  — 过滤未知名称后的 active 名称（perception key）

    为什么只识别 pending（未放置）物体：C++ 主节点每成功抓取一个物体就把它
    从 pending 列表移除（写回本文件）。推理端据此跳过已放置物体，避免
    （1）对已被抓走的物体做无意义推理浪费时间；
    （2）机械臂停在放置区时被误识别为抓取目标（历史误检来源之一）。
    未知名称被过滤掉，防止 C++ 写入的额外字段污染颜色分割。
    """
    path = os.path.join(shared_dir, 'pick_state.json')
    try:
        with open(path) as f:
            state = json.load(f)
        pending = state.get('pending')
        if not isinstance(pending, list):
            return None
        # 只保留已知物体名，未知项（如 C++ 的调试字段）一律丢弃
        return [n for n in pending if n in OBJECT_COLOR_RANGES]
    except (OSError, ValueError):
        # 文件不存在/解析失败时按"全部待抓取"处理（感知-only 模式兼容）
        return None


def segment_object_points(cloud, color, active_names=None):
    """Return mask for Gazebo colored objects and per-object point counts.

    active_names=None 时识别全部物体（原行为）；否则只对未放置的
    active 物体做掩膜——已放置物体不再被识别和推理。
    """
    if active_names is None:
        active_names = list(OBJECT_COLOR_RANGES.keys())
    masks = {}
    object_mask = np.zeros(len(color), dtype=bool)

    for name in active_names:
        ranges = OBJECT_COLOR_RANGES[name]
        mask = np.all(color >= ranges['lower'], axis=1) & np.all(color <= ranges['upper'], axis=1)
        masks[name] = mask
        object_mask |= mask

    stats = {
        name: int(np.count_nonzero(mask))
        for name, mask in masks.items()
    }
    stats['total_object_points'] = int(np.count_nonzero(object_mask))
    stats['all_objects_visible'] = all(
        stats[name] >= cfgs.min_object_points
        for name in masks
    )

    return object_mask, masks, stats


def object_centers_from_masks(cloud, object_masks):
    centers = {}
    for name, mask in object_masks.items():
        points = cloud[mask]
        if len(points) == 0:
            continue
        centers[name] = points.mean(axis=0).astype(float)
    return centers


def parse_roi(roi_text):
    if not roi_text:
        return None
    try:
        values = [float(v.strip()) for v in roi_text.split(',')]
    except ValueError:
        raise ValueError(f"Invalid --roi {roi_text!r}; expected x_min,y_min,x_max,y_max")
    if len(values) != 4:
        raise ValueError(f"Invalid --roi {roi_text!r}; expected four comma-separated values")
    x_min, y_min, x_max, y_max = values
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        raise ValueError(f"Invalid --roi {roi_text!r}; values must be normalized and ordered")
    return x_min, y_min, x_max, y_max


# 动态 ROI：由宿主机 test/roi_editor.py 写入共享目录 roi.json，
# 推理每帧读取，无需重启即可调整工作区范围。
# 不做 mtime 缓存：实测本机内核时间戳粒度下同刻连续写会得到相同
# st_mtime_ns，缓存会导致 ROI 更新静默丢失；文件仅 ~400B、帧间隔 2s，
# 每帧读一次开销可忽略。日志只在值变化/首次报错时打印。
_precheck_cache = {'warned': False, 'last_ts': None, 'last': None}


def load_precheck_results(shared_dir):
    """读 shared_dir/precheck_results.json（grasp_executor 预检结果，可视化过滤用）。

    无文件/非法返回 None（回退全量显示）；按 mtime 检测更新（与 ROI 同模式）。
    结构：{'approved': [{'xyz': [..], 'R': [[..]]}, ...],
          'selected': 同结构或 None, 'selected_index': int|None}（相机系位姿）。
    """
    path = os.path.join(shared_dir, 'precheck_results.json')
    try:
        mtime = os.path.getmtime(path)
        if _precheck_cache['last_ts'] == mtime:
            return _precheck_cache['last']
        with open(path, 'r') as f:
            data = json.load(f)
        if not isinstance(data.get('approved'), list):
            raise ValueError('approved 字段缺失或非法')
        _precheck_cache['last_ts'] = mtime
        _precheck_cache['last'] = data
        print(f"  [precheck] loaded: approved={len(data['approved'])} "
              f"selected={data.get('selected_index')}", flush=True)
        return data
    except OSError:
        return None
    except (KeyError, ValueError, json.JSONDecodeError) as e:
        if not _precheck_cache['warned']:
            _precheck_cache['warned'] = True
            print(f"  [precheck] WARNING invalid precheck_results.json ({e}); "
                  f"show all grasps", flush=True)
        return None


_dynamic_roi_cache = {'warned': False, 'last': None}


def load_dynamic_roi(shared_dir):
    """读取共享目录 roi.json；无文件/非法返回 None（回退 --roi 静态参数）。"""
    path = os.path.join(shared_dir, 'roi.json')
    try:
        with open(path, 'r') as f:
            data = json.load(f)
        roi = (float(data['x_min']), float(data['y_min']),
               float(data['x_max']), float(data['y_max']))
        if not (0.0 <= roi[0] < roi[2] <= 1.0 and 0.0 <= roi[1] < roi[3] <= 1.0):
            raise ValueError('values must be normalized and ordered')
        _dynamic_roi_cache['warned'] = False
        if roi != _dynamic_roi_cache['last']:
            _dynamic_roi_cache['last'] = roi
            print(f"  [dynamic roi] loaded: x=[{roi[0]:.2f},{roi[2]:.2f}] "
                  f"y=[{roi[1]:.2f},{roi[3]:.2f}]", flush=True)
        return roi
    except OSError:
        return None
    except (KeyError, ValueError, json.JSONDecodeError) as e:
        if not _dynamic_roi_cache['warned']:
            _dynamic_roi_cache['warned'] = True
            print(f"  [dynamic roi] WARNING invalid roi.json ({e}); "
                  f"using full frame", flush=True)
        return None


def make_point_cloud(points, colors):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))
    return cloud


def atomic_imwrite(path, image):
    tmp_path = f"{path}.tmp.png"
    if not cv2.imwrite(tmp_path, image):
        raise RuntimeError(f"Failed to write image: {tmp_path}")
    os.replace(tmp_path, path)


def safe_destroy_windows():
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


def cleanup_debug_outputs(output_dir):
    if cfgs.max_saved_debug_frames <= 0:
        return
    frame_ids = set()
    for path in glob.glob(os.path.join(output_dir, '*_[0-9][0-9][0-9][0-9].*')):
        stem = os.path.splitext(os.path.basename(path))[0]
        try:
            frame_ids.add(int(stem.rsplit('_', 1)[1]))
        except (IndexError, ValueError):
            continue
    stale_ids = sorted(frame_ids)[:-cfgs.max_saved_debug_frames]
    for frame_id in stale_ids:
        for pattern in (
            f'grasps_{frame_id:04d}.npy',
            f'scene_{frame_id:04d}.pcd',
            f'objects_{frame_id:04d}.pcd',
        ):
            path = os.path.join(output_dir, pattern)
            try:
                os.remove(path)
            except FileNotFoundError:
                pass


def cleanup_stale_outputs(shared_dir):
    """启动时清理上一轮运行残留的旧输出，控制磁盘占用。

    固定文件名（color.png/depth.png/result*.png/CSV 等）每帧覆盖，
    运行期间不会累积；这里清理的是跨运行残留：
      result.png / result_color.png / last_validation.json /
      perception_targets.csv / perception_grasps.csv / .ready / output/ 调试帧。
    保留 roi.json（ROI 编辑器的持久配置，见 test/roi_editor.py）。
    """
    removed = []
    for name in ('result.png', 'result_color.png', 'last_validation.json',
                 'perception_targets.csv', 'perception_grasps.csv', '.ready'):
        path = os.path.join(shared_dir, name)
        try:
            os.remove(path)
            removed.append(name)
        except FileNotFoundError:
            pass
    out_dir = os.path.join(shared_dir, 'output')
    if os.path.isdir(out_dir):
        for path in sorted(glob.glob(os.path.join(out_dir, '*'))):
            try:
                os.remove(path)
                removed.append(os.path.join('output', os.path.basename(path)))
            except OSError:
                pass
    if removed:
        print(f"  [cleanup] 启动时清理旧输出: {', '.join(removed)}", flush=True)
    else:
        print("  [cleanup] 共享目录无旧输出残留", flush=True)


def process_data(data, active_names=None):
    """将原始相机数据转换为 GraspNet 输入格式

    处理链：深度图反投影成点云 → 剔除无效深度点 → 颜色分割出物体点
    （同时滤掉灰色平台等背景）→ 只从物体点采样 num_point 个点喂给网络。
    active_names=None 时识别全部物体；否则只识别未放置（pending）的物体。
    """
    color = data['color']
    depth = data['depth']
    camera = data['camera']

    # 生成点云：用内参矩阵 + factor_depth 把 uint16 毫米深度图反投影成
    # organized 点云（与像素网格对齐，方便按颜色掩膜取点）。
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # 有效点筛选（移除深度为0的点）：深度 0 表示"无有效测量"，
    # 反投影后位置无意义，必须剔除。
    mask = depth > 0
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    if len(cloud_masked) == 0:
        return None, "No valid points in point cloud"

    print(f"  valid points: {len(cloud_masked)}", flush=True)

    if cfgs.segmentation_mode == 'yolo':
        instances = data.get('instances')
        if instances is None:
            return None, "YOLO segmentation selected but instances were not loaded"
        masks = np.asarray(instances['masks'], dtype=bool)
        names = [str(value) for value in np.asarray(instances['names']).tolist()]
        confidences = np.asarray(instances['confidences'], dtype=np.float32)
        if len(masks) == 0:
            return None, "YOLO frame contains no selected target instance"

        depth_m = data['depth'].astype(np.float32) / float(camera.scale)
        valid_target_depth = (
            mask & (depth_m >= cfgs.min_depth_m) & (depth_m <= cfgs.max_depth_m))
        union_mask = np.zeros_like(mask, dtype=bool)
        object_masks = {}
        object_points_by_name = {}
        object_centers_camera = {}
        object_stats = {}
        used_names = set()
        for index, instance_mask in enumerate(masks):
            name = names[index] if index < len(names) else f'target_{index}'
            if not name or name in used_names:
                name = f'{name or "target"}_{index}'
            used_names.add(name)
            pixel_mask = instance_mask & valid_target_depth
            points = cloud[pixel_mask]
            if len(points) < cfgs.min_object_points:
                print(
                    f"  WARNING: YOLO instance {name} has {len(points)} valid depth "
                    f"points; need {cfgs.min_object_points}",
                    flush=True,
                )
                continue
            union_mask |= pixel_mask
            object_masks[name] = pixel_mask[mask]
            object_points_by_name[name] = points
            object_centers_camera[name] = points.mean(axis=0).astype(float)
            object_stats[name] = int(len(points))
            confidence = float(confidences[index]) if index < len(confidences) else 0.0
            print(
                f"  YOLO instance: {name} confidence={confidence:.3f} "
                f"points={len(points)}",
                flush=True,
            )

        if not object_masks:
            return None, "No YOLO instance has enough valid depth points"
        object_points = cloud[union_mask]
        object_colors = color[union_mask]
        object_stats['total_object_points'] = int(len(object_points))
        object_stats['all_objects_visible'] = True

        replace = len(object_points) < cfgs.num_point
        indices = np.random.choice(len(object_points), cfgs.num_point, replace=replace)
        cloud_sampled = object_points[indices]
        color_sampled = object_colors[indices]
        scene_cloud = make_point_cloud(cloud_masked, color_masked)
        object_cloud = make_point_cloud(object_points, object_colors)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_tensor = torch.from_numpy(
            cloud_sampled[np.newaxis].astype(np.float32)).to(device)

        return {
            'end_points': {
                'point_clouds': cloud_tensor,
                'cloud_colors': color_sampled,
            },
            'scene_cloud': scene_cloud,
            'object_cloud': object_cloud,
            'object_points': object_points,
            'object_points_by_name': object_points_by_name,
            'object_masks': object_masks,
            'object_stats': object_stats,
            'object_centers_camera': object_centers_camera,
            'color_vis': data['color_vis'],
            'roi': None,
            'roi_source': 'yolo',
        }, None

    if cfgs.segmentation_mode == 'workspace':
        depth_m = data['depth'].astype(np.float32) / float(camera.scale)
        workspace_mask = (depth_m > cfgs.min_depth_m) & (depth_m < cfgs.max_depth_m) & mask
        roi = load_dynamic_roi(cfgs.shared_dir)
        roi_source = 'dynamic' if roi is not None else None
        if roi is None:
            roi = parse_roi(cfgs.roi)
            roi_source = 'static'
        if roi is not None:
            h, w = depth_m.shape
            x_min, y_min, x_max, y_max = roi
            roi_mask = np.zeros_like(workspace_mask, dtype=bool)
            x0, x1 = int(x_min * w), int(x_max * w)
            y0, y1 = int(y_min * h), int(y_max * h)
            roi_mask[y0:y1, x0:x1] = True
            workspace_mask &= roi_mask

        workspace_points = cloud[workspace_mask]
        if len(workspace_points) == 0:
            return None, "No points inside real-camera workspace"

        object_points = workspace_points
        object_colors = color[workspace_mask]
        if len(object_points) < cfgs.min_object_points:
            return None, (f"Only {len(object_points)} target points after workspace "
                          f"segmentation; need {cfgs.min_object_points}")

        object_mask_flat = workspace_mask[mask]
        object_masks = {cfgs.workspace_name: object_mask_flat}
        object_stats = {
            cfgs.workspace_name: int(len(object_points)),
            'total_object_points': int(len(object_points)),
            'all_objects_visible': len(object_points) >= cfgs.min_object_points,
        }
        object_centers_camera = {cfgs.workspace_name: object_points.mean(axis=0).astype(float)}
        object_points_by_name = {cfgs.workspace_name: object_points}

        roi_str = 'full'
        if roi is not None:
            roi_str = f"x=[{roi[0]:.2f},{roi[2]:.2f}] y=[{roi[1]:.2f},{roi[3]:.2f}]" \
                      f" ({roi_source})"
        print(
            f"  workspace points: {len(workspace_points)} "
            f"depth=({cfgs.min_depth_m:.2f},{cfgs.max_depth_m:.2f})m roi={roi_str}",
            flush=True,
        )

        num_point = cfgs.num_point
        replace = len(object_points) < num_point
        idxs = np.random.choice(len(object_points), num_point, replace=replace)
        cloud_sampled = object_points[idxs]
        color_sampled = object_colors[idxs]

        scene_cloud = make_point_cloud(cloud_masked, color_masked)
        object_cloud = make_point_cloud(object_points, object_colors)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)

        return {
            'end_points': {
                'point_clouds': cloud_tensor,
                'cloud_colors': color_sampled,
            },
            'scene_cloud': scene_cloud,
            'object_cloud': object_cloud,
            'object_points': object_points,
            'object_points_by_name': object_points_by_name,
            'object_masks': object_masks,
            'object_stats': object_stats,
            'object_centers_camera': object_centers_camera,
            'color_vis': data['color_vis'],
            'roi': roi,
            'roi_source': roi_source,
        }, None

    # 颜色分割 + 平面过滤：segment_object_points 按颜色区间把物体点
    # 从场景中分离出来（灰色平台不在任何颜色区间内 → 自动成为背景），
    # 得到"只含物体"的点云，这也是 GraspNet 的输入与推理范围。
    object_mask, object_masks, object_stats = segment_object_points(
        cloud_masked, color_masked, active_names)
    object_centers_camera = object_centers_from_masks(cloud_masked, object_masks)
    object_points_by_name = {
        name: cloud_masked[mask]
        for name, mask in object_masks.items()
    }
    cloud_no_plane = cloud_masked[object_mask]
    color_no_plane = color_masked[object_mask]

    print(
        "  object points: "
        f"{object_stats['total_object_points']} "
        + " ".join(f"{n}={object_stats[n]}" for n in sorted(object_masks)),
        flush=True,
    )

    if len(cloud_no_plane) == 0:
        return None, "No colored object points after plane filtering"

    if not object_stats['all_objects_visible']:
        # 物体点不足只告警不报错：可能是部分遮挡/物体已移动，
        # 仍把当前能看到的物体点送入推理（尽力而为）。
        print(
            f"  WARNING: not all objects meet min_object_points={cfgs.min_object_points}",
            flush=True,
        )

    # ── 采样 ────────────────────────────────────────────────
    num_point = cfgs.num_point
    has_objects = len(cloud_no_plane) > 0
    if has_objects:
        # 仅从物体点采样，不够则 repeat（不填充平面点）
        # 为什么只采样物体点：GraspNet 输入尺寸固定（num_point），
        # 把整帧场景点全采样会稀释物体占比，降低抓取质量。
        replace = len(cloud_no_plane) < num_point
        idxs = np.random.choice(len(cloud_no_plane), num_point, replace=replace)
        cloud_sampled = cloud_no_plane[idxs]
        color_sampled = color_no_plane[idxs]
    else:
        return None, "No object points available for sampling"

    # scene_cloud = 全部有效点（供碰撞检测 / 渲染），
    # object_cloud = 仅物体点（供渲染 / 验证）。
    scene_cloud = make_point_cloud(cloud_masked, color_masked)
    object_cloud = make_point_cloud(cloud_no_plane, color_no_plane)

    # 转换为 Tensor（用于网络 — 仅物体点云）
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)

    end_points = {
        'point_clouds': cloud_tensor,
        'cloud_colors': color_sampled,
    }

    return {
        'end_points': end_points,
        'scene_cloud': scene_cloud,
        'object_cloud': object_cloud,
        'object_points': cloud_no_plane,
        'object_points_by_name': object_points_by_name,
        'object_masks': object_masks,
        'object_stats': object_stats,
        'object_centers_camera': object_centers_camera,
        'color_vis': data['color_vis'],
        'roi': None,
        'roi_source': None,
    }, None


# ===================================================================
#  推理
# ===================================================================
def run_inference(net, end_points):
    """运行 GraspNet 推理

    网络前传（无梯度）→ pred_decode 解码出原始抓取（约 300×12×4 个候选）
    → 转成 GraspGroup（统一封装，带 NMS/排序/可视化工具）。
    """
    with torch.no_grad():
        end_points_out = net(end_points)
        grasp_preds = pred_decode(end_points_out)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    print(f"  decoded grasps: {len(gg)}", flush=True)
    return gg


def collision_filter(gg, scene_cloud):
    """碰撞检测过滤：剔除与场景点云发生碰撞的抓取。

    用 ModelFreeCollisionDetector 在完整场景（含平台）上做无模型碰撞检测，
    把会扎进桌面/平台的抓取删掉，只剩可执行的候选。
    """
    t0 = time.time()
    mfcdetector = ModelFreeCollisionDetector(
        np.asarray(scene_cloud.points), voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05,
                                        collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    print(f"  after collision filter: {len(gg)} ({time.time()-t0:.2f}s)", flush=True)
    return gg


def validate_grasps(gg, object_points):
    """Check whether final grasp centers are near the segmented object cloud."""
    result = {
        'grasp_count': int(len(gg)),
        'centers_inside_object_bbox': 0,
        'best_grasp_inside_object_bbox': False,
        'object_bbox_min': None,
        'object_bbox_max': None,
        'best_grasp_center': None,
    }

    if len(object_points) == 0:
        return result

    bbox_min = object_points.min(axis=0) - cfgs.grasp_object_margin
    bbox_max = object_points.max(axis=0) + cfgs.grasp_object_margin
    result['object_bbox_min'] = bbox_min.astype(float).tolist()
    result['object_bbox_max'] = bbox_max.astype(float).tolist()

    if len(gg) == 0:
        return result

    translations = np.asarray(gg.translations)
    inside = np.all((translations >= bbox_min) & (translations <= bbox_max), axis=1)
    result['centers_inside_object_bbox'] = int(np.count_nonzero(inside))
    result['best_grasp_inside_object_bbox'] = bool(inside[0])
    result['best_grasp_center'] = translations[0].astype(float).tolist()
    return result


def select_object_grasp_targets(gg, processed):
    """Select one GraspNet 6D grasp pose per colored object.

    为每个物体挑选候选抓取：
    1. 对全部抓取做 NMS（去重）+ 按分数排序；
    2. 用物体点云包围盒（加 grasp_object_margin 余量）筛选"落在该物体上"的抓取；
    3. 逐个检查世界系 XY 位置是否在物体源区域先验附近（防误抓）;
    4. 综合"网络分数 - 0.25×离源距离"排序，输出每个物体前
       candidates_per_object 个完整 6D 候选（含调试诊断字段）。
    """
    targets = {}
    if len(gg) == 0:
        return targets

    gg.nms()
    gg.sort_by_score()
    # grasp 数组布局（16 列）：
    #   [0]=score, [1..3]=width/height/depth, [4..12]=3x3 旋转(行主序),
    #   [13..15]=平移(camera 系)
    grasp_array = np.asarray(gg.grasp_group_array)
    translations = grasp_array[:, 13:16]

    for name, mask in processed['object_masks'].items():
        object_points = processed['object_points_by_name'].get(name)
        if object_points is None or len(object_points) == 0:
            continue

        # 包围盒过滤：抓取中心必须在物体点云包围盒（带余量）内，
        # 否则视为"抓空了"的候选。
        bbox_min = object_points.min(axis=0) - cfgs.grasp_object_margin
        bbox_max = object_points.max(axis=0) + cfgs.grasp_object_margin
        inside = np.all((translations >= bbox_min) & (translations <= bbox_max), axis=1)
        candidate_idxs = np.nonzero(inside)[0]
        if len(candidate_idxs) == 0:
            continue

        ranked = []
        for idx in candidate_idxs:
            grasp = grasp_array[int(idx)]
            translation = grasp[13:16].astype(np.float64)
            rotation = grasp[4:13].reshape(3, 3).astype(np.float64)
            # 相机系 → 世界系（平移用 CAMERA_TO_WORLD_R，
            # 旋转用 CAMERA_TO_WORLD_ROTATION_R，见上方常量注释）
            world_translation = CAMERA_TO_WORLD_R @ translation + CAMERA_TO_WORLD_T
            # 源区域先验过滤：世界系 XY 离物体初始位置太远的抓取直接丢弃
            # （通常是误检或机械臂把物体撞飞后产生的假候选）。
            source_prior = SOURCE_OBJECT_PRIORS_WORLD.get(name)
            source_dist_xy = 0.0
            if source_prior is not None:
                source_dist_xy = float(np.linalg.norm(world_translation[:2] - source_prior[:2]))
                if source_dist_xy > SOURCE_PRIOR_MAX_XY_DIST:
                    continue
            # 完整姿态链：camera 系旋转 → 世界系 → GraspNet→Piper TCP 轴映射
            world_tcp_rotation = CAMERA_TO_WORLD_ROTATION_R @ rotation @ GRASPNET_TO_PIPER_TCP_R
            tcp_forward_z = float(world_tcp_rotation[2, 2])  # TCP 前进方向(世界Z)分量，诊断用
            grasp_tcp_z = float(world_translation[2])
            link6_grasp_z = grasp_tcp_z  # 当前 TCP 与 link6 无 Z 偏移（PIPER_TOOL_Z_OFFSET=0）
            # 排序分 = 网络分数 - 0.25×离源距离：同等分数下优先抓"原位附近"的物体
            rank_score = (
                float(grasp[0])
                - 0.25 * source_dist_xy
            )
            ranked.append((rank_score, int(idx), link6_grasp_z, tcp_forward_z, source_dist_xy))

        if len(ranked) == 0:
            continue

        ranked.sort(reverse=True)
        center = processed['object_centers_camera'].get(name)
        candidates = []
        for rank_score, idx, link6_grasp_z, tcp_forward_z, source_dist_xy in ranked[:cfgs.candidates_per_object]:
            grasp = grasp_array[idx]
            candidates.append({
                'center': center,
                'count': int(processed['object_stats'].get(name, 0)),
                'grasp': grasp,
                'rank_score': float(rank_score),
                'link6_grasp_z': float(link6_grasp_z),
                'tcp_forward_z': float(tcp_forward_z),
                'source_dist_xy': float(source_dist_xy),
            })
        targets[name] = candidates

    return targets


def write_frame_validation(shared_dir, frame_idx, processed, gg, grasp_validation):
    """把本帧诊断与抓取候选写入共享目录，供 C++ 主节点 / 验收脚本使用。

    输出三份文件：
      last_validation.json   — 完整诊断（物体点数、中心、每候选的 6D 位姿等）
      perception_targets.csv — 物体中心（旧接口，C++ 早期版本用）
      perception_grasps.csv  — 抓取候选主输出：C++ graspnet_6d_grasp_planner
                               读取并转成 TCP 目标执行抓取。
    所有文件都先写 .tmp 再 rename（原子写），防止 C++ 侧读到半行。
    """
    grasp_targets = select_object_grasp_targets(gg, processed)
    output = {
        'frame': int(frame_idx),
        'timestamp': time.time(),
        'active_objects': sorted(processed['object_masks']),
        'valid_scene_points': int(len(processed['scene_cloud'].points)),
        'object_stats': processed['object_stats'],
        'object_centers_camera': {
            name: center.tolist()
            for name, center in processed['object_centers_camera'].items()
        },
        'plane_filtered_out_points': int(
            len(processed['scene_cloud'].points) - len(processed['object_cloud'].points)
        ),
        'grasp_validation': grasp_validation,
        'per_object_grasps': {
            name: [
                {
                    'score': float(target['grasp'][0]),
                    'rank_score': float(target['rank_score']),
                    'width': float(target['grasp'][1]),
                    'height': float(target['grasp'][2]),
                    'depth': float(target['grasp'][3]),
                    'rotation_matrix': target['grasp'][4:13].reshape(3, 3).astype(float).tolist(),
                    'translation': target['grasp'][13:16].astype(float).tolist(),
                    'link6_grasp_z': float(target['link6_grasp_z']),
                    'tcp_forward_z': float(target['tcp_forward_z']),
                    'source_dist_xy': float(target['source_dist_xy']),
                    'object_center_camera': target['center'].astype(float).tolist()
                    if target['center'] is not None else None,
                    'count': int(target['count']),
                }
                for target in targets
            ]
            for name, targets in grasp_targets.items()
        },
    }
    path = os.path.join(shared_dir, 'last_validation.json')
    with open(path, 'w') as f:
        json.dump(output, f, indent=2)

    # perception_targets.csv：每个物体的相机系中心（旧接口，仅保留兼容）
    csv_path = os.path.join(shared_dir, 'perception_targets.csv')
    tmp_path = f"{csv_path}.tmp"
    with open(tmp_path, 'w') as f:
        f.write('name,camera_x,camera_y,camera_z,count\n')
        for name in sorted(processed['object_masks']):
            center = processed['object_centers_camera'].get(name)
            if center is None:
                continue
            count = processed['object_stats'].get(name, 0)
            f.write(
                f"{name},{center[0]:.9f},{center[1]:.9f},{center[2]:.9f},{count}\n"
            )
    os.replace(tmp_path, csv_path)

    # perception_grasps.csv：抓取候选主输出（每行一个完整 6D 候选）。
    # 列布局与 C++ graspnet_6d_grasp_planner 的解析代码严格对应，
    # 修改列顺序时必须同步改 C++（历史坑：列错位导致抓取位姿完全错误）。
    grasp_csv_path = os.path.join(shared_dir, 'perception_grasps.csv')
    tmp_grasp_path = f"{grasp_csv_path}.tmp"
    header = [
        'name', 'camera_x', 'camera_y', 'camera_z', 'count',
        'score', 'width', 'height', 'depth',
        'r00', 'r01', 'r02', 'r10', 'r11', 'r12', 'r20', 'r21', 'r22',
        'center_camera_x', 'center_camera_y', 'center_camera_z',
        'link6_grasp_z', 'tcp_forward_z',
        'source_dist_xy',
    ]
    with open(tmp_grasp_path, 'w') as f:
        f.write(','.join(header) + '\n')
        for name in sorted(processed['object_masks']):
            targets = grasp_targets.get(name)
            if not targets:
                continue
            for target in targets:
                grasp = target['grasp']
                center = target['center']
                rotation = grasp[4:13]
                translation = grasp[13:16]
                row = [
                    name,
                    f"{translation[0]:.9f}",
                    f"{translation[1]:.9f}",
                    f"{translation[2]:.9f}",
                    str(target['count']),
                    f"{grasp[0]:.9f}",
                    f"{grasp[1]:.9f}",
                    f"{grasp[2]:.9f}",
                    f"{grasp[3]:.9f}",
                    *[f"{v:.9f}" for v in rotation],
                    f"{center[0]:.9f}" if center is not None else "nan",
                    f"{center[1]:.9f}" if center is not None else "nan",
                    f"{center[2]:.9f}" if center is not None else "nan",
                    f"{target['link6_grasp_z']:.9f}",
                    f"{target['tcp_forward_z']:.9f}",
                    f"{target['source_dist_xy']:.9f}",
                ]
                f.write(','.join(row) + '\n')
    os.replace(tmp_grasp_path, grasp_csv_path)


# ===================================================================
#  可视化 — 使用 Open3D 离屏渲染 + OpenCV 显示
# ===================================================================
def render_result(gg, scene_cloud, object_cloud, color_vis, object_stats=None,
                  grasp_validation=None, precheck=None, highlight=False):
    """将点云和抓取姿态渲染为图像

    用 Open3D 离屏渲染器把场景点云 + 物体点云 + 前 top_k 个抓取姿态
    画成一张图，叠加文字信息后返回给显示线程（OpenCV/matplotlib 窗口）。
    渲染失败时回退到纯彩色图并标注错误（不中断主循环）。

    precheck + highlight=True（执行候选页面专用）：只渲染 selected 匹配的
    唯一最佳候选。precheck=None 或 highlight=False：显示全部候选（原页面）。
    """
    if len(gg) == 0:
        print("  No grasps to visualize", flush=True)
        # 没有抓取，直接返回彩色图（color_vis 是内部管线 RGB，需转 BGR）
        img = cv2.cvtColor(color_vis, cv2.COLOR_RGB2BGR)
        cv2.putText(img, "No grasps detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return img

    gg.nms()
    gg.sort_by_score()
    top_gg = gg[:cfgs.top_k]
    if precheck is not None and highlight:
        # 只保留机械臂最终选中的 selected 候选。approved 可能包含多个通过项，
        # 但此页面不得显示不会执行的候选。匹配遍历 NMS 后的全部候选，避免
        # 执行侧低分优先候选落在可视化 top_k 之外而被错误隐藏。
        top_gg = gg
        # 位姿匹配（相机系）：ranked 序号与 GraspGroup 序不同源，不能按序号。
        # GraspGroup.__getitem__ 不支持 tuple 索引（grasp.py:268 仅 int/slice），
        # 用属性/底层数组提取（translations 即 grasp_group_array[:, 13:16]）
        xyz = np.asarray(top_gg.translations, dtype=np.float64)
        rot = top_gg.grasp_group_array[:, 4:13].reshape(-1, 3, 3).astype(np.float64)
        n_before = len(top_gg)
        try:
            _, selected_idx = match_precheck(
                xyz, rot, precheck.get('approved', []), precheck.get('selected'))
            selected_mask = np.zeros(n_before, dtype=bool)
            if selected_idx >= 0:
                selected_mask[selected_idx] = True
            top_gg = top_gg[selected_mask]
            if selected_idx >= 0:
                print(f"  [precheck] 只显示机械臂执行最佳候选 "
                      f"1/{n_before}（rank={precheck.get('selected_index')}）",
                      flush=True)
            else:
                print("  [precheck] 当前帧未匹配到机械臂执行候选，页面不显示抓手",
                      flush=True)
        except Exception as e:
            # 页面契约是“只显示执行项”；匹配异常时不得回退全量候选。
            top_gg = top_gg[np.zeros(n_before, dtype=bool)]
            print(f"  [precheck] 执行候选匹配失败，页面不显示抓手: {e}", flush=True)
    print(f"  showing top {len(top_gg)} grasps", flush=True)

    try:
        grippers = top_gg.to_open3d_geometry_list()
        W, H = cfgs.display_width, cfgs.display_height

        renderer = o3d.visualization.rendering.OffscreenRenderer(W, H)
        renderer.scene.set_background([0.9, 0.9, 0.9, 1.0])

        # 添加完整场景点云（浅色）和非平面物体点云（原色）
        mat_cloud = o3d.visualization.rendering.MaterialRecord()
        mat_cloud.shader = "defaultUnlit"
        renderer.scene.add_geometry("cloud", scene_cloud, mat_cloud)

        mat_object = o3d.visualization.rendering.MaterialRecord()
        mat_object.shader = "defaultUnlit"
        renderer.scene.add_geometry("object_cloud", object_cloud, mat_object)

        # 添加抓取姿态（全部 graspnet 默认线框色）
        for i, g in enumerate(grippers):
            mat_grip = o3d.visualization.rendering.MaterialRecord()
            mat_grip.shader = "defaultUnlit"
            renderer.scene.add_geometry(f"gripper_{i}", g, mat_grip)

        # camera-native view (2026-08-07): 渲染视点 = D435i 相机位置本身。
        # 点云在相机坐标系中（相机位于原点，+Z 向前，+Y 向下），所以：
        #   eye = 原点（相机位置）、up = -Y（相机系"上"方向）、
        #   center = 场景包围盒中心（物体在画面中央）。
        # 与相机原始视角一致（不再后上方俯视取景）。
        # 注意：Open3D 0.17 要求 float32 类型（不接收 float64，历史踩坑点）
        bbox = renderer.scene.bounding_box
        if bbox is not None:
            fov_deg = 58.0  # D435i RGB 垂直 FOV ≈ 57.5°
            center = bbox.get_center().astype(np.float32)
            eye = np.zeros(3, dtype=np.float32)  # 相机位置 = 原点
            up = np.array([0.0, -1.0, 0.0], dtype=np.float32)  # 相机系 -Y 是"上"
            renderer.setup_camera(fov_deg, center, eye, up)

        img = renderer.render_to_image()
        # Open3D Image → numpy → BGR for OpenCV
        result = np.asarray(img)
        if result.dtype == np.uint8 and len(result.shape) == 3:
            result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        else:
            result = result.copy()

        # 添加信息覆盖层（中文用 PIL 字体绘制——cv2.putText 仅 ASCII 会出 ? 乱码）
        if precheck is not None and highlight:
            if len(top_gg) == 1:
                put_text_cn(
                    result,
                    f"机械臂执行最佳候选: rank {precheck.get('selected_index')}",
                    (10, 30), 22, (0, 200, 0))
            else:
                put_text_cn(result, "当前帧无匹配的执行候选", (10, 30),
                            22, (0, 165, 255))
        else:
            cv2.putText(result, f"Grasps: {len(top_gg)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.putText(result, f"Frame: {frame_count}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        if object_stats:
            cv2.putText(
                result,
                f"Object pts: " + " ".join(
                    f"{n}={object_stats.get(n, 0)}"
                    for n in sorted(object_stats)
                    if n not in ('total_object_points', 'all_objects_visible')),
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (60, 60, 60),
                1,
            )
        if grasp_validation:
            cv2.putText(
                result,
                f"Grasp bbox hits: {grasp_validation['centers_inside_object_bbox']} final grasps",
                (10, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (60, 60, 60),
                1,
            )

        return result
    except Exception as e:
        print(f"  Render error: {e}", flush=True)
        # 回退到彩色图（color_vis 是内部管线 RGB，需转 BGR）
        img = cv2.cvtColor(color_vis, cv2.COLOR_RGB2BGR)
        cv2.putText(img, f"Render error: {e}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return img


_precheck_page_cache = {'mtime': None, 'selected_key': None, 'img': None}


def render_precheck_page(gg, processed, precheck):
    """执行候选页面：只渲染机械臂最终选中的 selected 最佳候选。

    预检结果 json 未变化（mtime 相同）时返回缓存图——预检是低频事件
    （秒级/分钟级），避免每帧二次 Open3D 渲染拖慢推理。
    """
    if precheck is None:
        return None
    path = os.path.join(cfgs.shared_dir, 'precheck_results.json')
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return None
    if _precheck_page_cache['mtime'] == mtime:
        return _precheck_page_cache['img']
    selected_key = json.dumps(
        precheck.get('selected'), sort_keys=True, separators=(',', ':'))
    if (_precheck_page_cache['img'] is not None and
            _precheck_page_cache['selected_key'] == selected_key):
        # scan-only 与真实执行会对同一冻结候选各写一次 JSON（仅 timestamp
        # 不同）。第二次写入发生在推理暂停期间；恢复后当前随机解码集合未必
        # 再含该姿态。候选未变时保留已经验证过的唯一候选画面，避免被后续
        # “当前帧未匹配”覆盖。
        _precheck_page_cache['mtime'] = mtime
        return _precheck_page_cache['img']
    img = render_result(gg, processed['scene_cloud'], processed['object_cloud'],
                        processed['color_vis'], processed.get('object_stats'),
                        None, precheck, highlight=True)
    _precheck_page_cache['mtime'] = mtime
    _precheck_page_cache['selected_key'] = selected_key
    _precheck_page_cache['img'] = img
    return img


def draw_roi_overlay(img, roi):
    """在内部管线 RGB 图上绘制归一化 ROI 矩形（绿色框）+ 数值标注。

    roi 为归一化 (x_min, y_min, x_max, y_max)，None 时返回原图副本。
    注：绿框 (0,255,0) 在 RGB/BGR 下对称，不受通道顺序影响。
    """
    if roi is None:
        return img.copy()
    h, w = img.shape[:2]
    x0, y0 = int(roi[0] * w), int(roi[1] * h)
    x1, y1 = int(roi[2] * w), int(roi[3] * h)
    out = img.copy()
    cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
    cv2.putText(out,
                f"ROI x=[{roi[0]:.2f},{roi[2]:.2f}] y=[{roi[1]:.2f},{roi[3]:.2f}]",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return out


def display_loop():
    """在单独的线程中运行显示循环

    将渲染结果实时显示在 matplotlib 窗口中（Docker 内 X11 透传），
    同时保存为 result.png 供主机的 result_viewer.py 使用。

    注：Docker 镜像中的 OpenCV 3.4.11 无 GTK 支持（GTK+: NO），
    因此使用 matplotlib TkAgg 后端替代 cv2.imshow。
    """
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    global running, latest_result_image, latest_color_image, latest_precheck_image
    global frame_count

    # 创建 matplotlib 窗口 1（左：相机彩色图+ROI 框；右：3D 渲染+全部抓取）
    plt.ion()
    fig, (ax_color, ax_3d) = plt.subplots(1, 2, figsize=(18, 7))
    ax_color.axis('off')
    ax_3d.axis('off')
    fig.canvas.manager.set_window_title("GraspNet - Perception Results")
    # 窗口置顶（X11 特定）
    try:
        fig.canvas.manager.window.attributes('-topmost', True)
    except Exception:
        pass

    # 窗口 2（独立页面）：机械臂最终选取的唯一执行候选
    fig2 = plt.figure(figsize=(9, 7))
    ax2 = fig2.add_subplot(111)
    ax2.axis('off')
    fig2.canvas.manager.set_window_title("GraspNet - 机械臂执行最佳候选")
    try:
        fig2.canvas.manager.window.attributes('-topmost', True)
    except Exception:
        pass

    # 预创建空显示对象（颜色图/渲染图各自独立更新）
    dummy_color = np.zeros((480, 640, 3), dtype=np.uint8)
    dummy = np.zeros((cfgs.display_height, cfgs.display_width, 3),
                     dtype=np.uint8)
    img_color = ax_color.imshow(dummy_color, aspect='auto')
    img_3d = ax_3d.imshow(dummy, aspect='auto')
    img_precheck = ax2.imshow(dummy, aspect='auto')

    while running:
        with display_lock:
            img = latest_result_image.copy() if latest_result_image is not None else None
            color = latest_color_image.copy() if latest_color_image is not None else None
            pimg = (latest_precheck_image.copy()
                    if latest_precheck_image is not None else None)
        # 新页面：机械臂执行最佳候选（有结果才更新，保持上次画面）
        if pimg is not None:
            try:
                img_precheck.set_data(cv2.cvtColor(pimg, cv2.COLOR_BGR2RGB))
                fig2.canvas.draw_idle()
                fig2.canvas.flush_events()
                atomic_imwrite(
                    os.path.join(cfgs.shared_dir, 'result_selected.png'), pimg)
            except Exception as e:
                print(f"  Selected-candidate display error: {e}", flush=True)
        if img is not None:
            try:
                # matplotlib 用 RGB，OpenCV 用 BGR
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img_3d.set_data(img_rgb)
                if color is not None:
                    # 注意：color 来自内部管线，read_data_from_shared 已做过
                    # BGR→RGB 转换（color_vis 即 RGB），这里直接显示，
                    # 再转换一次会红蓝互换（历史 bug 模式）
                    img_color.set_data(color)
                fig.canvas.draw()
                fig.canvas.flush_events()
                # 同时保存为文件供主机 viewer 使用（存盘需转回 BGR）
                result_path = os.path.join(cfgs.shared_dir, 'result.png')
                atomic_imwrite(result_path, img)
                if color is not None:
                    atomic_imwrite(os.path.join(cfgs.shared_dir, 'result_color.png'),
                                   cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
            except Exception as e:
                print(f"  Display error: {e}", flush=True)
        else:
            plt.pause(0.1)

    plt.close(fig)


# ===================================================================
#  就绪标志管理
# ===================================================================
def wait_for_new_frame(shared_dir):
    """等待新的 .ready 标志出现（与 camera_bridge_node 的握手协议）。

    轮询 .ready 文件存在性（poll_interval=0.1s）。.ready 由 bridge 在本帧
    所有数据落盘后最后写入（原子写），所以"出现"即代表"整帧就绪"。
    存在竞态：bridge 可能在我们 remove 之前又写下一帧的 .ready——
    但由于数据处理是顺序的，最多丢一帧，不会读到半帧。
    """
    ready_path = os.path.join(shared_dir, '.ready')
    while running:
        if os.path.exists(ready_path):
            return True
        time.sleep(cfgs.poll_interval)
    return False


def clear_ready_flag(shared_dir):
    """清除就绪标志：处理完本帧后删除 .ready，告诉 bridge"帧已消费"。

    删除失败（OSError，如文件刚被覆盖）直接忽略——下次 wait 会等到新的
    .ready，握手机制对偶发丢帧是自愈的。
    """
    ready_path = os.path.join(shared_dir, '.ready')
    try:
        if os.path.exists(ready_path):
            os.remove(ready_path)
    except OSError:
        pass


# ===================================================================
#  主循环
# ===================================================================
def main():
    global running, frame_count, latest_result_image, latest_color_image
    global latest_precheck_image, inference_times

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"=" * 60, flush=True)
    print(f"GraspNet Continuous Inference from Camera", flush=True)
    print(f"  checkpoint:  {cfgs.checkpoint_path}", flush=True)
    print(f"  shared_dir:  {cfgs.shared_dir}", flush=True)
    print(f"  device:      {device}", flush=True)
    print(f"  display:     {'ON' if not cfgs.no_display else 'OFF'}", flush=True)
    print(f"=" * 60, flush=True)

    # 启动时清理上一轮运行的旧输出（保留 roi.json 持久配置）
    cleanup_stale_outputs(cfgs.shared_dir)

    # 加载网络
    t0 = time.time()
    net = get_net()
    print(f"Model loaded in {time.time()-t0:.1f}s", flush=True)

    # 启动显示线程
    if not cfgs.no_display:
        disp_thread = threading.Thread(target=display_loop, daemon=True)
        disp_thread.start()
        print("Display thread started", flush=True)

    # 确保共享目录存在
    os.makedirs(cfgs.shared_dir, exist_ok=True)

    print(f"Waiting for first frame in {cfgs.shared_dir}...", flush=True)

    # ── 推理主循环 ─────────────────────────────────────────────
    # 每轮：等 .ready → 读 pick_state → 读数据 → 处理 → 推理 → 碰撞过滤
    # → 写 CSV/诊断 → 渲染 → 删 .ready（握手闭环）。
    # 任何一步出错都只跳过本帧（clear_ready_flag 后 continue），不退出进程，
    # 保证流水线在偶发坏帧下持续运行。
    last_active = None
    while running:
        # 1. 等待新帧（与 bridge 的 .ready 握手）
        if not wait_for_new_frame(cfgs.shared_dir):
            break

        # 感知-执行互锁（2026-08-07）：机械臂执行期间（grasp_executor 写
        # shared_dir/pause_inference 标记，归零完成后删除）暂停推理——
        # 跳过本帧、不更新 CSV（保留执行前的干净结果），避免把执行中的
        # 机械臂识别进规划目标。
        if os.path.exists(os.path.join(cfgs.shared_dir, 'pause_inference')):
            print("  Paused (arm executing); skipping frame.", flush=True)
            clear_ready_flag(cfgs.shared_dir)
            continue

        frame_count += 1
        t_start = time.time()
        print(f"\n--- Frame {frame_count} ---", flush=True)

        # 读取感知状态（pick_state.json）：只识别尚未放置的物体
        # （状态过滤的历史动机：减少误检、跳过已放置物体，见 read_pick_state）
        if cfgs.segmentation_mode == 'workspace':
            active_names = [cfgs.workspace_name]
        elif cfgs.segmentation_mode == 'yolo':
            # The bridge has already selected exactly one target instance.
            # A non-empty sentinel keeps the existing empty-pick-state guard out
            # of the YOLO path; process_data reads the names from instances.npz.
            active_names = ['yolo_selected_instance']
        else:
            active_names = read_pick_state(cfgs.shared_dir)
            if active_names is None:
                active_names = list(OBJECT_COLOR_RANGES.keys())
        if active_names != last_active:
            print(f"  pick_state: active={active_names}", flush=True)
            last_active = active_names

        try:
            # 全部已放置：跳过整帧（保留上一帧 CSV），避免空识别刷屏
            if not active_names:
                print("  No pending objects (all placed); skipping frame.", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 2. 读取数据
            data, err = read_data_from_shared(cfgs.shared_dir)
            if err:
                print(f"  Read error: {err}", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 3. 处理数据（只识别 active 物体）
            processed, err = process_data(data, active_names)
            if err:
                print(f"  Process error: {err}", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 4. 运行推理
            t2 = time.time()
            gg = run_inference(net, processed['end_points'])
            infer_time = time.time() - t2

            # 5. 碰撞过滤
            if cfgs.collision_thresh > 0:
                t3 = time.time()
                gg = collision_filter(gg, processed['scene_cloud'])
                col_time = time.time() - t3
            else:
                col_time = 0

            grasp_validation = validate_grasps(gg, processed['object_points'])
            write_frame_validation(cfgs.shared_dir, frame_count, processed, gg, grasp_validation)

            # 6. 渲染并显示（渲染在推理主线程，显示在 display_loop 线程，
            # 通过 latest_result_image + display_lock 交接，避免跨线程竞争）
            result_img = None
            roi_overlay = draw_roi_overlay(processed['color_vis'], processed.get('roi'))
            precheck = load_precheck_results(cfgs.shared_dir)
            if not cfgs.no_display:
                # 原页面：全部候选；新页面：机械臂最终选中的唯一候选
                result_img = render_result(gg, processed['scene_cloud'],
                                           processed['object_cloud'],
                                           processed['color_vis'],
                                           processed['object_stats'],
                                           grasp_validation)
                precheck_img = render_precheck_page(gg, processed, precheck)
                with display_lock:
                    latest_result_image = result_img
                    latest_color_image = roi_overlay
                    latest_precheck_image = precheck_img
            else:
                result_img = render_result(gg, processed['scene_cloud'],
                                           processed['object_cloud'],
                                           processed['color_vis'],
                                           processed['object_stats'],
                                           grasp_validation)
                atomic_imwrite(os.path.join(cfgs.shared_dir, 'result.png'), result_img)
                atomic_imwrite(os.path.join(cfgs.shared_dir, 'result_color.png'),
                               cv2.cvtColor(roi_overlay, cv2.COLOR_RGB2BGR))

            # 7. 保存结果（grasps_XXXX.npy / scene_XXXX.pcd / objects_XXXX.pcd，
            # 供主机端 open3d_result_viewer.py 复盘）
            output_dir = os.path.join(cfgs.shared_dir, 'output')
            if cfgs.save_debug_outputs and len(gg) > 0 and frame_count % max(cfgs.save_debug_every, 1) == 0:
                os.makedirs(output_dir, exist_ok=True)
                gg.save_npy(os.path.join(output_dir, f'grasps_{frame_count:04d}.npy'))
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, f'scene_{frame_count:04d}.pcd'),
                    processed['scene_cloud'])
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, f'objects_{frame_count:04d}.pcd'),
                    processed['object_cloud'])
                cleanup_debug_outputs(output_dir)

            # 统计
            total_time = time.time() - t_start
            inference_times.append(infer_time)
            avg_time = sum(inference_times[-50:]) / max(len(inference_times[-50:]), 1)
            print(f"  ├─ inference: {infer_time:.2f}s", flush=True)
            print(f"  ├─ collision: {col_time:.2f}s", flush=True)
            print(f"  ├─ total:     {total_time:.2f}s", flush=True)
            print(f"  └─ avg inf:  {avg_time:.2f}s (last {min(len(inference_times),50)})", flush=True)
            print(f"  ─> {len(gg)} grasp(s) found", flush=True)

        except Exception as e:
            print(f"  ERROR in inference loop: {e}", flush=True)
            import traceback
            traceback.print_exc()

        # 8. 清除标志 → 等待下一帧
        clear_ready_flag(cfgs.shared_dir)
        if cfgs.max_frames > 0 and frame_count >= cfgs.max_frames:
            print(f"Reached --max_frames={cfgs.max_frames}; stopping.", flush=True)
            break

    # ── 清理 ───────────────────────────────────────────────────
    print(f"\n{'='*60}", flush=True)
    print(f"Inference stopped after {frame_count} frames", flush=True)
    if inference_times:
        print(f"Average inference time: {np.mean(inference_times):.3f}s", flush=True)
        print(f"FPS: {1.0/np.mean(inference_times):.1f}", flush=True)

    if not cfgs.no_display:
        safe_destroy_windows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user", flush=True)
        running = False
    finally:
        running = False
        if not cfgs.no_display:
            safe_destroy_windows()
