#!/usr/bin/env python3
"""离线 TF 树可视化工具：纯静态数据（URDF + 手眼标定），不连 /tf，机器人可不上电。

数据源（全部本地文件）：
  1. piper_description.urdf        —— 机器人链 base_link->link1..6->gripper_base->link7->link8
  2. camera_data/handeye_eye_to_hand.yaml —— base_link <- camera_link 手眼外参
  3. realsense2_description xacro  —— 相机内链 camera_link->{depth,color,infra1,infra2}(->optical)

用法：
  python3 scripts/tf_tree_viewer.py                # 3D 窗口：真机抓取 TF 链（默认 10 帧：
      # base_link..link7 + camera_link），XYZ 三元组(朝向)+点位(位置)+连线(位移)
  python3 scripts/tf_tree_viewer.py --frames base_link,link7,camera_link
      # 自定义帧集（逗号分隔）
  python3 scripts/tf_tree_viewer.py --save tf.png  # 3D 图存盘（Agg）
  python3 scripts/tf_tree_viewer.py --ascii        # 终端 ASCII 树（全量数据，仅文本）
  python3 scripts/tf_tree_viewer.py --from camera_color_optical_frame --to link8
      # 链路查询：逐帧 + 每步变换 + 累计变换（全量数据）
  python3 scripts/tf_tree_viewer.py --camera-xacro ''  # 跳过相机链（仅机器人+手眼）

坐标语义（与 grasp_transform.py 一致）：edge 的 xyz/rpy 即父系中该子帧的位姿，
链路累计 = 各边矩阵依次左乘（T_root_to_child = T_e1 @ T_e2 @ ...）。
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_URDF = str(
    PROJECT_ROOT / 'src' / 'piper_ros' / 'src' / 'piper_description' / 'urdf'
    / 'piper_description.urdf')
DEFAULT_HANDEYE = str(PROJECT_ROOT / 'camera_data' / 'handeye_eye_to_hand.yaml')
DEFAULT_CAMERA_XACRO = str(
    PROJECT_ROOT / 'src' / 'realsense-ros' / 'realsense2_description' / 'urdf'
    / 'test_d435i_camera.urdf.xacro')
XACRO_BIN = shutil.which('xacro') or '/opt/ros/humble/bin/xacro'
# 真机抓取用到的 TF 链（grasp_execute.py 注释 / piper.srdf 依据）：
#   arm 组 = base_link->link6（joint1-6），gripper 组 = gripper_base->link7（joint7）；
#   link8 是 joint7 的镜像手指、不在任何规划组；感知侧 handeye 发布 base_link<-camera_link。
GRASP_CHAIN_FRAMES = ('base_link', 'link1', 'link2', 'link3', 'link4',
                      'link5', 'link6', 'gripper_base', 'link7', 'camera_link')


# ── 数据加载（纯函数） ──────────────────────────────────────────

def _edges_from_root(root: ET.Element) -> list:
    """URDF <robot> 根 → edge 列表。edge = {parent, child, joint, type, xyz, rpy}。"""
    edges = []
    for j in root.findall('joint'):
        parent = j.find('parent')
        child = j.find('child')
        if parent is None or child is None:
            raise ValueError(f'joint {j.get("name")!r} 缺 parent/child 元素')
        origin = j.find('origin')
        if origin is None:
            xyz, rpy = np.zeros(3), np.zeros(3)
        else:
            xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()])
            rpy = np.array([float(v) for v in origin.get('rpy', '0 0 0').split()])
        edges.append({
            'parent': parent.get('link'),
            'child': child.get('link'),
            'joint': j.get('name', ''),
            'type': j.get('type', 'fixed'),
            'xyz': xyz,
            'rpy': rpy,
        })
    return edges


def load_urdf_edges(urdf_path: str) -> list:
    """解析 URDF 全部关节 origin -> edge 列表。缺文件/坏 XML 抛 ValueError。"""
    try:
        root = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f'URDF 解析失败: {urdf_path}: {exc}') from exc
    return _edges_from_root(root)


def load_urdf_edges_from_text(text: str) -> list:
    return _edges_from_root(ET.fromstring(text))


def load_handeye_edge(yaml_path: str, parent: str = 'base_link',
                      child: str = 'camera_link') -> dict:
    """handeye yaml（R_cam2base/t_cam2base）-> 单条 edge（父系中相机位姿）。

    旋转矩阵 -> euler xyz（弧度），与 URDF origin 的 rpy 表示统一。
    """
    try:
        with open(yaml_path) as f:
            d = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'handeye yaml 解析失败: {yaml_path}: {exc}') from exc
    try:
        R = np.asarray(d['R_cam2base'], dtype=np.float64).reshape(3, 3)
        t = np.asarray(d['t_cam2base'], dtype=np.float64).reshape(3)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'handeye yaml 缺少 R_cam2base/t_cam2base: {yaml_path}') from exc
    return {
        'parent': parent,
        'child': child,
        'joint': 'handeye',
        'type': 'fixed',
        'xyz': t,
        'rpy': Rotation.from_matrix(R).as_euler('xyz'),
    }


def compile_camera_xacro(xacro_path: str) -> list:
    """xacro -> URDF 文本 -> edge 列表。xacro 缺失/失败：警告并返回 []（不阻断主链）。

    依赖 ament 索引解析嵌套 $(find realsense2_description)：把项目 install/ 前缀
    注入 AMENT_PREFIX_PATH（realsense2_description 已由 build_ws.sh 安装）。
    """
    xacro_path = Path(xacro_path)
    if not xacro_path.exists():
        print(f'WARN: 相机 xacro 不存在，跳过相机链: {xacro_path}', file=sys.stderr)
        return []
    if not os.path.exists(XACRO_BIN):
        print(f'WARN: xacro 工具不可用（{XACRO_BIN}），跳过相机链', file=sys.stderr)
        return []
    # colcon 布局：install/<pkg>/share/...，ament 索引前缀须是包级目录
    pkg_prefix = PROJECT_ROOT / 'install' / 'realsense2_description'
    if not (pkg_prefix / 'share' / 'ament_index').exists():
        print(f'WARN: install/ 缺少 realsense2_description 索引，先跑 build_ws.sh，'
              f'跳过相机链', file=sys.stderr)
        return []
    env = dict(os.environ)
    existing = env.get('AMENT_PREFIX_PATH', '')
    env['AMENT_PREFIX_PATH'] = str(pkg_prefix) + (':' + existing if existing else '')
    # 离线建模需要完整内链：xacro 默认 use_nominal_extrinsics:=false 时不输出
    # depth/color/infra 流关节（真实节点自行发布校准光学帧）。此处用标称值，
    # 与真实校准值的差异见 realsense xacro 注释。
    cmd = [XACRO_BIN, str(xacro_path), 'use_nominal_extrinsics:=true']
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60, env=env)
        if out.returncode != 0:
            print(f'WARN: xacro 编译失败，跳过相机链:\n{out.stderr}', file=sys.stderr)
            return []
        edges = load_urdf_edges_from_text(out.stdout)
        # 丢弃安装链：真实运行时树中 camera_link 的父帧是 base_link（handeye 发布），
        # xacro 的 base_link->bottom_screw->camera_link 不在真实树中。
        return [e for e in edges
                if e['parent'] != 'base_link' and e['child'] != 'camera_link']
    except (OSError, subprocess.SubprocessError, ET.ParseError) as exc:
        print(f'WARN: 相机链编译异常，跳过: {exc}', file=sys.stderr)
        return []


# ── 树结构（纯函数） ────────────────────────────────────────────

def compose_tree(edges: list) -> dict:
    """edges -> {'children': {parent: [children]}, 'parents': {child: parent},
    'roots': [根列表,排序]}。多父帧/环路抛 ValueError。"""
    children, parents = {}, {}
    for e in edges:
        p, c = e['parent'], e['child']
        if c in parents and parents[c] != p:
            raise ValueError(f'帧 {c} 有多个父帧: {parents[c]} 与 {p}')
        parents[c] = p
        children.setdefault(p, []).append(c)
    # 环路检测：从每个节点沿 parents 上溯，重访即环
    for f in set(children) | set(parents):
        seen, node = set(), f
        while node in parents:
            if node in seen:
                raise ValueError(f'TF 树存在环路（节点 {node}）')
            seen.add(node)
            node = parents[node]
    roots = sorted(f for f in children if f not in parents)
    return {'children': children, 'parents': parents, 'roots': roots}


def find_chain(edges: list, src: str, dst: str):
    """src->dst 的边序列（任意两帧，经最近公共祖先）；同帧 []；不同树 None。

    每项 = 原边 dict + 'reversed' 标记：True ⇔ 该步实际方向为子帧->父帧
    （累计时取逆）。上行段（src 向上到 LCA）reversed=True，下行段（LCA 向下
    到 dst）reversed=False。
    """
    if src == dst:
        return []
    by_child = {e['child']: e for e in edges}

    def up_path(frame):
        """frame 向上到根的边序列（自 frame 起，子->父方向）。"""
        path, node = [], frame
        while node in by_child:
            e = by_child[node]
            path.append(e)
            node = e['parent']
        return path

    up_src, up_dst = up_path(src), up_path(dst)
    anc_dst = {dst} | {e['parent'] for e in up_dst}
    src_up_len = None
    for i, a in enumerate([src] + [e['parent'] for e in up_src]):
        if a in anc_dst:
            src_up_len = i
            lca = a
            break
    if src_up_len is None:
        return None  # 不在同一棵树
    dst_up_len = 0
    node = dst
    while node != lca:
        dst_up_len += 1
        node = by_child[node]['parent']
    chain = [{**e, 'reversed': True} for e in up_src[:src_up_len]]
    chain += [{**e, 'reversed': False} for e in reversed(up_dst[:dst_up_len])]
    return chain


def chain_transform(edges: list, src: str, dst: str):
    """链路累计变换 -> (位置 xyz, 旋转矩阵 R)。T = T_e1 @ T_e2 @ ...；
    反向边（reversed=True）取逆矩阵。"""
    chain = find_chain(edges, src, dst)
    if chain is None:
        raise ValueError(f'帧 {src} -> {dst} 无通路')
    T = np.eye(4)
    for e in chain:
        T_e = np.eye(4)
        T_e[:3, :3] = Rotation.from_euler('xyz', e['rpy']).as_matrix()
        T_e[:3, 3] = e['xyz']
        if e.get('reversed'):
            T_e = np.linalg.inv(T_e)
        T = T @ T_e
    return T[:3, 3], T[:3, :3]


def visible_links(edges: list, keep: set) -> list:
    """keep 内帧沿树路径的最短连线 [(ancestor, child)]。

    child 的"最近 keep 祖先"（沿父链向上第一个在 keep 里的帧）；中间帧被跳过，
    使跨隐藏帧的链路（如 base_link 与 camera_link 之间）直接相连。根不在 keep
    且无可达 keep 祖先的帧不连线。
    """
    tree = compose_tree(edges)
    by_child = {e['child']: e for e in edges}
    links = []
    for root in tree['roots']:
        queue = [root]
        for name in queue:
            queue.extend(tree['children'].get(name, []))
            if name == root or name not in keep:
                continue
            anc = by_child[name]['parent']
            while anc not in keep:
                if anc not in by_child:
                    break  # 到达根且根不在 keep
                anc = by_child[anc]['parent']
            if anc in keep:
                links.append((anc, name))
    return links


def collect_descendants(edges: list, root: str) -> set:
    """root 的所有后代帧（不含 root 自身）；root 不在树中返回空集。"""
    tree = compose_tree(edges)
    if root not in tree['children']:
        return set()
    out, queue = set(), [root]
    for name in queue:
        for child in tree['children'].get(name, []):
            out.add(child)
            queue.append(child)
    return out


def find_coincident_frames(edges: list, atol: float = 1e-4) -> list:
    """位置重合的帧分组（同一 root 下原点距离 < atol），供显示端合并标签。

    返回组列表，每组 >=2 帧（不含根）。URDF 零位移关节/共享铰点会产生
    物理重合（如 link1==link2、link7==link8），数据本身合法。
    """
    tree = compose_tree(edges)
    groups = []
    for root in tree['roots']:
        poses = compute_frame_poses(edges, root)
        names = list(poses)
        used = set()
        for i, n1 in enumerate(names):
            if n1 in used or n1 == root:
                continue
            grp = [n1]
            for n2 in names[i + 1:]:
                if n2 in used or n2 == root:
                    continue
                if np.allclose(poses[n1][0], poses[n2][0], atol=atol):
                    grp.append(n2)
                    used.add(n2)
            if len(grp) > 1:
                groups.append(grp)
                used.add(n1)
    return groups


def compute_frame_poses(edges: list, root: str) -> dict:
    """root 子树内每帧相对 root 的累计位姿 -> {frame: (位置 xyz, 旋转矩阵 R)}。

    T_frame = T_e1 @ T_e2 @ ...（与 chain_transform 同一累加语义）；root 自身 = 单位阵。
    root 不在任何边中（孤立帧）时返回 {root: (0, I)}。
    """
    tree = compose_tree(edges)
    poses = {root: (np.zeros(3), np.eye(3))}
    if root not in tree['children']:
        return poses
    by_child = {e['child']: e for e in edges}
    queue = [root]
    for name in queue:
        pos, R = poses[name]
        for child in tree['children'].get(name, []):
            e = by_child[child]
            T_e = np.eye(4)
            T_e[:3, :3] = Rotation.from_euler('xyz', e['rpy']).as_matrix()
            T_e[:3, 3] = e['xyz']
            child_pos = R @ e['xyz'] + pos
            child_R = R @ T_e[:3, :3]
            poses[child] = (child_pos, child_R)
            queue.append(child)
    return poses


# ── 渲染（纯函数） ──────────────────────────────────────────────

def _edge_label(e: dict) -> str:
    xyz = ', '.join(f'{v:.3f}' for v in e['xyz'])
    rpy = ', '.join(f'{np.degrees(v):.1f}' for v in e['rpy'])
    return f'T[{xyz}] R[{rpy}]deg {e["joint"]}/{e["type"]}'


def render_ascii(edges: list) -> str:
    """ASCII 树（- | 风格，多根之间空行分隔），无尾部换行。"""
    tree = compose_tree(edges)
    by_parent = tree['children']
    edge_lookup = {(e['parent'], e['child']): e for e in edges}
    lines = []

    def visit_subtree(name, prefix):
        """打印以 name 为根的子树；name 自身行（含 label）由调用者输出。"""
        for child in by_parent.get(name, []):
            e = edge_lookup[(name, child)]
            lines.append(prefix + '|- ' + f'{child} ({_edge_label(e)})')
            visit_subtree(child, prefix + '|  ')

    for i, root in enumerate(tree['roots']):
        if i:
            lines.append('')
        lines.append(root)  # 根行：无 label 无前缀
        visit_subtree(root, '')
    return '\n'.join(lines)


def draw3d(edges: list, save_path: str = None, keep: set = None):
    """3D 坐标架视图：keep 内每帧画 XYZ 三元组（朝向）+ 原点点位（位置）+
    帧名；连线 = keep 内帧沿树路径的最短连线（位移关系，跳过中间隐藏帧）。
    keep=None 显示全部帧。save_path 给出时存图（Agg）。"""
    if save_path or not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    tree = compose_tree(edges)
    if keep is None:
        keep = set(tree['children']) | set(tree['parents'])
    # 位置重合帧合并标签（link7/link8 这类共享铰点；仅合并 keep 内可见帧）
    label_map = {}
    for grp in find_coincident_frames(edges):
        visible = [f for f in grp if f in keep]
        if len(visible) >= 2:
            merged = '/'.join(visible)
            for f in visible:
                label_map[f] = merged
    fig = plt.figure(figsize=(15, 11))
    ax = fig.add_subplot(111, projection='3d')

    # keep 内帧位姿（跨根子树）
    visible_poses = {}
    for root in tree['roots']:
        for name, pose in compute_frame_poses(edges, root).items():
            if name in keep:
                visible_poses[name] = pose

    # 骨架线（keep 内帧的最短连线）
    for anc, child in visible_links(edges, keep):
        ax.plot([visible_poses[anc][0][0], visible_poses[child][0][0]],
                [visible_poses[anc][0][1], visible_poses[child][0][1]],
                [visible_poses[anc][0][2], visible_poses[child][0][2]],
                color='#999999', lw=1.0, alpha=0.7, zorder=1)

    # 每帧：点位 + XYZ 三元组（朝向）+ 帧名
    points = [pos for pos, _ in visible_poses.values()]
    max_abs = np.max(np.abs(points), axis=0) if points else np.ones(3)
    axis_len = float(np.max(max_abs)) * 0.08 + 0.03
    for name, (pos, R) in visible_poses.items():
        ax.scatter(*pos, s=14, color='black', depthshade=False, zorder=4)
        for i, color in enumerate(('#e6194b', '#3cb44b', '#4363d8')):
            end = pos + R[:, i] * axis_len
            ax.plot([pos[0], end[0]], [pos[1], end[1]], [pos[2], end[2]],
                    color=color, lw=2.2, zorder=3)
        ax.text(pos[0], pos[1], pos[2] + axis_len * 0.6,
                label_map.get(name, name), fontsize=8, zorder=5)

    # 等比例
    if points:
        lo = np.min(points, axis=0)
        hi = np.max(points, axis=0)
        span = np.maximum(hi - lo, 0.1)
        ax.set_xlim(lo[0] - 0.1 * span[0], hi[0] + 0.1 * span[0])
        ax.set_ylim(lo[1] - 0.1 * span[1], hi[1] + 0.1 * span[1])
        ax.set_zlim(lo[2] - 0.1 * span[2], hi[2] + 0.1 * span[2])
        ax.set_box_aspect((span[0], span[1], span[2]))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    # 三元组颜色图例
    from matplotlib.lines import Line2D
    legend = [Line2D([0], [0], color='#e6194b', lw=2.5, label='X'),
              Line2D([0], [0], color='#3cb44b', lw=2.5, label='Y'),
              Line2D([0], [0], color='#4363d8', lw=2.5, label='Z'),
              Line2D([0], [0], color='#999999', lw=1.0, label='parent-child link')]
    ax.legend(handles=legend, loc='upper right', fontsize=9)
    ax.set_title('Grasp TF chain 3D (MoveIt arm/gripper + handeye)', fontsize=12)
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'已保存 TF 3D 图: {save_path}')
    else:
        plt.show()


# ── 主程序 ──────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='离线 TF 树可视化（URDF + 手眼标定 + 相机 xacro，不连 /tf）')
    ap.add_argument('--robot-urdf', default=DEFAULT_ROBOT_URDF,
                    help=f'机器人 URDF（默认 {DEFAULT_ROBOT_URDF}）')
    ap.add_argument('--handeye', default=DEFAULT_HANDEYE,
                    help=f'手眼标定 yaml（默认 {DEFAULT_HANDEYE}）')
    ap.add_argument('--camera-xacro', default=DEFAULT_CAMERA_XACRO,
                    help='相机 xacro（默认 d435i；传空串跳过相机链）')
    ap.add_argument('--ascii', action='store_true', help='仅打印终端 ASCII 树（不弹窗）')
    ap.add_argument('--save', metavar='PNG', help='3D 图存盘（无 DISPLAY 自动 Agg）')
    ap.add_argument('--frames', default='',
                    help='只显示指定帧（逗号分隔）；空 = 真机抓取链'
                         '（base_link..link7 + camera_link）')
    ap.add_argument('--from', dest='from_frame', metavar='FRAME', help='链路起点')
    ap.add_argument('--to', dest='to_frame', metavar='FRAME', help='链路终点')
    args = ap.parse_args()

    edges = []
    edges += load_urdf_edges(args.robot_urdf)
    edges.append(load_handeye_edge(args.handeye))  # dict 不能 +=（会迭代键）
    if args.camera_xacro:
        edges += compile_camera_xacro(args.camera_xacro)

    tree = compose_tree(edges)
    frames = set(edge['child'] for edge in edges) | set(edge['parent'] for edge in edges)
    print(f'[tf_tree_viewer] 边数={len(edges)} 帧数={len(frames)} 根={tree["roots"]}')
    if len(tree['roots']) > 1:
        print(f'WARN: 存在 {len(tree["roots"])} 个未连通子树（根不唯一）')
    coincident = find_coincident_frames(edges)
    if coincident:
        print('WARN: 位置重合的坐标系组（原点相距 <1e-4m，图中标签已合并显示）:')
        for g in coincident:
            print('   ', ' / '.join(g))

    if args.from_frame and args.to_frame:
        print(f'\n=== 链路 {args.from_frame} -> {args.to_frame} ===')
        chain = find_chain(edges, args.from_frame, args.to_frame)
        if chain is None:
            print('无通路（两帧不在同一棵子树）')
        elif not chain:
            print('起点即终点，累计变换 = 单位阵')
        else:
            for e in chain:
                a, b = ((e['child'], e['parent']) if e.get('reversed')
                        else (e['parent'], e['child']))
                print(f'  {a} -> {b}: {_edge_label(e)}')
            xyz, R = chain_transform(edges, args.from_frame, args.to_frame)
            rpy = np.degrees(Rotation.from_matrix(R).as_euler('xyz'))
            print(f'累计: T[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}] '
                  f'R[{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}]deg')

    if args.ascii:
        print('\n=== TF 树（ASCII） ===')
        print(render_ascii(edges))

    keep = (set(f.strip() for f in args.frames.split(',')) if args.frames
            else set(GRASP_CHAIN_FRAMES))
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    if args.save:
        draw3d(edges, save_path=args.save, keep=keep)
    elif args.ascii:
        pass  # 仅终端
    elif not has_display:
        print('\nWARN: 无 DISPLAY 且未给 --save，已降级打印 ASCII（加 --save 可出 3D 图）',
              file=sys.stderr)
        print(render_ascii(edges))
    else:
        draw3d(edges, keep=keep)


if __name__ == '__main__':
    main()
