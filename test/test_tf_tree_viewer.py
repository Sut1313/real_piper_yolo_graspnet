"""tf_tree_viewer 纯函数单测（离线 TF 树工具，无 ROS 依赖）。

被测模块：scripts/tf_tree_viewer.py（sys.path 注入，不依赖包安装）。
fixture 基于本文件位置（Path(__file__).parent），cwd 无关。
"""
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import tf_tree_viewer as tv  # noqa: E402

FIXTURES = Path(__file__).parent / 'fixtures'
URDF_FIXTURE = str(FIXTURES / 'two_link_robot.urdf')
HANDEYE_FIXTURE = str(FIXTURES / 'handeye_rotated.yaml')
PROJECT_ROOT = Path(__file__).resolve().parents[1]
REAL_HANDEYE = str(PROJECT_ROOT / 'camera_data' / 'handeye_eye_to_hand.yaml')
REAL_CAMERA_XACRO = str(PROJECT_ROOT / 'src' / 'realsense-ros' / 'realsense2_description'
                        / 'urdf' / 'test_d435i_camera.urdf.xacro')


# ── load_urdf_edges ─────────────────────────────────────────────

def test_load_urdf_edges_parses_origins_and_types():
    edges = tv.load_urdf_edges(URDF_FIXTURE)
    assert len(edges) == 2
    e1, e2 = edges
    assert (e1['parent'], e1['child']) == ('base_link', 'link1')
    assert e1['joint'] == 'joint1' and e1['type'] == 'revolute'
    assert np.allclose(e1['xyz'], [0, 0, 0.1])
    assert np.allclose(e1['rpy'], [0, 0, np.pi / 2])
    assert (e2['parent'], e2['child']) == ('link1', 'link2')
    assert e2['type'] == 'fixed'
    assert np.allclose(e2['xyz'], [0.2, 0, 0])
    assert np.allclose(e2['rpy'], [0.5, 0, 0])


def test_load_urdf_edges_missing_file_raises():
    with pytest.raises(ValueError):
        tv.load_urdf_edges(str(FIXTURES / 'no_such.urdf'))


# ── load_handeye_edge ───────────────────────────────────────────

def test_load_handeye_edge_rotation_and_translation():
    e = tv.load_handeye_edge(HANDEYE_FIXTURE)
    assert (e['parent'], e['child']) == ('base_link', 'camera_link')
    assert np.allclose(e['xyz'], [1.0, 2.0, 3.0])
    assert np.allclose(e['rpy'], [0.1, -0.2, 0.3], atol=1e-9)


def test_load_handeye_edge_missing_file_raises():
    with pytest.raises(ValueError):
        tv.load_handeye_edge(str(FIXTURES / 'no_such.yaml'))


def test_real_handeye_yaml_parses_to_camera_edge():
    # 回归防线：真实标定文件必须能解析成 base_link -> camera_link 边
    e = tv.load_handeye_edge(REAL_HANDEYE)
    assert (e['parent'], e['child']) == ('base_link', 'camera_link')
    assert np.allclose(e['xyz'], [0.4438091644448965, -0.09719881163171148, 0.7446342540373863])
    R = Rotation.from_euler('xyz', e['rpy']).as_matrix()
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)


# ── compile_camera_xacro（真实 d435i xacro 集成） ───────────────

@pytest.mark.skipif(not Path(tv.XACRO_BIN).exists(),
                    reason='xacro 工具不可用')
def test_compile_camera_xacro_returns_camera_chain():
    edges = tv.compile_camera_xacro(REAL_CAMERA_XACRO)
    pairs = {(e['parent'], e['child']) for e in edges}
    # 安装链被丢弃（真实树中 camera_link 的父帧是 base_link，handeye 发布）
    assert ('base_link', 'camera_bottom_screw_frame') not in pairs
    assert ('camera_bottom_screw_frame', 'camera_link') not in pairs
    # 相机内链各流帧 + 光学帧（camera_link 为子树根）
    for frame in ('camera_depth_frame', 'camera_color_frame',
                  'camera_infra1_frame', 'camera_infra2_frame'):
        assert ('camera_link', frame) in pairs
        assert (frame, frame.replace('_frame', '_optical_frame')) in pairs
    # 全部为 fixed 关节
    assert all(e['type'] == 'fixed' for e in edges)


@pytest.mark.skipif(not Path(tv.XACRO_BIN).exists(),
                    reason='xacro 工具不可用')
def test_compile_camera_xacro_missing_file_returns_empty():
    assert tv.compile_camera_xacro(str(FIXTURES / 'no_such.xacro')) == []


# ── compose_tree ────────────────────────────────────────────────

def _chain_edges(*names):
    return [{'parent': a, 'child': b, 'joint': f'j{i}', 'type': 'fixed',
             'xyz': np.zeros(3), 'rpy': np.zeros(3)}
            for i, (a, b) in enumerate(zip(names, names[1:]))]


def test_compose_tree_single_root():
    tree = tv.compose_tree(_chain_edges('a', 'b', 'c'))
    assert tree['roots'] == ['a']
    assert tree['children'] == {'a': ['b'], 'b': ['c']}
    assert tree['parents'] == {'b': 'a', 'c': 'b'}


def test_compose_tree_multiple_roots_sorted():
    edges = _chain_edges('z1', 'z2') + _chain_edges('a1', 'a2')
    tree = tv.compose_tree(edges)
    assert tree['roots'] == ['a1', 'z1']  # 排序稳定


def test_compose_tree_child_with_two_parents_raises():
    edges = _chain_edges('a', 'b') + _chain_edges('c', 'b')
    with pytest.raises(ValueError):
        tv.compose_tree(edges)


def test_compose_tree_cycle_raises():
    edges = _chain_edges('a', 'b') + _chain_edges('b', 'a')
    with pytest.raises(ValueError):
        tv.compose_tree(edges)


def test_compose_tree_empty_edges():
    tree = tv.compose_tree([])
    assert tree['roots'] == []
    assert tree['children'] == {}


# ── find_chain ──────────────────────────────────────────────────

def _pairs(chain):
    """链实际方向：(起点, 终点, reversed 标记)。reversed=True ⇔ 实际方向为子->父。"""
    return [(e['child'], e['parent'], True) if e.get('reversed')
            else (e['parent'], e['child'], False) for e in chain]


def test_find_chain_path_from_src_to_dst():
    edges = _chain_edges('a', 'b', 'c', 'd')
    chain = tv.find_chain(edges, 'a', 'd')
    assert _pairs(chain) == [('a', 'b', False), ('b', 'c', False), ('c', 'd', False)]


def test_find_chain_cross_branch_via_common_ancestor():
    # a 为公共祖先：d -> b -> a（上行，子->父，reversed）然后 a -> c -> e（下行）
    edges = _chain_edges('a', 'b', 'd') + _chain_edges('a', 'c', 'e')
    chain = tv.find_chain(edges, 'd', 'e')
    assert _pairs(chain) == [('d', 'b', True), ('b', 'a', True),
                             ('a', 'c', False), ('c', 'e', False)]


def test_find_chain_dst_is_ancestor_fully_reversed():
    edges = _chain_edges('a', 'b', 'c')
    chain = tv.find_chain(edges, 'c', 'a')
    assert _pairs(chain) == [('c', 'b', True), ('b', 'a', True)]


def test_find_chain_same_frame_returns_empty():
    edges = _chain_edges('a', 'b')
    assert tv.find_chain(edges, 'a', 'a') == []


def test_find_chain_disjoint_trees_returns_none():
    edges = _chain_edges('a', 'b') + _chain_edges('c', 'd')
    assert tv.find_chain(edges, 'a', 'd') is None


# ── chain_transform ─────────────────────────────────────────────

def test_chain_transform_accumulates_translation_and_rotation():
    # a->b: Rz(90°) 无平移；b->c: 平移 [1,0,0]。c 原点在 a 系 = Rz90*[1,0,0] = [0,1,0]
    e1 = {'parent': 'a', 'child': 'b', 'joint': 'j1', 'type': 'fixed',
          'xyz': np.zeros(3), 'rpy': np.array([0, 0, np.pi / 2])}
    e2 = {'parent': 'b', 'child': 'c', 'joint': 'j2', 'type': 'fixed',
          'xyz': np.array([1.0, 0.0, 0.0]), 'rpy': np.zeros(3)}
    xyz, R = tv.chain_transform([e1, e2], 'a', 'c')
    assert np.allclose(xyz, [0.0, 1.0, 0.0])
    assert np.allclose(R, Rotation.from_euler('xyz', [0, 0, np.pi / 2]).as_matrix())


def test_chain_transform_single_edge_is_edge_transform():
    e1 = {'parent': 'a', 'child': 'b', 'joint': 'j1', 'type': 'fixed',
          'xyz': np.array([0.1, 0.2, 0.3]), 'rpy': np.zeros(3)}
    xyz, R = tv.chain_transform([e1], 'a', 'b')
    assert np.allclose(xyz, [0.1, 0.2, 0.3])
    assert np.allclose(R, np.eye(3))


def test_chain_transform_cross_branch_inverse():
    # a->b 平移 [1,0,0]，a->c 平移 [0,1,0]：c 在 b 系 = [0,1,0]-[1,0,0] = [-1,1,0]
    e1 = {'parent': 'a', 'child': 'b', 'joint': 'j1', 'type': 'fixed',
          'xyz': np.array([1.0, 0.0, 0.0]), 'rpy': np.zeros(3)}
    e2 = {'parent': 'a', 'child': 'c', 'joint': 'j2', 'type': 'fixed',
          'xyz': np.array([0.0, 1.0, 0.0]), 'rpy': np.zeros(3)}
    xyz, R = tv.chain_transform([e1, e2], 'b', 'c')
    assert np.allclose(xyz, [-1.0, 1.0, 0.0])
    assert np.allclose(R, np.eye(3))


# ── find_coincident_frames（位置重合检测） ──────────────────────

def _edge(p, c, xyz, rpy=(0, 0, 0)):
    return {'parent': p, 'child': c, 'joint': f'{p}_{c}', 'type': 'fixed',
            'xyz': np.array(xyz, dtype=float), 'rpy': np.array(rpy, dtype=float)}


def test_find_coincident_frames_detects_groups():
    # b 与 c 同在原位（a 系原点），d 在 [1,0,0] 独立
    edges = [_edge('a', 'b', [0, 0, 0]), _edge('a', 'c', [0, 0, 0]),
             _edge('a', 'd', [1, 0, 0])]
    groups = tv.find_coincident_frames(edges)
    assert any(set(g) == {'b', 'c'} for g in groups)
    assert all('d' not in g for g in groups)
    assert all('a' not in g for g in groups)  # 根不参与


def test_find_coincident_frames_distinct_positions_empty():
    edges = [_edge('a', 'b', [1, 0, 0]), _edge('a', 'c', [0, 2, 0])]
    assert tv.find_coincident_frames(edges) == []


def test_find_coincident_frames_zero_offset_chain_merges_all():
    # 零位移链：a->b->c->d 全重合 → 单组 3 帧（不含根 a）
    edges = [_edge('a', 'b', [0, 0, 0]), _edge('b', 'c', [0, 0, 0]),
             _edge('c', 'd', [0, 0, 0])]
    groups = tv.find_coincident_frames(edges)
    assert len(groups) == 1
    assert set(groups[0]) == {'b', 'c', 'd'}


# ── visible_links（keep 集合内帧的连线） ───────────────────────

def test_visible_links_skips_hidden_intermediate():
    # a->b->d，keep={a,d}：b 被跳过，连线直接 a-d
    edges = _chain_edges('a', 'b', 'd')
    assert tv.visible_links(edges, {'a', 'd'}) == [('a', 'd')]


def test_visible_links_adjacent_frames():
    edges = _chain_edges('a', 'b', 'c')
    assert tv.visible_links(edges, {'a', 'b', 'c'}) == [('a', 'b'), ('b', 'c')]


def test_visible_links_root_not_in_keep():
    # a 不在 keep：b 无最近 keep 祖先（不连线），c 的最近 keep 祖先是 b
    edges = _chain_edges('a', 'b', 'c')
    assert tv.visible_links(edges, {'b', 'c'}) == [('b', 'c')]


def test_visible_links_camera_branch():
    # 感知支链：a->c->d，keep={a,d}：跳过 c，只连 a-d
    edges = _chain_edges('a', 'c', 'd')
    assert tv.visible_links(edges, {'a', 'd'}) == [('a', 'd')]


# ── collect_descendants（3D 视图子树隐藏） ──────────────────────

def test_collect_descendants_excludes_root():
    edges = _chain_edges('a', 'b', 'd') + _chain_edges('a', 'c')
    assert tv.collect_descendants(edges, 'a') == {'b', 'c', 'd'}


def test_collect_descendants_unknown_root_empty():
    edges = _chain_edges('a', 'b')
    assert tv.collect_descendants(edges, 'no_such_frame') == set()


# ── compute_frame_poses（3D 视图数据源） ────────────────────────

def test_compute_frame_poses_chain_positions_and_rotations():
    e1 = {'parent': 'a', 'child': 'b', 'joint': 'j1', 'type': 'fixed',
          'xyz': np.array([1.0, 0.0, 0.0]), 'rpy': np.array([0, 0, np.pi / 2])}
    e2 = {'parent': 'b', 'child': 'c', 'joint': 'j2', 'type': 'fixed',
          'xyz': np.array([0.0, 0.0, 2.0]), 'rpy': np.zeros(3)}
    poses = tv.compute_frame_poses([e1, e2], 'a')
    assert set(poses) == {'a', 'b', 'c'}
    # a 自身 = 单位位姿
    assert np.allclose(poses['a'][0], [0, 0, 0])
    assert np.allclose(poses['a'][1], np.eye(3))
    # b：位置 = 边平移，姿态 = Rz90
    assert np.allclose(poses['b'][0], [1.0, 0.0, 0.0])
    assert np.allclose(poses['b'][1],
                       Rotation.from_euler('xyz', [0, 0, np.pi / 2]).as_matrix())
    # c：T = T_ab @ T_bc → Rz90*[0,0,2]+[1,0,0] = [1,0,2]，姿态仍 = Rz90
    assert np.allclose(poses['c'][0], [1.0, 0.0, 2.0])
    assert np.allclose(poses['c'][1],
                       Rotation.from_euler('xyz', [0, 0, np.pi / 2]).as_matrix())


def test_compute_frame_poses_root_itself_without_edges():
    # dict 含 numpy 数组不能用 ==，逐分量断言
    poses = tv.compute_frame_poses([], 'a')
    assert set(poses) == {'a'}
    assert np.allclose(poses['a'][0], np.zeros(3))
    assert np.allclose(poses['a'][1], np.eye(3))


# ── render_ascii ────────────────────────────────────────────────

def test_render_ascii_exact_output():
    edges = tv.load_urdf_edges(URDF_FIXTURE)
    expected = (
        'base_link\n'
        '|- link1 (T[0.000, 0.000, 0.100] R[0.0, 0.0, 90.0]deg joint1/revolute)\n'
        '|  |- link2 (T[0.200, 0.000, 0.000] R[28.6, 0.0, 0.0]deg joint2/fixed)'
    )
    assert tv.render_ascii(edges) == expected


def test_render_ascii_multiple_roots_blank_separated():
    # 两次 _chain_edges 的 enumerate 各自从 0 开始，两条边 joint 名都是 j0
    edges = _chain_edges('a', 'b') + _chain_edges('c', 'd')
    out = tv.render_ascii(edges)
    assert out.split('\n\n') == [
        'a\n|- b (T[0.000, 0.000, 0.000] R[0.0, 0.0, 0.0]deg j0/fixed)',
        'c\n|- d (T[0.000, 0.000, 0.000] R[0.0, 0.0, 0.0]deg j0/fixed)',
    ]
