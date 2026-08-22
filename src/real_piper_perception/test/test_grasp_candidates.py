from pathlib import Path

import numpy as np
from real_piper_perception.grasp_candidates import (
    PerceptionTarget, parse_grasp_csv, rank_candidates, self_consistency,
    build_precheck_results,
)

# M3 修复：fixture 路径基于本文件位置（cwd 无关，包目录内运行 pytest 也通过）
FIXTURE_CSV = str(Path(__file__).parent / 'fixtures' / 'sample_grasps.csv')

def test_parse_csv_row_count():
    cands = parse_grasp_csv(FIXTURE_CSV)
    assert len(cands) == 3
    c = cands[0]
    assert c.rotation.shape == (3, 3)
    assert c.width > 0


def test_rank_score_ascending():
    """2026-08-14 策略：score 升序。"""
    cands = parse_grasp_csv(FIXTURE_CSV)
    ranked = rank_candidates(cands)
    scores = [c.score for c in ranked]
    assert scores == sorted(scores), f'应为 score 升序: {scores}'


def test_rank_does_not_mutate_input_and_is_stable_for_ties():
    cands = parse_grasp_csv(FIXTURE_CSV)
    cands[0].score = 0.8
    cands[1].score = 0.8
    original = list(cands)
    ranked = rank_candidates(cands)
    assert cands == original
    tied = [c for c in ranked if c.score == 0.8]
    assert tied == [cands[0], cands[1]]

def test_self_consistency_gate():
    good = PerceptionTarget(name='t', xyz_camera=np.array([0.0, 0.0, 0.4]),
                            rotation=np.eye(3), score=0.9, width=0.05,
                            center_camera=np.array([0.01, -0.01, 0.4]),
                            link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    assert self_consistency(good)
    bad = PerceptionTarget(name='t', xyz_camera=np.array([0.0, 0.0, 0.4]),
                           rotation=np.eye(3), score=0.9, width=0.05,
                           center_camera=np.array([0.5, 0.0, 0.4]),
                           link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    assert not self_consistency(bad)


def test_build_precheck_results_approved_and_selected():
    """预检结果序列化：approved 列表（相机系位姿）+ selected 单条。"""
    c1 = PerceptionTarget(name='t', xyz_camera=np.array([0.1, 0.2, 0.3]),
                          rotation=np.eye(3), score=0.9, width=0.05,
                          center_camera=np.array([0.1, 0.2, 0.3]),
                          link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    c2 = PerceptionTarget(name='t', xyz_camera=np.array([0.4, 0.5, 0.6]),
                          rotation=np.eye(3), score=0.8, width=0.05,
                          center_camera=np.array([0.4, 0.5, 0.6]),
                          link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    ranked = [c1, c2]
    out = build_precheck_results(ranked, approved_indices=[0, 1], selected_index=1)
    assert len(out['approved']) == 2
    assert np.allclose(out['approved'][0]['xyz'], [0.1, 0.2, 0.3])
    assert np.allclose(out['approved'][1]['R'], np.eye(3))
    assert np.allclose(out['selected']['xyz'], [0.4, 0.5, 0.6])
    assert out['selected_index'] == 1


def test_build_precheck_results_empty_approved():
    c1 = PerceptionTarget(name='t', xyz_camera=np.array([0.1, 0.2, 0.3]),
                          rotation=np.eye(3), score=0.9, width=0.05,
                          center_camera=np.array([0.1, 0.2, 0.3]),
                          link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    out = build_precheck_results([c1], approved_indices=[], selected_index=None)
    assert out['approved'] == []
    assert out['selected'] is None
    assert out['selected_index'] is None


def test_build_precheck_results_negative_index_treated_as_none():
    """防御：selected_index=-1（无全过候选）不得取 ranked[-1]。"""
    c1 = PerceptionTarget(name='t', xyz_camera=np.array([0.1, 0.2, 0.3]),
                          rotation=np.eye(3), score=0.9, width=0.05,
                          center_camera=np.array([0.1, 0.2, 0.3]),
                          link6_grasp_z=0.0, tcp_forward_z=1.0, source_dist_xy=0.0)
    out = build_precheck_results([c1], approved_indices=[], selected_index=-1)
    assert out['selected'] is None
    assert out['selected_index'] is None
