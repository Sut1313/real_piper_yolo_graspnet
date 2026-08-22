"""text_overlay 纯函数单测（中文叠加，无 torch 依赖，宿主机可跑）。"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'graspnet-baseline'))
import text_overlay as to  # noqa: E402

HOST_FONT = '/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc'


def test_put_text_cn_preserves_size_and_draws():
    img = np.zeros((100, 200, 3), dtype=np.uint8)
    out = to.put_text_cn(img, '可用候选: 3 选中: 红色', (5, 10), font_size=18,
                         color=(0, 0, 220), font_path=HOST_FONT)
    assert out.shape == (100, 200, 3)
    # 文字区应有非零像素（红色系 BGR(0,0,220)）
    roi = out[10:40, 5:150]
    assert roi.sum() > 0


def test_put_text_cn_inplace_modifies_input():
    img = np.zeros((60, 60, 3), dtype=np.uint8)
    to.put_text_cn(img, '测试', (2, 2), font_size=16, font_path=HOST_FONT)
    assert img.sum() > 0


def test_put_text_cn_missing_font_falls_back_ascii():
    img = np.zeros((60, 60, 3), dtype=np.uint8)
    # 字体路径不存在 → 回退 ASCII（中文丢弃，不抛异常不产生 ? 乱码）
    out = to.put_text_cn(img, 'ok 中文', (2, 2), font_size=16,
                         font_path='/no/such/font.ttc')
    assert out.shape == (60, 60, 3)
    # 回退路径绘制了 'ok '（ASCII 部分）→ 有非零像素
    assert out.sum() > 0


def test_put_text_cn_ascii_only_no_font_needed():
    img = np.zeros((60, 60, 3), dtype=np.uint8)
    out = to.put_text_cn(img, 'Grasps: 5', (2, 2), font_size=16,
                         font_path='/no/such/font.ttc')
    assert out.sum() > 0
