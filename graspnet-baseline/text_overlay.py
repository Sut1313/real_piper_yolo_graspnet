"""中文文字叠加模块（推理可视化用）。

cv2.putText 仅支持 ASCII（Hershey 字体），中文会渲染为 '?'——本模块用
PIL ImageDraw + 挂载的 Noto CJK 字体绘制中文。字体缺失/加载失败时回退
ASCII（不阻断渲染）。

宿主测试可跑：仅依赖 PIL/cv2/numpy（无 torch）。
"""

import os

import cv2
import numpy as np

DEFAULT_FONT = '/workspace/fonts/NotoSerifCJK-Bold.ttc'
_font_cache = {}


def _load_font(font_path: str, size: int):
    if font_path not in _font_cache:
        import PIL.ImageFont
        _font_cache[font_path] = PIL.ImageFont.truetype(font_path, size)
    return _font_cache[font_path]


def put_text_cn(img, text, org, font_size=20, color=(0, 0, 220),
                font_path: str = DEFAULT_FONT):
    """在 BGR 图像上叠加中文文本（原地修改，保持尺寸）。

    Args:
        img: BGR numpy 图像（cv2 约定），会被原地修改
        text: 文本（中文/ASCII 均可）
        org: (x, y) 左上角
        font_size: 字号（像素）
        color: BGR 颜色（cv2 约定，与 cv2.putText 一致）
        font_path: 中文字体 ttc/ttf 路径

    Returns:
        img（原地修改后返回；字体不可用时回退 cv2.putText ASCII）
    """
    try:
        import PIL.Image, PIL.ImageDraw
        font = _load_font(font_path, font_size)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pil = PIL.Image.fromarray(rgb)
        draw = PIL.ImageDraw.Draw(pil)
        draw.text((int(org[0]), int(org[1])), text, font=font,
                  fill=(int(color[2]), int(color[1]), int(color[0])))
        out = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        img[...] = out
        return img
    except Exception:
        # 回退：仅保留 ASCII 可显示字符（中文直接丢弃，不产生 '?' 乱码）
        ascii_text = ''.join(c for c in text if ord(c) < 128)
        if ascii_text:
            cv2.putText(img, ascii_text, org, cv2.FONT_HERSHEY_SIMPLEX,
                        font_size / 20.0, color, 2)
        return img
