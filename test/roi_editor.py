#!/usr/bin/env python3
"""动态 ROI 编辑器：运行时调整喂给 graspnet 的裁剪范围。

实时显示相机画面（读共享目录 color.png），通过 4 个归一化输入框
(x_min, y_min, x_max, y_max) 调整 ROI 矩形；回车/按钮生效后原子写入
shared_camera_data/roi.json，推理端下一帧即生效。

用法：
    python3 test/roi_editor.py [--shared-dir PATH]
    SHARED_DIR 环境变量可覆盖默认共享目录。
"""

import argparse
import json
import os
import sys
import tempfile
import time

import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

REFRESH_MS = 2000  # 与桥接发布帧率(~2s)一致


def default_shared_dir():
    d = os.environ.get('SHARED_DIR')
    if d:
        return d
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'shared_camera_data')


def validate(values):
    """校验 4 个归一化值；非法时返回 (ok, msg)。"""
    if len(values) != 4:
        return False, '需要 4 个数值'
    try:
        vals = [float(v) for v in values]
    except (TypeError, ValueError):
        return False, '必须为数字'
    x_min, y_min, x_max, y_max = vals
    if not all(0.0 <= v <= 1.0 for v in vals):
        return False, '数值必须在 0~1 之间'
    if x_min >= x_max:
        return False, 'x_min 必须小于 x_max'
    if y_min >= y_max:
        return False, 'y_min 必须小于 y_max'
    return True, vals


class RoiEditor:
    def __init__(self, root, shared_dir):
        self.root = root
        self.shared_dir = shared_dir
        self.image_mtime = None
        self.photo = None
        self.canvas_img = None
        self.img_w = self.img_h = 0

        root.title('GraspNet 动态 ROI 编辑器')
        root.geometry('920x700')

        # --- 输入区 ---
        ctrl = tk.Frame(root)
        ctrl.pack(fill=tk.X, padx=8, pady=8)
        self.entries = {}
        for i, (label, key) in enumerate(
                [('x_min', 'x_min'), ('y_min', 'y_min'),
                 ('x_max', 'x_max'), ('y_max', 'y_max')]):
            tk.Label(ctrl, text=label, width=6).grid(row=i // 2, column=(i % 2) * 2, padx=(8, 0))
            e = tk.Entry(ctrl, width=10, justify=tk.CENTER)
            e.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 16), pady=2)
            e.bind('<Return>', self.apply)
            e.bind('<KeyRelease>', self._redraw)
            self.entries[key] = e
        self.apply_btn = tk.Button(ctrl, text='生效', width=10, command=self.apply)
        self.apply_btn.grid(row=0, column=4, rowspan=2, padx=8)
        self.status = tk.Label(ctrl, text='就绪', fg='gray', anchor='w')
        self.status.grid(row=2, column=0, columnspan=5, sticky='we', pady=(4, 0))
        tk.Label(ctrl, text='归一化 0~1，回车生效', fg='gray').grid(
            row=3, column=0, columnspan=5, sticky='w')

        # --- 画面区 ---
        self.canvas = tk.Canvas(root, bg='black', highlightthickness=1)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self._load_initial_roi()
        self._refresh_image()
        root.after(REFRESH_MS, self._tick)

    # ---------- ROI 读写 ----------
    def _roi_path(self):
        return os.path.join(self.shared_dir, 'roi.json')

    def _load_initial_roi(self):
        try:
            with open(self._roi_path()) as f:
                data = json.load(f)
            vals = [data.get(k) for k in ('x_min', 'y_min', 'x_max', 'y_max')]
            ok, _ = validate(vals)
            if ok:
                for k, v in zip(('x_min', 'y_min', 'x_max', 'y_max'), vals):
                    self.entries[k].delete(0, tk.END)
                    self.entries[k].insert(0, str(v))
                self.status.config(text='已加载现有 roi.json', fg='gray')
        except (OSError, ValueError, json.JSONDecodeError):
            pass  # 无文件或非法：保持默认 0,0,1,1

    def _current_values(self):
        return [self.entries[k].get().strip() for k in ('x_min', 'y_min', 'x_max', 'y_max')]

    def apply(self, _event=None):
        ok, result = validate(self._current_values())
        keys = ('x_min', 'y_min', 'x_max', 'y_max')
        if not ok:
            for k in keys:
                self.entries[k].config(bg='#ffcccc')
            self.status.config(text=f'输入非法：{result}', fg='red')
            return
        for k in keys:
            self.entries[k].config(bg='white')
        data = {'x_min': result[0], 'y_min': result[1],
                'x_max': result[2], 'y_max': result[3],
                'updated_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        try:
            self._atomic_write(data)
            self.status.config(
                text=f'已写入 roi.json: x[{result[0]:.2f},{result[2]:.2f}] '
                     f'y[{result[1]:.2f},{result[3]:.2f}] （推理下一帧生效）',
                fg='green')
        except OSError as e:
            self.status.config(text=f'写入失败：{e}', fg='red')

    def _atomic_write(self, data):
        os.makedirs(self.shared_dir, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.shared_dir, prefix='roi.json.', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self._roi_path())
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # ---------- 画面 ----------
    def _tick(self):
        self._refresh_image()
        self.root.after(REFRESH_MS, self._tick)

    def _refresh_image(self):
        path = os.path.join(self.shared_dir, 'color.png')
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            self._show_placeholder('等待相机画面: shared_camera_data/color.png')
            return
        if mtime == self.image_mtime:
            self._redraw()
            return
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self._show_placeholder('color.png 读取失败')
            return
        self.image_mtime = mtime
        self.img_h, self.img_w = img.shape[:2]
        self._show_image(img)
        self._redraw()

    def _canvas_size(self):
        self.canvas.update_idletasks()
        w = self.canvas.winfo_width() or 900
        h = self.canvas.winfo_height() or 620
        return w, h

    def _fit_scale(self):
        if not self.img_w:
            return 1.0
        cw, ch = self._canvas_size()
        return min(cw / self.img_w, ch / self.img_h)

    def _show_image(self, img_bgr):
        scale = self._fit_scale()
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        if scale < 1.0:
            img_rgb = cv2.resize(img_rgb, None, fx=scale, fy=scale,
                                 interpolation=cv2.INTER_AREA)
        self.photo = ImageTk.PhotoImage(Image.fromarray(img_rgb))
        self.canvas.delete('all')
        self.canvas_img = self.canvas.create_image(
            self.canvas.winfo_width() // 2 or 0, 0, anchor=tk.N,
            image=self.photo)

    def _show_placeholder(self, text):
        self.canvas.delete('all')
        self.canvas.create_text(self.canvas.winfo_width() // 2 or 450,
                                self.canvas.winfo_height() // 2 or 300,
                                text=text, fill='gray', font=('Helvetica', 14))

    def _redraw(self, _event=None):
        """按当前输入框值在画面上叠加 ROI 矩形（不重新读图）。"""
        if self.photo is None:
            return
        ok, vals = validate(self._current_values())
        self.canvas.delete('roi')
        if not ok or not self.img_w:
            return
        x_min, y_min, x_max, y_max = vals
        scale = self._fit_scale()
        cw, _ = self._canvas_size()
        ox = (cw - self.img_w * scale) / 2  # 居中偏移
        self.canvas.create_rectangle(
            ox + x_min * self.img_w * scale,
            y_min * self.img_h * scale,
            ox + x_max * self.img_w * scale,
            y_max * self.img_h * scale,
            outline='#00ff00', width=2, tags='roi')


def main():
    ap = argparse.ArgumentParser(description='GraspNet 动态 ROI 编辑器')
    ap.add_argument('--shared-dir', default=default_shared_dir(),
                    help='共享目录（默认 ../shared_camera_data）')
    args = ap.parse_args()

    root = tk.Tk()
    RoiEditor(root, args.shared_dir)
    root.mainloop()


if __name__ == '__main__':
    main()
