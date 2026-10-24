"""Transactional shared-directory writer for YOLO + GraspNet frames."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np
import scipy.io as scio

from .yolo_detector import Detection, draw_detections


class FrameBundleWriter:
    """Write a complete frame and publish ``.ready`` only after commit.

    The producer never replaces a frame while ``.ready`` exists.  GraspNet
    removes that flag after consuming the bundle, so files from two RGB-D
    frames cannot be mixed.
    """

    def __init__(self, shared_dir: str, write_overlay: bool = True):
        self.shared_dir = Path(shared_dir).expanduser().resolve()
        self.shared_dir.mkdir(parents=True, exist_ok=True)
        self.ready_path = self.shared_dir / '.ready'
        self.write_overlay = bool(write_overlay)
        self.sequence = 0

    @property
    def busy(self) -> bool:
        return self.ready_path.exists()

    def _temporary_path(self, path: Path, suffix: str = '') -> Path:
        return path.with_name(
            f'.{path.name}.tmp-{os.getpid()}-{time.time_ns()}{suffix}')

    def _atomic_text(self, path: Path, content: str) -> None:
        temporary = self._temporary_path(path)
        try:
            temporary.write_text(content, encoding='utf-8')
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _atomic_json(self, path: Path, value: dict) -> None:
        self._atomic_text(path, json.dumps(value, ensure_ascii=False, indent=2) + '\n')

    def _atomic_image(self, path: Path, image: np.ndarray) -> None:
        temporary = self._temporary_path(path, path.suffix)
        try:
            if not cv2.imwrite(str(temporary), image):
                raise RuntimeError(f'cv2.imwrite failed for {temporary}')
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _atomic_mat(self, path: Path, value: dict) -> None:
        temporary = self._temporary_path(path)
        try:
            scio.savemat(str(temporary), value, appendmat=False)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _atomic_instances(
        self,
        path: Path,
        detections: Sequence[Detection],
        image_shape: tuple[int, int],
    ) -> None:
        height, width = image_shape
        masks = np.stack(
            [np.asarray(detection.mask, dtype=np.uint8) for detection in detections],
            axis=0,
        ) if detections else np.zeros((0, height, width), dtype=np.uint8)
        for mask in masks:
            if mask.shape != (height, width):
                raise ValueError(
                    f'detection mask shape {mask.shape} does not match {(height, width)}')
        class_ids = np.asarray([d.class_id for d in detections], dtype=np.int64)
        confidences = np.asarray([d.confidence for d in detections], dtype=np.float32)
        boxes = np.asarray([d.box_xyxy for d in detections], dtype=np.float32).reshape(-1, 4)
        names = np.asarray(
            [f'{d.class_name}_{index}' for index, d in enumerate(detections)],
            dtype='<U64',
        )
        temporary = self._temporary_path(path)
        try:
            with open(temporary, 'wb') as stream:
                np.savez_compressed(
                    stream,
                    masks=masks,
                    class_ids=class_ids,
                    confidences=confidences,
                    boxes_xyxy=boxes,
                    names=names,
                )
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    @staticmethod
    def normalize_depth(
        depth: np.ndarray,
        factor_depth: float,
        depth_is_meters: bool,
    ) -> np.ndarray:
        value = np.asarray(depth)
        if value.ndim == 3:
            value = value[:, :, 0]
        if np.issubdtype(value.dtype, np.floating) or depth_is_meters:
            value = value.astype(np.float32) * float(factor_depth)
        else:
            value = value.astype(np.float32)
        value = np.nan_to_num(value, nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(value, 0, np.iinfo(np.uint16).max).astype(np.uint16)

    def write(
        self,
        color_bgr: np.ndarray,
        depth: np.ndarray,
        intrinsic_matrix: np.ndarray,
        factor_depth: float,
        detections: Sequence[Detection],
        *,
        color_stamp_ns: int,
        depth_stamp_ns: int,
        frame_id: str,
        model_path: str,
        model_sha256: str,
        backend_name: str,
        depth_is_meters: bool = False,
    ) -> bool:
        """Commit one frame.  Return ``False`` when the consumer is still busy."""
        if self.busy:
            return False
        color = np.asarray(color_bgr)
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError(f'color must be HxWx3, got {color.shape}')
        depth_mm = self.normalize_depth(depth, factor_depth, depth_is_meters)
        if depth_mm.shape != color.shape[:2]:
            raise ValueError(
                f'color/depth shape mismatch: {color.shape[:2]} vs {depth_mm.shape}')
        intrinsic = np.asarray(intrinsic_matrix, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(intrinsic)):
            raise ValueError('intrinsic matrix contains non-finite values')

        sequence = self.sequence + 1
        image_shape = (int(color.shape[0]), int(color.shape[1]))
        self._atomic_image(self.shared_dir / 'color.png', color)
        self._atomic_image(self.shared_dir / 'depth.png', depth_mm)
        self._atomic_mat(
            self.shared_dir / 'meta.mat',
            {
                'intrinsic_matrix': intrinsic,
                'factor_depth': np.array([factor_depth], dtype=np.float64),
            },
        )
        self._atomic_instances(self.shared_dir / 'instances.npz', detections, image_shape)
        self._atomic_json(
            self.shared_dir / 'detections.json',
            {
                'frame_sequence': sequence,
                'detections': [
                    {
                        'name': f'{d.class_name}_{index}',
                        'class_id': int(d.class_id),
                        'class_name': d.class_name,
                        'confidence': float(d.confidence),
                        'box_xyxy': np.asarray(d.box_xyxy, dtype=float).tolist(),
                        'mask_pixels': int(np.count_nonzero(d.mask)),
                    }
                    for index, d in enumerate(detections)
                ],
            },
        )
        self._atomic_json(
            self.shared_dir / 'frame_manifest.json',
            {
                'frame_sequence': sequence,
                'source': 'real_piper_yolo_graspnet',
                'color_stamp_ns': int(color_stamp_ns),
                'depth_stamp_ns': int(depth_stamp_ns),
                'stamp_delta_ns': abs(int(color_stamp_ns) - int(depth_stamp_ns)),
                'frame_id': str(frame_id),
                'image_shape': list(image_shape),
                'detection_count': len(detections),
                'factor_depth': float(factor_depth),
                'model_path': str(model_path),
                'model_sha256': str(model_sha256),
                'backend': str(backend_name),
            },
        )
        if self.write_overlay:
            self._atomic_image(
                self.shared_dir / 'yolo_overlay.png',
                draw_detections(color, detections),
            )
        # Commit token must always be last.
        self._atomic_text(self.ready_path, f'{sequence}\n')
        self.sequence = sequence
        return True


def load_instance_archive(path: str, expected_shape: tuple[int, int] | None = None) -> dict:
    """Load and validate the file contract consumed by GraspNet."""
    with np.load(path, allow_pickle=False) as archive:
        required = {'masks', 'class_ids', 'confidences', 'boxes_xyxy', 'names'}
        missing = required.difference(archive.files)
        if missing:
            raise ValueError(f'instances archive missing keys: {sorted(missing)}')
        result = {name: archive[name] for name in required}
    masks = np.asarray(result['masks'])
    count = len(masks)
    if masks.ndim != 3:
        raise ValueError(f'masks must have shape NxHxW, got {masks.shape}')
    if expected_shape is not None and tuple(masks.shape[1:]) != tuple(expected_shape):
        raise ValueError(
            f'mask image shape {masks.shape[1:]} does not match {expected_shape}')
    if any(len(np.asarray(result[key])) != count for key in (
            'class_ids', 'confidences', 'boxes_xyxy', 'names')):
        raise ValueError('instances archive arrays have inconsistent lengths')
    if np.asarray(result['boxes_xyxy']).shape != (count, 4):
        raise ValueError('boxes_xyxy must have shape Nx4')
    return result
