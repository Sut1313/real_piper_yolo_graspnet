"""Small dependency-light YOLOv8-Seg ONNX runtime.

The ROS host does not necessarily have ``onnxruntime`` installed.  The detector
therefore prefers ONNX Runtime when available and otherwise uses OpenCV DNN.
Both backends feed the same explicit YOLOv8 segmentation decoder.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import sys
from typing import Sequence

import cv2
import numpy as np


TABLETOP_CLASS_NAMES = (
    'pen', 'pencil', 'marker', 'eraser', 'ruler', 'scissors', 'stapler',
    'tape', 'sharpener', 'block', 'cup', 'bottle', 'book', 'phone',
    'mouse', 'remote',
)


@dataclass(frozen=True)
class LetterboxTransform:
    original_height: int
    original_width: int
    input_size: int
    scale: float
    left: int
    top: int
    resized_width: int
    resized_height: int


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    box_xyxy: np.ndarray
    mask: np.ndarray


def model_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def letterbox(image: np.ndarray, input_size: int = 640) -> tuple[np.ndarray, LetterboxTransform]:
    """Resize with symmetric padding and return an NCHW RGB float tensor."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f'expected HxWx3 image, got {image.shape}')
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError('image dimensions must be positive')

    scale = min(input_size / width, input_size / height)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    left = (input_size - resized_width) // 2
    top = (input_size - resized_height) // 2

    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    padded[top:top + resized_height, left:left + resized_width] = resized
    tensor = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
    tensor = np.ascontiguousarray(tensor[np.newaxis])
    return tensor, LetterboxTransform(
        original_height=height,
        original_width=width,
        input_size=input_size,
        scale=scale,
        left=left,
        top=top,
        resized_width=resized_width,
        resized_height=resized_height,
    )


def _box_iou(one: np.ndarray, many: np.ndarray) -> np.ndarray:
    if len(many) == 0:
        return np.empty((0,), dtype=np.float32)
    upper_left = np.maximum(one[:2], many[:, :2])
    lower_right = np.minimum(one[2:], many[:, 2:])
    intersection = np.prod(np.maximum(lower_right - upper_left, 0.0), axis=1)
    one_area = max(float((one[2] - one[0]) * (one[3] - one[1])), 0.0)
    many_area = np.maximum(many[:, 2] - many[:, 0], 0.0) * np.maximum(
        many[:, 3] - many[:, 1], 0.0)
    union = one_area + many_area - intersection
    return np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)


def class_aware_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    iou_threshold: float,
) -> list[int]:
    """Pure NumPy class-aware non-maximum suppression."""
    kept: list[int] = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        order = indices[np.argsort(scores[indices])[::-1]]
        while len(order):
            current = int(order[0])
            kept.append(current)
            if len(order) == 1:
                break
            remaining = order[1:]
            order = remaining[_box_iou(boxes[current], boxes[remaining]) <= iou_threshold]
    return sorted(kept, key=lambda index: float(scores[index]), reverse=True)


def _sigmoid(values: np.ndarray) -> np.ndarray:
    values = np.clip(values, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-values))


def _restore_mask(mask_input: np.ndarray, transform: LetterboxTransform) -> np.ndarray:
    y0 = transform.top
    y1 = y0 + transform.resized_height
    x0 = transform.left
    x1 = x0 + transform.resized_width
    cropped = mask_input[y0:y1, x0:x1]
    if cropped.size == 0:
        return np.zeros(
            (transform.original_height, transform.original_width), dtype=np.float32)
    return cv2.resize(
        cropped,
        (transform.original_width, transform.original_height),
        interpolation=cv2.INTER_LINEAR,
    )


def decode_yolov8_segmentation(
    output0: np.ndarray,
    output1: np.ndarray,
    transform: LetterboxTransform,
    class_names: Sequence[str] = TABLETOP_CLASS_NAMES,
    confidence_threshold: float = 0.35,
    nms_iou_threshold: float = 0.50,
    mask_threshold: float = 0.50,
    max_detections: int = 100,
) -> list[Detection]:
    """Decode Ultralytics YOLOv8-Seg exports (boxes/classes/mask coefficients + prototypes)."""
    predictions = np.asarray(output0)
    prototypes = np.asarray(output1)
    if predictions.ndim == 3:
        predictions = predictions[0]
    if prototypes.ndim == 4:
        prototypes = prototypes[0]
    if predictions.ndim != 2 or prototypes.ndim != 3:
        raise ValueError(
            f'unexpected YOLO outputs: predictions={predictions.shape}, '
            f'prototypes={prototypes.shape}')

    mask_channels = int(prototypes.shape[0])
    expected_channels = 4 + len(class_names) + mask_channels
    if predictions.shape[0] == expected_channels:
        predictions = predictions.T
    if predictions.shape[1] != expected_channels:
        raise ValueError(
            f'YOLO output has {predictions.shape[1]} channels, expected {expected_channels}')

    class_scores = predictions[:, 4:4 + len(class_names)]
    class_ids = np.argmax(class_scores, axis=1).astype(np.int64)
    scores = class_scores[np.arange(len(class_scores)), class_ids]
    valid = np.flatnonzero(scores >= confidence_threshold)
    if len(valid) == 0:
        return []

    selected = predictions[valid]
    scores = scores[valid].astype(np.float32)
    class_ids = class_ids[valid]
    xywh = selected[:, :4].astype(np.float32)
    boxes = np.empty_like(xywh)
    boxes[:, 0] = xywh[:, 0] - xywh[:, 2] / 2.0
    boxes[:, 1] = xywh[:, 1] - xywh[:, 3] / 2.0
    boxes[:, 2] = xywh[:, 0] + xywh[:, 2] / 2.0
    boxes[:, 3] = xywh[:, 1] + xywh[:, 3] / 2.0
    coefficients = selected[:, 4 + len(class_names):].astype(np.float32)

    keep = class_aware_nms(boxes, scores, class_ids, nms_iou_threshold)[:max_detections]
    if not keep:
        return []

    proto_flat = prototypes.astype(np.float32).reshape(mask_channels, -1)
    detections: list[Detection] = []
    for index in keep:
        mask_small = _sigmoid(coefficients[index] @ proto_flat).reshape(prototypes.shape[1:])
        mask_input = cv2.resize(
            mask_small,
            (transform.input_size, transform.input_size),
            interpolation=cv2.INTER_LINEAR,
        )
        box_input = boxes[index]
        x0 = max(0, min(transform.input_size, int(np.floor(box_input[0]))))
        y0 = max(0, min(transform.input_size, int(np.floor(box_input[1]))))
        x1 = max(0, min(transform.input_size, int(np.ceil(box_input[2]))))
        y1 = max(0, min(transform.input_size, int(np.ceil(box_input[3]))))
        box_crop = np.zeros_like(mask_input, dtype=np.float32)
        if x1 > x0 and y1 > y0:
            box_crop[y0:y1, x0:x1] = mask_input[y0:y1, x0:x1]
        restored = _restore_mask(box_crop, transform)
        binary_mask = restored >= mask_threshold

        box_original = np.array([
            (box_input[0] - transform.left) / transform.scale,
            (box_input[1] - transform.top) / transform.scale,
            (box_input[2] - transform.left) / transform.scale,
            (box_input[3] - transform.top) / transform.scale,
        ], dtype=np.float32)
        box_original[[0, 2]] = np.clip(box_original[[0, 2]], 0, transform.original_width)
        box_original[[1, 3]] = np.clip(box_original[[1, 3]], 0, transform.original_height)
        class_id = int(class_ids[index])
        detections.append(Detection(
            class_id=class_id,
            class_name=str(class_names[class_id]),
            confidence=float(scores[index]),
            box_xyxy=box_original,
            mask=binary_mask,
        ))
    return detections


class _OnnxRuntimeBackend:
    name = 'onnxruntime'

    def __init__(self, model_path: str):
        import onnxruntime as ort

        available = ort.get_available_providers()
        preferred = [
            provider for provider in ('CUDAExecutionProvider', 'CPUExecutionProvider')
            if provider in available
        ]
        self.session = ort.InferenceSession(model_path, providers=preferred or available)
        self.input_name = self.session.get_inputs()[0].name

    def run(self, tensor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        outputs = self.session.run(None, {self.input_name: tensor})
        if len(outputs) != 2:
            raise RuntimeError(f'expected two YOLO outputs, got {len(outputs)}')
        return outputs[0], outputs[1]


class _OpenCvBackend:
    name = 'opencv-dnn'

    def __init__(self, model_path: str):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.output_names = self.net.getUnconnectedOutLayersNames()

    def run(self, tensor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.net.setInput(tensor)
        outputs = self.net.forward(self.output_names)
        if not isinstance(outputs, (list, tuple)):
            outputs = [outputs]
        if len(outputs) != 2:
            raise RuntimeError(f'expected two YOLO outputs, got {len(outputs)}')
        by_rank = sorted(outputs, key=lambda value: np.asarray(value).ndim)
        # output0 is rank 3; prototypes are rank 4.
        return by_rank[0], by_rank[1]

    def validate(self, input_size: int) -> None:
        try:
            self.run(np.zeros((1, 3, input_size, input_size), dtype=np.float32))
        except cv2.error as exc:
            raise RuntimeError(
                'OpenCV DNN cannot execute this YOLO ONNX model; install ONNX '
                'Runtime or set onnxruntime_site_packages') from exc


class YoloSegmentationDetector:
    def __init__(
        self,
        model_path: str,
        class_names: Sequence[str] = TABLETOP_CLASS_NAMES,
        confidence_threshold: float = 0.35,
        nms_iou_threshold: float = 0.50,
        mask_threshold: float = 0.50,
        input_size: int = 640,
        backend: str = 'auto',
        onnxruntime_site_packages: str = '',
    ):
        path = str(Path(model_path).expanduser().resolve())
        if not Path(path).is_file():
            raise FileNotFoundError(f'YOLO model does not exist: {path}')
        self.model_path = path
        self.model_sha256 = model_sha256(path)
        self.class_names = tuple(class_names)
        self.confidence_threshold = float(confidence_threshold)
        self.nms_iou_threshold = float(nms_iou_threshold)
        self.mask_threshold = float(mask_threshold)
        self.input_size = int(input_size)

        runtime_site = str(onnxruntime_site_packages).strip()
        if runtime_site:
            runtime_path = str(Path(runtime_site).expanduser().resolve())
            if Path(runtime_path).is_dir() and runtime_path not in sys.path:
                # NumPy/OpenCV are imported above from the ROS environment.
                # Appending (rather than prepending) lets an isolated training
                # venv provide ONNX Runtime without replacing ROS binary deps.
                sys.path.append(runtime_path)

        if backend not in ('auto', 'onnxruntime', 'opencv'):
            raise ValueError(f'unknown backend {backend!r}')
        if backend in ('auto', 'onnxruntime'):
            try:
                self._backend = _OnnxRuntimeBackend(path)
            except (ImportError, OSError, RuntimeError):
                if backend == 'onnxruntime':
                    raise
                self._backend = _OpenCvBackend(path)
        else:
            self._backend = _OpenCvBackend(path)
        if isinstance(self._backend, _OpenCvBackend):
            self._backend.validate(self.input_size)
        self.backend_name = self._backend.name

    def predict(self, image_bgr: np.ndarray) -> list[Detection]:
        tensor, transform = letterbox(image_bgr, self.input_size)
        output0, output1 = self._backend.run(tensor)
        return decode_yolov8_segmentation(
            output0,
            output1,
            transform,
            class_names=self.class_names,
            confidence_threshold=self.confidence_threshold,
            nms_iou_threshold=self.nms_iou_threshold,
            mask_threshold=self.mask_threshold,
        )


def draw_detections(image_bgr: np.ndarray, detections: Sequence[Detection]) -> np.ndarray:
    overlay = image_bgr.copy()
    colors = ((0, 180, 255), (0, 255, 0), (255, 120, 0), (255, 0, 180))
    for index, detection in enumerate(detections):
        color = colors[index % len(colors)]
        mask = np.asarray(detection.mask, dtype=bool)
        if mask.shape == overlay.shape[:2]:
            tinted = np.zeros_like(overlay)
            tinted[:, :] = color
            overlay[mask] = cv2.addWeighted(overlay, 0.55, tinted, 0.45, 0)[mask]
        x0, y0, x1, y1 = np.rint(detection.box_xyxy).astype(int)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), color, 2)
        cv2.putText(
            overlay,
            f'{detection.class_name} {detection.confidence:.2f}',
            (max(0, x0), max(18, y0 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return overlay
