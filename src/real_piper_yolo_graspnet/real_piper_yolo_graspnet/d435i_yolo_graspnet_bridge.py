#!/usr/bin/env python3
"""Synchronize D435i RGB-D, run YOLOv8-Seg, and commit GraspNet frames."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal

import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .frame_writer import FrameBundleWriter
from .yolo_detector import Detection, TABLETOP_CLASS_NAMES, YoloSegmentationDetector


def select_target_detections(
    detections: list[Detection],
    target_class: str,
    depth: np.ndarray,
    factor_depth: float,
    depth_is_meters: bool,
    min_depth_m: float,
    max_depth_m: float,
    min_mask_pixels: int,
    min_valid_depth_points: int,
    fail_on_multiple: bool,
) -> tuple[list[Detection], str]:
    """Apply fail-closed target and depth gates before publishing a frame."""
    if np.issubdtype(depth.dtype, np.floating) or depth_is_meters:
        depth_m = depth.astype(np.float32)
    else:
        depth_m = depth.astype(np.float32) / float(factor_depth)
    if depth_m.ndim == 3:
        depth_m = depth_m[:, :, 0]
    valid_depth = np.isfinite(depth_m) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)

    accepted: list[Detection] = []
    rejected: list[str] = []
    for detection in detections:
        if target_class and detection.class_name != target_class:
            continue
        mask = np.asarray(detection.mask, dtype=bool)
        pixels = int(np.count_nonzero(mask))
        valid_points = int(np.count_nonzero(mask & valid_depth))
        if pixels < min_mask_pixels:
            rejected.append(f'{detection.class_name}:mask={pixels}')
            continue
        if valid_points < min_valid_depth_points:
            rejected.append(f'{detection.class_name}:depth={valid_points}')
            continue
        accepted.append(detection)

    accepted.sort(key=lambda item: item.confidence, reverse=True)
    if not accepted:
        detail = ','.join(rejected) if rejected else 'no_matching_detection'
        return [], detail
    if len(accepted) > 1 and fail_on_multiple:
        return [], f'ambiguous_target_count={len(accepted)}'
    return [accepted[0]], 'selected'


class D435iYoloGraspNetBridge(Node):
    def __init__(self):
        super().__init__('d435i_yolo_graspnet_bridge')
        default_root = Path('~/agrobot_code').expanduser()
        self.declare_parameter(
            'shared_dir', str(default_root / 'real_piper_yolo_graspnet/shared_camera_data'))
        self.declare_parameter(
            'model_path',
            os.environ.get(
                'TABLETOP_YOLO_MODEL',
                str(default_root / 'tabletop_yolov8seg/models/tabletop_yolov8s_seg_v2.onnx'),
            ),
        )
        self.declare_parameter('backend', 'auto')
        self.declare_parameter(
            'onnxruntime_site_packages',
            os.environ.get(
                'YOLO_ONNXRUNTIME_SITE_PACKAGES',
                str(default_root / 'tabletop_yolov8seg/.venv/lib/python3.10/site-packages'),
            ),
        )
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('nms_iou_threshold', 0.50)
        self.declare_parameter('mask_threshold', 0.50)
        self.declare_parameter('target_class', 'cup')
        self.declare_parameter('fail_on_multiple', True)
        self.declare_parameter('min_mask_pixels', 200)
        self.declare_parameter('min_valid_depth_points', 120)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 1.20)
        self.declare_parameter('factor_depth', 1000.0)
        self.declare_parameter('depth_is_meters', False)
        self.declare_parameter('write_overlay', True)
        self.declare_parameter('sync_queue_size', 8)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter(
            'calibration_yaml',
            str(default_root / 'real_piper_yolo_graspnet/camera_data/calibration.yaml'),
        )
        self.declare_parameter('use_calibration_intrinsics', True)

        self.factor_depth = float(self.get_parameter('factor_depth').value)
        self.depth_is_meters = bool(self.get_parameter('depth_is_meters').value)
        self.target_class = str(self.get_parameter('target_class').value).strip()
        if self.target_class and self.target_class not in TABLETOP_CLASS_NAMES:
            raise ValueError(
                f'target_class={self.target_class!r} is not one of {TABLETOP_CLASS_NAMES}')
        self.fail_on_multiple = bool(self.get_parameter('fail_on_multiple').value)
        self.min_mask_pixels = int(self.get_parameter('min_mask_pixels').value)
        self.min_valid_depth_points = int(
            self.get_parameter('min_valid_depth_points').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.use_calibration_intrinsics = bool(
            self.get_parameter('use_calibration_intrinsics').value)

        model_path = str(Path(str(self.get_parameter('model_path').value)).expanduser())
        self.detector = YoloSegmentationDetector(
            model_path=model_path,
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').value),
            nms_iou_threshold=float(self.get_parameter('nms_iou_threshold').value),
            mask_threshold=float(self.get_parameter('mask_threshold').value),
            backend=str(self.get_parameter('backend').value),
            onnxruntime_site_packages=str(
                self.get_parameter('onnxruntime_site_packages').value),
        )
        self.writer = FrameBundleWriter(
            str(self.get_parameter('shared_dir').value),
            write_overlay=bool(self.get_parameter('write_overlay').value),
        )
        self.cv_bridge = CvBridge()
        self.latest_camera_info: CameraInfo | None = None
        self.calibration_matrix: np.ndarray | None = None
        if self.use_calibration_intrinsics:
            self.calibration_matrix = self._load_calibration(
                str(self.get_parameter('calibration_yaml').value))

        self.detections_publisher = self.create_publisher(
            String, '/real_piper_yolo/detections', 10)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        color_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter('color_topic').value),
            qos_profile=qos_profile_sensor_data,
        )
        depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter('depth_topic').value),
            qos_profile=qos_profile_sensor_data,
        )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_subscriber, depth_subscriber],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value),
        )
        synchronizer.registerCallback(self._rgbd_callback)
        self._color_subscriber = color_subscriber
        self._depth_subscriber = depth_subscriber
        self._synchronizer = synchronizer

        self.get_logger().info(
            'Real Piper YOLO GraspNet bridge started:\n'
            f'  model:       {self.detector.model_path}\n'
            f'  sha256:      {self.detector.model_sha256}\n'
            f'  backend:     {self.detector.backend_name}\n'
            f'  target:      {self.target_class or "any"}\n'
            f'  shared_dir:  {self.writer.shared_dir}\n'
            f'  fail_multi:  {self.fail_on_multiple}'
        )

    @staticmethod
    def _load_calibration(path: str) -> np.ndarray:
        calibration_path = Path(path).expanduser().resolve()
        with open(calibration_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        entry = data.get('camera_matrix')
        matrix = entry.get('data') if isinstance(entry, dict) else entry
        if matrix is None:
            matrix = data.get('K')
        value = np.asarray(matrix, dtype=np.float64)
        if value.size != 9 or not np.all(np.isfinite(value)):
            raise ValueError(f'invalid camera matrix in {calibration_path}')
        value = value.reshape(3, 3)
        if abs(float(value[2, 2])) < 1e-12:
            raise ValueError(f'camera K[2,2] is zero in {calibration_path}')
        return value / value[2, 2]

    @staticmethod
    def _camera_info_matrix(info: CameraInfo) -> np.ndarray:
        value = np.asarray(info.k, dtype=np.float64)
        if value.size != 9:
            raise ValueError('CameraInfo.k must contain nine values')
        return value.reshape(3, 3)

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.latest_camera_info = message

    def _rgbd_callback(self, color_message: Image, depth_message: Image) -> None:
        if self.writer.busy:
            return
        if self.latest_camera_info is None:
            self.get_logger().warning(
                'Waiting for CameraInfo before YOLO inference.', throttle_duration_sec=5.0)
            return
        try:
            color = self.cv_bridge.imgmsg_to_cv2(color_message, desired_encoding='bgr8')
            depth = self.cv_bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            color = np.asarray(color)
            depth = np.asarray(depth)
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            if color.shape[:2] != depth.shape:
                raise ValueError(
                    f'aligned color/depth shape mismatch: {color.shape[:2]} vs {depth.shape}')

            detections = self.detector.predict(color)
            selected, reason = select_target_detections(
                detections,
                self.target_class,
                depth,
                self.factor_depth,
                self.depth_is_meters,
                self.min_depth_m,
                self.max_depth_m,
                self.min_mask_pixels,
                self.min_valid_depth_points,
                self.fail_on_multiple,
            )
            intrinsic = (
                self.calibration_matrix
                if self.calibration_matrix is not None
                else self._camera_info_matrix(self.latest_camera_info)
            )
            written = self.writer.write(
                color,
                depth,
                intrinsic,
                self.factor_depth,
                selected,
                color_stamp_ns=self._stamp_ns(color_message),
                depth_stamp_ns=self._stamp_ns(depth_message),
                frame_id=color_message.header.frame_id,
                model_path=self.detector.model_path,
                model_sha256=self.detector.model_sha256,
                backend_name=self.detector.backend_name,
                depth_is_meters=self.depth_is_meters,
            )
            if not written:
                return
            payload = {
                'frame_sequence': self.writer.sequence,
                'target_class': self.target_class,
                'raw_detection_count': len(detections),
                'selected_count': len(selected),
                'selection': reason,
                'selected': [
                    {
                        'class_name': detection.class_name,
                        'confidence': detection.confidence,
                        'valid_mask_pixels': int(np.count_nonzero(detection.mask)),
                    }
                    for detection in selected
                ],
            }
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            self.detections_publisher.publish(message)
            self.get_logger().info(
                f'Committed frame {self.writer.sequence}: raw={len(detections)} '
                f'selected={len(selected)} ({reason})',
                throttle_duration_sec=1.0,
            )
        except (CvBridgeError, ValueError, OSError, RuntimeError) as exc:
            self.get_logger().error(
                f'YOLO RGB-D frame failed: {exc}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = D435iYoloGraspNetBridge()

    def handle_signal(_signal, _frame):
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
