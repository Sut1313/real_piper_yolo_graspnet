#!/usr/bin/env python3
"""Bridge RealSense D435i ROS images into GraspNet shared-folder frames."""

import json
import hashlib
import os
import signal
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

try:
    import scipy.io as scio
except ImportError:
    scio = None


class D435iGraspNetBridge(Node):
    def __init__(self):
        super().__init__('d435i_graspnet_bridge')

        self.declare_parameter(
            'shared_dir',
            os.path.expanduser('~/real_piper_graspnet/shared_camera_data'),
        )
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('factor_depth', 1000.0)
        self.declare_parameter('depth_is_meters', False)
        self.declare_parameter(
            'calibration_yaml',
            os.path.expanduser('~/real_piper_graspnet/camera_data/calibration.yaml'),
        )
        # Project authority: this K/D was estimated from the same 640x480 SDK color
        # stream and was also used by hand-eye solvePnP. The inference side must use
        # both K and D during deprojection; publishing K alone is insufficient.
        self.declare_parameter('use_calibration_intrinsics', True)

        self.shared_dir = self.get_parameter('shared_dir').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.factor_depth = float(self.get_parameter('factor_depth').value)
        self.depth_is_meters = bool(self.get_parameter('depth_is_meters').value)
        self.calibration_yaml = os.path.expanduser(
            str(self.get_parameter('calibration_yaml').value))
        self.use_calibration_intrinsics = bool(
            self.get_parameter('use_calibration_intrinsics').value)
        self.calibration_matrix = None
        self.calibration_distortion = None
        self.calibration_sha256 = ''
        if self.use_calibration_intrinsics:
            self._load_calibration()

        os.makedirs(self.shared_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.last_publish_time = 0.0

        self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self.publish_to_shared)

        self.get_logger().info(
            'D435i GraspNet bridge started:\n'
            f'  color_topic:       {self.color_topic}\n'
            f'  depth_topic:       {self.depth_topic}\n'
            f'  camera_info_topic: {self.camera_info_topic}\n'
            f'  shared_dir:        {self.shared_dir}\n'
            f'  publish_rate_hz:   {self.publish_rate}\n'
            f'  intrinsics_source: {self._intrinsics_source()}\n'
            f'  calibration_yaml:  {self.calibration_yaml}'
        )

    def _load_calibration(self):
        """Load and validate the project calibration used by GraspNet meta.mat."""
        with open(self.calibration_yaml, 'r') as file:
            data = yaml.safe_load(file) or {}
        matrix_entry = data.get('camera_matrix')
        matrix = (matrix_entry.get('data') if isinstance(matrix_entry, dict)
                  else matrix_entry)
        if matrix is None:
            matrix = data.get('K')
        distortion_entry = data.get('distortion_coefficients')
        distortion = (distortion_entry.get('data')
                      if isinstance(distortion_entry, dict) else distortion_entry)
        if distortion is None:
            distortion = data.get('dist_coeff', data.get('D', []))
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.size != 9 or not np.all(np.isfinite(matrix)):
            raise ValueError(f'calibration camera matrix is invalid: {self.calibration_yaml}')
        matrix = matrix.reshape(3, 3)
        if abs(float(matrix[2, 2])) < 1e-12:
            raise ValueError(f'calibration camera matrix K[2,2] is zero: {self.calibration_yaml}')
        matrix /= matrix[2, 2]
        distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion.size and not np.all(np.isfinite(distortion)):
            raise ValueError(f'calibration distortion is invalid: {self.calibration_yaml}')
        with open(self.calibration_yaml, 'rb') as file:
            self.calibration_sha256 = hashlib.sha256(file.read()).hexdigest()
        self.calibration_matrix = matrix
        self.calibration_distortion = distortion

    def _intrinsics_source(self):
        return 'calibration.yaml' if self.use_calibration_intrinsics else 'ros_camera_info'

    @staticmethod
    def _camera_info_matrix(info):
        k = np.asarray(info.k, dtype=np.float64)
        if k.size != 9:
            raise ValueError('ROS CameraInfo K must contain 9 values')
        return k.reshape(3, 3)

    def color_callback(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Color conversion failed: {exc}')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')

    def info_callback(self, msg):
        self.latest_camera_info = msg

    def publish_to_shared(self):
        now = time.time()
        if now - self.last_publish_time < 1.0 / max(self.publish_rate, 0.1):
            return
        if self.latest_color is None or self.latest_depth is None or self.latest_camera_info is None:
            self.get_logger().debug('Waiting for D435i color/depth/info...', throttle_duration_sec=5.0)
            return

        try:
            self._write_color()
            self._write_depth()
            self._write_meta()
            self._write_debug_info()
            self._atomic_text_write(os.path.join(self.shared_dir, '.ready'), f'{now}\n')
            self.last_publish_time = now
            self.get_logger().info('Published D435i frame to shared directory', throttle_duration_sec=2.0)
        except Exception as exc:
            self.get_logger().error(f'Failed to publish D435i frame: {exc}', throttle_duration_sec=5.0)

    def _write_color(self):
        self._atomic_imwrite(os.path.join(self.shared_dir, 'color.png'), self.latest_color)

    def _write_depth(self):
        depth = np.asarray(self.latest_depth)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if np.issubdtype(depth.dtype, np.floating) or self.depth_is_meters:
            depth_mm = depth.astype(np.float32) * self.factor_depth
        else:
            depth_mm = depth.astype(np.float32)
        depth_mm = np.nan_to_num(depth_mm, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = np.clip(depth_mm, 0, np.iinfo(np.uint16).max).astype(np.uint16)
        self._atomic_imwrite(os.path.join(self.shared_dir, 'depth.png'), depth_mm)

    def _write_meta(self):
        info = self.latest_camera_info
        runtime_matrix = self._camera_info_matrix(info)
        intrinsic_matrix = (
            self.calibration_matrix.copy()
            if self.use_calibration_intrinsics
            else runtime_matrix
        )
        meta = {
            'intrinsic_matrix': intrinsic_matrix,
            'factor_depth': np.array([self.factor_depth], dtype=np.float64),
        }
        if scio is None:
            self._atomic_text_write(
                os.path.join(self.shared_dir, 'meta.json'),
                json.dumps({
                    'intrinsic_matrix': intrinsic_matrix.tolist(),
                    'factor_depth': self.factor_depth,
                }, indent=2),
            )
            return
        path = os.path.join(self.shared_dir, 'meta.mat')
        tmp_path = f'{path}.tmp'
        scio.savemat(tmp_path, meta, appendmat=False)
        os.replace(tmp_path, path)

    def _write_debug_info(self):
        info = self.latest_camera_info
        runtime_matrix = self._camera_info_matrix(info)
        content = '\n'.join([
            f'Timestamp: {time.strftime("%Y-%m-%d %H:%M:%S")}',
            f'Color topic: {self.color_topic}',
            f'Depth topic: {self.depth_topic}',
            f'CameraInfo topic: {self.camera_info_topic}',
            f'Width: {info.width}, Height: {info.height}',
            f'Intrinsics source: {self._intrinsics_source()}',
            f'Calibration yaml: {self.calibration_yaml}',
            f'Calibration SHA256: {self.calibration_sha256}',
            f'Published K: {self.calibration_matrix.tolist() if self.use_calibration_intrinsics else runtime_matrix.tolist()}',
            f'Published D: {self.calibration_distortion.tolist() if self.use_calibration_intrinsics else list(info.d)}',
            f'Runtime CameraInfo K: {runtime_matrix.tolist()}',
            f'Runtime distortion model: {info.distortion_model}',
            f'Runtime CameraInfo D: {list(info.d)}',
        ]) + '\n'
        self._atomic_text_write(os.path.join(self.shared_dir, 'camera_info.txt'), content)

    def _atomic_imwrite(self, path, image):
        tmp_path = f'{path}.tmp.{os.getpid()}.{time.time_ns()}.png'
        try:
            if not cv2.imwrite(tmp_path, image):
                raise RuntimeError(f'cv2.imwrite failed for {tmp_path}')
            os.replace(tmp_path, path)
        finally:
            if os.path.exists(tmp_path):
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass

    def _atomic_text_write(self, path, content):
        tmp_path = f'{path}.tmp.{os.getpid()}.{time.time_ns()}'
        with open(tmp_path, 'w') as file:
            file.write(content)
        os.replace(tmp_path, path)


def main(args=None):
    rclpy.init(args=args)
    node = D435iGraspNetBridge()

    def handle_sigint(_sig, _frame):
        node.get_logger().info('Stopping D435i GraspNet bridge...')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_sigint)
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
