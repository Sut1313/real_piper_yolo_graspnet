"""handeye_tf_publisher：读取 eye-to-hand 标定文件，发布相机→机械臂静态 TF。

执行顺序（需求规定，日志逐步打点）：
    ① 读取标定文件（路径来自节点参数 handeye_yaml，不写死）
    ② 从文件查找对应参数（R_cam2base / t_cam2base；帧名优先取文件键
       parent_frame / child_frame，缺省回退节点参数）
    ③ 校验（旋转 det≈+1；冗余 4x4 T_cam2base 与 R/t 一致性核对）
    ④ 发布 base_link ← camera_link 静态变换（eye-to-hand 相机与基座刚性
       固定，用 StaticTransformBroadcaster）

标定文件格式（camera_data/handeye_eye_to_hand.yaml，调研文档 §4）：
    R_cam2base: 3x3 旋转矩阵（det=+1 正交），camera→base 姿态
    t_cam2base: 米制平移，相机原点在基座系中的位置
    T_cam2base: 冗余 4x4 齐次矩阵（与 R/t 一致，仅用于交叉核对）
    description: 描述（可选）
    parent_frame / child_frame: 可选；缺省用节点参数 base_link / camera_link

语义：p_base = R_cam2base @ p_cam + t_cam2base —— TransformStamped 的
header.frame_id=base_link、child_frame_id=camera_link 即该刚体变换。
"""

import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster


def load_handeye_tf(yaml_path: str,
                    default_parent: str = 'base_link',
                    default_child: str = 'camera_link') -> dict:
    """核心逻辑（纯函数，可单测）：文件 → 参数 → TransformStamped 数据。

    顺序：读取文件 → 从文件查找 R_cam2base/t_cam2base（及可选帧名键）→
    校验 → 组 TransformStamped 数据。

    Args:
        yaml_path: eye-to-hand 标定文件路径。
        default_parent / default_child: 帧名回退值（文件无对应键时使用）。

    Returns:
        dict：{'transform': TransformStamped, 'R': np.ndarray(3,3),
               't': np.ndarray(3,), 'parent': str, 'child': str}。
        文件缺失/解析失败/字段不完整：抛 ValueError（调用侧 fail-loud，
        不发布错误 TF——缺失的 TF 好过错误的 TF）。
    """
    # ① 读取标定文件
    try:
        with open(yaml_path) as f:
            d = yaml.safe_load(f)
    except OSError as exc:
        raise ValueError(f'手眼标定文件读取失败: {yaml_path} ({exc})') from exc
    except yaml.YAMLError as exc:
        raise ValueError(f'手眼标定文件 YAML 解析失败: {yaml_path} ({exc})') from exc

    # ② 从文件查找对应参数
    if 'R_cam2base' not in d or 't_cam2base' not in d:
        raise ValueError(f'标定文件缺少 R_cam2base/t_cam2base 字段: {yaml_path}')
    R = np.asarray(d['R_cam2base'], dtype=np.float64).reshape(3, 3)
    t = np.asarray(d['t_cam2base'], dtype=np.float64).reshape(3)
    # 帧名：文件键优先（需求：参数从文件读取），缺省回退节点参数
    parent = str(d.get('parent_frame', default_parent))
    child = str(d.get('child_frame', default_child))

    # ③ 校验
    det = float(np.linalg.det(R))
    if not np.isclose(det, 1.0, atol=1e-6):
        raise ValueError(f'R_cam2base det={det:.8f}，非正交归一真旋转（期望 +1），'
                         f'拒绝发布')
    if 'T_cam2base' in d:
        T = np.asarray(d['T_cam2base'], dtype=np.float64).reshape(4, 4)
        R_ok = np.allclose(T[:3, :3], R, atol=1e-6)
        t_ok = np.allclose(T[:3, 3], t, atol=1e-6)
        if not (R_ok and t_ok):
            raise ValueError('冗余 T_cam2base 与 R_cam2base/t_cam2base 不一致，'
                             '拒绝发布（数据自相矛盾）')

    # ④ 组 TransformStamped（旋转矩阵 → 四元数 [x,y,z,w]）
    q = Rotation.from_matrix(R).as_quat()
    ts = TransformStamped()
    ts.header.stamp = rclpy.time.Time().to_msg()
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = float(t[0])
    ts.transform.translation.y = float(t[1])
    ts.transform.translation.z = float(t[2])
    ts.transform.rotation.x = float(q[0])
    ts.transform.rotation.y = float(q[1])
    ts.transform.rotation.z = float(q[2])
    ts.transform.rotation.w = float(q[3])

    return {'transform': ts, 'R': R, 't': t, 'parent': parent, 'child': child}


class HandeyeTfPublisher(Node):
    """发布 base_link ← camera_link 静态 TF 的节点。"""

    def __init__(self):
        super().__init__('handeye_tf_publisher')
        self.declare_parameter('handeye_yaml', '')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_link')

        yaml_path = str(self.get_parameter('handeye_yaml').value)
        if not yaml_path:
            self.get_logger().error('handeye_yaml 参数为空（必须指定标定文件路径），退出')
            raise SystemExit(1)

        # ① 读取标定文件
        self.get_logger().info(f'[1/3] 读取标定文件: {yaml_path}')
        # ②③④ 查找参数 → 校验 → 组数据（load_handeye_tf 内部完成）
        try:
            data = load_handeye_tf(
                yaml_path,
                default_parent=str(self.get_parameter('parent_frame').value),
                default_child=str(self.get_parameter('child_frame').value),
            )
        except ValueError as exc:
            self.get_logger().error(f'标定文件处理失败（fail-loud，不发布错误 TF）: {exc}')
            raise SystemExit(1) from exc

        R, t = data['R'], data['t']
        parent, child = data['parent'], data['child']
        self.get_logger().info(
            f'[2/3] 从文件查找参数: R_cam2base det={np.linalg.det(R):.6f}\n'
            f'      R_cam2base =\n{R}\n'
            f'      t_cam2base = {t.tolist()} m')
        self.get_logger().info(f'[3/3] 发布静态 TF: {parent} ← {child} '
                               f'(translation={t.tolist()})')

        self._data = data
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(data['transform'])

        # 周期性重发（static 广播在部分 DDS 配置下需要保活；频率低、开销可忽略）
        self._timer = self.create_timer(5.0, self._republish)
        self.get_logger().info('handeye_tf_publisher 就绪（每 5s 重发静态 TF）')

    def _republish(self):
        self._broadcaster.sendTransform(self._data['transform'])


def main():
    rclpy.init()
    node = HandeyeTfPublisher()
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
