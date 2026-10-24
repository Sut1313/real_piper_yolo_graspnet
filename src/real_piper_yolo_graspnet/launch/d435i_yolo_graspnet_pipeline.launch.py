"""Start D435i, YOLO-GraspNet bridge, and the proven hand-eye TF publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('real_piper_yolo_graspnet')
    default_root = os.path.expanduser('~/agrobot_code')
    shared_dir = LaunchConfiguration('shared_dir')
    model_path = LaunchConfiguration('model_path')
    target_class = LaunchConfiguration('target_class')
    calibration_yaml = LaunchConfiguration('calibration_yaml')
    handeye_yaml = LaunchConfiguration('handeye_yaml')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'shared_dir',
            default_value=os.path.join(
                default_root, 'real_piper_yolo_graspnet/shared_camera_data'),
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.environ.get(
                'TABLETOP_YOLO_MODEL',
                os.path.join(
                    default_root,
                    'tabletop_yolov8seg/models/tabletop_yolov8s_seg_v2.onnx',
                ),
            ),
        ),
        DeclareLaunchArgument('target_class', default_value='cup'),
        DeclareLaunchArgument(
            'calibration_yaml',
            default_value=os.path.join(
                default_root, 'real_piper_yolo_graspnet/camera_data/calibration.yaml'),
        ),
        DeclareLaunchArgument(
            'handeye_yaml',
            default_value=os.path.join(
                default_root,
                'real_piper_yolo_graspnet/camera_data/handeye_eye_to_hand.yaml'),
        ),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('child_frame', default_value='camera_link'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'camera_name': 'camera',
                'serial_no': '',
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'align_depth.enable': True,
                'pointcloud.enable': False,
                'rgb_camera.color_profile': '640,480,30',
                'rgb_camera.enable_auto_exposure': True,
                'depth_module.depth_profile': '640,480,30',
                'depth_module.enable_auto_exposure': True,
            }],
            output='screen',
        ),

        Node(
            package='real_piper_yolo_graspnet',
            executable='d435i_yolo_graspnet_bridge',
            name='d435i_yolo_graspnet_bridge',
            parameters=[
                os.path.join(package_share, 'config', 'tabletop_yolo.yaml'),
                {
                    'shared_dir': shared_dir,
                    'model_path': model_path,
                    'target_class': target_class,
                    'calibration_yaml': calibration_yaml,
                    'color_topic': '/camera/camera/color/image_raw',
                    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                    'camera_info_topic': '/camera/camera/color/camera_info',
                },
            ],
            output='screen',
        ),

        Node(
            package='real_piper_perception',
            executable='handeye_tf_publisher',
            name='handeye_tf_publisher',
            parameters=[{
                'handeye_yaml': handeye_yaml,
                'parent_frame': parent_frame,
                'child_frame': child_frame,
            }],
            output='screen',
        ),
    ])
