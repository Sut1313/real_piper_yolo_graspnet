"""Start D435i driver, the GraspNet shared-folder bridge, and handeye TF.

handeye_tf_publisher：读取 eye-to-hand 标定文件并发布 base_link ← camera_link
静态 TF（连通相机 TF 树与机器人 TF 树）。帧名可经标定文件键或 launch 参数覆盖。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shared_dir = LaunchConfiguration('shared_dir')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    calibration_yaml = LaunchConfiguration('calibration_yaml')
    handeye_yaml = LaunchConfiguration('handeye_yaml')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'shared_dir',
            default_value=os.path.expanduser('~/real_piper_graspnet/shared_camera_data'),
        ),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'calibration_yaml',
            default_value=os.path.expanduser(
                '~/real_piper_graspnet/camera_data/calibration.yaml'),
        ),
        DeclareLaunchArgument(
            'handeye_yaml',
            default_value=os.path.expanduser(
                '~/real_piper_graspnet/camera_data/handeye_eye_to_hand.yaml'),
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
            package='real_piper_perception',
            executable='d435i_graspnet_bridge',
            name='d435i_graspnet_bridge',
            parameters=[{
                'shared_dir': shared_dir,
                'publish_rate_hz': publish_rate_hz,
                'calibration_yaml': calibration_yaml,
                # 使用项目离线标定 K；旧推理链按 K 直接反投影。
                'use_calibration_intrinsics': True,
                'factor_depth': 1000.0,
                'depth_is_meters': False,
                'color_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
            }],
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
