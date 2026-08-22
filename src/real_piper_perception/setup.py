import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'real_piper_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # M6（fixed 2026-08-07）：scripts/ 打包进 share（launch 推导的 ssik
        # 路径在常规拷贝安装下同样可用）。
        (os.path.join('share', package_name, 'scripts'),
         [os.path.join('scripts', 'piper_ssik_ik.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzu',
    maintainer_email='user@example.com',
    description='D435i to GraspNet shared-folder bridge for real Piper grasping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd435i_graspnet_bridge = real_piper_perception.d435i_graspnet_bridge:main',
            'grasp_executor = real_piper_perception.grasp_executor:main',
            'handeye_tf_publisher = real_piper_perception.handeye_tf_publisher:main',
        ],
    },
)
