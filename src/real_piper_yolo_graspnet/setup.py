import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'real_piper_yolo_graspnet'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzu',
    maintainer_email='user@example.com',
    description='D435i YOLO instance segmentation bridge for real Piper GraspNet.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd435i_yolo_graspnet_bridge = '
            'real_piper_yolo_graspnet.d435i_yolo_graspnet_bridge:main',
        ],
    },
)
