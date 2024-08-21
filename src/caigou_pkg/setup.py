from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'caigou_pkg'
submodules = "caigou_pkg/yolov5_deepsort", "caigou_pkg/yolov5_deepsort/models","caigou_pkg/yolov5_deepsort/deep_sort","caigou_pkg/yolov5_deepsort/utils"

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'yolov5_deepsort'), glob('caigou_pkg/yolov5_deepsort/*')),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eai',
    maintainer_email='hsssshan666@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_node = caigou_pkg.calibrate_node:main',  # 校准节点,手眼标定
            'awake_node = caigou_pkg.awake_node:main',  # 唤醒节点
            'tts_node = caigou_pkg.tts_node:main',  # 语音合成节点
            'stt_node = caigou_pkg.stt_node:main',  # 语音识别节点
            'bridge_node = caigou_pkg.bridge_node:main',  # 桥接节点
            'database_node = caigou_pkg.database_node:main',  # 数据库节点
            
            'realsense_node = caigou_pkg.realsense_node:main',  # 深度相机节点
            'ai_status_judge_node = caigou_pkg.ai_status_judge_node:main',  # AI状态判断节点
            'init_node = caigou_pkg.init_node:main',  # 执行初始化任务的节点
            'sort_node = caigou_pkg.sort_node:main',  # 执行分拣任务的节点
            'pickup_node = caigou_pkg.pickup_node:main',  # 执行取货任务的节点

            'debug_node = caigou_pkg.debug_node:main',  # 调试节点

            'track_node = caigou_pkg.hs_track_node:main',  # hs的跟踪节点

        ],
    },
)
