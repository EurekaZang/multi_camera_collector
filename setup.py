from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'multi_camera_collector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 安装包标识文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 安装package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pandas',  # 用于CSV文件处理
        'numpy',   # 数据处理依赖
    ],
    zip_safe=True,
    maintainer='Eureka Zang',
    maintainer_email='scytz4@nottingham.edu.cn',
    description='ROS 2 Foxy 多相机RGB-D数据采集包，支持时间同步的图像保存功能',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 定义可执行节点的入口点
            'multi_camera_collector = multi_camera_collector.data_collector:main',
            # 数据集制作工具的入口点
            'make_dataset = multi_camera_collector.make_dataset:main',
            'dataset_generator = multi_camera_collector.dataset_generator:main',
            # 数据集合并工具的入口点
            'merge_datasets = multi_camera_collector.dataset_merger:main',
        ],
    },
)