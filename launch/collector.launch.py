#!/usr/bin/env python3
"""
Multi-Camera Data Collector Launch File

这个launch文件启动多相机数据采集节点，并支持从命令行传递参数。

使用方法:
    ros2 launch multi_camera_collector collector.launch.py
    ros2 launch multi_camera_collector collector.launch.py output_dir:=/path/to/your/dataset

作者: ROS 2 机器人软件工程师
版本: ROS 2 Foxy Fitzroy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述，配置节点参数和启动选项"""
    
    # 声明launch参数
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='./dataset',
        description='数据保存的根目录路径。例如: /home/user/my_dataset'
    )
    
    # 声明FPS参数
    max_fps_arg = DeclareLaunchArgument(
        'max_fps',
        default_value='5.0',
        description='最大采集帧率(FPS)。设置为0或负数将禁用FPS限制。默认5.0FPS避免产生过多图片'
    )
    
    # 声明日志级别参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='节点日志级别 (debug, info, warn, error, fatal)'
    )
    
    # 配置多相机数据采集节点
    collector_node = Node(
        package='multi_camera_collector',
        executable='multi_camera_collector',
        name='multi_camera_collector',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'max_fps': LaunchConfiguration('max_fps'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # 在节点终止时重新映射
        remappings=[
            # 如果需要，可以在这里重新映射话题名称
            # ('/camera/color/image_raw', '/your_custom_topic'),
        ]
    )
    
    return LaunchDescription([
        # 添加所有launch参数
        output_dir_arg,
        max_fps_arg,
        log_level_arg,
        
        # 添加节点
        collector_node,
    ])