#!/usr/bin/env python3
"""
数据集合并工具的Launch文件

使用ROS2 launch启动数据集合并工具
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    生成Launch描述
    
    Launch参数:
        output_dir: 合并后数据集的输出目录
        dataset1: 第一个数据集路径
        dataset2: 第二个数据集路径
        dataset3: (可选) 第三个数据集路径
        dataset4: (可选) 第四个数据集路径
        format: CSV文件输出格式 (combined/separate/both)
        conflicts: 冲突解决策略 (offset/skip/overwrite)
        verbose: 是否显示详细输出 (true/false)
    """
    
    # 声明launch参数
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        description='合并后数据集的输出目录'
    )
    
    dataset1_arg = DeclareLaunchArgument(
        'dataset1',
        description='第一个数据集路径'
    )
    
    dataset2_arg = DeclareLaunchArgument(
        'dataset2',
        description='第二个数据集路径'
    )
    
    dataset3_arg = DeclareLaunchArgument(
        'dataset3',
        default_value='',
        description='第三个数据集路径（可选）'
    )
    
    dataset4_arg = DeclareLaunchArgument(
        'dataset4',
        default_value='',
        description='第四个数据集路径（可选）'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='combined',
        description='CSV文件输出格式 (combined/separate/both)'
    )
    
    conflicts_arg = DeclareLaunchArgument(
        'conflicts',
        default_value='offset',
        description='冲突解决策略 (offset/skip/overwrite)'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='是否显示详细输出'
    )
    
    # 获取launch配置
    output_dir = LaunchConfiguration('output_dir')
    dataset1 = LaunchConfiguration('dataset1')
    dataset2 = LaunchConfiguration('dataset2')
    dataset3 = LaunchConfiguration('dataset3')
    dataset4 = LaunchConfiguration('dataset4')
    format_type = LaunchConfiguration('format')
    conflicts = LaunchConfiguration('conflicts')
    verbose = LaunchConfiguration('verbose')
    
    # 创建合并命令
    # 注意：这里使用了一个技巧来处理可选的数据集参数
    merge_cmd = ExecuteProcess(
        cmd=[
            'merge_datasets',
            '--output', output_dir,
            '--format', format_type,
            '--conflicts', conflicts,
            dataset1,
            dataset2,
            dataset3,
            dataset4
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        output_dir_arg,
        dataset1_arg,
        dataset2_arg,
        dataset3_arg,
        dataset4_arg,
        format_arg,
        conflicts_arg,
        verbose_arg,
        merge_cmd,
    ])
