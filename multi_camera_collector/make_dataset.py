#!/usr/bin/env python3
"""
多相机数据集一键制作工具

这是一个命令行工具，用于为multi_camera_collector节点采集的数据集
生成便于深度学习使用的CSV索引文件。
"""

import os
import sys
import argparse
from pathlib import Path

# 尝试多种导入方式以支持不同的执行环境
try:
    # 方式1: 作为ROS2包导入（推荐）
    from multi_camera_collector.dataset_generator import DatasetGenerator
except ImportError:
    try:
        # 方式2: 从当前目录导入（开发环境）
        current_dir = Path(__file__).parent
        if str(current_dir) not in sys.path:
            sys.path.insert(0, str(current_dir))
        from dataset_generator import DatasetGenerator
    except ImportError:
        print("❌ 错误: 无法导入dataset_generator模块")
        print("请确保：")
        print("  1. ROS2包已正确安装 (colcon build)")
        print("  2. 或者此脚本与dataset_generator.py在同一目录中")
        sys.exit(1)


def main():
    """命令行入口函数"""
    print("🤖 多相机数据集一键制作工具")
    print("=" * 50)
    
    parser = argparse.ArgumentParser(
        description='为multi_camera_collector数据集生成CSV索引文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 生成组合CSV文件（推荐用于深度学习）
  ./make_dataset.py /path/to/dataset
  
  # 为每个相机生成单独的CSV文件
  ./make_dataset.py /path/to/dataset --format separate
  
  # 同时生成组合和单独的CSV文件
  ./make_dataset.py /path/to/dataset --format both

数据集目录结构应为:
  dataset/
  ├── camera/
  │   ├── rgb/           # 标准相机RGB图像
  │   ├── depth/         # 标准相机深度图像
  │   └── camera_info/   # 标准相机参数文件
  └── camera_femto/
      ├── rgb/           # Femto相机RGB图像
      ├── depth/         # Femto相机深度图像
      └── camera_info/   # Femto相机参数文件

生成的CSV文件可直接用于:
  - PyTorch Dataset类
  - TensorFlow data pipeline
  - 深度学习训练脚本
  - 数据分析和可视化
        """
    )
    
    parser.add_argument(
        'dataset_path',
        help='数据集根目录路径（包含camera和camera_femto子目录）'
    )
    
    parser.add_argument(
        '--format', '-f',
        choices=['combined', 'separate', 'both'],
        default='combined',
        help='CSV文件输出格式:\n'
             '  combined: 单个CSV包含所有相机数据（默认，推荐）\n'
             '  separate: 每个相机单独的CSV文件\n'
             '  both: 同时生成组合和单独的CSV文件'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='显示详细输出信息'
    )
    
    args = parser.parse_args()
    
    # 验证数据集路径
    dataset_path = Path(args.dataset_path).resolve()
    if not dataset_path.exists():
        print(f"❌ 错误: 数据集路径不存在: {dataset_path}")
        sys.exit(1)
    
    if not dataset_path.is_dir():
        print(f"❌ 错误: 指定路径不是目录: {dataset_path}")
        sys.exit(1)
    
    print(f"📂 数据集路径: {dataset_path}")
    print(f"📊 输出格式: {args.format}")
    print()
    
    try:
        # 创建数据集生成器
        if args.verbose:
            print("🔧 初始化数据集生成器...")
        
        generator = DatasetGenerator(str(dataset_path))
        
        # 生成CSV文件
        if args.verbose:
            print("🚀 开始生成CSV文件...")
        
        generated_files = generator.generate_csv_files(args.format)
        
        if generated_files:
            print("\n🎉 数据集制作完成！")
            print(f"✅ 成功生成 {len(generated_files)} 个文件:")
            for file_path in generated_files:
                file_size = Path(file_path).stat().st_size
                print(f"  📄 {Path(file_path).name} ({file_size:,} bytes)")
            
            print(f"\n💡 使用提示:")
            print(f"  • CSV文件路径均为相对于数据集根目录的相对路径")
            print(f"  • 可直接在深度学习代码中使用这些CSV文件作为数据索引")
            print(f"  • 推荐使用 dataset.csv（组合文件）进行训练")
            print(f"  • 统计信息已保存到 dataset_statistics.json")
            
        else:
            print("\n❌ 数据集制作失败")
            print("可能的原因:")
            print("  • 数据集目录结构不正确")
            print("  • 没有找到有效的图像数据")
            print("  • 缺少必要的camera_info文件")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断操作")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 错误: {str(e)}")
        if args.verbose:
            import traceback
            print("\n详细错误信息:")
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()