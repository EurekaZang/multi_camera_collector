#!/usr/bin/env python3
"""
多相机数据集合并工具

此模块负责合并多个由multi_camera_collector生成的数据集，
并生成统一的CSV索引文件。
"""

import os
import shutil
import json
import pandas as pd
from pathlib import Path
from typing import List, Dict, Optional
import argparse
import sys
from datetime import datetime


class DatasetMerger:
    """数据集合并器"""
    
    def __init__(self, output_path: str):
        """
        初始化数据集合并器
        
        Args:
            output_path (str): 合并后数据集的输出路径
        """
        self.output_path = Path(output_path).resolve()
        self.camera_types = ['camera', 'camera_femto']
        self.source_datasets: List[Path] = []
        
        print(f"初始化数据集合并器，输出路径: {self.output_path}")
    
    def add_dataset(self, dataset_path: str) -> bool:
        """
        添加一个数据集到合并列表
        
        Args:
            dataset_path (str): 数据集路径
            
        Returns:
            bool: 是否成功添加
        """
        path = Path(dataset_path).resolve()
        
        # 验证数据集路径
        if not path.exists():
            print(f"⚠️  警告: 数据集路径不存在: {path}")
            return False
        
        if not path.is_dir():
            print(f"⚠️  警告: 路径不是目录: {path}")
            return False
        
        # 验证数据集结构
        valid = False
        for camera_type in self.camera_types:
            camera_path = path / camera_type
            if camera_path.exists():
                required_dirs = ['rgb', 'depth', 'camera_info']
                if all((camera_path / d).exists() for d in required_dirs):
                    valid = True
                    break
        
        if not valid:
            print(f"⚠️  警告: 数据集结构不正确: {path}")
            return False
        
        self.source_datasets.append(path)
        print(f"✓ 已添加数据集: {path}")
        return True
    
    def _create_output_structure(self):
        """创建输出目录结构"""
        print("\n📁 创建输出目录结构...")
        
        for camera_type in self.camera_types:
            camera_path = self.output_path / camera_type
            (camera_path / 'rgb').mkdir(parents=True, exist_ok=True)
            (camera_path / 'depth').mkdir(parents=True, exist_ok=True)
            (camera_path / 'camera_info').mkdir(parents=True, exist_ok=True)
        
        print("✓ 输出目录结构已创建")
    
    def _extract_timestamp_from_filename(self, filename: str) -> Optional[int]:
        """
        从文件名中提取时间戳
        
        Args:
            filename (str): 文件名（不含路径）
            
        Returns:
            Optional[int]: 时间戳（纳秒），如果提取失败返回None
        """
        try:
            # 移除文件扩展名和可能的后缀
            name_without_ext = filename.split('.')[0]
            
            # 处理camera_info文件的特殊命名格式
            if '_rgb_camera_info' in name_without_ext:
                timestamp_str = name_without_ext.replace('_rgb_camera_info', '')
            elif '_depth_camera_info' in name_without_ext:
                timestamp_str = name_without_ext.replace('_depth_camera_info', '')
            else:
                timestamp_str = name_without_ext
            
            return int(timestamp_str)
        except ValueError:
            return None
    
    def _get_dataset_info(self, dataset_path: Path) -> Dict:
        """
        获取数据集的基本信息
        
        Args:
            dataset_path (Path): 数据集路径
            
        Returns:
            Dict: 数据集信息
        """
        info = {
            'path': str(dataset_path),
            'cameras': {}
        }
        
        for camera_type in self.camera_types:
            camera_path = dataset_path / camera_type
            if not camera_path.exists():
                continue
            
            rgb_path = camera_path / 'rgb'
            if rgb_path.exists():
                rgb_files = list(rgb_path.glob('*.png'))
                timestamps = []
                for rgb_file in rgb_files:
                    ts = self._extract_timestamp_from_filename(rgb_file.name)
                    if ts is not None:
                        timestamps.append(ts)
                
                if timestamps:
                    info['cameras'][camera_type] = {
                        'count': len(timestamps),
                        'min_timestamp': min(timestamps),
                        'max_timestamp': max(timestamps)
                    }
        
        return info
    
    def _check_timestamp_conflicts(self) -> Dict[str, List[tuple]]:
        """
        检查数据集之间的时间戳冲突
        
        Returns:
            Dict[str, List[tuple]]: 每个相机类型的冲突列表 (timestamp, dataset_index1, dataset_index2)
        """
        conflicts = {camera_type: [] for camera_type in self.camera_types}
        
        for camera_type in self.camera_types:
            timestamp_to_dataset = {}
            
            for idx, dataset_path in enumerate(self.source_datasets):
                camera_path = dataset_path / camera_type / 'rgb'
                if not camera_path.exists():
                    continue
                
                for rgb_file in camera_path.glob('*.png'):
                    ts = self._extract_timestamp_from_filename(rgb_file.name)
                    if ts is not None:
                        if ts in timestamp_to_dataset:
                            conflicts[camera_type].append((ts, timestamp_to_dataset[ts], idx))
                        else:
                            timestamp_to_dataset[ts] = idx
        
        return conflicts
    
    def _copy_dataset_files(self, dataset_path: Path, dataset_index: int, 
                           timestamp_offset: int = 0) -> Dict[str, int]:
        """
        复制一个数据集的所有文件到输出目录
        
        Args:
            dataset_path (Path): 源数据集路径
            dataset_index (int): 数据集索引
            timestamp_offset (int): 时间戳偏移量（用于解决冲突）
            
        Returns:
            Dict[str, int]: 每个相机类型复制的文件数量
        """
        copied_counts = {}
        
        for camera_type in self.camera_types:
            src_camera_path = dataset_path / camera_type
            if not src_camera_path.exists():
                continue
            
            dst_camera_path = self.output_path / camera_type
            count = 0
            
            # 获取所有时间戳
            rgb_path = src_camera_path / 'rgb'
            if not rgb_path.exists():
                continue
            
            rgb_files = list(rgb_path.glob('*.png'))
            
            for rgb_file in rgb_files:
                timestamp = self._extract_timestamp_from_filename(rgb_file.name)
                if timestamp is None:
                    continue
                
                # 应用时间戳偏移
                new_timestamp = timestamp + timestamp_offset
                
                # 定义文件路径
                files_to_copy = [
                    (src_camera_path / 'rgb' / f"{timestamp}.png",
                     dst_camera_path / 'rgb' / f"{new_timestamp}.png"),
                    (src_camera_path / 'depth' / f"{timestamp}.png",
                     dst_camera_path / 'depth' / f"{new_timestamp}.png"),
                    (src_camera_path / 'camera_info' / f"{timestamp}_rgb_camera_info.json",
                     dst_camera_path / 'camera_info' / f"{new_timestamp}_rgb_camera_info.json"),
                    (src_camera_path / 'camera_info' / f"{timestamp}_depth_camera_info.json",
                     dst_camera_path / 'camera_info' / f"{new_timestamp}_depth_camera_info.json"),
                ]
                
                # 检查所有源文件是否存在
                if all(src.exists() for src, _ in files_to_copy):
                    # 复制所有文件
                    for src, dst in files_to_copy:
                        shutil.copy2(src, dst)
                    count += 1
            
            copied_counts[camera_type] = count
        
        return copied_counts
    
    def merge(self, resolve_conflicts: str = 'offset') -> bool:
        """
        执行数据集合并操作
        
        Args:
            resolve_conflicts (str): 冲突解决策略
                - 'offset': 为冲突的时间戳添加偏移量
                - 'skip': 跳过冲突的文件
                - 'overwrite': 覆盖已存在的文件
                
        Returns:
            bool: 是否成功合并
        """
        if not self.source_datasets:
            print("❌ 错误: 没有添加任何数据集")
            return False
        
        print(f"\n🔄 开始合并 {len(self.source_datasets)} 个数据集...")
        print(f"冲突解决策略: {resolve_conflicts}")
        
        # 创建输出目录结构
        self._create_output_structure()
        
        # 显示数据集信息
        print("\n📊 数据集信息:")
        for idx, dataset_path in enumerate(self.source_datasets):
            info = self._get_dataset_info(dataset_path)
            print(f"\n数据集 {idx + 1}: {dataset_path.name}")
            for camera_type, camera_info in info['cameras'].items():
                duration = (camera_info['max_timestamp'] - camera_info['min_timestamp']) / 1e9
                print(f"  {camera_type}: {camera_info['count']} 组，时长 {duration:.1f}s")
        
        # 检查时间戳冲突
        if resolve_conflicts == 'offset':
            print("\n🔍 检查时间戳冲突...")
            conflicts = self._check_timestamp_conflicts()
            
            total_conflicts = sum(len(c) for c in conflicts.values())
            if total_conflicts > 0:
                print(f"⚠️  发现 {total_conflicts} 个时间戳冲突")
                for camera_type, conflict_list in conflicts.items():
                    if conflict_list:
                        print(f"  {camera_type}: {len(conflict_list)} 个冲突")
                print("将使用时间戳偏移策略解决冲突...")
            else:
                print("✓ 未发现时间戳冲突")
        
        # 复制文件
        print("\n📦 复制文件...")
        total_copied = {camera_type: 0 for camera_type in self.camera_types}
        
        timestamp_offset = 0
        for idx, dataset_path in enumerate(self.source_datasets):
            print(f"\n处理数据集 {idx + 1}/{len(self.source_datasets)}: {dataset_path.name}")
            
            # 计算时间戳偏移
            if resolve_conflicts == 'offset' and idx > 0:
                # 使用前一个数据集的最大时间戳作为偏移基准
                info = self._get_dataset_info(self.source_datasets[idx - 1])
                max_timestamp = 0
                for camera_info in info['cameras'].values():
                    max_timestamp = max(max_timestamp, camera_info['max_timestamp'])
                
                # 添加1秒的缓冲区
                timestamp_offset = max_timestamp + int(1e9)
            
            copied_counts = self._copy_dataset_files(dataset_path, idx, timestamp_offset)
            
            for camera_type, count in copied_counts.items():
                total_copied[camera_type] += count
                if count > 0:
                    print(f"  ✓ {camera_type}: {count} 组")
        
        print(f"\n✓ 文件复制完成")
        print("总计:")
        for camera_type, count in total_copied.items():
            if count > 0:
                print(f"  {camera_type}: {count} 组")
        
        return True
    
    def generate_csv(self, output_format: str = 'combined') -> List[str]:
        """
        为合并后的数据集生成CSV文件
        
        Args:
            output_format (str): 输出格式
                - 'combined': 生成单个包含所有相机数据的CSV文件
                - 'separate': 为每个相机生成单独的CSV文件
                - 'both': 同时生成组合和单独的CSV文件
                
        Returns:
            List[str]: 生成的CSV文件路径列表
        """
        print(f"\n📊 生成CSV文件...")
        
        # 使用DatasetGenerator生成CSV
        try:
            # 动态导入以避免循环依赖
            from multi_camera_collector.dataset_generator import DatasetGenerator
        except ImportError:
            try:
                current_dir = Path(__file__).parent
                if str(current_dir) not in sys.path:
                    sys.path.insert(0, str(current_dir))
                from dataset_generator import DatasetGenerator
            except ImportError:
                print("❌ 错误: 无法导入dataset_generator模块")
                return []
        
        try:
            generator = DatasetGenerator(str(self.output_path))
            generated_files = generator.generate_csv_files(output_format)
            
            if generated_files:
                print(f"✓ 成功生成 {len(generated_files)} 个CSV文件")
                for file_path in generated_files:
                    file_size = Path(file_path).stat().st_size
                    print(f"  📄 {Path(file_path).name} ({file_size:,} bytes)")
            
            return generated_files
            
        except Exception as e:
            print(f"❌ 生成CSV文件时出错: {str(e)}")
            return []
    
    def save_merge_info(self):
        """保存合并信息到JSON文件"""
        merge_info = {
            'merge_date': datetime.now().isoformat(),
            'output_path': str(self.output_path),
            'source_datasets': [str(p) for p in self.source_datasets],
            'num_datasets': len(self.source_datasets)
        }
        
        info_path = self.output_path / "merge_info.json"
        with open(info_path, 'w', encoding='utf-8') as f:
            json.dump(merge_info, f, indent=2, ensure_ascii=False)
        
        print(f"✓ 合并信息已保存: {info_path}")


def main():
    """命令行入口函数"""
    print("🔀 多相机数据集合并工具")
    print("=" * 50)
    
    parser = argparse.ArgumentParser(
        description='合并多个multi_camera_collector数据集',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 合并两个数据集
  ./dataset_merger.py --output merged_dataset dataset1/ dataset2/
  
  # 合并多个数据集并生成单独的CSV文件
  ./dataset_merger.py --output merged_dataset --format separate dataset1/ dataset2/ dataset3/
  
  # 使用不同的冲突解决策略
  ./dataset_merger.py --output merged_dataset --conflicts overwrite dataset1/ dataset2/

数据集结构要求:
  每个输入数据集应包含:
  ├── camera/
  │   ├── rgb/           # 标准相机RGB图像
  │   ├── depth/         # 标准相机深度图像
  │   └── camera_info/   # 标准相机参数文件
  └── camera_femto/
      ├── rgb/           # Femto相机RGB图像
      ├── depth/         # Femto相机深度图像
      └── camera_info/   # Femto相机参数文件

输出:
  合并后的数据集将包含所有输入数据集的数据，
  并生成统一的CSV索引文件，格式与make_dataset.py生成的相同。
        """
    )
    
    parser.add_argument(
        'datasets',
        nargs='+',
        help='要合并的数据集路径（至少2个）'
    )
    
    parser.add_argument(
        '--output', '-o',
        required=True,
        help='合并后数据集的输出路径'
    )
    
    parser.add_argument(
        '--format', '-f',
        choices=['combined', 'separate', 'both'],
        default='combined',
        help='CSV文件输出格式 (默认: combined)'
    )
    
    parser.add_argument(
        '--conflicts', '-c',
        choices=['offset', 'skip', 'overwrite'],
        default='offset',
        help='时间戳冲突解决策略 (默认: offset)\n'
             '  offset: 为冲突的时间戳添加偏移量\n'
             '  skip: 跳过冲突的文件\n'
             '  overwrite: 覆盖已存在的文件'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='显示详细输出信息'
    )
    
    args = parser.parse_args()
    
    # 验证输入
    if len(args.datasets) < 2:
        print("❌ 错误: 至少需要2个数据集进行合并")
        sys.exit(1)
    
    # 检查输出路径
    output_path = Path(args.output).resolve()
    if output_path.exists():
        print(f"⚠️  警告: 输出路径已存在: {output_path}")
        response = input("是否要覆盖? (y/N): ")
        if response.lower() != 'y':
            print("操作已取消")
            sys.exit(0)
        print("将覆盖现有内容...")
    
    print(f"\n📂 输出路径: {output_path}")
    print(f"📊 CSV格式: {args.format}")
    print(f"⚙️  冲突策略: {args.conflicts}")
    print()
    
    try:
        # 创建合并器
        merger = DatasetMerger(str(output_path))
        
        # 添加数据集
        print("📋 添加数据集:")
        for dataset_path in args.datasets:
            if not merger.add_dataset(dataset_path):
                print(f"⚠️  跳过无效数据集: {dataset_path}")
        
        if len(merger.source_datasets) < 2:
            print("\n❌ 错误: 至少需要2个有效数据集")
            sys.exit(1)
        
        # 执行合并
        if not merger.merge(resolve_conflicts=args.conflicts):
            print("\n❌ 数据集合并失败")
            sys.exit(1)
        
        # 生成CSV文件
        generated_files = merger.generate_csv(args.format)
        
        if not generated_files:
            print("\n⚠️  警告: 未能生成CSV文件")
        
        # 保存合并信息
        merger.save_merge_info()
        
        print("\n🎉 数据集合并完成！")
        print(f"✅ 输出位置: {output_path}")
        
        if generated_files:
            print(f"\n💡 使用提示:")
            print(f"  • 合并后的数据集可直接用于深度学习训练")
            print(f"  • CSV文件格式与make_dataset.py生成的相同")
            print(f"  • 查看 merge_info.json 了解合并详情")
            print(f"  • 查看 dataset_statistics.json 了解数据集统计信息")
        
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
