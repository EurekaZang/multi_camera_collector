#!/usr/bin/env python3
"""
多相机数据集CSV生成器

此模块负责扫描由data_collector节点生成的数据集目录，
并生成便于深度学习使用的CSV索引文件。
"""

import os
import glob
import pandas as pd
import json
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import argparse
import sys


class DatasetGenerator:
    """数据集CSV文件生成器"""
    
    def __init__(self, dataset_path: str):
        """
        初始化数据集生成器
        
        Args:
            dataset_path (str): 数据集根目录路径
        """
        self.dataset_path = Path(dataset_path).resolve()
        self.camera_types = ['camera', 'camera_femto']
        
        # 验证数据集路径
        if not self.dataset_path.exists():
            raise FileNotFoundError(f"数据集路径不存在: {self.dataset_path}")
        
        print(f"初始化数据集生成器，数据集路径: {self.dataset_path}")
    
    def _validate_dataset_structure(self) -> Dict[str, bool]:
        """
        验证数据集目录结构是否完整
        
        Returns:
            Dict[str, bool]: 每个相机类型的验证结果
        """
        validation_results = {}
        
        for camera_type in self.camera_types:
            camera_path = self.dataset_path / camera_type
            
            # 检查必要的子目录
            required_dirs = ['rgb', 'depth', 'camera_info']
            dirs_exist = all((camera_path / sub_dir).exists() for sub_dir in required_dirs)
            
            # 检查是否有数据文件
            has_data = any(
                len(list((camera_path / sub_dir).glob('*.png'))) > 0 
                for sub_dir in ['rgb', 'depth']
            )
            
            validation_results[camera_type] = dirs_exist and has_data
            
            status = "✓" if validation_results[camera_type] else "✗"
            print(f"{status} {camera_type}: 目录结构{'完整' if dirs_exist else '不完整'}，"
                  f"{'有数据' if has_data else '无数据'}")
        
        return validation_results
    
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
    
    def _get_image_pairs_for_camera(self, camera_type: str) -> List[Dict]:
        """
        获取指定相机的所有图像对信息
        
        Args:
            camera_type (str): 相机类型 ('camera' 或 'camera_femto')
            
        Returns:
            List[Dict]: 图像对信息列表
        """
        camera_path = self.dataset_path / camera_type
        rgb_path = camera_path / 'rgb'
        depth_path = camera_path / 'depth'
        camera_info_path = camera_path / 'camera_info'
        
        # 获取所有RGB图像文件
        rgb_files = list(rgb_path.glob('*.png'))
        rgb_timestamps = set()
        
        for rgb_file in rgb_files:
            timestamp = self._extract_timestamp_from_filename(rgb_file.name)
            if timestamp is not None:
                rgb_timestamps.add(timestamp)
        
        # 构建图像对列表
        image_pairs = []
        
        for timestamp in sorted(rgb_timestamps):
            # 构建文件路径
            rgb_file = rgb_path / f"{timestamp}.png"
            depth_file = depth_path / f"{timestamp}.png"
            rgb_info_file = camera_info_path / f"{timestamp}_rgb_camera_info.json"
            depth_info_file = camera_info_path / f"{timestamp}_depth_camera_info.json"
            
            # 检查所有必要文件是否存在
            if all(f.exists() for f in [rgb_file, depth_file, rgb_info_file, depth_info_file]):
                # 计算相对于数据集根目录的相对路径
                pair_info = {
                    'camera_type': camera_type,
                    'timestamp': timestamp,
                    'rgb_path': os.path.relpath(rgb_file, self.dataset_path),
                    'depth_path': os.path.relpath(depth_file, self.dataset_path),
                    'rgb_camera_info_path': os.path.relpath(rgb_info_file, self.dataset_path),
                    'depth_camera_info_path': os.path.relpath(depth_info_file, self.dataset_path),
                    'rgb_absolute_path': str(rgb_file),
                    'depth_absolute_path': str(depth_file),
                    'rgb_camera_info_absolute_path': str(rgb_info_file),
                    'depth_camera_info_absolute_path': str(depth_info_file),
                }
                
                # 尝试读取camera_info获取图像尺寸信息
                try:
                    with open(rgb_info_file, 'r') as f:
                        rgb_info = json.load(f)
                        pair_info['rgb_width'] = rgb_info['width']
                        pair_info['rgb_height'] = rgb_info['height']
                except:
                    pair_info['rgb_width'] = None
                    pair_info['rgb_height'] = None
                
                try:
                    with open(depth_info_file, 'r') as f:
                        depth_info = json.load(f)
                        pair_info['depth_width'] = depth_info['width']
                        pair_info['depth_height'] = depth_info['height']
                except:
                    pair_info['depth_width'] = None
                    pair_info['depth_height'] = None
                
                image_pairs.append(pair_info)
        
        return image_pairs
    
    def generate_csv_files(self, output_format: str = 'combined') -> List[str]:
        """
        生成CSV索引文件
        
        Args:
            output_format (str): 输出格式
                - 'combined': 生成单个包含所有相机数据的CSV文件
                - 'separate': 为每个相机生成单独的CSV文件
                - 'both': 同时生成组合和单独的CSV文件
                
        Returns:
            List[str]: 生成的CSV文件路径列表
        """
        # 验证数据集结构
        validation_results = self._validate_dataset_structure()
        
        valid_cameras = [camera for camera, valid in validation_results.items() if valid]
        
        if not valid_cameras:
            print("❌ 没有找到有效的相机数据，无法生成CSV文件")
            return []
        
        print(f"\n📊 开始生成CSV文件，有效相机: {valid_cameras}")
        
        generated_files = []
        all_pairs = []
        
        # 收集所有相机的数据
        for camera_type in valid_cameras:
            print(f"🔍 扫描 {camera_type} 数据...")
            pairs = self._get_image_pairs_for_camera(camera_type)
            all_pairs.extend(pairs)
            print(f"✓ {camera_type}: 找到 {len(pairs)} 组图像对")
            
            # 如果需要生成单独的CSV文件
            if output_format in ['separate', 'both']:
                df_single = pd.DataFrame(pairs)
                if not df_single.empty:
                    single_csv_path = self.dataset_path / f"{camera_type}_dataset.csv"
                    df_single.to_csv(single_csv_path, index=False)
                    generated_files.append(str(single_csv_path))
                    print(f"✓ 已生成 {camera_type} CSV文件: {single_csv_path}")
        
        # 如果需要生成组合的CSV文件
        if output_format in ['combined', 'both']:
            if all_pairs:
                df_combined = pd.DataFrame(all_pairs)
                # 按时间戳排序
                df_combined = df_combined.sort_values('timestamp').reset_index(drop=True)
                
                combined_csv_path = self.dataset_path / "dataset.csv"
                df_combined.to_csv(combined_csv_path, index=False)
                generated_files.append(str(combined_csv_path))
                print(f"✓ 已生成组合CSV文件: {combined_csv_path}")
            else:
                print("❌ 没有有效数据，无法生成组合CSV文件")
        
        # 生成统计信息
        self._generate_statistics(all_pairs, valid_cameras)
        
        return generated_files
    
    def _generate_statistics(self, all_pairs: List[Dict], valid_cameras: List[str]):
        """
        生成并保存数据集统计信息
        
        Args:
            all_pairs (List[Dict]): 所有图像对信息
            valid_cameras (List[str]): 有效的相机类型列表
        """
        if not all_pairs:
            return
        
        stats = {
            'dataset_path': str(self.dataset_path),
            'total_image_pairs': len(all_pairs),
            'cameras': {}
        }
        
        # 按相机类型统计
        for camera_type in valid_cameras:
            camera_pairs = [p for p in all_pairs if p['camera_type'] == camera_type]
            if camera_pairs:
                timestamps = [p['timestamp'] for p in camera_pairs]
                stats['cameras'][camera_type] = {
                    'count': len(camera_pairs),
                    'earliest_timestamp': min(timestamps),
                    'latest_timestamp': max(timestamps),
                    'duration_seconds': (max(timestamps) - min(timestamps)) / 1e9
                }
        
        # 保存统计信息
        stats_path = self.dataset_path / "dataset_statistics.json"
        with open(stats_path, 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=2, ensure_ascii=False)
        
        print(f"\n📈 数据集统计信息:")
        print(f"总图像对数: {stats['total_image_pairs']}")
        for camera_type, camera_stats in stats['cameras'].items():
            duration = camera_stats['duration_seconds']
            print(f"{camera_type}: {camera_stats['count']} 组，"
                  f"时长 {duration:.1f}s")
        print(f"✓ 统计信息已保存: {stats_path}")


def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='多相机数据集CSV生成器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 生成组合CSV文件（默认）
  python dataset_generator.py /path/to/dataset
  
  # 为每个相机生成单独的CSV文件
  python dataset_generator.py /path/to/dataset --format separate
  
  # 同时生成组合和单独的CSV文件
  python dataset_generator.py /path/to/dataset --format both

输出的CSV文件包含以下列:
  - camera_type: 相机类型 (camera/camera_femto)
  - timestamp: 纳秒级时间戳
  - rgb_path: RGB图像相对路径
  - depth_path: 深度图像相对路径
  - rgb_camera_info_path: RGB相机信息文件相对路径
  - depth_camera_info_path: 深度相机信息文件相对路径
  - rgb_absolute_path: RGB图像绝对路径
  - depth_absolute_path: 深度图像绝对路径
  - rgb_camera_info_absolute_path: RGB相机信息文件绝对路径
  - depth_camera_info_absolute_path: 深度相机信息文件绝对路径
  - rgb_width: RGB图像宽度
  - rgb_height: RGB图像高度
  - depth_width: 深度图像宽度
  - depth_height: 深度图像高度
        """
    )
    
    parser.add_argument(
        'dataset_path',
        help='数据集根目录路径'
    )
    
    parser.add_argument(
        '--format', '-f',
        choices=['combined', 'separate', 'both'],
        default='combined',
        help='CSV文件输出格式 (默认: combined)'
    )
    
    args = parser.parse_args()
    
    try:
        # 创建数据集生成器
        generator = DatasetGenerator(args.dataset_path)
        
        # 生成CSV文件
        generated_files = generator.generate_csv_files(args.format)
        
        if generated_files:
            print(f"\n🎉 成功生成 {len(generated_files)} 个CSV文件:")
            for file_path in generated_files:
                print(f"  📄 {file_path}")
        else:
            print("\n❌ 未生成任何CSV文件")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n❌ 错误: {str(e)}")
        sys.exit(1)


if __name__ == '__main__':
    main()