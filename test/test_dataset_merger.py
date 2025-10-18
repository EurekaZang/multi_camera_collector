#!/usr/bin/env python3
"""
数据集合并工具测试脚本

用于验证dataset_merger模块的功能
"""

import os
import sys
import tempfile
import shutil
from pathlib import Path

# 添加模块路径
current_dir = Path(__file__).parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

try:
    from multi_camera_collector.dataset_merger import DatasetMerger
except ImportError:
    try:
        from dataset_merger import DatasetMerger
    except ImportError:
        print("❌ 无法导入dataset_merger模块")
        sys.exit(1)


def create_mock_dataset(base_path: Path, dataset_name: str, num_frames: int = 5):
    """
    创建一个模拟数据集用于测试
    
    Args:
        base_path (Path): 基础路径
        dataset_name (str): 数据集名称
        num_frames (int): 帧数
    """
    dataset_path = base_path / dataset_name
    
    for camera_type in ['camera', 'camera_femto']:
        camera_path = dataset_path / camera_type
        (camera_path / 'rgb').mkdir(parents=True, exist_ok=True)
        (camera_path / 'depth').mkdir(parents=True, exist_ok=True)
        (camera_path / 'camera_info').mkdir(parents=True, exist_ok=True)
        
        # 创建模拟文件
        for i in range(num_frames):
            timestamp = 1000000000000000000 + i * 1000000000  # 1秒间隔
            
            # 创建空的图像文件
            (camera_path / 'rgb' / f"{timestamp}.png").touch()
            (camera_path / 'depth' / f"{timestamp}.png").touch()
            
            # 创建camera_info文件
            camera_info = {
                'width': 640,
                'height': 480,
                'camera_name': camera_type,
                'distortion_model': 'plumb_bob'
            }
            
            import json
            with open(camera_path / 'camera_info' / f"{timestamp}_rgb_camera_info.json", 'w') as f:
                json.dump(camera_info, f)
            
            with open(camera_path / 'camera_info' / f"{timestamp}_depth_camera_info.json", 'w') as f:
                json.dump(camera_info, f)
    
    print(f"✓ 创建模拟数据集: {dataset_path}")
    return dataset_path


def test_basic_merge():
    """测试基本的数据集合并功能"""
    print("\n" + "="*50)
    print("测试1: 基本数据集合并")
    print("="*50)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        base_path = Path(tmpdir)
        
        # 创建两个模拟数据集
        dataset1 = create_mock_dataset(base_path, "dataset1", num_frames=3)
        dataset2 = create_mock_dataset(base_path, "dataset2", num_frames=3)
        
        # 创建合并器
        output_path = base_path / "merged"
        merger = DatasetMerger(str(output_path))
        
        # 添加数据集
        assert merger.add_dataset(str(dataset1)), "添加dataset1失败"
        assert merger.add_dataset(str(dataset2)), "添加dataset2失败"
        
        # 执行合并
        assert merger.merge(resolve_conflicts='offset'), "合并失败"
        
        # 验证输出
        assert output_path.exists(), "输出目录不存在"
        assert (output_path / 'camera' / 'rgb').exists(), "camera/rgb目录不存在"
        assert (output_path / 'camera_femto' / 'rgb').exists(), "camera_femto/rgb目录不存在"
        
        print("✅ 测试1通过: 基本合并功能正常")
        return True


def test_conflict_resolution():
    """测试冲突解决策略"""
    print("\n" + "="*50)
    print("测试2: 时间戳冲突解决")
    print("="*50)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        base_path = Path(tmpdir)
        
        # 创建两个具有相同时间戳的数据集
        dataset1 = create_mock_dataset(base_path, "dataset1", num_frames=3)
        dataset2 = create_mock_dataset(base_path, "dataset2", num_frames=3)
        
        # 测试offset策略
        output_path = base_path / "merged_offset"
        merger = DatasetMerger(str(output_path))
        merger.add_dataset(str(dataset1))
        merger.add_dataset(str(dataset2))
        
        assert merger.merge(resolve_conflicts='offset'), "offset策略合并失败"
        
        # 统计文件数
        rgb_files = list((output_path / 'camera' / 'rgb').glob('*.png'))
        print(f"  offset策略: 找到 {len(rgb_files)} 个RGB文件")
        assert len(rgb_files) == 6, f"期望6个文件，实际{len(rgb_files)}个"
        
        print("✅ 测试2通过: 冲突解决策略正常")
        return True


def test_invalid_dataset():
    """测试无效数据集处理"""
    print("\n" + "="*50)
    print("测试3: 无效数据集处理")
    print("="*50)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        base_path = Path(tmpdir)
        
        # 创建一个空目录
        invalid_path = base_path / "invalid"
        invalid_path.mkdir()
        
        output_path = base_path / "merged"
        merger = DatasetMerger(str(output_path))
        
        # 尝试添加无效数据集
        result = merger.add_dataset(str(invalid_path))
        assert not result, "应该拒绝无效数据集"
        
        # 尝试添加不存在的路径
        result = merger.add_dataset("/nonexistent/path")
        assert not result, "应该拒绝不存在的路径"
        
        print("✅ 测试3通过: 无效数据集处理正常")
        return True


def test_dataset_info():
    """测试数据集信息提取"""
    print("\n" + "="*50)
    print("测试4: 数据集信息提取")
    print("="*50)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        base_path = Path(tmpdir)
        
        # 创建数据集
        dataset1 = create_mock_dataset(base_path, "dataset1", num_frames=5)
        
        output_path = base_path / "merged"
        merger = DatasetMerger(str(output_path))
        
        # 获取数据集信息
        info = merger._get_dataset_info(dataset1)
        
        assert 'path' in info, "数据集信息应包含path"
        assert 'cameras' in info, "数据集信息应包含cameras"
        assert 'camera' in info['cameras'], "应该识别到camera"
        assert 'camera_femto' in info['cameras'], "应该识别到camera_femto"
        
        camera_info = info['cameras']['camera']
        assert camera_info['count'] == 5, f"期望5帧，实际{camera_info['count']}帧"
        
        print(f"  识别的帧数: {camera_info['count']}")
        print("✅ 测试4通过: 数据集信息提取正常")
        return True


def run_all_tests():
    """运行所有测试"""
    print("🧪 数据集合并工具测试套件")
    print("="*50)
    
    tests = [
        ("基本合并功能", test_basic_merge),
        ("冲突解决策略", test_conflict_resolution),
        ("无效数据集处理", test_invalid_dataset),
        ("数据集信息提取", test_dataset_info),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
            else:
                failed += 1
                print(f"❌ {test_name} 失败")
        except Exception as e:
            failed += 1
            print(f"❌ {test_name} 异常: {str(e)}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*50)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("="*50)
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
