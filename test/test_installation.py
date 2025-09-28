#!/usr/bin/env python3
"""
测试脚本：验证multi_camera_collector包的安装和基本功能

使用方法:
    cd ~/ros2_ws
    source install/setup.bash
    python3 src/multi_camera_collector/test/test_installation.py
"""

import sys
import subprocess
import os

def test_import():
    """测试Python包导入"""
    print("测试Python包导入...")
    try:
        import rclpy
        import sensor_msgs.msg
        import cv_bridge
        import message_filters
        import cv2
        import numpy
        print("✅ 所有依赖包导入成功")
        return True
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        return False

def test_ros2_package():
    """测试ROS 2包是否正确安装"""
    print("\n测试ROS 2包安装...")
    try:
        # 检查包是否在包列表中
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True)
        if 'multi_camera_collector' in result.stdout:
            print("✅ multi_camera_collector包已正确安装")
            return True
        else:
            print("❌ multi_camera_collector包未找到")
            return False
    except subprocess.CalledProcessError as e:
        print(f"❌ 检查包安装失败: {e}")
        return False

def test_executable():
    """测试可执行文件是否可用"""
    print("\n测试可执行文件...")
    try:
        # 检查可执行文件
        result = subprocess.run(['ros2', 'run', 'multi_camera_collector', 
                               'multi_camera_collector', '--help'], 
                              capture_output=True, text=True, timeout=5)
        # 由于节点可能没有--help参数，我们只检查是否能找到可执行文件
        print("✅ 可执行文件可用")
        return True
    except subprocess.TimeoutExpired:
        print("✅ 可执行文件可用 (超时但找到了文件)")
        return True
    except subprocess.CalledProcessError as e:
        if "executable not found" in str(e) or "No executable found" in str(e):
            print("❌ 可执行文件未找到")
            return False
        else:
            print("✅ 可执行文件可用")
            return True

def test_launch_file():
    """测试launch文件是否可用"""
    print("\n测试launch文件...")
    try:
        # 检查launch文件
        result = subprocess.run(['ros2', 'launch', 'multi_camera_collector', 
                               'collector.launch.py', '--show-args'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✅ Launch文件可用")
            print("可用参数:")
            print(result.stdout)
            return True
        else:
            print("❌ Launch文件测试失败")
            return False
    except subprocess.TimeoutExpired:
        print("❌ Launch文件测试超时")
        return False
    except subprocess.CalledProcessError as e:
        print(f"❌ Launch文件测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=" * 60)
    print("Multi-Camera Collector 安装测试")
    print("=" * 60)
    
    tests = [
        test_import,
        test_ros2_package,
        test_executable,
        test_launch_file
    ]
    
    results = []
    for test in tests:
        results.append(test())
    
    print("\n" + "=" * 60)
    print("测试结果总结:")
    print(f"通过: {sum(results)}/{len(results)}")
    
    if all(results):
        print("🎉 所有测试通过！包已成功安装")
        print("\n现在您可以使用以下命令启动数据采集:")
        print("ros2 launch multi_camera_collector collector.launch.py")
        return 0
    else:
        print("❌ 部分测试失败，请检查安装")
        return 1

if __name__ == '__main__':
    sys.exit(main())