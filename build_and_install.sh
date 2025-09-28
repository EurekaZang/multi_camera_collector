#!/bin/bash

# Multi-Camera Data Collector 构建和安装脚本
# ROS 2 Foxy Fitzroy

set -e  # 遇到错误时退出

echo "=========================================="
echo "Multi-Camera Data Collector 构建脚本"
echo "ROS 2 Foxy Fitzroy"
echo "=========================================="

# 检查ROS 2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 未检测到ROS 2环境"
    echo "请先运行: source /opt/ros/foxy/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "foxy" ]; then
    echo "⚠️  警告: 检测到ROS版本为 $ROS_DISTRO，但此包是为Foxy设计的"
    read -p "是否继续？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "✅ ROS 2 $ROS_DISTRO 环境已激活"

# 检查当前目录
CURRENT_DIR=$(pwd)
PACKAGE_NAME="multi_camera_collector"

if [[ ! "$CURRENT_DIR" == *"$PACKAGE_NAME" ]]; then
    echo "❌ 错误: 请在multi_camera_collector包目录中运行此脚本"
    exit 1
fi

# 查找ROS 2工作空间
WORKSPACE_DIR=""
SEARCH_DIR="$CURRENT_DIR"

while [[ "$SEARCH_DIR" != "/" ]]; do
    if [[ -d "$SEARCH_DIR/src" && -f "$SEARCH_DIR/src/$PACKAGE_NAME/package.xml" ]]; then
        WORKSPACE_DIR="$SEARCH_DIR"
        break
    fi
    SEARCH_DIR=$(dirname "$SEARCH_DIR")
done

if [[ -z "$WORKSPACE_DIR" ]]; then
    echo "❌ 错误: 未找到ROS 2工作空间"
    echo "请确保此包位于ROS 2工作空间的src目录中"
    exit 1
fi

echo "✅ 找到工作空间: $WORKSPACE_DIR"

# 切换到工作空间根目录
cd "$WORKSPACE_DIR"

# 安装依赖
echo ""
echo "📦 安装依赖包..."
if command -v rosdep &> /dev/null; then
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    echo "✅ 依赖安装完成"
else
    echo "⚠️  警告: rosdep未安装，跳过依赖安装"
fi

# 构建包
echo ""
echo "🔨 构建包..."
colcon build --packages-select multi_camera_collector

if [ $? -eq 0 ]; then
    echo "✅ 构建成功"
else
    echo "❌ 构建失败"
    exit 1
fi

# 源化环境
echo ""
echo "🔄 源化环境..."
source install/setup.bash
echo "✅ 环境已源化"

# 运行测试
echo ""
echo "🧪 运行安装测试..."
if [ -f "src/multi_camera_collector/test/test_installation.py" ]; then
    python3 src/multi_camera_collector/test/test_installation.py
else
    echo "⚠️  测试文件未找到，跳过测试"
fi

echo ""
echo "=========================================="
echo "🎉 安装完成！"
echo "=========================================="
echo ""
echo "使用方法:"
echo "  1. 确保相机节点正在运行并发布正确的话题"
echo "  2. 启动数据采集:"
echo "     ros2 launch multi_camera_collector collector.launch.py"
echo ""
echo "  3. 自定义输出目录:"
echo "     ros2 launch multi_camera_collector collector.launch.py output_dir:=/path/to/dataset"
echo ""
echo "  4. 按Ctrl+C优雅关闭"
echo ""
echo "更多信息请查看 README.md"
echo "=========================================="