#!/bin/bash

# 快速启动示例脚本
# 展示如何使用 multi_camera_collector 包

echo "=========================================="
echo "Multi-Camera Data Collector 快速启动"
echo "=========================================="

# 检查ROS 2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先激活ROS 2环境:"
    echo "   source /opt/ros/foxy/setup.bash"
    echo "   source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "当前ROS 2版本: $ROS_DISTRO"
echo ""

# 显示可用的启动选项
echo "📋 可用的启动选项："
echo ""

echo "1️⃣  基本启动 (默认5FPS限制):"
echo "   ros2 launch multi_camera_collector collector.launch.py"
echo ""

echo "2️⃣  自定义输出目录:"
echo "   ros2 launch multi_camera_collector collector.launch.py output_dir:=/home/user/my_dataset"
echo ""

echo "3️⃣  自定义FPS限制:"
echo "   ros2 launch multi_camera_collector collector.launch.py max_fps:=1.0  # 1FPS，适合长时间采集"
echo "   ros2 launch multi_camera_collector collector.launch.py max_fps:=10.0 # 10FPS，适合动态场景"
echo "   ros2 launch multi_camera_collector collector.launch.py max_fps:=0    # 无限制，谨慎使用！"
echo ""

echo "4️⃣  调试模式 (详细日志):"
echo "   ros2 launch multi_camera_collector collector.launch.py log_level:=debug"
echo ""

echo "5️⃣  组合参数:"
echo "   ros2 launch multi_camera_collector collector.launch.py \\"
echo "     output_dir:=/home/user/dataset \\"
echo "     max_fps:=2.0 \\"
echo "     log_level:=info"
echo ""

# 检查话题是否存在
echo "🔍 检查相机话题状态..."
topics=(
    "/camera/color/image_raw"
    "/camera/color/camera_info"
    "/camera/depth/image_rect_raw" 
    "/camera/depth/camera_info"
    "/camera/color/image_raw_femto"
    "/camera/color/camera_info_femto"
    "/camera/depth/image_raw_femto"
    "/camera/depth/camera_info_femto"
)

for topic in "${topics[@]}"; do
    if ros2 topic list | grep -q "^$topic$"; then
        echo "✅ $topic - 已发现"
    else
        echo "❌ $topic - 未发现"
    fi
done

echo ""
echo "💡 提示："
echo "   - 确保所有相机话题都在发布数据"
echo "   - 使用 'ros2 topic hz [topic_name]' 检查话题频率"
echo "   - 按 Ctrl+C 优雅关闭采集节点"
echo "   - 采集的数据将按时间戳配对保存"

echo ""
echo "🚀 准备好后，选择上述任一命令启动数据采集！"
echo "=========================================="