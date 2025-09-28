# Multi-Camera Data Collector

一个生产级的 ROS 2 Foxy 多相机RGB-D数据采集包，专为同步采集和保存来自Intel Realsense相机和Orbbec Femto相机的图像数据和相机标定参数而设计。

## 功能特性

- ✅ **双相机支持**: 同时采集标准相机和Femto相机数据
- ✅ **完整数据采集**: 同时保存RGB图像、深度图像和相机标定信息(camera_info)
- ✅ **精确时间同步**: 使用`message_filters.ApproximateTimeSynchronizer`确保所有数据时间同步
- ✅ **高精度保存**: 深度图像保持16位精度，RGB图像保存为标准格式
- ✅ **纳秒级时间戳**: 文件名使用纳秒级时间戳确保唯一性和关联性
- ✅ **标定信息保存**: camera_info保存为JSON格式，便于后续处理
- ✅ **优雅关闭**: 支持Ctrl+C优雅关闭并显示采集统计
- ✅ **参数化配置**: 使用ROS 2参数系统配置输出目录
- ✅ **生产就绪**: 包含错误处理、日志记录和线程安全

## 系统要求

- **ROS 2**: Foxy Fitzroy
- **Python**: 3.8+
- **操作系统**: Ubuntu 20.04 (推荐)

## 依赖包

### ROS 2 依赖
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `message_filters`

### Python 依赖
- `opencv-python`
- `numpy`

## 安装与构建

### 1. 克隆到工作空间

```bash
cd ~/ros2_ws/src
# 将此包复制到您的ROS 2工作空间
cp -r /path/to/multi_camera_collector .
```

### 2. 安装依赖

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建包

```bash
cd ~/ros2_ws
colcon build --packages-select multi_camera_collector
```

### 4. 源化环境

```bash
source ~/ros2_ws/install/setup.bash
```

## 使用方法

### 基本启动

使用默认设置启动数据采集：

```bash
ros2 launch multi_camera_collector collector.launch.py
```

### 自定义输出目录

指定自定义数据保存路径：

```bash
ros2 launch multi_camera_collector collector.launch.py output_dir:=/home/user/my_dataset
```

### 调试模式

启用详细日志输出：

```bash
ros2 launch multi_camera_collector collector.launch.py log_level:=debug
```

### 直接运行节点

也可以直接运行节点（不推荐用于生产环境）：

```bash
ros2 run multi_camera_collector multi_camera_collector --ros-args -p output_dir:=/path/to/dataset
```

## 话题订阅

节点订阅以下话题：

### 标准相机
- **RGB图像**: `/camera/color/image_raw` (sensor_msgs/Image)
- **RGB相机信息**: `/camera/color/camera_info` (sensor_msgs/CameraInfo)
- **深度图像**: `/camera/depth/image_rect_raw` (sensor_msgs/Image)
- **深度相机信息**: `/camera/depth/camera_info` (sensor_msgs/CameraInfo)

### Femto相机
- **RGB图像**: `/camera/color/image_raw_femto` (sensor_msgs/Image)
- **RGB相机信息**: `/camera/color/camera_info_femto` (sensor_msgs/CameraInfo)
- **深度图像**: `/camera/depth/image_raw_femto` (sensor_msgs/Image)
- **深度相机信息**: `/camera/depth/camera_info_femto` (sensor_msgs/CameraInfo)

## 目录结构

采集的数据将按以下结构组织：

```
<output_dir>/
├── camera/                      # 标准相机数据
│   ├── rgb/                    # RGB图像 (8位BGR)
│   │   ├── 1678886400123456789.png
│   │   └── ...
│   ├── depth/                  # 深度图像 (16位单通道)
│   │   ├── 1678886400123456789.png
│   │   └── ...
│   └── camera_info/            # 相机标定信息 (JSON格式)
│       ├── 1678886400123456789_rgb_camera_info.json
│       ├── 1678886400123456789_depth_camera_info.json
│       └── ...
└── camera_femto/               # Femto相机数据
    ├── rgb/                    # RGB图像 (8位BGR)
    │   ├── 1678886400987654321.png
    │   └── ...
    ├── depth/                  # 深度图像 (16位单通道)
    │   ├── 1678886400987654321.png
    │   └── ...
    └── camera_info/            # 相机标定信息 (JSON格式)
        ├── 1678886400987654321_rgb_camera_info.json
        ├── 1678886400987654321_depth_camera_info.json
        └── ...
```

## 文件命名规则

### 图像文件
- 格式: `{纳秒级时间戳}.png`
- 示例: `1678886400123456789.png`

### Camera Info文件
- RGB相机信息: `{纳秒级时间戳}_rgb_camera_info.json`
- 深度相机信息: `{纳秒级时间戳}_depth_camera_info.json`
- 示例: `1678886400123456789.png`
- 这确保了RGB和深度图像的完美对应关系

## 技术细节

### 时间同步

- 使用`message_filters.ApproximateTimeSynchronizer`
- 同步4个数据流：RGB图像 + RGB相机信息 + 深度图像 + 深度相机信息
- 同步容忍度(slop): 0.1秒
- 队列大小: 10个消息

### 数据格式

- **RGB图像**: 转换为BGR8格式 (OpenCV标准)
- **深度图像**: 保持16UC1格式 (16位无符号单通道)
- **相机信息**: 保存为JSON格式，包含内参矩阵、畸变参数、ROI等完整标定信息

### 线程安全

- 使用`threading.Lock`保护计数器
- 确保多线程环境下的数据一致性

## 优雅关闭

按`Ctrl+C`停止采集时，节点会显示详细的统计信息：

```
============================================================
数据采集完成总结:
标准相机 (camera): 已保存 150 组RGB-D图像对
Femto相机 (camera_femto): 已保存 147 组RGB-D图像对
总计: 297 组图像对
数据保存位置: /home/user/dataset
============================================================
```

## 故障排除

### 常见问题

1. **没有接收到图像数据**
   - 检查相机是否正常发布话题
   - 使用`ros2 topic list`确认话题存在
   - 使用`ros2 topic echo /camera/color/image_raw`测试数据流

2. **图像同步问题**
   - 检查时间戳是否合理
   - 考虑调整`slop`参数（在代码中修改）
   - 确认两个相机的时间同步

3. **权限问题**
   - 确保输出目录有写权限
   - 检查磁盘空间是否充足

### 调试命令

```bash
# 检查话题列表
ros2 topic list

# 监控话题频率
ros2 topic hz /camera/color/image_raw

# 查看节点信息
ros2 node info /multi_camera_collector

# 查看节点参数
ros2 param list /multi_camera_collector
```

## 性能优化

- 根据需要调整队列大小和同步参数
- 对于高频采集，考虑使用SSD存储
- 监控CPU和内存使用情况

## 许可证

Apache-2.0

## 作者

Eureka Zang

---

**注意**: 此包专为ROS 2 Foxy Fitzroy设计，确保您的系统环境匹配。