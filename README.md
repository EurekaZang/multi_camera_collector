# Multi-Camera Data Collector

一个生产级的 ROS 2 Foxy 多相机RGB-D数据采集包，专为同步采集和保存来自Intel Realsense相机和Orbbec Femto相机的图像数据和相机标定参数而设计。

## 功能特性

- ✅ **双相机支持**: 同时采集标准相机和Femto相机数据
- ✅ **完整数据采集**: 同时保存RGB图像、深度图像和相机标定信息(camera_info)
- ✅ **精确时间同步**: 使用`message_filters.ApproximateTimeSynchronizer`确保所有数据时间同步
- ✅ **智能FPS控制**: 可配置最大采集帧率，避免产生过多图片，默认5FPS
- ✅ **高精度保存**: 深度图像保持16位精度，RGB图像保存为标准格式
- ✅ **纳秒级时间戳**: 文件名使用纳秒级时间戳确保唯一性和关联性
- ✅ **标定信息保存**: camera_info保存为JSON格式，便于后续处理
- ✅ **优雅关闭**: 支持Ctrl+C优雅关闭并显示采集统计
- ✅ **参数化配置**: 使用ROS 2参数系统配置输出目录和采集帧率
- ✅ **生产就绪**: 包含错误处理、日志记录和线程安全
- 🆕 **一键数据集制作**: 自动生成深度学习友好的CSV索引文件
- 🆕 **多种输出格式**: 支持组合式和分离式数据集文件
- 🆕 **数据集验证**: 智能验证数据完整性和结构正确性
- 🆕 **统计分析**: 自动生成数据集统计报告

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
- `pandas` (用于数据集制作功能)

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

使用默认设置启动数据采集（5FPS限制）：

```bash
ros2 launch multi_camera_collector collector.launch.py
```

### 自定义输出目录

指定自定义数据保存路径：

```bash
ros2 launch multi_camera_collector collector.launch.py output_dir:=/home/user/my_dataset
```

### 自定义FPS设置

控制采集帧率以避免产生过多图片：

```bash
# 设置最大1FPS（适合长时间采集）
ros2 launch multi_camera_collector collector.launch.py max_fps:=1.0

# 设置最大10FPS（适合快速场景变化）
ros2 launch multi_camera_collector collector.launch.py max_fps:=10.0

# 禁用FPS限制（采集所有数据，谨慎使用！）
ros2 launch multi_camera_collector collector.launch.py max_fps:=0
```

### 组合参数

同时设置多个参数：

```bash
ros2 launch multi_camera_collector collector.launch.py \
  output_dir:=/home/user/dataset \
  max_fps:=2.0 \
  log_level:=info
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

## 🆕 数据集制作功能

采集完数据后，您可以使用内置的数据集制作工具生成便于深度学习使用的CSV索引文件。

### 快速开始

```bash
# 一键生成数据集CSV文件（推荐）
./make_dataset.sh /path/to/your/dataset

# 使用ROS2命令
ros2 run multi_camera_collector make_dataset /path/to/your/dataset

# 直接运行Python脚本
python3 multi_camera_collector/make_dataset.py /path/to/your/dataset
```

### 输出格式选项

```bash
# 生成组合CSV文件（默认，推荐用于深度学习）
./make_dataset.sh /path/to/dataset --format combined

# 为每个相机生成单独的CSV文件
./make_dataset.sh /path/to/dataset --format separate

# 同时生成组合和单独的CSV文件
./make_dataset.sh /path/to/dataset --format both
```

### 生成的文件

数据集制作工具会在数据集根目录生成以下文件：

- `dataset.csv` - 主数据索引文件（包含所有相机数据，推荐用于深度学习）
- `camera_dataset.csv` - 标准相机数据索引（仅在separate/both模式下生成）
- `camera_femto_dataset.csv` - Femto相机数据索引（仅在separate/both模式下生成）
- `dataset_statistics.json` - 数据集统计信息

### CSV文件格式

生成的CSV文件包含以下列，便于深度学习框架使用：

| 列名 | 说明 |
|-----|------|
| `camera_type` | 相机类型 (camera/camera_femto) |
| `timestamp` | 纳秒级时间戳 |
| `rgb_path` | RGB图像相对路径 |
| `depth_path` | 深度图像相对路径 |
| `rgb_camera_info_path` | RGB相机信息文件相对路径 |
| `depth_camera_info_path` | 深度相机信息文件相对路径 |
| `rgb_absolute_path` | RGB图像绝对路径 |
| `depth_absolute_path` | 深度图像绝对路径 |
| `rgb_width` | RGB图像宽度 |
| `rgb_height` | RGB图像高度 |
| `depth_width` | 深度图像宽度 |
| `depth_height` | 深度图像高度 |

### 深度学习集成示例

```python
import pandas as pd
import cv2
from torch.utils.data import Dataset

class MultiCameraDataset(Dataset):
    def __init__(self, csv_file, dataset_root):
        self.data_frame = pd.read_csv(csv_file)
        self.dataset_root = Path(dataset_root)
    
    def __getitem__(self, idx):
        row = self.data_frame.iloc[idx]
        
        # 加载RGB和深度图像
        rgb_path = self.dataset_root / row['rgb_path']
        depth_path = self.dataset_root / row['depth_path']
        
        rgb_image = cv2.imread(str(rgb_path))
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        
        return {
            'rgb': rgb_image,
            'depth': depth_image,
            'camera_type': row['camera_type'],
            'timestamp': row['timestamp']
        }
```

详细的数据集制作功能使用指南请参考 [DATASET_GUIDE.md](DATASET_GUIDE.md)。

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