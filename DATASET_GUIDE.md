# 数据集制作工具使用指南

## 概述

本文档介绍如何使用 `multi_camera_collector` 包提供的一键数据集制作功能。该功能可以为您采集的RGB-D数据生成便于深度学习使用的CSV索引文件。

## 功能特点

- 🚀 **一键生成**: 自动扫描数据集目录并生成CSV索引文件
- 📊 **多种格式**: 支持组合式和分离式CSV文件格式
- 🔍 **智能验证**: 自动验证数据集完整性和结构
- 📈 **统计信息**: 生成详细的数据集统计报告
- 🎯 **深度学习友好**: CSV格式直接适用于PyTorch/TensorFlow等框架

## 数据集结构

使用数据集制作工具前，请确保您的数据集具有以下目录结构：

```
dataset/                    # 数据集根目录
├── camera/                 # 标准相机数据
│   ├── rgb/               # RGB图像文件 (.png)
│   ├── depth/             # 深度图像文件 (.png)
│   └── camera_info/       # 相机参数文件 (.json)
└── camera_femto/          # Femto相机数据
    ├── rgb/               # RGB图像文件 (.png)
    ├── depth/             # 深度图像文件 (.png)
    └── camera_info/       # 相机参数文件 (.json)
```

## 使用方法

### 方法1: 使用Shell脚本（推荐）

最简单的使用方式是运行提供的Shell脚本：

```bash
# 基本用法 - 生成组合CSV文件
./make_dataset.sh /path/to/your/dataset

# 生成所有类型的CSV文件
./make_dataset.sh /path/to/your/dataset --format both

# 显示详细输出
./make_dataset.sh /path/to/your/dataset --verbose
```

### 方法2: 使用ROS2命令

如果已经安装了ROS2包，可以直接使用ROS2命令：

```bash
# 确保ROS2环境已设置
source /opt/ros/foxy/setup.bash  # 或您的ROS2版本
source install/setup.bash

# 运行数据集制作工具
ros2 run multi_camera_collector make_dataset /path/to/your/dataset

# 使用不同的输出格式
ros2 run multi_camera_collector make_dataset /path/to/your/dataset --format both
```

### 方法3: 直接运行Python脚本

```bash
# 进入包目录
cd src/multi_camera_collector/multi_camera_collector/

# 运行Python脚本
python3 make_dataset.py /path/to/your/dataset

# 或使用数据集生成器
python3 dataset_generator.py /path/to/your/dataset --format combined
```

## 命令行选项

### 输出格式选项 (`--format` / `-f`)

- **`combined`** (默认): 生成单个CSV文件包含所有相机数据
  - 输出文件: `dataset.csv`
  - 推荐用于深度学习训练
  
- **`separate`**: 为每个相机生成单独的CSV文件
  - 输出文件: `camera_dataset.csv`, `camera_femto_dataset.csv`
  - 适用于单独分析某个相机的数据
  
- **`both`**: 同时生成组合和分离的CSV文件
  - 输出所有上述文件
  - 适用于需要灵活性的场景

### 其他选项

- **`--verbose` / `-v`**: 显示详细的处理信息
- **`--help` / `-h`**: 显示帮助信息

## 输出文件说明

### CSV文件结构

生成的CSV文件包含以下列：

| 列名 | 类型 | 说明 |
|-----|------|------|
| `camera_type` | string | 相机类型 (`camera` 或 `camera_femto`) |
| `timestamp` | int64 | 纳秒级时间戳 |
| `rgb_path` | string | RGB图像相对路径 |
| `depth_path` | string | 深度图像相对路径 |
| `rgb_camera_info_path` | string | RGB相机信息文件相对路径 |
| `depth_camera_info_path` | string | 深度相机信息文件相对路径 |
| `rgb_absolute_path` | string | RGB图像绝对路径 |
| `depth_absolute_path` | string | 深度图像绝对路径 |
| `rgb_camera_info_absolute_path` | string | RGB相机信息文件绝对路径 |
| `depth_camera_info_absolute_path` | string | 深度相机信息文件绝对路径 |
| `rgb_width` | int | RGB图像宽度 |
| `rgb_height` | int | RGB图像高度 |
| `depth_width` | int | 深度图像宽度 |
| `depth_height` | int | 深度图像高度 |

### 统计文件

`dataset_statistics.json` 包含数据集的统计信息：

```json
{
  "dataset_path": "/path/to/dataset",
  "total_image_pairs": 1500,
  "cameras": {
    "camera": {
      "count": 750,
      "earliest_timestamp": 1633024800000000000,
      "latest_timestamp": 1633024950000000000,
      "duration_seconds": 150.0
    },
    "camera_femto": {
      "count": 750,
      "earliest_timestamp": 1633024800000000000,
      "latest_timestamp": 1633024950000000000,
      "duration_seconds": 150.0
    }
  }
}
```

## 深度学习集成示例

### PyTorch Dataset类

```python
import pandas as pd
import cv2
import torch
from torch.utils.data import Dataset
from pathlib import Path

class MultiCameraDataset(Dataset):
    def __init__(self, csv_file, dataset_root, transform=None):
        """
        Args:
            csv_file (str): CSV文件路径
            dataset_root (str): 数据集根目录
            transform: 数据增强变换
        """
        self.data_frame = pd.read_csv(csv_file)
        self.dataset_root = Path(dataset_root)
        self.transform = transform
    
    def __len__(self):
        return len(self.data_frame)
    
    def __getitem__(self, idx):
        row = self.data_frame.iloc[idx]
        
        # 加载RGB图像
        rgb_path = self.dataset_root / row['rgb_path']
        rgb_image = cv2.imread(str(rgb_path))
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        # 加载深度图像
        depth_path = self.dataset_root / row['depth_path']
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        
        sample = {
            'rgb': rgb_image,
            'depth': depth_image,
            'camera_type': row['camera_type'],
            'timestamp': row['timestamp']
        }
        
        if self.transform:
            sample = self.transform(sample)
        
        return sample

# 使用示例
dataset = MultiCameraDataset(
    csv_file='/path/to/dataset/dataset.csv',
    dataset_root='/path/to/dataset'
)
dataloader = torch.utils.data.DataLoader(dataset, batch_size=32, shuffle=True)
```

### TensorFlow数据管道

```python
import tensorflow as tf
import pandas as pd

def create_dataset_from_csv(csv_file, dataset_root, batch_size=32):
    """从CSV文件创建TensorFlow数据集"""
    df = pd.read_csv(csv_file)
    
    def load_image_pair(rgb_path, depth_path):
        # 加载RGB图像
        rgb_image = tf.io.read_file(rgb_path)
        rgb_image = tf.image.decode_png(rgb_image, channels=3)
        rgb_image = tf.cast(rgb_image, tf.float32) / 255.0
        
        # 加载深度图像
        depth_image = tf.io.read_file(depth_path)
        depth_image = tf.image.decode_png(depth_image, channels=1, dtype=tf.uint16)
        depth_image = tf.cast(depth_image, tf.float32)
        
        return rgb_image, depth_image
    
    # 构建完整路径
    rgb_paths = [str(Path(dataset_root) / path) for path in df['rgb_path']]
    depth_paths = [str(Path(dataset_root) / path) for path in df['depth_path']]
    
    # 创建数据集
    dataset = tf.data.Dataset.from_tensor_slices((rgb_paths, depth_paths))
    dataset = dataset.map(load_image_pair, num_parallel_calls=tf.data.AUTOTUNE)
    dataset = dataset.batch(batch_size)
    dataset = dataset.prefetch(tf.data.AUTOTUNE)
    
    return dataset
```

## 常见问题解决

### 1. 数据集结构验证失败

**问题**: 工具报告数据集结构不完整

**解决方案**:
- 确保数据集目录包含 `camera` 和/或 `camera_femto` 子目录
- 检查每个相机目录下是否有 `rgb/`, `depth/`, `camera_info/` 子目录
- 确保有实际的图像文件存在

### 2. 时间戳不匹配

**问题**: RGB和深度图像的时间戳不匹配

**解决方案**:
- 检查数据采集时的时间同步设置
- 使用 `data_collector.py` 中的 `slop` 参数调整时间同步容差
- 手动检查文件命名是否正确

### 3. 依赖包缺失

**问题**: 运行时提示缺少 pandas 等依赖

**解决方案**:
```bash
# 安装必要的Python包
pip3 install pandas numpy opencv-python

# 或者重新构建ROS2包
cd /path/to/workspace
colcon build --packages-select multi_camera_collector
```

### 4. 权限问题

**问题**: 脚本无法执行

**解决方案**:
```bash
# 添加执行权限
chmod +x make_dataset.sh
chmod +x multi_camera_collector/make_dataset.py
```

## 性能优化建议

1. **大数据集处理**: 对于包含大量图像的数据集，建议先使用 `--format combined` 生成主索引文件

2. **存储优化**: 使用相对路径可以方便地移动数据集目录

3. **内存管理**: 在深度学习训练中，建议使用数据加载器的多进程功能

4. **文件系统**: 推荐使用SSD存储以提高I/O性能

## 完整工作流程示例

```bash
# 1. 启动数据采集
ros2 run multi_camera_collector multi_camera_collector --ros-args -p output_dir:=./my_dataset -p max_fps:=10.0

# 2. 采集完成后，制作数据集
./make_dataset.sh ./my_dataset --format both --verbose

# 3. 验证生成的文件
ls -la my_dataset/
# 应该看到:
# - dataset.csv
# - camera_dataset.csv  
# - camera_femto_dataset.csv
# - dataset_statistics.json

# 4. 在深度学习代码中使用
python3 your_training_script.py --dataset-csv ./my_dataset/dataset.csv
```

## 技术支持

如果遇到问题，请检查：

1. 日志输出中的错误信息
2. 数据集目录结构是否正确
3. 文件权限是否正确设置
4. 依赖包是否完整安装

更多技术细节请参考源代码中的注释和文档字符串。