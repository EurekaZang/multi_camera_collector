# 多相机数据集制作功能 - 快速使用指南

## 完整工作流程示例

### 1. 数据采集

```bash
# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 启动数据采集（保存到 ./my_dataset 目录）
ros2 run multi_camera_collector multi_camera_collector \
  --ros-args \
  -p output_dir:=./my_dataset \
  -p max_fps:=5.0

# 或使用launch文件
ros2 launch multi_camera_collector collector.launch.py \
  output_dir:=./my_dataset \
  max_fps:=5.0
```

### 2. 数据集制作

数据采集完成后，使用以下任一方法生成CSV索引文件：

#### 方法1: Shell脚本（推荐）

```bash
# 基本使用
./make_dataset.sh ./my_dataset

# 生成所有格式的CSV文件
./make_dataset.sh ./my_dataset --format both --verbose
```

#### 方法2: ROS2命令

```bash
# 使用ROS2包中的工具
ros2 run multi_camera_collector make_dataset ./my_dataset

# 指定输出格式
ros2 run multi_camera_collector make_dataset ./my_dataset --format both
```

#### 方法3: 直接运行Python脚本

```bash
# 进入包目录
cd src/multi_camera_collector/multi_camera_collector/

# 运行Python脚本
python3 make_dataset.py ../../../my_dataset --format combined
```

### 3. 验证结果

生成完成后，检查输出文件：

```bash
ls -la my_dataset/
```

应该看到：
- `dataset.csv` - 主数据索引文件
- `dataset_statistics.json` - 统计信息
- 可能还有 `camera_dataset.csv` 和 `camera_femto_dataset.csv`（取决于格式选择）

### 4. 查看CSV内容

```bash
# 查看CSV文件头部
head -5 my_dataset/dataset.csv

# 统计行数
wc -l my_dataset/dataset.csv

# 查看统计信息
cat my_dataset/dataset_statistics.json
```

## 深度学习使用示例

### PyTorch Dataset

```python
import pandas as pd
import cv2
import torch
from torch.utils.data import Dataset, DataLoader
from pathlib import Path

class MultiCameraDataset(Dataset):
    def __init__(self, csv_file, dataset_root, transform=None):
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
            'timestamp': row['timestamp'],
            'rgb_width': row['rgb_width'],
            'rgb_height': row['rgb_height']
        }
        
        if self.transform:
            sample = self.transform(sample)
        
        return sample

# 使用示例
if __name__ == "__main__":
    dataset = MultiCameraDataset(
        csv_file='./my_dataset/dataset.csv',
        dataset_root='./my_dataset'
    )
    
    # 创建数据加载器
    dataloader = DataLoader(dataset, batch_size=16, shuffle=True)
    
    # 测试加载
    for batch_idx, sample in enumerate(dataloader):
        print(f"Batch {batch_idx}:")
        print(f"  RGB shape: {sample['rgb'].shape}")
        print(f"  Depth shape: {sample['depth'].shape}")
        print(f"  Camera types: {sample['camera_type']}")
        if batch_idx >= 2:  # 只打印前3个batch
            break
```

### 数据分析脚本

```python
import pandas as pd
import json
import matplotlib.pyplot as plt

def analyze_dataset(csv_file, stats_file):
    """分析数据集统计信息"""
    
    # 读取CSV文件
    df = pd.read_csv(csv_file)
    print(f"数据集总样本数: {len(df)}")
    
    # 按相机类型统计
    camera_counts = df['camera_type'].value_counts()
    print(f"\n按相机类型统计:")
    for camera, count in camera_counts.items():
        print(f"  {camera}: {count} 样本")
    
    # 读取统计文件
    with open(stats_file, 'r') as f:
        stats = json.load(f)
    
    print(f"\n数据集统计信息:")
    print(f"总图像对数: {stats['total_image_pairs']}")
    
    for camera_type, camera_stats in stats['cameras'].items():
        duration = camera_stats['duration_seconds']
        print(f"{camera_type}:")
        print(f"  样本数: {camera_stats['count']}")
        print(f"  时长: {duration:.1f}s")
        print(f"  平均FPS: {camera_stats['count']/duration:.2f}")

# 使用示例
if __name__ == "__main__":
    analyze_dataset('./my_dataset/dataset.csv', './my_dataset/dataset_statistics.json')
```

## 常见使用场景

### 1. 机器人视觉训练数据

```bash
# 采集训练数据
ros2 run multi_camera_collector multi_camera_collector \
  --ros-args -p output_dir:=./robot_vision_dataset -p max_fps:=10.0

# 制作数据集
./make_dataset.sh ./robot_vision_dataset --format combined
```

### 2. 深度估计数据集

```bash
# 采集深度估计数据
ros2 run multi_camera_collector multi_camera_collector \
  --ros-args -p output_dir:=./depth_estimation_data -p max_fps:=2.0

# 为每个相机生成单独的CSV文件以便比较
./make_dataset.sh ./depth_estimation_data --format separate
```

### 3. 多视角数据集

```bash
# 采集多视角数据
ros2 run multi_camera_collector multi_camera_collector \
  --ros-args -p output_dir:=./multi_view_dataset -p max_fps:=5.0

# 生成所有格式的CSV文件
./make_dataset.sh ./multi_view_dataset --format both
```

## 故障排除

### 构建问题

如果遇到构建错误：

```bash
# 安装缺失的依赖
pip3 install pandas numpy opencv-python

# 重新构建
cd /path/to/workspace
colcon build --packages-select multi_camera_collector --cmake-clean-cache
```

### 运行问题

如果脚本无法执行：

```bash
# 检查并添加执行权限
chmod +x make_dataset.sh
chmod +x multi_camera_collector/make_dataset.py

# 检查Python路径
which python3
python3 --version
```

### 数据问题

如果数据集验证失败：

```bash
# 检查目录结构
tree my_dataset/

# 检查文件数量
find my_dataset/ -name "*.png" | wc -l
find my_dataset/ -name "*.json" | wc -l
```