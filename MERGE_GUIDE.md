# 数据集合并工具使用指南

## 概述

`dataset_merger` 是一个用于合并多个由 `multi_camera_collector` 包生成的数据集的工具。它可以将多个独立采集的数据集合并为一个统一的数据集，并生成与 `make_dataset.py` 相同格式的CSV索引文件。

## 功能特性

- ✅ 合并多个数据集到单个目录
- ✅ 自动处理时间戳冲突
- ✅ 生成统一的CSV索引文件
- ✅ 保留所有camera_info元数据
- ✅ 支持多种冲突解决策略
- ✅ 生成合并统计信息

## 安装

确保已经构建并安装了 `multi_camera_collector` 包：

```bash
cd ~/ros2_ws
colcon build --packages-select multi_camera_collector
source install/setup.bash
```

## 使用方法

### 方法1：使用Python命令（推荐）

```bash
merge_datasets --output merged_dataset dataset1/ dataset2/ dataset3/
```

### 方法2：使用Shell脚本

```bash
./merge_datasets.sh --output merged_dataset dataset1/ dataset2/ dataset3/
```

### 方法3：直接运行Python模块

```bash
python3 -m multi_camera_collector.dataset_merger --output merged_dataset dataset1/ dataset2/
```

## 命令行参数

### 必需参数

- `datasets`: 要合并的数据集路径（至少需要2个）
- `--output, -o`: 合并后数据集的输出路径

### 可选参数

- `--format, -f`: CSV文件输出格式
  - `combined` (默认): 生成单个包含所有相机数据的CSV文件
  - `separate`: 为每个相机生成单独的CSV文件
  - `both`: 同时生成组合和单独的CSV文件

- `--conflicts, -c`: 时间戳冲突解决策略
  - `offset` (默认): 为冲突的时间戳添加偏移量（推荐）
  - `skip`: 跳过冲突的文件
  - `overwrite`: 用后续数据集覆盖冲突的文件

- `--verbose, -v`: 显示详细输出信息

- `--help, -h`: 显示帮助信息

## 使用示例

### 示例1：基本合并

合并两个数据集：

```bash
merge_datasets --output merged_dataset dataset1/ dataset2/
```

### 示例2：合并多个数据集

合并三个或更多数据集：

```bash
merge_datasets --output merged_dataset dataset1/ dataset2/ dataset3/ dataset4/
```

### 示例3：生成单独的CSV文件

为每个相机生成单独的CSV文件：

```bash
merge_datasets --output merged_dataset --format separate dataset1/ dataset2/
```

### 示例4：同时生成组合和单独的CSV文件

```bash
merge_datasets --output merged_dataset --format both dataset1/ dataset2/
```

### 示例5：使用覆盖策略

当遇到重复时间戳时，使用后面数据集的数据覆盖：

```bash
merge_datasets --output merged_dataset --conflicts overwrite dataset1/ dataset2/
```

### 示例6：详细输出模式

查看详细的合并过程信息：

```bash
merge_datasets --output merged_dataset --verbose dataset1/ dataset2/
```

## 输入数据集要求

每个输入数据集应遵循以下目录结构：

```
dataset/
├── camera/
│   ├── rgb/                    # 标准相机RGB图像
│   ├── depth/                  # 标准相机深度图像
│   └── camera_info/            # 标准相机参数文件
└── camera_femto/
    ├── rgb/                    # Femto相机RGB图像
    ├── depth/                  # Femto相机深度图像
    └── camera_info/            # Femto相机参数文件
```

## 输出数据集结构

合并后的数据集将具有与输入数据集相同的结构，并包含以下额外文件：

```
merged_dataset/
├── camera/
│   ├── rgb/                    # 合并的RGB图像
│   ├── depth/                  # 合并的深度图像
│   └── camera_info/            # 合并的相机参数文件
├── camera_femto/
│   ├── rgb/
│   ├── depth/
│   └── camera_info/
├── dataset.csv                 # 组合的CSV索引文件
├── camera_dataset.csv          # (可选) 标准相机单独的CSV
├── camera_femto_dataset.csv    # (可选) Femto相机单独的CSV
├── dataset_statistics.json     # 数据集统计信息
└── merge_info.json             # 合并信息记录
```

## 输出CSV文件格式

生成的CSV文件包含以下列（与 `make_dataset.py` 生成的格式完全相同）：

| 列名 | 说明 |
|------|------|
| `camera_type` | 相机类型 (camera/camera_femto) |
| `timestamp` | 纳秒级时间戳 |
| `rgb_path` | RGB图像相对路径 |
| `depth_path` | 深度图像相对路径 |
| `rgb_camera_info_path` | RGB相机信息文件相对路径 |
| `depth_camera_info_path` | 深度相机信息文件相对路径 |
| `rgb_absolute_path` | RGB图像绝对路径 |
| `depth_absolute_path` | 深度图像绝对路径 |
| `rgb_camera_info_absolute_path` | RGB相机信息文件绝对路径 |
| `depth_camera_info_absolute_path` | 深度相机信息文件绝对路径 |
| `rgb_width` | RGB图像宽度 |
| `rgb_height` | RGB图像高度 |
| `depth_width` | 深度图像宽度 |
| `depth_height` | 深度图像高度 |

## 冲突解决策略详解

### Offset策略（推荐）

这是默认且推荐的策略。当检测到时间戳冲突时，会为后续数据集的所有时间戳添加偏移量。

**优点：**
- 保留所有数据
- 不丢失任何图像
- 时间顺序保持正确

**适用场景：**
- 多次采集的数据需要完整合并
- 时间戳重复是由于独立采集会话造成的

**示例：**
```bash
merge_datasets --output merged --conflicts offset dataset1/ dataset2/
```

### Skip策略

跳过时间戳冲突的文件，只保留第一个遇到的。

**优点：**
- 简单直接
- 避免数据重复

**缺点：**
- 会丢失部分数据

**适用场景：**
- 数据集部分重叠
- 只需要唯一时间戳的数据

**示例：**
```bash
merge_datasets --output merged --conflicts skip dataset1/ dataset2/
```

### Overwrite策略

用后续数据集的文件覆盖已存在的同时间戳文件。

**优点：**
- 可以用新数据替换旧数据

**缺点：**
- 会丢失被覆盖的数据

**适用场景：**
- 需要用新采集的数据替换旧数据
- 修正之前采集的错误数据

**示例：**
```bash
merge_datasets --output merged --conflicts overwrite dataset1/ dataset2/
```

## 合并信息文件

合并完成后，会生成 `merge_info.json` 文件，记录合并的详细信息：

```json
{
  "merge_date": "2025-10-18T10:30:45.123456",
  "output_path": "/path/to/merged_dataset",
  "source_datasets": [
    "/path/to/dataset1",
    "/path/to/dataset2",
    "/path/to/dataset3"
  ],
  "num_datasets": 3
}
```

## 统计信息文件

与 `make_dataset.py` 类似，会生成 `dataset_statistics.json` 文件：

```json
{
  "dataset_path": "/path/to/merged_dataset",
  "total_image_pairs": 1500,
  "cameras": {
    "camera": {
      "count": 750,
      "earliest_timestamp": 1634567890000000000,
      "latest_timestamp": 1634567920000000000,
      "duration_seconds": 30.0
    },
    "camera_femto": {
      "count": 750,
      "earliest_timestamp": 1634567890000000000,
      "latest_timestamp": 1634567920000000000,
      "duration_seconds": 30.0
    }
  }
}
```

## 在深度学习中使用

合并后的数据集可以直接用于深度学习训练，使用方式与单个数据集完全相同：

### PyTorch示例

```python
import pandas as pd
from PIL import Image
from torch.utils.data import Dataset

class MergedRGBDDataset(Dataset):
    def __init__(self, csv_file, dataset_root):
        self.data = pd.read_csv(csv_file)
        self.dataset_root = dataset_root
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        row = self.data.iloc[idx]
        
        # 使用相对路径加载图像
        rgb_path = self.dataset_root / row['rgb_path']
        depth_path = self.dataset_root / row['depth_path']
        
        rgb = Image.open(rgb_path)
        depth = Image.open(depth_path)
        
        return rgb, depth, row['timestamp']

# 使用合并后的数据集
dataset = MergedRGBDDataset('merged_dataset/dataset.csv', 'merged_dataset')
```

## 注意事项

1. **磁盘空间**: 合并操作会复制所有文件，确保有足够的磁盘空间

2. **时间戳唯一性**: 使用 `offset` 策略时，时间戳会被修改以避免冲突

3. **数据完整性**: 只有同时包含RGB、深度和camera_info的完整数据对才会被包含

4. **相对路径**: CSV中的路径是相对于数据集根目录的，便于数据集移动

5. **大数据集**: 对于非常大的数据集，合并操作可能需要较长时间

## 故障排除

### 问题1: 找不到 merge_datasets 命令

**解决方案：**
```bash
cd ~/ros2_ws
colcon build --packages-select multi_camera_collector
source install/setup.bash
```

### 问题2: 数据集结构不正确

**错误信息：**
```
⚠️  警告: 数据集结构不正确: /path/to/dataset
```

**解决方案：**
检查数据集是否包含必要的目录结构（camera/rgb, camera/depth等）

### 问题3: 磁盘空间不足

**解决方案：**
- 清理不需要的数据
- 使用更大的磁盘
- 分批合并数据集

### 问题4: 时间戳冲突

**解决方案：**
使用 `offset` 策略（默认）或根据需要选择其他策略

## 相关工具

- `multi_camera_collector`: 数据采集节点
- `make_dataset`: 为单个数据集生成CSV文件
- `dataset_generator`: CSV生成器模块

## 技术支持

如有问题或建议，请：
1. 查看包的主README文件
2. 检查相关文档（DATASET_GUIDE.md等）
3. 提交issue到项目仓库

## 更新日志

- **v1.0.0** (2025-10-18): 初始版本
  - 基本合并功能
  - 三种冲突解决策略
  - CSV文件生成
  - 统计信息记录
