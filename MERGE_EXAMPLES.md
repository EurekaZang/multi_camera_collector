# 数据集合并快速示例

## 基础示例

### 1. 合并两个数据集

最简单的使用方式：

```bash
# 确保环境已经设置
source ~/ros2_ws/install/setup.bash

# 合并dataset1和dataset2
merge_datasets --output merged_dataset dataset1/ dataset2/
```

### 2. 合并多个数据集

合并三个或更多数据集：

```bash
merge_datasets --output merged_dataset \
    ~/datasets/session1/ \
    ~/datasets/session2/ \
    ~/datasets/session3/
```

## 高级示例

### 3. 为每个相机生成单独的CSV

如果你需要单独处理每个相机的数据：

```bash
merge_datasets --output merged_dataset \
    --format separate \
    dataset1/ dataset2/
```

这将生成：
- `camera_dataset.csv` - 标准相机的数据
- `camera_femto_dataset.csv` - Femto相机的数据

### 4. 同时生成组合和单独的CSV

```bash
merge_datasets --output merged_dataset \
    --format both \
    dataset1/ dataset2/
```

这将生成：
- `dataset.csv` - 所有相机的组合数据
- `camera_dataset.csv` - 标准相机的数据
- `camera_femto_dataset.csv` - Femto相机的数据

### 5. 使用不同的冲突解决策略

#### Offset策略（默认，推荐）
为后续数据集的时间戳添加偏移量，保留所有数据：

```bash
merge_datasets --output merged_dataset \
    --conflicts offset \
    dataset1/ dataset2/
```

#### Skip策略
跳过重复的时间戳：

```bash
merge_datasets --output merged_dataset \
    --conflicts skip \
    dataset1/ dataset2/
```

#### Overwrite策略
用后续数据集覆盖重复的数据：

```bash
merge_datasets --output merged_dataset \
    --conflicts overwrite \
    dataset1/ dataset2/
```

### 6. 详细输出模式

查看详细的处理过程：

```bash
merge_datasets --output merged_dataset \
    --verbose \
    dataset1/ dataset2/
```

## 使用Shell脚本

如果你更喜欢使用shell脚本：

```bash
cd ~/ros2_ws/src/multi_camera_collector
./merge_datasets.sh --output merged_dataset dataset1/ dataset2/
```

## 使用ROS2 Launch

通过launch文件启动（适合集成到ROS2工作流）：

```bash
ros2 launch multi_camera_collector merge_datasets.launch.py \
    output_dir:=/path/to/merged_dataset \
    dataset1:=/path/to/dataset1 \
    dataset2:=/path/to/dataset2
```

## 实际应用场景

### 场景1：多次采集合并

你在不同时间采集了多个数据集，现在需要合并它们用于训练：

```bash
merge_datasets --output training_dataset \
    ~/data/morning_session/ \
    ~/data/afternoon_session/ \
    ~/data/evening_session/
```

### 场景2：增量数据集

你已经有一个数据集，想要添加新采集的数据：

```bash
merge_datasets --output expanded_dataset \
    --conflicts offset \
    ~/data/original_dataset/ \
    ~/data/new_collection/
```

### 场景3：多地点数据整合

从不同地点采集的数据合并成一个大数据集：

```bash
merge_datasets --output combined_dataset \
    --format both \
    ~/data/location_a/ \
    ~/data/location_b/ \
    ~/data/location_c/
```

## 验证结果

合并完成后，检查生成的文件：

```bash
cd merged_dataset

# 查看目录结构
tree -L 2

# 查看CSV文件
head dataset.csv

# 查看统计信息
cat dataset_statistics.json

# 查看合并信息
cat merge_info.json
```

## 在Python中使用合并后的数据集

```python
import pandas as pd
from pathlib import Path

# 读取合并后的数据集
df = pd.read_csv('merged_dataset/dataset.csv')

print(f"总帧数: {len(df)}")
print(f"相机类型: {df['camera_type'].unique()}")

# 按相机类型分组统计
print("\n各相机帧数:")
print(df.groupby('camera_type').size())

# 查看第一条记录
print("\n第一条记录:")
print(df.iloc[0])
```

## 常见工作流程

### 完整的数据采集到训练流程

```bash
# 1. 采集数据
ros2 launch multi_camera_collector collector.launch.py \
    output_dir:=~/datasets/session1

# 2. 采集更多数据
ros2 launch multi_camera_collector collector.launch.py \
    output_dir:=~/datasets/session2

# 3. 合并数据集
merge_datasets --output ~/datasets/training_set \
    ~/datasets/session1 \
    ~/datasets/session2

# 4. 验证数据集
cd ~/datasets/training_set
python3 << EOF
import pandas as pd
df = pd.read_csv('dataset.csv')
print(f"Total frames: {len(df)}")
print(f"Cameras: {df['camera_type'].unique()}")
EOF

# 5. 现在可以使用training_set进行模型训练
```

## 疑难解答

### 问题：合并失败

```bash
# 检查数据集结构
for dir in dataset1 dataset2; do
    echo "检查 $dir:"
    ls -R $dir | head -20
done
```

### 问题：时间戳冲突太多

```bash
# 使用offset策略（推荐）
merge_datasets --output merged --conflicts offset dataset1/ dataset2/

# 或者检查是否有重复的数据集
diff -r dataset1/ dataset2/
```

### 问题：输出目录已存在

```bash
# 删除旧的输出目录
rm -rf merged_dataset

# 或者使用新的输出路径
merge_datasets --output merged_dataset_v2 dataset1/ dataset2/
```

## 性能提示

1. **磁盘空间**: 确保有足够的空间存储合并后的数据
   ```bash
   du -sh dataset1/ dataset2/  # 查看源数据集大小
   df -h .                      # 查看可用空间
   ```

2. **大数据集**: 对于非常大的数据集，合并可能需要一些时间
   ```bash
   # 使用time命令监控执行时间
   time merge_datasets --output merged dataset1/ dataset2/
   ```

3. **并行处理**: 文件复制是串行的，但可以先合并较小的数据集组
   ```bash
   # 分组合并
   merge_datasets --output group1 dataset1/ dataset2/
   merge_datasets --output group2 dataset3/ dataset4/
   merge_datasets --output final group1/ group2/
   ```

## 更多信息

详细文档请参考：
- [MERGE_GUIDE.md](MERGE_GUIDE.md) - 完整使用指南
- [README.md](README.md) - 包的主文档
- [DATASET_GUIDE.md](DATASET_GUIDE.md) - 数据集结构说明
