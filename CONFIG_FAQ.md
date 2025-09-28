# Multi-Camera Data Collector 配置和FAQ

## 配置参数

### 节点参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `output_dir` | string | `./dataset` | 数据保存的根目录路径 |

### 高级配置

以下配置需要修改源代码 `data_collector.py`:

| 配置项 | 位置 | 默认值 | 描述 |
|--------|------|--------|------|
| 同步容忍度 (slop) | `ApproximateTimeSynchronizer` | 0.1秒 | 允许的最大时间戳差异 |
| 队列大小 | `ApproximateTimeSynchronizer` | 10 | 消息缓冲队列大小 |
| 日志级别 | `self.get_logger()` | INFO | 节点日志输出级别 |

## 话题配置

### 默认话题映射

| 相机类型 | RGB话题 | 深度话题 |
|----------|---------|----------|
| 标准相机 | `/camera/color/image_raw` | `/camera/depth/image_rect_raw` |
| Femto相机 | `/camera/color/image_raw_femto` | `/camera/depth/image_raw_femto` |

### 自定义话题映射

如果您的相机使用不同的话题名称，可以在launch文件中添加重映射：

```python
# 在 collector.launch.py 中修改 remappings 部分
collector_node = Node(
    # ... 其他配置 ...
    remappings=[
        ('/camera/color/image_raw', '/your_camera/rgb/image'),
        ('/camera/depth/image_rect_raw', '/your_camera/depth/image'),
        ('/camera/color/image_raw_femto', '/femto_camera/rgb/image'),
        ('/camera/depth/image_raw_femto', '/femto_camera/depth/image'),
    ]
)
```

## 常见问题解答 (FAQ)

### Q1: 为什么没有保存任何图像？

**A:** 检查以下几点：
1. 确保相机节点正在运行并发布话题：
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /camera/color/image_raw
   ```
2. 检查话题名称是否匹配
3. 确保RGB和深度话题的时间戳在同步容忍度内
4. 检查输出目录的写权限

### Q2: 图像同步不准确怎么办？

**A:** 可以调整同步参数：
1. 增加slop值（容忍更大的时间戳差异）
2. 增加队列大小（保存更多消息用于匹配）
3. 检查相机的时间同步设置

### Q3: 深度图像看起来不正确？

**A:** 确认以下设置：
1. 深度图像保存为16位格式（16UC1）
2. 检查深度相机的输出格式是否正确
3. 使用正确的深度话题（通常是经过校正的）

### Q4: 如何查看保存的深度图像？

**A:** 深度图像是16位格式，需要特殊处理：

```python
import cv2
import numpy as np

# 读取16位深度图像
depth_img = cv2.imread('depth_image.png', cv2.IMREAD_UNCHANGED)

# 转换为可视化格式
depth_colormap = cv2.applyColorMap(
    cv2.convertScaleAbs(depth_img, alpha=0.03), 
    cv2.COLORMAP_JET
)
cv2.imshow('Depth', depth_colormap)
cv2.waitKey(0)
```

### Q5: 内存使用过高怎么办？

**A:** 可以优化以下方面：
1. 减小队列大小
2. 增加同步频率（减小slop值）
3. 监控磁盘空间
4. 考虑压缩保存格式

### Q6: 如何修改文件命名格式？

**A:** 在 `data_collector.py` 中修改回调函数中的文件名生成逻辑：

```python
# 当前格式: timestamp.png
timestamp = rgb_msg.header.stamp.sec * 1000000000 + rgb_msg.header.stamp.nanosec

# 自定义格式示例
import datetime
dt = datetime.datetime.fromtimestamp(rgb_msg.header.stamp.sec)
filename = f"{dt.strftime('%Y%m%d_%H%M%S')}_{rgb_msg.header.stamp.nanosec}.png"
```

### Q7: 支持其他图像格式吗？

**A:** 当前支持PNG格式。要支持其他格式，修改保存代码：

```python
# JPEG格式 (有损压缩，文件更小)
cv2.imwrite(rgb_path.replace('.png', '.jpg'), rgb_image, 
           [cv2.IMWRITE_JPEG_QUALITY, 95])

# TIFF格式 (无损，支持16位)
cv2.imwrite(depth_path.replace('.png', '.tiff'), depth_image)
```

### Q8: 如何在多台机器上运行？

**A:** 确保以下配置：
1. 所有机器在同一ROS网络中
2. 设置正确的 `ROS_DOMAIN_ID`
3. 网络延迟可能需要增加同步容忍度
4. 考虑时钟同步问题

### Q9: 性能优化建议？

**A:** 以下是一些优化建议：

1. **硬件优化**：
   - 使用SSD存储
   - 确保足够的RAM
   - 使用有线网络连接

2. **软件优化**：
   ```python
   # 在回调函数中添加异步保存
   import threading
   
   def save_async(self, image, path):
       threading.Thread(target=cv2.imwrite, args=(path, image)).start()
   ```

3. **系统优化**：
   ```bash
   # 增加系统文件句柄限制
   ulimit -n 4096
   
   # 设置高优先级
   sudo nice -n -10 ros2 launch multi_camera_collector collector.launch.py
   ```

### Q10: 如何验证数据质量？

**A:** 使用提供的验证脚本：

```bash
# 检查文件数量和完整性
python3 -c "
import os
rgb_files = len(os.listdir('dataset/camera/rgb/'))
depth_files = len(os.listdir('dataset/camera/depth/'))
print(f'RGB: {rgb_files}, Depth: {depth_files}')
print('匹配:' if rgb_files == depth_files else '不匹配')
"
```

## 故障排除检查清单

- [ ] ROS 2环境已正确设置
- [ ] 包已成功构建和安装
- [ ] 相机节点正在运行
- [ ] 话题名称正确匹配
- [ ] 输出目录有写权限
- [ ] 磁盘空间充足
- [ ] 网络连接稳定（如果使用网络相机）
- [ ] 时间同步正确

## 获得帮助

如果问题仍未解决，请：

1. 启用调试日志：`log_level:=debug`
2. 检查ROS 2日志文件
3. 使用 `ros2 topic echo` 检查消息内容
4. 检查系统资源使用情况

## 联系信息

- 维护者：ROS 2 机器人软件工程师
- 邮箱：developer@example.com
- 问题追踪：GitHub Issues