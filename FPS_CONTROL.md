# FPS控制功能说明文档

## 概述

Multi-Camera Data Collector 现在支持智能的FPS（帧率）控制功能，可以有效防止相机以过高频率产生大量图片文件，同时确保数据采集的质量和系统资源的合理使用。

## 功能特性

### 🎯 核心功能
- **独立FPS控制**: 每个相机（标准相机和Femto相机）独立控制采集频率
- **时间精确控制**: 基于系统时钟的精确时间间隔控制
- **线程安全**: 使用锁机制确保多线程环境下的安全性
- **灵活配置**: 支持命令行参数动态配置
- **智能跳帧**: 在高频数据流中智能跳过不必要的帧

### 📊 默认设置
- **默认FPS**: 5.0 FPS
- **最小时间间隔**: 0.2秒（1/5秒）
- **禁用设置**: max_fps ≤ 0 时禁用FPS限制

## 使用方法

### 基本配置

```bash
# 使用默认5FPS
ros2 launch multi_camera_collector collector.launch.py

# 设置1FPS，适合长时间静态场景采集
ros2 launch multi_camera_collector collector.launch.py max_fps:=1.0

# 设置10FPS，适合动态场景
ros2 launch multi_camera_collector collector.launch.py max_fps:=10.0

# 禁用FPS限制（采集所有数据，谨慎使用！）
ros2 launch multi_camera_collector collector.launch.py max_fps:=0
```

### 高级组合

```bash
# 低频长时间采集
ros2 launch multi_camera_collector collector.launch.py \
  max_fps:=0.5 \
  output_dir:=/data/long_term_collection

# 高精度短时间采集
ros2 launch multi_camera_collector collector.launch.py \
  max_fps:=30.0 \
  output_dir:=/data/high_frequency \
  log_level:=debug
```

## 技术实现

### 算法原理

FPS控制基于时间间隔检查机制：

```python
# 伪代码
current_time = system_time()
min_interval = 1.0 / max_fps
time_since_last = current_time - last_save_time

if time_since_last >= min_interval:
    # 保存数据
    save_data()
    last_save_time = current_time
else:
    # 跳过此帧
    skip_frame()
```

### 独立控制

每个相机维护独立的时间戳：
- `last_save_time_camera`: 标准相机最后保存时间
- `last_save_time_femto`: Femto相机最后保存时间

这确保了两个相机可以在不同时刻触发保存，最大化数据采集效率。

### 线程安全

使用`threading.Lock`保护时间戳更新：

```python
with self.fps_lock:
    if time_since_last >= min_interval:
        self.last_save_time = current_time
        return True
```

## FPS设置建议

### 根据应用场景选择

| 应用场景 | 推荐FPS | 说明 |
|----------|---------|------|
| **静态场景长期监控** | 0.1-1.0 | 节省存储空间，适合安防、环境监测 |
| **慢速运动捕获** | 1.0-5.0 | 适合人体动作、机器人移动 |
| **标准数据采集** | 5.0-10.0 | 平衡质量和存储，通用场景 |
| **快速运动分析** | 10.0-30.0 | 适合运动分析、快速场景变化 |
| **无限制采集** | 0 (禁用) | 采集所有数据，用于特殊研究 |

### 存储空间估算

假设单次采集生成6个文件（2个RGB + 2个深度 + 4个JSON）：

| FPS | 每小时文件数 | 每天文件数 | 估算存储空间/天 |
|-----|-------------|-----------|----------------|
| 1.0 | 21,600 | 518,400 | ~25-50GB |
| 5.0 | 108,000 | 2,592,000 | ~125-250GB |
| 10.0 | 216,000 | 5,184,000 | ~250-500GB |
| 30.0 | 648,000 | 15,552,000 | ~750GB-1.5TB |

*注意：实际大小取决于图像分辨率和内容复杂度*

## 性能影响

### CPU使用
- **FPS控制开销**: 极低，每次仅涉及简单的时间比较
- **跳帧优化**: 减少图像转换和文件I/O操作
- **线程锁影响**: 微秒级锁定时间，可忽略

### 内存使用
- **额外内存开销**: 几乎为零
- **缓冲区优化**: 跳帧减少内存峰值使用

### 磁盘I/O
- **显著减少**: 按比例减少文件写入操作
- **更稳定的I/O**: 避免突发性大量写入

## 日志信息

### 启动信息
```
[INFO] 多相机数据采集节点已启动，数据将保存到: ./dataset
[INFO] 最大采集帧率: 5.0 FPS (最小间隔: 0.200s)
```

### 调试信息（log_level:=debug时）
```
[DEBUG] 标准相机数据被FPS限制跳过
[DEBUG] Femto相机数据被FPS限制跳过
```

### 关闭统计
```
============================================================
数据采集完成总结:
FPS设置: 5.0 FPS 
标准相机 (camera): 已保存 150 组RGB-D图像对
Femto相机 (camera_femto): 已保存 147 组RGB-D图像对
总计: 297 组图像对
数据保存位置: ./dataset
============================================================
```

## 故障排除

### 常见问题

#### Q: FPS限制不生效？
**A**: 检查以下项目：
1. 确认`max_fps > 0`
2. 检查相机数据流频率是否高于设定FPS
3. 使用调试模式查看跳帧日志

#### Q: 数据采集频率不稳定？
**A**: 可能的原因：
1. 系统负载过高影响精确计时
2. 相机数据流本身不稳定
3. 网络延迟（如果使用网络相机）

#### Q: 设置很低的FPS但仍产生很多文件？
**A**: 检查：
1. 确认参数传递正确：`max_fps:=1.0`（注意冒号和等号）
2. 检查是否有多个采集节点运行
3. 确认时间同步器的容忍度设置

### 调试命令

```bash
# 检查当前参数设置
ros2 param list /multi_camera_collector
ros2 param get /multi_camera_collector max_fps

# 监控采集频率
watch -n 1 'ls -1 dataset/camera/rgb/ | wc -l'

# 查看详细日志
ros2 launch multi_camera_collector collector.launch.py \
  max_fps:=2.0 log_level:=debug
```

## 最佳实践

### 生产环境建议
1. **根据需求选择合适的FPS**：避免过度采集
2. **监控存储空间**：定期清理或归档旧数据
3. **系统资源监控**：确保足够的CPU和I/O能力
4. **网络带宽考虑**：网络相机需要考虑带宽限制

### 开发和测试
1. **使用较低FPS进行初始测试**：避免产生大量测试数据
2. **调试时启用详细日志**：了解系统行为
3. **基准测试**：测试不同FPS下的系统性能

---

**注意**: FPS控制是在ROS消息同步之后进行的，这意味着时间同步的精度不受FPS限制影响。被跳过的帧仍然参与时间同步计算，只是不会被保存到磁盘。