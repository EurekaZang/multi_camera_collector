# Camera Info JSON 格式说明

`multi_camera_collector` 将相机的标定信息保存为 JSON 格式，便于后续处理和分析。

## JSON文件结构

每个 camera_info JSON 文件包含以下完整的相机标定信息：

```json
{
  "header": {
    "stamp": {
      "sec": 1678886400,
      "nanosec": 123456789
    },
    "frame_id": "camera_color_optical_frame"
  },
  "height": 480,
  "width": 640,
  "distortion_model": "plumb_bob",
  "D": [
    0.123456,
    -0.234567,
    0.001234,
    0.005678,
    0.0
  ],
  "K": [
    525.123,  0.0,      320.456,
    0.0,      525.789,  240.123,
    0.0,      0.0,      1.0
  ],
  "R": [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  ],
  "P": [
    525.123,  0.0,      320.456,  0.0,
    0.0,      525.789,  240.123,  0.0,
    0.0,      0.0,      1.0,      0.0
  ],
  "binning_x": 0,
  "binning_y": 0,
  "roi": {
    "x_offset": 0,
    "y_offset": 0,
    "height": 0,
    "width": 0,
    "do_rectify": false
  }
}
```

## 字段说明

### Header 信息
- **stamp**: 时间戳信息
  - **sec**: 秒部分
  - **nanosec**: 纳秒部分
- **frame_id**: 坐标系ID

### 图像尺寸
- **height**: 图像高度（像素）
- **width**: 图像宽度（像素）

### 畸变模型
- **distortion_model**: 畸变模型类型（通常为 "plumb_bob"）
- **D**: 畸变参数数组 [k1, k2, t1, t2, k3]
  - **k1, k2, k3**: 径向畸变系数
  - **t1, t2**: 切向畸变系数

### 相机内参矩阵
- **K**: 3x3 内参矩阵（行优先存储）
  ```
  [fx,  0, cx,
    0, fy, cy,
    0,  0,  1]
  ```
  - **fx, fy**: 焦距（像素单位）
  - **cx, cy**: 主点坐标（像素单位）

### 校正矩阵
- **R**: 3x3 校正旋转矩阵（行优先存储）
  - 对于单目相机通常为单位矩阵

### 投影矩阵
- **P**: 3x4 投影矩阵（行优先存储）
  ```
  [fx',  0, cx', Tx,
    0, fy', cy', Ty,
    0,   0,   1,  0]
  ```

### 像素合并
- **binning_x**: X方向像素合并因子
- **binning_y**: Y方向像素合并因子

### 感兴趣区域 (ROI)
- **x_offset**: ROI的X偏移
- **y_offset**: ROI的Y偏移
- **height**: ROI高度
- **width**: ROI宽度
- **do_rectify**: 是否进行校正

## 使用示例

### Python 读取示例

```python
import json

def load_camera_info(json_file):
    """加载camera_info JSON文件"""
    with open(json_file, 'r') as f:
        camera_info = json.load(f)
    return camera_info

def extract_intrinsics(camera_info):
    """提取相机内参"""
    K = camera_info['K']
    fx, fy = K[0], K[4]
    cx, cy = K[2], K[5]
    return fx, fy, cx, cy

def extract_distortion(camera_info):
    """提取畸变参数"""
    D = camera_info['D']
    k1, k2, t1, t2, k3 = D[:5] if len(D) >= 5 else D + [0] * (5 - len(D))
    return k1, k2, t1, t2, k3

# 使用示例
camera_info = load_camera_info('1678886400123456789_rgb_camera_info.json')
fx, fy, cx, cy = extract_intrinsics(camera_info)
k1, k2, t1, t2, k3 = extract_distortion(camera_info)

print(f"内参: fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}")
print(f"畸变: k1={k1:.6f}, k2={k2:.6f}, t1={t1:.6f}, t2={t2:.6f}, k3={k3:.6f}")
```

### OpenCV 标定矩阵转换

```python
import numpy as np

def camera_info_to_opencv(camera_info):
    """将camera_info转换为OpenCV格式"""
    # 内参矩阵
    K = np.array(camera_info['K']).reshape(3, 3)
    
    # 畸变参数
    D = np.array(camera_info['D'])
    
    return K, D

# 示例用法
K, D = camera_info_to_opencv(camera_info)
print("OpenCV内参矩阵K:")
print(K)
print("OpenCV畸变参数D:")
print(D)
```

## 文件命名对应关系

每个时间戳对应的文件：

```
1678886400123456789.png                    # RGB图像
1678886400123456789.png                    # 深度图像 (在depth文件夹中)
1678886400123456789_rgb_camera_info.json   # RGB相机标定信息
1678886400123456789_depth_camera_info.json # 深度相机标定信息
```

这种命名方式确保了：
1. 时间戳完全匹配
2. RGB和深度数据完美对应
3. 标定信息与图像数据同步
4. 便于批量处理和分析

## 注意事项

1. **时间同步**: 所有文件的时间戳都经过精确同步
2. **数据完整性**: 每个时间戳都包含完整的RGB-D和标定信息
3. **格式兼容性**: JSON格式便于各种编程语言读取
4. **精度保持**: 所有浮点数保持原始精度

这种结构化的数据保存方式为后续的SLAM、3D重建、机器学习等应用提供了完整、准确的数据支持。