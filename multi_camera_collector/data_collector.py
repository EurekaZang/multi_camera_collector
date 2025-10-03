#!/usr/bin/env python3
"""
ROS 2 Multi-Camera Data Collector Node

这个节点订阅两个相机（标准相机和Femto相机）的RGB和深度图像，
通过精确的时间同步将匹配的图像对保存为PNG文件。
"""

import os
import cv2
import numpy as np
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from threading import Lock


class DataCollectorNode(Node):
    """多相机数据采集节点，支持时间同步的RGB-D数据保存"""
    
    def __init__(self):
        super().__init__('multi_camera_collector')
        
        # 声明参数
        self.declare_parameter('output_dir', './dataset')
        self.declare_parameter('max_fps', 5.0)  # 默认最大5FPS，避免产生过多图片
        
        # 初始化变量
        self.bridge = CvBridge()
        self.camera_count = 0  # 标准相机保存的图像对数量
        self.femto_count = 0   # Femto相机保存的图像对数量
        self.count_lock = Lock()  # 线程安全的计数器
        
        # FPS控制相关变量
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        self.min_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0  # 最小时间间隔
        self.last_save_time_camera = 0.0    # 标准相机上次保存时间
        self.last_save_time_femto = 0.0     # Femto相机上次保存时间
        self.fps_lock = Lock()  # FPS控制锁
        
        # 获取输出目录参数
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        
        # 创建目录结构
        self._create_directories()
        
        # 设置订阅器和同步器
        self._setup_subscribers()
        
        self.get_logger().info(f'多相机数据采集节点已启动，数据将保存到: {self.output_dir}')
        self.get_logger().info(f'最大采集帧率: {self.max_fps} FPS (最小间隔: {self.min_interval:.3f}s)')
        if self.max_fps <= 0:
            self.get_logger().warn('FPS限制已禁用 (max_fps <= 0)，将采集所有接收到的数据')
        self.get_logger().info('按 Ctrl+C 优雅关闭节点')
    
    def _create_directories(self):
        """创建必要的目录结构"""
        directories = [
            os.path.join(self.output_dir, 'camera', 'rgb'),
            os.path.join(self.output_dir, 'camera', 'depth'),
            os.path.join(self.output_dir, 'camera', 'camera_info'),
            os.path.join(self.output_dir, 'camera_femto', 'rgb'),
            os.path.join(self.output_dir, 'camera_femto', 'depth'),
            os.path.join(self.output_dir, 'camera_femto', 'camera_info')
        ]
        
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
            
        self.get_logger().info(f'目录结构已创建: {self.output_dir}')
    
    def _setup_subscribers(self):
        """设置订阅器和时间同步器"""
        
        # 标准相机的订阅器
        self.camera_rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw'
        )
        self.camera_depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_rect_raw'
        )
        self.camera_rgb_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/color/camera_info'
        )
        self.camera_depth_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/depth/camera_info'
        )
        
        # Femto相机的订阅器
        self.femto_rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw_femto'
        )
        self.femto_depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw_femto'
        )
        self.femto_rgb_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/color/camera_info_femto'
        )
        self.femto_depth_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/depth/camera_info_femto'
        )
        
        # 标准相机的时间同步器
        # 使用ApproximateTimeSynchronizer处理硬件和网络延迟
        # slop=0.1秒允许最大100ms的时间戳差异
        self.camera_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.camera_rgb_sub, self.camera_depth_sub, 
             self.camera_rgb_info_sub, self.camera_depth_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.camera_synchronizer.registerCallback(self._camera_callback)
        
        # Femto相机的时间同步器
        self.femto_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.femto_rgb_sub, self.femto_depth_sub,
             self.femto_rgb_info_sub, self.femto_depth_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.femto_synchronizer.registerCallback(self._femto_callback)
        
        self.get_logger().info('订阅器和时间同步器已设置完成')
    
    def _should_save_data(self, camera_type):
        """
        检查是否应该根据FPS限制保存数据
        
        Args:
            camera_type (str): 'camera' 或 'femto'
            
        Returns:
            bool: True表示应该保存，False表示跳过
        """
        if self.max_fps <= 0:
            # FPS限制禁用，保存所有数据
            return True
            
        current_time = time.time()
        
        with self.fps_lock:
            if camera_type == 'camera':
                time_since_last = current_time - self.last_save_time_camera
                if time_since_last >= self.min_interval:
                    self.last_save_time_camera = current_time
                    return True
                else:
                    return False
            elif camera_type == 'femto':
                time_since_last = current_time - self.last_save_time_femto
                if time_since_last >= self.min_interval:
                    self.last_save_time_femto = current_time
                    return True
                else:
                    return False
        return False
    
    def _save_camera_info(self, camera_info_msg, file_path):
        """保存camera_info消息为JSON文件"""
        try:
            # 将CameraInfo消息转换为字典
            camera_info_dict = {
                'header': {
                    'stamp': {
                        'sec': camera_info_msg.header.stamp.sec,
                        'nanosec': camera_info_msg.header.stamp.nanosec
                    },
                    'frame_id': camera_info_msg.header.frame_id
                },
                'height': camera_info_msg.height,
                'width': camera_info_msg.width,
                'distortion_model': camera_info_msg.distortion_model,
                'D': list(camera_info_msg.d),
                'K': list(camera_info_msg.k),
                'R': list(camera_info_msg.r),
                'P': list(camera_info_msg.p),
                'binning_x': camera_info_msg.binning_x,
                'binning_y': camera_info_msg.binning_y,
                'roi': {
                    'x_offset': camera_info_msg.roi.x_offset,
                    'y_offset': camera_info_msg.roi.y_offset,
                    'height': camera_info_msg.roi.height,
                    'width': camera_info_msg.roi.width,
                    'do_rectify': camera_info_msg.roi.do_rectify
                }
            }
            
            # 保存为JSON文件
            with open(file_path, 'w') as f:
                json.dump(camera_info_dict, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f'保存camera_info失败 {file_path}: {str(e)}')
    
    def _camera_callback(self, rgb_msg, depth_msg, rgb_info_msg, depth_info_msg):
        """标准相机的同步回调函数"""
        try:
            # 检查是否应该根据FPS限制保存数据
            if not self._should_save_data('camera'):
                self.get_logger().debug('标准相机数据被FPS限制跳过')
                return
                
            # 使用RGB消息的时间戳作为文件名（纳秒级精度）
            timestamp = rgb_msg.header.stamp.sec * 1000000000 + rgb_msg.header.stamp.nanosec
            
            # 转换RGB图像（转换为BGR格式用于OpenCV）
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            # 转换深度图像（保持16UC1格式，保留精确深度信息）
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # 保存图像文件
            rgb_path = os.path.join(self.output_dir, 'camera', 'rgb', f'{timestamp}.png')
            depth_path = os.path.join(self.output_dir, 'camera', 'depth', f'{timestamp}.png')
            
            # 保存camera_info文件
            rgb_info_path = os.path.join(self.output_dir, 'camera', 'camera_info', f'{timestamp}_rgb_camera_info.json')
            depth_info_path = os.path.join(self.output_dir, 'camera', 'camera_info', f'{timestamp}_depth_camera_info.json')
            
            # 使用OpenCV保存图像
            cv2.imwrite(rgb_path, rgb_image)
            cv2.imwrite(depth_path, depth_image)
            
            # 保存camera_info
            self._save_camera_info(rgb_info_msg, rgb_info_path)
            self._save_camera_info(depth_info_msg, depth_info_path)
            
            # 线程安全地更新计数器
            with self.count_lock:
                self.camera_count += 1
                
            self.get_logger().debug(f'标准相机图像对已保存: {timestamp}')
            
        except Exception as e:
            self.get_logger().error(f'保存标准相机数据时出错: {str(e)}')
    
    def _femto_callback(self, rgb_msg, depth_msg, rgb_info_msg, depth_info_msg):
        """Femto相机的同步回调函数"""
        try:
            # 检查是否应该根据FPS限制保存数据
            if not self._should_save_data('femto'):
                self.get_logger().debug('Femto相机数据被FPS限制跳过')
                return
                
            # 使用RGB消息的时间戳作为文件名（纳秒级精度）
            timestamp = rgb_msg.header.stamp.sec * 1000000000 + rgb_msg.header.stamp.nanosec
            
            # 转换RGB图像（转换为BGR格式用于OpenCV）
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            # 转换深度图像（保持16UC1格式，保留精确深度信息）
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # 保存图像文件
            rgb_path = os.path.join(self.output_dir, 'camera_femto', 'rgb', f'{timestamp}.png')
            depth_path = os.path.join(self.output_dir, 'camera_femto', 'depth', f'{timestamp}.png')
            
            # 保存camera_info文件
            rgb_info_path = os.path.join(self.output_dir, 'camera_femto', 'camera_info', f'{timestamp}_rgb_camera_info.json')
            depth_info_path = os.path.join(self.output_dir, 'camera_femto', 'camera_info', f'{timestamp}_depth_camera_info.json')
            
            # 使用OpenCV保存图像
            cv2.imwrite(rgb_path, rgb_image)
            cv2.imwrite(depth_path, depth_image)
            
            # 保存camera_info
            self._save_camera_info(rgb_info_msg, rgb_info_path)
            self._save_camera_info(depth_info_msg, depth_info_path)
            
            # 线程安全地更新计数器
            with self.count_lock:
                self.femto_count += 1
                
            self.get_logger().debug(f'Femto相机图像对已保存: {timestamp}')
            
        except Exception as e:
            self.get_logger().error(f'保存Femto相机数据时出错: {str(e)}')
    
    def get_statistics(self):
        """获取采集统计信息"""
        with self.count_lock:
            return self.camera_count, self.femto_count


def main(args=None):
    """主函数，处理节点生命周期和优雅关闭"""
    rclpy.init(args=args)
    
    # 创建节点实例
    collector_node = DataCollectorNode()
    
    try:
        # 启动节点
        rclpy.spin(collector_node)
        
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        collector_node.get_logger().info('收到关闭信号，正在优雅关闭...')
        
    finally:
        # 获取最终统计信息
        camera_count, femto_count = collector_node.get_statistics()
        
        # 打印总结信息
        collector_node.get_logger().info('=' * 60)
        collector_node.get_logger().info('数据采集完成总结:')
        collector_node.get_logger().info(f'FPS设置: {collector_node.max_fps} FPS {"(无限制)" if collector_node.max_fps <= 0 else ""}')
        collector_node.get_logger().info(f'标准相机 (camera): 已保存 {camera_count} 组RGB-D图像对')
        collector_node.get_logger().info(f'Femto相机 (camera_femto): 已保存 {femto_count} 组RGB-D图像对')
        collector_node.get_logger().info(f'总计: {camera_count + femto_count} 组图像对')
        collector_node.get_logger().info(f'数据保存位置: {collector_node.output_dir}')
        collector_node.get_logger().info('=' * 60)
        
        # 清理资源
        collector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()