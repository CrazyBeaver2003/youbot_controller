#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from youbot_interfaces.msg import BoundingBoxArray, BoundingBox
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObjectCoordinateFinder(Node):
    def __init__(self):
        super().__init__('object_coordinate_finder')
        
        # Параметры позиции камеры относительно base_link (из Webots модели)
        # Эти значения нужно взять из URDF или модели Webots
        self.declare_parameter('camera_x', 0.28)  # Примерная позиция камеры YouBot
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.05)
        
        self.camera_offset_x = self.get_parameter('camera_x').value
        self.camera_offset_y = self.get_parameter('camera_y').value
        self.camera_offset_z = self.get_parameter('camera_z').value
        
        # Параметры камеры - будем получать из camera_info
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_width = None
        self.camera_height = None
        
        # Bridge для конвертации изображений
        self.bridge = CvBridge()
        
        # Подписки
        self.depth_image = None
        self.detected_objects = None
        
        # Подписка на camera_info для получения параметров камеры
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/range_finder/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/range_finder/image',
            self.depth_callback,
            10
        )
        
        self.detection_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/detected_objects',
            self.detection_callback,
            10
        )
        
        # Публикация координат объектов
        self.points_publisher = self.create_publisher(
            PointStamped,
            '/object_coordinates',
            10
        )
        
        self.get_logger().info('Object Coordinate Finder initialized.')
    
    def camera_info_callback(self, msg):
        """Получаем параметры камеры из camera_info"""
        if self.fx is None:
            self.fx = msg.k[0]  # K[0,0]
            self.fy = msg.k[4]  # K[1,1]
            self.cx = msg.k[2]  # K[0,2]
            self.cy = msg.k[5]  # K[1,2]
            self.camera_width = msg.width
            self.camera_height = msg.height
            
            self.get_logger().info(
                f'Camera params from camera_info: '
                f'fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}, '
                f'size={self.camera_width}x{self.camera_height}'
            )
    
    def depth_callback(self, msg):
        """Сохраняем последнее изображение глубины"""
        try:
            # RangeFinder в Webots обычно возвращает 32FC1 (float32)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def detection_callback(self, msg):
        """Обрабатываем обнаруженные объекты"""
        if self.depth_image is None:
            self.get_logger().warn('No depth image available yet.')
            return
        
        if self.fx is None:
            self.get_logger().warn('Camera parameters not received yet.')
            return
        
        try:
            self.detected_objects = msg
            
            # Обрабатываем каждый обнаруженный объект
            for bbox in msg.boxes:
                self.process_object(bbox, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')
            import traceback
            traceback.print_exc()
    
    def process_object(self, bbox: BoundingBox, header):
        """
        Обрабатываем один обнаруженный объект:
        1. Получаем координаты центра на изображении
        2. Читаем глубину в этой точке
        3. Конвертируем в 3D координаты относительно камеры
        4. Трансформируем в систему координат base_link
        """
        try:
            # Координаты центра объекта на изображении
            u = int(bbox.center_x)
            v = int(bbox.center_y)
            
            # Проверяем, что координаты в пределах изображения
            if u < 0 or u >= self.depth_image.shape[1] or v < 0 or v >= self.depth_image.shape[0]:
                self.get_logger().warn(f'Object center out of bounds: ({u}, {v})')
                return
            
            # Получаем глубину в точке центра объекта
            # В Webots RangeFinder возвращает расстояние напрямую
            depth = float(self.depth_image[v, u])
            
            # Проверяем валидность глубины
            if np.isnan(depth) or np.isinf(depth) or depth <= 0.0 or depth > 100.0:
                self.get_logger().warn(f'Invalid depth at ({u}, {v}): {depth}')
                return
            
            # Конвертируем 2D координаты + глубина в 3D координаты камеры
            # Используем модель pinhole камеры
            x_camera = float((u - self.cx) * depth / self.fx)
            y_camera = float((v - self.cy) * depth / self.fy)
            z_camera = float(depth)
            
            # В Webots RangeFinder обычно:
            # - Z направлен вперед (глубина)
            # - X направлен вправо
            # - Y направлен вниз
            
            # Трансформируем в систему координат base_link
            # Камера установлена на высоте camera_offset_z и смещена вперед на camera_offset_x
            point_base_link = PointStamped()
            point_base_link.header.stamp = header.stamp
            point_base_link.header.frame_id = 'base_link'
            
            # Координаты в системе камеры -> координаты в системе base_link
            # Предполагаем, что камера смотрит вперед без поворота
            point_base_link.point.x = float(self.camera_offset_x + z_camera)  # Вперед
            point_base_link.point.y = float(self.camera_offset_y - x_camera)  # Влево/вправо (инверсия)
            point_base_link.point.z = float(self.camera_offset_z - y_camera)  # Вверх/вниз (инверсия)
            
            # Публикуем результат
            self.points_publisher.publish(point_base_link)
            
            self.get_logger().info(
                f'Object "{bbox.class_name}" (conf={bbox.confidence:.2f}) detected at:\n'
                f'  Image pixel: ({u}, {v})\n'
                f'  Depth: {depth:.3f}m\n'
                f'  Camera coords: x={x_camera:.3f}, y={y_camera:.3f}, z={z_camera:.3f}\n'
                f'  Base_link coords: x={point_base_link.point.x:.3f}, '
                f'y={point_base_link.point.y:.3f}, z={point_base_link.point.z:.3f}'
            )
                
        except Exception as e:
            self.get_logger().error(f'Error processing object: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCoordinateFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
