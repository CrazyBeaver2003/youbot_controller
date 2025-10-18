import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import numpy as np
import math

class RangeFinderCenterPixel(Node):
    def __init__(self):
        super().__init__('range_finder_test')
        
        self.bridge = CvBridge()
        
        # Подписка на Range Finder (обычно публикуется как Image)
        self.range_sub = self.create_subscription(
            Image,
            '/range_finder/image',
            self.range_callback,
            10
        )
        
        # Подписка на камеру
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_color',
            self.camera_callback,
            10
        )
        
        # Если range-finder публикуется как PointCloud2
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/range_finder/point_cloud',
            self.pointcloud_callback,
            10
        )
        
        # Подписка на лидар
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Если лидар публикуется как PointCloud2
        self.lidar_pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar/point_cloud',
            self.lidar_pointcloud_callback,
            10
        )
        
        self.get_logger().info('Range Finder Center Pixel Test started')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /range_finder/image')
        self.get_logger().info('  - /range_finder/point_cloud')
        self.get_logger().info('  - /camera/image_color')
        self.get_logger().info('  - /scan')
        self.get_logger().info('  - /lidar/point_cloud')

    def range_callback(self, msg):
        """
        RangeFinder в Webots публикуется как Image с глубиной
        msg.encoding обычно '32FC1' (float32, один канал)
        """
        # Конвертируем в numpy array
        range_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        height, width = range_image.shape
        
        # Центральный пиксель
        center_x = width // 2
        center_y = height // 2
        
        # Получаем расстояние в центре
        center_distance = range_image[center_y, center_x]
        
        self.get_logger().info(f'\n=== RANGE FINDER (Image) ===')
        self.get_logger().info(f'Size: {width}x{height}')
        self.get_logger().info(f'Center pixel: ({center_x}, {center_y})')
        self.get_logger().info(f'Center distance: {center_distance:.3f} m')
        
        # Если нужна дистанция в определенной области (например, 3x3 вокруг центра)
        roi_size = 3
        roi_y_start = max(0, center_y - roi_size // 2)
        roi_y_end = min(height, center_y + roi_size // 2 + 1)
        roi_x_start = max(0, center_x - roi_size // 2)
        roi_x_end = min(width, center_x + roi_size // 2 + 1)
        
        roi = range_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        avg_distance = np.mean(roi[np.isfinite(roi)])  # Исключаем inf/nan
        
        self.get_logger().info(f'Average distance in 3x3 ROI: {avg_distance:.3f} m')

    def camera_callback(self, msg):
        """Обработка изображения с камеры"""
        try:
            camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            height, width, _ = camera_image.shape
            center_x = width // 2
            center_y = height // 2
            
            # Цвет центрального пикселя
            center_color = camera_image[center_y, center_x]
            
            self.get_logger().info(f'\n=== CAMERA ===')
            self.get_logger().info(f'Size: {width}x{height}')
            self.get_logger().info(f'Center pixel color (BGR): {center_color}')
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')

    def pointcloud_callback(self, msg):
        """
        Если RangeFinder публикуется как PointCloud2
        """
        try:
            # Получаем размеры облака точек
            width = msg.width
            height = msg.height
            
            # Центральные координаты
            center_x = width // 2
            center_y = height // 2
            
            # Центральный индекс (row-major order)
            center_index = center_y * width + center_x
            
            # Читаем точки как генератор и конвертируем в список
            points_generator = point_cloud2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=False
            )
            
            # Конвертируем генератор в список
            points_list = list(points_generator)
            
            self.get_logger().info(f'\n=== RANGE FINDER (PointCloud2) ===')
            self.get_logger().info(f'Size: {width}x{height}')
            self.get_logger().info(f'Total points: {len(points_list)}')
            self.get_logger().info(f'Center index: {center_index}')
            
            if center_index < len(points_list):
                center_point = points_list[center_index]
                
                # Проверяем тип данных
                if isinstance(center_point, (list, tuple)):
                    x, y, z = center_point[0], center_point[1], center_point[2]
                elif isinstance(center_point, np.ndarray):
                    x, y, z = float(center_point[0]), float(center_point[1]), float(center_point[2])
                else:
                    # Если это структурированный массив numpy
                    x = float(center_point[0])
                    y = float(center_point[1]) 
                    z = float(center_point[2])
                
                self.get_logger().info(f'Center point: x={x:.3f}, y={y:.3f}, z={z:.3f}')
                
                # Расстояние до центральной точки
                distance = np.sqrt(x**2 + y**2 + z**2)
                self.get_logger().info(f'Distance to center: {distance:.3f} m')
            else:
                self.get_logger().warn(f'Center index {center_index} out of range')
                
        except Exception as e:
            self.get_logger().error(f'PointCloud callback error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def lidar_callback(self, msg):
        """
        Обработка данных лидара (LaserScan)
        Получаем расстояние прямо перед роботом
        """
        try:
            num_readings = len(msg.ranges)
            
            # Находим центральный луч (прямо перед роботом)
            # angle_min обычно -PI, angle_max обычно +PI
            # Центр = 0 радиан (прямо вперед)
            center_index = num_readings // 2
            
            # Получаем расстояние в центре
            center_distance = msg.ranges[center_index]
            
            # Вычисляем угол центрального луча
            center_angle = msg.angle_min + center_index * msg.angle_increment
            center_angle_deg = math.degrees(center_angle)
            
            self.get_logger().info(f'\n=== LIDAR (LaserScan) ===')
            self.get_logger().info(f'Total readings: {num_readings}')
            self.get_logger().info(f'Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°')
            self.get_logger().info(f'Angular resolution: {math.degrees(msg.angle_increment):.2f}°')
            self.get_logger().info(f'Center index: {center_index}')
            self.get_logger().info(f'Center angle: {center_angle_deg:.1f}°')
            self.get_logger().info(f'Center distance: {center_distance:.3f} m')
            
            # Получаем среднее расстояние в переднем секторе (±5 градусов)
            angle_tolerance = math.radians(5)  # ±5 градусов
            front_indices = []
            
            for i in range(num_readings):
                angle = msg.angle_min + i * msg.angle_increment
                if abs(angle) <= angle_tolerance:
                    front_indices.append(i)
            
            if front_indices:
                front_distances = [msg.ranges[i] for i in front_indices 
                                 if msg.range_min <= msg.ranges[i] <= msg.range_max]
                
                if front_distances:
                    avg_front_distance = np.mean(front_distances)
                    min_front_distance = np.min(front_distances)
                    max_front_distance = np.max(front_distances)
                    
                    self.get_logger().info(f'Front sector (±5°): {len(front_indices)} rays')
                    self.get_logger().info(f'  Average: {avg_front_distance:.3f} m')
                    self.get_logger().info(f'  Min: {min_front_distance:.3f} m')
                    self.get_logger().info(f'  Max: {max_front_distance:.3f} m')
            
            # Находим ближайшее препятствие во всех направлениях
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_distance = np.min(valid_ranges)
                min_index = msg.ranges.index(min_distance)
                min_angle = msg.angle_min + min_index * msg.angle_increment
                min_angle_deg = math.degrees(min_angle)
                
                self.get_logger().info(f'Closest obstacle: {min_distance:.3f} m at {min_angle_deg:.1f}°')
                
        except Exception as e:
            self.get_logger().error(f'Lidar callback error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def lidar_pointcloud_callback(self, msg):
        """
        Обработка данных лидара как PointCloud2
        """
        try:
            # Читаем точки
            points_generator = point_cloud2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            )
            
            points_list = list(points_generator)
            
            self.get_logger().info(f'\n=== LIDAR (PointCloud2) ===')
            self.get_logger().info(f'Total points: {len(points_list)}')
            
            if points_list:
                # Находим точку прямо перед роботом (x > 0, y ≈ 0)
                front_points = []
                for point in points_list:
                    if isinstance(point, (list, tuple)):
                        x, y, z = point[0], point[1], point[2]
                    else:
                        x, y, z = float(point[0]), float(point[1]), float(point[2])
                    
                    # Фильтруем точки в переднем секторе (±5 градусов)
                    angle = math.atan2(y, x)
                    if abs(angle) <= math.radians(5) and x > 0:
                        distance = math.sqrt(x**2 + y**2 + z**2)
                        front_points.append((x, y, z, distance))
                
                if front_points:
                    # Сортируем по расстоянию
                    front_points.sort(key=lambda p: p[3])
                    
                    # Ближайшая точка
                    closest = front_points[0]
                    self.get_logger().info(f'Closest front point:')
                    self.get_logger().info(f'  Position: x={closest[0]:.3f}, y={closest[1]:.3f}, z={closest[2]:.3f}')
                    self.get_logger().info(f'  Distance: {closest[3]:.3f} m')
                    
                    # Средняя дистанция первых 10 точек
                    avg_distances = [p[3] for p in front_points[:10]]
                    avg_dist = np.mean(avg_distances)
                    self.get_logger().info(f'Average of 10 closest: {avg_dist:.3f} m')
                
        except Exception as e:
            self.get_logger().error(f'Lidar PointCloud callback error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = RangeFinderCenterPixel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()