#!/usr/bin/env python3
"""
Нода для автоматической установки начальной позы робота в AMCL.
Публикует начальную позу на топик /initialpose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        
        # Параметры начальной позы (можно настроить через launch-файл)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)  # В радианах
        
        # Получаем значения параметров
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.yaw = self.get_parameter('yaw').value
        
        # Publisher для начальной позы
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Даем время системе запуститься
        time.sleep(2.0)
        
        # Публикуем начальную позу
        self.publish_initial_pose()
        
        self.get_logger().info(f'Initial pose set: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad')
        
        # Завершаем ноду после публикации
        time.sleep(1.0)
        self.get_logger().info('Initial pose published successfully. Shutting down.')
        rclpy.shutdown()
    
    def publish_initial_pose(self):
        """Публикует начальную позу робота."""
        msg = PoseWithCovarianceStamped()
        
        # Заголовок
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Позиция
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        
        # Ориентация (конвертируем yaw в кватернион)
        # Кватернион для поворота вокруг оси Z:
        # q = [0, 0, sin(yaw/2), cos(yaw/2)]
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # Ковариационная матрица (6x6, но используется только позиция и yaw)
        # Диагональная матрица с небольшой неопределенностью
        msg.pose.covariance = [
            0.25, 0.0,  0.0, 0.0, 0.0, 0.0,  # x variance
            0.0,  0.25, 0.0, 0.0, 0.0, 0.0,  # y variance
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # z variance (не используется для 2D)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # roll variance (не используется)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # pitch variance (не используется)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.068 # yaw variance
        ]
        
        # Публикуем несколько раз для надежности
        for i in range(3):
            self.pose_pub.publish(msg)
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SetInitialPose()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in set_initial_pose node: {e}')
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
