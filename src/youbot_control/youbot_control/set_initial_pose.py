#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Ждем, пока топик будет готов
        time.sleep(1.0)
        
        # Публикуем начальную позицию
        self.publish_initial_pose()
        
        self.get_logger().info('Начальная позиция установлена: x=0.0, y=0.0, yaw=0.0')
        
        # Завершаем ноду после публикации
        time.sleep(0.5)
        rclpy.shutdown()
    
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Начальная позиция робота (x, y, z)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Начальная ориентация (quaternion для yaw=0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Ковариация (неопределенность позиции)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        
        # Публикуем несколько раз для надежности
        for _ in range(5):
            self.publisher.publish(msg)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    

if __name__ == '__main__':
    main()
