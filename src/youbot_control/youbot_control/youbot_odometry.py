import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')

        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.gps_subscriber = self.create_subscription(PointStamped, 'my_robot/gps', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu = Imu()
        self.gps = PointStamped()

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_time = None
        
        # Фильтрация скорости (простое экспоненциальное сглаживание)
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0
        self.alpha = 0.3  # Коэффициент сглаживания (0-1, меньше = более гладко)
        
        # Флаг инициализации
        self.initialized = False

        self.timer = self.create_timer(0.1, self.publish_odometry)

    def gps_callback(self, msg):
        self.gps = msg

    def imu_callback(self, msg):
        self.imu = msg
    
    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        gps_values = self.gps.point
        x = gps_values.x
        y = gps_values.y
        z = gps_values.z

        angular_velocities = self.imu.angular_velocity
        angular_x = angular_velocities.x
        angular_y = angular_velocities.y
        angular_z = angular_velocities.z

        orientation = self.imu.orientation
        x_orient = orientation.x
        y_orient = orientation.y
        z_orient = orientation.z
        w_orient = orientation.w

        # Инициализация при первом запуске
        if not self.initialized:
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            self.prev_time = current_time
            self.initialized = True
            return

        # Вычисление реального dt
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            
            # Защита от деления на ноль и слишком больших dt
            if dt > 0.001 and dt < 1.0:
                # Вычисление мгновенных скоростей
                raw_vx = (x - self.prev_x) / dt 
                raw_vy = (y - self.prev_y) / dt
                raw_vz = (z - self.prev_z) / dt
                
                # Экспоненциальное сглаживание
                self.filtered_vx = self.alpha * raw_vx + (1 - self.alpha) * self.filtered_vx
                self.filtered_vy = self.alpha * raw_vy + (1 - self.alpha) * self.filtered_vy
                self.filtered_vz = self.alpha * raw_vz + (1 - self.alpha) * self.filtered_vz
                
                # Ограничение максимальной скорости (защита от выбросов)
                max_velocity = 5.0  # м/с
                self.filtered_vx = np.clip(self.filtered_vx, -max_velocity, max_velocity)
                self.filtered_vy = np.clip(self.filtered_vy, -max_velocity, max_velocity)
                self.filtered_vz = np.clip(self.filtered_vz, -max_velocity, max_velocity)

        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_time = current_time

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.x = x_orient
        odom_msg.pose.pose.orientation.y = y_orient
        odom_msg.pose.pose.orientation.z = z_orient
        odom_msg.pose.pose.orientation.w = w_orient

        # Используем отфильтрованные скорости
        odom_msg.twist.twist.linear.x = self.filtered_vx
        odom_msg.twist.twist.linear.y = self.filtered_vy
        odom_msg.twist.twist.linear.z = self.filtered_vz

        odom_msg.twist.twist.angular.x = angular_x
        odom_msg.twist.twist.angular.y = angular_y
        odom_msg.twist.twist.angular.z = angular_z
        
        # Установка ковариации (важно для SLAM)
        # Позиция (x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[14] = 0.01  # z
        odom_msg.pose.covariance[21] = 0.1   # roll
        odom_msg.pose.covariance[28] = 0.1   # pitch
        odom_msg.pose.covariance[35] = 0.05  # yaw
        
        # Скорость (vx, vy, vz, vroll, vpitch, vyaw)
        odom_msg.twist.covariance[0] = 0.1   # vx
        odom_msg.twist.covariance[7] = 0.1   # vy
        odom_msg.twist.covariance[14] = 0.1  # vz
        odom_msg.twist.covariance[21] = 0.1  # vroll
        odom_msg.twist.covariance[28] = 0.1  # vpitch
        odom_msg.twist.covariance[35] = 0.1  # vyaw

        self.odometry_publisher.publish(odom_msg)

        # Broadcast TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        tf_msg.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()