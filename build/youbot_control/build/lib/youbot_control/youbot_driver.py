import rclpy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class YouBotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Инициализация колес
        self.__wheel_1 = self.__robot.getDevice('wheel1')
        self.__wheel_2 = self.__robot.getDevice('wheel2')
        self.__wheel_3 = self.__robot.getDevice('wheel3')
        self.__wheel_4 = self.__robot.getDevice('wheel4')

        self.__wheel_1.setPosition(float('inf'))
        self.__wheel_1.setVelocity(0.0)
        self.__wheel_2.setPosition(float('inf'))
        self.__wheel_2.setVelocity(0.0)
        self.__wheel_3.setPosition(float('inf'))
        self.__wheel_3.setVelocity(0.0)
        self.__wheel_4.setPosition(float('inf'))
        self.__wheel_4.setVelocity(0.0)

        # Инициализация энкодеров
        self.__sensor_1 = self.__robot.getDevice('wheel1sensor')
        self.__sensor_2 = self.__robot.getDevice('wheel2sensor')
        self.__sensor_3 = self.__robot.getDevice('wheel3sensor')
        self.__sensor_4 = self.__robot.getDevice('wheel4sensor')
        
        timestep = int(self.__robot.getBasicTimeStep())
        self.__sensor_1.enable(timestep)
        self.__sensor_2.enable(timestep)
        self.__sensor_3.enable(timestep)
        self.__sensor_4.enable(timestep)

        # Параметры робота
        self.__wheel_radius = 0.05
        self.__lx = 0.228  # половина расстояния между колесами по X
        self.__ly = 0.158  # половина расстояния между колесами по Y

        # Состояние одометрии
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0
        self.__last_positions = [0.0, 0.0, 0.0, 0.0]

        self.__target_twist = Twist()

        # ROS2 узел
        rclpy.init(args=None)
        self.__node = rclpy.create_node('youbot_driver_node')
        
        # Подписка на cmd_vel
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
        # Публикация одометрии
        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # Время последнего обновления
        self.__last_time = self.__node.get_clock().now()

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __compute_odometry(self):
        """Вычисление одометрии на основе энкодеров"""
        # Получаем текущие значения энкодеров
        current_positions = [
            self.__sensor_1.getValue(),
            self.__sensor_2.getValue(),
            self.__sensor_3.getValue(),
            self.__sensor_4.getValue()
        ]
        
        # Вычисляем изменение углов поворота колес
        delta_positions = [
            current_positions[i] - self.__last_positions[i]
            for i in range(4)
        ]
        
        # Обратная кинематика для меканум-колес
        # Вычисляем движение робота в его системе координат
        delta_x_robot = self.__wheel_radius * (delta_positions[0] + delta_positions[1] + 
                                                 delta_positions[2] + delta_positions[3]) / 4.0
        delta_y_robot = self.__wheel_radius * (-delta_positions[0] + delta_positions[1] + 
                                                 delta_positions[2] - delta_positions[3]) / 4.0
        delta_theta = self.__wheel_radius * (-delta_positions[0] + delta_positions[1] - 
                                               delta_positions[2] + delta_positions[3]) / (4.0 * (self.__lx + self.__ly))
        
        # Преобразуем в глобальную систему координат
        delta_x_global = delta_x_robot * math.cos(self.__theta) - delta_y_robot * math.sin(self.__theta)
        delta_y_global = delta_x_robot * math.sin(self.__theta) + delta_y_robot * math.cos(self.__theta)
        
        # Обновляем позицию
        self.__x += delta_x_global
        self.__y += delta_y_global
        self.__theta += delta_theta
        
        # Нормализуем угол
        self.__theta = math.atan2(math.sin(self.__theta), math.cos(self.__theta))
        
        # Сохраняем текущие позиции
        self.__last_positions = current_positions

    def __publish_odometry(self):
        """Публикация одометрии в ROS2"""
        current_time = self.__node.get_clock().now()
        dt = (current_time - self.__last_time).nanoseconds / 1e9
        
        # Вычисляем скорости
        vx = self.__target_twist.linear.x
        vy = self.__target_twist.linear.y
        vth = self.__target_twist.angular.z
        
        # Создаем сообщение одометрии
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Позиция
        odom.pose.pose.position.x = self.__x
        odom.pose.pose.position.y = self.__y
        odom.pose.pose.position.z = 0.0
        
        # Ориентация (quaternion)
        odom.pose.pose.orientation = self.__euler_to_quaternion(0, 0, self.__theta)
        
        # Скорости
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        # Публикуем
        self.__odom_pub.publish(odom)
        
        # Публикуем TF трансформацию
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.__x
        t.transform.translation.y = self.__y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.__euler_to_quaternion(0, 0, self.__theta)
        
        self.__tf_broadcaster.sendTransform(t)
        
        self.__last_time = current_time

    def __euler_to_quaternion(self, roll, pitch, yaw):
        """Преобразование углов Эйлера в кватернион"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

    def step(self):
        # Обрабатываем ROS2 события
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Вычисляем одометрию
        self.__compute_odometry()
        
        # Публикуем одометрию
        self.__publish_odometry()

        # Управление колесами
        linear_x = self.__target_twist.linear.x
        linear_y = self.__target_twist.linear.y
        angular_z = self.__target_twist.angular.z

        command_wheel_1 = (linear_x - linear_y - (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_2 = (linear_x + linear_y + (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_3 = (linear_x + linear_y - (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_4 = (linear_x - linear_y + (self.__lx + self.__ly) * angular_z) / self.__wheel_radius

        self.__wheel_1.setVelocity(command_wheel_1)
        self.__wheel_2.setVelocity(command_wheel_2)
        self.__wheel_3.setVelocity(command_wheel_3)
        self.__wheel_4.setVelocity(command_wheel_4)